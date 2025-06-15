// JoyShockLibrary.cpp : Defines the exported functions for the DLL application.
//

#include "JoyShockLibrary.h"
#include <bitset>
#include "hidapi.h"
#include <chrono>
#include <thread>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <atomic>
#include "GamepadMotion.hpp"
#include "JoyShock.cpp"
#include "InputHelpers.cpp"

std::shared_timed_mutex _callbackLock;
std::shared_timed_mutex _connectedLock;
void(*_pollCallback)(int, JOY_SHOCK_STATE, JOY_SHOCK_STATE, IMU_STATE, IMU_STATE, float) = nullptr;
void(*_pollTouchCallback)(int, TOUCH_STATE, TOUCH_STATE, float) = nullptr;
void(*_connectCallback)(int) = nullptr;
void(*_disconnectCallback)(int, bool) = nullptr;
std::unordered_map<int, JoyShock*> _joyshocks;
std::unordered_map<std::string, JoyShock*> _byPath;
std::mutex _pathHandleLock;
std::unordered_map<std::string, int> _pathHandle;
// https://stackoverflow.com/questions/41206861/atomic-increment-and-return-counter
static std::atomic<int> _joyshockHandleCounter;

static int GetUniqueHandle(const std::string &path)
{
	_pathHandleLock.lock();
	auto iter = _pathHandle.find(path);

	if (iter != _pathHandle.end())
	{
		_pathHandleLock.unlock();
		return iter->second;
	}
	const int handle = _joyshockHandleCounter++;
	_pathHandle.emplace(path, handle);
	_pathHandleLock.unlock();

	return handle;
}

// https://stackoverflow.com/questions/25144887/map-unordered-map-prefer-find-and-then-at-or-try-at-catch-out-of-range
// not thread-safe -- because you probably want to do something with the object you get out of it, I've left locking to the caller
static JoyShock* GetJoyShockFromHandle(int handle) {
	auto iter = _joyshocks.find(handle);

	if (iter != _joyshocks.end())
	{
		return iter->second;
		// iter is item pair in the map. The value will be accessible as `iter->second`.
	}
	return nullptr;
}

void pollIndividualLoop(JoyShock *jc) {
	if (!jc->handle) { return; }

	hid_set_nonblocking(jc->handle, 0);
	//hid_set_nonblocking(jc->handle, 1); // temporary, to see if it helps. this means we'll have a crazy spin

	int numTimeOuts = 0;
	int numNoIMU = 0;
	bool hasIMU = false;
	bool lockedThread = false;
	int noIMULimit;
	switch (jc->controller_type)
	{
	case ControllerType::s_ds4:
		noIMULimit = 250;
		break;
	case ControllerType::s_ds:
		noIMULimit = 250;
		break;
	case ControllerType::n_switch:
	default:
		noIMULimit = 67;
	}
	float wakeupTimer = 0.0f;

	while (!jc->cancel_thread) {
		// get input:
		unsigned char buf[64];
		memset(buf, 0, 64);

		// 10 seconds of no signal means forget this controller
		int reuseCounter = jc->reuse_counter;
		int res = hid_read_timeout(jc->handle, buf, 64, 1000);

		if (res == -1)
		{
			// disconnected!
			printf("Controller %d disconnected\n", jc->intHandle);

			_connectedLock.lock();
			lockedThread = true;
			const bool gettingReused = jc->reuse_counter != reuseCounter;
			jc->delete_on_finish = true;
			if (gettingReused)
			{
				jc->remove_on_finish = false;
				jc->delete_on_finish = false;
				lockedThread = false;
				_connectedLock.unlock();
			}
			break;
		}

		if (res == 0)
		{
			numTimeOuts++;
			if (numTimeOuts >= 10)
			{
				printf("Controller %d timed out\n", jc->intHandle);

				// just make sure we get this thing deleted before someone else tries to start a new connection
				_connectedLock.lock();
				lockedThread = true;
				const bool gettingReused = jc->reuse_counter != reuseCounter;
				jc->delete_on_finish = true;
				if (gettingReused)
				{
					jc->remove_on_finish = false;
					jc->delete_on_finish = false;
					lockedThread = false;
					_connectedLock.unlock();
				}
				break;
			}
			else
			{
				// try wake up the controller with the appropriate message
				if (jc->controller_type != ControllerType::n_switch)
				{
					// TODO
				}
				else
				{
					if (jc->is_usb)
					{
						printf("Attempting to re-initialise controller %d\n", jc->intHandle);
						if (jc->init_usb())
						{
							numTimeOuts = 0;
						}
					}
					else
					{
						printf("Attempting to re-initialise controller %d\n", jc->intHandle);
						if (jc->init_bt())
						{
							numTimeOuts = 0;
						}
					}
				}
			}
		}
		else
		{
			numTimeOuts = 0;
			// we want to be able to do these check-and-calls without fear of interruption by another thread. there could be many threads (as many as connected controllers),
			// and the callback could be time-consuming (up to the user), so we use a readers-writer-lock.
			if (handle_input(jc, buf, 64, hasIMU)) { // but the user won't necessarily have a callback at all, so we'll skip the lock altogether in that case
				// accumulate gyro
				IMU_STATE imuState = jc->get_transformed_imu_state(jc->imu_state);
				jc->push_cumulative_gyro(imuState.gyroX, imuState.gyroY, imuState.gyroZ);
				if (_pollCallback != nullptr || _pollTouchCallback != nullptr)
				{
					std::shared_lock<std::shared_timed_mutex> lock(_callbackLock);
					if (_pollCallback != nullptr) {
						_pollCallback(jc->intHandle, jc->simple_state, jc->last_simple_state, imuState, jc->get_transformed_imu_state(jc->last_imu_state), jc->delta_time);
					}
					// touchpad will have its own callback so that it doesn't change the existing api
					if (jc->controller_type != ControllerType::n_switch && _pollTouchCallback != nullptr) {
						_pollTouchCallback(jc->intHandle, jc->touch_state, jc->last_touch_state, jc->delta_time);
					}
				}
				// count how many have no IMU result. We want to periodically attempt to enable IMU if it's not present
				if (!hasIMU)
				{
					numNoIMU++;
					if (numNoIMU == noIMULimit)
					{
						memset(buf, 0, 64);
						jc->enable_IMU(buf, 64);
						numNoIMU = 0;
					}
				}
				else
				{
					numNoIMU = 0;
				}
				
				// dualshock 4 bluetooth might need waking up
				if (jc->controller_type == ControllerType::s_ds4 && !jc->is_usb)
				{
					wakeupTimer += jc->delta_time;
					if (wakeupTimer > 30.0f)
					{
						jc->init_ds4_bt();
						wakeupTimer = 0.0f;
					}
				}
			}
		}
	}

	if (jc->cancel_thread)
	{
		printf("\tending cancelled thread\n");
	}
	else
	{
		printf("\ttiming out thread\n");
	}

	// remove
	if (jc->remove_on_finish)
	{
		printf("\t\tremoving jc\n");
		if (!lockedThread)
		{
			_connectedLock.lock();
		}
		_joyshocks.erase(jc->intHandle);
		_byPath.erase(jc->path);
		if (!lockedThread)
		{
			_connectedLock.unlock();
		}
	}

	const int intHandle = jc->intHandle;
	// disconnect this device
	if (jc->delete_on_finish)
	{
		printf("\t\tdeleting jc\n");
		delete jc;
	}

	if (lockedThread)
	{
		_connectedLock.unlock();
	}

	// notify that we disconnected this device, and say whether or not it was a timeout (if not a timeout, then an explicit disconnect)
	{
		std::shared_lock<std::shared_timed_mutex> lock(_callbackLock);
		if (_disconnectCallback != nullptr)
		{
			_disconnectCallback(intHandle, numTimeOuts >= 10);
		}
	}
}

int JslConnectDevices()
{
	// for writing to console:
	//freopen("CONOUT$", "w", stdout);

	// most of the joycon and pro controller stuff here is thanks to mfosse's vjoy feeder
	// Enumerate and print the HID devices on the system
	struct hid_device_info *devs, *cur_dev;

	std::vector<int> createdIds;

	_connectedLock.lock();

	int res = hid_init();

	devs = hid_enumerate(0x0, 0x0);
	cur_dev = devs;
	while (cur_dev) {
		bool isSupported = false;
		bool isSwitch = false;
		switch (cur_dev->vendor_id)
		{
		case JOYCON_VENDOR:
			isSupported = cur_dev->product_id == JOYCON_L_BT ||
				cur_dev->product_id == JOYCON_R_BT ||
				cur_dev->product_id == PRO_CONTROLLER ||
				cur_dev->product_id == JOYCON_CHARGING_GRIP;
			isSwitch = true;
			break;
		case DS_VENDOR:
			isSupported = cur_dev->product_id == DS4_USB ||
				cur_dev->product_id == DS4_USB_V2 ||
				cur_dev->product_id == DS4_USB_DONGLE ||
				cur_dev->product_id == DS4_BT ||
				cur_dev->product_id == DS_USB ||
				cur_dev->product_id == DS_USB_V2;
			break;
		case BROOK_DS4_VENDOR:
			isSupported = cur_dev->product_id == BROOK_DS4_USB;
			break;
		default:
			break;
		}
		if (!isSupported)
		{
			cur_dev = cur_dev->next;
			continue;
		}

		const std::string path = std::string(cur_dev->path);
		auto iter = _byPath.find(path);
		JoyShock* currentJc = nullptr;
		bool isSameController = false;
		if (iter != _byPath.end())
		{
			currentJc = iter->second;
			isSameController = isSwitch == (currentJc->controller_type == ControllerType::n_switch);
		}
		printf("path: %s\n", cur_dev->path);

		hid_device* handle = hid_open_path(cur_dev->path);
		if (handle != nullptr)
		{
			printf("\topened new handle\n");
			if (isSameController)
			{
				printf("\tending old thread and joining\n");
				// we'll get a new thread, so we need to delete the old one, but we need to actually wait for it, I think, because it'll be affected by init and all that...
				// but we can't wait for it! That could deadlock if it happens to be about to disconnect or time out!
				std::thread* thread = currentJc->thread;
				currentJc->delete_on_finish = false;
				currentJc->remove_on_finish = false;
				currentJc->cancel_thread = true;
				currentJc->reuse_counter++;

				// finishing thread may be about to hit a lock
				_connectedLock.unlock();
				thread->join();
				_connectedLock.lock();

				delete thread;
				currentJc->thread = nullptr;
				// don't immediately cancel the next thread:
				currentJc->cancel_thread = false;
				currentJc->delete_on_finish = false;
				currentJc->remove_on_finish = true;

				printf("\tinitialising with new handle\n");
				// keep calibration stuff, but reset other stuff just in case it's actually a new controller
				currentJc->init(cur_dev, handle, GetUniqueHandle(path), path);
				currentJc = nullptr;
			}
			else
			{
				printf("\tcreating new JoyShock\n");
				JoyShock* jc = new JoyShock(cur_dev, handle, GetUniqueHandle(path), path);
				_joyshocks.emplace(jc->intHandle, jc);
				_byPath.emplace(path, jc);
				createdIds.push_back(jc->intHandle);
			}

			if (currentJc != nullptr)
			{
				printf("\tdeinitialising old controller\n");
				// it's been replaced! get rid of it
				if (currentJc->controller_type == ControllerType::s_ds4) {
					if (currentJc->is_usb) {
						currentJc->deinit_ds4_usb();
					}
					else {
						currentJc->deinit_ds4_bt();
					}
				}
				else if (currentJc->controller_type == ControllerType::s_ds) {

				}
				else if (currentJc->is_usb) {
					currentJc->deinit_usb();
				}
				printf("\tending old thread\n");
				std::thread* thread = currentJc->thread;
				currentJc->delete_on_finish = true;
				currentJc->remove_on_finish = false;
				currentJc->cancel_thread = true;
				thread->detach();
				delete thread;
				currentJc->thread = nullptr;
			}
		}

		cur_dev = cur_dev->next;
	}
	hid_free_enumeration(devs);

	// init joyshocks:
	for (std::pair<int, JoyShock*> pair : _joyshocks)
	{
		JoyShock* jc = pair.second;
		
		if (jc->initialised)
		{
			continue;
		}

		if (jc->controller_type == ControllerType::s_ds4) {
			if (!jc->is_usb) {
				jc->init_ds4_bt();
			}
			else {
				jc->init_ds4_usb();
			}
		} // dualsense
		else if (jc->controller_type == ControllerType::s_ds)
		{
			jc->initialised = true;
		} // charging grip
		else if (jc->is_usb) {
			//printf("USB\n");
			jc->init_usb();
		}
		else {
			//printf("BT\n");
			jc->init_bt();
		}
		// all get time now for polling
		jc->last_polled = std::chrono::steady_clock::now();
		jc->delta_time = 0.0;

		jc->deviceNumber = 0; // left
	}

	unsigned char buf[64];

	// set lights:
	//printf("setting LEDs...\n");
	int switchIndex = 1;
	int dualSenseIndex = 1;
	for (std::pair<int, JoyShock*> pair : _joyshocks)
	{
		JoyShock *jc = pair.second;

		// restore colours if we have them set for this controller
		switch (jc->controller_type)
		{
		case ControllerType::s_ds4:
			jc->set_ds4_rumble_light(0, 0, jc->led_r, jc->led_g, jc->led_b);
			break;
		case ControllerType::s_ds:
			jc->set_ds5_rumble_light(0, 0, jc->led_r, jc->led_g, jc->led_b, dualSenseIndex++);
			break;
		case ControllerType::n_switch:
			jc->player_number = switchIndex++;
			memset(buf, 0x00, 0x40);
			buf[0] = (unsigned char)jc->player_number;
			jc->send_subcommand(0x01, 0x30, buf, 1);
			break;
		}

		// threads for polling
		if (jc->thread == nullptr)
		{
			printf("\tstarting new thread\n");
			jc->thread = new std::thread(pollIndividualLoop, jc);
		}
	}

	const int totalDevices = (int)_joyshocks.size();

	_connectedLock.unlock();

	// notify that we created the new object (now that we're not in a lock that might prevent reading data)
	{
		std::shared_lock<std::shared_timed_mutex> lock(_callbackLock);
		if (_connectCallback != nullptr)
		{
			for (int newConnectionHandle : createdIds)
			{
				_connectCallback(newConnectionHandle);
			}
		}
	}

	return totalDevices;
}

int JslGetConnectedDeviceHandles(int* deviceHandleArray, int size)
{
	int i = 0;
	_connectedLock.lock_shared();
	for (std::pair<int, JoyShock*> pair : _joyshocks)
	{
		if (i >= size) {
			break;
		}
		deviceHandleArray[i] = pair.first;
		i++;
	}
	_connectedLock.unlock_shared();
	return i; // return num actually found
}

void JslDisconnectAndDisposeAll()
{
	// no more callback
	JslSetCallback(nullptr);
	JslSetTouchCallback(nullptr);

	_connectedLock.lock();

	for (std::pair<int, JoyShock*> pair : _joyshocks)
	{
		JoyShock* jc = pair.second;
		if (jc->controller_type == ControllerType::s_ds4) {
			if (jc->is_usb) {
				jc->deinit_ds4_usb();
			}
			else {
				jc->deinit_ds4_bt();
			}
		}
		else if (jc->controller_type == ControllerType::s_ds) {

		} // TODO: Charging grip? bluetooth?
		else if (jc->is_usb) {
			jc->deinit_usb();
		}
		// cleanup
		std::thread* thread = jc->thread;
		jc->delete_on_finish = true;
		jc->remove_on_finish = false;
		jc->cancel_thread = true;
		thread->detach();
		delete thread;
	}
	_joyshocks.clear();
	_byPath.clear();

	_connectedLock.unlock();

	// Finalize the hidapi library
	int res = hid_exit();
}

bool JslStillConnected(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	return GetJoyShockFromHandle(deviceId) != nullptr;
}

void JslDisconnect(int deviceId)
{
	_connectedLock.lock();

	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr)
	{
		if (jc->controller_type == ControllerType::s_ds4) {
			if (jc->is_usb) {
				jc->deinit_ds4_usb();
			}
			else {
				jc->deinit_ds4_bt();
			}
		}
		else if (jc->controller_type == ControllerType::s_ds) {

		} // TODO: Charging grip? bluetooth?
		else if (jc->is_usb) {
			jc->deinit_usb();
		}
		// cleanup
		std::thread* thread = jc->thread;
		jc->delete_on_finish = true;
		jc->remove_on_finish = false;
		jc->cancel_thread = true;
		thread->detach();
		delete thread;

		_joyshocks.erase(deviceId);
		_byPath.erase(jc->path);

		hid_close(jc->handle);
	}

	_connectedLock.unlock();
}

// get buttons as bits in the following order, using North South East West to name face buttons to avoid ambiguity between Xbox and Nintendo layouts:
// 0x00001: up
// 0x00002: down
// 0x00004: left
// 0x00008: right
// 0x00010: plus
// 0x00020: minus
// 0x00040: left stick click
// 0x00080: right stick click
// 0x00100: L
// 0x00200: R
// ZL and ZR are reported as analogue inputs (GetLeftTrigger, GetRightTrigger), because DS4 and XBox controllers use analogue triggers, but we also have them as raw buttons
// 0x00400: ZL
// 0x00800: ZR
// 0x01000: S
// 0x02000: E
// 0x04000: W
// 0x08000: N
// 0x10000: home / PS
// 0x20000: capture / touchpad-click
// 0x40000: SL
// 0x80000: SR
// if you want the whole state, this is the best way to do it
JOY_SHOCK_STATE JslGetSimpleState(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state;
	}
	return {};
}
IMU_STATE JslGetIMUState(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->get_transformed_imu_state(jc->imu_state);
	}
	return {};
}
MOTION_STATE JslGetMotionState(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->get_motion_state();
	}
	return {};
}
TOUCH_STATE JslGetTouchState(int deviceId, bool previous)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return previous ? jc->last_touch_state : jc->touch_state;
	}
	return {};
}

bool JslGetTouchpadDimension(int deviceId, int &sizeX, int &sizeY)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	// I am assuming a single touchpad (or all touchpads are the same dimension)?
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr)
	{
		switch (jc->controller_type)
		{
		case ControllerType::s_ds4:
		case ControllerType::s_ds:
			sizeX = 1920;
			sizeY = 943;
			break;
		default:
			sizeX = 0;
			sizeY = 0;
			break;
		}
		return true;
	}
	return false;
}

int JslGetButtons(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state.buttons;
	}
	return 0;
}

// get thumbsticks
float JslGetLeftX(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state.stickLX;
	}
	return 0.0f;
}
float JslGetLeftY(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state.stickLY;
	}
	return 0.0f;
}
float JslGetRightX(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state.stickRX;
	}
	return 0.0f;
}
float JslGetRightY(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state.stickRY;
	}
	return 0.0f;
}

// get triggers. Switch controllers don't have analogue triggers, but will report 0.0 or 1.0 so they can be used in the same way as others
float JslGetLeftTrigger(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state.lTrigger;
	}
	return 0.0f;
}
float JslGetRightTrigger(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state.rTrigger;
	}
	return 0.0f;
}

// get gyro
float JslGetGyroX(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->get_transformed_imu_state(jc->imu_state).gyroX;
	}
	return 0.0f;
}
float JslGetGyroY(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->get_transformed_imu_state(jc->imu_state).gyroY;
	}
	return 0.0f;
}
float JslGetGyroZ(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->get_transformed_imu_state(jc->imu_state).gyroZ;
	}
	return 0.0f;
}

void JslGetAndFlushAccumulatedGyro(int deviceId, float& gyroX, float& gyroY, float& gyroZ)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		jc->get_and_flush_cumulative_gyro(gyroX, gyroY, gyroZ);
		return;
	}
	gyroX = gyroY = gyroZ = 0.f;
}

void JslSetGyroSpace(int deviceId, int gyroSpace)
{
	if (gyroSpace < 0) {
		gyroSpace = 0;
	}
	if (gyroSpace > 2) {
		gyroSpace = 2;
	}
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		jc->modifying_lock.lock();
		jc->gyroSpace = gyroSpace;
		jc->modifying_lock.unlock();
	}
}

// get accelerometor
float JslGetAccelX(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->imu_state.accelX;
	}
	return 0.0f;
}
float JslGetAccelY(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->imu_state.accelY;
	}
	return 0.0f;
}
float JslGetAccelZ(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->imu_state.accelZ;
	}
	return 0.0f;
}

// get touchpad
int JslGetTouchId(int deviceId, bool secondTouch)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		if (!secondTouch) {
			return jc->touch_state.t0Id;
		}
		else {
			return jc->touch_state.t1Id;
		}
	}
	return false;
}
bool JslGetTouchDown(int deviceId, bool secondTouch)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		if (!secondTouch) {
			return jc->touch_state.t0Down;
		}
		else {
			return jc->touch_state.t1Down;
		}
	}
	return false;
}

float JslGetTouchX(int deviceId, bool secondTouch)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		if (!secondTouch) {
			return jc->touch_state.t0X;
		}
		else {
			return jc->touch_state.t1X;
		}
	}
	return 0.0f;
}
float JslGetTouchY(int deviceId, bool secondTouch)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		if (!secondTouch) {
			return jc->touch_state.t0Y;
		}
		else {
			return jc->touch_state.t1Y;
		}
	}
	return 0.0f;
}

// analog parameters have different resolutions depending on device
float JslGetStickStep(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		if (jc->controller_type != ControllerType::n_switch) {
			return 1.0f / 128.0;
		}
		else {
			if (jc->left_right == 2) // right joycon has no calibration for left stick
			{
				return 1.0f / (jc->stick_cal_x_r[2] - jc->stick_cal_x_r[1]);
			}
			else {
				return 1.0f / (jc->stick_cal_x_l[2] - jc->stick_cal_x_l[1]);
			}
		}
	}
	return 0.0f;
}
float JslGetTriggerStep(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->controller_type != ControllerType::n_switch ? 1 / 256.0f : 1.0f;
	}
	return 1.0f;
}
float JslGetPollRate(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->controller_type != ControllerType::n_switch ? 250.0f : 66.6667f;
	}
	return 0.0f;
}
float JslGetTimeSinceLastUpdate(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		auto time_now = std::chrono::steady_clock::now();
		return (float)(std::chrono::duration_cast<std::chrono::microseconds>(time_now - jc->last_polled).count() / 1000000.0);
	}
	return 0.0f;
}

// calibration
void JslResetContinuousCalibration(int deviceId) {
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		jc->reset_continuous_calibration();
	}
}
void JslStartContinuousCalibration(int deviceId) {
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		jc->use_continuous_calibration = true;
		jc->cue_motion_reset = true;
	}
}
void JslPauseContinuousCalibration(int deviceId) {
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		jc->use_continuous_calibration = false;
	}
}
void JslSetAutomaticCalibration(int deviceId, bool enabled) {
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		jc->modifying_lock.lock();
		jc->motion.SetCalibrationMode(enabled ? GamepadMotionHelpers::CalibrationMode::SensorFusion | GamepadMotionHelpers::CalibrationMode::Stillness : GamepadMotionHelpers::CalibrationMode::Manual);
		jc->modifying_lock.unlock();
	}
}
void JslGetCalibrationOffset(int deviceId, float& xOffset, float& yOffset, float& zOffset) {
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		// not technically modifying, but also not a simple getter
		jc->modifying_lock.lock();
		jc->motion.GetCalibrationOffset(xOffset, yOffset, zOffset);
		jc->modifying_lock.unlock();
	}
}
void JslSetCalibrationOffset(int deviceId, float xOffset, float yOffset, float zOffset) {
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		jc->modifying_lock.lock();
		jc->motion.SetCalibrationOffset(xOffset, yOffset, zOffset, 1);
		jc->modifying_lock.unlock();
	}
}
JSL_AUTO_CALIBRATION JslGetAutoCalibrationStatus(int deviceId) {
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		JSL_AUTO_CALIBRATION calibration;

		calibration.autoCalibrationEnabled = jc->motion.GetCalibrationMode() != GamepadMotionHelpers::CalibrationMode::Manual;
		calibration.confidence = jc->motion.GetAutoCalibrationConfidence();
		calibration.isSteady = jc->motion.GetAutoCalibrationIsSteady();

		return calibration;
	}

	return {};
}

// this function will get called for each input event from each controller
void JslSetCallback(void(*callback)(int, JOY_SHOCK_STATE, JOY_SHOCK_STATE, IMU_STATE, IMU_STATE, float)) {
	// exclusive lock
	_callbackLock.lock();
	_pollCallback = callback;
	_callbackLock.unlock();
}

// this function will get called for each input event, even if touch data didn't update
void JslSetTouchCallback(void(*callback)(int, TOUCH_STATE, TOUCH_STATE, float)) {
	_callbackLock.lock();
	_pollTouchCallback = callback;
	_callbackLock.unlock();
}

// this function will get called for each device when it is newly connected
void JslSetConnectCallback(void(*callback)(int)) {
	// exclusive lock
	_callbackLock.lock();
	_connectCallback = callback;
	_callbackLock.unlock();
}

// this function will get called for each device when it is disconnected (and whether it was a timeout (true))
void JslSetDisconnectCallback(void(*callback)(int, bool)) {
	// exclusive lock
	_callbackLock.lock();
	_disconnectCallback = callback;
	_callbackLock.unlock();
}

// super-getter for reading a whole lot of state at once
JSL_SETTINGS JslGetControllerInfoAndSettings(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		JSL_SETTINGS settings;

		settings.gyroSpace = jc->gyroSpace;
		settings.playerNumber = jc->player_number;
		settings.splitType = jc->left_right;
		settings.isConnected = true;
		settings.isCalibrating = jc->use_continuous_calibration;
		settings.autoCalibrationEnabled = jc->motion.GetCalibrationMode() != GamepadMotionHelpers::CalibrationMode::Manual;
		std::strcpy(settings.path, jc->path.c_str());

		switch (jc->controller_type)
		{
		case ControllerType::s_ds4:
			settings.controllerType = JS_TYPE_DS4;
			break;
		case ControllerType::s_ds:
			settings.controllerType = JS_TYPE_DS;
			break;
		default:
		case ControllerType::n_switch:
			settings.controllerType = jc->left_right;
			settings.colour = jc->body_colour;
			break;
		}

		if (jc->controller_type != ControllerType::n_switch)
		{
			// get led colour
			settings.colour = (int)(jc->led_b) | ((int)(jc->led_g) << 8) | ((int)(jc->led_r) << 16);
		}

		return settings;
	}
	return {};
}

// what split type of controller is this?
int JslGetControllerType(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		switch (jc->controller_type)
		{
		case ControllerType::s_ds4:
			return JS_TYPE_DS4;
		case ControllerType::s_ds:
			return JS_TYPE_DS;
		default:
		case ControllerType::n_switch:
			return jc->left_right;
		}
	}
	return 0;
}
// what split type of controller is this?
int JslGetControllerSplitType(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->left_right;
	}
	return 0;
}
// what colour is the controller (not all controllers support this; those that don't will report white)
int JslGetControllerBodyColour(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	// this just reports body colour. Switch controllers also give buttons colour, and in Pro's case, left and right grips
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->body_colour;
	}
	return 0xFFFFFF;
}
// what colour is the controller (not all controllers support this; those that don't will report white)
int JslGetControllerLeftGripColour(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	// this just reports body colour. Switch controllers also give buttons colour, and in Pro's case, left and right grips
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->left_grip_colour;
	}
	return 0xFFFFFF;
}
// what colour is the controller (not all controllers support this; those that don't will report white)
int JslGetControllerRightGripColour(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	// this just reports body colour. Switch controllers also give buttons colour, and in Pro's case, left and right grips
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->right_grip_colour;
	}
	return 0xFFFFFF;
}
// what colour is the controller (not all controllers support this; those that don't will report white)
int JslGetControllerButtonColour(int deviceId)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	// this just reports body colour. Switch controllers also give buttons colour, and in Pro's case, left and right grips
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->button_colour;
	}
	return 0xFFFFFF;
}

// set controller light colour (not all controllers have a light whose colour can be set, but that just means nothing will be done when this is called -- no harm)
void JslSetLightColour(int deviceId, int colour)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr && jc->controller_type == ControllerType::s_ds4) {
		jc->modifying_lock.lock();
		jc->led_r = (colour >> 16) & 0xff;
		jc->led_g = (colour >> 8) & 0xff;
		jc->led_b = colour & 0xff;
		jc->set_ds4_rumble_light(
			jc->small_rumble,
			jc->big_rumble,
			jc->led_r,
			jc->led_g,
			jc->led_b);
		jc->modifying_lock.unlock();
	}
	else if(jc != nullptr && jc->controller_type == ControllerType::s_ds) {
		jc->modifying_lock.lock();
        jc->led_r = (colour >> 16) & 0xff;
        jc->led_g = (colour >> 8) & 0xff;
        jc->led_b = colour & 0xff;
        jc->set_ds5_rumble_light(
                jc->small_rumble,
                jc->big_rumble,
                jc->led_r,
                jc->led_g,
                jc->led_b,
                jc->player_number);
		jc->modifying_lock.unlock();
	}
}

// set controller rumble
void JslSetRumble(int deviceId, int smallRumble, int bigRumble)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr && jc->controller_type == ControllerType::s_ds4) {
		jc->modifying_lock.lock();
		jc->small_rumble = smallRumble;
		jc->big_rumble = bigRumble;
		jc->set_ds4_rumble_light(
			jc->small_rumble,
			jc->big_rumble,
			jc->led_r,
			jc->led_g,
			jc->led_b);
		jc->modifying_lock.unlock();
	}
    else if (jc != nullptr && jc->controller_type == ControllerType::s_ds) {
		jc->modifying_lock.lock();
        jc->small_rumble = smallRumble;
        jc->big_rumble = bigRumble;
        jc->set_ds5_rumble_light(
                jc->small_rumble,
                jc->big_rumble,
                jc->led_r,
                jc->led_g,
                jc->led_b,
                jc->player_number);
		jc->modifying_lock.unlock();
    }
}

void JslSetDS5TriggersOff(int deviceId, EDS5AffectedTriggers affectedTriggers)
{
    std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
    JoyShock* jc = GetJoyShockFromHandle(deviceId);
    if (jc != nullptr && jc->controller_type == ControllerType::s_ds) {
        ds5_trigger_effect leftTrigger, rightTrigger;
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Left)
        {
            leftTrigger.bDirty = true;
            leftTrigger.motorMode = DS5_TRIGGER_OFF;
            leftTrigger.startResistance = 0x00;
            leftTrigger.effectForce = 0x00;
            leftTrigger.rangeForce = 0x00;
            leftTrigger.nearReleaseStrength = 0x00;
            leftTrigger.nearMiddleStrength = 0x00;
            leftTrigger.pressedStrength = 0x00;
            leftTrigger.P6 = 0x00;
            leftTrigger.P7 = 0x00;
            leftTrigger.actuationFrequency = 0x00;
            leftTrigger.P9 = 0x00;
        }
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Right)
        {
			rightTrigger.bDirty = true;
            rightTrigger.motorMode = DS5_TRIGGER_OFF;
            rightTrigger.startResistance = 0x00;
            rightTrigger.effectForce = 0x00;
            rightTrigger.rangeForce = 0x00;
            rightTrigger.nearReleaseStrength = 0x00;
            rightTrigger.nearMiddleStrength = 0x00;
            rightTrigger.pressedStrength = 0x00;
            rightTrigger.P6 = 0x00;
            rightTrigger.P7 = 0x00;
            rightTrigger.actuationFrequency = 0x00;
            rightTrigger.P9 = 0x00;
        }
        jc->modifying_lock.lock();

		jc->set_ds5_trigger_effects(leftTrigger, rightTrigger);

        jc->modifying_lock.unlock();
    }
}

void JslSetDS5TriggersFeedback(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char position, unsigned char strength)
{
    std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
    JoyShock* jc = GetJoyShockFromHandle(deviceId);
    if (jc != nullptr && jc->controller_type == ControllerType::s_ds) 
	{
		if (position < 0)
			position = 0;
        if (position > 9)
            position = 9;
        if (strength < 1)
            strength = 1;
        if (strength > 8)
            strength = 8;

        unsigned char forceValue = ((strength - 1) & 0x07);
        unsigned int forceZones = 0;
        unsigned short activeZones = 0;
        for (int i = position; i < 10; i++)
        {
            forceZones = forceZones | (forceValue << (3 * i));
            activeZones = activeZones | (1 << i);
        }

		
        ds5_trigger_effect leftTrigger, rightTrigger;
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Left)
        {
			leftTrigger.bDirty = true;
            leftTrigger.motorMode = DS5_TRIGGER_FEEDBACK;
            leftTrigger.startResistance = (activeZones >> 0) & 0xff;
			leftTrigger.effectForce = (activeZones >> 8) & 0xff;
			leftTrigger.rangeForce = (forceZones >> 0) & 0xff;
			leftTrigger.nearReleaseStrength = (forceZones >> 8) & 0xff;
			leftTrigger.nearMiddleStrength = (forceZones >> 16) & 0xff;
			leftTrigger.pressedStrength = (forceZones >> 24) & 0xff;
            leftTrigger.P6 = 0x00;
            leftTrigger.P7 = 0x00;
            leftTrigger.actuationFrequency = 0x00;
            leftTrigger.P9 = 0x00;
        }
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Right)
        {
            rightTrigger.bDirty = true;
            rightTrigger.motorMode = DS5_TRIGGER_FEEDBACK;
            rightTrigger.startResistance = (activeZones >> 0) & 0xff;
            rightTrigger.effectForce = (activeZones >> 8) & 0xff;
            rightTrigger.rangeForce = (forceZones >> 0) & 0xff;
            rightTrigger.nearReleaseStrength = (forceZones >> 8) & 0xff;
            rightTrigger.nearMiddleStrength = (forceZones >> 16) & 0xff;
            rightTrigger.pressedStrength = (forceZones >> 24) & 0xff;
            rightTrigger.P6 = 0x00;
            rightTrigger.P7 = 0x00;
            rightTrigger.actuationFrequency = 0x00;
            rightTrigger.P9 = 0x00;
        }
        jc->modifying_lock.lock();

        jc->set_ds5_trigger_effects(leftTrigger, rightTrigger);

        jc->modifying_lock.unlock();

    }

}

void JslSetDS5TriggersWeapon(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char startPosition, unsigned char endPosition, unsigned char strength)
{
    std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
    JoyShock* jc = GetJoyShockFromHandle(deviceId);
    if (jc != nullptr && jc->controller_type == ControllerType::s_ds)
    {
        if (startPosition < 2)
            startPosition = 2;
        if (startPosition > 7)
            startPosition = 7;
        if (endPosition <= startPosition)
			endPosition = startPosition + 1;
        if (endPosition > 8)
			endPosition = 8;
        if (strength < 1)
            strength = 1;
        if (strength > 8)
            strength = 8;

        unsigned int startAndStopZones = (1 << startPosition) | (1 << endPosition);

        ds5_trigger_effect leftTrigger, rightTrigger;
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Left)
        {
            leftTrigger.bDirty = true;
            leftTrigger.motorMode = DS5_TRIGGER_WEAPON;
            leftTrigger.startResistance = (startAndStopZones >> 0) & 0xff;
            leftTrigger.effectForce = (startAndStopZones >> 8) & 0xff;
            leftTrigger.rangeForce = strength - 1;
            leftTrigger.nearReleaseStrength = 0x00;
            leftTrigger.nearMiddleStrength = 0x00;
            leftTrigger.pressedStrength = 0x00;
            leftTrigger.P6 = 0x00;
            leftTrigger.P7 = 0x00;
            leftTrigger.actuationFrequency = 0x00;
            leftTrigger.P9 = 0x00;
        }
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Right)
        {
			rightTrigger.bDirty = true;
            rightTrigger.motorMode = DS5_TRIGGER_WEAPON;
            rightTrigger.startResistance = (startAndStopZones >> 0) & 0xff;
            rightTrigger.effectForce = (startAndStopZones >> 8) & 0xff;
            rightTrigger.rangeForce = strength - 1;
            rightTrigger.nearReleaseStrength = 0x00;
            rightTrigger.nearMiddleStrength = 0x00;
            rightTrigger.pressedStrength = 0x00;
            rightTrigger.P6 = 0x00;
            rightTrigger.P7 = 0x00;
            rightTrigger.actuationFrequency = 0x00;
            rightTrigger.P9 = 0x00;
        }
        jc->modifying_lock.lock();

        jc->set_ds5_trigger_effects(leftTrigger, rightTrigger);

        jc->modifying_lock.unlock();

    }

}

void JslSetDS5TriggersVibration(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char position, unsigned char amplitude, unsigned char frequency)
{
    std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
    JoyShock* jc = GetJoyShockFromHandle(deviceId);
    if (jc != nullptr && jc->controller_type == ControllerType::s_ds)
    {
        if (position < 0)
            position = 0;
        if (position > 9)
            position = 9;
        if (amplitude < 1)
			amplitude = 1;
        if (amplitude > 8)
            amplitude = 8;
        if (frequency < 0)
			frequency = 0;

        unsigned char strengthValue = (amplitude - 1) & 0x07;
        unsigned int amplitudeZones = 0;
        unsigned short activeZones = 0;
        for (int i = position; i < 10; i++)
        {
            amplitudeZones = amplitudeZones | (strengthValue << (3 * i));
            activeZones = activeZones | (1 << i);
        }

        ds5_trigger_effect leftTrigger, rightTrigger;
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Left)
        {
            leftTrigger.bDirty = true;
            leftTrigger.motorMode = DS5_TRIGGER_VIBRATION;
            leftTrigger.startResistance = (activeZones >> 0) & 0xff;
			leftTrigger.effectForce = (activeZones >> 8) & 0xff;
			leftTrigger.rangeForce = (amplitudeZones >> 0) & 0xff;
			leftTrigger.nearReleaseStrength = (amplitudeZones >> 8) & 0xff;
			leftTrigger.nearMiddleStrength = (amplitudeZones >> 16) & 0xff;
			leftTrigger.pressedStrength = (amplitudeZones >> 24) & 0xff;
            leftTrigger.P6 = 0x00;
            leftTrigger.P7 = 0x00;
            leftTrigger.actuationFrequency = frequency;
            leftTrigger.P9 = 0x00;
        }
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Right)
        {
            rightTrigger.bDirty = true;
            rightTrigger.motorMode = DS5_TRIGGER_VIBRATION;
            rightTrigger.startResistance = (activeZones >> 0) & 0xff;
            rightTrigger.effectForce = (activeZones >> 8) & 0xff;
            rightTrigger.rangeForce = (amplitudeZones >> 0) & 0xff;
            rightTrigger.nearReleaseStrength = (amplitudeZones >> 8) & 0xff;
            rightTrigger.nearMiddleStrength = (amplitudeZones >> 16) & 0xff;
            rightTrigger.pressedStrength = (amplitudeZones >> 24) & 0xff;
            rightTrigger.P6 = 0x00;
            rightTrigger.P7 = 0x00;
            rightTrigger.actuationFrequency = frequency;
            rightTrigger.P9 = 0x00;
        }
        jc->modifying_lock.lock();

        jc->set_ds5_trigger_effects(leftTrigger, rightTrigger);

        jc->modifying_lock.unlock();

    }
}

void JslSetDS5TriggersMultiPosFeedback(int deviceId, EDS5AffectedTriggers affectedTriggers, const std::vector<unsigned char>& strength)
{
    std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
    JoyShock* jc = GetJoyShockFromHandle(deviceId);
    if (jc != nullptr && jc->controller_type == ControllerType::s_ds)
    {
		if (strength.size() == 0)
		{
			JslSetDS5TriggersOff(deviceId, affectedTriggers);
			return;
		}

		unsigned int forceZones = 0;
		unsigned short activeZones = 0;
		for (int i = 0; i < 10; i++)
		{
			unsigned char currentStrength;
			if(strength.size() <= i) currentStrength = 0;
			else currentStrength = strength[i];

			unsigned char forceValue = (currentStrength - 1) & 0x07;
			forceZones = forceZones | (forceValue << (3 * i));
			activeZones = activeZones | (1 << i);
		}

		ds5_trigger_effect leftTrigger, rightTrigger;
		if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Left)
		{
			leftTrigger.bDirty = true;
			leftTrigger.motorMode = DS5_TRIGGER_FEEDBACK;
			leftTrigger.startResistance = (activeZones >> 0) & 0xff;
			leftTrigger.effectForce = (activeZones >> 8) & 0xff;
			leftTrigger.rangeForce = (forceZones >> 0) & 0xff;
			leftTrigger.nearReleaseStrength = (forceZones >> 8) & 0xff;
			leftTrigger.nearMiddleStrength = (forceZones >> 16) & 0xff;
			leftTrigger.pressedStrength = (forceZones >> 24) & 0xff;
			leftTrigger.P6 = 0x00;
			leftTrigger.P7 = 0x00;
			leftTrigger.actuationFrequency = 0x00;
			leftTrigger.P9 = 0x00;
		}
		if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Right)
		{
			rightTrigger.bDirty = true;
			rightTrigger.motorMode = DS5_TRIGGER_FEEDBACK;
			rightTrigger.startResistance = (activeZones >> 0) & 0xff;
			rightTrigger.effectForce = (activeZones >> 8) & 0xff;
			rightTrigger.rangeForce = (forceZones >> 0) & 0xff;
			rightTrigger.nearReleaseStrength = (forceZones >> 8) & 0xff;
			rightTrigger.nearMiddleStrength = (forceZones >> 16) & 0xff;
			rightTrigger.pressedStrength = (forceZones >> 24) & 0xff;
			rightTrigger.P6 = 0x00;
			rightTrigger.P7 = 0x00;
			rightTrigger.actuationFrequency = 0x00;
			rightTrigger.P9 = 0x00;
		}
		jc->modifying_lock.lock();

		jc->set_ds5_trigger_effects(leftTrigger, rightTrigger);

		jc->modifying_lock.unlock();
	}
}

void JslSetDS5TriggersSlopeFeedback(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char startPosition, unsigned char endPosition, unsigned char startStrength, unsigned char endStrength)
{
    std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
    JoyShock* jc = GetJoyShockFromHandle(deviceId);
    if (jc != nullptr && jc->controller_type == ControllerType::s_ds)
    {
        if (startPosition < 2)
            startPosition = 2;
        if (startPosition > 7)
            startPosition = 7;
        if (endPosition <= startPosition)
            endPosition = startPosition + 1;
        if (endPosition > 8)
            endPosition = 8;
        if (startStrength < 1)
			startStrength = 1;
        if (startStrength > 8)
			startStrength = 8;
        if (endStrength < 1)
			endStrength = 1;
        if (endStrength > 8)
			endStrength = 8;

        std::vector<unsigned char> strength;
        float slope = 1.0f * (endStrength - startStrength) / (endPosition - startPosition);
        for (unsigned int i = startPosition; i < 10; i++)
		{
            if (i <= endPosition)
			{
                strength.push_back(static_cast<unsigned char>(std::lround(startStrength + slope * (i - startPosition))));
			}
            else
            {
                strength.push_back(endStrength);
			}
		}

		JslSetDS5TriggersMultiPosFeedback(deviceId, affectedTriggers, strength);
    }
}

void JslSetDS5TriggersMultiPosVibration(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char frequency, const std::vector<unsigned char>& amplitude)
{
    std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
    JoyShock* jc = GetJoyShockFromHandle(deviceId);
    if (jc != nullptr && jc->controller_type == ControllerType::s_ds)
    {
        if (amplitude.size() == 0)
        {
            JslSetDS5TriggersOff(deviceId, affectedTriggers);
            return;
        }

        unsigned int strengthZones = 0;
        unsigned short activeZones = 0;
        for (int i = 0; i < 10; i++)
        {
            unsigned char currentAmplitude;
            if (amplitude.size() <= i) currentAmplitude = 0;
            else currentAmplitude = amplitude[i];

            unsigned char strengthValue = (currentAmplitude - 1) & 0x07;
			strengthZones = strengthZones | (strengthValue << (3 * i));
            activeZones = activeZones | (1 << i);
        }

        ds5_trigger_effect leftTrigger, rightTrigger;
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Left)
        {
            leftTrigger.bDirty = true;
            leftTrigger.motorMode = DS5_TRIGGER_VIBRATION;
            leftTrigger.startResistance = (activeZones >> 0) & 0xff;
            leftTrigger.effectForce = (activeZones >> 8) & 0xff;
            leftTrigger.rangeForce = (strengthZones >> 0) & 0xff;
            leftTrigger.nearReleaseStrength = (strengthZones >> 8) & 0xff;
            leftTrigger.nearMiddleStrength = (strengthZones >> 16) & 0xff;
            leftTrigger.pressedStrength = (strengthZones >> 24) & 0xff;
            leftTrigger.P6 = 0x00;
            leftTrigger.P7 = 0x00;
            leftTrigger.actuationFrequency = frequency;
            leftTrigger.P9 = 0x00;
        }
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Right)
        {
            rightTrigger.bDirty = true;
            rightTrigger.motorMode = DS5_TRIGGER_VIBRATION;
            rightTrigger.startResistance = (activeZones >> 0) & 0xff;
            rightTrigger.effectForce = (activeZones >> 8) & 0xff;
            rightTrigger.rangeForce = (strengthZones >> 0) & 0xff;
            rightTrigger.nearReleaseStrength = (strengthZones >> 8) & 0xff;
            rightTrigger.nearMiddleStrength = (strengthZones >> 16) & 0xff;
            rightTrigger.pressedStrength = (strengthZones >> 24) & 0xff;
            rightTrigger.P6 = 0x00;
            rightTrigger.P7 = 0x00;
            rightTrigger.actuationFrequency = frequency;
            rightTrigger.P9 = 0x00;
        }
        jc->modifying_lock.lock();

        jc->set_ds5_trigger_effects(leftTrigger, rightTrigger);

        jc->modifying_lock.unlock();
    }
}

void JslSetDS5TriggersBow(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char startPosition, unsigned char endPosition, unsigned char strength, unsigned char snapForce)
{
    std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
    JoyShock* jc = GetJoyShockFromHandle(deviceId);
    if (jc != nullptr && jc->controller_type == ControllerType::s_ds)
    {
        if (startPosition < 2)
            startPosition = 2;
        if (startPosition > 7)
            startPosition = 7;
        if (endPosition <= startPosition)
            endPosition = startPosition + 1;
        if (endPosition > 8)
            endPosition = 8;
        if (strength < 1)
            strength = 1;
        if (strength > 8)
            strength = 8;


        unsigned short startAndStopZones = (1 << startPosition) | (1 << endPosition);
        unsigned int forcePair = (((strength - 1) & 0x07) << (3 * 0)) | (((snapForce - 1) & 0x07) << (3 * 1));


        ds5_trigger_effect leftTrigger, rightTrigger;
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Left)
        {
            leftTrigger.bDirty = true;
            leftTrigger.motorMode = DS5_TRIGGER_BOW;
            leftTrigger.startResistance = (startAndStopZones >> 0) & 0xff;
            leftTrigger.effectForce = (startAndStopZones >> 8) & 0xff;
            leftTrigger.rangeForce = (forcePair >> 0) & 0xff;
            leftTrigger.nearReleaseStrength = (forcePair >> 8) & 0xff;
            leftTrigger.nearMiddleStrength = 0x00;
            leftTrigger.pressedStrength = 0x00;
            leftTrigger.P6 = 0x00;
            leftTrigger.P7 = 0x00;
            leftTrigger.actuationFrequency = 0x00;
            leftTrigger.P9 = 0x00;
        }
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Right)
        {
            rightTrigger.bDirty = true;
            rightTrigger.motorMode = DS5_TRIGGER_BOW;
            rightTrigger.startResistance = (startAndStopZones >> 0) & 0xff;
            rightTrigger.effectForce = (startAndStopZones >> 8) & 0xff;
            rightTrigger.rangeForce = (forcePair >> 0) & 0xff;
            rightTrigger.nearReleaseStrength = (forcePair >> 8) & 0xff;
            rightTrigger.nearMiddleStrength = 0x00;
            rightTrigger.pressedStrength = 0x00;
            rightTrigger.P6 = 0x00;
            rightTrigger.P7 = 0x00;
            rightTrigger.actuationFrequency = 0x00;
            rightTrigger.P9 = 0x00;
        }
        jc->modifying_lock.lock();

        jc->set_ds5_trigger_effects(leftTrigger, rightTrigger);

        jc->modifying_lock.unlock();
    }
}

void JslSetDS5TriggersGalloping(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char startPosition, unsigned char endPosition, unsigned char firstFoot, unsigned char secondFoot, unsigned char frequency)
{
    std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
    JoyShock* jc = GetJoyShockFromHandle(deviceId);
    if (jc != nullptr && jc->controller_type == ControllerType::s_ds)
    {
        if (startPosition < 2)
            startPosition = 2;
        if (startPosition > 7)
            startPosition = 7;
        if (endPosition <= startPosition)
            endPosition = startPosition + 1;
        if (endPosition > 8)
            endPosition = 8;
        if (firstFoot < 2)
			firstFoot = 2;
        if (firstFoot > 7)
			firstFoot = 7;
        if (secondFoot <= firstFoot)
			secondFoot = firstFoot + 1;
        if (secondFoot > 8)
			secondFoot = 8;
        if (frequency < 0)
            frequency = 0;


        unsigned short startAndStopZones = (1 << startPosition) | (1 << endPosition);
        unsigned int timeAndRatio = ((secondFoot & 0x07) << (3 * 0)) | ((firstFoot & 0x07) << (3 * 1));

        ds5_trigger_effect leftTrigger, rightTrigger;
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Left)
        {
            leftTrigger.bDirty = true;
            leftTrigger.motorMode = DS5_TRIGGER_GALLOPING;
            leftTrigger.startResistance = (startAndStopZones >> 0) & 0xff;
            leftTrigger.effectForce = (startAndStopZones >> 8) & 0xff;
            leftTrigger.rangeForce = (timeAndRatio >> 0) & 0xff;
            leftTrigger.nearReleaseStrength = frequency;
            leftTrigger.nearMiddleStrength = 0x00;
            leftTrigger.pressedStrength = 0x00;
            leftTrigger.P6 = 0x00;
            leftTrigger.P7 = 0x00;
            leftTrigger.actuationFrequency = 0x00;
            leftTrigger.P9 = 0x00;
        }
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Right)
        {
            rightTrigger.bDirty = true;
            rightTrigger.motorMode = DS5_TRIGGER_GALLOPING;
            rightTrigger.startResistance = (startAndStopZones >> 0) & 0xff;
            rightTrigger.effectForce = (startAndStopZones >> 8) & 0xff;
            rightTrigger.rangeForce = (timeAndRatio >> 0) & 0xff;
            rightTrigger.nearReleaseStrength = frequency;
            rightTrigger.nearMiddleStrength = 0x00;
            rightTrigger.pressedStrength = 0x00;
            rightTrigger.P6 = 0x00;
            rightTrigger.P7 = 0x00;
            rightTrigger.actuationFrequency = 0x00;
            rightTrigger.P9 = 0x00;
        }
        jc->modifying_lock.lock();

        jc->set_ds5_trigger_effects(leftTrigger, rightTrigger);

        jc->modifying_lock.unlock();
    }
}

void JslSetDS5TriggersMachine(int deviceId, EDS5AffectedTriggers affectedTriggers, unsigned char startPosition, unsigned char endPosition, unsigned char amplitudeA, unsigned char amplitudeB, unsigned char frequency, unsigned char period)
{
    std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
    JoyShock* jc = GetJoyShockFromHandle(deviceId);
    if (jc != nullptr && jc->controller_type == ControllerType::s_ds)
    {
        if (startPosition < 2)
            startPosition = 2;
        if (startPosition > 7)
            startPosition = 7;
        if (endPosition <= startPosition)
            endPosition = startPosition + 1;
        if (endPosition > 8)
            endPosition = 8;
        if (amplitudeA > 7)
            amplitudeA = 7;
        if (amplitudeB > 7)
            amplitudeB = 7;
        if (frequency < 0)
            frequency = 0;

        unsigned short startAndStopZones = (1 << startPosition) | (1 << endPosition);
        unsigned int strengthPair = ((amplitudeA & 0x07) << (3 * 0)) | ((amplitudeB & 0x07) << (3 * 1));

        ds5_trigger_effect leftTrigger, rightTrigger;
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Left)
        {
            leftTrigger.bDirty = true;
            leftTrigger.motorMode = DS5_TRIGGER_MACHINE;
            leftTrigger.startResistance = (startAndStopZones >> 0) & 0xff;
            leftTrigger.effectForce = (startAndStopZones >> 8) & 0xff;
            leftTrigger.rangeForce = (strengthPair >> 0) & 0xff;
            leftTrigger.nearReleaseStrength = frequency;
            leftTrigger.nearMiddleStrength = period;
            leftTrigger.pressedStrength = 0x00;
            leftTrigger.P6 = 0x00;
            leftTrigger.P7 = 0x00;
            leftTrigger.actuationFrequency = 0x00;
            leftTrigger.P9 = 0x00;
        }
        if (affectedTriggers == EDS5AffectedTriggers::Both || affectedTriggers == EDS5AffectedTriggers::Right)
        {
            rightTrigger.bDirty = true;
            rightTrigger.motorMode = DS5_TRIGGER_MACHINE;
            rightTrigger.startResistance = (startAndStopZones >> 0) & 0xff;
            rightTrigger.effectForce = (startAndStopZones >> 8) & 0xff;
            rightTrigger.rangeForce = (strengthPair >> 0) & 0xff;
            rightTrigger.nearReleaseStrength = frequency;
            rightTrigger.nearMiddleStrength = period;
            rightTrigger.pressedStrength = 0x00;
            rightTrigger.P6 = 0x00;
            rightTrigger.P7 = 0x00;
            rightTrigger.actuationFrequency = 0x00;
            rightTrigger.P9 = 0x00;
        }
        jc->modifying_lock.lock();

        jc->set_ds5_trigger_effects(leftTrigger, rightTrigger);

        jc->modifying_lock.unlock();
    }
}

int encodeLowFreq(float lowFreq) {
    float lf = std::clamp(lowFreq, 40.875885f, 626.286133f);
    return std::lround(32 * std::log2(lf * 0.1)) - 0x40;
}
int encodeHighFreq(float highFreq) {
    float hf = std::clamp(highFreq, 81.75177f, 1252.572266f);
    return (std::lround(32 * std::log2(hf * 0.1)) - 0x60) * 4;
}
int encodeLowAmpli(float rawAmpli) {
    int encodedAmpli = 0;
    if (0 < rawAmpli && rawAmpli < 0.012) {
        encodedAmpli = 1;
    }
    else if (0.012 <= rawAmpli && rawAmpli < 0.112) {
        encodedAmpli = std::lround(4 * std::log2(rawAmpli * 110));
    }
    else if (0.112 <= rawAmpli && rawAmpli < 0.225) {
        encodedAmpli = std::lround(16 * std::log2(rawAmpli * 17));
    }
    else if (0.225 <= rawAmpli && rawAmpli <= 1) {
        encodedAmpli = std::lround(32 * std::log2(rawAmpli * 8.7));
    }
    return static_cast<int>(encodedAmpli / 2) + 64;
}
int encodeHighAmpli(float rawAmpli) {
    int encodedAmpli = 0;
    if (0 < rawAmpli && rawAmpli < 0.012) {
        encodedAmpli = 1;
    }
    else if (0.012 <= rawAmpli && rawAmpli < 0.112) {
        encodedAmpli = std::lround(4 * std::log2(rawAmpli * 110));
    }
    else if (0.112 <= rawAmpli && rawAmpli < 0.225) {
        encodedAmpli = std::lround(16 * std::log2(rawAmpli * 17));
    }
    else if (0.225 <= rawAmpli && rawAmpli <= 1) {
        encodedAmpli = std::lround(32 * std::log2(rawAmpli * 8.7));
    }
    return encodedAmpli * 2;
}

void JslEnableHDRumble(int deviceId)
{
    JoyShock* jc = GetJoyShockFromHandle(deviceId);
    if (jc != nullptr && jc->controller_type == ControllerType::n_switch)
    {
        jc->enable_joycon_rumble();
    }
}

void JslDisableHDRumble(int deviceId)
{
    JoyShock* jc = GetJoyShockFromHandle(deviceId);
    if (jc != nullptr && jc->controller_type == ControllerType::n_switch)
    {
        jc->disable_joycon_rumble();
    }
}

void JslSetHDRumble(int deviceId, float lowFreq, float lowAmpli, float highFreq, float highAmpli)
{
    JoyShock* jc = GetJoyShockFromHandle(deviceId);
    if (jc != nullptr && jc->controller_type == ControllerType::n_switch) {
        int lf = encodeLowFreq(lowFreq);
        int la = encodeLowAmpli(lowAmpli);
        int hf = encodeHighFreq(highFreq);
        int ha = encodeHighAmpli(highAmpli);

        jc->small_rumble = int(lowAmpli * 255);
        jc->big_rumble = int(highAmpli * 255);
        jc->set_joycon_rumble(
            lf, la, hf, ha,
            lf, la, hf, ha
        );
    }
}
void JslSetHDRumbleLR(int deviceId, float lowFreq_L, float lowAmpli_L, float highFreq_L, float highAmpli_L,
    float lowFreq_R, float lowAmpli_R, float highFreq_R, float highAmpli_R)
{
    JoyShock* jc = GetJoyShockFromHandle(deviceId);
    if (jc != nullptr && jc->controller_type == ControllerType::n_switch) {
        jc->small_rumble = int((lowAmpli_L + lowAmpli_R) / 2 * 255);
        jc->big_rumble = int((highAmpli_L + highAmpli_R) / 2 * 255);
        jc->set_joycon_rumble(
            encodeLowFreq(lowFreq_L),
            encodeLowAmpli(lowAmpli_L),
            encodeHighFreq(highFreq_L),
            encodeHighAmpli(highAmpli_L),
            encodeLowFreq(lowFreq_R),
            encodeLowAmpli(lowAmpli_R),
            encodeHighFreq(highFreq_R),
            encodeHighAmpli(highAmpli_R)
        );
    }
}

// set controller player number indicator (not all controllers have a number indicator which can be set, but that just means nothing will be done when this is called -- no harm)
void JslSetPlayerNumber(int deviceId, int number)
{
	std::shared_lock<std::shared_timed_mutex> lock(_connectedLock);
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr && jc->controller_type == ControllerType::n_switch) {
		jc->modifying_lock.lock();
		jc->player_number = number;
		unsigned char buf[64];
		memset(buf, 0x00, 0x40);
		buf[0] = (unsigned char)number;
		jc->send_subcommand(0x01, 0x30, buf, 1);
		jc->modifying_lock.unlock();
	}
	else if(jc != nullptr && jc->controller_type == ControllerType::s_ds) {
		jc->modifying_lock.lock();
	    jc->player_number = number;
        jc->set_ds5_rumble_light(
                jc->small_rumble,
                jc->big_rumble,
                jc->led_r,
                jc->led_g,
                jc->led_b,
                jc->player_number);
		jc->modifying_lock.unlock();
	}
}
