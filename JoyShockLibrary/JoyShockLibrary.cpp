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
static JoyShock* GetJoyShockFromHandle(int handle) {
	_connectedLock.lock_shared();
	auto iter = _joyshocks.find(handle);

	if (iter != _joyshocks.end())
	{
		_connectedLock.unlock_shared();
		return iter->second;
		// iter is item pair in the map. The value will be accessible as `iter->second`.
	}
	_connectedLock.unlock_shared();
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
				jc->push_cumulative_gyro(jc->imu_state.gyroX, jc->imu_state.gyroY, jc->imu_state.gyroZ);
				if (_pollCallback != nullptr || _pollTouchCallback != nullptr)
				{
					_callbackLock.lock_shared();
					if (_pollCallback != nullptr) {
						_pollCallback(jc->intHandle, jc->simple_state, jc->last_simple_state, jc->imu_state, jc->last_imu_state, jc->delta_time);
					}
					// touchpad will have its own callback so that it doesn't change the existing api
					if (jc->controller_type != ControllerType::n_switch && _pollTouchCallback != nullptr) {
						_pollTouchCallback(jc->intHandle, jc->touch_state, jc->last_touch_state, jc->delta_time);
					}
					_callbackLock.unlock_shared();
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
}

int JslConnectDevices()
{
	// for writing to console:
	//freopen("CONOUT$", "w", stdout);

	// most of the joycon and pro controller stuff here is thanks to mfosse's vjoy feeder
	// Enumerate and print the HID devices on the system
	struct hid_device_info *devs, *cur_dev;

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
				cur_dev->product_id == DS_USB;
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

	_connectedLock.unlock();
	_connectedLock.lock_shared();

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

	_connectedLock.unlock_shared();

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
	return GetJoyShockFromHandle(deviceId) != nullptr;
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
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state;
	}
	return {};
}
IMU_STATE JslGetIMUState(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->imu_state;
	}
	return {};
}
MOTION_STATE JslGetMotionState(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->get_motion_state();
	}
	return {};
}
TOUCH_STATE JslGetTouchState(int deviceId, bool previous)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return previous ? jc->last_touch_state : jc->touch_state;
	}
	return {};
}

bool JslGetTouchpadDimension(int deviceId, int &sizeX, int &sizeY)
{
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
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state.buttons;
	}
	return 0;
}

// get thumbsticks
float JslGetLeftX(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state.stickLX;
	}
	return 0.0f;
}
float JslGetLeftY(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state.stickLY;
	}
	return 0.0f;
}
float JslGetRightX(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state.stickRX;
	}
	return 0.0f;
}
float JslGetRightY(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state.stickRY;
	}
	return 0.0f;
}

// get triggers. Switch controllers don't have analogue triggers, but will report 0.0 or 1.0 so they can be used in the same way as others
float JslGetLeftTrigger(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state.lTrigger;
	}
	return 0.0f;
}
float JslGetRightTrigger(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->simple_state.rTrigger;
	}
	return 0.0f;
}

// get gyro
float JslGetGyroX(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->imu_state.gyroX;
	}
	return 0.0f;
}
float JslGetGyroY(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->imu_state.gyroY;
	}
	return 0.0f;
}
float JslGetGyroZ(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->imu_state.gyroZ;
	}
	return 0.0f;
}

void JslGetAndFlushAccumulatedGyro(int deviceId, float& gyroX, float& gyroY, float& gyroZ)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->get_and_flush_cumulative_gyro(gyroX, gyroY, gyroZ);
	}
	gyroX = gyroY = gyroZ = 0.f;
}

// get accelerometor
float JslGetAccelX(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->imu_state.accelX;
	}
	return 0.0f;
}
float JslGetAccelY(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->imu_state.accelY;
	}
	return 0.0f;
}
float JslGetAccelZ(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->imu_state.accelZ;
	}
	return 0.0f;
}

// get touchpad
int JslGetTouchId(int deviceId, bool secondTouch)
{
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
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->controller_type != ControllerType::n_switch ? 1 / 256.0f : 1.0f;
	}
	return 1.0f;
}
float JslGetPollRate(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->controller_type != ControllerType::n_switch ? 250.0f : 66.6667f;
	}
	return 0.0f;
}

// calibration
void JslResetContinuousCalibration(int deviceId) {
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		// TODO: might need a lock around this to prevent race conditions.
		// risk of miscounting samples
		jc->reset_continuous_calibration();
	}
}
void JslStartContinuousCalibration(int deviceId) {
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		jc->use_continuous_calibration = true;
		jc->cue_motion_reset = true;
	}
}
void JslPauseContinuousCalibration(int deviceId) {
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		jc->use_continuous_calibration = false;
	}
}
void JslSetAutomaticCalibration(int deviceId, bool enabled) {
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		jc->motion.SetCalibrationMode(enabled ? GamepadMotionHelpers::CalibrationMode::SensorFusion | GamepadMotionHelpers::CalibrationMode::Stillness : GamepadMotionHelpers::CalibrationMode::Manual);
	}
}
void JslGetCalibrationOffset(int deviceId, float& xOffset, float& yOffset, float& zOffset) {
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		jc->motion.GetCalibrationOffset(xOffset, yOffset, zOffset);
	}
}
void JslSetCalibrationOffset(int deviceId, float xOffset, float yOffset, float zOffset) {
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		jc->motion.SetCalibrationOffset(xOffset, yOffset, zOffset, 1);
	}
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

// what split type of controller is this?
int JslGetControllerType(int deviceId)
{
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
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->left_right;
	}
	return 0;
}
// what colour is the controller (not all controllers support this; those that don't will report white)
int JslGetControllerColour(int deviceId)
{
	// this just reports body colour. Switch controllers also give buttons colour, and in Pro's case, left and right grips
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->body_colour;
	}
	return 0xFFFFFF;
}
// set controller light colour (not all controllers have a light whose colour can be set, but that just means nothing will be done when this is called -- no harm)
void JslSetLightColour(int deviceId, int colour)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr && jc->controller_type == ControllerType::s_ds4) {
		jc->led_r = (colour >> 16) & 0xff;
		jc->led_g = (colour >> 8) & 0xff;
		jc->led_b = colour & 0xff;
		jc->set_ds4_rumble_light(
			jc->small_rumble,
			jc->big_rumble,
			jc->led_r,
			jc->led_g,
			jc->led_b);
	}
	else if(jc != nullptr && jc->controller_type == ControllerType::s_ds) {
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
	}
}
// set controller rumble
void JslSetRumble(int deviceId, int smallRumble, int bigRumble)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr && jc->controller_type == ControllerType::s_ds4) {
		jc->small_rumble = smallRumble;
		jc->big_rumble = bigRumble;
		jc->set_ds4_rumble_light(
			jc->small_rumble,
			jc->big_rumble,
			jc->led_r,
			jc->led_g,
			jc->led_b);
	}
    else if (jc != nullptr && jc->controller_type == ControllerType::s_ds) {
        jc->small_rumble = smallRumble;
        jc->big_rumble = bigRumble;
        jc->set_ds5_rumble_light(
                jc->small_rumble,
                jc->big_rumble,
                jc->led_r,
                jc->led_g,
                jc->led_b,
                jc->player_number);
    }
}
// set controller player number indicator (not all controllers have a number indicator which can be set, but that just means nothing will be done when this is called -- no harm)
void JslSetPlayerNumber(int deviceId, int number)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr && jc->controller_type == ControllerType::n_switch) {
		jc->player_number = number;
		unsigned char buf[64];
		memset(buf, 0x00, 0x40);
		buf[0] = (unsigned char)number;
		jc->send_subcommand(0x01, 0x30, buf, 1);
	}
	else if(jc != nullptr && jc->controller_type == ControllerType::s_ds) {
	    jc->player_number = number;
        jc->set_ds5_rumble_light(
                jc->small_rumble,
                jc->big_rumble,
                jc->led_r,
                jc->led_g,
                jc->led_b,
                jc->player_number);
	}
}
