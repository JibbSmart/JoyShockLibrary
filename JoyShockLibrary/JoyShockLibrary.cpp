// JoyShockLibrary.cpp : Defines the exported functions for the DLL application.
//

#include "JoyShockLibrary.h"
#include <bitset>
#include "hidapi.h"
#include <chrono>
#include <thread>
#include <shared_mutex>
#include <unordered_map>
#include <atomic>
#include "GamepadMotion.hpp"
#include "JoyShock.cpp"
#include "InputHelpers.cpp"

std::shared_timed_mutex _callbackLock;
void(*_pollCallback)(int, JOY_SHOCK_STATE, JOY_SHOCK_STATE, IMU_STATE, IMU_STATE, float) = nullptr;
void(*_pollTouchCallback)(int, TOUCH_STATE, TOUCH_STATE, float) = nullptr;
std::unordered_map<int, JoyShock*> _joyshocks;
// https://stackoverflow.com/questions/41206861/atomic-increment-and-return-counter
static std::atomic<int> _joyshockHandleCounter;
static int GetUniqueHandle()
{
	return _joyshockHandleCounter++;
}

// https://stackoverflow.com/questions/25144887/map-unordered-map-prefer-find-and-then-at-or-try-at-catch-out-of-range
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
		int res = hid_read_timeout(jc->handle, buf, 64, 1000);

		if (res == 0)
		{
			numTimeOuts++;
			if (numTimeOuts == 10)
			{
				printf("Controller %d timed out\n", jc->handle);
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
						printf("Attempting to re-initialise controller %d\n", jc->handle);
						if (jc->init_usb())
						{
							numTimeOuts = 0;
						}
					}
					else
					{
						printf("Attempting to re-initialise controller %d\n", jc->handle);
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
}

int JslConnectDevices()
{
	// for writing to console:
	//freopen("CONOUT$", "w", stdout);
	if (_joyshocks.size() > 0) {
		// already connected? clean up old stuff!
		JslDisconnectAndDisposeAll();
	}

	// most of the joycon and pro controller stuff here is thanks to mfosse's vjoy feeder
	int read;	// number of bytes read
	int written;// number of bytes written
	const char *device_name;

	// Enumerate and print the HID devices on the system
	struct hid_device_info *devs, *cur_dev;

	int res = hid_init();

	devs = hid_enumerate(JOYCON_VENDOR, 0x0);
	cur_dev = devs;
	while (cur_dev) {

		// identify by vendor:
		if (cur_dev->vendor_id == JOYCON_VENDOR) {

			// bluetooth, left / right joycon:
			if (cur_dev->product_id == JOYCON_L_BT || cur_dev->product_id == JOYCON_R_BT) {
				//printf("JOYCON\n");
				JoyShock* jc = new JoyShock(cur_dev, GetUniqueHandle());
				_joyshocks.emplace(jc->intHandle, jc);
			}

			// pro controller:
			if (cur_dev->product_id == PRO_CONTROLLER) {
				JoyShock* jc = new JoyShock(cur_dev, GetUniqueHandle());
				//printf("PRO\n");
				_joyshocks.emplace(jc->intHandle, jc);
			}

			// charging grip:
			if (cur_dev->product_id == JOYCON_CHARGING_GRIP) {
				JoyShock* jc = new JoyShock(cur_dev, GetUniqueHandle());
				//printf("GRIP\n");
				_joyshocks.emplace(jc->intHandle, jc);
			}

		}


		cur_dev = cur_dev->next;
	}
	hid_free_enumeration(devs);

	// find dualshocks
	devs = hid_enumerate(DS4_VENDOR, 0x0);
	cur_dev = devs;
	while (cur_dev) {
		// do we need to confirm vendor id if this is what we asked for?
		if (cur_dev->vendor_id == DS4_VENDOR) {
			// usb or bluetooth ds4:
			printf("DS4\n");
			if (cur_dev->product_id == DS4_USB ||
				cur_dev->product_id == DS4_USB_V2 ||
				cur_dev->product_id == DS4_USB_DONGLE ||
				cur_dev->product_id == DS4_BT) {
				JoyShock* jc = new JoyShock(cur_dev, GetUniqueHandle());
				_joyshocks.emplace(jc->intHandle, jc);
			}
		}

		cur_dev = cur_dev->next;
	}
	hid_free_enumeration(devs);
	
	// find Brook controllers (DS4 compatible)
	devs = hid_enumerate(BROOK_DS4_VENDOR, 0x0);
	cur_dev = devs;
	while (cur_dev) {
		// brook usb ds4:
		printf("Brook DS4\n");
		if (cur_dev->product_id == BROOK_DS4_USB) {
			JoyShock* jc = new JoyShock(cur_dev, GetUniqueHandle());
			_joyshocks.emplace(jc->intHandle, jc);
		}

		cur_dev = cur_dev->next;
	}
	hid_free_enumeration(devs);

	// find dualsenses
	devs = hid_enumerate(DS_VENDOR, 0x0);
	cur_dev = devs;
	while (cur_dev) {
		// do we need to confirm vendor id if this is what we asked for?
		if (cur_dev->vendor_id == DS_VENDOR) {
			// usb or bluetooth ds4:
			printf("DS\n");
			if (cur_dev->product_id == DS_USB) {
				JoyShock* jc = new JoyShock(cur_dev, GetUniqueHandle());
				_joyshocks.emplace(jc->intHandle, jc);
			}
		}

		cur_dev = cur_dev->next;
	}
	hid_free_enumeration(devs);

	// init joyshocks:
	for (std::pair<int, JoyShock*> pair : _joyshocks)
	{
		JoyShock* jc = pair.second;
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
	int i = 0;
	for (std::pair<int, JoyShock*> pair : _joyshocks)
	{
		JoyShock *jc = pair.second;
		if (jc->controller_type != ControllerType::n_switch) {
			// don't do joycon LED stuff with DS4
			continue;
		}

		// player LED
		memset(buf, 0x00, 0x40);
		buf[0] = (unsigned char)(i+1);
		jc->send_subcommand(0x01, 0x30, buf, 1);
		i++;
	}

	// now let's get polling!
	for (std::pair<int, JoyShock*> pair : _joyshocks)
	{
		JoyShock* jc = pair.second;
		// threads for polling
		jc->thread = new std::thread(pollIndividualLoop, jc);
	}

	return _joyshocks.size();
}

int JslGetConnectedDeviceHandles(int* deviceHandleArray, int size)
{
	int i = 0;
	for (std::pair<int, JoyShock*> pair : _joyshocks)
	{
		if (i >= size) {
			break;
		}
		deviceHandleArray[i] = pair.first;
		i++;
	}
	return i; // return num actually found
}

void JslDisconnectAndDisposeAll()
{
	// no more callback
	JslSetCallback(nullptr);
	JslSetTouchCallback(nullptr);

	for (std::pair<int, JoyShock*> pair : _joyshocks)
	{
		JoyShock* jc = pair.second;
		// threads for polling
		jc->cancel_thread = true;
		jc->thread->join();
		delete jc->thread;
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
		delete pair.second;
	}
	_joyshocks.clear();

	// Finalize the hidapi library
	int res = hid_exit();
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
			return 1.0 / 128.0;
		}
		else {
			if (jc->left_right == 2) // right joycon has no calibration for left stick
			{
				return 1.0 / (jc->stick_cal_x_r[2] - jc->stick_cal_x_r[1]);
			}
			else {
				return 1.0 / (jc->stick_cal_x_l[2] - jc->stick_cal_x_l[1]);
			}
		}
	}
	return 0.0f;
}
float JslGetTriggerStep(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->controller_type != ControllerType::n_switch ? 1 / 256.0 : 1.0;
	}
	return 1.0f;
}
float JslGetPollRate(int deviceId)
{
	JoyShock* jc = GetJoyShockFromHandle(deviceId);
	if (jc != nullptr) {
		return jc->controller_type != ControllerType::n_switch ? 250.0 : 66.6667;
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
    else if (jc != nullptr && jc->controller_type == ControllerType::s_ds4) {
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
