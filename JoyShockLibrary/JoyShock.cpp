#pragma once

#include "JoyShockLibrary.h"
#include <bitset>
#include "hidapi.h"
#include <chrono>
#include <thread>
#include <unordered_map>
#include <atomic>
#include "tools.cpp"
#include <cstring>
#include "SensorFusion.cpp"


#ifdef __GNUC__
#define _wcsdup wcsdup
#endif

enum ControllerType { n_switch, s_ds4, s_ds };

// PS5 stuff
#define DS_VENDOR 0x054C
#define DS_USB 0x0CE6

// PS4 stuff
// http://www.psdevwiki.com/ps4/DS4-USB
// http://www.psdevwiki.com/ps4/DS4-BT
// http://eleccelerator.com/wiki/index.php?title=DualShock_4
// and a little bit of https://github.com/chrippa/ds4drv
#define DS4_VENDOR 0x054C
#define DS4_USB 0x05C4
#define DS4_USB_V2 0x09CC
#define DS4_USB_DONGLE 0x0BA0
#define DS4_BT 0x081F
// DS4 compatible controllers
#define BROOK_DS4_VENDOR 0x0C12
#define BROOK_DS4_USB 0x0E20

// Joycon and Pro conroller stuff is mostly from
// https://github.com/mfosse/JoyCon-Driver
// https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/
#define JOYCON_VENDOR 0x057e
#define JOYCON_L_BT 0x2006
#define JOYCON_R_BT 0x2007
#define PRO_CONTROLLER 0x2009
#define JOYCON_CHARGING_GRIP 0x200e
#define L_OR_R(lr) (lr == 1 ? 'L' : (lr == 2 ? 'R' : '?'))

// internal. Don't expose this in JoyShockLibrary.hpp:
typedef struct GYRO_AVERAGE_WINDOW {
	float x;
	float y;
	float z;
	float accelMagnitude;
	int numSamples;
} GYRO_AVERAGE_WINDOW;

class JoyShock {

public:

	hid_device * handle;
	int intHandle = 0;
	wchar_t *serial;

	std::string name;

	int deviceNumber = 0;// left(0) or right(1) vjoy

	int left_right = 0;// 1: left joycon, 2: right joycon, 3: pro controller

	std::chrono::steady_clock::time_point last_polled;
	float delta_time = 1.0;

	JOY_SHOCK_STATE simple_state = {};
	JOY_SHOCK_STATE last_simple_state = {};

	IMU_STATE imu_state = {};
	IMU_STATE last_imu_state = {};

	TOUCH_STATE touch_state = {};
	TOUCH_STATE last_touch_state = {};

	Motion motion;

	int8_t dstick;
	uint8_t battery;

	int global_count = 0;

	// calibration data:
	struct brcm_hdr {
		uint8_t cmd;
		uint8_t rumble[9];
	};

	struct brcm_cmd_01 {
		uint8_t subcmd;
		uint32_t offset;
		uint8_t size;
	};

	int timing_byte = 0x0;

	float acc_cal_coeff[3] = {0.0f, 0.0f, 0.0f};
	float gyro_cal_coeff[3] = {0.0f, 0.0f, 0.0f};
	float cal_x[1] = { 0.0f };
	float cal_y[1] = { 0.0f };

	bool has_user_cal_stick_l = false;
	bool has_user_cal_stick_r = false;
	bool has_user_cal_sensor = false;

	ControllerType controller_type = ControllerType::n_switch;
	bool is_usb = false;

	unsigned char small_rumble = 0;
	unsigned char big_rumble = 0;
	unsigned char led_r = 0;
	unsigned char led_g = 0;
	unsigned char led_b = 0;

	unsigned int body_colour = 0xFFFFFF;
	unsigned int button_colour = 0xFFFFFF;
	unsigned int left_grip_colour = 0xFFFFFF;
	unsigned int right_grip_colour = 0xFFFFFF;

	int player_number = 0;

	bool cancel_thread = false;
	std::thread* thread;

	// for calibration:
	bool use_continuous_calibration = false;
	bool cue_motion_reset = false;
	float offset_x = 0.0f;
	float offset_y = 0.0f;
	float offset_z = 0.0f;
	float accel_magnitude = 1.0f;

	// for continuous calibration
	static const int num_gyro_average_windows = 16;
	int gyro_average_window_front_index = 0;
	int gyro_average_window_seconds = 600; // TODO: Function to set this for this device and for all devices. Different players might have different settings
	GYRO_AVERAGE_WINDOW gyro_average_window[num_gyro_average_windows];

	unsigned char factory_stick_cal[0x12];
	unsigned char device_colours[0xC];
	unsigned char user_stick_cal[0x16];
	unsigned char sensor_model[0x6];
	unsigned char stick_model[0x24];
	unsigned char factory_sensor_cal[0x18];
	unsigned char user_sensor_cal[0x1A];
	uint16_t factory_sensor_cal_calm[0xC];
	uint16_t user_sensor_cal_calm[0xC];
	int16_t sensor_cal[0x2][0x3];
	uint16_t stick_cal_x_l[0x3];
	uint16_t stick_cal_y_l[0x3];
	uint16_t stick_cal_x_r[0x3];
	uint16_t stick_cal_y_r[0x3];

	uint32_t crc_table[256] = {
		0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA,
		0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
		0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988,
		0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
		0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE,
		0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
		0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC,
		0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
		0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
		0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
		0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940,
		0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
		0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116,
		0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
		0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924,
		0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
		0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A,
		0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
		0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818,
		0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
		0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E,
		0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
		0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C,
		0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
		0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2,
		0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
		0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
		0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
		0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086,
		0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
		0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4,
		0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
		0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A,
		0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
		0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8,
		0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
		0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE,
		0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
		0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC,
		0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
		0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252,
		0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
		0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60,
		0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
		0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
		0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
		0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04,
		0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
		0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A,
		0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
		0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38,
		0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
		0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E,
		0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
		0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C,
		0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45,
		0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2,
	    0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
		0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0,
		0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
		0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6,
		0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
		0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
		0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
	};


	//// https://docs.microsoft.com/en-us/openspecs/office_protocols/ms-abs/06966aa2-70da-4bf9-8448-3355f277cd77
	uint32_t crc_32(unsigned char* buf, int length) {
		uint32_t result = 0xFFFFFFFF;
		int index = 0;
		while (index < length) {
			result = crc_table[(result & 0xFF) ^ buf[index]] ^ (result >> 8);
			index++;
		}
		return result ^ 0xFFFFFFFF;
	}

	void enable_gyro_ds4_bt(unsigned char *buf, int bufLength)
	{
		// gyro is enabled by getting feature report 0x05 on BT controllers.
		// in addition, this request is also responsible for getting current calibration info.
		buf[0] = 0x05; // controller calibration request for BT

		hid_get_feature_report(handle, buf, 41);
		//hid_write(handle, buf, 38);
		//hid_read_timeout(handle, buf, bufLength, 100);
	}

public:
	JoyShock(struct hid_device_info *dev, int uniqueHandle) {

		if (dev->product_id == JOYCON_CHARGING_GRIP) {

			if (dev->interface_number == 0 || dev->interface_number == -1) {
				this->name = std::string("Joy-Con (R)");
				this->left_right = 2;// right joycon
				this->is_usb = true;
			}
			else if (dev->interface_number == 1) {
				this->name = std::string("Joy-Con (L)");
				this->left_right = 1;// left joycon
				this->is_usb = true;
			}
		}

		if (dev->product_id == JOYCON_L_BT) {
			this->name = std::string("Joy-Con (L)");
			this->left_right = 1;// left joycon
		}
		else if (dev->product_id == JOYCON_R_BT) {
			this->name = std::string("Joy-Con (R)");
			this->left_right = 2;// right joycon
		}
		else if (dev->product_id == PRO_CONTROLLER) {
			this->name = std::string("Pro Controller");
			this->left_right = 3;// left joycon
		}

		if (dev->product_id == DS4_BT ||
			dev->product_id == DS4_USB ||
			dev->product_id == DS4_USB_DONGLE ||
			dev->product_id == DS4_USB_V2) {
			this->name = std::string("DualShock 4");
			this->left_right = 3; // left and right?
			this->controller_type = ControllerType::s_ds4;
			this->is_usb = (dev->product_id != DS4_BT);
		}
		
		if (dev->product_id == BROOK_DS4_USB) {
			this->name = std::string("DualShock 4");
			this->left_right = 3; // left and right?
			this->controller_type = ControllerType::s_ds4;
			this->is_usb = true; // this controller is wired
		}

		if (dev->product_id == DS_USB) {
			this->name = std::string("DualSense");
			this->left_right = 3; // left and right?
			this->controller_type = ControllerType::s_ds;
			this->is_usb = true; // for now, only usb
		}

		this->serial = _wcsdup(dev->serial_number);
		this->intHandle = uniqueHandle;

		//printf("Found device %c: %ls %s\n", L_OR_R(this->left_right), this->serial, dev->path);
		this->handle = hid_open_path(dev->path);

		if (this->controller_type == ControllerType::s_ds4) {
			unsigned char buf[64];
			memset(buf, 0, 64);

			enable_gyro_ds4_bt(buf, 64);

			hid_read_timeout(handle, buf, 64, 100);
			// choose between BT and USB
			if (buf[0] == 0x11) {
				this->is_usb = false;
			}
		}
		else if(this->controller_type == ControllerType::s_ds) {
            unsigned char buf[64];
            memset(buf, 0, 64);

            // We can reuse the same command on the DS5 to enable Full Mode.
            enable_gyro_ds4_bt(buf, 64);

            hid_read_timeout(handle, buf, 64, 100);

            // The DS's protocol is literally so similar to the DS4 that we can reuse the same reports to get the same results.
            // Meet the new boss - the same as the old boss.
            if (buf[0] == 0x31) {
                this->is_usb = false;
            }

        }

		// initialise continuous calibration windows
		reset_continuous_calibration();

		if (this->handle == nullptr) {
			//printf("Could not open serial %ls: %s\n", this->serial, strerror(errno));
			throw;
		}
	}

	void reset_continuous_calibration() {
		for (int i = 0; i < num_gyro_average_windows; i++) {
			this->gyro_average_window[i] = {};
		}
	}

	int get_gyro_average_window_total_samples_for_device() {
		if (this->controller_type == ControllerType::s_ds4) {
			// 250 samples per second
			return 250 * this->gyro_average_window_seconds;
		}
		// 67 samples per second
		return 67 * this->gyro_average_window_seconds;
	}

	int get_gyro_average_window_single_samples_for_device() {
		return get_gyro_average_window_total_samples_for_device() / (num_gyro_average_windows - 2);
	}

	void push_sensor_samples(float x, float y, float z, float accelMagnitude) {
		// push samples
		GYRO_AVERAGE_WINDOW* windowPointer = this->gyro_average_window + this->gyro_average_window_front_index;
		if (windowPointer->numSamples >= get_gyro_average_window_single_samples_for_device()) {
			// next
			this->gyro_average_window_front_index = (this->gyro_average_window_front_index + num_gyro_average_windows - 1) % num_gyro_average_windows;
			this->gyro_average_window[this->gyro_average_window_front_index] = {};
			windowPointer = this->gyro_average_window + this->gyro_average_window_front_index;
		}
		// accumulate
		windowPointer->numSamples++;
		windowPointer->x += x;
		windowPointer->y += y;
		windowPointer->z += z;
		windowPointer->accelMagnitude += accelMagnitude;
	}

	void get_average_gyro(float& x, float& y, float& z, float& accelMagnitude) {
		float weight = 0.0f;
		float totalX = 0.0f;
		float totalY = 0.0f;
		float totalZ = 0.0f;
		float totalAccelMagnitude = 0.0f;
		int samplesAccumulated = 0;
		int samplesWanted = this->get_gyro_average_window_total_samples_for_device();
		float samplesPerWindow = (float)(this->get_gyro_average_window_single_samples_for_device());
		//int numSamplesAvailable = 0;
		//for (int i = 0; i < num_gyro_average_windows && samplesWanted > 0; i++) {
		//	numSamplesAvailable += this->gyro_average_window[i].numSamples;
		//}

		// get the average of each window
		// and a weighted average of all those averages, weighted by the number of samples it has compared to how many samples a full window will have.
		// this isn't a perfect rolling average. the last window, which has more samples than we need, will have its contribution weighted according to how many samples it would ideally have for the current span of time.
		for (int i = 0; i < num_gyro_average_windows && samplesWanted > 0; i++) {
			int cycledIndex = (i + this->gyro_average_window_front_index) % num_gyro_average_windows;
			GYRO_AVERAGE_WINDOW* windowPointer = this->gyro_average_window + cycledIndex;
			if (windowPointer->numSamples == 0)
			{
				continue;
			}
			float thisWeight = 1.0f;
			float fNumSamples = (float)(windowPointer->numSamples);
			if (samplesWanted < windowPointer->numSamples)
			{
				thisWeight = ((float)(samplesWanted)) / windowPointer->numSamples;
				samplesWanted = 0;
			}
			else
			{
				thisWeight = fNumSamples / samplesPerWindow;
				samplesWanted -= windowPointer->numSamples;
			}

			//printf("[%.1f] [%d] [%.1f]; ", \
						//	windowPointer->y,
//	(int)fNumSamples,
//	thisWeight);

			totalX += (windowPointer->x / fNumSamples) * thisWeight;
			totalY += (windowPointer->y / fNumSamples) * thisWeight;
			totalZ += (windowPointer->z / fNumSamples) * thisWeight;
			totalAccelMagnitude += (windowPointer->accelMagnitude / fNumSamples) * thisWeight;
			weight += thisWeight;
		}
		if (weight > 0.0) {
			x = totalX / weight;
			y = totalY / weight;
			z = totalZ / weight;
			accelMagnitude = totalAccelMagnitude / weight;
		}
		//printf("{%.1f, %.1f, %.1f} {%d}\n", x, y, z, numSamplesAvailable);
		//printf("{%.1f} {%d}\n", y, numSamplesAvailable);
	}

	MOTION_STATE get_motion_state()
	{
		return motion.GetMotionState();
	}

	bool hid_exchange(hid_device *handle, unsigned char *buf, int len) {
		if (!handle) return false;

		int res;

		res = hid_write(handle, buf, len);

		res = hid_read_timeout(handle, buf, 0x40, 1000);
		if (res == 0)
		{
			return false;
		}
		return true;
	}


	bool send_command(int command, uint8_t *data, int len) {
		unsigned char buf[0x40];
		memset(buf, 0, 0x40);

		if (is_usb) {
			buf[0x00] = 0x80;
			buf[0x01] = 0x92;
			buf[0x03] = 0x31;
		}

		buf[is_usb ? 0x8 : 0x0] = command;
		if (data != nullptr && len != 0) {
			memcpy(buf + (is_usb ? 0x9 : 0x1), data, len);
		}

		if (!hid_exchange(this->handle, buf, len + (is_usb ? 0x9 : 0x1)))
		{
			return false;
		}

		if (data) {
			memcpy(data, buf, 0x40);
		}
		return true;
	}

	bool send_subcommand(int command, int subcommand, uint8_t *data, int len) {
		unsigned char buf[0x40];
		memset(buf, 0, 0x40);

		uint8_t rumble_base[9] = { std::uint8_t((++global_count) & 0xF), 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40 };
		memcpy(buf, rumble_base, 9);

		if (global_count > 0xF) {
			global_count = 0x0;
		}

		// set neutral rumble base only if the command is vibrate (0x01)
		// if set when other commands are set, might cause the command to be misread and not executed
		//if (subcommand == 0x01) {
		//	uint8_t rumble_base[9] = { (++global_count) & 0xF, 0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40 };
		//	memcpy(buf + 10, rumble_base, 9);
		//}

		buf[9] = subcommand;
		if (data && len != 0) {
			memcpy(buf + 10, data, len);
		}

		if (!send_command(command, buf, 10 + len))
		{
			return false;
		}

		if (data) {
			memcpy(data, buf, 0x40); //TODO
		}
		return true;
	}

	void rumble(int frequency, int intensity) {

		unsigned char buf[0x400];
		memset(buf, 0, 0x40);

		// intensity: (0, 8)
		// frequency: (0, 255)

		//	 X	AA	BB	 Y	CC	DD
		//[0 1 x40 x40 0 1 x40 x40] is neutral.


		//for (int j = 0; j <= 8; j++) {
		//	buf[1 + intensity] = 0x1;//(i + j) & 0xFF;
		//}

		buf[1 + 0 + intensity] = 0x1;
		buf[1 + 4 + intensity] = 0x1;

		// Set frequency to increase
		if (this->left_right == 1) {
			buf[1 + 0] = frequency;// (0, 255)
		}
		else {
			buf[1 + 4] = frequency;// (0, 255)
		}

		// set non-blocking:
		hid_set_nonblocking(this->handle, 1);

		send_command(0x10, (uint8_t*)buf, 0x9);
	}

	bool get_switch_controller_info() {
		bool result = false;

		memset(factory_stick_cal, 0, 0x12);
		memset(device_colours, 0, 0xC);
		memset(user_stick_cal, 0, 0x16);
		memset(sensor_model, 0, 0x6);
		memset(stick_model, 0, 0x12);
		memset(factory_sensor_cal, 0, 0x18);
		memset(user_sensor_cal, 0, 0x1A);
		memset(factory_sensor_cal_calm, 0, 0xC);
		memset(user_sensor_cal_calm, 0, 0xC);
		memset(sensor_cal, 0, sizeof(sensor_cal));
		memset(stick_cal_x_l, 0, sizeof(stick_cal_x_l));
		memset(stick_cal_y_l, 0, sizeof(stick_cal_y_l));
		memset(stick_cal_x_r, 0, sizeof(stick_cal_x_r));
		memset(stick_cal_y_r, 0, sizeof(stick_cal_y_r));


		if (!get_spi_data(0x6020, 0x18, factory_sensor_cal)) { return false; }
		if (!get_spi_data(0x603D, 0x12, factory_stick_cal)) { return false; }
		if (!get_spi_data(0x6050, 0xC, device_colours)) { return false; }
		if (!get_spi_data(0x6080, 0x6, sensor_model)) { return false; }
		if (!get_spi_data(0x6086, 0x12, stick_model)) { return false; }
		if (!get_spi_data(0x6098, 0x12, &stick_model[0x12])) { return false; }
		if (!get_spi_data(0x8010, 0x16, user_stick_cal)) { return false; }
		if (!get_spi_data(0x8026, 0x1A, user_sensor_cal)) { return false; }


		// get stick calibration data:

		// factory calibration:

		if (this->left_right == 1 || this->left_right == 3) {
			stick_cal_x_l[1] = (factory_stick_cal[4] << 8) & 0xF00 | factory_stick_cal[3];
			stick_cal_y_l[1] = (factory_stick_cal[5] << 4) | (factory_stick_cal[4] >> 4);
			stick_cal_x_l[0] = stick_cal_x_l[1] - ((factory_stick_cal[7] << 8) & 0xF00 | factory_stick_cal[6]);
			stick_cal_y_l[0] = stick_cal_y_l[1] - ((factory_stick_cal[8] << 4) | (factory_stick_cal[7] >> 4));
			stick_cal_x_l[2] = stick_cal_x_l[1] + ((factory_stick_cal[1] << 8) & 0xF00 | factory_stick_cal[0]);
			stick_cal_y_l[2] = stick_cal_y_l[1] + ((factory_stick_cal[2] << 4) | (factory_stick_cal[2] >> 4));

		}

		if (this->left_right == 2 || this->left_right == 3) {
			stick_cal_x_r[1] = (factory_stick_cal[10] << 8) & 0xF00 | factory_stick_cal[9];
			stick_cal_y_r[1] = (factory_stick_cal[11] << 4) | (factory_stick_cal[10] >> 4);
			stick_cal_x_r[0] = stick_cal_x_r[1] - ((factory_stick_cal[13] << 8) & 0xF00 | factory_stick_cal[12]);
			stick_cal_y_r[0] = stick_cal_y_r[1] - ((factory_stick_cal[14] << 4) | (factory_stick_cal[13] >> 4));
			stick_cal_x_r[2] = stick_cal_x_r[1] + ((factory_stick_cal[16] << 8) & 0xF00 | factory_stick_cal[15]);
			stick_cal_y_r[2] = stick_cal_y_r[1] + ((factory_stick_cal[17] << 4) | (factory_stick_cal[16] >> 4));

		}


		// if there is user calibration data:
		if ((user_stick_cal[0] | user_stick_cal[1] << 8) == 0xA1B2) {
			stick_cal_x_l[1] = (user_stick_cal[6] << 8) & 0xF00 | user_stick_cal[5];
			stick_cal_y_l[1] = (user_stick_cal[7] << 4) | (user_stick_cal[6] >> 4);
			stick_cal_x_l[0] = stick_cal_x_l[1] - ((user_stick_cal[9] << 8) & 0xF00 | user_stick_cal[8]);
			stick_cal_y_l[0] = stick_cal_y_l[1] - ((user_stick_cal[10] << 4) | (user_stick_cal[9] >> 4));
			stick_cal_x_l[2] = stick_cal_x_l[1] + ((user_stick_cal[3] << 8) & 0xF00 | user_stick_cal[2]);
			stick_cal_y_l[2] = stick_cal_y_l[1] + ((user_stick_cal[4] << 4) | (user_stick_cal[3] >> 4));
			//FormJoy::myform1->textBox_lstick_ucal->Text = String::Format(L"L Stick User:\r\nCenter X,Y: ({0:X3}, {1:X3})\r\nX: [{2:X3} - {4:X3}] Y: [{3:X3} - {5:X3}]",
			//stick_cal_x_l[1], stick_cal_y_l[1], stick_cal_x_l[0], stick_cal_y_l[0], stick_cal_x_l[2], stick_cal_y_l[2]);
		}
		else {
			//FormJoy::myform1->textBox_lstick_ucal->Text = L"L Stick User:\r\nNo calibration";
			//printf("no user Calibration data for left stick.\n");
		}

		if ((user_stick_cal[0xB] | user_stick_cal[0xC] << 8) == 0xA1B2) {
			stick_cal_x_r[1] = (user_stick_cal[14] << 8) & 0xF00 | user_stick_cal[13];
			stick_cal_y_r[1] = (user_stick_cal[15] << 4) | (user_stick_cal[14] >> 4);
			stick_cal_x_r[0] = stick_cal_x_r[1] - ((user_stick_cal[17] << 8) & 0xF00 | user_stick_cal[16]);
			stick_cal_y_r[0] = stick_cal_y_r[1] - ((user_stick_cal[18] << 4) | (user_stick_cal[17] >> 4));
			stick_cal_x_r[2] = stick_cal_x_r[1] + ((user_stick_cal[20] << 8) & 0xF00 | user_stick_cal[19]);
			stick_cal_y_r[2] = stick_cal_y_r[1] + ((user_stick_cal[21] << 4) | (user_stick_cal[20] >> 4));
			//FormJoy::myform1->textBox_rstick_ucal->Text = String::Format(L"R Stick User:\r\nCenter X,Y: ({0:X3}, {1:X3})\r\nX: [{2:X3} - {4:X3}] Y: [{3:X3} - {5:X3}]",
			//stick_cal_x_r[1], stick_cal_y_r[1], stick_cal_x_r[0], stick_cal_y_r[0], stick_cal_x_r[2], stick_cal_y_r[2]);
		}
		else {
			//FormJoy::myform1->textBox_rstick_ucal->Text = L"R Stick User:\r\nNo calibration";
			//printf("no user Calibration data for right stick.\n");
		}

		// get gyro / accelerometer calibration data:

		// factory calibration:

		// Acc cal origin position
		sensor_cal[0][0] = uint16_to_int16(factory_sensor_cal[0] | factory_sensor_cal[1] << 8);
		sensor_cal[0][1] = uint16_to_int16(factory_sensor_cal[2] | factory_sensor_cal[3] << 8);
		sensor_cal[0][2] = uint16_to_int16(factory_sensor_cal[4] | factory_sensor_cal[5] << 8);

		// Gyro cal origin position
		sensor_cal[1][0] = uint16_to_int16(factory_sensor_cal[0xC] | factory_sensor_cal[0xD] << 8);
		sensor_cal[1][1] = uint16_to_int16(factory_sensor_cal[0xE] | factory_sensor_cal[0xF] << 8);
		sensor_cal[1][2] = uint16_to_int16(factory_sensor_cal[0x10] | factory_sensor_cal[0x11] << 8);


		//hex_dump(user_sensor_cal, 0x14);

		// user calibration:
		if ((user_sensor_cal[0x0] | user_sensor_cal[0x1] << 8) == 0xA1B2) {
			//printf("User calibration available\n");
			//if (true) {
			//FormJoy::myform1->textBox_6axis_ucal->Text = L"6-Axis User (XYZ):\r\nAcc:  ";
			//for (int i = 0; i < 0xC; i = i + 6) {
			//	FormJoy::myform1->textBox_6axis_ucal->Text += String::Format(L"{0:X4} {1:X4} {2:X4}\r\n      ",
			//		user_sensor_cal[i + 2] | user_sensor_cal[i + 3] << 8,
			//		user_sensor_cal[i + 4] | user_sensor_cal[i + 5] << 8,
			//		user_sensor_cal[i + 6] | user_sensor_cal[i + 7] << 8);
			//}
			// Acc cal origin position
			sensor_cal[0][0] = uint16_to_int16(user_sensor_cal[2] | user_sensor_cal[3] << 8);
			sensor_cal[0][1] = uint16_to_int16(user_sensor_cal[4] | user_sensor_cal[5] << 8);
			sensor_cal[0][2] = uint16_to_int16(user_sensor_cal[6] | user_sensor_cal[7] << 8);
			//FormJoy::myform1->textBox_6axis_ucal->Text += L"\r\nGyro: ";
			//for (int i = 0xC; i < 0x18; i = i + 6) {
			//	FormJoy::myform1->textBox_6axis_ucal->Text += String::Format(L"{0:X4} {1:X4} {2:X4}\r\n      ",
			//		user_sensor_cal[i + 2] | user_sensor_cal[i + 3] << 8,
			//		user_sensor_cal[i + 4] | user_sensor_cal[i + 5] << 8,
			//		user_sensor_cal[i + 6] | user_sensor_cal[i + 7] << 8);
			//}
			// Gyro cal origin position
			sensor_cal[1][0] = uint16_to_int16(user_sensor_cal[0xE] | user_sensor_cal[0xF] << 8);
			sensor_cal[1][1] = uint16_to_int16(user_sensor_cal[0x10] | user_sensor_cal[0x11] << 8);
			sensor_cal[1][2] = uint16_to_int16(user_sensor_cal[0x12] | user_sensor_cal[0x13] << 8);
		}
		else {
			//FormJoy::myform1->textBox_6axis_ucal->Text = L"\r\n\r\nUser:\r\nNo calibration";
		}

		// Use SPI calibration and convert them to SI acc unit
		acc_cal_coeff[0] = (float)(1.0 / (float)(16384 - uint16_to_int16(sensor_cal[0][0]))) * 4.0f  * 9.8f;
		acc_cal_coeff[1] = (float)(1.0 / (float)(16384 - uint16_to_int16(sensor_cal[0][1]))) * 4.0f  * 9.8f;
		acc_cal_coeff[2] = (float)(1.0 / (float)(16384 - uint16_to_int16(sensor_cal[0][2]))) * 4.0f  * 9.8f;

		// Use SPI calibration and convert them to SI gyro unit
		gyro_cal_coeff[0] = (float)(936.0 / (float)(13371 - uint16_to_int16(sensor_cal[1][0])) * 0.01745329251994);
		gyro_cal_coeff[1] = (float)(936.0 / (float)(13371 - uint16_to_int16(sensor_cal[1][1])) * 0.01745329251994);
		gyro_cal_coeff[2] = (float)(936.0 / (float)(13371 - uint16_to_int16(sensor_cal[1][2])) * 0.01745329251994);

		// Device colours
		body_colour =
			(((int)device_colours[0]) << 16) +
			(((int)device_colours[1]) << 8) +
			(((int)device_colours[2]));
		button_colour =
			(((int)device_colours[3]) << 16) +
			(((int)device_colours[4]) << 8) +
			(((int)device_colours[5]));
		left_grip_colour =
			(((int)device_colours[6]) << 16) +
			(((int)device_colours[7]) << 8) +
			(((int)device_colours[8]));
		right_grip_colour =
			(((int)device_colours[9]) << 16) +
			(((int)device_colours[10]) << 8) +
			(((int)device_colours[11]));

		printf("Body: %#08x; Buttons: %#08x; Left Grip: %#08x; Right Grip: %#08x;\n",
			body_colour,
			button_colour,
			left_grip_colour,
			right_grip_colour);

		//hex_dump(reinterpret_cast<unsigned char*>(sensor_cal[0]), 6);
		//hex_dump(reinterpret_cast<unsigned char*>(sensor_cal[1]), 6);

		return true;
	}

	void enable_IMU(unsigned char *buf, int bufLength) {
		memset(buf, 0, bufLength);

		// Enable IMU data
		printf("Enabling IMU data...\n");
		if (controller_type == ControllerType::s_ds4)
		{
			if (is_usb)
			{
				init_ds4_bt();
				enable_gyro_ds4_bt(buf, bufLength);
			}
			else
			{
				init_ds4_usb();
			}
		}
		else
		{
			buf[0] = 0x01; // Enabled
			send_subcommand(0x1, 0x40, buf, 1);
		}
	}

	bool init_usb() {
		unsigned char buf[0x400];
		memset(buf, 0, 0x400);

		// set blocking:
		// this insures we get the MAC Address
		hid_set_nonblocking(this->handle, 0);

		//Get MAC Left
		printf("Getting MAC...\n");
		memset(buf, 0x00, 0x40);
		buf[0] = 0x80;
		buf[1] = 0x01;
		hid_exchange(this->handle, buf, 0x2);

		//if (buf[2] == 0x3) {
		//	printf("%s disconnected!\n", this->name.c_str());
		//}
		//else {
		//	printf("Found %s, MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", this->name.c_str(), buf[9], buf[8], buf[7], buf[6], buf[5], buf[4]);
		//}

		// set non-blocking:
		//hid_set_nonblocking(jc->handle, 1);

		// Do handshaking
		printf("Doing handshake...\n");
		memset(buf, 0x00, 0x40);
		buf[0] = 0x80;
		buf[1] = 0x02;
		hid_exchange(this->handle, buf, 0x2);

		// Switch baudrate to 3Mbit
		printf("Switching baudrate...\n");
		memset(buf, 0x00, 0x40);
		buf[0] = 0x80;
		buf[1] = 0x03;
		hid_exchange(this->handle, buf, 0x2);

		//Do handshaking again at new baudrate so the firmware pulls pin 3 low?
		printf("Doing handshake...\n");
		memset(buf, 0x00, 0x40);
		buf[0] = 0x80;
		buf[1] = 0x02;
		hid_exchange(this->handle, buf, 0x2);

		//Only talk HID from now on
		printf("Only talk HID...\n");
		memset(buf, 0x00, 0x40);
		buf[0] = 0x80;
		buf[1] = 0x04;
		hid_exchange(this->handle, buf, 0x2);

		// Enable vibration
		printf("Enabling vibration...\n");
		memset(buf, 0x00, 0x400);
		buf[0] = 0x01; // Enabled
		send_subcommand(0x1, 0x48, buf, 1);

		enable_IMU(buf, 0x400);

		printf("Getting calibration data...\n");
		bool result = get_switch_controller_info();

		if (result)
		{
			printf("Successfully initialized %s!\n", this->name.c_str());
		}
		else
		{
			printf("Could not initialise %s! Will try again later.\n", this->name.c_str());
		}
		return result;
	}

	bool init_bt() {
		bool result = true;
		unsigned char buf[0x40];
		memset(buf, 0, 0x40);
		printf("Initialising Bluetooth connection...\n");

		// set blocking to ensure command is recieved:
		hid_set_nonblocking(this->handle, 0);

		// first, check if this is a USB connection
		buf[0] = 0x80;
		buf[1] = 0x01;
		hid_write(this->handle, buf, 2);
		// wait for up to 5 messages for a USB acknowledgement
		for (int idx = 0; idx < 5; idx++)
		{
			if (hid_read_timeout(this->handle, buf, 0x40, 200) && buf[0] == 0x81)
			{
				//printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
				//	buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10]);
				printf("Attempting USB connection\n");

				// it's usb!
				is_usb = true;

				init_usb();
				return 1;

				break;
			}
			//printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			//	buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10]);
			printf("Not a USB response...\n");
		}
		memset(buf, 0, 0x40);
		//if (hid_exchange(this->handle, buf, 2))
		//{
		//	printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
		//		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10]);
		//	printf("Attempting USB connection\n");
		//	// it's usb!
		//	is_usb = true;
		//
		//	init_usb();
		//	return 1;
		//}
		buf[1] = 0x00;

		// Enable vibration
		printf("Enabling vibration...\n");
		buf[0] = 0x01; // Enabled
		send_subcommand(0x1, 0x48, buf, 1);

		//printf("Set vibration\n");

		// Enable IMU data
		enable_IMU(buf, 0x40);

		// Set input report mode (to push at 60hz)
		// x00	Active polling mode for IR camera data. Answers with more than 300 bytes ID 31 packet
		// x01	Active polling mode
		// x02	Active polling mode for IR camera data.Special IR mode or before configuring it ?
		// x21	Unknown.An input report with this ID has pairing or mcu data or serial flash data or device info
		// x23	MCU update input report ?
		// 30	NPad standard mode. Pushes current state @60Hz. Default in SDK if arg is not in the list
		// 31	NFC mode. Pushes large packets @60Hz
		printf("Set input report mode to 0x30...\n");
		buf[0] = 0x30;
		send_subcommand(0x01, 0x03, buf, 1);

		// @CTCaer

		// get calibration data:
		printf("Getting calibration data...\n");
		result = get_switch_controller_info();

		return result;
	}

	void init_ds4_bt() {
		printf("initialise, set colour\n");
		unsigned char buf[78];
		memset(buf, 0, 78);

		//buf[0] = 0x11;
		//buf[1] = 0x80;
		//buf[3] = 0xff;

		// https://github.com/Ryochan7/DS4Windows/blob/jay/DS4Windows/DS4Library/DS4Device.cs
		buf[0] = 0x15;
		buf[1] = 0xC0 | 1;
		buf[2] = 0xA0;
		buf[3] = 0xf7;
		buf[4] = 0x04;

		//// https://github.com/chrippa/ds4drv/blob/master/ds4drv/device.py
		//buf[0] = 0xa2; // 0x80;
		////buf[1] = 0xff;
		//// trying to do colour stuff
		//// http://eleccelerator.com/wiki/index.php?title=DualShock_4
		//// this is only for bt
		//buf[1] = 0x11;
		//buf[2] = 0xc0;
		//buf[3] = 0x20;
		//buf[4] = 0xf3;
		//buf[5] = 0x04;
		//// rumble
		//buf[7] = 0xFF;
		//buf[8] = 0x00;
		//// colour
		//buf[9] = 0x00;
		//buf[10] = 0x00;
		//buf[11] = 0x00;
		//// flash time
		//buf[12] = 0xff;
		//buf[13] = 0x00;
		//// now we need a CRC-32 of previous bytes
		//uint32_t crc = crc_32(buf, 75);
		//buf[75] = (crc >> 24) & 0xFF;
		//buf[76] = (crc >> 16) & 0xFF;
		//buf[77] = (crc >> 8) & 0xFF;
		//buf[78] = crc & 0xFF;

		//// https://github.com/chrippa/ds4drv/blob/master/ds4drv/device.py
		//buf[0] = 0x80;
		//buf[1] = 0xff;
		//// trying to do colour stuff
		//// http://eleccelerator.com/wiki/index.php?title=DualShock_4
		//// this is only for bt
		//buf[2] = 0x11;
		//// rumble
		//buf[6] = 0xFF;
		//buf[7] = 0xFF;	
		//// colour
		//buf[8] = 0xFF; // 0x00;
		//buf[9] = 0x80; // 0x00;
		//buf[10] = 0x00;
		//// flash time
		//buf[11] = 0xff;
		//buf[12] = 0x00;
		//// now we need a CRC-32 of previous bytes
		//uint32_t crc = crc_32(buf, 75);
		//buf[75] = (crc >> 24) & 0xFF;
		//buf[76] = (crc >> 16) & 0xFF;
		//buf[77] = (crc >> 8) & 0xFF;
		//buf[78] = crc & 0xFF;

		// set blocking:
		// this insures we get the MAC Address
		hid_set_nonblocking(this->handle, 0);

		hid_write(handle, buf, 78);

		// initialise stuff
		memset(factory_stick_cal, 0, 0x12);
		memset(device_colours, 0, 0xC);
		memset(user_stick_cal, 0, 0x16);
		memset(sensor_model, 0, 0x6);
		memset(stick_model, 0, 0x12);
		memset(factory_sensor_cal, 0, 0x18);
		memset(user_sensor_cal, 0, 0x1A);
		memset(factory_sensor_cal_calm, 0, 0xC);
		memset(user_sensor_cal_calm, 0, 0xC);
		memset(sensor_cal, 0, sizeof(sensor_cal));
		memset(stick_cal_x_l, 0, sizeof(stick_cal_x_l));
		memset(stick_cal_y_l, 0, sizeof(stick_cal_y_l));
		memset(stick_cal_x_r, 0, sizeof(stick_cal_x_r));
		memset(stick_cal_y_r, 0, sizeof(stick_cal_y_r));
		stick_cal_x_l[0] =
			stick_cal_y_l[0] =
			stick_cal_x_r[0] =
			stick_cal_y_r[0] = 0;
		stick_cal_x_l[1] =
			stick_cal_y_l[1] =
			stick_cal_x_r[1] =
			stick_cal_y_r[1] = 127;
		stick_cal_x_l[2] =
			stick_cal_y_l[2] =
			stick_cal_x_r[2] =
			stick_cal_y_r[2] = 255;
		//// Acc cal origin position
		//sensor_cal[0][0] = 0;
		//sensor_cal[0][1] = 0;
		//sensor_cal[0][2] = 0;
		//
		//// Gyro cal origin position
		//sensor_cal[1][0] = 0;
		//sensor_cal[1][1] = 0;
		//sensor_cal[1][2] = 0;

		enable_gyro_ds4_bt(buf, 78);
	}

	// placeholder to get things working quickly. overdue for a refactor
	void init_ds_usb() {
		// initialise stuff
		memset(factory_stick_cal, 0, 0x12);
		memset(device_colours, 0, 0xC);
		memset(user_stick_cal, 0, 0x16);
		memset(sensor_model, 0, 0x6);
		memset(stick_model, 0, 0x12);
		memset(factory_sensor_cal, 0, 0x18);
		memset(user_sensor_cal, 0, 0x1A);
		memset(factory_sensor_cal_calm, 0, 0xC);
		memset(user_sensor_cal_calm, 0, 0xC);
		memset(sensor_cal, 0, sizeof(sensor_cal));
		memset(stick_cal_x_l, 0, sizeof(stick_cal_x_l));
		memset(stick_cal_y_l, 0, sizeof(stick_cal_y_l));
		memset(stick_cal_x_r, 0, sizeof(stick_cal_x_r));
		memset(stick_cal_y_r, 0, sizeof(stick_cal_y_r));
		stick_cal_x_l[0] =
			stick_cal_y_l[0] =
			stick_cal_x_r[0] =
			stick_cal_y_r[0] = 0;
		stick_cal_x_l[1] =
			stick_cal_y_l[1] =
			stick_cal_x_r[1] =
			stick_cal_y_r[1] = 127;
		stick_cal_x_l[2] =
			stick_cal_y_l[2] =
			stick_cal_x_r[2] =
			stick_cal_y_r[2] = 255;
	}

	// this is mostly copied from init_usb() below, but modified to speak DS4
	void init_ds4_usb() {
		unsigned char buf[31];
		memset(buf, 0, 31);

		// report id?
		buf[0] = 0x05;
		// I dunno what this is
		buf[1] = 0xf7;
		buf[2] = 0x04;
		//// http://eleccelerator.com/wiki/index.php?title=DualShock_4
		//// https://github.com/chrippa/ds4drv/blob/master/ds4drv/device.py
		//// rumble
		//buf[4] = 0x00;
		//buf[5] = 0x00;
		//// colour
		//buf[6] = 0x00;
		////buf[7] = 0xff;
		//buf[7] = 0x00;
		//buf[8] = 0x00;
		//// flash time
		//buf[9] = 0xff;
		//buf[10] = 0x00;
		// now we need a CRC-32 of previous bytes
		//uint32_t = crc_32(buf, 75);
		//buf[75] = 

		// set blocking:
		// this insures we get the MAC Address
		hid_set_nonblocking(this->handle, 0);

		hid_write(handle, buf, 31);

		// initialise stuff
		memset(factory_stick_cal, 0, 0x12);
		memset(device_colours, 0, 0xC);
		memset(user_stick_cal, 0, 0x16);
		memset(sensor_model, 0, 0x6);
		memset(stick_model, 0, 0x12);
		memset(factory_sensor_cal, 0, 0x18);
		memset(user_sensor_cal, 0, 0x1A);
		memset(factory_sensor_cal_calm, 0, 0xC);
		memset(user_sensor_cal_calm, 0, 0xC);
		memset(sensor_cal, 0, sizeof(sensor_cal));
		memset(stick_cal_x_l, 0, sizeof(stick_cal_x_l));
		memset(stick_cal_y_l, 0, sizeof(stick_cal_y_l));
		memset(stick_cal_x_r, 0, sizeof(stick_cal_x_r));
		memset(stick_cal_y_r, 0, sizeof(stick_cal_y_r));
		stick_cal_x_l[0] =
			stick_cal_y_l[0] =
			stick_cal_x_r[0] =
			stick_cal_y_r[0] = 0;
		stick_cal_x_l[1] =
			stick_cal_y_l[1] =
			stick_cal_x_r[1] =
			stick_cal_y_r[1] = 127;
		stick_cal_x_l[2] =
			stick_cal_y_l[2] =
			stick_cal_x_r[2] =
			stick_cal_y_r[2] = 255;
		//// Acc cal origin position
		//sensor_cal[0][0] = 0;
		//sensor_cal[0][1] = 0;
		//sensor_cal[0][2] = 0;
		//
		//// Gyro cal origin position
		//sensor_cal[1][0] = 0;
		//sensor_cal[1][1] = 0;
		//sensor_cal[1][2] = 0;
	}

	void deinit_ds4_bt() {
		// TODO. For now, init, which stops rumbling and disables light
		init_ds4_bt();
	}

	// TODO: implement this
	void deinit_ds4_usb() {
		unsigned char buf[40];
		memset(buf, 0, 40);

		// report id?
		buf[0] = 0x05;
		// don't know what this is
		buf[1] = 0xff;
		// rumble
		buf[4] = 0x00;
		buf[5] = 0x00;
		// colour
		buf[6] = 0x00;
		buf[7] = 0x00;
		buf[8] = 0x00;
		// flash time
		buf[9] = 0x00;
		buf[10] = 0x00;
		// now we need a CRC-32 of previous bytes
		//uint32_t = crc_32(buf, 75);
		//buf[75] = 

		// set non-blocking
		hid_set_nonblocking(this->handle, 1);

		hid_write(handle, buf, 31);
	}

	void deinit_usb() {
		unsigned char buf[0x40];
		memset(buf, 0x00, 0x40);

		//Let the Joy-Con talk BT again    
		buf[0] = 0x80;
		buf[1] = 0x05;
		hid_exchange(this->handle, buf, 0x2);
		//printf("Deinitialized %s\n", this->name.c_str());
	}

	void set_ds5_rumble_light(unsigned char smallRumble, unsigned char bigRumble,
                           unsigned char colourR,
                           unsigned char colourG,
                           unsigned char colourB,
                           unsigned char playerlights) {
	    if(!is_usb) {
	        set_ds5_rumble_light_bt(smallRumble, bigRumble, colourR, colourG, colourB, playerlights);
	    }
	    else {
            set_ds5_rumble_light_usb(smallRumble, bigRumble, colourR, colourG, colourB, playerlights);
        }

	}

	void set_ds4_rumble_light(unsigned char smallRumble, unsigned char bigRumble,
		unsigned char colourR,
		unsigned char colourG,
		unsigned char colourB) {
		if (!is_usb) {
			set_ds4_rumble_light_bt(smallRumble, bigRumble, colourR, colourG, colourB);
		}
		else {
			set_ds4_rumble_light_usb(smallRumble, bigRumble, colourR, colourG, colourB);
		}
	}

	void set_ds4_rumble_light_usb(unsigned char smallRumble, unsigned char bigRumble,
		unsigned char colourR,
		unsigned char colourG,
		unsigned char colourB) {
		// todo: based on bluetoothness, switch report id to 0x11, offset everything by 2 -- basically use init stuff as basis
		unsigned char buf[40];
		memset(buf, 0, 40);

		// report id?
		buf[0] = 0x05;
		// don't know what this is
		buf[1] = 0xff;
		// rumble
		buf[4] = smallRumble;
		buf[5] = bigRumble;
		// colour
		buf[6] = colourR;
		buf[7] = colourG;
		buf[8] = colourB;
		// flash time
		buf[9] = 0xff;
		buf[10] = 0x00;
		// now we need a CRC-32 of previous bytes
		//uint32_t = crc_32(buf, 75);
		//buf[75] = 

		hid_write(handle, buf, 31);
	}

	void set_ds4_rumble_light_bt(unsigned char smallRumble, unsigned char bigRumble,
		unsigned char colourR,
		unsigned char colourG,
		unsigned char colourB) {
		unsigned char buf[79];
		memset(buf, 0, 79);

		// https://github.com/chrippa/ds4drv/blob/master/ds4drv/device.py
		//buf[0] = 0xa2; // 0x80;
		//buf[1] = 0xff;
		// trying to do colour stuff
		// http://eleccelerator.com/wiki/index.php?title=DualShock_4
		// this is only for bt

		buf[0] = 0xa2; // Output report header, needs to be included in crc32
		buf[1] = 0x11; // Output report 0x11
		buf[2] = 0xc0; // HID + CRC according to hid-sony
		buf[3] = 0x20; // ????
		buf[4] = 0x07; // Set blink + leds + motor
		buf[5] = 0x00;
		buf[6] = 0x00;
		// rumble
		buf[7] = smallRumble;
		buf[8] = bigRumble;
		// colour
		buf[9] = colourR;
		buf[10] = colourG;
		buf[11] = colourB;
		// flash time
		buf[12] = 0xff;
		buf[13] = 0x00;
		// now we need a CRC-32 of previous bytes

		/*
		// test
        buf[0] = 0xa2; // Output report header, needs to be included in crc32
        buf[1] = 0x11; // Output report 0x11
        buf[2] = 0xc0; // HID + CRC according to hid-sony
        buf[3] = 0x00; // ????
        buf[4] = 0x07; // Set blink + leds + motor
        buf[5] = 0x00;
        buf[6] = 0x00;
        buf[7] = 0xff;
        buf[8] = 0xff;
        buf[9] = 0xff;
        buf[10] = 0xff;
        buf[11] = 0xff;
        buf[12] = 0xff;
        buf[22] = 0x43;
        buf[23] = 0x43;
        buf[25] = 0x4d;
        buf[26] = 0x85;
*/

		uint32_t crc = crc_32(buf, 75);
		memcpy(&buf[75], &crc, 4);
		//buf[75] = (crc >> 24) & 0xFF;
		//buf[76] = (crc >> 16) & 0xFF;
		//buf[77] = (crc >> 8) & 0xFF;
		//buf[78] = crc & 0xFF;

		hid_write(handle, &buf[1], 78);
	}

    void set_ds5_rumble_light_usb(unsigned char smallRumble, unsigned char bigRumble,
                                 unsigned char colourR,
                                 unsigned char colourG,
                                 unsigned char colourB,
                                 unsigned char playerlights) { // DS5 actually has player lights.
        unsigned char buf[79];
        memset(buf, 0, 79);

        // https://github.com/Ryochan7/DS4Windows/blob/jay/DS4Windows/DS4Library/InputDevices/DualSenseDevice.cs
        // DS4Windows to the rescue.
        // Also thanks to Neilk1 for sharing his doc on the DS5 protocol.

        // Header & Report Information
        buf[0] = 0xa2; // Output report header, needs to be included in crc32
        buf[1] = 0x02; // DualSense output report is 0x02 for USB
        //buf[1] = 0x02; // DATA (0x02)


        buf[2] = 0x03;

        buf[3] = 0x54; // Toggle LED Strips, player lights, motor effect. Ignore Mic LED

        // Rumble emulation bytes.
        buf[4] = smallRumble;
        buf[5] = bigRumble;

        // 7-10 are mostly just audio settings.

        // Mute Button state. 0x00 = off, 0x01 = solid, 0x02 = pulsating.
        buf[10] = 0x00;

        // Skip to about 41, since we are ignoring trigger effect data.
        // Enable LED brightness
        buf[40] = 0x02; // ???
        buf[41] = 0x02;
        buf[44] = 0x02;

        // Controls the player lights, which the DS5 has.
        // Last two bits are unused - unset them to avoid issues.
        buf[45] = playerlights;
        buf[45] &= ~(1 << 7);
        buf[45] &= ~(1 << 8);

        // colour
        buf[46] = colourR;
        buf[47] = colourG;
        buf[48] = colourB;

        // USB does not send CRC32

        //uint32_t crc = crc_32(buf, 74);
        //memcpy(&buf[74], &crc, 4);
        //buf[75] = (crc >> 24) & 0xFF;
        //buf[76] = (crc >> 16) & 0xFF;
        //buf[77] = (crc >> 8) & 0xFF;
        //buf[78] = crc & 0xFF;

        hid_write(handle, &buf[1], 74);
    }

	// Calling the Dualsense anything but the DS5 is confusing, since DS also = DualShock, and the DualSense is the PS5 Controller anyway
    void set_ds5_rumble_light_bt(unsigned char smallRumble, unsigned char bigRumble,
                                 unsigned char colourR,
                                 unsigned char colourG,
                                 unsigned char colourB,
                                 unsigned char playerlights) { // DS5 actually has player lights.
        unsigned char buf[79];
        memset(buf, 0, 79);

        // https://github.com/Ryochan7/DS4Windows/blob/jay/DS4Windows/DS4Library/InputDevices/DualSenseDevice.cs
        // DS4Windows to the rescue.
        // Also thanks to Neilk1 for sharing his doc on the DS5 protocol.

        // Header & Report Information
        buf[0] = 0xa2; // Output report header, needs to be included in crc32
        buf[1] = 0x31; // DualSense output report is 0x31
        buf[2] = 0x02; // DATA (0x02)

        // Comment stolen from DS4Windows:
        // 0x01 Set the main motors (also requires flag 0x02)
        // 0x02 Set the main motors (also requires flag 0x01)
        // 0x04 Set the right trigger motor
        // 0x08 Set the left trigger motor
        // 0x10 Enable modification of audio volume
        // 0x20 Enable internal speaker (even while headset is connected)
        // 0x40 Enable modification of microphone volume
        // 0x80 Enable internal mic (even while headset is connected)
        buf[3] = 0x03;

        // Comment stolen from DS4Windows:
        // 0x01 Toggling microphone LED, 0x02 Toggling Audio/Mic Mute
        // 0x04 Toggling LED strips on the sides of the Touchpad, 0x08 Turn off all LED lights
        // 0x10 Toggle player LED lights below Touchpad, 0x20 ???
        // 0x40 Adjust overall motor/effect power, 0x80 ???
        buf[4] = 0x54; // Toggle LED Strips, player lights, motor effect. Ignore Mic LED

        // Rumble emulation bytes.
        buf[5] = smallRumble;
        buf[6] = bigRumble;

        // 7-10 are mostly just audio settings.

        // Mute Button state. 0x00 = off, 0x01 = solid, 0x02 = pulsating.
        buf[11] = 0x00;

        // Skip to about 41, since we are ignoring trigger effect data.
        // Enable LED brightness
        buf[41] = 0x02; // ???
        buf[44] = 0x02;
        buf[45] = 0x02;

        // Last two bits are unused - unset them to avoid issues.
        buf[46] = playerlights;
        buf[46] &= ~(1 << 7);
        buf[46] &= ~(1 << 8);

        // colour
        buf[47] = colourR;
        buf[48] = colourG;
        buf[49] = colourB;

        uint32_t crc = crc_32(buf, 75);
        memcpy(&buf[75], &crc, 4);
        //buf[75] = (crc >> 24) & 0xFF;
        //buf[76] = (crc >> 16) & 0xFF;
        //buf[77] = (crc >> 8) & 0xFF;
        //buf[78] = crc & 0xFF;

        hid_write(handle, &buf[1], 78);
    }

	//// mfosse credits Hypersect (Ryan Juckett), but I've removed deadzones so the consuming application can deal with them
	//// http://blog.hypersect.com/interpreting-analog-sticks/
	void CalcAnalogStick2
	(
		float &pOutX,       // out: resulting stick X value
		float &pOutY,       // out: resulting stick Y value
		uint16_t x,              // in: initial stick X value
		uint16_t y,              // in: initial stick Y value
		uint16_t x_calc[3],      // calc -X, CenterX, +X
		uint16_t y_calc[3]       // calc -Y, CenterY, +Y
	)
	{

		float x_f, y_f;

		// convert to float based on calibration and valid ranges per +/-axis
		x = clamp(x, x_calc[0], x_calc[2]);
		y = clamp(y, y_calc[0], y_calc[2]);
		if (x >= x_calc[1]) {
			x_f = (float)(x - x_calc[1]) / (float)(x_calc[2] - x_calc[1]);
		}
		else {
			x_f = -((float)(x - x_calc[1]) / (float)(x_calc[0] - x_calc[1]));
		}
		if (y >= y_calc[1]) {
			y_f = (float)(y - y_calc[1]) / (float)(y_calc[2] - y_calc[1]);
		}
		else {
			y_f = -((float)(y - y_calc[1]) / (float)(y_calc[0] - y_calc[1]));
		}

		pOutX = x_f;
		pOutY = y_f;
	}

	// SPI (@CTCaer):
	bool get_spi_data(uint32_t offset, const uint16_t read_len, uint8_t *test_buf) {
		int res;
		uint8_t buf[0x100];
		while (1) {
			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr *)buf;
			auto pkt = (brcm_cmd_01 *)(hdr + 1);
			hdr->cmd = 1;
			hdr->rumble[0] = timing_byte;

			buf[1] = timing_byte;

			timing_byte++;
			if (timing_byte > 0xF) {
				timing_byte = 0x0;
			}
			pkt->subcmd = 0x10;
			pkt->offset = offset;
			pkt->size = read_len;

			for (int i = 11; i < 22; ++i) {
				buf[i] = buf[i + 3];
			}

			res = hid_write(handle, buf, sizeof(*hdr) + sizeof(*pkt));

			res = hid_read_timeout(handle, buf, sizeof(buf), 1000);
			if (res == 0)
			{
				return false;
			}

			if ((*(uint16_t*)&buf[0xD] == 0x1090) && (*(uint32_t*)&buf[0xF] == offset)) {
				break;
			}
		}
		if (res >= 0x14 + read_len) {
			for (int i = 0; i < read_len; i++) {
				test_buf[i] = buf[0x14 + i];
			}
		}

		return true;
	}

	int write_spi_data(uint32_t offset, const uint16_t write_len, uint8_t* test_buf) {
		int res;
		uint8_t buf[0x100];
		int error_writing = 0;
		while (1) {
			memset(buf, 0, sizeof(buf));
			auto hdr = (brcm_hdr *)buf;
			auto pkt = (brcm_cmd_01 *)(hdr + 1);
			hdr->cmd = 1;
			hdr->rumble[0] = timing_byte;
			timing_byte++;
			if (timing_byte > 0xF) {
				timing_byte = 0x0;
			}
			pkt->subcmd = 0x11;
			pkt->offset = offset;
			pkt->size = write_len;
			for (int i = 0; i < write_len; i++) {
				buf[0x10 + i] = test_buf[i];
			}
			res = hid_write(handle, buf, sizeof(*hdr) + sizeof(*pkt) + write_len);

			res = hid_read(handle, buf, sizeof(buf));

			if (*(uint16_t*)&buf[0xD] == 0x1180)
				break;

			error_writing++;
			if (error_writing == 125) {
				return 1;
			}
		}

		return 0;

	}
};
