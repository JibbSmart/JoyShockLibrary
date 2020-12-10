#include "JoyShockLibrary.h"
#include <cstring>
#include "JoyShock.h"

void JoyShock::enable_gyro_ds4_bt(unsigned char* buf, int bufLength)
{
	// enable gyro?
	// assume everything already zeroed
	buf[0] = 0xa3;
	buf[1] = 0x02;
	buf[2] = 0x01;
	//buf[3] = 0x00;
	//buf[4] = 0xff;
	//buf[5] = 0xff;
	//buf[6] = 0x01;
	//buf[7] = 0x00;
	//buf[8] = 0x5e;
	//buf[9] = 0x22;
	//buf[10] = 0x84;
	//buf[11] = 0x22;
	//buf[12] = 0x9b;
	//buf[13] = 0x22;
	//buf[14] = 0xa6;
	//buf[15] = 0xdd;
	//buf[16] = 0x79;
	//buf[17] = 0xdd;
	//buf[18] = 0x64;
	//buf[19] = 0xdd;
	//buf[20] = 0x1c;
	//buf[21] = 0x02;
	//buf[22] = 0x1c;
	//buf[23] = 0x02;
	//buf[24] = 0x85;
	//buf[25] = 0x1f;
	//buf[26] = 0x9f;
	//buf[27] = 0xe0;
	//buf[28] = 0x92;
	//buf[29] = 0x20;
	//buf[30] = 0xdc;
	//buf[31] = 0xe0;
	//buf[32] = 0x4d;
	//buf[33] = 0x1c;
	//buf[34] = 0x1e;
	//buf[35] = 0xde;
	//buf[36] = 0x08;
	//buf[37] = 0x00;

	hid_write(handle, buf, 38);
	//hid_read_timeout(handle, buf, bufLength, 100);
}

JoyShock::JoyShock(struct hid_device_info* dev, int uniqueHandle) {

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

	// initialise continuous calibration windows
	reset_continuous_calibration();

	if (this->handle == nullptr) {
		//printf("Could not open serial %ls: %s\n", this->serial, strerror(errno));
		throw;
	}
}

void JoyShock::reset_continuous_calibration() {
	for (int i = 0; i < num_gyro_average_windows; i++) {
		this->gyro_average_window[i] = {};
	}
};

int JoyShock::get_gyro_average_window_total_samples_for_device() {
	if (this->controller_type == ControllerType::s_ds4) {
		// 250 samples per second
		return 250 * this->gyro_average_window_seconds;
	}
	// 67 samples per second
	return 67 * this->gyro_average_window_seconds;
}

int JoyShock::get_gyro_average_window_single_samples_for_device() {
	return get_gyro_average_window_total_samples_for_device() / (num_gyro_average_windows - 2);
}

void JoyShock::push_sensor_samples(float x, float y, float z, float accelMagnitude) {
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

void JoyShock::get_average_gyro(float& x, float& y, float& z, float& accelMagnitude) {
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

MOTION_STATE JoyShock::get_motion_state()
{
	return motion.GetMotionState();
}

bool JoyShock::hid_exchange(hid_device* handle, unsigned char* buf, int len) {
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

bool JoyShock::send_command(int command, uint8_t* data, int len) {
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

bool JoyShock::send_subcommand(int command, int subcommand, uint8_t* data, int len) {
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

void JoyShock::rumble(int frequency, int intensity) {

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

bool JoyShock::get_switch_controller_info() {
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
	acc_cal_coeff[0] = (float)(1.0 / (float)(16384 - uint16_to_int16(sensor_cal[0][0]))) * 4.0f * 9.8f;
	acc_cal_coeff[1] = (float)(1.0 / (float)(16384 - uint16_to_int16(sensor_cal[0][1]))) * 4.0f * 9.8f;
	acc_cal_coeff[2] = (float)(1.0 / (float)(16384 - uint16_to_int16(sensor_cal[0][2]))) * 4.0f * 9.8f;

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

void JoyShock::enable_IMU(unsigned char* buf, int bufLength) {
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

bool JoyShock::init_usb() {
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

bool JoyShock::init_bt() {
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

void JoyShock::init_ds4_bt() {
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
void JoyShock::init_ds_usb() {
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
void JoyShock::init_ds4_usb() {
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

void JoyShock::deinit_ds4_bt() {
	// TODO. For now, init, which stops rumbling and disables light
	init_ds4_bt();
}

// TODO: implement this
void JoyShock::deinit_ds4_usb() {
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

void JoyShock::deinit_usb() {
	unsigned char buf[0x40];
	memset(buf, 0x00, 0x40);

	//Let the Joy-Con talk BT again    
	buf[0] = 0x80;
	buf[1] = 0x05;
	hid_exchange(this->handle, buf, 0x2);
	//printf("Deinitialized %s\n", this->name.c_str());
}

void JoyShock::set_ds4_rumble_light(unsigned char smallRumble, unsigned char bigRumble,
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

void JoyShock::set_ds4_rumble_light_usb(unsigned char smallRumble, unsigned char bigRumble,
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

void JoyShock::set_ds4_rumble_light_bt(unsigned char smallRumble, unsigned char bigRumble,
	unsigned char colourR,
	unsigned char colourG,
	unsigned char colourB) {
	unsigned char buf[79];
	memset(buf, 0, 79);

	// https://github.com/chrippa/ds4drv/blob/master/ds4drv/device.py
	buf[0] = 0xa2; // 0x80;
	//buf[1] = 0xff;
	// trying to do colour stuff
	// http://eleccelerator.com/wiki/index.php?title=DualShock_4
	// this is only for bt
	buf[1] = 0x11;
	buf[2] = 0xc0;
	buf[3] = 0x20;
	buf[4] = 0xf3;
	buf[5] = 0x04;
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
	//uint32_t crc = crc_32(buf, 75);
	//buf[75] = (crc >> 24) & 0xFF;
	//buf[76] = (crc >> 16) & 0xFF;
	//buf[77] = (crc >> 8) & 0xFF;
	//buf[78] = crc & 0xFF;

	hid_write(handle, buf, 79);
}

//// mfosse credits Hypersect (Ryan Juckett), but I've removed deadzones so the consuming application can deal with them
//// http://blog.hypersect.com/interpreting-analog-sticks/
void JoyShock::CalcAnalogStick2
(
	float& pOutX,       // out: resulting stick X value
	float& pOutY,       // out: resulting stick Y value
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
bool JoyShock::get_spi_data(uint32_t offset, const uint16_t read_len, uint8_t* test_buf) {
	int res;
	uint8_t buf[0x100];
	while (1) {
		memset(buf, 0, sizeof(buf));
		auto hdr = (brcm_hdr*)buf;
		auto pkt = (brcm_cmd_01*)(hdr + 1);
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

int JoyShock::write_spi_data(uint32_t offset, const uint16_t write_len, uint8_t* test_buf) {
	int res;
	uint8_t buf[0x100];
	int error_writing = 0;
	while (1) {
		memset(buf, 0, sizeof(buf));
		auto hdr = (brcm_hdr*)buf;
		auto pkt = (brcm_cmd_01*)(hdr + 1);
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