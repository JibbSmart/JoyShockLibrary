#pragma once

#include "JoyShockLibrary.h"
#include "JoyShock.cpp"

bool handle_input(JoyShock *jc, uint8_t *packet, int len) {
	if (packet[0] == 0) return false; // ignore non-responses
									  // remember last input
	jc->last_simple_state = jc->simple_state;
	jc->simple_state.buttons = 0;
	jc->last_imu_state = jc->imu_state;
	// delta time
	auto time_now = std::chrono::steady_clock::now();
	jc->delta_time = ((float)std::chrono::duration_cast<std::chrono::microseconds>(time_now - jc->last_polled).count()) / 1000000.0;
	jc->last_polled = time_now;
	// ds4
	if (jc->is_ds4) {
		if (packet[0] == 0x01) {
			// Gyroscope:
			// Gyroscope data is relative (degrees/s)
			int16_t gyroSampleY = uint16_to_int16(packet[13] | (packet[14] << 8) & 0xFF00);
			int16_t gyroSampleZ = uint16_to_int16(packet[15] | (packet[16] << 8) & 0xFF00);
			int16_t gyroSampleX = uint16_to_int16(packet[17] | (packet[18] << 8) & 0xFF00);
			int16_t accelSampleX = uint16_to_int16(packet[19] | (packet[20] << 8) & 0xFF00);
			int16_t accelSampleY = uint16_to_int16(packet[21] | (packet[22] << 8) & 0xFF00);
			int16_t accelSampleZ = uint16_to_int16(packet[23] | (packet[24] << 8) & 0xFF00);
			// convert to real units
			jc->imu_state.gyroX = (float)(gyroSampleY) * -(2000.0 / 32767.0);
			jc->imu_state.gyroY = (float)(gyroSampleZ) * (2000.0 / 32767.0);
			jc->imu_state.gyroZ = (float)(gyroSampleX) * (2000.0 / 32767.0);

			jc->imu_state.accelX = (float)(accelSampleX) / 8192.0;
			jc->imu_state.accelZ = (float)(accelSampleY) / -8192.0;
			jc->imu_state.accelY = (float)(accelSampleZ) / 8192.0;

			//printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n",
			//	jc->gyro.yaw, jc->gyro.pitch, jc->gyro.roll, jc->accel.x, jc->accel.y, jc->accel.z, universal_counter++);

			// DS4 dpad is a hat...  0x08 is released, 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW
			// http://eleccelerator.com/wiki/index.php?title=DualShock_4
			uint8_t hat = packet[5] & 0x0f;

			if ((hat > 2) & (hat < 6)) jc->simple_state.buttons |= JSMASK_DOWN; // down = SE | S | SW
			if ((hat == 7) | (hat < 2)) jc->simple_state.buttons |= JSMASK_UP; // up = N | NE | NW
			if ((hat > 0) & (hat < 4)) jc->simple_state.buttons |= JSMASK_RIGHT; // right = NE | E | SE
			if ((hat > 4) & (hat < 8)) jc->simple_state.buttons |= JSMASK_LEFT; // left = SW | W | NW

			jc->simple_state.buttons |= ((int)(packet[5] >> 4) << JSOFFSET_W) & JSMASK_W;
			jc->simple_state.buttons |= ((int)(packet[5] >> 7) << JSOFFSET_N) & JSMASK_N;
			jc->simple_state.buttons |= ((int)(packet[5] >> 5) << JSOFFSET_S) & JSMASK_S;
			jc->simple_state.buttons |= ((int)(packet[5] >> 6) << JSOFFSET_E) & JSMASK_E;
			jc->simple_state.buttons |= ((int)(packet[6] >> 6) << JSOFFSET_LCLICK) & JSMASK_LCLICK;
			jc->simple_state.buttons |= ((int)(packet[6] >> 7) << JSOFFSET_RCLICK) & JSMASK_RCLICK;
			jc->simple_state.buttons |= ((int)(packet[6] >> 5) << JSOFFSET_OPTIONS) & JSMASK_OPTIONS;
			jc->simple_state.buttons |= ((int)(packet[6] >> 4) << JSOFFSET_SHARE) & JSMASK_SHARE;
			jc->simple_state.buttons |= ((int)(packet[6] >> 1) << JSOFFSET_R) & JSMASK_R;
			jc->simple_state.buttons |= ((int)(packet[6]) << JSOFFSET_L) & JSMASK_L;
			jc->simple_state.buttons |= ((int)(packet[7]) << JSOFFSET_PS) & JSMASK_PS;
			jc->simple_state.buttons |= ((int)(packet[7] >> 1) << JSOFFSET_TOUCHPAD_CLICK) & JSMASK_TOUCHPAD_CLICK;
			//jc->btns.zr = (packet[6] >> 3) & 1;
			//jc->btns.zl = (packet[6] >> 2) & 1;
			jc->simple_state.rTrigger = packet[9] / 255.0f;
			jc->simple_state.lTrigger = packet[8] / 255.0f;

			if (jc->simple_state.rTrigger > 0.0) jc->simple_state.buttons |= JSMASK_ZR;
			if (jc->simple_state.lTrigger > 0.0) jc->simple_state.buttons |= JSMASK_ZL;

			uint16_t stick_x = packet[1];
			uint16_t stick_y = packet[2];
			stick_y = 255 - stick_y;

			uint16_t stick2_x = packet[3];
			uint16_t stick2_y = packet[4];
			stick2_y = 255 - stick2_y;

			jc->simple_state.stickLX = (std::fmin)(1.0, (stick_x - 127.0) / 127.0);
			jc->simple_state.stickLY = (std::fmin)(1.0, (stick_y - 127.0) / 127.0);
			jc->simple_state.stickRX = (std::fmin)(1.0, (stick2_x - 127.0) / 127.0);
			jc->simple_state.stickRY = (std::fmin)(1.0, (stick2_y - 127.0) / 127.0);

			if (jc->use_continuous_calibration) {
				jc->push_sensor_samples(jc->imu_state.gyroX, jc->imu_state.gyroY, jc->imu_state.gyroZ);
				jc->get_average_gyro(jc->offset_x, jc->offset_y, jc->offset_z);
			}

			jc->imu_state.gyroX -= jc->offset_x;
			jc->imu_state.gyroY -= jc->offset_y;
			jc->imu_state.gyroZ -= jc->offset_z;
		}

		//printf("Buttons: %d LX: %.5f LY: %.5f RX: %.5f RY: %.5f GX: %.4f GY: %.4f GZ: %.4f\n", \
				//	jc->simple_state.buttons, (jc->simple_state.stickLX + 1), (jc->simple_state.stickLY + 1), (jc->simple_state.stickRX + 1), (jc->simple_state.stickRY + 1), jc->imu_state.gyroX, jc->imu_state.gyroY, jc->imu_state.gyroZ);


		return true;
	}

	// most of this JoyCon and Pro Controller stuff is adapted from MFosse's Joycon driver.

	// bluetooth button pressed packet:
	if (packet[0] == 0x3F) {

		//uint16_t old_buttons = jc->buttons;
		//int8_t old_dstick = jc->dstick;

		jc->dstick = packet[3];
		// todo: get button states here aswell:
	}

	int buttons_pressed = 0;

	// input update packet:
	// 0x21 is just buttons, 0x30 includes gyro, 0x31 includes NFC (large packet size)
	if (packet[0] == 0x21 || packet[0] == 0x30 || packet[0] == 0x31) {

		// offset for usb or bluetooth data:
		/*int offset = settings.usingBluetooth ? 0 : 10;*/
		int offset = jc->bluetooth ? 0 : 10;

		uint8_t *btn_data = packet + offset + 3;

		// get button states:
		{
			uint16_t states = 0;
			uint16_t states2 = 0;

			// Left JoyCon:
			if (jc->left_right == 1) {
				states = (btn_data[1] << 8) | (btn_data[2] & 0xFF);
				// Right JoyCon:
			}
			else if (jc->left_right == 2) {
				states = (btn_data[1] << 8) | (btn_data[0] & 0xFF);
				// Pro Controller:
			}
			else if (jc->left_right == 3) {
				states = (btn_data[1] << 8) | (btn_data[2] & 0xFF);
				states2 = (btn_data[1] << 8) | (btn_data[0] & 0xFF);
			}

			buttons_pressed = states;
			// Pro Controller:
			if (jc->left_right == 3) {
				buttons_pressed |= states2 << 16;

				// fix some non-sense the Pro Controller does
				// clear nth bit
				//num &= ~(1UL << n);
				buttons_pressed &= ~(1L << 9);
				buttons_pressed &= ~(1L << 12);
				buttons_pressed &= ~(1L << 14);

				buttons_pressed &= ~(1UL << (8 + 16));
				buttons_pressed &= ~(1UL << (11 + 16));
				buttons_pressed &= ~(1UL << (13 + 16));
			}
			else if (jc->left_right == 2) {
				buttons_pressed = buttons_pressed << 16;
			}
		}

		// get stick data:
		uint8_t *stick_data = packet + offset;
		if (jc->left_right == 1) {
			stick_data += 6;
		}
		else if (jc->left_right == 2) {
			stick_data += 9;
		}

		uint16_t stick_x = stick_data[0] | ((stick_data[1] & 0xF) << 8);
		uint16_t stick_y = (stick_data[1] >> 4) | (stick_data[2] << 4);

		// use calibration data:
		if (jc->left_right == 1) {
			jc->CalcAnalogStick2(jc->simple_state.stickLX, jc->simple_state.stickLY,
				stick_x,
				stick_y,
				jc->stick_cal_x_l,
				jc->stick_cal_y_l);
		}
		else if (jc->left_right == 2) {
			jc->CalcAnalogStick2(jc->simple_state.stickRX, jc->simple_state.stickRY,
				stick_x,
				stick_y,
				jc->stick_cal_x_r,
				jc->stick_cal_y_r);
		}
		else if (jc->left_right == 3) {
			// pro controller
			stick_data += 6;
			uint16_t stick_x = stick_data[0] | ((stick_data[1] & 0xF) << 8);
			uint16_t stick_y = (stick_data[1] >> 4) | (stick_data[2] << 4);
			jc->CalcAnalogStick2(jc->simple_state.stickLX, jc->simple_state.stickLY,
				stick_x,
				stick_y,
				jc->stick_cal_x_l,
				jc->stick_cal_y_l);
			stick_data += 3;
			uint16_t stick_x2 = stick_data[0] | ((stick_data[1] & 0xF) << 8);
			uint16_t stick_y2 = (stick_data[1] >> 4) | (stick_data[2] << 4);
			jc->CalcAnalogStick2(jc->simple_state.stickRX, jc->simple_state.stickRY,
				stick_x2,
				stick_y2,
				jc->stick_cal_x_r,
				jc->stick_cal_y_r);
		}

		jc->battery = (stick_data[1] & 0xF0) >> 4;
		//printf("JoyCon battery: %d\n", jc->battery);

		// Accelerometer:
		// Accelerometer data is absolute
		{
			// get accelerometer X:
			int16_t accelSampleX = uint16_to_int16(packet[13] | (packet[14] << 8) & 0xFF00);
			int16_t accelSampleY = uint16_to_int16(packet[15] | (packet[16] << 8) & 0xFF00);
			int16_t accelSampleZ = uint16_to_int16(packet[17] | (packet[18] << 8) & 0xFF00);
			int16_t gyroSampleX = uint16_to_int16(packet[19] | (packet[20] << 8) & 0xFF00);
			int16_t gyroSampleY = uint16_to_int16(packet[21] | (packet[22] << 8) & 0xFF00);
			int16_t gyroSampleZ = uint16_to_int16(packet[23] | (packet[24] << 8) & 0xFF00);
			//jc->push_sensor_samples(accelSampleX, accelSampleY, accelSampleZ, gyroSampleX, gyroSampleY, gyroSampleZ);
			float accelX = accelSampleX;
			float accelY = accelSampleY;
			float accelZ = accelSampleZ;
			float roll = gyroSampleX - jc->sensor_cal[1][0];
			float pitch = gyroSampleY - jc->sensor_cal[1][1];
			float yaw = gyroSampleZ - jc->sensor_cal[1][2];
			// each packet actually has 3 samples worth of data, so collect sample 2
			accelSampleX = uint16_to_int16(packet[25] | (packet[26] << 8) & 0xFF00);
			accelSampleY = uint16_to_int16(packet[27] | (packet[28] << 8) & 0xFF00);
			accelSampleZ = uint16_to_int16(packet[29] | (packet[30] << 8) & 0xFF00);
			gyroSampleX = uint16_to_int16(packet[31] | (packet[32] << 8) & 0xFF00);
			gyroSampleY = uint16_to_int16(packet[33] | (packet[34] << 8) & 0xFF00);
			gyroSampleZ = uint16_to_int16(packet[35] | (packet[36] << 8) & 0xFF00);
			//jc->push_sensor_samples(accelSampleX, accelSampleY, accelSampleZ, gyroSampleX, gyroSampleY, gyroSampleZ);
			accelX += accelSampleX;
			accelY += accelSampleY;
			accelZ += accelSampleZ;
			roll += gyroSampleX - jc->sensor_cal[1][0];
			pitch += gyroSampleY - jc->sensor_cal[1][1];
			yaw += gyroSampleZ - jc->sensor_cal[1][2];
			// ... and sample 3
			accelSampleX = uint16_to_int16(packet[37] | (packet[38] << 8) & 0xFF00);
			accelSampleY = uint16_to_int16(packet[39] | (packet[40] << 8) & 0xFF00);
			accelSampleZ = uint16_to_int16(packet[41] | (packet[42] << 8) & 0xFF00);
			gyroSampleX = uint16_to_int16(packet[43] | (packet[44] << 8) & 0xFF00);
			gyroSampleY = uint16_to_int16(packet[45] | (packet[46] << 8) & 0xFF00);
			gyroSampleZ = uint16_to_int16(packet[47] | (packet[48] << 8) & 0xFF00);
			//jc->push_sensor_samples(accelSampleX, accelSampleY, accelSampleZ, gyroSampleX, gyroSampleY, gyroSampleZ);
			accelX += accelSampleX;
			accelY += accelSampleY;
			accelZ += accelSampleZ;
			roll += gyroSampleX - jc->sensor_cal[1][0];
			pitch += gyroSampleY - jc->sensor_cal[1][1];
			yaw += gyroSampleZ - jc->sensor_cal[1][2];
			// average the 3 samples
			accelX /= 3;
			accelY /= 3;
			accelZ /= 3;
			roll /= 3;
			pitch /= 3;
			yaw /= 3;
			jc->imu_state.accelX = (float)(accelX) / 4096.0;
			jc->imu_state.accelY = (float)(accelY) / 4096.0;
			jc->imu_state.accelZ = (float)(accelZ) / 4096.0;
			jc->imu_state.gyroZ = (float)(roll) * (2294.0 / 32767.0);
			jc->imu_state.gyroX = (float)(pitch) * (2294.0 / 32767.0);
			jc->imu_state.gyroY = (float)(yaw) * (2294.0 / 32767.0);

			if (jc->use_continuous_calibration) {
				jc->push_sensor_samples(jc->imu_state.gyroX, jc->imu_state.gyroY, jc->imu_state.gyroZ);
				jc->get_average_gyro(jc->offset_x, jc->offset_y, jc->offset_z);
			}

			jc->imu_state.gyroX -= jc->offset_x;
			jc->imu_state.gyroY -= jc->offset_y;
			jc->imu_state.gyroZ -= jc->offset_z;
		}

	}

	// handle buttons
	{
		// left:
		if (jc->left_right == 1) {
			jc->simple_state.buttons |= ((buttons_pressed >> 1) << JSOFFSET_UP) & JSMASK_UP;
			jc->simple_state.buttons |= ((buttons_pressed) << JSOFFSET_DOWN) & JSMASK_DOWN;
			jc->simple_state.buttons |= ((buttons_pressed >> 3) << JSOFFSET_LEFT) & JSMASK_LEFT;
			jc->simple_state.buttons |= ((buttons_pressed >> 2) << JSOFFSET_RIGHT) & JSMASK_RIGHT;
			jc->simple_state.buttons |= ((buttons_pressed >> 11) << JSOFFSET_LCLICK) & JSMASK_LCLICK;
			jc->simple_state.buttons |= ((buttons_pressed >> 8) << JSOFFSET_MINUS) & JSMASK_MINUS;
			jc->simple_state.buttons |= ((buttons_pressed >> 6) << JSOFFSET_L) & JSMASK_L;
			jc->simple_state.buttons |= ((buttons_pressed >> 13) << JSOFFSET_CAPTURE) & JSMASK_CAPTURE;
			jc->simple_state.lTrigger = (buttons_pressed >> 7) & 1;
			jc->simple_state.buttons |= ((int)(jc->simple_state.lTrigger) << JSOFFSET_ZL) & JSMASK_ZL;

			// just need to negate gyroZ
			jc->imu_state.gyroZ = -jc->imu_state.gyroZ;
		}

		// right:
		if (jc->left_right == 2) {
			jc->simple_state.buttons |= ((buttons_pressed >> 16) << JSOFFSET_W) & JSMASK_W;
			jc->simple_state.buttons |= ((buttons_pressed >> 17) << JSOFFSET_N) & JSMASK_N;
			jc->simple_state.buttons |= ((buttons_pressed >> 18) << JSOFFSET_S) & JSMASK_S;
			jc->simple_state.buttons |= ((buttons_pressed >> 19) << JSOFFSET_E) & JSMASK_E;
			jc->simple_state.buttons |= ((buttons_pressed >> 26) << JSOFFSET_RCLICK) & JSMASK_RCLICK;
			jc->simple_state.buttons |= ((buttons_pressed >> 25) << JSOFFSET_PLUS) & JSMASK_PLUS;
			jc->simple_state.buttons |= ((buttons_pressed >> 22) << JSOFFSET_R) & JSMASK_R;
			jc->simple_state.buttons |= ((buttons_pressed >> 28) << JSOFFSET_HOME) & JSMASK_HOME;
			jc->simple_state.rTrigger = (buttons_pressed >> 23) & 1;
			jc->simple_state.buttons |= ((int)(jc->simple_state.rTrigger) << JSOFFSET_ZR) & JSMASK_ZR;

			// for some reason we need to negate x and y, and z on the right joycon
			jc->imu_state.gyroX = -jc->imu_state.gyroX;
			jc->imu_state.gyroY = -jc->imu_state.gyroY;
			jc->imu_state.gyroZ = -jc->imu_state.gyroZ;

		}

		// pro controller:
		if (jc->left_right == 3) {
			jc->simple_state.buttons |= ((buttons_pressed >> 1) << JSOFFSET_UP) & JSMASK_UP;
			jc->simple_state.buttons |= ((buttons_pressed) << JSOFFSET_DOWN) & JSMASK_DOWN;
			jc->simple_state.buttons |= ((buttons_pressed >> 3) << JSOFFSET_LEFT) & JSMASK_LEFT;
			jc->simple_state.buttons |= ((buttons_pressed >> 2) << JSOFFSET_RIGHT) & JSMASK_RIGHT;
			jc->simple_state.buttons |= ((buttons_pressed >> 16) << JSOFFSET_W) & JSMASK_W;
			jc->simple_state.buttons |= ((buttons_pressed >> 17) << JSOFFSET_N) & JSMASK_N;
			jc->simple_state.buttons |= ((buttons_pressed >> 18) << JSOFFSET_S) & JSMASK_S;
			jc->simple_state.buttons |= ((buttons_pressed >> 19) << JSOFFSET_E) & JSMASK_E;
			jc->simple_state.buttons |= ((buttons_pressed >> 11) << JSOFFSET_LCLICK) & JSMASK_LCLICK;
			jc->simple_state.buttons |= ((buttons_pressed >> 26) << JSOFFSET_RCLICK) & JSMASK_RCLICK;
			jc->simple_state.buttons |= ((buttons_pressed >> 25) << JSOFFSET_PLUS) & JSMASK_PLUS;
			jc->simple_state.buttons |= ((buttons_pressed >> 8) << JSOFFSET_MINUS) & JSMASK_MINUS;
			jc->simple_state.buttons |= ((buttons_pressed >> 22) << JSOFFSET_R) & JSMASK_R;
			jc->simple_state.buttons |= ((buttons_pressed >> 6) << JSOFFSET_L) & JSMASK_L;
			jc->simple_state.buttons |= ((buttons_pressed >> 28) << JSOFFSET_HOME) & JSMASK_HOME;
			jc->simple_state.buttons |= ((buttons_pressed >> 13) << JSOFFSET_CAPTURE) & JSMASK_CAPTURE;
			jc->simple_state.rTrigger = (buttons_pressed >> 23) & 1;
			jc->simple_state.lTrigger = (buttons_pressed >> 7) & 1;
			jc->simple_state.buttons |= ((int)(jc->simple_state.lTrigger) << JSOFFSET_ZL) & JSMASK_ZL;
			jc->simple_state.buttons |= ((int)(jc->simple_state.rTrigger) << JSOFFSET_ZR) & JSMASK_ZR;

			// just need to negate gyroZ
			jc->imu_state.gyroZ = -jc->imu_state.gyroZ;
		}

	}

	return true;
}
