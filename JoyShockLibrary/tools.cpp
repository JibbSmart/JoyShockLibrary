#pragma once

#include "tools.h"
#include <string>
#include <iostream>

//#include <curl/curl.h>

int16_t unsignedToSigned16(uint16_t n) {
	uint16_t A = n;
	uint16_t B = 0xFFFF - A;
	if (A < B) {
		return (int16_t)A;
	} else {
		return (int16_t)(-1 * B);
	}
}

int16_t uint16_to_int16(uint16_t a) {
	int16_t b;
	char* aPointer = (char*)&a, *bPointer = (char*)&b;
	memcpy(bPointer, aPointer, sizeof(a));
	return b;
}

uint16_t combine_uint8_t(uint8_t a, uint8_t b) {
	uint16_t c = ((uint16_t)a << 8) | b;
	return c;
}

int16_t combine_gyro_data(uint8_t a, uint8_t b) {
	uint16_t c = combine_uint8_t(a, b);
	int16_t d = uint16_to_int16(c);
	return d;
}

float clamp(float a, float min, float max) {
	if (a < min) {
		return min;
	} else if (a > max) {
		return max;
	} else {
		return a;
	}
}

unsigned createMask(unsigned a, unsigned b) {
	unsigned r = 0;
	for (unsigned i = a; i <= b; i++)
		r |= 1 << i;

	return r;
}

void hex_dump(unsigned char *buf, int len) {
	for (int i = 0; i < len; i++) {
		printf("%02x ", buf[i]);
	}
	printf("\n");
}

void hex_dump2(unsigned char *buf, int len) {
	for (int i = 0; i < len; i++) {
		printf("%02x ", buf[i]);
	}
}

void hex_dump_0(unsigned char *buf, int len) {
	for (int i = 0; i < len; i++) {
		if (buf[i] != 0) {
			printf("%02x ", buf[i]);
		}
	}
}

void int_dump(unsigned char *buf, int len) {
	for (int i = 0; i < len; i++) {
		printf("%i ", buf[i]);
	}
	printf("\n");
}
