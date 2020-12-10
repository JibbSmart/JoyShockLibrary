#pragma once

#include "JoyShock.h"

bool handle_input(JoyShock* jc, uint8_t* packet, int len, bool& hasIMU);