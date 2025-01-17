/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * num.c - 16bit floating point handling functions
 */

#ifndef __NUM_H
#define __NUM_H

#include <stdint.h>

#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))

#undef abs
#define abs(a) ((a) > 0 ? (a) : (-1*(a)))

#undef sign
#define sign(a) ((a) > 0 ? (1) : (-1))

#undef isnan
#define isnan(n) ((n != n) ? 1 : 0)


uint16_t single2half(float number);
float half2single(uint16_t number);

uint16_t limitUint16(int32_t value);
float constrain(float value, const float minVal, const float maxVal);
float deadband(float value, const float threshold);

#endif
