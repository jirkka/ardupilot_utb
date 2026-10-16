/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.
 */
#pragma once

#ifndef AP_UTB_ENABLED
#define AP_UTB_ENABLED 0
#endif

#include <AP_HAL/AP_HAL_Boards.h>
#ifndef AP_UTB_RATE_SITL_ENABLED
#define AP_UTB_RATE_SITL_ENABLED 0
#endif
#define AP_UTB_RATE_SITL_AVAILABLE (AP_UTB_ENABLED && AP_UTB_RATE_SITL_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
