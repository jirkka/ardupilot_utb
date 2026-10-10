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
#include <AP_Logger/AP_Logger_UTBBench.h>
#if AP_UTB_BENCH_ENABLED
void utb_bench_stack();
void utb_bench_platform_init();
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/AP_HAL_ChibiOS.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
static uintptr_t bench_identity()
{
    return uintptr_t(chThdGetSelfX());
}
void utb_bench_stack()
{
    thread_t *tp = chThdGetSelfX();
    UTBBench::stack_sample(uint32_t(tp) - uint32_t(tp->wabase), stack_free(tp->wabase));
}
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <pthread.h>
static uintptr_t bench_identity()
{
    return uintptr_t(pthread_self());
}
void utb_bench_stack() {}
#else
static uintptr_t bench_identity()
{
    return 0;
}
void utb_bench_stack() {}
#endif
void utb_bench_platform_init()
{
    UTBBench::init(bench_identity);
}
#endif
