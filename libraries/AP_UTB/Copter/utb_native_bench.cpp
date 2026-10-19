// GNU GPL v3 or later. Diagnostic only; no pilot mode dispatch.
/// @file utb_native_bench.cpp
/// @author Jiri Kovar
/// @version 2026-10-19
#include "../../../ArduCopter/Copter.h"
#if AP_UTB_NATIVE_BENCH_AVAILABLE && FRAME_CONFIG == MULTICOPTER_FRAME
void Copter::utb_native_bench_update()
{
    utb_native_bench.update(motors->armed() || using_rate_thread);
}
#endif
