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
#include <AP_Logger/AP_Logger_UTBBench.h>
#if AP_UTB_BENCH_ENABLED && HAL_LOGGING_ENABLED
// @LoggerMessage: UBMT
// @Description: Benchmark cumulative wall time; metric IDs in bench guide
// @Field: TimeUS: Summary monotonic time
// @Field: Epoch: UTB epoch at export
// @Field: Var: Variant 1 A, 2 B, 3 C, 4 D, 5 custom
// @Field: Met: Metric enum
// @Field: N: Sample count since boot
// @Field: Sum: Cumulative microseconds
// @Field: Min: Minimum microseconds
// @Field: Max: Maximum microseconds
// @Field: Last: Most recent microseconds
struct PACKED log_UBMT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t epoch;
    uint8_t variant, metric;
    uint32_t count;
    uint64_t sum;
    uint32_t minimum, maximum, last;
};
// @LoggerMessage: UBHI
// @Description: Cumulative logarithmic histogram: bin 0 0..1us, bin i 2^i..2^(i+1)-1us, final bin >=2048us
// @Field: TimeUS: Summary monotonic time
// @Field: Epoch: UTB epoch
// @Field: Met: Metric enum
// @Field: H0: Bin 0 count
// @Field: H1: Bin 1 count
// @Field: H2: Bin 2 count
// @Field: H3: Bin 3 count
// @Field: H4: Bin 4 count
// @Field: H5: Bin 5 count
// @Field: H6: Bin 6 count
// @Field: H7: Bin 7 count
// @Field: H8: Bin 8 count
// @Field: H9: Bin 9 count
// @Field: H10: Bin 10 count
// @Field: H11: Bin 11 count
struct PACKED log_UBHI {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t epoch;
    uint8_t metric;
    uint32_t h[UTBBench::BINS];
};
// @LoggerMessage: UBST
// @Description: Independent benchmark lifecycle, collisions and export diagnostics
// @Field: TimeUS: Summary time
// @Field: Epoch: UTB epoch
// @Field: Var: Benchmark variant
// @Field: Key: State key in hardware bench guide
// @Field: Val: Unsigned state/counter; UINT32_MAX means unavailable stack reserve
struct PACKED log_UBST {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t epoch;
    uint8_t variant, key;
    uint32_t value;
};
// @LoggerMessage: UBEP
// @Description: Epoch companion for diagnostic set; completeness requires all seven records
// @Field: TimeUS: Original shadow time
// @Field: Seq: Shadow sequence
// @Field: Epoch: Shadow epoch
struct PACKED log_UBEP {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq, epoch;
};
#define UTB_BENCH_LOG_STRUCTURES \
    { LOG_UBMT_MSG, sizeof(log_UBMT), "UBMT", "QIBBIQIII", "TimeUS,Epoch,Var,Met,N,Sum,Min,Max,Last", "s----ssss", "F----FFFF" }, \
    { LOG_UBHI_MSG, sizeof(log_UBHI), "UBHI", "QIBIIIIIIIIIIII", "TimeUS,Epoch,Met,H0,H1,H2,H3,H4,H5,H6,H7,H8,H9,H10,H11", "s--------------", "F--------------" }, \
    { LOG_UBST_MSG, sizeof(log_UBST), "UBST", "QIBBI", "TimeUS,Epoch,Var,Key,Val", "s----", "F----" }, \
    { LOG_UBEP_MSG, sizeof(log_UBEP), "UBEP", "QII", "TimeUS,Seq,Epoch", "s--", "F--" },
#endif
