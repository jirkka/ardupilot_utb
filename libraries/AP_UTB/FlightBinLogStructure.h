// GNU GPL v3 or later.
/// @file FlightBinLogStructure.h
/// @brief Version 1 packed Flight BIN messages. Float outputs retain native normalized scale.
/// @author Jiri Kovar
/// @version 2026-10-26
#pragma once
#include "AP_UTB_FlightBin.h"
#if AP_UTB_FLIGHT_BIN_AVAILABLE
#include <AP_Logger/LogStructure.h>
namespace UTBFlightLog
{
struct PACKED PID {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq;
    uint8_t axis, mode;
    uint16_t flags;
    float ap_p, ap_i, ap_d, ap_ff, ap_dff, utb_p, utb_i, utb_d, utb_ff, utb_dff;
};
struct PACKED Rate {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq;
    uint8_t axis, mode;
    uint16_t flags;
    uint64_t source_us;
    uint32_t loop, source;
    float dt, gyro, ap_target, utb_target, ap_output, utb_output, ap_ff, utb_ff;
};
struct PACKED Attitude {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint32_t seq;
    uint8_t flags;
    uint64_t source_us;
    float q1, q2, q3, q4, ap1, ap2, ap3, ap4, utb1, utb2, utb3, utb4;
};
struct PACKED Status {
    LOG_PACKET_HEADER;
    uint64_t time_us, start_us;
    uint8_t version, state, reason, rate;
    uint32_t requested, queued, accepted, dropped, invalid, unsupported, rejected;
    uint8_t hwm, depth;
    uint16_t seconds;
};
static_assert(sizeof(PID) == 59 && sizeof(Rate) == 67 && sizeof(Attitude) == 72 && sizeof(Status) == 55,
              "Flight BIN v1 wire layout changed");
// The writer accepts individual packets, never a transaction. Attempt every packet once.
template<typename Writer> bool write_sample(const AP_UTB_FlightBin::Sample &value, Writer &writer)
{
    bool complete = true; // Admission of all seven packets, not durable storage.
    for (uint8_t axis = 0; axis < 3; axis++) {
        const auto &a = value.pid[axis];
        const auto &b = value.rate[axis];
        const PID pid = {LOG_PACKET_HEADER_INIT(LOG_UFPI_MSG), value.time_us, value.seq,
                         axis, value.mode, value.flags[axis], a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8],a[9]
                        };
        const Rate rate = {LOG_PACKET_HEADER_INIT(LOG_UFRS_MSG), value.time_us, value.seq,
                           axis, value.mode, value.flags[axis], value.source_us, value.loop, value.source,
                           value.dt,b[0],b[1],b[2],b[3],b[4],b[5],b[6]
                          };
        complete &= writer.WriteBlock_first_succeed(&pid, sizeof(pid));
        complete &= writer.WriteBlock_first_succeed(&rate, sizeof(rate));
    }
    const auto &q = value.attitude;
    const Attitude att = {LOG_PACKET_HEADER_INIT(LOG_UFAT_MSG),value.time_us,value.seq,3,
                          value.source_us,q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7],q[8],q[9],q[10],q[11]
                         };
    complete &= writer.WriteBlock_first_succeed(&att, sizeof(att));
    return complete;
}
}
// Status bits: 0 ARMED, 1 valid, 2 saturated, 3 I_set, 4 filter_reset, 5 independent I.
#define UTB_FLIGHT_BIN_LOG_STRUCTURES \
    { LOG_UFPI_MSG, sizeof(UTBFlightLog::PID), "UFPI", "QIBBHffffffffff", "TimeUS,Seq,Ax,Md,Flg,AP,AI,AD,AFF,ADF,UP,UI,UD,UFF,UDF", "s--------------", "F--------------" }, \
    { LOG_UFRS_MSG, sizeof(UTBFlightLog::Rate), "UFRS", "QIBBHQIIffffffff", "TimeUS,Seq,Ax,Md,Flg,SrcUS,Lp,Src,Dt,Gy,AT,UT,AO,UO,AF,UF", "s----s--sEEE----", "F----F----------" }, \
    { LOG_UFAT_MSG, sizeof(UTBFlightLog::Attitude), "UFAT", "QIBQffffffffffff", "TimeUS,Seq,Flg,SrcUS,Q1,Q2,Q3,Q4,A1,A2,A3,A4,U1,U2,U3,U4", "s--s------------", "F--F------------" }, \
    { LOG_UFLS_MSG, sizeof(UTBFlightLog::Status), "UFLS", "QQBBBBIIIIIIIBBH", "TimeUS,Start,V,St,Rsn,Hz,Req,Que,Acc,Drp,Inv,Uns,Rej,HWM,Dep,Sec", "ss---z---------s", "FF--------------" },
#endif
