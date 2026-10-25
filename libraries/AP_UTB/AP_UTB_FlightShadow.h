// GNU GPL v3 or later; original ArduPilot algorithms retain their copyright.
/// @file AP_UTB_FlightShadow.h
/// @brief Bounded value recorder for the existing passive chain; no motor or logger interface.
/// @author Jiri Kovar
/// @version 2026-10-25
#pragma once
#include "AP_UTB_config.h"
#if AP_UTB_FLIGHT_SHADOW_AVAILABLE
#if !AP_UTB_CHAIN_SHADOW_ENABLED
#error Flight recording requires the existing Chain shadow
#endif
#include <AP_Param/AP_Param.h>
class UTB_ChainShadowEngine;
class ExpandingString;

class AP_UTB_FlightShadow
{
public:
    static constexpr uint16_t CAPACITY = 128;
    static constexpr uint8_t PAGE_ROWS = 16;
    // Rate and the consumed Attitude source have different timestamps by design.
    struct Record {
        uint32_t id, loop, source_loop, time_us, source_us, epoch;
        uint32_t resets_at_end;
        float dt;
        float attitude[4], native_attitude_target[4], own_attitude_target[4];
        float command[3], gyro[3], native_target[3], own_target[3];
        float native_pid[3][5], own_pid[3][5]; // P, I, D, FF, D_FF; original AP units.
        float native_output[3], own_output[3], native_ff_output[3], own_ff_output[3];
        uint8_t mode, spool, flags, limits, reset_flags, reserved[3];
    };
    struct Statistics {
        uint32_t candidates = 0, selected = 0, decimated = 0, overwritten = 0;
        uint32_t epochs = 0, arm_changes = 0, mode_exits = 0, unsupported = 0;
        uint32_t record_max_us = 0;
        uint64_t record_sum_us = 0;
    };
    AP_UTB_FlightShadow();
    void init();
    bool requested() const
    {
        return _run.get() != 0;
    }
    int8_t requested_run() const
    {
        return _run.get();
    }
    bool start();
    void stop(uint8_t reason);
    void context(bool armed, uint8_t spool);
    void record(const UTB_ChainShadowEngine &engine, uint32_t loop, bool force);
    void mode_exit()
    {
        _stats.mode_exits++;
    }
    void unsupported()
    {
        _stats.unsupported++;
    }
    uint32_t limit() const
    {
        return _limit;
    }
    const Statistics &statistics() const
    {
        return _stats;
    }
    const Record &oldest(uint16_t index) const;
    uint16_t size() const
    {
        return _used;
    }
    void dump_header(ExpandingString &text) const;
    void dump_page(ExpandingString &text, uint8_t page) const;
    static const AP_Param::GroupInfo var_info[];
private:
    friend class UTBFlightTestAccess;
    AP_Int8 _run;
    AP_Int32 _count;
    AP_Int16 _decim;
    Record _records[CAPACITY] {};
    Statistics _stats {};
    uint64_t _start_us = 0, _stop_us = 0;
    uint32_t _limit = 0;
    uint16_t _head = 0, _used = 0, _stride = 0;
    uint8_t _reason = 0, _spool = 0;
    bool _started = false, _closed = false, _armed = false;
};
#endif // AP_UTB_FLIGHT_SHADOW_AVAILABLE
