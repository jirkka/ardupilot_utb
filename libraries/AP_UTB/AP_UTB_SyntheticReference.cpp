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
#include "AP_UTB_SyntheticReference.h"
#if AP_UTB_ENABLED && AP_UTB_BENCH_ENABLED
static constexpr uint16_t phase_end[] = {200, 1800, 3400, 4200, 5000, 5800, 6600, 7400, 7800, 8000};
uint8_t AP_UTB_SyntheticReference::phase(uint32_t cycle)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(phase_end); i++) {
        if (cycle < phase_end[i]) {
            return i;
        }
    }
    return 9;
}

Vector3f AP_UTB_SyntheticReference::signal(uint32_t cycle, uint8_t profile)
{
    if (cycle >= CYCLES) {
        return {};
    }
    const uint8_t section = phase(cycle);
    const uint32_t start = section == 0 ? 0 : phase_end[section - 1];
    const float angle = 2.0f * M_PI * float(cycle - start) / 800.0f;
    float amplitude = 0.3f;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    amplitude = profile == 2 ? 8.0f : amplitude;
#else
    if (profile != 1) {
        return {};
    }
#endif
    switch (section) {
    case 1:
        return {amplitude, 0, 0};
    case 2:
        return {-amplitude, 0, 0};
    case 3:
        return {0, amplitude, 0};
    case 4:
        return {0, -amplitude, 0};
    case 5:
        return {amplitude * sinf(angle), 0, 0};
    case 6:
        return {0, 0, amplitude * sinf(angle)};
    case 7:
        return {amplitude * sinf(angle), amplitude * cosf(angle), amplitude * sinf(2 * angle)};
    default:
        return {};
    }
}

void AP_UTB_SyntheticReference::abort()
{
    _status = Status::ABORTED;
}

AP_UTB_SyntheticReference::Sample AP_UTB_SyntheticReference::update(
    uint8_t request, bool eligible, bool armed, bool watchdog, uint8_t mode)
{
    if (request == 0) {
        _cycle = 0;
        _preroll = 0;
        _profile = 0;
        _status = Status::OFF;
        return {};
    }
    bool supported = request == 1;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    supported |= request == 2;
#endif
    if (!supported || armed || watchdog || (_profile != 0 && _profile != request) ||
        ((_status == Status::RUNNING || _status == Status::PREROLL) && (!eligible || mode != _mode))) {
        abort();
    }
    if (_status == Status::OFF) {
        _status = Status::WAITING;
    }
    if (_status == Status::WAITING && eligible) {
        _status = Status::PREROLL;
        _preroll = 400;
        _mode = mode;
        _profile = request;
    }
    Sample result;
    result.cycle = _cycle;
    result.status = _status;
    if (_status == Status::PREROLL) {
        result.valid = true;
        if (--_preroll == 0) {
            _status = Status::RUNNING;
        }
        return result;
    }
    if (_status != Status::RUNNING) {
        return result;
    }
    if (_cycle >= CYCLES) {
        _status = result.status = Status::COMPLETE;
        return result;
    }
    result.phase = phase(_cycle);
    result.rates = signal(_cycle++, request);
    result.valid = true;
    return result;
}
#endif
