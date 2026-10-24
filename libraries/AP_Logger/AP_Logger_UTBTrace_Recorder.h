//----------------------------------------------------------------------------------------------------------------------
// NOTICE: UTB diagnostic extension, GPL-3.0-or-later; no regulator mathematics.
//----------------------------------------------------------------------------------------------------------------------
/// @file AP_Logger_UTBTrace_Recorder.h
/// @brief Fixed main-thread timing ring, frozen before read-only export.
/// @author Jiri Kovar
/// @version 2026-10-21
/// @date 2026-10-21
#pragma once

// Include Files
#include <stdint.h>
#include <atomic>

// Data type, constant, macro definition
namespace UTBLoggerTrace
{
enum class Event : uint8_t { START_LOG, FIND_PAGE, PAGE_READ, WAIT_READY,
                             CHIP_WAIT, CHIP_HOLD, SPI_WAIT, SPI_HOLD, PARAM_SET,
                             WRITE_WAIT, WRITE_HOLD, BUDGET_OVERRUN, COUNT
                           };
struct Entry {
    uint32_t ticket;
    uint32_t cycle;
    uint32_t start;
    uint32_t duration;
    int16_t task;
    Event event;
    bool complete;
};

struct Summary {
    uint32_t count;
    uint32_t maximum;
    uint32_t start;
    int16_t task;
};

// Public functions: only the main thread may mutate a Recorder.
class Recorder
{
public:
    static constexpr uint16_t CAPACITY = 128;
    static constexpr uint32_t INVALID = 0;
    /// @brief Prime the clock; freeze the preceding cycle above the native PM threshold.
    /// @param[in] now Monotonic microseconds, modulo uint32.
    /// @param[in] nominal Expected period in microseconds.
    void loop(uint32_t now, uint32_t nominal)
    {
        // Elapsed wall time of the previous complete cycle.
        uint32_t elapsed;
        if (_frozen_main) {
            return;
        }
        elapsed = now - _previous;
        if (_primed && elapsed > nominal + nominal / 5U) {
            _period = elapsed;
            _trigger = now;
            _frozen_main = true;
            _published.store(true, std::memory_order_release);
            return;
        }
        _previous = now;
        _primed = true;
        _cycle++;
    }
    /// @brief Start a bounded span; zero means inactive or frozen.
    uint32_t begin(Event event, uint32_t now, int16_t task)
    {
        // Monotonic span identity; matching it prevents overwritten completion updates.
        uint32_t ticket;
        if (!_primed || _frozen_main) {
            return INVALID;
        }
        if (_tickets == UINT32_MAX) {
            // Ticket exhaustion is explicit and cannot alias an unfinished slot.
            _exhausted = true;
            return INVALID;
        }
        ticket = ++_tickets;
        _entries[(ticket - 1U) % CAPACITY] = {ticket, _cycle, now, 0, task, event, false};
        return ticket;
    }
    /// @brief Complete only the original slot; nested spans may have overwritten it.
    void end(uint32_t ticket, uint32_t now, uint32_t start, Event event)
    {
        if (ticket == INVALID || _frozen_main) {
            return;
        }
        // Slot metadata is trusted only when the original ticket is still retained.
        Entry &entry = _entries[(ticket - 1U) % CAPACITY];
        // Aggregate original scope metadata even if its detail slot was overwritten.
        Summary &summary = _summary[uint8_t(event)];
        summary.count++;
        if (now - start > summary.maximum) {
            summary.maximum = now - start;
            summary.start = start;
            summary.task = entry.ticket == ticket ? entry.task : INT16_MIN;
        }
        // Slot belongs exclusively to the main producer until freeze publication.
        if (entry.ticket != ticket) {
            _lost_completions++;
            return;
        }
        entry.duration = now - entry.start;
        entry.complete = true;
    }
    /// @brief Capture native scheduler overrun timing without another clock read.
    void task_overrun(int16_t task, uint32_t start, uint32_t duration)
    {
        // A completed task is inserted after its children; ticket order is insertion order.
        uint32_t ticket;
        ticket = begin(Event::BUDGET_OVERRUN, start, task);
        end(ticket, start + duration, start, Event::BUDGET_OVERRUN);
    }
    /// @brief Publish an armed-operation abort, distinct from a long-cycle trigger.
    void abort(uint32_t now)
    {
        if (!_frozen_main) {
            _trigger = now;
            _aborted = true;
            _frozen_main = true;
            _published.store(true, std::memory_order_release);
        }
    }
    bool aborted() const
    {
        return _aborted;
    }
    /// @brief Acquire immutable capture; no reset or writes are allowed after publication.
    bool frozen() const
    {
        return _published.load(std::memory_order_acquire);
    }
    uint32_t count() const
    {
        return _tickets < CAPACITY ? _tickets : CAPACITY;
    }
    bool exhausted() const
    {
        return _exhausted;
    }
    uint32_t tickets() const
    {
        return _tickets;
    }
    uint32_t period() const
    {
        return _period;
    }
    uint32_t trigger() const
    {
        return _trigger;
    }
    const Summary &summary(Event event) const
    {
        return _summary[uint8_t(event)];
    }
    uint32_t lost_completions() const
    {
        return _lost_completions;
    }
    /// @brief Read chronological retained entries only after frozen() succeeds.
    const Entry &entry(uint32_t index) const
    {
        return _entries[(_tickets - count() + index) % CAPACITY];
    }
private:
    // All mutable fields below are main-thread owned; readers require published freeze.
    Entry _entries[CAPACITY] {};
    Summary _summary[uint8_t(Event::COUNT)] {};
    uint32_t _tickets {};
    uint32_t _cycle {};
    uint32_t _previous {};
    uint32_t _period {};
    uint32_t _trigger {};
    uint32_t _lost_completions {};
    bool _primed {};
    bool _frozen_main {};
    bool _exhausted {};
    bool _aborted {};
    std::atomic<bool> _published {false};
};
} // namespace UTBLoggerTrace
