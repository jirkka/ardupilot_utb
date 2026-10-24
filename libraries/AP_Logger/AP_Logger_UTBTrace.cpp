//----------------------------------------------------------------------------------------------------------------------
// NOTICE: UTB diagnostic extension, GPL-3.0-or-later.
//----------------------------------------------------------------------------------------------------------------------
/// @file AP_Logger_UTBTrace.cpp
/// @brief Main-only RAM capture; no logger calls, allocation or new locks in capture.
/// @author Jiri Kovar
/// @version 2026-10-21
/// @date 2026-10-21
#include "AP_Logger_UTBTrace.h"
#if AP_UTB_LOGGER_TRACE_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/ExpandingString.h>
extern const AP_HAL::HAL &hal;
namespace UTBLoggerTrace
{
// Static data declarations: one writer; publication happens once per boot.
static Recorder recorder;
static bool started;
static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "Trace requires lock-free publication");

// Public functions
void loop(uint32_t now, uint32_t nominal)
{
    // Exclude MCU startup and armed operation; never re-arm or alter vehicle state.
    if (hal.util->get_soft_armed()) {
        recorder.abort(now);
        return;
    }
    if (started || now >= 30000000U) {
        started = true;
        recorder.loop(now, nominal);
    }
}
void task_overrun(int16_t task, uint32_t start, uint32_t duration)
{
    if (hal.scheduler->in_main_thread() && !hal.util->get_soft_armed()) {
        recorder.task_overrun(task, start, duration);
    }
}
Span::Span(Event event)
{
    if (hal.scheduler->in_main_thread() && !hal.util->get_soft_armed()) {
        _start = AP_HAL::micros();
        _event = event;
        _ticket = recorder.begin(event, _start, hal.util->persistent_data.scheduler_task);
    }
}
void Span::finish()
{
    if (_ticket != Recorder::INVALID) {
        recorder.end(_ticket, AP_HAL::micros(), _start, _event);
        _ticket = Recorder::INVALID;
    }
}
void dump(ExpandingString &text)
{
    if (hal.util->get_soft_armed()) {
        text.printf("UTBLoggerTraceV1,armed-unavailable\n");
        return;
    }
    // No state is read until main has published an immutable snapshot.
    if (!recorder.frozen()) {
        text.printf("UTBLoggerTraceV1,pending\n");
        return;
    }
    text.printf("UTBLoggerTraceV1,frozen,trigger=%u,period=%u,tickets=%u,lost=%u,aborted=%u,exhausted=%u\n",
                unsigned(recorder.trigger()), unsigned(recorder.period()),
                unsigned(recorder.tickets()), unsigned(recorder.lost_completions()),
                unsigned(recorder.aborted()), unsigned(recorder.exhausted()));
    text.printf("summary,event,count,max_us,start_us,task\n");
    for (uint8_t i = 0; i < uint8_t(Event::COUNT); i++) {
        const Summary &s = recorder.summary(Event(i));
        text.printf("summary,%u,%u,%u,%u,%d\n", unsigned(i), unsigned(s.count),
                    unsigned(s.maximum), unsigned(s.start), int(s.task));
    }
    text.printf("ticket,cycle,start_us,duration_us,task,event,complete\n");
    for (uint32_t i = 0; i < recorder.count(); i++) {
        const Entry &e = recorder.entry(i);
        text.printf("%u,%u,%u,%u,%d,%u,%u\n", unsigned(e.ticket), unsigned(e.cycle),
                    unsigned(e.start), unsigned(e.duration), int(e.task),
                    unsigned(e.event), unsigned(e.complete));
    }
}
}
#endif
