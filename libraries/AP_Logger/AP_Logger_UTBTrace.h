//----------------------------------------------------------------------------------------------------------------------
// NOTICE: UTB diagnostic extension, GPL-3.0-or-later.
//----------------------------------------------------------------------------------------------------------------------
/// @file AP_Logger_UTBTrace.h
/// @brief Default-OFF DISARMED main-thread logger timing scopes.
/// @author Jiri Kovar
/// @version 2026-10-21
/// @date 2026-10-21
#pragma once
#ifndef AP_UTB_LOGGER_TRACE_ENABLED
#define AP_UTB_LOGGER_TRACE_ENABLED 0
#endif
#if AP_UTB_LOGGER_TRACE_ENABLED
#include "AP_Logger_UTBTrace_Recorder.h"
class ExpandingString;
namespace UTBLoggerTrace
{
// Public functions: export is permitted only after the immutable freeze.
void loop(uint32_t now, uint32_t nominal);
void dump(ExpandingString &text);
void task_overrun(int16_t task, uint32_t start, uint32_t duration);
class Span
{
public:
    /// @brief Begin a main-thread-only scope using the hardware clock.
    explicit Span(Event event);
    /// @brief Complete the span, including early-return paths.
    ~Span()
    {
        finish();
    }
    /// @brief Explicitly end a wait immediately after the original lock acquisition.
    void finish();
private:
    // Zero ticket denotes a scope excluded by thread/arming/startup policy.
    uint32_t _ticket {};
    uint32_t _start {};
    Event _event {};
};
}
#define UTB_TRACE_SCOPE(name, event) UTBLoggerTrace::Span name(UTBLoggerTrace::Event::event)
#define UTB_TRACE_FINISH(name) name.finish()
#else
#define UTB_TRACE_SCOPE(name, event)
#define UTB_TRACE_FINISH(name)
#endif
