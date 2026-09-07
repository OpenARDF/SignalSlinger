#include "session_history.h"
#include "thermal_shutdown.h"
#include <assert.h>

int main() {
    const uint32_t start = 1788526800, finish = start + 9 * 3600;
    SessionLifecycle session;
    assert(sessionDayIndex(start, finish, 3, start - 3 * 86400) == 0);
    assert(session.begin(start, start, finish, true));
    assert(session.pause(REASON_THERMAL));
    assert(!session.pause(REASON_THERMAL)); // No repeated EEPROM records while hot.
    assert(!session.canResume(start + 100, false, true));
    assert(!session.canResume(start + 100, true, false)); // Explicit cancellation wins.
    assert(!session.canResume(finish, true, true)); // Never extend the original window.
    assert(session.resume(start + 100, true, true));
    assert(session.pause(REASON_SENSOR));
    assert(session.resume(start + 200, true, true));
    assert(session.stop(REASON_FINISH));
    assert(session.record.action == SESSION_FINISHED_INTERRUPTED);
    assert(!session.stop(REASON_FINISH)); // Finish must be idempotent.
    assert(sessionDayIndex(start, finish, 3, finish) == 1);
    assert(session.begin(start, start + 86400, finish + 86400, true));
    assert(session.pause(REASON_THERMAL));
    assert(session.stop(REASON_FINISH));
    assert(session.record.action == SESSION_INTERRUPTED && session.record.reason == REASON_THERMAL);
    assert(sessionDayIndex(start, finish, 3, finish + 86400) == 2);
    assert(session.begin(start, start + 2 * 86400, finish + 2 * 86400, true));
    assert(session.stop(REASON_FINISH));
    assert(session.record.action == SESSION_COMPLETED);
    assert(sessionDayIndex(start, finish, 3, finish + 2 * 86400) == 3);
    assert(sessionDayIndex(start, finish, 3, finish + 20 * 86400) == 3);
    assert(session.begin(start, start, finish, false));
    assert(session.pause(REASON_THERMAL));
    assert(!session.resume(start + 1, true, true)); // Manual operation requires another command.
    assert(session.stop(REASON_RESET));
    assert(session.record.action == SESSION_INTERRUPTED && session.record.reason == REASON_RESET);
    assert(sessionSequenceAfter(0, UINT32_MAX));
    assert(!sessionSequenceAfter(UINT32_MAX, 0));
    assert(evaluateThermalShutdownStateForPolicy(65, 65, 5, true, false));
    assert(evaluateThermalShutdownStateForPolicy(60.1F, 65, 5, true, true));
    assert(!evaluateThermalShutdownStateForPolicy(60, 65, 5, true, true));
}
