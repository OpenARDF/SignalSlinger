#ifndef EXTERNAL_BATTERY_FEEDBACK_H
#define EXTERNAL_BATTERY_FEEDBACK_H

#include <stdint.h>

/* Awake-only requests for the existing ADC scheduler. TCB0 runs at 300 Hz
 * (24 MHz / 2 / 40000); the legacy TIMER2_* names describe a different timer.
 * The controller never performs a conversion or changes transmission state.
 */
class ExternalBatteryFeedback
{
public:
    static const uint16_t TICKS_PER_SECOND = 300;
    static const uint16_t SETTLE_TICKS = 3; // At least 10 ms after requesting power.
    static const uint16_t VISIBLE_INTERVAL = TICKS_PER_SECOND;
    static const uint16_t PROBE_INTERVAL = 10 * TICKS_PER_SECOND;
    static const uint16_t BACKGROUND_INTERVAL = 8 * TICKS_PER_SECOND;
    static const uint16_t REQUEST_TIMEOUT = TICKS_PER_SECOND / 10;

    void reset()
    {
        initialized_ = visible_ = powered_ = managed_ = pending_ = probe_ = false;
        settle_ = age_ = remaining_ = 0;
    }

    void tick(bool awake, bool visible, bool managed, bool powerRequested)
    {
        if(!awake)
        {
            reset();
            return;
        }

        // With battery control disabled, the auxiliary output may be a fan.
        // Sample the directly connected supply without touching that output.
        bool powered = !managed || powerRequested;
        bool refresh = !initialized_ || (visible && !visible_) || (powered && !powered_);
        bool becameDark = initialized_ && !visible && visible_;
        bool changedMode = initialized_ && managed != managed_;
        initialized_ = true;
        visible_ = visible;
        powered_ = powered;
        managed_ = managed;

        if(changedMode || (pending_ && !probe_ && !powered) || (probe_ && !visible))
        {
            complete();
            refresh = true;
        }

        if(pending_)
        {
            if(settle_) --settle_;
            if(++age_ >= REQUEST_TIMEOUT) complete(); // Bound a failed probe's power use.
            return;
        }

        if(becameDark) remaining_ = interval();
        if(remaining_) --remaining_;
        if((refresh || !remaining_) && (powered || visible))
        {
            pending_ = true;
            probe_ = managed && !powered;
            settle_ = SETTLE_TICKS;
            age_ = 0;
        }
    }

    bool probeRequested() const { return probe_; }
    bool sampleDue() const { return pending_ && !settle_; }

    void complete()
    {
        pending_ = probe_ = false;
        settle_ = age_ = 0;
        remaining_ = interval();
    }

private:
    uint16_t interval() const
    {
        return !visible_ ? BACKGROUND_INTERVAL : (powered_ ? VISIBLE_INTERVAL : PROBE_INTERVAL);
    }

    bool initialized_ = false;
    bool visible_ = false;
    bool powered_ = false;
    bool managed_ = false;
    bool pending_ = false;
    bool probe_ = false;
    uint16_t settle_ = 0;
    uint16_t age_ = 0;
    uint16_t remaining_ = 0;
};

#endif
