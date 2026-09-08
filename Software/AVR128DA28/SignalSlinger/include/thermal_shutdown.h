/*
 * Pure thermal-shutdown policy shared by firmware and host regression tests.
 */
#ifndef __THERMAL_SHUTDOWN_H__
#define __THERMAL_SHUTDOWN_H__

#include <stdint.h>

#define THERMAL_SHUTDOWN_ENABLED_MARKER ((uint8_t)0xA5)
#define THERMAL_SHUTDOWN_DISABLED_MARKER ((uint8_t)0x5A)

static inline bool thermalShutdownEnabledFromMarker(uint8_t marker)
{
	/* Only an explicit saved OFF choice disables protection. Legacy padding was zero. */
	return marker != THERMAL_SHUTDOWN_DISABLED_MARKER;
}

static inline bool thermalShutdownMarkerIsKnown(uint8_t marker)
{
	return marker == THERMAL_SHUTDOWN_ENABLED_MARKER || marker == THERMAL_SHUTDOWN_DISABLED_MARKER;
}

static inline uint8_t thermalShutdownMarkerForEnabled(bool enabled)
{
	return enabled ? THERMAL_SHUTDOWN_ENABLED_MARKER : THERMAL_SHUTDOWN_DISABLED_MARKER;
}

static inline bool evaluateThermalShutdownStateForPolicy(float processor_temperature,
	                                                      int8_t threshold,
	                                                      int8_t hysteresis,
	                                                      bool enabled,
	                                                      bool current_state)
{
	if(!enabled)
	{
		return false;
	}

	const int16_t clear_threshold = (int16_t)threshold - (int16_t)hysteresis;
	return (processor_temperature >= (float)threshold) ? true
	       : (processor_temperature <= (float)clear_threshold) ? false
	                                                               : current_state;
}

#endif
