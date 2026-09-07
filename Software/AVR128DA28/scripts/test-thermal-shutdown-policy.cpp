#include "thermal_shutdown.h"

#include <assert.h>

int main()
{
	assert(!thermalShutdownEnabledFromMarker(THERMAL_SHUTDOWN_DISABLED_MARKER));
	assert(thermalShutdownEnabledFromMarker(THERMAL_SHUTDOWN_ENABLED_MARKER));
	assert(thermalShutdownEnabledFromMarker(0xFF));
	assert(thermalShutdownEnabledFromMarker(0x00));
	assert(thermalShutdownEnabledFromMarker(0x37));
	assert(!thermalShutdownMarkerIsKnown(0x00));
	assert(thermalShutdownMarkerForEnabled(false) == THERMAL_SHUTDOWN_DISABLED_MARKER);
	assert(thermalShutdownMarkerForEnabled(true) == THERMAL_SHUTDOWN_ENABLED_MARKER);

	assert(!evaluateThermalShutdownStateForPolicy(125.0F, 85, 5, false, true));
	assert(!evaluateThermalShutdownStateForPolicy(84.9F, 85, 5, true, false));
	assert(evaluateThermalShutdownStateForPolicy(85.0F, 85, 5, true, false));
	assert(evaluateThermalShutdownStateForPolicy(82.0F, 85, 5, true, true));
	assert(!evaluateThermalShutdownStateForPolicy(80.0F, 85, 5, true, true));

	return 0;
}
