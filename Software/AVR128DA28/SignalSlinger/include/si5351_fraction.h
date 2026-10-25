/*
 * Platform-neutral bounded-fraction arithmetic for Si5351 register fields.
 */

#ifndef SIGNALSLINGER_SI5351_FRACTION_H
#define SIGNALSLINGER_SI5351_FRACTION_H

#include <stdint.h>

struct Si5351Fraction
{
	uint32_t numerator;
	uint32_t denominator;
};

/**
 * Reduce numerator/denominator exactly when possible, otherwise return the
 * nearest fraction whose numerator and denominator both fit max_value.
 */
Si5351Fraction si5351BoundedFraction(uint32_t numerator,
	                                 uint32_t denominator,
	                                 uint32_t max_value);

#endif /* SIGNALSLINGER_SI5351_FRACTION_H */
