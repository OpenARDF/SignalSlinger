/*
 * Platform-neutral bounded-fraction arithmetic for Si5351 register fields.
 */

#include "si5351_fraction.h"

static uint32_t gcd(uint32_t left, uint32_t right)
{
	while(right)
	{
		const uint32_t remainder = left % right;
		left = right;
		right = remainder;
	}
	return left;
}

/* Compare two non-negative fractions without cross multiplication, whose
 * 64-bit product could overflow. Continued-fraction quotients preserve the
 * ordering, reversing it each time the fractional remainders are inverted. */
static int compareFractions(uint64_t left_numerator,
	                        uint64_t left_denominator,
	                        uint64_t right_numerator,
	                        uint64_t right_denominator)
{
	bool reversed = false;
	for(;;)
	{
		const uint64_t left_quotient = left_numerator / left_denominator;
		const uint64_t right_quotient = right_numerator / right_denominator;
		if(left_quotient != right_quotient)
		{
			const int result = left_quotient < right_quotient ? -1 : 1;
			return reversed ? -result : result;
		}

		const uint64_t left_remainder = left_numerator % left_denominator;
		const uint64_t right_remainder = right_numerator % right_denominator;
		if(!left_remainder || !right_remainder)
		{
			const int result = left_remainder == right_remainder ? 0 :
			                   (!left_remainder ? -1 : 1);
			return reversed ? -result : result;
		}

		left_numerator = left_denominator;
		left_denominator = left_remainder;
		right_numerator = right_denominator;
		right_denominator = right_remainder;
		reversed = !reversed;
	}
}

static uint64_t absoluteDifference(uint64_t left, uint64_t right)
{
	return left >= right ? left - right : right - left;
}

Si5351Fraction si5351BoundedFraction(uint32_t numerator,
	                                 uint32_t denominator,
	                                 uint32_t max_value)
{
	if(!denominator || !max_value || !numerator)
		return {0, 1};

	const uint32_t divisor = gcd(numerator, denominator);
	numerator /= divisor;
	denominator /= divisor;
	if(numerator <= max_value && denominator <= max_value)
		return {numerator, denominator};

	/* Continued-fraction convergents bracket the exact value. When the next
	 * convergent exceeds the field limit, the last valid semiconvergent and
	 * convergent are the only possible nearest bounded answers. */
	const uint64_t original_numerator = numerator;
	const uint64_t original_denominator = denominator;
	uint64_t p0 = 0;
	uint64_t q0 = 1;
	uint64_t p1 = 1;
	uint64_t q1 = 0;
	uint64_t n = numerator;
	uint64_t d = denominator;

	for(;;)
	{
		const uint64_t quotient = n / d;
		if(q0 + quotient * q1 > max_value)
			break;

		const uint64_t p2 = p0 + quotient * p1;
		const uint64_t q2 = q0 + quotient * q1;
		p0 = p1;
		q0 = q1;
		p1 = p2;
		q1 = q2;

		const uint64_t remainder = n % d;
		if(!remainder)
			return {(uint32_t)p1, (uint32_t)q1};
		n = d;
		d = remainder;
	}

	const uint64_t multiplier = (max_value - q0) / q1;
	const uint64_t bound1_numerator = p0 + multiplier * p1;
	const uint64_t bound1_denominator = q0 + multiplier * q1;
	const uint64_t bound2_numerator = p1;
	const uint64_t bound2_denominator = q1;

	const uint64_t error1 = absoluteDifference(original_numerator * bound1_denominator,
	                                          original_denominator * bound1_numerator);
	const uint64_t error2 = absoluteDifference(original_numerator * bound2_denominator,
	                                          original_denominator * bound2_numerator);

	/* The common original denominator cancels. Prefer the current convergent on
	 * an exact tie, matching the conventional limit-denominator algorithm. */
	if(compareFractions(error2, bound2_denominator,
	                    error1, bound1_denominator) <= 0)
		return {(uint32_t)bound2_numerator, (uint32_t)bound2_denominator};

	return {(uint32_t)bound1_numerator, (uint32_t)bound1_denominator};
}
