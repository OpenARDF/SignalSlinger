#include "si5351_fraction.h"

#include <assert.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>

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

static long double fractionError(uint32_t numerator,
	                             uint32_t denominator,
	                             const Si5351Fraction &fraction)
{
	return std::fabs((long double)numerator / denominator -
	                 (long double)fraction.numerator / fraction.denominator);
}

static void verifySmallLimitsAgainstBruteForce()
{
	for(uint32_t limit = 1; limit <= 32; ++limit)
	{
		for(uint32_t denominator = 1; denominator <= 96; ++denominator)
		{
			for(uint32_t numerator = 0; numerator < denominator; ++numerator)
			{
				const Si5351Fraction actual =
				    si5351BoundedFraction(numerator, denominator, limit);
				assert(actual.denominator >= 1 && actual.denominator <= limit);
				assert(actual.numerator <= limit);

				long double best_error = 2.0L;
				for(uint32_t candidate_denominator = 1;
				    candidate_denominator <= limit;
				    ++candidate_denominator)
				{
					for(uint32_t candidate_numerator = 0;
					    candidate_numerator <= limit;
					    ++candidate_numerator)
					{
						const Si5351Fraction candidate = {
						    candidate_numerator, candidate_denominator};
						const long double error =
						    fractionError(numerator, denominator, candidate);
						if(error < best_error)
							best_error = error;
					}
				}
				if(fractionError(numerator, denominator, actual) > best_error + 1e-15L)
					std::fprintf(stderr, "bounded fraction mismatch: %u/%u limit %u -> %u/%u\n",
					             numerator, denominator, limit,
					             actual.numerator, actual.denominator);
				assert(fractionError(numerator, denominator, actual) <= best_error + 1e-15L);
			}
		}
	}
}

static uint32_t transmitterDivider(uint32_t frequency_hz)
{
	return 900000000UL / frequency_hz;
}

int main()
{
	static const uint32_t field_max = 1048575UL;
	verifySmallLimitsAgainstBruteForce();

	const uint32_t reference_frequencies[] = {24975000UL, 25000000UL, 25025000UL};
	long double worst_error_hz = 0.0L;
	uint32_t worst_frequency_hz = 0;
	long double nominal_worst_error_hz = 0.0L;
	uint32_t nominal_worst_frequency_hz = 0;
	uint32_t exact_count = 0;
	uint32_t bounded_count = 0;
	bool endings_seen[100] = {};

	for(uint32_t output_hz = 3500000UL; output_hz <= 4000000UL; ++output_hz)
	{
		const uint32_t divider = transmitterDivider(output_hz);
		assert(divider >= 4);
		const uint32_t vco_hz = output_hz * divider;
		assert(vco_hz >= 600000000UL && vco_hz <= 900000000UL);

		for(const uint32_t reference_hz : reference_frequencies)
		{
			const uint32_t integer = vco_hz / reference_hz;
			const uint32_t remainder = vco_hz % reference_hz;
			const uint32_t divisor = remainder ? gcd(remainder, reference_hz) : reference_hz;
			const uint32_t exact_numerator = remainder / divisor;
			const uint32_t exact_denominator = reference_hz / divisor;
			Si5351Fraction fraction =
			    si5351BoundedFraction(remainder, reference_hz, field_max);
			uint32_t programmed_integer = integer;
			if(fraction.numerator == fraction.denominator)
			{
				++programmed_integer;
				fraction = {0, 1};
			}

			assert(fraction.numerator <= field_max);
			assert(fraction.denominator <= field_max);
			assert(fraction.numerator < fraction.denominator);

			if(exact_numerator <= field_max && exact_denominator <= field_max)
			{
				assert(fraction.numerator == exact_numerator);
				assert(fraction.denominator == exact_denominator);
				if(reference_hz == 25000000UL)
					++exact_count;
			}
			else if(reference_hz == 25000000UL)
			{
				++bounded_count;
			}

			const uint64_t bx128 = (uint64_t)fraction.numerator << 7;
			const uint32_t bx128_over_c = (uint32_t)(bx128 / fraction.denominator);
			const uint64_t p1 = ((uint64_t)programmed_integer << 7) + bx128_over_c - 512;
			const uint64_t p2 = bx128 -
			                    (uint64_t)fraction.denominator * bx128_over_c;
			assert(p1 <= 0x3FFFFUL);
			assert(p2 <= field_max);

			const long double actual_vco_hz =
			    reference_hz * (programmed_integer +
			                    (long double)fraction.numerator / fraction.denominator);
			const long double error_hz =
			    std::fabs(actual_vco_hz / divider - output_hz);
			if(error_hz > worst_error_hz)
			{
				worst_error_hz = error_hz;
				worst_frequency_hz = output_hz;
			}
			if(reference_hz == 25000000UL && error_hz > nominal_worst_error_hz)
			{
				nominal_worst_error_hz = error_hz;
				nominal_worst_frequency_hz = output_hz;
			}
		}

		const uint32_t ending = output_hz % 100;
		if(ending == 1 || ending == 3 || ending == 7 || ending == 9 ||
		   ending == 10 || ending == 30 || ending == 70 || ending == 90)
			endings_seen[ending] = true;
	}

	for(const uint32_t ending : {1U, 3U, 7U, 9U, 10U, 30U, 70U, 90U})
		assert(endings_seen[ending]);
	assert(exact_count + bounded_count == 500001UL);
	assert(worst_error_hz <= 0.1L);

	std::printf("Si5351 exhaustive 1 Hz calibration sweep: %u exact, %u bounded; "
	            "nominal worst %.12Lf Hz at %u Hz; correction-limit worst "
	            "%.12Lf Hz at %u Hz\n",
	            exact_count, bounded_count,
	            nominal_worst_error_hz, nominal_worst_frequency_hz,
	            worst_error_hz, worst_frequency_hz);
	return 0;
}
