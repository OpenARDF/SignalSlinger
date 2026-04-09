/*
 *  MIT License
 *
 *  Copyright (c) 2026 DigitalConfections
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

/*
 * Atomic access helpers for shared firmware state.
 *
 * This header provides small inline wrappers around critical sections so the
 * foreground code can safely read or update globals that are also touched by
 * ISRs or other shared objects.
 *
 * These helpers should be preferred over direct multi-byte accesses whenever
 * globals cross interrupt or module boundaries.
 */

#ifndef SIGNALSLINGER_SHARED_STATE_H_
#define SIGNALSLINGER_SHARED_STATE_H_

#include <atomic.h>
#include <string.h>
#include <time.h>

#include "globals.h"

/**
 * Read a shared time_t value inside a critical section.
 *
 * @param src Shared time value to snapshot.
 * @return Stable copy of the current value.
 */
static inline time_t atomic_read_time(const volatile time_t* src)
{
	time_t value;
	ENTER_CRITICAL(shared_time_read);
	value = *src;
	EXIT_CRITICAL(shared_time_read);
	return value;
}

/**
 * Read two related shared time_t values under one critical section.
 *
 * This is useful when the caller needs a consistent start/finish pair rather
 * than two separately sampled timestamps.
 *
 * @param a First shared time value.
 * @param b Second shared time value.
 * @param out_a Output pointer for the first snapshot.
 * @param out_b Output pointer for the second snapshot.
 */
static inline void atomic_read_time_pair(const volatile time_t* a, const volatile time_t* b, time_t* out_a, time_t* out_b)
{
	ENTER_CRITICAL(shared_time_pair_read);
	*out_a = *a;
	*out_b = *b;
	EXIT_CRITICAL(shared_time_pair_read);
}

/**
 * Write a shared time_t value inside a critical section.
 *
 * @param dst Shared destination to update.
 * @param value New time value.
 */
static inline void atomic_write_time(volatile time_t* dst, time_t value)
{
	ENTER_CRITICAL(shared_time_write);
	*dst = value;
	EXIT_CRITICAL(shared_time_write);
}

/**
 * Write two related shared time_t values under one critical section.
 *
 * @param a First shared destination.
 * @param b Second shared destination.
 * @param value_a New value for a.
 * @param value_b New value for b.
 */
static inline void atomic_write_time_pair(volatile time_t* a, volatile time_t* b, time_t value_a, time_t value_b)
{
	ENTER_CRITICAL(shared_time_pair_write);
	*a = value_a;
	*b = value_b;
	EXIT_CRITICAL(shared_time_pair_write);
}

/**
 * Read a shared float value inside a critical section.
 *
 * @param src Shared float to snapshot.
 * @return Stable copy of the current value.
 */
static inline float atomic_read_float(const volatile float* src)
{
	float value;
	ENTER_CRITICAL(shared_float_read);
	value = *src;
	EXIT_CRITICAL(shared_float_read);
	return value;
}

/**
 * Write a shared float value inside a critical section.
 *
 * @param dst Shared destination to update.
 * @param value New float value.
 */
static inline void atomic_write_float(volatile float* dst, float value)
{
	ENTER_CRITICAL(shared_float_write);
	*dst = value;
	EXIT_CRITICAL(shared_float_write);
}

/**
 * Read a shared uint32_t value inside a critical section.
 *
 * @param src Shared value to snapshot.
 * @return Stable copy of the current value.
 */
static inline uint32_t atomic_read_u32(const volatile uint32_t* src)
{
	uint32_t value;
	ENTER_CRITICAL(shared_u32_read);
	value = *src;
	EXIT_CRITICAL(shared_u32_read);
	return value;
}

/**
 * Read a shared int32_t value inside a critical section.
 *
 * @param src Shared value to snapshot.
 * @return Stable copy of the current value.
 */
static inline int32_t atomic_read_i32(const volatile int32_t* src)
{
	int32_t value;
	ENTER_CRITICAL(shared_i32_read);
	value = *src;
	EXIT_CRITICAL(shared_i32_read);
	return value;
}

/**
 * Write a shared int32_t value inside a critical section.
 *
 * @param dst Shared destination to update.
 * @param value New value to store.
 */
static inline void atomic_write_i32(volatile int32_t* dst, int32_t value)
{
	ENTER_CRITICAL(shared_i32_write);
	*dst = value;
	EXIT_CRITICAL(shared_i32_write);
}

/**
 * Read a shared uint16_t value inside a critical section.
 *
 * @param src Shared value to snapshot.
 * @return Stable copy of the current value.
 */
static inline uint16_t atomic_read_u16(const volatile uint16_t* src)
{
	uint16_t value;
	ENTER_CRITICAL(shared_u16_read);
	value = *src;
	EXIT_CRITICAL(shared_u16_read);
	return value;
}

/**
 * Read a shared int value inside a critical section.
 *
 * @param src Shared value to snapshot.
 * @return Stable copy of the current value.
 */
static inline int atomic_read_int(const volatile int* src)
{
	int value;
	ENTER_CRITICAL(shared_int_read);
	value = *src;
	EXIT_CRITICAL(shared_int_read);
	return value;
}

/**
 * Write a shared uint16_t value inside a critical section.
 *
 * @param dst Shared destination to update.
 * @param value New value to store.
 */
static inline void atomic_write_u16(volatile uint16_t* dst, uint16_t value)
{
	ENTER_CRITICAL(shared_u16_write);
	*dst = value;
	EXIT_CRITICAL(shared_u16_write);
}

/**
 * Write a shared int value inside a critical section.
 *
 * @param dst Shared destination to update.
 * @param value New value to store.
 */
static inline void atomic_write_int(volatile int* dst, int value)
{
	ENTER_CRITICAL(shared_int_write);
	*dst = value;
	EXIT_CRITICAL(shared_int_write);
}

/**
 * Replace a shared uint16_t value and return the previous one atomically.
 *
 * @param dst Shared destination to update.
 * @param value New value to store.
 * @return Previous value held in dst.
 */
static inline uint16_t atomic_exchange_u16(volatile uint16_t* dst, uint16_t value)
{
	uint16_t prev;
	ENTER_CRITICAL(shared_u16_exchange);
	prev = *dst;
	*dst = value;
	EXIT_CRITICAL(shared_u16_exchange);
	return prev;
}

/**
 * Raise a shared uint16_t to at least the supplied minimum value.
 *
 * @param dst Shared destination to update.
 * @param min_value Lower bound to enforce.
 */
static inline void atomic_max_u16(volatile uint16_t* dst, uint16_t min_value)
{
	ENTER_CRITICAL(shared_u16_max);
	if(*dst < min_value)
	{
		*dst = min_value;
	}
	EXIT_CRITICAL(shared_u16_max);
}

/**
 * Report whether the shared text buffer is currently empty.
 *
 * @return true when no characters are queued, false otherwise.
 */
static inline bool text_buff_empty_atomic(void)
{
	bool is_empty;
	ENTER_CRITICAL(shared_text_buff_empty);
	is_empty = g_text_buff.empty();
	EXIT_CRITICAL(shared_text_buff_empty);
	return is_empty;
}

/**
 * Clear the shared text buffer inside a critical section.
 */
static inline void text_buff_reset_atomic(void)
{
	ENTER_CRITICAL(shared_text_buff_reset);
	g_text_buff.reset();
	EXIT_CRITICAL(shared_text_buff_reset);
}

/**
 * Attempt to remove one character from the shared text buffer.
 *
 * @param out Output pointer for the dequeued character.
 * @return true if a character was returned, false if the buffer was empty.
 */
static inline bool text_buff_try_get_atomic(char* out)
{
	bool has_char = false;
	ENTER_CRITICAL(shared_text_buff_get);
	if(!g_text_buff.empty())
	{
		*out = g_text_buff.get();
		has_char = true;
	}
	EXIT_CRITICAL(shared_text_buff_get);
	return has_char;
}

/**
 * Clear one shared message-text slot.
 *
 * @param slot Message slot index to clear.
 */
static inline void messages_text_slot_clear_atomic(uint8_t slot)
{
	ENTER_CRITICAL(shared_messages_text_clear);
	g_messages_text[slot][0] = '\0';
	EXIT_CRITICAL(shared_messages_text_clear);
}

/**
 * Copy a bounded string fragment into one shared message-text slot.
 *
 * The helper always appends a null terminator, even when the source length is
 * clipped to MAX_PATTERN_TEXT_LENGTH.
 *
 * @param slot Message slot index to update.
 * @param src Source character data.
 * @param len Maximum number of source characters to consider.
 */
static inline void messages_text_slot_write_atomic(uint8_t slot, const char* src, size_t len)
{
	size_t copy_len = MIN((size_t)MAX_PATTERN_TEXT_LENGTH, len);
	ENTER_CRITICAL(shared_messages_text_write);
	strncpy(g_messages_text[slot], src, copy_len);
	g_messages_text[slot][copy_len] = '\0';
	EXIT_CRITICAL(shared_messages_text_write);
}

/**
 * Copy one shared message-text slot into a caller buffer safely.
 *
 * @param slot Message slot index to read.
 * @param dst Destination buffer.
 * @param dst_size Size of the destination buffer in bytes.
 */
static inline void messages_text_slot_copy_atomic(uint8_t slot, char* dst, size_t dst_size)
{
	if(!dst || !dst_size) return;
	ENTER_CRITICAL(shared_messages_text_copy);
	strncpy(dst, g_messages_text[slot], dst_size - 1);
	dst[dst_size - 1] = '\0';
	EXIT_CRITICAL(shared_messages_text_copy);
}

/**
 * Publish a full null-terminated string into one shared message-text slot.
 *
 * @param slot Message slot index to update.
 * @param src Null-terminated source string.
 */
static inline void messages_text_slot_publish_atomic(uint8_t slot, const char* src)
{
	ENTER_CRITICAL(shared_messages_text_publish);
	strncpy(g_messages_text[slot], src, MAX_PATTERN_TEXT_LENGTH + 1);
	g_messages_text[slot][MAX_PATTERN_TEXT_LENGTH + 1] = '\0';
	EXIT_CRITICAL(shared_messages_text_publish);
}

/**
 * Read the fox setting for the current event under one critical section.
 *
 * @return Fox selection associated with g_event at the time of the snapshot.
 */
static inline Fox_t fox_setting_current_atomic(void)
{
	Fox_t fox;
	ENTER_CRITICAL(shared_fox_current_read);
	fox = g_fox[g_event];
	EXIT_CRITICAL(shared_fox_current_read);
	return fox;
}

/**
 * Read the current event and its current fox selection together.
 *
 * @param event_out Optional output pointer for the current event.
 * @param fox_out Optional output pointer for the current fox setting.
 */
static inline void event_and_fox_current_atomic(Event_t* event_out, Fox_t* fox_out)
{
	ENTER_CRITICAL(shared_event_fox_current_read);
	if(event_out) *event_out = g_event;
	if(fox_out) *fox_out = g_fox[g_event];
	EXIT_CRITICAL(shared_event_fox_current_read);
}

/**
 * Update the fox selection stored for one event slot.
 *
 * @param event_slot Event slot to modify.
 * @param fox New fox value for that slot.
 */
static inline void fox_setting_slot_write_atomic(Event_t event_slot, Fox_t fox)
{
	ENTER_CRITICAL(shared_fox_slot_write);
	g_fox[event_slot] = fox;
	EXIT_CRITICAL(shared_fox_slot_write);
}

/**
 * Update the fox selection for whichever event is currently active.
 *
 * @param fox New fox value for the current event slot.
 */
static inline void fox_setting_current_slot_write_atomic(Fox_t fox)
{
	ENTER_CRITICAL(shared_fox_current_slot_write);
	g_fox[g_event] = fox;
	EXIT_CRITICAL(shared_fox_current_slot_write);
}

#endif /* SIGNALSLINGER_SHARED_STATE_H_ */
