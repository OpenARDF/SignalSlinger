#ifndef SIGNALSLINGER_SHARED_STATE_H_
#define SIGNALSLINGER_SHARED_STATE_H_

#include <atomic.h>
#include <string.h>
#include <time.h>

#include "globals.h"

static inline time_t atomic_read_time(const volatile time_t* src)
{
	time_t value;
	ENTER_CRITICAL(shared_time_read);
	value = *src;
	EXIT_CRITICAL(shared_time_read);
	return value;
}

static inline void atomic_read_time_pair(const volatile time_t* a, const volatile time_t* b, time_t* out_a, time_t* out_b)
{
	ENTER_CRITICAL(shared_time_pair_read);
	*out_a = *a;
	*out_b = *b;
	EXIT_CRITICAL(shared_time_pair_read);
}

static inline void atomic_write_time(volatile time_t* dst, time_t value)
{
	ENTER_CRITICAL(shared_time_write);
	*dst = value;
	EXIT_CRITICAL(shared_time_write);
}

static inline void atomic_write_time_pair(volatile time_t* a, volatile time_t* b, time_t value_a, time_t value_b)
{
	ENTER_CRITICAL(shared_time_pair_write);
	*a = value_a;
	*b = value_b;
	EXIT_CRITICAL(shared_time_pair_write);
}

static inline float atomic_read_float(const volatile float* src)
{
	float value;
	ENTER_CRITICAL(shared_float_read);
	value = *src;
	EXIT_CRITICAL(shared_float_read);
	return value;
}

static inline uint32_t atomic_read_u32(const volatile uint32_t* src)
{
	uint32_t value;
	ENTER_CRITICAL(shared_u32_read);
	value = *src;
	EXIT_CRITICAL(shared_u32_read);
	return value;
}

static inline uint16_t atomic_read_u16(const volatile uint16_t* src)
{
	uint16_t value;
	ENTER_CRITICAL(shared_u16_read);
	value = *src;
	EXIT_CRITICAL(shared_u16_read);
	return value;
}

static inline void atomic_write_u16(volatile uint16_t* dst, uint16_t value)
{
	ENTER_CRITICAL(shared_u16_write);
	*dst = value;
	EXIT_CRITICAL(shared_u16_write);
}

static inline uint16_t atomic_exchange_u16(volatile uint16_t* dst, uint16_t value)
{
	uint16_t prev;
	ENTER_CRITICAL(shared_u16_exchange);
	prev = *dst;
	*dst = value;
	EXIT_CRITICAL(shared_u16_exchange);
	return prev;
}

static inline void atomic_max_u16(volatile uint16_t* dst, uint16_t min_value)
{
	ENTER_CRITICAL(shared_u16_max);
	if(*dst < min_value)
	{
		*dst = min_value;
	}
	EXIT_CRITICAL(shared_u16_max);
}

static inline bool text_buff_empty_atomic(void)
{
	bool is_empty;
	ENTER_CRITICAL(shared_text_buff_empty);
	is_empty = g_text_buff.empty();
	EXIT_CRITICAL(shared_text_buff_empty);
	return is_empty;
}

static inline void text_buff_reset_atomic(void)
{
	ENTER_CRITICAL(shared_text_buff_reset);
	g_text_buff.reset();
	EXIT_CRITICAL(shared_text_buff_reset);
}

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

static inline void messages_text_slot_clear_atomic(uint8_t slot)
{
	ENTER_CRITICAL(shared_messages_text_clear);
	g_messages_text[slot][0] = '\0';
	EXIT_CRITICAL(shared_messages_text_clear);
}

static inline void messages_text_slot_write_atomic(uint8_t slot, const char* src, size_t len)
{
	size_t copy_len = MIN((size_t)MAX_PATTERN_TEXT_LENGTH, len);
	ENTER_CRITICAL(shared_messages_text_write);
	strncpy(g_messages_text[slot], src, copy_len);
	g_messages_text[slot][copy_len] = '\0';
	EXIT_CRITICAL(shared_messages_text_write);
}

static inline void messages_text_slot_copy_atomic(uint8_t slot, char* dst, size_t dst_size)
{
	if(!dst || !dst_size) return;
	ENTER_CRITICAL(shared_messages_text_copy);
	strncpy(dst, g_messages_text[slot], dst_size - 1);
	dst[dst_size - 1] = '\0';
	EXIT_CRITICAL(shared_messages_text_copy);
}

static inline void messages_text_slot_publish_atomic(uint8_t slot, const char* src)
{
	ENTER_CRITICAL(shared_messages_text_publish);
	strncpy(g_messages_text[slot], src, MAX_PATTERN_TEXT_LENGTH);
	g_messages_text[slot][MAX_PATTERN_TEXT_LENGTH] = '\0';
	EXIT_CRITICAL(shared_messages_text_publish);
}

static inline Fox_t fox_setting_current_atomic(void)
{
	Fox_t fox;
	ENTER_CRITICAL(shared_fox_current_read);
	fox = g_fox[g_event];
	EXIT_CRITICAL(shared_fox_current_read);
	return fox;
}

static inline void event_and_fox_current_atomic(Event_t* event_out, Fox_t* fox_out)
{
	ENTER_CRITICAL(shared_event_fox_current_read);
	if(event_out) *event_out = g_event;
	if(fox_out) *fox_out = g_fox[g_event];
	EXIT_CRITICAL(shared_event_fox_current_read);
}

static inline void fox_setting_slot_write_atomic(Event_t event_slot, Fox_t fox)
{
	ENTER_CRITICAL(shared_fox_slot_write);
	g_fox[event_slot] = fox;
	EXIT_CRITICAL(shared_fox_slot_write);
}

static inline void fox_setting_current_slot_write_atomic(Fox_t fox)
{
	ENTER_CRITICAL(shared_fox_current_slot_write);
	g_fox[g_event] = fox;
	EXIT_CRITICAL(shared_fox_current_slot_write);
}

#endif /* SIGNALSLINGER_SHARED_STATE_H_ */
