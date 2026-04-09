/**********************************************************************************************
    Copyright © 2026 Digital Confections LLC

    Permission is hereby granted, free of charge, to any person obtaining a copy of
    this software and associated documentation files (the "Software"), to deal in the
    Software without restriction, including without limitation the rights to use, copy,
    modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
    and to permit persons to whom the Software is furnished to do so, subject to the
    following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
    INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
    PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
    FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
    OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.

**********************************************************************************************/

/*
 * Fixed-size circular character buffer with overwrite-on-full behavior.
 *
 * This module contains support functions for:
 * - staging short text messages in FIFO order
 * - normalizing stored characters to uppercase
 * - dropping the oldest entry automatically when new data arrives and the buffer is full
 *
 * Message ownership, parsing, and higher-level text policy belong elsewhere.
 */

#include "CircularStringBuff.h"
#include <stdlib.h>
#include <ctype.h>

/**
 * Construct a circular buffer with the requested capacity.
 *
 * @param size Maximum number of characters the buffer can hold.
 */
CircularStringBuff::CircularStringBuff(size_t size)
{
	buf_ = (char *)malloc(size);
	max_size_ = size;
	head_ = 0;
	tail_ = 0;
	full_ = false;
}

/**
 * Destroy the buffer and release its storage.
 */
CircularStringBuff::~CircularStringBuff()
{
	free(buf_);
}

/**
 * Clear the buffer contents without releasing storage.
 */
void CircularStringBuff::reset()
{
	head_ = tail_;
	full_ = false;
}

/**
 * Report whether the buffer currently holds no characters.
 *
 * @return true when the buffer is empty.
 */
bool CircularStringBuff::empty() const
{
	/* Equal indices indicate empty only when the "full" latch is clear. */
	return (!full_ && (head_ == tail_));
}

/**
 * Report whether the buffer has reached capacity.
 *
 * @return true when the buffer is full.
 */
bool CircularStringBuff::full() const
{
	return (full_);
}

/**
 * Insert one character at the head of the FIFO.
 *
 * Characters are converted to uppercase before storage. If the buffer is
 * already full, the oldest character is discarded to make room.
 *
 * @param item Character to store.
 */
void CircularStringBuff::put(char item)
{
	buf_[head_] = toupper(static_cast<unsigned char>(item));

	if(full_)
	{
		/* Advancing the tail preserves a fixed capacity by dropping the oldest item. */
		tail_ = (tail_ + 1) % max_size_;
	}

	head_ = (head_ + 1) % max_size_;

	full_ = head_ == tail_;
}

/**
 * Remove and return the oldest queued character.
 *
 * @return The oldest buffered character, or '\0' when the buffer is empty.
 */
char CircularStringBuff::get()
{
	if(empty())
	{
		return ('\0');
	}

	/* Advance the tail after reading so the slot becomes available for future writes. */
	char val = buf_[tail_];
	full_ = false;
	tail_ = (tail_ + 1) % max_size_;

	return (val);
}
