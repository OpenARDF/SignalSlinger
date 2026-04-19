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

#ifndef CIRCULARSTRINGBUFF_H_
#define CIRCULARSTRINGBUFF_H_

#include <stddef.h>

/**
 * Simple fixed-size FIFO buffer for characters.
 *
 * The buffer stores characters in a ring. When full, inserting another
 * character overwrites the oldest one so writers never block.
 */
class CircularStringBuff {
	public:
	/**
	 * Destroy the buffer and release its storage.
	 */
	~CircularStringBuff();

	/**
	 * Construct a circular buffer with the requested capacity.
	 *
	 * @param size Maximum number of characters the buffer can hold.
	 */
	CircularStringBuff(size_t);

	/**
	 * Insert one character at the head of the FIFO.
	 *
	 * Characters are converted to uppercase before storage. If the buffer is
	 * already full, the oldest character is discarded to make room.
	 *
	 * @param item Character to store.
	 */
	void put(char item);

	/**
	 * Remove and return the oldest queued character.
	 *
	 * @return The oldest buffered character, or '\0' when the buffer is empty.
	 */
	char get(void);

	/**
	 * Clear the buffer contents without releasing storage.
	 */
	void reset(void);

	/**
	 * Report whether the buffer currently holds no characters.
	 *
	 * @return true when the buffer is empty.
	 */
	bool empty(void) const;

	/**
	 * Report whether the buffer has reached capacity.
	 *
	 * @return true when the buffer is full.
	 */
	bool full(void) const;

	private:
	/* Non-copyable (C++98 style). */
	CircularStringBuff(const CircularStringBuff&);
	CircularStringBuff& operator=(const CircularStringBuff&);

	int head_;
	int tail_;
	bool full_;
	char* buf_;
	size_t max_size_;
};

#endif /* CIRCULARSTRINGBUFF_H_ */
