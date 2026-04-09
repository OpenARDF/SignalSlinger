/**
 * \file
 *
 * \brief Configuration Change Protection write functions
 *
 (c) 2020 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms,you may use this software and
    any derivatives exclusively with Microchip products.It is your responsibility
    to comply with third party license terms applicable to your use of third party
    software (including open source software) that may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

/*
 * Declarations for protected low-level register writes.
 *
 * This header exposes the shared helper used to perform writes that require
 * AVR Configuration Change Protection (CCP) timing or similar guarded update
 * sequences.
 */

#ifndef PROTECTED_IO_H
#define PROTECTED_IO_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__DOXYGEN__)
//! \name IAR Memory Model defines.
//@{

/**
 * \def CONFIG_MEMORY_MODEL_TINY
 * \brief Configuration symbol to enable 8 bit pointers.
 *
 */
#define CONFIG_MEMORY_MODEL_TINY

/**
 * \def CONFIG_MEMORY_MODEL_SMALL
 * \brief Configuration symbol to enable 16 bit pointers.
 * \note If no memory model is defined, SMALL is default.
 *
 */
#define CONFIG_MEMORY_MODEL_SMALL

/**
 * \def CONFIG_MEMORY_MODEL_LARGE
 * \brief Configuration symbol to enable 24 bit pointers.
 *
 */
#define CONFIG_MEMORY_MODEL_LARGE

//@}
#endif

/**
 * Write one byte to a protected AVR register using the supplied unlock value.
 *
 * The implementation lives in the companion assembly source so the unlock and
 * write sequence can meet the device's timing requirements exactly.
 *
 * @param addr Address of the protected register.
 * @param magic CCP unlock value or other protection mask expected by the target register.
 * @param value Byte value to write after unlocking.
 */
extern void protected_write_io(void *addr, uint8_t magic, uint8_t value);

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PROTECTED_IO_H */
