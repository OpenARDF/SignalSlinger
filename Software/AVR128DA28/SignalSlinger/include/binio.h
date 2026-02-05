/*
 *  MIT License
 *
 *  Copyright (c) 2022 DigitalConfections
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


#ifndef __BINIO_H__
#define __BINIO_H__

#include "defs.h"

/* PORTA *************************************************************************************/
#ifdef HW_TARGET_3_5
#define COOLING_FAN_ENABLE 7
#else
#define BOOST_PWR_ENABLE 7
#endif

#define STRAIGHTKEY 6 /* Straight key */
#define PADDLE_DAH 5 /* Paddle Dah */
#define PADDLE_DIT 4 /* Paddle Dit */
#define POWER_ENABLE 3
#define V3V3_PWR_ENABLE 2
#define FET_DRIVER_ENABLE 1
#define CHARGE_AUX_ENABLE 0

/* PORTC *************************************************************************************/
#define SI5351_SCL 3
#define SI5351_SDA 2
#define SERIAL_RX 1
#define SERIAL_TX 0

/* PORTD *************************************************************************************/
#define unusedD7 7
#define DAC_OUTPUT 6
#define unusedD5 5
#define LED_GREEN 4
#define SWITCH 3
#define LED_RED 2
#define VBAT_EXT 1
#define VBAT_INT 0

/* PORTF *************************************************************************************/
#define unusedF6 6
#define X32KHZ_SQUAREWAVE 0
#define unusedF1 1

enum hardwareResourceClients {
	INTERNAL_BATTERY_CHARGING,
	TRANSMITTER,
	NUMBER_OF_LS_CONTROLLERS,
	INITIALIZE_LS,
	RE_APPLY_LS_STATE
};

/**
 */
void BINIO_init(void);

/**
 */
void debounce(void);

/**
 */
uint8_t portDdebouncedVals(void);

/**
 */
bool setFETDriverLoadSwitch(bool onoff, hardwareResourceClients sender);
void fet_driver(bool state);
bool get_fet_driver(void);

/**
 */
bool get_V3V3_enable(void);

/**
 */
bool setExtBatLoadSwitch(hardwareResourceClients client);
bool setExtBatLoadSwitch(bool onoff, hardwareResourceClients sender);

/**
 */
bool setSignalGeneratorEnable(bool onoff, hardwareResourceClients sender);

#ifdef HW_TARGET_3_5
void setCoolingFanLSEnable(bool onoff);
bool getCoolingFanLSEnable(void);
#else
/**
 */
bool setBoostEnable(bool onoff);
#endif

/**
 */
bool getExtBatLSEnable(void);

class binio
{
//variables
public:
protected:
private:

//functions
public:
	binio();
	~binio();
protected:
private:
	binio( const binio &c );
	binio& operator=( const binio &c );

}; //binio

#endif //__BINIO_H__
