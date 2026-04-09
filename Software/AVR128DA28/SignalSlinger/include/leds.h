/*
 * LEDs status and blink-pattern helpers.
 *
 * This module contains support functions for:
 * - driving red/green LED indications through a timer-backed blink engine
 * - coordinating temporary overrides such as wake-authorization blinking
 * - queuing short LED-enunciated character sequences
 *
 * Higher-level UI policy and state selection belong elsewhere.
 */

#ifndef __LEDS_H__
#define __LEDS_H__

typedef enum
{
	LEDS_OFF,
	LEDS_RED_OFF,
	LEDS_GREEN_OFF,
	LEDS_RED_BLINK_FAST,
	LEDS_RED_BLINK_SLOW,
	LEDS_GREEN_BLINK_FAST,
	LEDS_GREEN_BLINK_SLOW,
	LEDS_RED_ON_CONSTANT,
	LEDS_GREEN_ON_CONSTANT,
	LEDS_RED_AND_GREEN_BLINK_FAST,
	LEDS_RED_AND_GREEN_BLINK_WAKE_AUTH,
	LEDS_RED_AND_GREEN_BLINK_SLOW,
	LEDS_RED_THEN_GREEN_BLINK_FAST,
	LEDS_RED_THEN_GREEN_BLINK_SLOW,
	LEDS_NUMBER_OF_SETTINGS,
	LEDS_NO_CHANGE
} Blink_t;

class leds
{
  public:
  protected:
  private:
  public:
	/**
	 * Initialize the LED subsystem with LEDs off.
	 */
	void init(void);

	/**
	 * Initialize the LED subsystem with an optional starting blink pattern.
	 *
	 * @param setBlink Initial blink pattern to apply.
	 */
	void init(Blink_t setBlink);

	/**
	 * Apply a new blink pattern without resetting the inactivity timeout.
	 *
	 * @param blinkMode Blink pattern to apply.
	 */
	void blink(Blink_t blinkMode);

	/**
	 * Apply a new blink pattern and optionally reset the inactivity timeout.
	 *
	 * @param blinkMode Blink pattern to apply.
	 * @param resetTimeout true to refresh the LED inactivity timeout.
	 */
	void blink(Blink_t blinkMode, bool resetTimeout);

	/**
	 * Report whether the timer-driven LED subsystem is currently active.
	 *
	 * @return true when LED indications are active and not timed out.
	 */
	bool active(void);

	/**
	 * Disable timer-driven LED control and blank both LEDs.
	 */
	void deactivate(void);

	/**
	 * Force the red LED on or off immediately.
	 *
	 * @param on Desired red-LED state.
	 */
	void setRed(bool on);

	/**
	 * Force the green LED on or off immediately.
	 *
	 * @param on Desired green-LED state.
	 */
	void setGreen(bool on);

	/**
	 * Queue a short text string for LED-based manual transmission.
	 *
	 * @param str Null-terminated string to enqueue.
	 */
	void sendCode(char *str);

	/**
	 * Enable or disable the special wake-authorization blink override.
	 *
	 * @param active true to force the wake-authorization blink pattern.
	 */
	void setWakeAuthorizationBlink(bool active);

  protected:
	/**
	 * Reset LED state, blink configuration, and timeout tracking.
	 */
	void reset(void);
	void reset(Blink_t setBlink);

  private:
	/* Non-copyable. */
	leds &operator=(const leds &c);

}; // leds

#endif //__LEDS_H__
