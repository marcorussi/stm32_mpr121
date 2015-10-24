/*
* The MIT License (MIT)
*
* Copyright (c) 2015 Marco Russi
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/


/* ---------------- Inclusions ----------------- */

#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/f4/nvic.h>
/* MPR121 module */
#include "mpr121.h"




/* ------------- Local functions prototypes --------------- */

static void clock_setup(void);




/* ------------ Local functions implementation ----------- */

/* Function to setup the clock device */
static void clock_setup(void)
{
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

	/* Enable GPIOD clock. */
	rcc_periph_clock_enable(RCC_GPIOD);

	/* Set GPIO12, GPIO13, GPIO14, GPIO15 (in GPIO port D) to 'output push-pull'. */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12 | GPIO13 | GPIO14 | GPIO15);
	/* clear LEDs */
	gpio_clear(GPIOD, GPIO12);
	gpio_clear(GPIOD, GPIO13);
	gpio_clear(GPIOD, GPIO14);
	gpio_clear(GPIOD, GPIO15);
}




/* Main function */
int main(void)
{
	uint16_t touch_status_flags;

	/* setup clock */
	clock_setup();

	/* init MPR121 */
	if(true == mpr121_init()) {
		/* init success: Green LED ON */
		gpio_set(GPIOD, GPIO12);
	} else {
		/* init fail: Red LED ON */
		gpio_set(GPIOD, GPIO14);
		/* hang here... */
		while(1);
	}

	/* infinite loop */
	while (1) {
		/* update electrodes touch status flags */
		touch_status_flags = mpr121_get_touch();

		/* if electrode 0 is pressed */
		if ((touch_status_flags & 0x0001) > 0) {
			/* set LED */
			gpio_set(GPIOD, GPIO15);
		} else {
			/* clear LED */
			gpio_clear(GPIOD, GPIO15);
		}

		/* if electrode 11 is pressed */
		if ((touch_status_flags & 0x0800) > 0) {
			/* set LED */
			gpio_set(GPIOD, GPIO13);
		} else {
			/* clear LED */
			gpio_clear(GPIOD, GPIO13);
		}

	};

	return 0;
}




/* End of file */
