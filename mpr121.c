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
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/f4/nvic.h>

#include "mpr121.h"
#include "mpr121_def.h"




/* ------------- Local defines --------------- */

/* Uncomment this define to enable MPR121 IRQ interrupt */
//#define MPR121_INT_ENABLED


/* Number of electrodes of MPR121 */
#define NUM_OF_ELECTRODES	13


/* MPR121 I2C bus address */
#define MPR121_ADDRESS_BYTE	0x5A


#ifdef MPR121_INT_ENABLED
/* MPR121 IRQ port, pins, exti defines */
#define MPR121_IRQ_RCC		RCC_GPIOA
#define MPR121_IRQ_PORT		GPIOA
#define MPR121_IRQ_PIN 		GPIO4
#define MPR121_IRQ_EXTI 	EXTI4
#define MPR121_IRQ_isr 		exti4_isr
#define MPR121_IRQ_NVIC 	NVIC_EXTI4_IRQ
#endif




/* ------------- Local functions prototypes ------------ */

static void i2c_init(void);
static void pins_init(void);
static bool write_register(uint8_t, uint8_t);
static bool read_register(uint8_t, uint8_t *);




/* ---------------- Exported functions ----------------- */

/* get touch status */
uint16_t mpr121_get_touch( void )
{
	uint8_t reg_value = 0;
	uint16_t touch_flags = 0; 

	/* read Touch 1 Status register */
	read_register(TS1, &reg_value);
	/* store lower 8 electrodes status flags */
	touch_flags = reg_value;
	/* read Touch 2 Status register */
	read_register(TS2, &reg_value);	
	/* clear unused higher flags */
	reg_value &= 0x1F;
	/* store higher 5 electrodes status flags */
	touch_flags |= (reg_value << 8);

	return touch_flags;
}


/* MPR121 init */
bool mpr121_init( void )
{
	bool success;
	uint8_t electrodes_count;
  	uint8_t reg_value = 0;

	success = true;

	/* init port pins */
	pins_init();
	/* init I2C interface */
	i2c_init();

	/* soft reset */
	write_register(SRST, 0x63); 

	/* read AFE Configuration 2 */
	read_register(AFE2, &reg_value);
	/* check default value */
	if (reg_value != 0x24) {
		/* error */
		success = false;
	} else {
		/* OK */
	}

	/* read Touch Status register */
	read_register(TS2, &reg_value);
	/* check default value */
	if ((reg_value & 0x80) != 0) {
		/* error */
		success = false;
	} else {
		/* OK */
	}	

	/* if no previous error */
	if (success == true)
	{
		/* turn off all electrodes to stop */
		write_register(ECR, 0x00); 

		/* write register with initial values */
		write_register(MHDR, 0x01);
		write_register(NHDR, 0x01);
		write_register(NCLR, 0x10);
		write_register(FDLR, 0x20);
		write_register(MHDF, 0x01);
		write_register(NHDF, 0x01);
		write_register(NCLF, 0x10);
		write_register(FDLF, 0x20);
		write_register(NHDT, 0x01);
		write_register(NCLT, 0x10);
		write_register(FDLT, 0xFF);
		write_register(MHDPROXR, 0x0F);
		write_register(NHDPROXR, 0x0F);
		write_register(NCLPROXR, 0x00);
		write_register(FDLPROXR, 0x00);
		write_register(MHDPROXF, 0x01);
		write_register(NHDPROXF, 0x01);
		write_register(NCLPROXF, 0xFF);
		write_register(FDLPROXF, 0xFF);
		write_register(NHDPROXT, 0x00);
		write_register(NCLPROXT, 0x00);
		write_register(FDLPROXT, 0x00);
	  	write_register(DTR, 0x11);
		write_register(AFE1, 0xFF);  
		write_register(AFE2, 0x30);
		write_register(ACCR0, 0x00);
		write_register(ACCR1, 0x00);
		write_register(USL, 0x00); 
		write_register(LSL, 0x00); 
		write_register(TL, 0x00); 
		write_register(ECR, 0xCC); // default to fast baseline startup and 12 electrodes enabled, no prox
	
		/* apply next setting for all electrodes */
		for (electrodes_count = 0; electrodes_count < NUM_OF_ELECTRODES; electrodes_count++) {
			write_register((E0TTH + (electrodes_count<<1)), 40); 
			write_register((E0RTH + (electrodes_count<<1)), 20); 
		}

		/* enable electrodes and set the current to 16uA */
		write_register(ECR, 0x10); 
	}
	else
	{
		/* init error */
	}

	return success;
}




/* ---------------- Local functions ----------------- */

/* function to write a value to a register into the MPR121. Returned value is not used at the moment */
static bool write_register(uint8_t reg_addr, uint8_t value)
{
	bool success = true;

	while ((I2C_SR2(I2C1) & I2C_SR2_BUSY) != 0);

	/* send START and wait for completion */
	i2c_send_start(I2C1);
	while ((I2C_SR1(I2C1) & I2C_SR1_SB) == 0);

	/* send device address, r/w request and wait for completion */
	i2c_send_7bit_address(I2C1, MPR121_ADDRESS_BYTE, I2C_WRITE);
	while ((I2C_SR1(I2C1) & I2C_SR1_ADDR) == 0);

	/* check SR2 and go on if OK */
	if ((I2C_SR2(I2C1) & I2C_SR2_MSL)		/* master mode */
	&&  (I2C_SR2(I2C1) & I2C_SR2_BUSY)) {		/* communication ongoing  */

		/* send register address */
		i2c_send_data(I2C1, reg_addr);
		while ((I2C_SR1(I2C1) & I2C_SR1_TxE) == 0);

		/* send data byte */
		i2c_send_data(I2C1, value);
		while ((I2C_SR1(I2C1) & I2C_SR1_TxE) == 0);

		/* send stop */
		i2c_send_stop(I2C1);

		/* ATTENTION: consider to wait for a while */
	} else {
		/* error */
		success = false;
	}

	return success;
}


/* function to read a register value from the MPR121. Returned value is not used at the moment */
static bool read_register(uint8_t reg_addr, uint8_t *value_ptr)
{
	bool success = true;

	while ((I2C_SR2(I2C1) & I2C_SR2_BUSY) != 0);

	/* send START and wait for completion */
	i2c_send_start(I2C1);
	while ((I2C_SR1(I2C1) & I2C_SR1_SB) == 0);

	/* send device address, write request and wait for completion */
	i2c_send_7bit_address(I2C1, MPR121_ADDRESS_BYTE, I2C_WRITE);
	while ((I2C_SR1(I2C1) & I2C_SR1_ADDR) == 0);

	/* check SR2 and go on if OK */
	if ((I2C_SR2(I2C1) & I2C_SR2_MSL)		/* master mode */
	&&	(I2C_SR2(I2C1) & I2C_SR2_BUSY)) {	/* communication ongoing  */

		/* send register address */
		i2c_send_data(I2C1, reg_addr);
		while ((I2C_SR1(I2C1) & I2C_SR1_TxE) == 0);

		/* send START and wait for completion */
		i2c_send_start(I2C1);
		while ((I2C_SR1(I2C1) & I2C_SR1_SB) == 0);

		/* send device address, read request and wait for completion */
		i2c_send_7bit_address(I2C1, MPR121_ADDRESS_BYTE, I2C_READ);
		while ((I2C_SR1(I2C1) & I2C_SR1_ADDR) == 0);

		/* if communication ongoing  */
		if (I2C_SR2(I2C1) & I2C_SR2_BUSY) {
			/* read received byte */
			while ((I2C_SR1(I2C1) & I2C_SR1_RxNE) == 0);
			*value_ptr = i2c_get_data(I2C1);
			/* send stop */
			i2c_send_stop(I2C1);
		} else {
			/* error */
			success = false;
		}
	} else {
		/* error */
		success = false;
	}

	return success;
}


/* I2C interface function */
static void i2c_init(void)
{
	/* Enable I2C1 clock. */
	rcc_periph_clock_enable(RCC_I2C1);
	/* Enable I2C1 interrupt. */
	nvic_enable_irq(NVIC_I2C1_EV_IRQ);
	/* reset I2C1 */
	i2c_reset(I2C1);
	/* standard mode */
	i2c_set_standard_mode(I2C1);
	/* clock and bus frequencies */
	i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_2MHZ);
	i2c_set_ccr(I2C1, 20);
	/* enable error event interrupt only */
	i2c_enable_interrupt(I2C1, I2C_CR2_ITERREN);
	/* enable I2C */
	i2c_peripheral_enable(I2C1);
}


/* port pins init function */
static void pins_init(void)
{
#ifdef MPR121_INT_ENABLED
	/* --- IRQ pin init --- */
	/* Enable IRQ PORT clock. */
	rcc_periph_clock_enable(MPR121_IRQ_RCC);

	/* Enable MPR121 IRQ interrupt. */
	nvic_enable_irq(MPR121_IRQ_NVIC);

	/* set MPR121 IRQ as input with external pull-up resistors */
	gpio_mode_setup(MPR121_IRQ_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, MPR121_IRQ_PIN);

	/* Configure the EXTI subsystem. */
	exti_select_source(MPR121_IRQ_EXTI, MPR121_IRQ_PORT);
	exti_set_trigger(MPR121_IRQ_EXTI, EXTI_TRIGGER_FALLING);

	/* enable MPR121 interrupt */
	exti_enable_request(MPR121_IRQ_EXTI);
#endif

	/* --- I2C interface init --- */
	/* Enable GPIOB clock. */
	rcc_periph_clock_enable(RCC_GPIOB);

	/* set I2C1_SCL and I2C1_SDA, external pull-up resistors */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
	/* Open Drain, Speed 100 MHz */
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ, GPIO6 | GPIO7);
	/* Alternate Function: I2C1 */
	gpio_set_af(GPIOB, GPIO_AF4,  GPIO6 | GPIO7);
}


#ifdef MPR121_INT_ENABLED
/* MPR121 IRQ function */
void MPR121_IRQ_isr(void)
{
	exti_reset_request(MPR121_IRQ_EXTI);

	/* do what you want... */
}
#endif



/* End of file */

