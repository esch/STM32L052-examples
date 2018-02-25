/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *******************************************************************************/

#include "lmic.h"
#include "config.h" 

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/timer.h> 
#include <libopencm3/cm3/nvic.h>

// -----------------------------------------------------------------------------
// I/O
#ifdef REV0 
	#define PORT_DIO0			GPIOB	//DIO0  14 PB0
	#define PIN_DIO0			GPIO0
	#define PORT_DIO1			GPIOB	//DIO1  NC (15 PB1)
	#define PIN_DIO1			GPIO1
	#define PORT_DIO2			GPIOA	//DIO2  9  PA3
	#define PIN_DIO2			GPIO3
#else 
	#define PORT_DIO0			GPIOB	//DIO0  14 PB0
	#define PIN_DIO0			GPIO0
	#define PORT_DIO1			GPIOB	//DIO1  NC (15 PB1)
	#define PIN_DIO1			GPIO1
	#define PORT_DIO2			GPIOA	//DIO2  9  PA7
	#define PIN_DIO2			GPIO7
#endif




	// #define PORT_DIO4			gpioPortA	//DIO4 = PA1
	// #define PIN_DIO4			1U
	// #define PORT_DIO5			gpioPortA	//DIO5 = PA0
	// #define PIN_DIO5			0U
	#define USART_USED                USART0
	#define USART_LOCATION            USART_ROUTE_LOCATION_LOC0
	#define USART_CLK                 cmuClock_USART0
	#define PIN_SPI_TX                10			//MOSI = PB10
	#define PORT_SPI_TX               GPIOB
	#define PIN_SPI_RX                11			//MISO = PB11
	#define PORT_SPI_RX               GPIOB

#ifdef REV0
	#define PIN_SPI_CLK               12			//SCK = PA12
	#define PORT_SPI_CLK              GPIOA
#else
	#define PIN_SPI_CLK               3			//SCK = PB3
	#define PORT_SPI_CLK              GPIOB
#endif

	#define PIN_SPI_CS                GPIO15			//
	#define PORT_SPI_CS               GPIOA

#ifdef REV0
 	#define PIN_RST    				  GPIO15
 	#define PORT_RST    			  GPIOC
#else
 	#define PIN_RST    				  GPIO1
 	#define PORT_RST    			  GPIOA
#endif

// HAL state
static struct
{
    //int irqlevel;
    u4_t ticks;
} HAL;

extern unsigned int dbg_failed;

// void hal_io_check();

// -----------------------------------------------------------------------------
// I/O

void hal_io_init ()
{

	// rcc_clock_setup_hsi(&rcc_clock_config[RCC_CLOCK_VRANGE1_HSI_PLL_32MHZ]);

  	rcc_periph_clock_enable(RCC_SPI1);
  	/* For spi signal pins */
  	rcc_periph_clock_enable(RCC_GPIOA);
  	rcc_periph_clock_enable(RCC_GPIOB);
  	rcc_periph_clock_enable(RCC_GPIOC);

}

// val ==1  => tx 1, rx 0 ; val == 0 => tx 0, rx 1
void hal_pin_rxtx (u1_t val)
{
	//not used in PA52 nor PA53
}


// set radio NSS pin to given value
void hal_pin_nss (u1_t val)
{
	if (val)
		// GPIO_PinOutSet(GPIOA, PIN_SPI_CS);	//  SPI Disable
	    gpio_set(GPIOA,PIN_SPI_CS);
	else
	    gpio_clear(GPIOA,PIN_SPI_CS);
		// GPIO_PinOutClear(GPIOA, PIN_SPI_CS);	//  SPI Enable (Active Low)
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val)
{
	if( val ) { // drive pin
        gpio_set(PORT_RST, PIN_RST);
       } else {
        gpio_clear(PORT_RST, PIN_RST);
    }
}

// extern void radio_irq_handler(u1_t dio);

#ifdef ESCHLATER 	
void GPIO_ODD_IRQHandler(void)	//impar
 {
	u4_t i = GPIO_IntGet();
	if (i & 1<<PIN_DIO0)
		radio_irq_handler(0);
	else if (i & 1<<PIN_DIO1)
		radio_irq_handler(1);
	GPIO_IntClear(0xAAAA);
 }

void GPIO_EVEN_IRQHandler(void)	//par
 {
	u4_t i = GPIO_IntGet();
	if (i & 1<<PIN_DIO2)
		radio_irq_handler(2);
	GPIO_IntClear(0x5555);
 }

#endif

#define NUM_DIO  3

static bool dio_states[NUM_DIO] = {0,0,0};
static uint32_t dio_port[NUM_DIO] = {PORT_DIO0,PORT_DIO1,PORT_DIO2}; 
static uint16_t dio_pin[NUM_DIO] =  {PIN_DIO0,PIN_DIO1,PIN_DIO2};

static void hal_io_check() {
    uint8_t i;
    for (i = 0; i < NUM_DIO; ++i) {
        if (dio_states[i] != gpio_get(dio_port[i],dio_pin[i]) ) {
            dio_states[i] = !dio_states[i];
            if (dio_states[i])
                radio_irq_handler(i);
        }
    }
	// GPIO_IntClear(0x5555);

}


void hal_spi_init (void)
{

 
#define noTESTEXT

#ifdef TESTEXT
  /* Setup GPIO pins for AF0 for SPI1 signals. */

  // rev0
  // PORT A, AF0, PA5 SCK, PA6 MISO, PA7 MOSI -  PA4 NSS

  // gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,   GPIO4 | GPIO5 | GPIO6 | GPIO7);
  // gpio_set_af(GPIOA, GPIO_AF0,  GPIO4 | GPIO5 | GPIO6 | GPIO7);
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,    GPIO5 | GPIO6 | GPIO7);
  gpio_mode_setup(PORT_SPI_CS, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, PIN_SPI_CS);    // NSS
  gpio_mode_setup(PORT_RST, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_RST );  // RST




  gpio_set_af(GPIOA, GPIO_AF0,   GPIO5 | GPIO6 | GPIO7);

  gpio_set(PORT_SPI_CS,PIN_SPI_CS);


#else // RFM95 | NRF24
  // rev1
  // PORT A, AF0, PB3 SCK, PB4 MISO, PB5 MOSI -  PA15 NSS
  /* Setup GPIO pins for AF0 for SPI1 signals. */

  // PORT B, AF0, PB3 SCK, PB4 MISO, PB5 MOSI  NSS PA15
  // gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,  GPIO3 | GPIO4 | GPIO5 );
  // gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP,  GPIO3 | GPIO5 );
  // gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP,  GPIO4 );

// without the pullup in SCK, there is a problem
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP,  GPIO3 );
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,  GPIO5 );
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP,  GPIO4 );

  gpio_mode_setup(PORT_RST, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_RST );  

  gpio_set_af(GPIOB, GPIO_AF0, GPIO3 | GPIO4 | GPIO5);

  gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, PIN_SPI_CS);    //

  gpio_mode_setup(PORT_DIO0, GPIO_MODE_INPUT, GPIO_PUPD_NONE, PIN_DIO0 );  // DIO0
  gpio_mode_setup(PORT_DIO1, GPIO_MODE_INPUT, GPIO_PUPD_NONE, PIN_DIO1 );  // DIO1
  gpio_mode_setup(PORT_DIO2, GPIO_MODE_INPUT, GPIO_PUPD_NONE, PIN_DIO2 );  // DIO2



   gpio_set(GPIOA,PIN_SPI_CS);
#endif


  //spi initialization;

  // spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                  // SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR2_DS_8BIT, SPI_CR1_MSBFIRST);

  spi_set_master_mode(SPI1);

  spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_2);
  spi_set_clock_polarity_0(SPI1);
  spi_set_clock_phase_0(SPI1);
  spi_send_msb_first(SPI1);


  spi_set_full_duplex_mode(SPI1);
  spi_set_unidirectional_mode(SPI1); /* bidirectional but in 3-wire */
  // spi_set_data_size(SPI1, SPI_CR2_DS_8BIT);

  // spi_enable_software_slave_management(SPI1);
  spi_enable_ss_output(SPI1);
  
  spi_fifo_reception_threshold_8bit(SPI1);
  // SPI_I2SCFGR(SPI1) &= ~SPI_I2SCFGR_I2SMOD;
  spi_enable(SPI1);

}

// perform SPI transaction with radio
u1_t hal_spi (u1_t out)
{
	/* For every byte sent, one is received */
	spi_send8(SPI1, out);
	u1_t res = spi_read8(SPI1);

	return res;  // esch
}


#ifdef LATER
void get_data() {
  uint8_t spi_dr =  SPI_DR8(SPI1);
  print_hex8(spi_dr);

  uint16_t spi_sr =  SPI_SR(SPI1);
  print_hex16(spi_sr);

}
#endif

// -----------------------------------------------------------------------------
// TIME
// static uint8_t       rtcInitialized = 0;    /**< 1 if rtc is initialized */
// static uint32_t      rtcFreq;               /**< RTC Frequence. 32.768 kHz */

/***************************************************************************//**
 * @brief RTC Interrupt Handler, invoke callback function if defined.
 ******************************************************************************/
// void RTC_IRQHandler(void)
void rtc_isr(void)
{
#ifdef ESCHLATER	
	if (RTC_IntGet() & RTC_IF_OF)
	{
		HAL.ticks ++;
	}

    if(RTC_IntGet() & RTC_IF_COMP0) // expired
    {
        // do nothing, only wake up cpu
    }
	RTC_IntClear(_RTC_IF_MASK); // clear IRQ flags

	/* The interrupt flag isn't cleared by hardware, we have to do it. */
	rtc_clear_wakeup_flag();

	/* Visual output. */
	gpio_toggle(GPIOA, GPIO8);

	volatile uint32_t j = 0, c = 0;
	c = rtc_get_counter_val();

	/* Display the current counter value in binary via USART1. */
	for (j = 0; j < 32; j++) {
		if ((c & (0x80000000 >> j)) != 0)
			usart_send_blocking(USART1, '1');
		else
			usart_send_blocking(USART1, '0');
	}
#endif

	print("rtc_isr");
	usart_send_blocking(USART1, '\n');
	usart_send_blocking(USART1, '\r');


}


// uint16_t compare_time  = 0;
uint32_t msec;
u8_t usec;
u8_t lastmicros;
u8_t addticks;


void hal_time_init (void)
{


	/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM2);

	/* Enable TIM2 interrupt. */
	nvic_enable_irq(NVIC_TIM2_IRQ);

	/* Reset TIM2 peripheral to defaults. */
	rcc_periph_reset_pulse(RST_TIM2);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 * (These are actually default values after reset above, so this call
	 * is strictly unnecessary, but demos the api for alternative settings)
	 */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/*
	 * Please take note that the clock source for STM32 timers
	 * might not be the raw APB1/APB2 clocks.  In various conditions they
	 * are doubled.  See the Reference Manual for full details!
	 * In our case, TIM2 on APB1 is running at double frequency, so this
	 * sets the prescaler to have the timer run at 5kHz
	 */
	// timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 5000));  // .4 ms / period
	// timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 1000)); // 2ms / period
	timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 10000)); // 0.2ms / period

	/* Disable preload. */
	timer_disable_preload(TIM2);
	timer_continuous_mode(TIM2);

	/* count full range, as we'll update compare value continuously */
	// timer_set_period(TIM2, 65535);
	// timer_set_period(TIM2, 65535);
	// timer_set_period(TIM2, 30000);
	timer_set_period(TIM2, 500);

	/* Set the inital output compare value for OC1. */
	// timer_set_oc_value(TIM2, TIM_OC1, 1000);

	/* Counter enable. */
	timer_enable_counter(TIM2);

	/* Enable Channel 1 compare interrupt to recalculate compare values */
	timer_enable_irq(TIM2, TIM_DIER_CC1IE);


	nvic_enable_irq(NVIC_RTC_IRQ);
	nvic_set_priority(NVIC_RTC_IRQ, 1);


	msec = 0;
	usec = 0;
	lastmicros=0;
	addticks=0;


}


void tim2_isr(void)
{
	if (timer_get_flag(TIM2, TIM_SR_CC1IF)) {

		/* Clear compare interrupt flag. */
		timer_clear_flag(TIM2, TIM_SR_CC1IF);

		/*
		 * Get current timer value 
		 */
		// compare_time += timer_get_counter(TIM2);
		// compare_time += 100;
		msec += 100;
		usec += 100000;
	}
}


u8_t hal_ticks (void) {

// LMIC requires ticks to be 15.5μs - 100 μs long
// Check for overflow of micros()
if ( usec  < lastmicros ) {
    addticks += (u8_t)4294967296 / US_PER_OSTICK;
}
lastmicros = usec;
return ((u8_t)usec / US_PER_OSTICK) + addticks;

}


// Returns the number of ticks until time. 
static u4_t delta_time(u8_t time) {
      u8_t t = hal_ticks( );
      s4_t d = time - t;
      if (d<=1) { return 0; }
      else {
        return (u4_t)(time - hal_ticks());
      }
}



void hal_waitUntil (u8_t time)
{
    while( delta_time(time) != 0 ); // busy wait until timestamp is reached
}

// check and rewind for target time
u1_t hal_checkTimer (u8_t time) {
    // No need to schedule wakeup, since we're not sleeping
    return delta_time(time) <= 0;
}



// -----------------------------------------------------------------------------
// IRQ
static uint8_t irqlevel = 0;

void hal_disableIRQs ()
{
	// INT_Disable();

	irqlevel++;
}

void hal_enableIRQs ()
{
    if(--irqlevel == 0) {
        // sei();
        // enable_irq();

        // Instead of using proper interrupts (which are a bit tricky
        // and/or not available on all pins on AVR), just poll the pin
        // values. Since os_runloop disables and re-enables interrupts,
        // putting this here makes sure we check at least once every
        // loop.
        //
        // As an additional bonus, this prevents the can of worms that
        // we would otherwise get for running SPI transfers inside ISRs
        hal_io_check();
    }
}

void hal_sleep ()
{

	for(int x=0;x<10;x++)
      __asm__("nop");

}

// -----------------------------------------------------------------------------

void hal_init ()
{
    // CHIP_Init();

    memset(&HAL, 0x00, sizeof(HAL));
    hal_disableIRQs();

    hal_io_init();	// configure radio I/O and interrupt handler

    hal_spi_init();	// configure radio SPI

    hal_time_init();	// configure timer and interrupt handler

    hal_enableIRQs();
}

void hal_failed ()
{
	// debug_led(1);
	// HALT...
    hal_disableIRQs();
    hal_sleep();
    while(1) dbg_failed++;
}

