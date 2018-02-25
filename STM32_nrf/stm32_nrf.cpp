/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (C) 2013 Piotr Esden-Tempski <piotr@esden.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/timer.h>

#include <libopencm3/cm3/scb.h>

#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/exti.h>

#include "cstring"
#include "tinysh.h" 
#include "tinysh_setup.h" 
#include "nRF24L01P_Maniacbug.h"
#include "stm32_nrf.h" 

#include "epd_app.h" 


int atohc(char c);


void dng_fnt(int argc, char **argv);


void stm32_flash_read(void);
// uint32_t dbg_dosend = 0;

int channel = 88;
rf24_datarate_e _dataRate = RF24_250KBPS;
rf24_crclength_e _crcLength = RF24_CRC_8;

bool runState = true;
uint16_t sent_interval = 20;

uint16_t tracer = 0;


// ====================================================
// FLASH Settings struct
// ====================================================
// #define MY_FLASH_PAGE_ADDR 0x800FC00

#define MY_FLASH_PAGE_ADDR 0x08080000     // eeprom
#define EEPROM_DATA_ID 0xAA55

// const uint64_t pipes[2] = { 0xE8E8F0F0D1LL, 0xE8E8F0F0E1LL };
uint64_t pipes[2] = { 0xE8E8F0F0D1LL, 0xE8E8F0F0E1LL };

typedef struct {
    // id of the device -> max is 32 characters
    char id[16];
    // type of the device -> a simple int
    int16_t type;
    // table of value -> 8 values max
    int16_t val[7];
} Payload;

Payload ps;
Payload pr;

/* Morse standard timings */
#define ELEMENT_TIME 500
#define DIT (1*ELEMENT_TIME)
#define DAH (3*ELEMENT_TIME)
#define INTRA (1*ELEMENT_TIME)
#define INTER (3*ELEMENT_TIME)
#define WORD (7*ELEMENT_TIME)


uint16_t frequency_sequence[] = {
  DIT,
  INTRA,
  DIT,
  INTRA,
  DIT,
  INTER,
  DAH,
  INTRA,
  DAH,
  INTRA,
  DAH,
  INTER,
  DIT,
  INTRA,
  DIT,
  INTRA,
  DIT,
  WORD,
};

int frequency_sel = 0;

eeprom_Settings settings;

bool eeprom_SettingsValid = false;
 
static inline __attribute__((always_inline)) void __WFI(void)
{
  __asm volatile ("wfi");
}
 
static void clock_setup(void)
{
	/* Enable GPIOA clock . */
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable clocks for USART1. */
	rcc_periph_clock_enable(RCC_USART1);
  rcc_clock_setup_msi(&rcc_clock_config[RCC_CLOCK_VRANGE1_MSI_RAW_2MHZ]);

  // rcc_clock_setup_msi(&rcc_clock_config[RCC_CLOCK_VRANGE1_MSI_RAW_131KHZ]);

}


static void usart_setup(void)
{
	/* Enable the USART1 interrupt. */
  // review
	nvic_enable_irq(NVIC_USART1_IRQ);

	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);

	/* Setup GPIO pins for USART1 receive. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
	// gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO10);

	/* Setup USART1 TX and RX pin as alternate function. */
	gpio_set_af(GPIOA, GPIO_AF4, GPIO9);
	gpio_set_af(GPIOA, GPIO_AF4, GPIO10);

	/* Setup USART1 parameters. */
  // usart_set_baudrate(USART1, 115200);
  // usart_set_baudrate(USART1, 9600);
  usart_set_baudrate(USART1, 19200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Enable USART1 Receive interrupt. */
	usart_enable_rx_interrupt(USART1);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

static void gpio_setup(void)
{
	/* Setup GPIO pin GPIO8 on GPIO port D for LED. */
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 |  GPIO1 |GPIO8);
}


uint16_t adc_setup(void)
{

  // uint8_t channel_array[18];
  // channel_array[0] = ADC_CHANNEL_TEMP;
  // channel_array[0] = ADC_CHANNEL_VREF;

  rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);

  rcc_periph_clock_enable(RCC_ADC1);

  adc_set_clk_source(ADC1,ADC_CFGR2_CKMODE_PCLK);
  // rcc_periph_clock_enable(RCC_GPIOA);

  adc_enable_lfmode();
  print_s_hex32("adc cr 0  ",getADC_CR() );


  // seems only a reset can put the ADC_CR into 0 
  uint32_t to = adc_clear_cr();
  print_s_hex32("adc clear timeout   ",(uint32_t)to );

  // print_s_hex32("RCC_APB2ENR   ",(uint32_t)getRCC_APB2ENR() );
  // print_s_hex32("adc cr address  ",(uint32_t)adc_cr(ADC1) );

  print_s_hex32("adc cr 1  ",getADC_CR() );
  print_s_hex32("adc ISR 1  ",getADC_ISR() );
   print_s_hex32("adc CALF 1  ",getADC_CALFACT() );

   // for testing 
  adc_eocal_interrupt();
  adc_calibrate(ADC1);
  // power off for calib
  // uint16_t ct = adc_calibrate(ADC1);
  // print_s_decimal((char *)"cal timeout ",ct);

#ifdef DEBUG
  print_s_hex32("adc cr 2  ",getADC_CR() );
  print_s_hex32("adc ISR 2  ",getADC_ISR() );
   print_s_hex32("adc CALF 2  ",getADC_CALFACT() );
#endif

  println( adc_is_power_off(ADC1) ? (char *)"adc power off" : (char *)"adc power on" );
  // uint16_t u =  adc_power_off(ADC1);
  adc_power_off(ADC1);

  // print_s_decimal((char *) "timeout ",u);

  adc_set_single_conversion_mode(ADC1);

  println((char *)"adc_setup 2");
  // adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
  // adc_disable_external_trigger_regular(ADC1);
  adc_set_right_aligned(ADC1);

  
    // adc_enable_temperature_sensor();
    // adc_enable_chan(ADC_CHANNEL18);


  // adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_160DOT5);
  adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_079DOT5);
  
  // adc_set_regular_sequence(ADC1, 1, channel_array);


  adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
  adc_disable_analog_watchdog(ADC1);


  adc_power_on(ADC1);

print_s_hex32("adc cr 4  ",getADC_CR() );


return 0;
  /* Wait for ADC starting up. */
  // int i;
  // for (i = 0; i < 800000; i++) {    /* Wait a bit. */
  //   __asm__("nop");
  // }

}


static void tim_setup(void)
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
  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
    TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  /*
   * Please take note that the clock source for STM32 timers
   * might not be the raw APB1/APB2 clocks.  In various conditions they
   * are doubled.  See the Reference Manual for full details!
   * In our case, TIM2 on APB1 is running at double frequency, so this
   * sets the prescaler to have the timer run at 5kHz
   */
  timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 5000));

  /* Disable preload. */
  timer_disable_preload(TIM2);
  timer_continuous_mode(TIM2);

  /* count full range, as we'll update compare value continuously */
  // timer_set_period(TIM2, 65535);
  timer_set_period(TIM2, 6553);
  // timer_set_period(TIM2, 3655);
  // timer_set_period(TIM2, 1055);

  /* Set the initual output compare value for OC1. */
  timer_set_oc_value(TIM2, TIM_OC1, frequency_sequence[frequency_sel++]);

  /* Counter enable. */
  timer_enable_counter(TIM2);

  /* Enable Channel 1 compare interrupt to recalculate compare values */
  timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}

void tim2_isr(void)
{
  if (timer_get_flag(TIM2, TIM_SR_CC1IF)) {

    /* Clear compare interrupt flag. */
    timer_clear_flag(TIM2, TIM_SR_CC1IF);

    /*
     * Get current timer value to calculate next
     * compare register value.
     */
    // uint16_t compare_time = timer_get_counter(TIM2);

    // /* Calculate and set the next compare value. */
    // uint16_t frequency = frequency_sequence[frequency_sel++];
    // uint16_t new_time = compare_time + frequency;

    timer_set_oc_value(TIM2, TIM_OC1, 500);
    // if (frequency_sel == ARRAY_LEN(frequency_sequence)) {
    //  frequency_sel = 0;
    // }

    /* Toggle LED to indicate compare event. */
    gpio_toggle(GPIOA, GPIO8);
  }
}

void enter_Standby( void )  // 200 nA 
{
    /* Enable Clocks */
    RCC_APB1ENR |= RCC_APB1ENR_PWREN;
    RCC_APB2ENR = 0 ;
    RCC_AHBRSTR  = 0;
     
    /* Prepare for Standby */
    // if WKUP pins are already high, the WUF bit will be set
    PWR_CSR |= PWR_CSR_EWUP1 | PWR_CSR_EWUP2;
     
    PWR_CR |= PWR_CR_CWUF; // clear the WUF flag after 2 clock cycles
    PWR_CR |= PWR_CR_ULP;   // V_{REFINT} is off in low-power mode
    PWR_CR |= PWR_CR_PDDS; // Enter Standby mode when the CPU enters deepsleep
     

//   print_s_hex32("RTC CR ",getRTC_CR() );
//   print_s_hex32("RTC ISR ",getRTC_ISR() );
//   print_s_hex32("RTC PRER ",getRTC_PRER() );
//   print_s_hex32("RTC WUTR ",getRTC_WUTR() );
//   print_s_hex32("RTC TAFCR ",getRTC_TAFCR() );
//   print_s_hex32("RTC APB1ENR ",getRCC_APB1ENR() );
//   print_s_hex32("RTC CCIPR ",getRCC_CCIPR() );

// wait(2);
    SCB_SCR |= SCB_SCR_SLEEPDEEP; // low-power mode = stop mode
    SCB_SCR |= SCB_SCR_SLEEPONEXIT; // reenter low-power mode after ISR

    __WFI(); // enter low-power mode
}


void enter_Sleep( void )  // about 35 uA with 131 KHz clock - powerRange1 - 
{
    /* Configure low-power mode */
  SCB_SCR &= ~( SCB_SCR_SLEEPDEEP );
  SCB_SCR |= SCB_SCR_SLEEPONEXIT;
     
    /* Ensure Flash memory stays on */
    FLASH_ACR |= FLASH_ACR_SLEEPPD;

    __WFI();  // enter low-power mode
}


void enter_LPSleep( void )  // about 20 uA 131 KHz clock
{
    /* 1. The Flash memory can be switched off by using the control bits
              (SLEEP_PD in the FLASH_ACR register). This reduces power consumption
              but increases the wake-up time. */
    // FLASH->ACR |= FLASH_ACR_SLEEP_PD;
    FLASH_ACR |= FLASH_ACR_SLEEPPD;

    /* 2. Each digital IP clock must be enabled or disabled by using the
                RCC_APBxENR and RCC_AHBENR registers */
    // RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    RCC_APB1ENR |= RCC_APB1ENR_PWREN;

    /* 3. The frequency of the system clock must be decreased to not exceed the
                frequency of f_MSI range1. */
    // Set MSI 131.072 kHz as system clock
    // Config_SysClk_MSI_131();

    rcc_clock_setup_msi(&rcc_clock_config[RCC_CLOCK_VRANGE1_MSI_RAW_131KHZ]);


    // Reinitialize peripherals dependent on clock speed
    // USART1_Init();
    // SysTick_Init( 0.001 );
    // I2C1_Init();

      flash_unlock_pdkey();

    /* 4. The regulator is forced in low-power mode by software
                (LPSDSR bits set ) */
    // PWR->CR |= PWR_CR_LPSDSR; // voltage regulator in low-power mode during sleep
  pwr_low_power();


    /* 5. Follow the steps described in Section 6.3.5: Entering low-power mode */
    // SCB->SCR &= ~( SCB_SCR_SLEEPDEEP_Msk ); // low-power mode = sleep mode
    // SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk; // reenter low-power mode after ISR
  SCB_SCR &= ~( SCB_SCR_SLEEPDEEP );
  SCB_SCR |= SCB_SCR_SLEEPONEXIT;


    __WFI(); // enter low-power mode
}

/* Temperature sensor calibration value address */
#define TEMP130_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007E))
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FF8007A))
#define VDD_CALIB ((uint16_t) (300))
#define VDD_APPLI ((uint16_t) (330))

#define VREFINT_CAL_ADDR ((uint16_t*) ((uint32_t)0x1FF80078))

int real_VDD = VDD_APPLI;


int32_t ComputeTemperature(uint32_t measure)
{
int32_t temperature;

  temperature = ((measure * real_VDD * (int32_t)100/ VDD_CALIB)- ((int32_t) *TEMP30_CAL_ADDR) * 100)  ;
  temperature = temperature  * (int32_t)(130 - 30);
  temperature = temperature / (int32_t)(*TEMP130_CAL_ADDR - *TEMP30_CAL_ADDR);
  temperature +=  3000;

  return(temperature);
}



void readTempADC(void) {
  uint16_t temp;
  
    adc_enable_temperature_sensor();
    adc_enable_chan(ADC_CHANNEL18);
    wait_ms(20);

    if(settings.dbg > 0) {
      println("------ temp sensor -----------");
      print_s_hex32("adc cr 1  ",getADC_CR() );

      print_s_hex32("adc cfgr1  ",getADC_CFGR1() );
      print_s_hex32("adc chselr  ",getADC_CHSELR() );
      print_s_hex32("adc dr  ",getADC_DR() );
    }

    adc_start_conversion_regular(ADC1);
    while (!(adc_eoc(ADC1)));
    if(settings.dbg > 0) print_s_hex32("adc dr 1 ",getADC_DR() );

    temp = adc_read_regular(ADC1);    
    if(settings.dbg > 0) print_s_decimal("temp sensor raw ", temp);

    int32_t celc = ComputeTemperature(temp);
    ps.val[4] = celc;
    if(settings.dbg > 0) print_s_decimal("temp sensor  ", celc);

    adc_disable_temperature_sensor();
    adc_disable_chan(ADC_CHANNEL18);
    if(settings.dbg > 0) println("------ ----------------- -----------");
}


// why do we have only 330 as voltage read ?
// can we set the init of the ADC

void readVREFADC(void) {
  uint16_t vref;

    adc_enable_vrefint();
    adc_enable_chan(ADC_CHANNEL17);
    if(settings.dbg > 0) {
      println("------ vref sensor -----------");

      print_s_hex32("adc cfgr1  ",getADC_CFGR1() );
      print_s_hex32("adc chselr  ",getADC_CHSELR() );
      print_s_hex32("adc dr  ",getADC_DR() );
    }

    adc_start_conversion_regular(ADC1);
    while (!(adc_eoc(ADC1)));
    if(settings.dbg > 0) print_s_hex32("adc dr 1 ",getADC_DR() );

    vref = adc_read_regular(ADC1);    
    if(settings.dbg > 0) print_s_decimal("vref raw ", vref);

    int32_t VDD;

      VDD =   300 * (*VREFINT_CAL_ADDR)/vref;
    if(settings.dbg > 0) print_s_decimal("VDD  ", VDD);
    real_VDD = VDD;

    // ps.val[0] = VDD;

    adc_disable_vrefint();
    adc_disable_chan(ADC_CHANNEL17);
    if(settings.dbg > 0) println("------ ----------------- -----------");

    // return VDD;
}


void rtc_awwake(void ) 
{

  // print_s_hex32("RCC CSR 0 ",getRCC_CSR() ); // check if we need to enable ??

  uint32_t rts = rcc_enable_rtc_clock();
  // print_s_hex32("rtc enable ", rts );

  // print_s_hex32("RCC ",getRCC_CSR() ); // check if we need to enable ??
    // print_s_hex32("RTC_TR 3 ", getRTC_TR() );

  // print_s_hex32(" pwr cr (enabled ?) ", getPWR_CR() );

     // print_s_hex32("RTC_TR 4 ", getRTC_TR() );

// if(!init) 
  rcc_set_rtc_clock_source( RCC_LSI );

  rcc_osc_ready_int_clear( RCC_LSI );
// wait_ms(500);

  // print_s_hex32("RTC_TR ", getRTC_TR() );
  // print_s_hex32("RCC CSR ",getRCC_CSR() );

  rtc_unlock();

// if(!init) {
  // println("init rtc ...");
  // print_s_hex32("RTC_TR 5", getRTC_TR() );
  rtc_start_init();

 // prescaler setting for 1 HZ on the LSI clock
  rtc_set_prescaler(295,124);
  rtc_exit_init();
  // }

// 
  // rtc_set_wakeup_time( 0x10 /* [sec]*/ ,RTC_CR_WUCLKSEL_SPRE ); // 

  uint32_t si = (settings.sent_interval ) ? settings.sent_interval : 0x10; 
  rtc_set_wakeup_time( si /* [sec]*/ ,RTC_CR_WUCLKSEL_SPRE ); // 

  rtc_enable_wakeup_irq();
  rtc_wait_for_synchro();
  rtc_lock();




  exti_set_trigger(EXTI20,EXTI_TRIGGER_RISING);
  exti_enable_request(EXTI20);


}

void rtc_isr(void)
{
  /* The interrupt flag isn't cleared by hardware, we have to do it. */
  rtc_clear_wakeup_flag();
  rtc_disable_wakeup_irq();

  // rtc_clear_flag(RTC_SEC);
  // rcc_clock_setup_msi(&rcc_clock_config[RCC_CLOCK_VRANGE1_MSI_RAW_131KHZ]);
  // rcc_clock_setup_msi(&rcc_clock_config[RCC_CLOCK_VRANGE1_MSI_RAW_2MHZ ]);

  // gpio_setup();
    clock_setup();
  gpio_setup();
  usart_setup();
 
  /* Visual output. */
  int y = 8;

  while(y--) {
    for(int h=0;h<88;h++)
      __asm__("NOP");

    gpio_toggle(GPIOA, GPIO8);
  }

gpio_set(GPIOA, GPIO8);
nvic_disable_irq(NVIC_RTC_IRQ);
    // scb_reset_system();
}


RF24 radio;    // mosi, miso, sck, csn (SSEL), ce, irq

time_t seconds;

uint8_t tt;

static uint32_t slc = 0;

// #define RADIOON 1

bool go_sleep = false; 

void sleepLoop(void) {

  while(1) {
    if(go_sleep) {
      go_sleep = false;

        rtc_awwake();

        // uint32_t bck = getRTC_BKPXR(0);
        // setRTC_BKPXR(0, (nrst ? 0 : bck+1) );
        // setRTC_BKPXR(1, settings.sent_interval );
        // setRTC_BKPXR(2, pr.val[1] );

        rcc_clock_setup_msi(&rcc_clock_config[RCC_CLOCK_VRANGE1_MSI_RAW_131KHZ]);

        wait_ms(2);
        nvic_enable_irq(NVIC_RTC_IRQ);
        wait_ms(2);

        // enter_Standby( );
        enter_Sleep();


    }
    else
      __asm__("NOP");

  }

}


int main(void)
{

	clock_setup();
	gpio_setup();
	usart_setup();
 uint16_t as = adc_setup();  // 0 unsuccessfull

 // if(!as) scb_reset_system();

  // tim_setup();

  // read settings from eeprom
  stm32_flash_read();

  if( !eeprom_SettingsValid) {  // we take the default values
    println("init flash data");
    settings.ID = EEPROM_DATA_ID;
    settings.s_channel = channel;
    settings.pipeWr = pipes[0];
    settings.pipeRd = pipes[1];
    settings.sCrc = _crcLength;
    settings.sDrate = _dataRate;
    settings.sent_interval = sent_interval;

  }
  tracer = 1;
    
  // the input is captured in the ISR
  tinyshell_thread(); 
  // sleepLoop();


    int cc = 10;

#ifdef NOEPD
  setup_ep();
#endif


  // get reset info

  bool nrst = false;
  uint32_t csr =  getRCC_CSR();
  print_s_hex32("reset ? ", csr );

  if(getRTC_BKPXR(2) & 0x1) cc = 0;

  if( csr & RCC_CSR_PINRSTF ) {
    println("reset - NRST ");
    cc = 10;
    nrst = true;
  }
  else if( csr & 0xff000000 )
    println("reset - active ");

  rcc_clear_int();

    println((char *)"This is a stm32 nrf24 - ep  implementation +-+\n");
  // print_s_hex32("RCC ",getRCC_CSR() );

  uint32_t rtcv = getRTC_TR();
  // print_s_hex32("RTC_TR 1 ", rtcv );
  // print_s_hex32("RTC_TSTR ", getRTC_TSTR() );

    while(cc--) {
        gpio_set(GPIOA,GPIO8);
        wait_ms(200);
        gpio_clear(GPIOA,GPIO8);
        wait_ms(200);
    }
        gpio_set(GPIOA,GPIO8);

    // gpio_set(GPIOA,GPIO0);
    // gpio_clear(GPIOA,GPIO0);
    // gpio_set(GPIOA,GPIO0);

// #ifdef RADIOON 
      tracer = 2;

    println((char *)"Start radio ....");

    radio.begin();

    initRadio();
// #endif
   // radio.printDetails();
    
    println((char *)"Start setup done...");

    ps.id[0] = 5;
    for (int i=1; i < 16; i++) {
        ps.id[i] = 0;
    }

    ps.type = 6;
    for (int i=0; i < 7; i++) {
        ps.val[i] = i*i;
    }


    // radio.powerUp();

    println((char *)"power up done  .....--");

    uint8_t timeout=8;
    // gpio_set(GPIOA,GPIO0);
    // gpio_clear(GPIOA,GPIO0);
    // gpio_set(GPIOA,GPIO0);

    settings.sent_interval = getRTC_BKPXR(1);

    while(1) {

      tracer = 3;

// #ifdef RADIOON      
        radio.stopListening();

        ps.val[1] = getRTC_BKPXR(0);

        ps.val[3] = (int16_t) settings.sent_interval;

        radio.powerUp();
// #endif        

        if(!as) println("adc_setup failed");

        if(as) readTempADC();

        wait_ms(200);
        // ps.val[0] = readVREFADC();
        if(as)readVREFADC();
      // println("awake s");
        ps.val[0] = real_VDD;


// #ifdef RADIOON 
        // gpio_clear(GPIOA,GPIO8);
        // Send to hub
        if ( radio.write(&ps, sizeof(ps)) ) {

           // if( !debug ) print((char *)"Send successfull \n\r");
        } else {

           // if( !debug ) print((char *)"Send failed \n\r");
//            break; // we can give up this try and go sleeping
        }
        // gpio_set(GPIOA,GPIO8);

        //wait response


        wait_ms(100);

        radio.startListening();
        // for(int f = 0;f<4; f++) {
        //   uint8_t stat = radio.get_status();
        //   print_s_hex8("status ",stat );
        //   wait_ms(500);
        // }
        wait_ms(300);


        println("listening ..");
          do {
            uint8_t len = radio.getDynamicPayloadSize();

            print_s_decimal("payload size ??? ",len);
            print_s_decimal("read size ", sizeof(pr));

            if( radio.available() ) {
              radio.powerDown();

              radio.read( &pr, sizeof(pr));
              print_s_decimal("val rec 0: ", pr.val[0] );
              if( pr.val[0] < 120 && pr.val[0] > 4 )
                  settings.sent_interval = pr.val[0];

              print_s_decimal("interval 0: ", settings.sent_interval );

              print_s_hex32("val rec 1: 0x", pr.val[1] );


              print_s_decimal("val rec 2: ", pr.val[2] );
              break;
            }
            else
              wait_ms(100);

        } while ( --timeout );
// #endif
print_s_decimal("timeout ",timeout);
        // ?? review  

	rtc_awwake();

  uint32_t bck = getRTC_BKPXR(0);

        if(!timeout) {
            radio.stopListening();
            radio.powerDown();
          }
        else {
          setRTC_BKPXR(0, (nrst ? 0 : bck+1) );
          setRTC_BKPXR(1, settings.sent_interval );
          setRTC_BKPXR(2, pr.val[1] );
        }


  rcc_clock_setup_msi(&rcc_clock_config[RCC_CLOCK_VRANGE1_MSI_RAW_131KHZ]);



        wait_ms(2);
	nvic_enable_irq(NVIC_RTC_IRQ);
        wait_ms(2);

	enter_Standby( );
         tracer = 4;
  // enter_Sleep();
// #endif
      tracer = 5;

  // rcc_clock_setup_msi(&rcc_clock_config[RCC_CLOCK_VRANGE1_MSI_RAW_2MHZ ]);
  // clock_setup();
  // gpio_setup();
  // usart_setup();
println("woke up ");



#ifdef NOEPD
        loop_ep();
#endif        

        // wait(settings.sent_interval);

    }

	return 0;
}


static uint8_t data = 0xff;
static uint8_t char_[0];

void listRadio(void) {
    radio.printDetails();
}

void initRadio(void) {

    println((char *)"init Radio ...");

    // Enable this seems to work better
    radio.enableDynamicPayloads();
    radio.setAutoAck(false);
    radio.setDataRate( (rf24_datarate_e )settings.sDrate);
//  radio.setPALevel(RF24_PA_MAX);
    radio.setChannel(settings.s_channel);
    radio.setRetries(15,15);
    radio.setCRCLength( (rf24_crclength_e )settings.sCrc);


    radio.openWritingPipe(settings.pipeWr);
    radio.openReadingPipe(1,settings.pipeRd);

}


void wait(uint32_t t) {

      wait_ms(1000*t);
}


void wait_ms(uint32_t t) {

        for(uint32_t x = 0;x< 340*t;x++)
        __asm__("NOP");

}



uint8_t *ugetc(void) {
	bool nc = false;


	if( data != 0xff ) {
		*char_ =  data;
		nc = true;
		data = 0xff;
	}

	return (nc) ? char_ : NULL;
	
}

void uputc(uint8_t c) {
	usart_send_blocking(USART1, c);

}


void usart1_isr(void)
{

  // usart_disable_rx_interrupt(USART1);
  nvic_disable_irq(NVIC_USART1_IRQ);


	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_ISR(USART1) & USART_SR_RXNE) != 0)) {

		/* Indicate that we got data. */
		// gpio_toggle(GPIOA, GPIO8);

		/* Retrieve the data from the peripheral. */
		data = usart_recv(USART1);

		tinysh_char_in( data );
		/* Enable transmit interrupt so it sends back the data. */
		// usart_enable_tx_interrupt(USART1);
	}
  nvic_enable_irq(NVIC_USART1_IRQ);

}






////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
/////// TINY SHELL EXAMPLE AS THREAD ///////////////////////////

static  char *cmds [] = {
                        (char *)"Syntax: help",
                        (char *)"Function:",
                        (char *)"  'help'  print this help info",
                        (char *)"  'dbg'  ",
                        };

int nItems = (sizeof(cmds)/sizeof(cmds[0]));



void tinysh_char_out(uint8_t c)
{
    uputc(c);
}


////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
bool debug = true;


void print(char *s)
{

    if(debug) {
        for(unsigned int x =0; x<strlen((char *)s); x++)
            uputc(s[x]);
    }
}

void println(char *s)
{
    print(s);
    print((char *)"\r\n");
}



void tinysh_puts(uint8_t *s)
{

    for(unsigned int x =0; x<strlen((char *)s); x++)
       uputc(s[x]);
}


void print_hex8_(uint8_t v) {
    int i, h;
    for (i = 4; i >= 0; i -= 4) {
        h = (v >> i) & 0x0f;
        if (h < 10) {
            uputc('0' + h);
        } else {
            uputc('A' + h - 10);
        }
    }
}

void print_hex8(uint8_t v) {
    uputc('0');
    uputc('x');

    print_hex8_(v);
}


void print_hex16(uint16_t v) {
    int i, h;
    uputc('0');
    uputc('x');

    for (i = 12; i >= 0; i -= 4) {
        h = (v >> i) & 0x0f;
        if (h < 10) {
            // usart_send_blocking(USART1,'0' + h);
            uputc('0' + h);
        } else {
            // usart_send_blocking(USART1,'A' + h - 10);
            uputc('A' + h - 10);
        }
    }
  uputc('\r');  
  uputc('\n');  
}

void print_hex32(uint32_t v) {
    int i, h;
    uputc('0');
    uputc('x');

    for (i = 28; i >= 0; i -= 4) {
        h = (v >> i) & 0x0f;
        if (h < 10) {
            uputc('0' + h);
        } else {
            uputc('A' + h - 10);
        }
    }

  uputc('\r');  
  uputc('\n');  
}


void print_hex64(uint64_t v) {
    int i, h;
    uputc('0');
    uputc('x');

    for (i = 60; i >= 0; i -= 4) {
        h = (v >> i) & 0x0f;
        if (h < 10) {
            uputc('0' + h);
        } else {
            uputc('A' + h - 10);
        }
    }

  uputc('\r');  
  uputc('\n');  
}

void print_decimal (int32_t i) {
    uint8_t buf[16];
    uint32_t j = 0;

    if (i == 0) {
        // usart_send_blocking(USART1,'0');
        uputc('0');
        return;
    }

    if (i < 0) {
        // usart_send_blocking(USART1,'-');
        uputc('-');
        i *= -1;
    }
    while (i > 0) {
        buf[j++] = '0' + i % 10;
        i /= 10;
    }
    while (j > 0) {
        // usart_send_blocking(USART1,buf[--j]);
        uputc(buf[--j]);
    }

}

void print_decimaln (int32_t i) {

  print_decimaln(i);
  // usart_send_blocking(USART1, '\r');
  // usart_send_blocking(USART1, '\n');
  uputc('\r');  
  uputc('\n');  


}


void print_s_decimal(char *s, int32_t i) {
    print(s);
    print_decimal(i);
    print((char *)"\r\n");

}

void print_s_hex8(char *s, int32_t i) {
    print(s);
    print_hex8(i);
    print((char *)"\r\n");

}

void print_s_hex32(char *s, int32_t i) {
    print(s);
    print_hex32(i);
    print((char *)"\r\n");

}

void print_s_hex64(char *s, int64_t i) {
    print(s);
    print_hex64(i);

}

void print_s_key(char *s, uint8_t *key,uint8_t nSize) {

    print(s);
    for(int i = 0;i<nSize;i++)
      print_hex8_(key[i]);
    println((char *)"");
}

// ascii to hex , single character
int atohc(char c) {

if(c>='A' && c<='F') return (c - 'A' + 10);
if(c>='a' && c<='f') return (c - 'a' + 10);
if(c>='0' && c<='9') return (c - '0');

return 0;
}



// ascii to hex for double character
int atoh(char *c) {
int v = 0;
for(int x=0;x<2;x++) 
  v+= (x==1)  ? atohc(c[x]):atohc(c[x])*16;

return v;
}



#define SETTINGS_WORDS sizeof(settings)/4

void FLASH_Init(void) {
  /* Next commands may be used in SysClock initialization function
     In this case using of FLASH_Init is not obligatorily */
  /* Enable Prefetch Buffer */
  // FLASH_PrefetchBufferCmd( FLASH_PrefetchBuffer_Enable);
  flash_prefetch_enable();
  /* Flash 2 wait state */
  flash_set_ws( FLASH_ACR_LATENCY_1WS );
  // FLASH_SetLatency( FLASH_Latency_2);
}

void FLASH_ReadSettings(void) {
  //Read settings
  uint32_t *source_addr = (uint32_t *)MY_FLASH_PAGE_ADDR;
  uint32_t *dest_addr = (uint32_t *)&settings;
  for (uint16_t i=0; i<SETTINGS_WORDS; i++) {
    *dest_addr = *(uint32_t*)source_addr;
    source_addr++;
    dest_addr++;
  }
}

void FLASH_WriteSettings(void) {
  // FLASH_Unlock();
  // flash_unlock();
  // FLASH_ErasePage(MY_FLASH_PAGE_ADDR);

  // Write settings
  uint32_t *source_addr = (uint32_t *)&settings;
  uint32_t *dest_addr = (uint32_t *) MY_FLASH_PAGE_ADDR;
  for (uint16_t i=0; i<SETTINGS_WORDS; i++) {

    // FLASH_ProgramWord((uint32_t)dest_addr, *source_addr);
    eeprom_program_word((uint32_t)dest_addr, *source_addr);
    source_addr++;
    dest_addr++;
  }
  // flash_lock();
  // FLASH_Lock();
}
// ====================================================


void stm32_flash_read(void) {
  FLASH_Init();
  FLASH_ReadSettings();
  if( settings.ID == EEPROM_DATA_ID ) {
    println((char *)"EEPROM_DATA_ID OK");
    eeprom_SettingsValid = true;
  }
  else {
    println((char *)"EEPROM_DATA_ID NOT OK");
    settings.ID = EEPROM_DATA_ID;
    eeprom_SettingsValid = false;
  }
}


void stm32_flash_write(void) {
  FLASH_Init();
  FLASH_WriteSettings();
}
