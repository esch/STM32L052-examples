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
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/flash.h>
// #include <stdio.h>
// #include <string.h>
#include "tinysh.h" 
#include "lmic.h" 

uint8_t *ugetc(void);
void uputc(uint8_t c);
void print(char *s);
void tinyshell_thread(void);
char* itoa(int i, char b[]);

int atohc(char c);

void println(char *s);
void print_s_decimal(char *s, int32_t i);

void print_decimaln (int32_t i);
void print_decimal (int32_t i);
void print_hex32(uint32_t v);
void print_hex16(uint16_t v);
void print_hex8(uint8_t v);
void print_s_hex8(char *s, int32_t );
void print_s_key(char *s, u1_t *key,uint8_t );
int atohc(char c);
int atoh(char *c);
void print_hex8_(uint8_t v);
void dng_fnt(int argc, char **argv);



uint32_t dbg_dosend = 0;

unsigned int dbg_runloop = 0;
unsigned int dbg_radio_irq = 0;
unsigned int dbg_failed = 0;


bool runState = true;
uint16_t sent_interval = 100;  // could be in sec

// ====================================================
// FLASH Settings struct
// ====================================================
// #define MY_FLASH_PAGE_ADDR 0x800FC00

#define MY_FLASH_PAGE_ADDR 0x08080000     // eeprom
#define EEPROM_DATA_ID 0xAA55


typedef struct
  {
  uint16_t ID;
  u1_t   _devAddr[4];
  u1_t   _AppSkey[16];
  u1_t   _NwkSkey[16];
  u1_t   _DEVEUI[8];
  u1_t   _APPEUI[8];

              // 8 byte = 2  32-bits words.  It's - OK
              // !!! Full size (bytes) must be a multiple of 4 !!!
  } eeprom_Settings;

eeprom_Settings settings;

  bool eeprom_SettingsValid = false;
 

// LoRaWAN Application identifier (AppEUI)
// Not used in this example
static  u1_t APPEUI[8]  = { 0x02, 0x00, 0x00, 0x00, 0x00, 0xEE, 0xFF, 0xC0 };

// LoRaWAN DevEUI, unique device ID (LSBF)
// Not used in this example
static  u1_t DEVEUI[8]   = { 0x42, 0x42, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

// LoRaWAN NwkSKey, network session key 
// Use this key for The Things Network
unsigned char NwkSkey[16] =         { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// LoRaWAN AppSKey, application session key
// Use this key to get your data decrypted by The Things Network
unsigned char AppSkey[16] =     { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace

#define msbf4_read(p)   (u4_t)((u4_t)(p)[0]<<24 | (u4_t)(p)[1]<<16 | (p)[2]<<8 | (p)[3])
unsigned char DevAddr[4] = { 0x63, 0x19, 0x06, 0x18 };

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
    memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
    memcpy(buf, NwkSkey, 16);
}

uint8_t mydata[64];
static osjob_t sendjob;


void onEvent (ev_t ev) {
    //debug_event(ev);

    switch(ev) {
      // scheduled data sent (optionally data received)
      // note: this includes the receive window!
      case EV_TXCOMPLETE:
          // use this event to keep track of actual transmissions
          print("EV_TXCOMPLETE, time: \r\n");
          // Serial.println(millis() / 1000);
          if(LMIC.dataLen) { // data received in rx slot after tx
              //debug_buf(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              print("Data Received \r\n");
          }
          break;
       default:
          break;
    }
}

void do_send(osjob_t* j) {

        __asm__("NOP");                                                // XXX delay is added for Serial

    if (LMIC.opmode & OP_TXRXPEND) {
      print("OP_TXRXPEND, not sending \r\n");
      print_hex32(LMIC.opmode);
    } 
    else {
    
      print("ready to send: \r\n");
      strcpy((char *) mydata,"{\"Hello\":\"World\"}"); 
      LMIC_setTxData2(1, mydata, strlen((char *)mydata), 0);
    }
    // Schedule a timed job to run at the given timestamp (absolute system time)
    // os_setTimedCallback(j, os_getTime()+sec2osticks(WAIT_SECS), do_send);

    // defined send interval
    os_setTimedCallback(j, os_getTime()+sec2osticks( sent_interval ), do_send,"do_send");
         
}


char* itoa(int i, char b[])  // Convert Integer to ASCII!!
{
    char const digit[] = "0123456789";
    char* p = b;
    if(i<0){
        *p++ = '-';
        i *= -1;
    }
    int shifter = i;
    do{ //Move to where representation ends
        ++p;
        shifter = shifter/10;
    }while(shifter);
    *p = '\0';
    do{ //Move back, inserting digits as u go
        *--p = digit[i%10];
        i = i/10;
    }while(i);
    return b;
}

void smt32_read_accel(int argc, char **argv);

static void clock_setup(void)
{
	/* Enable GPIOD clock for LED & USARTs. */
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable clocks for USART1. */
	rcc_periph_clock_enable(RCC_USART1);
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


void LMIC_setup(void) {


  print("main 1 .. \r\n");
  // LMIC init
  os_init();
  print("main 2 \r\n");
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  print("main 3 \r\n");
  // Set static session parameters. Instead of dynamically establishing a session 
  // by joining the network, precomputed session parameters are be provided.
  if(eeprom_SettingsValid)
    LMIC_setSession (0x1, msbf4_read(settings._devAddr), (uint8_t*)settings._NwkSkey, (uint8_t*)settings._AppSkey);
  else
    LMIC_setSession (0x1, msbf4_read(DevAddr), (uint8_t*)NwkSkey, (uint8_t*)AppSkey);
  // Disable data rate adaptation
  LMIC_setAdrMode(0);
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // Disable beacon tracking
  LMIC_disableTracking ();
  // Stop listening for downstream data (periodical reception)

  LMIC_stopPingable();
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)

  LMIC_setDrTxpow(DR_SF7,14);


}


uint8_t tt;

extern uint32_t msec;

int main(void)
{
	clock_setup();
	gpio_setup();
	usart_setup();
  println("stm32_rfm95 .. ");

  // read settings from eeprom
  stm32_flash_read();

  // the input is captured in the ISR
  tinyshell_thread(); 

  LMIC_setup();
    
  do_send(&sendjob);

    int cr = 0;
    while(1) {

        if( !(cr++%10) && runState ) {
          // myled1 = myled1 ? 0 : 1;
          os_runloop();
          // cr = 0;
          } 

        for(int x = 0;x< 10;x++)
        __asm__("NOP");
        // wait_ms(50);

    }

	return 0;
}


static uint8_t data = 0xff;
static uint8_t char_[0];


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
char *upperCase(char *c,int size) {

    for(int x=0;x<size;x++) {
        if(c[x] >= 'a' && c[x] <= 'z' ) 
         c[x] -= 0x20;
    }

return c;
}


void smt32_devAddr(int argc, char **argv)
{
      uint16_t kSize = sizeof(DevAddr)/sizeof(DevAddr[0]);  // key size

     print_s_key("\r\nDevAddr: ",DevAddr,kSize );

    if(argc<2) {
        print("DevAddr \r\n");
        return;
    }


    upperCase(argv[1],strlen(argv[1]) );

    if( !strcmp(argv[1],"GET") ) 
            print_s_key("DevAddr: ",DevAddr,kSize );
    else if( !strcmp(argv[1],"SET") ) {
        if(argc>1) {
          println("set DevAddr ");

          if( (strlen(argv[2])>>1) != kSize ) {
              print_s_decimal(" keysize  not correct",(strlen(argv[2])>>1) );
              return;
            }

          print_s_decimal("key ",strlen(argv[2]));

          for(int x = 0; x< strlen(argv[2]) ;x+=2)
              DevAddr[x>>1] = atohc(argv[2][x]) << 4  | atohc(argv[2][x+1]);

          // put it into the settings
          memcpy(&settings._devAddr[0],DevAddr,sizeof(u1_t)*kSize);  
        }
        else
            print("no number \r\n");        
    }
    else {
        print("unknown command \r\n");   
    }  


}

tinysh_cmd_t cmd_devAddr= {0,"devAddr","device address [get/set]","[get/set]",smt32_devAddr,0,0,0};

////////////////////////////////////////////////////////////////

// e.g. 
// 70B3D57EF000396F
// { 0x70, 0xB3, 0xD5, 0x7E, 0xF0, 0x00, 0x39, 0x6F }

void smt32_appeui(int argc, char **argv)
{
  // int nSize = (sizeof(APPEUI)/sizeof(APPEUI[0]));

    uint16_t kSize = sizeof(APPEUI)/sizeof(APPEUI[0]);  // key size

    print_s_key("\r\nappeui: ",APPEUI,kSize );

    if(argc<2) {
        print("appeui \r\n");
        return;
    }


    upperCase(argv[1],strlen(argv[1]) );

    if( !strcmp(argv[1],"GET") ) 
        print_s_key("\r\nappeui: ",APPEUI,kSize );
    else if( !strcmp(argv[1],"SET") ) {

        if(argc>1) {
          println("set APPEUI ");

          if( (strlen(argv[2])>>1) != kSize ) {
              print_s_decimal(" keysize  not correct",(strlen(argv[2])>>1) );
              return;
            }

          print_s_decimal("key ",strlen(argv[2]));

          for(int x = 0; x< strlen(argv[2]) ;x+=2)
              APPEUI[x>>1] = atohc(argv[2][x]) << 4  | atohc(argv[2][x+1]);

          // put it into the settings
          memcpy(&settings._APPEUI[0],APPEUI,sizeof(u1_t)*kSize);  
        }
        else
            print("no number \r\n");        
    }
    else {
        print("unknown command \r\n");   
    }  


}

tinysh_cmd_t cmd_appeui= {0,"appeui","app eui value [get/set]","[get/set]",smt32_appeui,0,0,0};


////////////////////////////////////////////////////////////////


void smt32_deveui(int argc, char **argv)
{

    uint16_t kSize = sizeof(DEVEUI)/sizeof(DEVEUI[0]);  // key size

    print_s_key("\r\ndeveui: ",DEVEUI,kSize );

    if(argc<2) {
        print("deveui \r\n");
        return;
    }


    upperCase(argv[1],strlen(argv[1]) );

    if( !strcmp(argv[1],"GET") ) 
        print_s_key("\r\ndeveui: ",DEVEUI,kSize );
    else if( !strcmp(argv[1],"SET") ) {
        if(argc>1) {
          println("set DEVEUI ");

          if( (strlen(argv[2])>>1) != kSize )
              print_s_decimal(" keysize  not correct",(strlen(argv[2])>>1) );

          print_s_decimal("key ",strlen(argv[2]));

          for(int x = 0; x< strlen(argv[2]) ;x+=2)
              DEVEUI[x>>1] = atohc(argv[2][x]) << 4  | atohc(argv[2][x+1]);

          // put it into the settings
          memcpy(&settings._DEVEUI[0],DEVEUI,sizeof(u1_t)*kSize);  

          print_s_key("new deveui: ",DEVEUI,kSize );
        }
        else
            print("no number \r\n");        
    }
    else {
        print("unknown command \r\n");   
    }  


}

tinysh_cmd_t cmd_deveui= {0,"deveui","device eui [get/set]","[get/set]",smt32_deveui,0,0,0};



void smt32_nwkSkey(int argc, char **argv)
{

    uint16_t kSize = sizeof(NwkSkey)/sizeof(NwkSkey[0]);  // key size

    print_s_key("nwkSkey: ",NwkSkey,kSize );

   if(argc<2) {
        print("nwkSkey \r\n");
        return;
    }


    upperCase(argv[1],strlen(argv[1]) );

    if( !strcmp(argv[1],"GET") ) 
        print_s_key("\r\nnwkSkey: ",NwkSkey,kSize );
    else if( !strcmp(argv[1],"SET") ) {
        if(argc>1) {
          println("set NwkSkey ");

          if( (strlen(argv[2])>>1) != kSize )
              print_s_decimal(" keysize  not correct",(strlen(argv[2])>>1) );

          print_s_decimal("key ",strlen(argv[2]));

          for(int x = 0; x< strlen(argv[2]) ;x+=2)
              NwkSkey[x>>1] = atohc(argv[2][x]) << 4  | atohc(argv[2][x+1]);

                      // put it into the settings
          memcpy(&settings._NwkSkey[0],NwkSkey,sizeof(u1_t)*kSize);  

          print_s_key("new DevAddr: ",NwkSkey,kSize );
        }
        else
            print("no number \r\n");        
    }
    else {
        print("unknown command \r\n");   
    }  


}

tinysh_cmd_t cmd_nwkSkey= {0,"nwkSkey","network session key [get/set]","[get/set]",smt32_nwkSkey,0,0,0};


void smt32_appSkey(int argc, char **argv)
{


    uint16_t kSize = sizeof(AppSkey)/sizeof(AppSkey[0]);  // key size

    print_s_key("\r\nappSkey: ",AppSkey,kSize );

    if(argc<2) {
        print("appSkey \r\n");
        return;
    }


    upperCase(argv[1],strlen(argv[1]) );

    if( !strcmp(argv[1],"GET") ) 
            print_s_key("appSkey: ",AppSkey,kSize );
    else if( !strcmp(argv[1],"SET") ) {
        if(argc>1) {
          println("set appSkey ");

          if( (strlen(argv[2])>>1) != kSize )
              print_s_decimal(" keysize  not correct",(strlen(argv[2])>>1) );

          print_s_decimal("key ",strlen(argv[2]));

          for(uint32_t x = 0; x<strlen(argv[2]);x+=2)
              AppSkey[x>>1] = atohc(argv[2][x]) << 4  | atohc(argv[2][x+1]);

                      // put it into the settings
          memcpy(&settings._AppSkey[0],AppSkey,sizeof(u1_t)*kSize);  

          print_s_key("new appSkey: ",AppSkey,kSize );


        }
        else
            print("no number \r\n");
        
    }
    else {
        print("unknown command \r\n");   
    }  


}

tinysh_cmd_t cmd_appSkey= {0,"appSkey","application session key [get/set]","[get/set]",smt32_appSkey,0,0,0};




void smt32_loraCtrl(int argc, char **argv)
{

    if(runState)
      println("\r\nlora Ctrl: running" );
    else
      println("\r\nlora Ctrl: stopped" );

    if(argc<2) {
        // print("appSkey \r\n");
        return;
    }


    upperCase(argv[1],strlen(argv[1]) );

    if( !strcmp(argv[1],"START") ) {
          println("start lora ");
         runState = true;
       }
    else if( !strcmp(argv[1],"STOP") ) {
        println("stop lora ");
        runState = false;
      }
    else if( !strcmp(argv[1],"INIT") ) {
            println("init \r\n");
            // maybe the restart can be done automatically ??
            if(runState) runState = false;
            LMIC_setup();
            println("lora new init applied, please start it again");          
    }
    else {
        print("unknown command \r\n");   
    }  


}

tinysh_cmd_t cmd_ctrllora = {0,"lora","lora control [start/stop/init]","[start/stop/init]",smt32_loraCtrl,0,0,0};


void smt32_loraFreq(int argc, char **argv)
{

    println("\r\nlora Frequency: " );

    if(argc<2) {
        // print("appSkey \r\n");
        return;
    }


    upperCase(argv[1],strlen(argv[1]) );

    if( !strcmp(argv[1],"FIX") ) 
          println("set freq 1 (868 MHz) ");
    else if( !strcmp(argv[1],"RND") )
          println("stop lora ");
    else {
        print("unknown command \r\n");   
    }  


}

tinysh_cmd_t cmd_freq_lora = {0,"f_lora","set lora frequency [fix 1...7/random]","[fix/rnd]",smt32_loraFreq,0,0,0};


void smt32_loraTimeInt(int argc, char **argv)
{

    print_s_decimal("\r\nlora sending time interval: ",sent_interval );

    if(argc<2) {
        // print("appSkey \r\n");
        return;
    }


    upperCase(argv[1],strlen(argv[1]) );

    if( !strcmp(argv[1],"GET") ) 
          print_s_decimal("\r\nlora sending time interval: ",sent_interval );
    else if( !strcmp(argv[1],"SET") ) {
          sent_interval = _atoi(argv[2]);
          print_s_decimal("setting time interval ",sent_interval);
        }
    else {
        print("unknown command \r\n");   
    }  


}

tinysh_cmd_t cmd_timeInt = {0,"tint","time interval [get/set 100] in sec","[get/set]",smt32_loraTimeInt,0,0,0};



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


void dng_fnt(int argc, char **argv)
{

  println("\r\n");

  if( argc>0 )
    println(argv[1]);

  print_s_decimal("time [msec]: ",msec);
  print_s_decimal("run loop: ",dbg_runloop);
  print_s_decimal("radio irqs dbg: ",dbg_radio_irq);
  print_s_decimal("assert dbg: ",dbg_failed);

}

tinysh_cmd_t mydbgcmd= {0,"dbg","dbg command / [set] dbg level 0 - 6","[set]",dng_fnt,0,0,0};


void stm32_flash(int argc, char **argv)
{

    println("\r\nflash: " );

    if(argc<2) {
        // print("appSkey \r\n");
        return;
    }


    upperCase(argv[1],strlen(argv[1]) );

    if( !strcmp(argv[1],"READ") ) {
          println("read flash ");
          stm32_flash_read();
    }
    else if( !strcmp(argv[1],"WRITE") ) {
          println("write flash ");
          stm32_flash_write();
    }
    else if( !strcmp(argv[1],"PRINT") ) {
          println("print settings: ");
          // print_s_hex8("para 1: ", settings.Parameter1);
          // print_s_hex8("para 2: ", settings.Parameter2);

          // print_s_hex32("para 4: ", settings.Parameter4);
          print_s_key("set DevAddr: ",settings._devAddr,(sizeof(DevAddr)/sizeof(DevAddr[0])) );
          print_s_key("set AppSKey: ",settings._AppSkey,(sizeof(AppSkey)/sizeof(AppSkey[0])) );
          print_s_key("set NwkSkey: ",settings._NwkSkey,(sizeof(NwkSkey)/sizeof(NwkSkey[0])) );
          print_s_key("set DEVEUI : ",settings._DEVEUI,(sizeof(DEVEUI)/sizeof(DEVEUI[0])) );
          print_s_key("set APPEUI : ",settings._APPEUI,(sizeof(APPEUI)/sizeof(APPEUI[0])) );
    }
    else {
        print("unknown command \r\n");   
    }  

}

tinysh_cmd_t cmd_flash= {0,"flash","flash [read/write/print] setting","[read/write/print]",stm32_flash,0,0,0};


void tinysh_char_out(uint8_t c)
{
    uputc(c);
}

//void tinyshell_thread(void const *args)
void tinyshell_thread(void)
{
    // pc.baud(115200);
    print("STM32 tiny shell build (nonrtos) \n\r");
    tinysh_set_prompt("$ ");
    tinysh_add_command(&mydbgcmd);

    tinysh_add_command(&cmd_ctrllora);
    tinysh_add_command(&cmd_flash);

    tinysh_add_command(&cmd_freq_lora);
    tinysh_add_command(&cmd_timeInt);


    

    tinysh_add_command(&cmd_devAddr);

    tinysh_add_command(&cmd_deveui);
    tinysh_add_command(&cmd_appeui);

    tinysh_add_command(&cmd_nwkSkey);
    tinysh_add_command(&cmd_appSkey);

}

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
bool debug = true;

// void print_d(char *s)
// {

//     if(debug) {
//         for(unsigned int x =0; x<strlen((uint8_t *)s); x++)
//             uputc(s[x]);
//     }
// }


void print(char *s)
{

    if(debug) {
        for(unsigned int x =0; x<strlen((uint8_t *)s); x++)
            uputc(s[x]);
    }
}

void println(char *s)
{
    print(s);
    print("\r\n");
}



void tinysh_puts(uint8_t *s)
{

    for(unsigned int x =0; x<strlen(s); x++)
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
            usart_send_blocking(USART1,'0' + h);
        } else {
            usart_send_blocking(USART1,'A' + h - 10);
        }
    }
  usart_send_blocking(USART1, '\r');
  usart_send_blocking(USART1, '\n');
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

void print_decimal (int32_t i) {
    uint8_t buf[16];
    uint32_t j = 0;

    if (i == 0) {
        usart_send_blocking(USART1,'0');
        return;
    }

    if (i < 0) {
        usart_send_blocking(USART1,'-');
        i *= -1;
    }
    while (i > 0) {
        buf[j++] = '0' + i % 10;
        i /= 10;
    }
    while (j > 0) {
        usart_send_blocking(USART1,buf[--j]);
    }

}

void print_decimaln (int32_t i) {

  print_decimaln(i);
  usart_send_blocking(USART1, '\r');
  usart_send_blocking(USART1, '\n');

}


void print_s_decimal(char *s, int32_t i) {
    print(s);
    print_decimal(i);
    print("\r\n");

}

void print_s_hex8(char *s, int32_t i) {
    print(s);
    print_hex8(i);
    print("\r\n");

}

void print_s_hex32(char *s, int32_t i) {
    print(s);
    print_hex32(i);
    print("\r\n");

}


void print_s_key(char *s, u1_t *key,uint8_t nSize) {

    print(s);
    for(int i = 0;i<nSize;i++)
      print_hex8_(key[i]);
    println("");
}

// ascii to hex , single character
int atohc(char c) {

if(c>='A' && c<='F') return (c - 'A' + 10);
if(c>='a' && c<='f') return (c - 'a' + 10);
if(c>='0' && c<='9') return (c - '0');

return 0;
}


int _atoi(char *str)
{
    int res = 0; // Initialize result
  
    // Iterate through all characters of input string and
    // update result
    for (int i = 0; str[i] != '\0'; ++i)
        res = res*10 + str[i] - '0';
  
    // return result.
    return res;
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
  uint32_t *dest_addr = (void *)&settings;
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
  uint32_t *source_addr = (void *)&settings;
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
    println("EEPROM_DATA_ID OK");
    eeprom_SettingsValid = true;
  }
  else {
    println("EEPROM_DATA_ID NOT OK");
    settings.ID = EEPROM_DATA_ID;
    eeprom_SettingsValid = false;
  }
}


void stm32_flash_write(void) {
  FLASH_Init();
  FLASH_WriteSettings();
}
