/**
 *  @filename   :   epdif.h
 *  @brief      :   Header file of epdif.cpp providing EPD interface functions
 *                  Users have to implement all the functions in epdif.cpp
 *  @author     :   Yehui from Waveshare
 *
 *  Copyright (C) Waveshare     August 10 2017
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documnetation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to  whom the Software is
 * furished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef EPDIF_H
#define EPDIF_H

#include <libopencm3/stm32/gpio.h>
// #include <arduino.h>

// Pin definition
// #define RST_PIN         8
// #define DC_PIN          9
// #define CS_PIN          10
// #define BUSY_PIN        7


#define EP_PIN_SPI_MOSI               GPIO7      //
#define EP_PORT_SPI_MOSI              GPIOA

#define EP_PIN_SPI_MISO               GPIO6      
#define EP_PORT_SPI_MISO              GPIOA

#define EP_PIN_SPI_CLK               GPIO5      //SCK = PE12
#define EP_PORT_SPI_CLK              GPIOA

#define EP_PIN_SPI_CE                GPIO3     //
#define EP_PORT_SPI_CE               GPIOA

#define EP_PIN_SPI_CSN               GPIO4      //
#define EP_PORT_SPI_CSN              GPIOA

#define EP_PIN_BUSY        			 GPIO1
#define EP_PORT_BUSY       			 GPIOA

#define EP_PIN_RST        			 GPIO2
#define EP_PORT_RST       			 GPIOA

#define EP_PIN_DC        			 GPIO0
#define EP_PORT_DC       			 GPIOA


#define HIGH  1
#define LOW   0

class EpdIf {
public:
    EpdIf(void);
    // ~EpdIf(void);

    static int  IfInit(void);
    static void DigitalWrite(int pin, int value); 
    static int  DigitalRead(int pin);
    static void DelayMs(unsigned int delaytime);
    static void SpiTransfer(unsigned char data);
};

#endif
