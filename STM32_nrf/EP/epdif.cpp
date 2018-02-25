/**
 *  @filename   :   epdif.cpp
 *  @brief      :   Implements EPD interface functions
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
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#include "epdif.h"
#include "stm32_nrf.h"

EpdIf::EpdIf() {
};

// EpdIf::~EpdIf() {
// };

void EpdIf::DigitalWrite(int pin, int value) {
    // digitalWrite(pin, value);
    // we assume that the write is on GPIOA
    if(value)
        gpio_set(GPIOA,pin);
    else
        gpio_clear(GPIOA,pin);

}

int EpdIf::DigitalRead(int pin) {
     return (gpio_get(GPIOA,pin) ? HIGH : LOW );
}

void EpdIf::DelayMs(unsigned int delaytime) {
    // delay(delaytime);
    wait_ms(delaytime);
}

void EpdIf::SpiTransfer(unsigned char data) {
    // digitalWrite(CS_PIN, LOW);
    gpio_clear(EP_PORT_SPI_CSN,EP_PIN_SPI_CSN);

    // SPI.transfer(data);
    spi_send8_sw(SPI1, data);

    for(int i = 0;i< 8;i++)
        __asm__("NOP");

    // digitalWrite(CS_PIN, HIGH);
    gpio_set(EP_PORT_SPI_CSN,EP_PIN_SPI_CSN);
}

int EpdIf::IfInit(void) {
    // pinMode(CS_PIN, OUTPUT);
    // pinMode(RST_PIN, OUTPUT);
    // pinMode(DC_PIN, OUTPUT);
    // pinMode(BUSY_PIN, INPUT);

    gpio_mode_setup(EP_PORT_SPI_CSN, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, EP_PIN_SPI_CSN);    // 
    gpio_mode_setup(EP_PORT_RST, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EP_PIN_RST);    // 
    gpio_mode_setup(EP_PORT_DC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, EP_PIN_DC);    // 
    gpio_mode_setup(EP_PORT_BUSY, GPIO_MODE_INPUT, GPIO_PUPD_NONE, EP_PIN_BUSY);    // 

// SPI
    gpio_mode_setup(EP_PORT_SPI_MOSI, GPIO_MODE_AF, GPIO_PUPD_NONE,     EP_PIN_SPI_MOSI );
    gpio_mode_setup(EP_PORT_SPI_MISO, GPIO_MODE_AF, GPIO_PUPD_PULLUP,  EP_PIN_SPI_MISO );
    gpio_mode_setup(EP_PORT_SPI_CLK,  GPIO_MODE_AF, GPIO_PUPD_PULLUP,  EP_PIN_SPI_CLK  );
    gpio_set_af(EP_PORT_SPI_CLK, GPIO_AF0,    EP_PIN_SPI_CLK | EP_PIN_SPI_MISO | EP_PIN_SPI_MOSI );

    // we have only SPI1 so have to share it with the nrf


    // SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    // SPI.begin();

    return 0;
}

