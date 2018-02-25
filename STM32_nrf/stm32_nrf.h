#ifndef _STM32_NRF_
#define _STM32_NRF_


#include "sys/types.h"

#ifdef __cplusplus
    #define EXPORT_C extern "C"
#else
    #define EXPORT_C
#endif

typedef struct
  {
  uint16_t ID;
  uint16_t s_channel;
  uint64_t pipeWr;
  uint64_t pipeRd;
  uint16_t sCrc;
  uint16_t sDrate;
  uint16_t sent_interval;
  uint16_t dbg;  
  } eeprom_Settings;



EXPORT_C void println(char *s);
EXPORT_C void print_s_decimal(char *s, int32_t i);

EXPORT_C void print_decimaln (int32_t i);
EXPORT_C void print_decimal (int32_t i);
EXPORT_C void print_hex32(uint32_t v);
EXPORT_C void print_hex64(uint64_t v);
EXPORT_C void print_hex16(uint16_t v);
EXPORT_C void print_hex8(uint8_t v);
EXPORT_C void print_s_hex8(char *s, int32_t );
EXPORT_C void print_s_hex32(char *s, int32_t );
EXPORT_C void print_s_key(char *s, uint8_t *key,uint8_t );
// EXPORT_C int atohc(char c);
EXPORT_C void print_s_hex64(char *s, int64_t i);
EXPORT_C int atoh(char *c);
EXPORT_C void print_hex8_(uint8_t v);
EXPORT_C uint8_t *ugetc(void);
EXPORT_C void uputc(uint8_t c);
EXPORT_C void print(char *s);
// EXPORT_C void os_getDevEui (u1_t* buf);
EXPORT_C void wait(uint32_t t);
EXPORT_C void wait_ms(uint32_t t);

EXPORT_C void initRadio(void);
EXPORT_C void listRadio(void);


EXPORT_C void stm32_flash_write(void);
EXPORT_C void stm32_flash_read(void);


EXPORT_C void readTempADC(void);
EXPORT_C void enter_Standby( void ) ;
EXPORT_C void enter_LPSleep( void );
EXPORT_C void enter_Sleep( void );

#endif