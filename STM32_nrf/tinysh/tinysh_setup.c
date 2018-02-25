#include <stdint.h>
#include <stdbool.h>
#include "tinysh.h" 
#include "tinysh_setup.h"
#include "nRF24L01P_Maniacbug.h"
#include "stm32_nrf.h"

#include <libopencm3/cm3/scb.h>


unsigned int dbg_runloop = 0;
unsigned int dbg_radio_irq = 0;
unsigned int dbg_failed = 0;

extern int channel;
extern rf24_datarate_e _dataRate;
extern rf24_crclength_e _crcLength;
extern char *strDR[3];
extern char *strCRC[3];
extern uint64_t pipes[2];
extern eeprom_Settings settings;
extern uint16_t tracer;

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


void dng_fnt(int argc, char **argv)
{

  println((char *)"\r\n");

  if( argc>0 )
    println(argv[1]);

  // print_s_decimal((char *)"time [msec]: ",msec);
  // print_s_decimal((char *)"run loop: ",dbg_runloop);
  // print_s_decimal((char *)"radio irqs dbg: ",dbg_radio_irq);
  // print_s_decimal((char *)"assert dbg: ",dbg_failed);

    if( !strcmp(argv[1],"get") ) 
          print_s_decimal((char *)"\r\n debug output level: ",settings.dbg );
    else if( !strcmp(argv[1],"set") ) {
          uint16_t ti = _atoi(argv[2]);
          if(ti > 6 ) ti = 0;
          settings.dbg = ti;
          print_s_decimal((char  *)"setting debug output level ",settings.dbg);
        }
    else {
        print((char *)"unknown command \r\n");   
    }  


}

tinysh_cmd_t mydbgcmd= {0,(char *)"dbg",(char *)"dbg command / [set] dbg level 0 - 6",(char *)"[set]",dng_fnt,0,0,0};


void stm32_flash(int argc, char **argv)
{
    println((char *)"\r\nflash: " );

    if(argc<2) {
        // print("appSkey \r\n");
        return;
    }

    upperCase(argv[1],strlen(argv[1]) );

    if( !strcmp(argv[1],"READ") ) {
          println((char *)"read flash ");
          stm32_flash_read();
    }
    else if( !strcmp(argv[1],"WRITE") ) {
          println((char *)"writing flash ");
          stm32_flash_write();
    }
    else if( !strcmp(argv[1],"PRINT") ) {
          println((char *)"print settings: ");
          print_s_hex64("pipe wr: ", settings.pipeWr);
          print_s_hex64("pipe rd: ", settings.pipeRd);
          print_s_decimal("crc : ", settings.sCrc);
          print_s_decimal("data rate : ", settings.sDrate);
          print_s_decimal("channel : ", settings.s_channel);
          print_s_decimal("time interval : ", settings.sent_interval);
    }
    else {
        print((char *)"unknown command \r\n");   
    }  

}

tinysh_cmd_t cmd_flash= {0,(char *)"flash",(char *)"flash [read/write/print] setting",(char *)"[read/write/print]",stm32_flash,0,0,0};

void smt32_loraTimeInt(int argc, char **argv)
{

    print_s_decimal((char *)"\r\nlora sending time interval: ",settings.sent_interval );

    if(argc<2) {
        // print("appSkey \r\n");
        return;
    }

    upperCase(argv[1],strlen(argv[1]) );

    if( !strcmp(argv[1],"GET") ) 
          print_s_decimal((char *)"\r\nlora sending time interval: ",settings.sent_interval );
    else if( !strcmp(argv[1],"SET") ) {
          uint16_t ti = _atoi(argv[2]);
          if(ti > 2000 || ti < 2) ti = 100;
          settings.sent_interval = ti;
          print_s_decimal((char  *)"setting time interval ",settings.sent_interval);
        }
    else {
        print((char *)"unknown command \r\n");   
    }  


}

tinysh_cmd_t cmd_timeInt = {0,(char *)"sint",(char *)"sent interval [get/set 1000] in sec",(char *)"[get/set]",smt32_loraTimeInt,0,0,0};


void smt32_init(int argc, char **argv)
{

    println((char *)"\r\ninit : " );

    initRadio();
}

tinysh_cmd_t cmd_settingsInit = {0,(char *)"init",(char *)"init settings",(char *)"[-]",smt32_init,0,0,0};


void smt32_list(int argc, char **argv)
{

    println((char *)"\r\nlist : " );

    listRadio();
}

tinysh_cmd_t cmd_list = {0,(char *)"list",(char *)"list settings",(char *)"[-]",smt32_list,0,0,0};


void smt32_channel(int argc, char **argv)
{
    // char _str[1024];

    if(argc<2) {
        print("channel set \r\n");
        return;
    }

    if( !strcmp(argv[1],"get") ) { 
        print_s_decimal("chan: ",channel);
        }
    else if( !strcmp(argv[1],"set") ) {
        if(argc==3) {
        	channel = _atoi( argv[2] );
            settings.s_channel = channel;
        	println(" ");
            // sim->setPinNumber(argv[2]);
        }
        else
            println("no channel \r\n");
        
    }
    else {
        println("unknown command \r\n");   
    }    


}

tinysh_cmd_t cmd_chan = {0,(char *)"chan",(char *)"channel [get/set]",(char *)"[args]",smt32_channel,0,0,0};


void smt32_dataRate(int argc, char **argv)
{
    // char _str[1024];

    if(argc<2) {
        print("data rate  \r\n");
        return;
    }

    if( !strcmp(argv[1],"get") ) { 
        print_s_decimal("\r\ndata rate: ",_dataRate);
        println( strDR[_dataRate] );
        }
    else if( !strcmp(argv[1],"set") ) {
        if(argc==3) {
        	rf24_datarate_e dr = _atoi( argv[2] );
        	if( dr == RF24_1MBPS || dr == RF24_2MBPS || dr == RF24_250KBPS )
        	_dataRate = dr ;
            settings.sDrate = _dataRate;
        }
        else
            println("no data rate \r\n");
        
    }
    else {
        println("unknown command \r\n");   
    }    


}

tinysh_cmd_t cmd_datarate = {0,(char *)"dr",(char *)"data rate [get/set] 0 = 1MBPS; 1 = 2MBPS; 2 = 250KBPS ",(char *)"[args]",smt32_dataRate,0,0,0};


void smt32_crc(int argc, char **argv)
{
    // char _str[1024];

    if(argc<2) {
        print("crc length  \r\n");
        return;
    }

    if( !strcmp(argv[1],"get") ) { 
        print_s_decimal("\r\ncrc length: ",_crcLength);
        println( strCRC[_crcLength] );
        }
    else if( !strcmp(argv[1],"set") ) {
        if(argc==3) {
        	rf24_crclength_e crc = _atoi( argv[2] );
        	if( crc == RF24_CRC_DISABLED || crc == RF24_CRC_8 || crc == RF24_CRC_16 )
        	_crcLength = crc ;
            settings.sCrc = _crcLength;
        	println(" ");
        }
        else
            println("no crc length \r\n");
        
    }
    else {
        println("unknown command \r\n");   
    }    


}

tinysh_cmd_t cmd_crc = {0,(char *)"crc",(char *)"crc length [get/set] 0 = DISABLED; 1 = CRC_8; 2 = CRC_16 ",(char *)"[args]",smt32_crc,0,0,0};


void smt32_pipew(int argc, char **argv)
{
    // char _str[1024];

    if(argc<2) {
        print("pipew  \r\n");
        return;
    }

    if( !strcmp(argv[1],"get") ) { 
        print_s_hex64("\r\npipew: ",pipes[0]  );
        }
    else if( !strcmp(argv[1],"set") ) {
        if(argc==3) {
        	uint64_t tt = tinysh_atoxi(argv[2]);
	        print_s_hex64("\r\npipew: ",tt  );
	        pipes[0] = tt;
            settings.pipeWr = tt;
        }
        else
            println("no pipe settings\r\n");
        
    }
    else {
        println("unknown command \r\n");   
    }    


}

tinysh_cmd_t cmd_pipew = {0,(char *)"pipew",(char *)"pipew [get/set] (write)",(char *)"[args]",smt32_pipew,0,0,0};

void smt32_piper(int argc, char **argv)
{
    // char _str[1024];

    if(argc<2) {
        print("piper  \r\n");
        return;
    }

    if( !strcmp(argv[1],"get") ) { 
        print_s_hex64("\r\npiper: ",pipes[1]  );
        }
    else if( !strcmp(argv[1],"set") ) {
        if(argc==3) {
            uint64_t tt = tinysh_atoxi(argv[2]);
            print_s_hex64("\r\npiper: ",tt  );
            pipes[1] = tt;
            settings.pipeRd = tt;
        }
        else
            println("no pipe settings\r\n");
        
    }
    else {
        println("unknown command \r\n");   
    }    


}

tinysh_cmd_t cmd_piper = {0,(char *)"piper",(char *)"piper [get/set] (read)",(char *)"[args]",smt32_piper,0,0,0};

void smt32_temp(int argc, char **argv)
{

    println((char *)"\r\ntemp : " );

    readTempADC();
}

tinysh_cmd_t cmd_temp = {0,(char *)"temp",(char *)"list temp",(char *)"[-]",smt32_temp,0,0,0};


void smt32_sleep(int argc, char **argv)
{

    println((char *)"\r\nsleep : " );

    // enter_Standby( );
    // enter_LPSleep();
    // enter_Sleep();
}

tinysh_cmd_t cmd_sleep = {0,(char *)"sleep",(char *)"start sleep",(char *)"[-]",smt32_sleep,0,0,0};

void smt32_rst(int argc, char **argv)
{

    println((char *)"\r\nreset : " );

    scb_reset_system();

}

tinysh_cmd_t cmd_rst = {0,(char *)"reset",(char *)"system reset",(char *)"[-]",smt32_rst,0,0,0};


void smt32_tra(int argc, char **argv)
{

    print_s_decimal((char *)"\r\nrunning at : ", tracer );

  

}

tinysh_cmd_t cmd_w = {0,(char *)"tra",(char *)"dbg tracer",(char *)"[-]",smt32_tra,0,0,0};

//void tinyshell_thread(void const *args)
void tinyshell_thread(void)
{
    // pc.baud(115200);
    print((char *)"STM32 tiny shell build (nonrtos) \n\r");
    tinysh_set_prompt((char *)"$ ");
    tinysh_add_command(&mydbgcmd);

    tinysh_add_command(&cmd_flash);

    tinysh_add_command(&cmd_timeInt);
    tinysh_add_command(&cmd_settingsInit);
    tinysh_add_command(&cmd_chan);
    tinysh_add_command(&cmd_datarate);
    tinysh_add_command(&cmd_crc);
    tinysh_add_command(&cmd_pipew);
    tinysh_add_command(&cmd_piper);
    tinysh_add_command(&cmd_list);
    tinysh_add_command(&cmd_temp);
    tinysh_add_command(&cmd_sleep);
    tinysh_add_command(&cmd_rst);
    tinysh_add_command(&cmd_w);

}
