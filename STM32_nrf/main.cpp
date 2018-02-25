#include "mbed.h"
#include "nRF24L01P_Maniacbug.h"

//DigitalOut green_led(LED1);
DigitalOut myled1(PA_8);
DigitalOut dbgPin;
DigitalOut csPin(PA_2);

RawSerial uart(PA_9, PA_10); //TX: PA9 ; RX: PA10

void print(char *,int );

const uint64_t pipes[2] = { 0xE8E8F0F0D1LL, 0xE8E8F0F0E1LL };
	
// MOSI: PB5;  MISO: PB4; SCK: PB3; CSN: PA15; CE: PA2
//RF24 radio(PB_5, PB_4, PB_3, PA_15, PA_1);    // mosi, miso, sck, csn (SSEL), ce, irq
RF24 radio(PA_7, PA_6, PA_5, PA_4, PA_3);    // mosi, miso, sck, csn (SSEL), ce, irq


float brightness = 0.0f;
//char txData[TRANSFER_SIZE], rxData[TRANSFER_SIZE] ;
int arrivedcount = 0;
time_t seconds;

typedef struct {
    // id of the device -> max is 32 characters
    char id[16];
    // type of the device -> a simple int
    int16_t type;
    // table of value -> 8 values max
    int16_t val[7];
} Payload;

Payload p;
Payload pr;

int main()
{
    set_time(0);
    seconds = time(NULL);
    int cc = 2;
    char str[64];
			
		dbgPin.init(PA_0);
	
	
   	myled1 = 0;
		dbgPin = 0;

    sprintf(str,"This is a stm32 test...\n");
    print(str,24);

    while(cc--) {
        myled1 = 1;
        wait(.5);
        myled1 = 0;
        wait(.5);
    }
        myled1 = 1;

		
    sprintf(str,"Start radio...\n");
    print(str,24);

		dbgPin = 1;
		dbgPin = 0;

		csPin = 0;


    radio.begin();
exit(0);
    // Enable this seems to work better
    radio.enableDynamicPayloads();
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);
//  radio.setPALevel(RF24_PA_MAX);
    radio.setChannel(88);
    radio.setRetries(15,15);
    radio.setCRCLength(RF24_CRC_8);


    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
//#ifdef LATER

//		radio.printDetails();
		
    sprintf(str,"Start setup done...\n");
    print(str,21);

    p.id[0] = 6;
    for (int i=1; i < 16; i++) {
        p.id[i] = 0;
    }

    p.type = 5;
    for (int i=0; i < 4; i++) {
        p.val[i] = i*i;
    }


    radio.powerUp();

    dbgPin = 1;
		dbgPin = 0;


    sprintf(str,"power up done  ..... \n");
    print(str,21);

    //prepare

    bool timeout=0;
//  uint16_t nodeID = pipes[0] & 0xff;


    // Stop listening and write to radio

    while(1) {
        radio.stopListening();
        wait_ms(15);
p.val[1]++;
        // Send to hub
        if ( radio.write(&p, sizeof(p)) ) {
            sprintf(str, "Send successfull \n\r" );
	//	        myled1 = 0;
		//				wait(.5);
		//        myled1 = 1;

            print(str,20);
        } else {
            sprintf(str, "Send failed \n\r" );
						myled1 = 0;
						wait(.5);
		        myled1 = 1;

					print(str,20);
//            break; // we can give up this try and go sleeping
        }

        //wait response
        radio.startListening();
        wait(10);
        while ( radio.available() && !timeout ) {
            uint8_t len = radio.getDynamicPayloadSize();
            radio.read( &pr, len);
            // ....
            timeout=true; // set it true
            wait_ms(500);
        } // End while

        radio.powerDown();
    }
	//	while(1);
		
}

bool _debug = true;

void print(char *s,int len)
{

    if(_debug) {
        for(int x =0; x<len; x++)
            uart.putc(s[x]);
        uart.putc('\n');
    }
}
