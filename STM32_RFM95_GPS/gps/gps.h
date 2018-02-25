#ifndef GPS_H
#define GPS_H



typedef struct  {
	char *name;
	// char data[32];  // maybe we should assign a fixed size
	char *data;  // maybe we should assign a fixed size
} _item;

typedef struct  {
	char *name;
	char data[32];  // maybe we should assign a fixed size
} _itemn;


void serbridgeUartCb(char *buf, short length);
void decodeGP(char *gp,char* str,int size);
void process_gps();
void print_gps();

#endif