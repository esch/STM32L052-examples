#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "stdarg.h"
#include "stdint.h"

#include "gps.h"
#include "stm32_rfm95_gps.h"

#include <libopencm3/cm3/nvic.h>


int wbufP = 0;
int rbufP = 0;

// char tBuf[256];
char tBuf[512];

int lastStartP = 0;
int lastEndP = 0;

int _len = 0; // internal length
int dbg = 0;

extern int  atoh(char *);
extern void println(char *s);
extern void print_s_decimal(char *s, uint32_t i);
extern uint8_t strt_[512];



void decodeGP(char *gp,char* str,int size);

// $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

// Where:
//      RMC          Recommended Minimum sentence C
//      123519       Fix taken at 12:35:19 UTC
//      A            Status A=active or V=Void.
//      4807.038,N   Latitude 48 deg 07.038' N
//      01131.000,E  Longitude 11 deg 31.000' E
//      022.4        Speed over the ground in knots
//      084.4        Track angle in degrees True
//      230394       Date - 23rd of March 1994
//      003.1,W      Magnetic Variation
//      *6A          The checksum data, always begins with *



// !! string initialization has to be different, else the compiler will assing each string to one addresse

_itemn rmcList[] = {  	{(char *)"TOD"},
						{(char *)"STAT"},
						{(char *)"LAT"},
						{(char *)"LAT"},
						{(char *)"LON"},
						{(char *)"LONGI"},
						{(char *)"SPEED [knots]"},
						{(char *)"track angle"},
						{(char *)"DATE"},
						{(char *)"DUMMY"},
						{(char *)"DUMMY"},
						{(char *)"DUMMY"},
						{(char *)"DUMMY"}
					};



// $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
// $GNGGA,160408.000,4612.7511,N,00556.4940,E,1,09,1.2,633.0,M,0.0,M,,*79
// Where:
//      GGA          Global Positioning System Fix Data
//      123519       Fix taken at 12:35:19 UTC
//      4807.038,N   Latitude 48 deg 07.038' N
//      01131.000,E  Longitude 11 deg 31.000' E
//      1            Fix quality: 0 = invalid
//                                1 = GPS fix (SPS)
//                                2 = DGPS fix
//                                3 = PPS fix
// 			       4 = Real Time Kinematic
// 			       5 = Float RTK
//                                6 = estimated (dead reckoning) (2.3 feature)
// 			       7 = Manual input mode
// 			       8 = Simulation mode
//      08           Number of satellites being tracked
//      0.9          Horizontal dilution of position
//      545.4,M      Altitude, Meters, above mean sea level
//      46.9,M       Height of geoid (mean sea level) above WGS84
//                       ellipsoid
//      (empty field) time in seconds since last DGPS update
//      (empty field) DGPS station ID number
//      *47          the checksum data, always begins with *


// _item ggaList[] = {  	{(char *)"TOD",(char *)"1x-------------------"},
// 						{(char *)"LAT",(char *)"2x-------------------"},
// 						{(char *)"LAT",(char *)"3x-------------------"},
// 						{(char *)"LON",(char *)"4x--------------------"},
// 						{(char *)"LONGI",(char *)"5x-------------------"},
// 						{(char *)"QUAL",(char *)"6x-------------------"},
// 						{(char *)"NUM SATS",(char *)"7x-------------------"},
// 						{(char *)"DIL HOR",(char *)"8x-------------------"},
// 						{(char *)"ALT",(char *)"9x-------------------"},
// 						{(char *)"Meter",(char *)"ax-------------------"},
// 						{(char *)"GEOID",(char *)"bx-------------------"},
// 						{(char *)"Meter",(char *)"cx-------------------"},
// 						{(char *)"DUMMY",(char *)"dx-------------------"},
// 						{(char *)"DUMMY",(char *)"ex-------------------"}
// 					};


_itemn ggaList[] = {  	{(char *)"TOD"},
						{(char *)"LAT"},
						{(char *)"LAT"},
						{(char *)"LON"},
						{(char *)"LONGI"},
						{(char *)"QUAL"},
						{(char *)"NUM SATS"},
						{(char *)"DIL HOR"},
						{(char *)"ALT"},
						{(char *)"Meter"},
						{(char *)"GEOID"},
						{(char *)"Meter"},
						{(char *)"DUMMY"},
						{(char *)"DUMMY"}
					};



     // GSA      Satellite status
     // A        Auto selection of 2D or 3D fix (M = manual) 
     // 3        3D fix - values include: 1 = no fix
     //                                   2 = 2D fix
     //                                   3 = 3D fix
     // 04,05... PRNs of satellites used for fix (space for 12) 
     // 2.5      PDOP (dilution of precision) 
     // 1.3      Horizontal dilution of precision (HDOP) 
     // 2.1      Vertical dilution of precision (VDOP)
     // *39      the checksum data, always begins with *

// char  _gsa[] = "$GPGSA,A,3,15,13,24,17,19,28,18,12,20,,,,1.98,1.01,1.70*08";

_itemn gsaList[] = {  	{(char *)"AUTO SEL"},
						{(char *)"3D FIX"},
						{(char *)"PR1"},
						{(char *)"PR2"},
						{(char *)"PR3"},
						{(char *)"PR4"},
						{(char *)"PR5"},
						{(char *)"PR6"},
						{(char *)"PR7"},
						{(char *)"PR8"},
						{(char *)"PR9"},
						{(char *)"PR10"},
						{(char *)"PR11"},
						{(char *)"PR12"},
						{(char *)"DOP"},
						{(char *)"HDOP"},
						{(char *)"VDOP"},
						{(char *)"DUMMY"},
						{(char *)"DUMMY"},
						{(char *)"DUMMY"},
						{(char *)"DUMMY"}
					};

//   $GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75

// Where:
//       GSV          Satellites in view
//       2            Number of sentences for full data
//       1            sentence 1 of 2
//       08           Number of satellites in view

//       01           Satellite PRN number
//       40           Elevation, degrees
//       083          Azimuth, degrees
//       46           SNR - higher is better
//            for up to 4 satellites per sentence
//       *75          the checksum data, always begins with *

// char  _gsv1[] = "$GPGSV,3,2,10,17,37,085,17,18,25,285,23,19,34,112,38,20,13,218,16*7A"   

_item gsvList[] = {  	{(char *)"# SETS",(char *)"*1------------------"},
						{(char *)"Sent #",(char *)"*2-----------------"},
						{(char *)"SATinVIEW",(char *)"*3-----------------"},
						{(char *)"PRN#",(char *)"*4-----------------"},
						{(char *)"ELEV",(char *)"*5-----------------"},
						{(char *)"AZIM",(char *)"*6-----------------"},
						{(char *)"SNR",(char *)"*7-----------------"},
						{(char *)"PRN#",(char *)"pk------------------"},
						{(char *)"ELEV",(char *)"pl------------------"},
						{(char *)"AZIM",(char *)"-m------------------"},
						{(char *)"SNR",(char *)"--n-----------------"},
						{(char *)"PRN#",(char *)"-o------------------"},
						{(char *)"ELEV",(char *)"-p------------------"},
						{(char *)"AZIM",(char *)"-q------------------"},
						{(char *)"SNR",(char *)"-r------------------"},
						{(char *)"PRN#",(char *)"-s------------------"},
						{(char *)"ELEV",(char *)"-t------------------"},
						{(char *)"AZIM",(char *)"-u------------------"},
						{(char *)"SNR",(char *)"-v------------------"},
						{(char *)"DUMMY",(char *)"-w------------------"},
						{(char *)"DUMMY",(char *)"-x------------------"},
						{(char *)"DUMMY",(char *)"-y------------------"},
						{(char *)"DUMMY",(char *)"-z------------------"}
					};

//   $GPGLL,4916.45,N,12311.12,W,225444,A,*1D

// Where:
//      GLL          Geographic position, Latitude and Longitude
//      4916.46,N    Latitude 49 deg. 16.45 min. North
//      12311.12,W   Longitude 123 deg. 11.12 min. West
//      225444       Fix taken at 22:54:44 UTC
//      A            Data Active or V (void)
//      *iD          checksum data

_itemn gllList[] = {  	{(char *)"LAT"},
						{(char *)"LAT"},
						{(char *)"LON"},
						{(char *)"LONGI"},
						{(char *)"TSTMP"},
						{(char *)"ACTIV"},
						{(char *)"ACTIV"},
						{(char *)"DUMMY"},
						{(char *)"DUMMY"}
					};

// VTG - Velocity made good. The gps receiver may use the LC prefix instead of GP if it is emulating Loran output.

//   $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48

// where:
//         VTG          Track made good and ground speed
//         054.7,T      True track made good (degrees)
//         034.4,M      Magnetic track made good
//         005.5,N      Ground speed, knots
//         010.2,K      Ground speed, Kilometers per hour
//         *48          Checksum


_item vtgList[] = {  	{(char *)"TRK",(char *)"=1-------------------"},
						{(char *)"Unit ",(char *)"=2-------------------"},
						{(char *)"MagTRK",(char *)"=3-------------------"},
						{(char *)"Unit ",(char *)"=4-------------------"},
						{(char *)"GRD SPEED",(char *)"=5-------------------"},
						{(char *)"Unit [n]",(char *)"=6-------------------"},
						{(char *)"GRD SPEED",(char *)"=7-------------------"},
						{(char *)"Unit [km]",(char *)"=8----------"},
						{(char *)"DUMMY",(char *)"=9----------"}
					};

char  *NMEAstr[] = { (char *)"$GNRMC",
					(char *)"$GNGGA",
					(char *)"$GNGSA",
					(char *)"$GNGSV",
					(char *)"$GNGLL",
					(char *)"$GNVTG"
					 };

_itemn *arr[] = { rmcList ,ggaList ,gsaList,gsvList,gllList ,vtgList };


void decodeGP(char *gp,char* str,int size) {

	_itemn *_listP;
	// char tt3[50];
	int  comaP = 0;
	int  startP =  0;
	int  endP =  0;
	int  idx = 0;

	// char strTT[32];

	unsigned int x = 0;

	for( x=0;x<sizeof(NMEAstr)/sizeof(NMEAstr[0]) ;x++) {

		// print("NMEA ");
		// println(NMEAstr[x]);
		if( !strncmp( NMEAstr[x],str,6) )
			break;
	}
		// print_s_decimal("size ",size);

		// print("--NMEA: ");
		// println(str);
		// print_s_decimal("NEMM ",x);
	

// for safety..
	if( x >= sizeof(NMEAstr)/sizeof(NMEAstr[0]) )
		return;

	// print_s_decimal("x ",x);

	_listP = arr[x];
	// print(" _listP ");
	// print_hex32( _listP );

	// print_s_decimal("start ",comaP);

	while(str[comaP++] != ',');
	startP = comaP;
	// print_s_decimal("start ",startP);

	do {
		while(str[comaP] != '*' && str[comaP++] != ',' && comaP < size);

		endP = comaP;
		int _lenx = (str[comaP] == '*') ? endP - startP : endP - startP - 1;

		if( _lenx ) {
			strncpy(_listP[idx].data ,str+startP,_lenx);			

		    _listP[idx].data[_lenx] = '\0';
			// print( _listP[idx].name );
			// print( "  ");
			// println(_listP[idx].data );
		}
		else {
			strcpy(_listP[idx].data,"+");
		}

		// print_s_decimal("start>> ",startP);
		// print_s_decimal("idx>> ",idx);

		idx++;
		startP = endP;

	} while(str[comaP] != '*' && comaP < size);

	// println(_listP[0].data);
	// println(_listP[1].data);
	// println(_listP[2].data);

}


void process_gps() {

  nvic_disable_irq(NVIC_USART2_IRQ);

int lastStartP = 2000;
int endP = 2000;

    char tStr[256];

    int c = 0;

    do {
       if( ! strncmp( strt_+c,"$GNGGA",6) || ! strncmp( strt_+c,"$GNRMC",6) )
          lastStartP = c;

       if( lastStartP != 2000 && strt_[c] == '*')      
          endP = c;
 
      } while( c++ < 450 && endP == 2000 );

   // print_s_decimal("lastStartP ",lastStartP);
   // print_s_decimal("c ",c);

  nvic_enable_irq(NVIC_USART2_IRQ);

    if( lastStartP  != 2000 && endP != 2000 )  {
      strncpy(tStr,strt_+lastStartP, endP - lastStartP + 2); 

      decodeGP("$GNGGA",tStr, endP - lastStartP );

      tStr[lastStartP+ (endP - lastStartP)+3] = 0;   
      // print_s_hex8(tStr,0x01);
    }
 

    // print_s_decimal("\r\ngps ",strPtr );
    // print_s_hex8("\r\nlast char: ",data2 );
    // print_s_hex32("\r\nuart2: ",reg32 );
    // print_s_hex32("dbg CR1: ",dbgCr1 );
    // print_s_hex32("dbg CR3: ",dbgCr3 );
    // print_s_hex32("dbg ISR: ",dbgISR );
    // print_s_hex32("dbg ICR: ",dbgICR );
    
    // print_s_decimal("isr counter ",isrC );
    // println("");
}



void print_gps() {

	_itemn *_listP;

	_listP = arr[0];

	// // print(" _listP ");

	// print_hex32( _listP );

	print(_listP[0].name);
	print(" ");
	println(_listP[0].data);

	print(_listP[1].name);
	print(" ");
	println(_listP[1].data);

	print(_listP[2].name);
	print(" ");
	println(_listP[2].data);

	print(_listP[3].name);
	print(" ");
	println(_listP[3].data);

	print(_listP[4].name);
	print(" ");
	println(_listP[4].data);

	_listP = arr[1];


	print(_listP[0].name);
	print(" ");
	println(_listP[0].data);

	print(_listP[1].name);
	print(" ");
	println(_listP[1].data);

	print(_listP[2].name);
	print(" ");
	println(_listP[2].data);

	print(_listP[3].name);
	print(" ");
	println(_listP[3].data);

	print(_listP[4].name);
	print(" ");
	println(_listP[4].data);

}
