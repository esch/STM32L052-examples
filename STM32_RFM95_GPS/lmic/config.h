#ifndef _lmic_config_h_
#define _lmic_config_h_

#define LOWPOWER 1

#define CFG_eu868 1
//#define CFG_us915 1
//#define CFG_sx1272_radio 1
#define CFG_sx1276_radio 1

#define US_PER_OSTICK 20		// To be determined

#define OSTICKS_PER_SEC (1000000 / US_PER_OSTICK)


#define REV1 

#ifdef REV0 
 	#define REV  				"REV 0"
#else 
 	#define REV  				"REV 1"
#endif


#endif // _lmic_config_h_
