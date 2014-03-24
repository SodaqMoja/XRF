#ifndef PINDEFS_H
#define PINDEFS_H

#define SODAQ_VARIANT_V2        2
#define SODAQ_VARIANT_MOJA      3

#if defined(__AVR_ATmega32U4__)
#define SODAQ_VARIANT   SODAQ_VARIANT_V2
#elif defined(__AVR_ATmega328P__)
#define SODAQ_VARIANT   SODAQ_VARIANT_MOJA
#endif

#if SODAQ_VARIANT == SODAQ_VARIANT_V2
//#########   pin definitions SODAQ V2   ########

#define RAINPIN1        4
#define RAINPIN2        5
#define RAINPIN3        13

#define SOLARPIN        A0

#define DF_MOSI         16
#define DF_MISO         14
#define DF_SPICLOCK     15
#define DF_SLAVESELECT  12

#define BATVOLTPIN      A5
#define BATVOLT_R1      10              // in fact 10M
#define BATVOLT_R2      2               // in fact 2M
#define BATSTATPIN      A4

#define XBEEDTR_PIN     8
#define XBEECTS_PIN     9

#define GROVEPWR_PIN    6
#define GROVEPWR_OFF    HIGH
#define GROVEPWR_ON     OFF

#define DIAGPORT_RX     10
#define DIAGPORT_TX     11

#define BEEPORT         Serial1

#elif SODAQ_VARIANT == SODAQ_VARIANT_MOJA
//#########   pin definitions SODAQ Moja   ########

#define RAINPIN1        A0 //2
#define RAINPIN2        A1 //3

#define WINDTICKPIN      3
#define WINDDIR_RAW_MAX 1023
#define WINDDIR_RAW_MIN 0
#define WINDDIRPIN      A2

#define SOLARPIN        A3

#define DF_MOSI         11
#define DF_MISO         12
#define DF_SPICLOCK     13
#define DF_SLAVESELECT  10

#define BATVOLTPIN      A7
#define BATVOLT_R1      10              // in fact 10M
#define BATVOLT_R2      2               // in fact 2M

#define XBEEDTR_PIN     7
#define XBEECTS_PIN     8

#define GROVEPWR_PIN    6
#define GROVEPWR_OFF    LOW
#define GROVEPWR_ON     HIGH

// Only needed if DIAG is enabled
#define DIAGPORT_RX     4       // PD4 Note. No interrupt. Cannot be used for input
#define DIAGPORT_TX     5       // PD5

#define BEEPORT         Serial

#else
#error "Unknown SODAQ_VARIANT"
#endif

#endif // PINDEFS_H
