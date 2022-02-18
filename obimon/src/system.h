
#ifndef SYSTEM_H
#define SYSTEM_H

#include <p24FJ64GB004.h>
#include <stdbool.h>

//#include <buttons.h>
//#include <leds.h>

#include "mem_s25fl164k.h"

#include <io_mapping.h>
#include <power.h>
#include <stdio.h>


#define MAIN_RETURN int

/*** System States **************************************************/
typedef enum
{
    SYSTEM_STATE_USB_START,
    SYSTEM_STATE_USB_SUSPEND,
    SYSTEM_STATE_USB_RESUME
} SYSTEM_STATE;

extern char printbuf[180];
extern char btbuf[180];

//extern char usblog[];

#define plog(...) {sprintf(printbuf, "LOG "__VA_ARGS__); println(printbuf);}
//#define plog(...) {sprintf(printbuf, "\nLOG "__VA_ARGS__); strcat(usblog, printbuf);}


//#define plog(...) {}

#define send(...) {sprintf(btbuf, __VA_ARGS__); sendbt(btbuf, 1);}

// 8 Meg spansion flash
#define MEMSIZE 8388608L

#define CONF_NAME       0


#define RXN 100
#define RXL 30
extern unsigned char rxbuf[RXL][RXN];
extern volatile int rxn;
extern volatile int wptr, rptr;

extern volatile char rtcintflag;
extern volatile char t1intflag;
extern unsigned long redpattern;
extern unsigned long greenpattern;

typedef enum  {
    READYTOSEND,
    WAIT
} BTSTATE ;


char *GetRxLine();
void ReleaseRxLine();

extern char *compiledate;

extern const char CompileDate[];
extern const char CompileTime[];

extern char apiversion;
extern char name[];
//extern char group[];
extern unsigned char hexname[];

extern int btv;

extern char erasing;
extern unsigned long eraseMem;

extern unsigned short nsent;
extern unsigned long long lastUsb;

extern unsigned char protect[4];
extern unsigned char page[256];
extern unsigned int npage;

extern char btversion[];
extern char sleeping;
extern unsigned int btconnected;


extern double vbat, vdd;

//#define log(...) {};

extern unsigned long status;

extern unsigned long hres;
extern unsigned long G;

//extern unsigned long long uptime;
//extern unsigned long long uptime_meas;
extern float last_charge_bat; 

//extern uint64_t tick;
//extern uint64_t lastSync;
extern int needSync;
extern int role;
//extern unsigned int sessionid;

//extern unsigned long long nextAdjust;
//extern unsigned long long adjust;
//extern int adjustWith;

extern unsigned long memptr;

typedef enum  {
    NONE,
    AOK,
    NFAIL,
    CHARWRITE,
    OTHER
} BTRESP ;

extern BTRESP btresp;

//#define logflash(...) {char s[80]; sprintf(s, __VA_ARGS__); writes(memptr, (unsigned char*)s, 64);memptr += 64;}

/////////////////////////////////////////// LED PORTS
#define LED1_LAT      LATBbits.LATB8
#define LED1_TRIS     TRISBbits.TRISB8

#define LED2_LAT    LATBbits.LATB7
#define LED2_TRIS   TRISBbits.TRISB7


////////////////////////////////////////// MEMORY SPI PORTS
#define SDI1_TRIS       TRISCbits.TRISC3
#define SDI1            PORTCbits.RC3

#define SDO1_TRIS       TRISCbits.TRISC4
#define SDO1            PORTCbits.RC4

#define MEM_CS_TRIS     TRISAbits.TRISA9
#define MEM_CS          PORTAbits.RA9

#define LIS_CS_TRIS     TRISAbits.TRISA8
#define LIS_CS          PORTAbits.RA8

////////////////////////////////////////// ADC SPI PORTS
#define SDI2_TRIS       TRISBbits.TRISB3
#define SDI2            PORTBbits.RB3

#define ADC_CS_TRIS     TRISBbits.TRISB2
#define ADC_CS          LATBbits.LATB2

#define ADC_SCK_TRIS    TRISCbits.TRISC0
#define ADC_SCK         LATCbits.LATC0


////////////////////////////////////////// PWROPAMP
#define PWROPAMP_TRIS  TRISAbits.TRISA1   // powering external op amps

#define PWROPAMP       PORTAbits.RA1

////////////////////////////////////////// WAKE HW RN4020
#define WAKEHW_TRIS   TRISCbits.TRISC9
#define WAKEHW        PORTCbits.RC9

////////////////////////////////////////// CHARGE STATUS
#define CHGSTAT_TRIS   TRISBbits.TRISB13
#define CHGSTAT        PORTBbits.RB13
#define CHGSTAT_PULLUP CNPU1bits.CN13PUE

////////////////////////////////////////// CHARGE CURRENT
#define CHGCUR_TRIS   TRISBbits.TRISB15
#define CHGCUR        PORTBbits.RB15


////////////////////////////////////////// BATSENSE
#define BATSENSE_TRIS   TRISBbits.TRISB14
#define BATSENSE        PORTBbits.RB14
// set batsense as analog input
#define BATSENSE_ADINPUT AD1PCFGbits.PCFG10
// add batsense to adc scanning
#define BATSENSE_ADCSCAN AD1CSSLbits.CSSL10

////////////////////////////////////////// LOG 
#define LOG_TRIS TRISCbits.TRISC6

////////////////////////////////////////// USB SENSE
#define USE_USB_BUS_SENSE_IO 1
#define USB_BUS_SENSE_TRIS TRISBbits.TRISB9
#define USB_BUS_SENSE PORTBbits.RB9

//#define self_power 1


#define FCY 16000000

//extern unsigned int DT;
//#define D 10

//void __delay_us(unsigned long d);
long long my_atoll(char *instr);

void sendbt(char *s, char rn);
void WaitResp();
char* WaitLine();
bool RxIdle();
void ProcRx();
void WaitForTimestamp();

void SendTimestamp();
void SendGsr(unsigned long gsr, unsigned char acc);
void SendLowBat() ;
void SetLedPattern();
void SendStat();
void SendName();

void print(char *s);
void println(char *s);
//void GetTimeStamp();
void PowerFlash(bool b);
void PowerOpamp(bool b);
void InitPorts();
void InitSPI1();
void InitSPI2(unsigned char spre, unsigned char ppre);
void InitT1();
void InitT2();
void InitUART1(unsigned long rate, unsigned long fcy);
void InitUART2(unsigned long rate, unsigned long fcy);
void USBOn();
void USBOff();
void USBStuff();
void SetRTC();
void FastClock();
void SlowClock();

void ChangeName(char *n);
//void ChangeGroup(char *n);

int InitRN4020();
void WakeRN4020();
void DormantRN4020();
void SetCharData();

void InitInternalADC();
void ReadVoltage();
void BatCheck();
void ProcLeds();

void NameToHex();
void WriteConfig(char conf);
void ReadConfig(char conf);
void ReadConfigAll();
void WriteLogFlash();

void CreateNewSession();
void RN4020OTA();

/*********************************************************************
* Function: void SYSTEM_Initialize( SYSTEM_STATE state )
*
* Overview: Initializes the system.
*
* PreCondition: None
*
* Input:  SYSTEM_STATE - the state to initialize the system into
*
* Output: None
*
********************************************************************/
void SYSTEM_Initialize( SYSTEM_STATE state );

/*********************************************************************
* Function: void SYSTEM_Tasks(void)
*
* Overview: Runs system level tasks that keep the system running
*
* PreCondition: System has been initalized with SYSTEM_Initialize()
*
* Input: None
*
* Output: None
*
********************************************************************/
//void SYSTEM_Tasks(void);
#define SYSTEM_Tasks()

#endif //SYSTEM_H
