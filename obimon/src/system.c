/********************************************************************
 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") for its PIC(R) Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *******************************************************************/


#include <system.h>
#include <system_config.h>
#include <usb/usb.h>
#include <libpic30.h>
//#include <Rtcc.h>
#include <leds.h>
#include <stdio.h>
#include <stdlib.h>
#include <app_device_cdc_basic.h>

/** CONFIGURATION Bits **********************************************/

#pragma config DSWDTPS = DSWDTPS3       // DSWDT Postscale Select (1:128 (132 ms))
#pragma config DSWDTOSC = SOSC          // Deep Sleep Watchdog Timer Oscillator Select (DSWDT uses Secondary Oscillator (SOSC))
#pragma config RTCOSC = SOSC            // RTCC Reference Oscillator  Select (RTCC uses Secondary Oscillator (SOSC))
#pragma config DSBOREN = ON             // Deep Sleep BOR Enable bit (BOR enabled in Deep Sleep)
#pragma config DSWDTEN = OFF            // Deep Sleep Watchdog Timer (DSWDT disabled)

// CONFIG3
#pragma config WPFP = WPFP0             // Write Protection Flash Page Segment Boundary (Page 0 (0x0))
#pragma config SOSCSEL = SOSC // Or LPSOSC  // Secondary Oscillator Pin Mode Select (SOSC pins in Default (high drive-strength) Oscillator Mode)
#pragma config WUTSEL = LEG             // Voltage Regulator Wake-up Time Select (Default regulator start-up time used)
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS         // Write Protect Configuration Page Select (Last page and Flash Configuration words are unprotected)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = NONE           // Primary Oscillator Select (Primary Oscillator disabled)
#pragma config I2C1SEL = PRI            // I2C1 Pin Select bit (Use default SCL1/SDA1 pins for I2C1 )
#pragma config IOL1WAY = OFF            // IOLOCK One-Way Set Enable (The IOLOCK bit can be set and cleared using the unlock sequence)
#pragma config OSCIOFNC = ON            // OSCO Pin Configuration (OSCO pin functions as port I/O (RA3))
#pragma config FCKSM = CSECME           // Clock Switching and Fail-Safe Clock Monitor (Sw Enabled, Mon Enabled)

#pragma config FNOSC = FRCPLL           // Initial Oscillator Select (Fast RC Oscillator with Postscaler and PLL module (FRCPLL))
#pragma config PLL96MHZ = ON            // 96MHz PLL Startup Select (96 MHz PLL Startup is enabled automatically on start-up)
//#pragma config FNOSC = FRCDIV           // Initial Oscillator Select (Fast RC Oscillator with Postscaler and PLL module (FRCPLL))
//#pragma config PLL96MHZ = OFF            // 96MHz PLL Startup Select (96 MHz PLL Startup is enabled automatically on start-up)

#pragma config PLLDIV = NODIV           // USB 96 MHz PLL Prescaler Select (Oscillator input used directly (4 MHz input))
#pragma config IESO = OFF               // Internal External Switchover (IESO mode (Two-Speed Start-up) disabled)

// CONFIG1
#pragma config WDTPS = PS1              // Watchdog Timer Postscaler (1:1)
#pragma config FWPSA = PR32             // WDT Prescaler (Prescaler ratio of 1:32)
#pragma config WINDIS = OFF             // Windowed WDT (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer (Watchdog Timer is disabled)
#pragma config ICS = PGx1               // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC1/PGED1)
#pragma config GWRP = OFF               // General Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)


#define READ_LOW_BYTE(v)   ((unsigned char) (v))
#define READ_HIGH_BYTE(v)  ((unsigned char) (((unsigned int) (v)) >> 8))

#define BTLOG 1

unsigned char rxbuf[RXL][RXN];
char printbuf[180];

char *compiledate;

volatile unsigned int writeptr=0;
volatile unsigned int readptr=0;

volatile int rxn=0;
volatile int wptr=0;
volatile int rptr=0;

volatile char rtcintflag = 0;
//volatile char t1intflag = 0;
unsigned long redpattern  = 0;
unsigned long greenpattern= 0;

unsigned int vddraw, vbatraw; 

//unsigned long long uptime=0;
unsigned long long uptime_meas=0;
float last_charge_bat=0; 

char apiversion;
char name[20];
char group[20];
unsigned char hexname[60];
unsigned char hexbuild[60];

//char btversion[60];
BTSTATE btstate=WAIT;
unsigned int btconnected = 0;

char erasing=0;
unsigned long eraseMem=0;
char sleeping=0;

unsigned long G=0;
uint32_t acc=0;
uint32_t maxacc=0;

unsigned char page[256];
unsigned int npage=0;

double vdd=0;
double vbat = 0;
unsigned short nsent=0;

bool opamp = false;

BTRESP btresp=NONE;

//int n_blocks_req=0;
//unsigned int blocks_req[100];

unsigned long sessionid=0;
int measuring = 0;

extern uint16_t SLEEPRATE;
extern int btv;

char tmp[256];

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
void SYSTEM_Initialize( SYSTEM_STATE state )
{
    switch(state)
    {
        case SYSTEM_STATE_USB_START:
            //On the PIC24FJ64GB004 Family of USB microcontrollers, the PLL will not power up and be enabled
            //by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
            //This allows the device to power up at a lower initial operating frequency, which can be
            //advantageous when powered from a source which is not gauranteed to be adequate for 32MHz
            //operation.  On these devices, user firmware needs to manually set the CLKDIV<PLLEN> bit to
            //power up the PLL.
            {
                unsigned int pll_startup_counter = 600;
                CLKDIVbits.PLLEN = 1;
                while(pll_startup_counter--);
            }
        
            //LED_Enable(LED_USB_DEVICE_STATE);
            //BUTTON_Enable(BUTTON_DEVICE_CDC_BASIC_DEMO);
            break;
            
        default:
            break;
    }
}

#if defined(USB_INTERRUPT)
void __attribute__((interrupt,auto_psv)) _USB1Interrupt()
{
    USBDeviceTasks();
}
#endif

void USBStuff() {
        SYSTEM_Tasks();

        #if defined(USB_POLLING)
            // Interrupt or polling method.  If using polling, must call
            // this function periodically.  This function will take care
            // of processing and responding to SETUP transactions
            // (such as during the enumeration process when you first
            // plug in).  USB hosts require that USB devices should accept
            // and process SETUP packets in a timely fashion.  Therefore,
            // when using polling, this function should be called
            // regularly (such as once every 1.8ms or faster** [see
            // inline code comments in usb_device.c for explanation when
            // "or faster" applies])  In most cases, the USBDeviceTasks()
            // function does not take very long to execute (ex: <100
            // instruction cycles) before it returns.
            USBDeviceTasks();
        #endif


        /* If the USB device isn't configured yet, we can't really do anything
         * else since we don't have a host to talk to.  So jump back to the
         * top of the while loop. */
        if( USBGetDeviceState() < CONFIGURED_STATE )
        {
            /* Jump back to the top of the while loop. */
            return;
        }

        /* If we are currently suspended, then we need to see if we need to
         * issue a remote wakeup.  In either case, we shouldn't process any
         * keyboard commands since we aren't currently communicating to the host
         * thus just continue back to the start of the while loop. */
        if( USBIsDeviceSuspended()== true )
        {
            /* Jump back to the top of the while loop. */
            return;
        }

        //Application specific tasks
        APP_DeviceCDCBasicDemoTasks();
}

unsigned long memptr;      // current page to write to the flash

unsigned long long tick=0;
//unsigned long fcy = 0;
unsigned long status = 0;


//void __delay_us(unsigned long d) {
//    __delay32( (unsigned long) ((d)*(fcy)/1000000ULL));
//}

//void __delay_ms(unsigned long d) {
//    __delay32( (unsigned long) ((d)*(fcy)/1000ULL));
//}

void ProcLeds() {
    
    // remove 32768 / DT worth of bits = 128 = 7bits
    //unsigned int shift = tick >> (7+5);
    unsigned int shift = tick >> (7+5);

    shift = shift & 31U; // get 0-31     
    
    int red = 1 - ((redpattern >> shift) &1L);
    int green = 1 - ((greenpattern >> shift) &1L);
    
    LED2_LAT = red;
    LED1_LAT = green;
        
    return;    
}

void CreateNewSession() {
    // session id has to start with most significant bit of 1 (to differentiate from normal GSR data)

    sessionid = rand();    
    sessionid |= 0x80000000; // set MSB
    
    plog("session %lu", sessionid);
}

long long my_atoll(char *instr)
{
  long long retval;

  retval = 0;
  for (; *instr; instr++) {
      char c = *instr;
      if(c<'0' || c>'9') break;
      retval = 10*retval + (c - '0');
  }
  return retval;
}

// bluetooth print
void sendbt(char *s, char rn) {
#ifdef BTLOG
    print("-->");
    if(rn)println(s);
    else print(s);
#endif    
    
    char *r = s;
    while((*r)!=0) {
        while(!U2STAbits.TRMT);
        U2TXREG = *r;
        r++;
    }

    while(!U2STAbits.TRMT);
    
    if(rn) {
        U2TXREG = '\n';
        while(!U2STAbits.TRMT);
        U2TXREG = '\r';    
        while(!U2STAbits.TRMT);
    }

}

void print(char *s) {
    char *r = s;
    while((*r)!=0) {
        while(!U1STAbits.TRMT);
        U1TXREG = *r;
        r++;
    }

    while(!U1STAbits.TRMT);
}

void println(char *s) {
    print(s);
    print("\n");
}

/*
void printtime() {

   GetTimeStamp();

   unsigned char min = timestamp[7];
   unsigned char sec = timestamp[6];

   unsigned char hour = timestamp[4];
   unsigned char month = timestamp[3];
   unsigned char day = timestamp[2];

   unsigned char year = timestamp[0];

   log("y%02xm%02xd%02xh%02xm%02xs%02x", year, month, day, hour, min, sec);
   //log("m%02xs%02x", min, sec);
}
*/

void WakeRN4020() {
    WAKEHW_TRIS = 0;
    WAKEHW = 1;
    //WAKEHW = 0;
    //__delay_ms(100);       
}

void DormantRN4020() {
    WAKEHW_TRIS = 0;
    WAKEHW = 0;
    __delay_ms(1);    
    send("O"); //  Goto Dormant state
    __delay_ms(1);    
}

///////////////////////////// TODO: EXTERNAL LINE
void InitInternalADC() {
    AD1CON1bits.ADON = 0; // disable AD
    AD1CON1bits.ADSIDL = 0; // Continue module operation in Idle mode
    AD1CON1bits.FORM = 0b00; //  Data Output Format: Integer
    AD1CON1bits.SSRC = 0b111; // Internal counter ends sampling and starts conversion (auto-convert)
    AD1CON1bits.ASAM = 0; // Sampling begins when the SAMP bit is set (manual start)

    AD1CON2bits.VCFG = 0b000; // Voltage References are VDD and VSS

    //AD1CON2bits.CSCNA = 0; // Do not scan inputs
    AD1CON2bits.CSCNA = 1; // Scan inputs

    AD1CON2bits.SMPI = 1; // Interrupt after every 2nd samples
    AD1CON2bits.BUFM = 0; // Buffer is configured as one 16-word buffer (ADC1BUFn<15:0>)
    AD1CON2bits.ALTS = 0; // Use only MUXA

    AD1CON3bits.ADRC = 0; // A/D Conversion Clock Source bit, 0 = Clock derived from system clock
    
    // ANDRAS: Had to reduce sampling time because too long sampling caused a voltage drop over the small capacitor
    // (note we have very large resistors on batsense, so it is only the cap that we rely on practically)    
    //AD1CON3bits.SAMC = 0b11111; // Auto-Sample Time bits 11111 = 31 TAD
    AD1CON3bits.SAMC = 0b00111; // Auto-Sample Time bits 11111 = 7 TAD
    
    AD1CON3bits.ADCS = 0b00111111; // A/D Conversion Clock Select bits: 00111111 = 64 TCY
    
    AD1CHSbits.CH0NA = 0; // MUX A negative input is VR-
    AD1CHSbits.CH0SA = 0b01111; // MUXA positive input is VBG

    AD1CHSbits.CH0NB = 0; // MuxB negative input is VR-
    AD1CHSbits.CH0SB = 0b01011; // MUXB positive input is AN11  (RB13)

    
    AD1CSSL = 0; // No inputs are scanned.
    AD1CSSLbits.CSSL15 = 1; // Enable VBG for scanning
    BATSENSE_ADCSCAN = 1; // Enable batsense input for scanning

    AD1PCFGbits.PCFG = 0b1111111111111;      // <12:0>: Analog Input Pin Configuration Control bits(1)
                            // 1 = Pin for corresponding analog channel is configured in Digital mode; I/O port read is enabled
                            // 0 = Pin is configured in Analog mode; I/O port read is disabled, A/D samples pin voltage

    AD1PCFGbits.PCFG15 = 0; //A/D Input Band Gap Reference Enable bit
                            // 0 = Internal band gap reference channel is enabled
    AD1PCFGbits.PCFG14 = 1; // A/D Input Half Band Gap Reference Enable bit (1=disabled)
    AD1PCFGbits.PCFG13 = 1; // A/D Input Voltage Regulator Output Reference Enable bit (1=disabled)
    
    BATSENSE_ADINPUT = 0; // Set batsense input as analog input
    
    IFS0bits.AD1IF = 0; // Clear A/D conversion interrupt.
                        // Configure A/D interrupt priority bits (AD1IP<2:0>) here, if
                        // required. Default priority level is 4.
    IEC0bits.AD1IE = 0; // Disable A/D conversion interrupt
//Delay(); //Ensure the correct sampling time has elapsed
// before starting conversion.
    
    // enabling draws 1mA, so it is only done when conversion is necessary
    // AD1CON1bits.ADON = 1; // enable AD


}

char *private_service = "43974957348793475977654321987654";

char *ch_gsr = "43789734979834798347983479887878";
char *ch_data = "43789734979834798347983479887879";
char *ch_data_req = "4378973497983479834798347988787a";

char handle_gsr[5];

bool RxIdle() {
    return U2STAbits.RIDLE && (rxn==0) && (wptr == rptr);
}

char *GetRxLine() {
    if(rptr == wptr) return NULL;
        
    char *b = (char*)rxbuf[rptr];

#ifdef BTLOG
    print("<--");
    println(b);
#endif
    return b;    
}

void ReleaseRxLine() {
    //log("Release %u %u", rptr, wptr);
    int i = rptr + 1;
    if(i>=RXL) i = 0;
    rptr = i;
}

void FindHandle(char *ch, char *handle) {
    plog("FindHandle");
    
    handle[0]=0;

    char *b;
    while(true) {
        b = GetRxLine();
        if(b==NULL) break;
        
        plog("unexpected %s", b);
        ReleaseRxLine();
    }
    
    send("LS"); //  list services and char
    
    handle[0]=0;

    while(1) {
        b = GetRxLine();
        if(b==NULL) continue;

        if(strncmp(b+2, ch, 32)==0 && strncmp(b+2+32+6, "02", 2)==0) {
            memcpy(handle, b+2+32+1, 4);
            handle[4]=0;
            plog("Char found");
        }

        if(strncmp(b, "END", 3)==0) {
            plog("End found"); 

            ReleaseRxLine();
            
            return ;
        }
        
        ReleaseRxLine();
        
    }
}


void ChangeName(char *n) {
    strncpy(name, n, 8);
    plog("Change name %s", name);
    WriteConfig(CONF_NAME);
    ReadConfig(CONF_NAME);
}

void ChangeGroup(char *n) {    
    strncpy(group, n, 8);
    plog("Change group %s", group);

    WriteConfig(CONF_NAME);
    ReadConfig(CONF_NAME);
    
}

char* WaitLine() {
    char *b;
    unsigned long long t = tick;
    while((b=GetRxLine())==NULL && (tick - t < 32768L*5));
    unsigned long dt = tick - t;
    //log("   dt=%lu", dt);
    
//#ifdef BTLOG
//    if(b!=NULL) plog("<-- (len %i) %s", strlen(b), b);
//#endif
    
    return b;
}


void WaitResp() {
    unsigned long long t = tick;
    btresp = NONE;
    unsigned int n=0;
    
    while(btresp==NONE) {
        
        n++;
        
        unsigned long dt = tick - t;

        __delay_ms(10);
        if((n%100)==0) {
            plog("dt %lu %llu", dt, tick);
        }
        ProcRx();
        
        if(dt>=32768L * 2L) break;
    }
}


//void ProcCharWrite(char *b) {
//    n_blocks_req = 16;
//    
//}

void ProcRx() {
    
    // rptr is last processed line
    // wptr is the last received line    

    while(true) {
        char *b = GetRxLine();
        if(b==NULL) {
            return;
        }
        
//#ifdef BTLOG
//        plog("<-- (len %i) %s", strlen(b), b);
//#endif        
        
        btresp = OTHER;
        
        if(strncmp(b, "AOK", 3)==0) {
            // OK
            btresp = AOK;

        } else if(strncmp(b, "NFail", 5)==0) {
            btresp = NFAIL;

        } else if(strncmp(b, "Connected", 9)==0) {
            btconnected = 1;
        
        } else if(strncmp(b, "Connection End", 14)==0) {
            btconnected = 0;

        } else if(strncmp(b, "WC", 2)==0) {
            // log("RX %s", rxbuf);
            // received config change from other end, e.g., set notification                    
            
        } else if(strncmp(b, "WV", 2)==0) {
            // log("RX %s", rxbuf);
            // received Write
            // ProcCharWrite(b);
            btresp = CHARWRITE;
            
        } else plog("RX");

        ReleaseRxLine();
    }
}

int InitRN4020() {
    
    plog("Wake RN4020");
    WakeRN4020();
    __delay_ms(100);    
    WaitResp();
    __delay_ms(100);
    WaitResp();
    __delay_ms(100);

    //sendbt("-"); //  DEBUG echo
    //WaitResp();
    //if(btresp != AOK) log("ERR 1");

    send("SF,1"); //  reset to the factory default configuration
    __delay_ms(100);

    WaitResp();
    if(btresp != AOK) {
        return 1;
    }
    
    return 0;
    
}

void UpgradeRN() {
    plog("RN4020 upgrade ====");
    WakeRN4020();
    __delay_ms(1000);

    //greenpattern = 0b0000000000000000000000001010101;
    //greenpattern = 0b0001000000100000010000001010101;
    LED_On(RED);
    LED_On(GREEN);

    RN4020OTA();

    int connected = 0;
    int lastcmd = 0;
    
    unsigned long long t = tick;

    while (1) {
        if(connected==0 && tick - t > 32768L * 60L) {
            asm("RESET");            
        }
        
        char *b = WaitLine();
        if (b == NULL) continue;
        
        plog("%s", b);

        if (strncmp(b, "Conn", 3) == 0) {
            connected = 1;
            plog("CONNECTED ----- ");
        }

        if (connected && strcmp(b, "CMD") == 0) {
            plog("Upgrade OK -----------------");
            asm("RESET");
        }
        
        ReleaseRxLine();
        
    }    
}

int ConfRN4020() {
    
    send("SS,C0000001"); //  enable support of the Device Information, Battery and Private services
    WaitResp();
    if(btresp != AOK) {
        return 3;
    }

        
    //sendbt("SR,20000000"); //   set the RN4020 module as a peripheral and auto advertise
    send("SR,00000100"); //  unfiltered observer    
    WaitResp();
    if(btresp != AOK) {
        return 4;
    }
    
    send("PZ"); //  Clean private Service
    WaitResp();
    if(btresp != AOK) {
        return 5;
    }

    send("PS,%s\n", private_service); // set private service UUID
    WaitResp();
    if(btresp != AOK) return 6;

    send("PC,%s,12,05\n",ch_gsr); // set characterictic,readable+notify, 5 bytes
    WaitResp();
    if(btresp != AOK) return 7;
   
    
    send("U"); // unbond?
    WaitResp();
    if(btresp != AOK) {
        plog("Warning Unbind cmd failed");
    }
    
      
    send("Q,1"); // BT bonding statusﬂ
    WaitResp();
    if(btresp != AOK) {
        plog("Warning command failed");
    }
    
    plog("now reboot bt");
    
    send("R,1"); // reboot
    __delay_ms(2000);
    WaitResp();
    WaitResp();

    
    //sendbt("-"); //  DEBUG echo
    //__delay_ms(100);    

    FindHandle(ch_gsr, handle_gsr);
    //println("+"); //  DEBUG echo

    // OK
    return 0;
}

int ConfRN4020_new() {
    plog("ConfRN4020_new");
    
    //sendbt("SS,C0000001"); //  enable support of the Device Information, Battery and Private services
    sendbt("SS,00000001",1); //  enable support of Private services
    WaitResp();
    if(btresp != AOK) {
        return 3;
    }

    sendbt("SR,04000000",1); //   set the RN4020 module as a peripheral and auto advertise
    //sendbt("SR,00000100"); //  unfiltered observer    
    WaitResp();
    if(btresp != AOK) {
        return 4;
    }
    
    sendbt("PZ",1); //  Clean private Service
    WaitResp();
    if(btresp != AOK) {
        return 5;
    }

    send("PS,%s\n", private_service); // set private service UUID
    WaitResp();
    if(btresp != AOK) return 6;

    send("PC,%s,12,05\n",ch_gsr); // set characterictic,readable+notify, 2 bytes
    WaitResp();
    if(btresp != AOK) return 7;
    
    send("PC,%s,10,14\n",ch_data); // set characterictic,notify, 20 bytes
    WaitResp();
    if(btresp != AOK) return 7;

    send("PC,%s,0a,14\n",ch_data_req); // set characterictic, read, write, 20 bytes
    WaitResp();
    if(btresp != AOK) return 7;    
    
    send("U"); // unbond?
    WaitResp();
    if(btresp != AOK) {
        plog("Warning Unbind cmd failed");
    }
    
      
    send("Q,1"); // BT bonding statusﬂ
    WaitResp();
    if(btresp != AOK) {
        plog("Warning command failed");
    }
    
    
    send("S-,OBIMON");    
    WaitResp();
    if(btresp != AOK) {
        plog("Warning command failed");
    }
    
    plog("now reboot bt");
    
    send("R,1"); // reboot
    __delay_ms(2000);
    WaitResp();
    WaitResp();

    
    //sendbt("-"); //  DEBUG echo
    //__delay_ms(100);    

    FindHandle(ch_gsr, handle_gsr);
    //println("+"); //  DEBUG echo

    // OK
    return 0;
}

void SetLedPattern() {    
    
    if(memptr==MEMSIZE) {
        plog("\t\tMemory full!");
        greenpattern = 0b0000000000000000000000001010101;
        redpattern = 0;        
        return;
    }
    
    if(USB_BUS_SENSE==1) {
        if(CHGSTAT==1) {
            plog("BAT fully charged");
            greenpattern = 0xffffffff;
            redpattern = 0;
        } else {
            plog("BAT charging");
            greenpattern = 0;
            redpattern = 0x0000ffff;                    
        }

        return;
      
    } 

    unsigned long p = 0b1;
    
    if(measuring) {
        p = p + (p << 16L);        
    }
        
    if(vbat<3.3) {               
            greenpattern = 0;
            redpattern = p;      
    } else {
        greenpattern = p;
        redpattern = 0;
    }
    
    
    //if(btconnected) {
    //    greenpattern |= 0x00010000LU;
    //    redpattern |= 0x00010000LU;
    //}
}

void WriteConfig(char conf) {
    //char tmp[256]; // WARNING!
    unsigned int confptr = conf*4096; // 4k sectors
    
    switch(conf) {
        case CONF_NAME: {
            plog("WriteConfig NAME");        
            strcpy(tmp, name);
            strcpy(tmp+20, group);
            break;
        }
            
    }    

    allowlower32k();
    waitbusy();

    sectorerase(confptr);
    waitbusy();

    pageprogram(confptr, (unsigned char*)tmp, 256);
    waitbusy();    
    
    protectlower32k();
    waitbusy();
}

// Read all config settings
void ReadConfigAll() {
    plog("a");
    ReadConfig(CONF_NAME);
    
}

void ReadConfig(char conf) {
    //char tmp[256]; // WARNING!
    unsigned int confptr = conf*4096; // 4k sectors

    // Read
    waitbusy();
    readmem(confptr, tmp, 256);
   
    
    switch(conf) {
        case CONF_NAME: {

            if(tmp[0]==255 || strlen(tmp)>8 || strlen(tmp)==0) {
                strcpy((char*)name, "unset");
            } else {
                strcpy((char*)name, tmp);        
            }
            
            if(tmp[20]==255 || strlen(tmp+20)>8 || strlen(tmp+20)==0) {
                strcpy((char*)group, "nogroup");
            } else {
                //plog("l %x %x %x %x %x", tmp[20], tmp[21], tmp[22], tmp[23], tmp[24]);
                strcpy((char*)group, tmp+20);
            }
            
            // plog("N %s", group);
            
            NameToHex();    
            plog("ReadConfig name %s group %s", name, group);
        
            break;
        }
        
    }
    
    
}

void NameToHex() {
    hexname[0]=0;
    
    int i=0;    
    while(name[i]!=0) {
        sprintf(hexname, "%s%02x", (char*)hexname, name[i]);
        i++;            
    }

    sprintf(hexname, "%s00", (char*)hexname);

    i=0;
    while(group[i]!=0) {
        sprintf((char*)hexname, "%s%02x", (char*)hexname, group[i]);
        i++;            
    }    

}

void BatCheck() {    

    ReadVoltage();
        
    if(vdd > 3.0) return;
    
    plog("VDD too low %f", vdd);
    
    int nbt=0;
    
    // histeresis
    while(vdd < 3.2) {
        
        DormantRN4020();
        
        unsigned long long last = tick;
        while(tick - last < 30 * 32768) {
            Sleep();
            if(USB_BUS_SENSE) break;
        }
        WakeRN4020();
        
        LED_On(RED);
        Sleep();
        LED_Off(RED);

        nbt ++;
        
        if((nbt%2)==0) SendCompact();
        else SendSession();

        ReadVoltage();        
    }
    
    asm ("RESET");
}

//unsigned long long last_log_flash = 0;
//void WriteLogFlash() {
//    return; ////////////////////////////////////////////////////////// TEST
//    
//    if(npage != 0) {
//        return;
//    }
//    
//    if(tick - last_log_flash < 32768L * 60) return;
//    last_log_flash = tick;
//    
//    if(memptr >= MEMSIZE) return;
//    
//    plog("WriteLogFlash");
//    
//    memset(page, 0, 256);
//    
//    page[npage++] = 254; // LOG 
//
//    unsigned int uptime_h = (uptime / DT) / 60;
//    unsigned int uptime_meas_h = (uptime_meas / DT) / 60;
//    
//    unsigned int lastb = (unsigned char) (last_charge_bat*10.0);    
//    unsigned int b = (unsigned char) (vbat*10.0);   
//    
//    memcpy(page+npage, (unsigned char*) (&uptime_h), 4);
//    npage += 4;
//
//    memcpy(page+npage, (unsigned char*) (&uptime_meas_h), 4);
//    npage += 4;
//
//    memcpy(page+npage, (unsigned char*) (&lastb), 4);
//    npage += 4;
//
//    memcpy(page+npage, (unsigned char*) (&b), 4);
//    npage += 4;
//        
//    waitbusy();
//    pageprogram(memptr, page, 256); 
//    memptr += 256;
//    npage = 0;
//                    
//}


void BuildToHex() {
    char buf[20];
    sprintf(buf, "%s", CompileDate);
    int i=0;
    hexbuild[0]=0;
    while(buf[i]!=0) {
        sprintf((char*)hexbuild, "%s%02x", (char*)hexbuild, buf[i]);
        i++;            
    }
}

void Adverstise() {
    //send("A,0050,0060"); // advesrtise interval 80ms for 81 sec (so that only sent once)
    send("A,0050,07d0"); // interval 80 msec, duration 2 sec 
}

//void SendTimestamp() {
//    unsigned long long tt = tick/32768;
//    
//    //unsigned long b1 = tick & 0xffffffff;
//    //unsigned long b2 = tick >> 16;
//        
//    send("N,55%04x%016llx", nsent, tt);    
//    nsent++;
//    Adverstise();    
//}

void SetAdvData(char *data) {
    
    if(btv==133) {
        sendbt("Y",1);    
        sendbt("NZ",1);

        if(btconnected) sendbt("NB,FF",0);
        else sendbt("NA,FF",0);

        sendbt(data,1);

    } else {
        sendbt("N,",0);
        sendbt(data,1);
    }
}

void SendSession() {
    plog("SendSession");
    
    char s[20];
    
    sprintf(s,"60%04x%08lx%016llx", nsent, sessionid, tick);
    SetAdvData(s);

    //send("N,60%04x%08lx%016llx", nsent, sessionid, tick);    
    
    nsent++;
    Adverstise();    
}

void SendCompact() {
    plog("SendCompact");
    
    char s[40];
    
    unsigned char b = (unsigned char) (vbat*10.0);
    unsigned char m = (unsigned char) (100 * (memptr-65536) / (MEMSIZE-65536));
    uint16_t ss = (uint16_t) ((tick + TMR1)/1024); // resolution 32Hz, max difference 2048sec
    
    plog("SendCompact %f b:%u m:%u s:%u", vbat, b,m,ss);
    
    sprintf(s,"14%04x%02x%02x%02x%04x%s", nsent, apiversion, b, m, ss, hexname);
    SetAdvData(s);
    
    nsent++;        
    Adverstise();    
}

//void SendBatMem() {    
//    // send battery, memory and tick
//    unsigned int b = (unsigned char) (vbat*10.0);
//    //unsigned long mutil = 100L*memptr/(64L*1024L*1024L);
//                
//    send("N,11%04x%02x%08lx%016llx", nsent, b, memptr, tick + TMR1);
//    nsent++;
//    Adverstise();    
//}

//void SendName() {
//    // send name
//    send("N,12%04x%s", nsent, hexname);    
//    nsent++;        
//    Adverstise();    
//}

void SendBuild() {
    plog("SendBuild");
    
    char s[70];
    // send build date and api version
    
    sprintf(s,"13%04x%s", nsent, hexbuild);    
    SetAdvData(s);
    nsent++;        
    Adverstise();            
}

//void SendUptime() {
//    unsigned int uptime_h = 0; // (uptime / DT) / 60;
//    unsigned int uptime_meas_h = 0; //(uptime_meas / DT) / 60;
//    
//    unsigned int lastb = (unsigned char) (last_charge_bat*10.0);    
//    unsigned int b = (unsigned char) (vbat*10.0);    
//    
//    // send uptime data
//    send("N,15%04x%08x%08x%02x%02x", nsent, uptime_h, uptime_meas_h, lastb, b);    
//    nsent++;        
//    Adverstise();            
//}

void RN4020OTA() {
    LED_On(RED);

    __delay_ms(500);
    
    // send name
    //send("+");    
    //if(WaitLine()==NULL) return;
    //ReleaseRxLine();
    __delay_ms(500);
    send("SF,2");    // factory reset
    if(WaitLine()==NULL) return;
    ReleaseRxLine();
    __delay_ms(5000);
    send("SR,10008000");    
    //send("SR,32008000");
    
    if(WaitLine()==NULL) return;
    ReleaseRxLine();
        
    send("S-,OTA");    
    if(WaitLine()==NULL) return;
    ReleaseRxLine();
    
    __delay_ms(2000);
    send("R,1");    
    //__delay_ms(5000);
    if(WaitLine()==NULL) return;
    ReleaseRxLine();
    if(WaitLine()==NULL) return;
    ReleaseRxLine();
    __delay_ms(5000);    
    
    
//    send("GR");    
//    if(WaitLine()==NULL) return;
//    ReleaseRxLine();
    send("A");    
    __delay_ms(100);
    if(WaitLine()==NULL) return;
    ReleaseRxLine();

    // 001EC041F1DE
    
//    send("D");    
//    __delay_ms(100);
//    if(WaitLine()==NULL) return;
//    ReleaseRxLine();


    plog("RN is in OTA mode");
    //if(WaitLine()==NULL) return;
    //ReleaseRxLine();

}


unsigned nbt=0;
void SendStat() {
    plog("Send stat");
    //log("%s", compiledate);
    
    nbt++;
    if(nbt>=10) nbt=0;
    
    if(nbt == 1) {
        SendBuild();
    } if(nbt == 3 || nbt == 5 || nbt == 7 || nbt == 9) {
        SendSession();
    } else{
        SendCompact();
    }

}

unsigned long long last_gsr_send = 0;
void SendGsr(unsigned long gsr, unsigned char acc) {
    char s[60];
        
    unsigned long d = gsr;
    d |= ((unsigned long)acc)<<24;
    
    if(btconnected) {
        send("SHW,%s,%08lx", handle_gsr, d); 
    }
    
    if(acc > maxacc) maxacc = acc;
        
    if (tick - last_gsr_send >= 32768L / 1L) {
        maxacc=0;
        last_gsr_send = tick;

        sprintf(s,"22%04x%08lx", nsent, d);
        SetAdvData(s);
    
        //send("N,22%04x%08lx", nsent, d);    
        nsent++;    
        Adverstise();
    }
    
}

void ReadVoltage() {
        
    AD1CON1bits.ADON = 1; //Turn on A/D
    __delay_us(500); // wait for voltages to stabilize -- probably the bandgap reference    
    
    IFS0bits.AD1IF = 0; // clear ADC interrupt flag
    AD1CON1bits.ASAM = 1; // auto start sampling for 31Tad then go to conversion
    while (!IFS0bits.AD1IF){}; // conversion done?
    AD1CON1bits.ASAM = 0; // yes then stop sample/convert    

    unsigned int buf1 = ADC1BUF1;
    unsigned int buf0 = ADC1BUF0;
    
    vddraw = buf1;
    vbatraw = buf0;
    
    //plog("buf %u %u", buf0, buf1);
    
    vdd = 1.2 * 1024 / buf1;
    vbat = buf0 / 1024.0 * vdd * 2.0;
    
    // TODO: Strange behavior: Switching off AD also disconnects USB but only on a few devices. Maybe the USB lib uses ADC?
    // Or is the bandgap reference affected somehow?
    if(USB_BUS_SENSE==0) AD1CON1bits.ADON = 0; //Turn off A/D
       
    plog("vdd %f vbat %f", vdd, vbat);

}


/*
void FastClock() {
   // FCY 16Mhz
   // PLL enabled

   //;Place the new oscillator selection in W0
   //;OSCCONH (high byte) Unlock Sequence
   asm("MOV #0x01, w0");

   asm("MOV #OSCCONH, w1");
   asm("MOV #0x78, w2");
   asm("MOV #0x9A, w3");
   asm("MOV.b w2, [w1]");
   asm("MOV.b w3, [w1]");
   //;Set new oscillator selection
   asm("MOV.b WREG, OSCCONH");
   //;OSCCONL (low byte) unlock sequence
   asm("MOV #OSCCONL, w1");
   asm("MOV #0x46, w2");
   asm("MOV #0x57, w3");
   asm("MOV.b w2, [w1]");
   asm("MOV.b w3, [w1]");
   //;Start oscillator switch operation
   asm("BSET OSCCON,#0");

   fcy = 16000000;

   unsigned int pll_startup_counter = 600;
                CLKDIVbits.PLLEN = 1;
                while(pll_startup_counter--);

   InitUART2(9600, 16000000);
   InitSPI(2,2);



}

void SlowClock() {
   // FCY 2Mhz

   //;Place the new oscillator selection in W0
   //;OSCCONH (high byte) Unlock Sequence
   asm("MOV #0x07, w0");

   asm("MOV #OSCCONH, w1");
   asm("MOV #0x78, w2");
   asm("MOV #0x9A, w3");
   asm("MOV.b w2, [w1]");
   asm("MOV.b w3, [w1]");
   //;Set new oscillator selection
   asm("MOV.b WREG, OSCCONH");
   //;OSCCONL (low byte) unlock sequence
   asm("MOV #OSCCONL, w1");
   asm("MOV #0x46, w2");
   asm("MOV #0x57, w3");
   asm("MOV.b w2, [w1]");
   asm("MOV.b w3, [w1]");
   //;Start oscillator switch operation
   asm("BSET OSCCON,#0");

   fcy = 2000000;

   InitUART2(9600, 2000000);
   InitSPI(7,3);
}
*/

/*
void SetRTC() {
   mRtccAlrmDisable(); // need to disable before we can set a new alarm
   RtccWrOn();            		//write enable the rtcc registers
   RtccSetAlarmRpt(RTCC_RPT_TEN_SEC,1);	//Set the alarm repeat to every 10 seconds
   ALCFGRPTbits.CHIME = 0;
   mRtccOn();				//enable the rtcc
   mRtccAlrmEnable();			//enable the rtcc alarm to wake the device up from deep sleep
   mRtccSetInt(1); // enable RTCC interrupt with priority 1

   // Set alarm
   NVMKEY = 0x55;              // 55/AA sequence to enable write to RTCWREN
   NVMKEY = 0xAA;
   RCFGCALbits.RTCWREN = 1;    // enable write to RTC register
   ALCFGRPTbits.ALRMPTR = 3;
   ALRMVAL = 0x0101;
   ALRMVAL = 0x0101;
   ALRMVAL = 0x0101;
   ALRMVAL = 0x0100;
}
*/

void USBOn() {

   U1PWRCbits.USBPWR = 1;
   U1CONbits.USBEN = 1;
   SYSTEM_Initialize(SYSTEM_STATE_USB_START);
   USBDeviceInit();
   USBDeviceAttach();

   // For some reason the USB device wants to reuse the B15 pin for its own use (VBUSST)
   PowerOpamp(true);
}

void USBOff() {

   //Blink(LED1, 5);

   USBDeviceDetach();
   // ANDRAS: DISABLE AND POWER OFF USB MODULE (50 uA)
   U1CONbits.USBEN = 0;
   U1PWRCbits.USBPWR = 0;

}

// UART1 <--> LOG
void InitUART1(unsigned long rate, unsigned long fcy) {
    unsigned long BAUDRATE = rate;
    unsigned long BAUDRATEREG = fcy/4/BAUDRATE-1;

    LOG_TRIS = 0;

    __builtin_write_OSCCONL(OSCCON & 0xbf);
    // Assign UART1TX (code 3) To Pin RP22
    RPOR11bits.RP22R = 3;
    __builtin_write_OSCCONL(OSCCON | 0x40);

    // configure U2MODE
    U1MODEbits.UARTEN = 0;	// Bit15 TX, RX DISABLED, ENABLE at end of func
    U1MODEbits.USIDL = 0;	// Bit13 Continue in Idle
    U1MODEbits.IREN = 0;	// Bit12 No IR translation
    U1MODEbits.RTSMD = 0;	// Bit11 Simplex Mode
    U1MODEbits.UEN = 0;		// Bits8,9 TX,RX enabled, CTS,RTS not
    U1MODEbits.WAKE = 0;	// Bit7 Wake up
    U1MODEbits.LPBACK = 0;	// Bit6 No Loop Back
    U1MODEbits.ABAUD = 0;	// Bit5 No Autobaud (would require sending '55')
    U1MODEbits.RXINV = 0;	// Bit4 IdleState = 1
    U1MODEbits.BRGH = 1;	// Bit3 16 clocks per bit period
    U1MODEbits.PDSEL = 0;	// Bits1,2 8bit, No Parity
    U1MODEbits.STSEL = 0;	// Bit0 One Stop Bit

    U1BRG = BAUDRATEREG;	// baud rate

    U1STA = 0;

    IEC0bits.U1TXIE = 0;	// Disable Transmit Interrupts
    IEC0bits.U1RXIE = 0;	// Disable Recieve Interrupts

    U1MODEbits.UARTEN = 1;	// And turn the peripheral on

    U1STAbits.UTXEN = 1;
}


// UART2 <--> BLUETOOTH
void InitUART2(unsigned long rate, unsigned long fcy) {
    unsigned long BAUDRATE2 = rate;
    unsigned long BAUDRATEREG2 = fcy/4/BAUDRATE2-1;

    //TX_TRIS = 0;

    __builtin_write_OSCCONL(OSCCON & 0xbf);
	// Configure Input Functions **********************
	// Assign UART2RX To Pin RP23
	RPINR19bits.U2RXR = 23;
	// Configure Output Functions *********************
	// Assign UART2TX (code 5) To Pin RP24
	RPOR12bits.RP24R = 5;
	// Lock Registers
	__builtin_write_OSCCONL(OSCCON | 0x40);

	// This is an EXAMPLE, so brutal typing goes into explaining all bit sets

	// The Explorer 16 board has a DB9 connector wired to UART2, so we will
	// be configuring this port only

	// configure U2MODE
	U2MODEbits.UARTEN = 0;	// Bit15 TX, RX DISABLED, ENABLE at end of func
	U2MODEbits.USIDL = 0;	// Bit13 Continue in Idle
	U2MODEbits.IREN = 0;	// Bit12 No IR translation
	U2MODEbits.RTSMD = 0;	// Bit11 Simplex Mode
	U2MODEbits.UEN = 0;		// Bits8,9 TX,RX enabled, CTS,RTS not
	U2MODEbits.WAKE = 1;	// Bit7 Wake up
	U2MODEbits.LPBACK = 0;	// Bit6 No Loop Back
	U2MODEbits.ABAUD = 0;	// Bit5 No Autobaud (would require sending '55')
	U2MODEbits.RXINV = 0;	// Bit4 IdleState = 1
	U2MODEbits.BRGH = 1;	// Bit3 16 clocks per bit period
	U2MODEbits.PDSEL = 0;	// Bits1,2 8bit, No Parity
	U2MODEbits.STSEL = 0;	// Bit0 One Stop Bit

	U2BRG = BAUDRATEREG2;	// baud rate

	// Load all values in for U1STA SFR
	U2STAbits.UTXISEL1 = 0;	//Bit15 Int when Char is transferred (1/2 config!)
	U2STAbits.UTXINV = 0;	//Bit14 N/A, IRDA config
	U2STAbits.UTXISEL0 = 0;	//Bit13 Other half of Bit15
	U2STAbits.UTXBRK = 0;	//Bit11 Disabled
	U2STAbits.UTXEN = 0;	//Bit10 TX pins controlled by periph
	U2STAbits.UTXBF = 0;	//Bit9 *Read Only Bit*
	U2STAbits.TRMT = 0;		//Bit8 *Read Only bit*
	U2STAbits.URXISEL = 0;	//Bits6,7 Int. on character recieved
	U2STAbits.ADDEN = 0;	//Bit5 Address Detect Disabled
	U2STAbits.RIDLE = 0;	//Bit4 *Read Only Bit*
	U2STAbits.PERR = 0;		//Bit3 *Read Only Bit*
	U2STAbits.FERR = 0;		//Bit2 *Read Only Bit*
	U2STAbits.OERR = 0;		//Bit1 *Read Only Bit*
	U2STAbits.URXDA = 0;	//Bit0 *Read Only Bit*

	IFS1bits.U2TXIF = 0;	// Clear the Transmit Interrupt Flag
	IEC1bits.U2TXIE = 1;	// Enable Transmit Interrupts
	IFS1bits.U2RXIF = 0;	// Clear the Recieve Interrupt Flag
	IEC1bits.U2RXIE = 1;	// Enable Recieve Interrupts

	U2MODEbits.UARTEN = 1;	// And turn the peripheral on

	U2STAbits.UTXEN = 1;
}

void InitSPI1_TEST(unsigned char spre, unsigned char ppre, int cke, int ckp) {
    SPI1STATbits.SPIEN = 0;  // disable spi module (spicon registers are only writable if disabled)

    SPI1CON1bits.DISSCK = 0; // CLK enabled
    SPI1CON1bits.DISSDO = 0; // SDO pin is used
    SPI1CON1bits.MODE16 = 0; // 8 BIT MODE
    SPI1CON1bits.SSEN = 0;   // built-in chip select is not used

    SPI1CON1bits.SMP = 0;    // sampled at the middle
    //SPI1CON1bits.CKE = 0;    // MODE 1,1
    //SPI1CON1bits.CKP = 1;    // MODE 1,1
    SPI1CON1bits.CKE = cke;    // MODE x
    SPI1CON1bits.CKP = ckp;    // MODE x 

    SPI1CON1bits.MSTEN = 1;  // master mode
    SPI1CON1bits.SPRE = spre;   // secondary prescale  -- IMPORTANT: CANNOT BE TOO FAST, FLASH DOES NOT WORK PROPERLY
    SPI1CON1bits.PPRE = ppre;   // primary prescale

    SPI1CON2bits.FRMEN = 0;  // framing disabled
    SPI1CON2bits.SPIBEN = 0; // legacy mode

    SPI1STATbits.SPIROV = 0;
    SPI1STATbits.SPISIDL = 1;// module does not work in idle mode

    SPI1STATbits.SPIEN = 1;  // enable spi module

}

void InitSPI1() {
    SPI1STATbits.SPIEN = 0;  // disable spi module (spicon registers are only writable if disabled)

    SPI1CON1bits.DISSCK = 0; // CLK enabled
    SPI1CON1bits.DISSDO = 0; // SDO pin is used
    SPI1CON1bits.MODE16 = 0; // 8 BIT MODE
    SPI1CON1bits.SSEN = 0;   // built-in chip select is not used

    SPI1CON1bits.SMP = 0;    // sampled at the middle
    //SPI1CON1bits.CKE = 0;    // MODE 1,1
    //SPI1CON1bits.CKP = 1;    // MODE 1,1
    SPI1CON1bits.CKE = 0;    // MODE x
    SPI1CON1bits.CKP = 1;    // MODE x 

    SPI1CON1bits.MSTEN = 1;  // master mode
    
    
    // 0 0 is very slow    
    SPI1CON1bits.SPRE = 6;   // secondary prescale  -- IMPORTANT: CANNOT BE TOO FAST, FLASH DOES NOT WORK PROPERLY
    SPI1CON1bits.PPRE = 3;   // primary prescale

    SPI1CON2bits.FRMEN = 0;  // framing disabled
    SPI1CON2bits.SPIBEN = 0; // legacy mode

    SPI1STATbits.SPIROV = 0;
    SPI1STATbits.SPISIDL = 1;// module does not work in idle mode

    SPI1STATbits.SPIEN = 1;  // enable spi module

}

void InitSPI2(unsigned char spre, unsigned char ppre) {
    SPI2STATbits.SPIEN = 0;  // disable spi module (spicon registers are only writable if disabled)

    SPI2CON1bits.DISSCK = 0; // CLK enabled
    SPI2CON1bits.DISSDO = 1; // SDO pin is NOT used
    SPI2CON1bits.MODE16 = 0; // 8 BIT MODE
    SPI2CON1bits.SMP = 0;    // sampled at the middle
    SPI2CON1bits.CKE = 0;    // MODE 1,1
    SPI2CON1bits.SSEN = 0;   // built-in chip select is not used
    SPI2CON1bits.CKP = 1;    // MODE 1,1
    SPI2CON1bits.MSTEN = 1;  // master mode
    SPI2CON1bits.SPRE = spre;   // secondary prescale  -- IMPORTANT: CANNOT BE TOO FAST, FLASH DOES NOT WORK PROPERLY
    SPI2CON1bits.PPRE = ppre;   // primary prescale

    SPI2CON2bits.FRMEN = 0;  // framing disabled
    SPI2CON2bits.SPIBEN = 0; // legacy mode

    SPI2STATbits.SPIROV = 0;
    SPI2STATbits.SPISIDL = 1;// module does not work in idle mode

    SPI2STATbits.SPIEN = 1;  // enable spi module

}

void InitT1() {
    // timer setup
    T1CON = 0x00; //Stops the Timer1 and reset control reg.
    TMR1 = 0x00; //Clear contents of the timer register
    PR1 = 32768 / SLEEPRATE; //Load the Period register
    IPC0bits.T1IP = 7; // Prioty 7 highest

    IFS0bits.T1IF = 0; //Clear the Timer1 interrupt status flag
    IEC0bits.T1IE = 1; //Enable Timer1 interrupts
    T1CONbits.TCS = 1; // external clock source
    T1CONbits.TCKPS = 0b00;
    T1CONbits.TSYNC = 0;
            
    T1CONbits.TON = 1; //Start Timer1 
}

void InitT2() {
    // timer setup
    T2CON = 0x00; //Stops the Timer2 and reset control reg.
    
    T2CONbits.TCS = 0; // internal clock source
    T2CONbits.T32 = 0; // 16bit counter
    T2CONbits.TCKPS = 0b11; // prescaler settings at 1:256
    T2CONbits.TSIDL = 0; // Continue operation in idle mode
    
    TMR2 = 0x00; //Clear contents of the timer register
    PR2 = 32768 ;
    IPC1bits.T2IP = 0x01; //Setup Timer2 interrupt for desired priority level

    IFS0bits.T2IF = 0; //Clear the Timer2 interrupt status flag
    IEC0bits.T2IE = 1; //Enable Timer2 interrupts

    T2CONbits.TON = 1; //Start Timer2
}

void InitPorts() {
    plog("InitPorts");

     AD1PCFG = 0xffff; // all ports digital
     AD1PCFGbits.PCFG11 = 0; // Set AN11 (PORTB13) as analog input

    // SPI config

    __builtin_write_OSCCONL (OSCCON & 0xbf);
    
    // SDI1 is assigned to Flash
    RPINR20bits.SDI1R = 19; // assign SDI1 input to RP19
    RPOR10bits.RP21R = 8;    // assign RP21 to SCK1 (peripheral function 8)
    RPOR10bits.RP20R = 7;    // assign RP20 to SDO1 (peripheral function 7)
    
    // SDI2 is assigned to ADC
    RPINR22bits.SDI2R = 3; // assign SDI1 input to RP3
    RPOR8bits.RP16R = 11;    // assign RP21 to SCK2 (peripheral function 11)
    
    __builtin_write_OSCCONL (OSCCON | 0x40);

    TRISBbits.TRISB2 = 0; // SCK1 output /////////////////////////////////////// CHECK THIS!!!!!!!!!!!

    ADC_CS = 1;
    ADC_CS_TRIS = 0;

    MEM_CS = 1;
    MEM_CS_TRIS = 0;

    LIS_CS = 1;
    LIS_CS_TRIS = 0;

    SDI1_TRIS = 1;
    SDO1_TRIS = 0;

    SDI2_TRIS = 1;
    
    PWROPAMP = 0;
    PWROPAMP_TRIS = 0;

    USB_BUS_SENSE_TRIS = 1;
    
    CHGSTAT_TRIS = 1;
    CHGSTAT_PULLUP = 1;

    BATSENSE_TRIS = 1;
    
    
    TRISAbits.TRISA0 = 1;                // WHAT IS THIS????????????????????

    LED_Enable(LED1);
    LED_Off(LED1);

    LED_Enable(LED2);
    LED_Off(LED2);

}

void PowerOpamp(bool b) {
    plog("Power opamp %u", b);

    opamp = b;
        
    if(b) {
        PWROPAMP_TRIS = 0;
        PWROPAMP = 1;
    } else {
        PWROPAMP_TRIS = 1;
        //PWROPAMP = 0;
    }
}

unsigned long long ts;
/*
unsigned char timestamp[8];
void GetTimeStamp() {
   unsigned int i;
   RCFGCALbits.RTCPTR = 3;

   i = RTCVAL;
   timestamp[0] = READ_LOW_BYTE(i);
   timestamp[1] = READ_HIGH_BYTE(i);

   i = RTCVAL;
   timestamp[2] = READ_LOW_BYTE(i);
   timestamp[3] = READ_HIGH_BYTE(i);

   i = RTCVAL;
   timestamp[4] = READ_LOW_BYTE(i);
   timestamp[5] = READ_HIGH_BYTE(i);

   i = RTCVAL;
   timestamp[6] = READ_LOW_BYTE(i);
   timestamp[7] = READ_HIGH_BYTE(i);
}
*/