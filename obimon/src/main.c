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

// ANDRAS Need to add hid_boot_p24fj64gb0004.gld to "Linker files "-- this is linked from the bootloader project


/** INCLUDES *******************************************************/
#include "system.h"
#include <system_config.h>

#include <app_device_cdc_basic.h>
//#include <app_led_usb_status.h>

#include <usb/usb.h>
#include <usb/usb_device.h>
#include <usb/usb_device_cdc.h>
#include <stdio.h>
#include <stdlib.h>

//#include "mem_sst25vf032b.h"
#include "mem_s25fl164k.h"

//#include <Rtcc.h>
#include "adc.h"
#include "leds.h"
#include <libpic30.h>
#include <p24FJ64GB004.h>

#include <math.h>

////////////////////////////////////////////////////////////////////////////// TODO


// 12pf HIGH    24ppm     
// 15pf HIGH    17ppm
// 22pf HIGH    3.9ppm

const char __attribute__((space(prog), address(0x2000))) CCC[] = __DATE__;

const char CompileDate[] =__DATE__ ;
const char CompileTime[]=__TIME__;
 
int dump=0;
unsigned long dumpaddr=0;

void SleepObi();





char is=0;

unsigned long long nextAdjust = 0;
unsigned long long adjust = 0; //96590;
int adjustWith = 0; //1;

void __attribute__((interrupt,auto_psv)) _T1Interrupt(void)
{
   IFS0bits.T1IF = 0; //Reset Timer1 interrupt flag
      
   tick += PR1+1; // update time first
   
   unsigned long long remainder = tick % (32768 / DT);

   PR1 = 32768 / DT - 1;
   if(remainder>0) PR1 += 32768/DT - remainder;

   if(tick>nextAdjust && adjust) {
      while(TMR1==0);
      
      TMR1 += adjustWith;
      nextAdjust += adjust;
      
      plog("ADJ");
       
   }
   
   //unsigned int tmr1 = TMR1;
   // log("%u", tmr1);

   //log("R %llu %u", remainder, PR1);

   //ProcLeds();
   
   uptime++;
   
   t1intflag++;
}

void __attribute__((interrupt,auto_psv)) _T2Interrupt(void)
{
   IFS0bits.T2IF = 0; //Reset Timer2 interrupt flag
   
   LED_Toggle(RED);
   LED_Toggle(GREEN);
   plog("T");

}


void __attribute__((interrupt,auto_psv)) _RTCCInterrupt(void)
 {
     
     IFS3bits.RTCIF = 0;

     ALCFGRPTbits.ALRMEN = 1;   // enable alarm (needs to be reset after alarm)
     RCFGCALbits.RTCEN = 1;

     rtcintflag=1;
 }

void __attribute__ ((interrupt, no_auto_psv)) _U2RXInterrupt(void) {
    is+=1;
    
    if(U2STAbits.OERR) {
        //log("OERR");
        U2STAbits.OERR = 0;
    }
    if(U2STAbits.FERR) {
        //log("FERR");
        U2STAbits.FERR = 0;
    }
    if(U2STAbits.PERR) {
        //log("PERR");
        U2STAbits.PERR = 0;
    }
    
    while(U2STAbits.URXDA) {
        char b = U2RXREG;
               
        if(b=='\n' || b=='\r' || b==0) {
            if(rxn>0) {
                
                rxbuf[wptr][rxn]=0;
                
                int newwptr = wptr + 1;
                
                if(newwptr >= RXL) {
                    newwptr = 0;
                }
                
                if(newwptr == rptr) {
                    plog("rxl overflow");                    
                } else wptr = newwptr;
                                  
            }

            rxn = 0;

            continue;
        }

        rxbuf[wptr][rxn] = b;
        rxn++;

        if(rxn>=RXN) {
            rxbuf[wptr][rxn-1]=0;
            plog("rxn overflow");
            rxn=0;
        }
    }
    
    IFS1bits.U2RXIF = 0;

}

void __attribute__ ((interrupt, no_auto_psv)) _U2TXInterrupt(void) {
	IFS1bits.U2TXIF = 0;
    
}

void __attribute__ ((interrupt, no_auto_psv)) _ADC1Interrupt(void)
{
    plog("WTF");
}





int randoffset=0;
int randshift = 0;
unsigned long CalcGsr(unsigned long h) {
    
    if(role == ROLE_TEST) {
        
        if(randoffset == 0) {
            randoffset = rand() % 32768;
            randshift = rand() % 10000;
        }
        
        float d;
        unsigned long msec = (unsigned long) (tick % 327680LLU);
        
        //plog("msec %lu", msec);
        float t = 1.0 * (msec+randoffset) / 32768.0 * 2.0 * 3.1415 / 10.0; 
        
        d = sinf(t) * 1000.0 + 2000 + randshift;
        
        //plog("TEST %f", t);
        
        //plog("D %f", d);
        
        return (unsigned long)d;
    }
    
    double alpha;
    alpha = 100.0/(100+680.0);

    double R = 100e3;
    //double R = 100e3;

    double A = 1.0*h / 2097152.0;
    double gsr = A / alpha / R * 1e6;

    if(gsr>140) gsr=0;

    return (unsigned long)(gsr*1000);
}

void printstatusreg() {
    int s1 = readstatus();
    int s2 = readstatus2();
    int s3 = readstatus3();

    if(s1 & 1) plog("BUSY");
    if(s1 & 2) plog("WEL");
    if(s1 & 4) plog("BP0");
    if(s1 & 8) plog("BP2");
    if(s1 & 16) plog("BP3");
    if(s1 & 32) plog("TB protect from bottom");
    if(s1 & 64) plog("4kB protection") else plog("64kB protection")
    if(s1 & 128) plog("SRP0=1 WP input can protect SR or OTP lockdown") else plog("SRP0=0 WP input has no effect or power supply lock down mode");
    
    if(s2 & 1) plog("SRP1=1 WP input protects status register") else  plog("SRP1=0 Power Supply Lock Down or OTP Lock Down");
    if(s2 & 2) plog("QE");
    if(s2 & 4) plog("LB0");
    if(s2 & 8) plog("LB1");
    if(s2 & 16) plog("LB2");
    if(s2 & 32) plog("LB3");
    if(s2 & 64) plog("CMP Inverted protection map");
    if(s2 & 128) plog("SUS Erase/Program suspended");

    if(s3 & 1) plog("LC0");
    if(s3 & 2) plog("LC0");
    if(s3 & 4) plog("LC0");
    if(s3 & 8) plog("LC0");
    if(s3 & 16) plog("Burst wrap disabled") else plog("Burst wrap enabled");
    if(s3 & 32) plog("W4");
    if(s3 & 64) plog("W5");
    
}

void testflash() {
    unsigned long i, j;
    unsigned char data[16];

    plog("TEST FLASH ==============================");

    printstatusreg();
    
   // plog("Erase flash");
    //eraseDevice();
    //plog("Erase flash END");
    //while(1);
    
    for(i=65536; i<MEMSIZE; i+=65536L * 8L) {
        plog("---- WRITE %lu", i);
        memcpy(data, &j, 4);
        //data[0]=1;
        //data[1]=2;
        //data[2]=3;
        //data[3]=4;
        pageprogram(i, data, 4);
        //plog("WRITE %x %x %x %x", data[0], data[1], data[2], data[3]);

        //__delay_ms(1000);
        
        readmem(i, data, 16);
        plog("READ %x %x %x %x", data[0], data[1], data[2], data[3]);
    }
 
    for(i=0; i<MEMSIZE; i+=65536 * 8L) {
        //plog("----- %lu", i);
        //readmem(i, data, 16);
        //for(j=0; j<4; j++) plog("%x %c", data[j], data[j]);
    }


    LED_On(RED);
    LED_On(GREEN);
    __delay_ms(1000);
    //erasing=1;
    //eraseMem=0;
    while(1);
        
}

// ===========================================================================
// ========================================= MAIN ============================
// ===========================================================================
MAIN_RETURN main(void)
{
    apiversion = 2;
    
    uptime = 0;
    uptime_meas = 0;
        
    tick = 0;
    InitPorts(); // Set TRIS and set all CS, power etc to off state
    
    LED_On(GREEN);
    __delay_ms(100);
    LED_Off(GREEN);

    LED_On(RED);
    __delay_ms(100);
    LED_Off(RED);
    
    //InitUART2(9600, FCY);
    InitUART1(115200, FCY); // used for logging
    plog("-------------")
    plog("Boot");    

    BuildToHex();

    InitT1();
            
    InitInternalADC();
        
    //InitT2();

    // TEST
    OSCCONbits.POSCEN = 0; // Primary oscillator disabled during Sleep mode            
    
    InitUART2(115200, FCY); // used for BLE
    __delay_ms(10);
        
    do {
        plog("InitRN4020");    
        int ret = InitRN4020();
        
        if(ret == 0) break;
        
        plog("RN init err %i", ret);
        DormantRN4020();
        __delay_ms(1000);
 
        
    } while(1);
    
    plog("InitRN4020 finished");

    //sendbt("D");  
    //WaitLine();
    //plog("D: %s", GetRxLine());
    
    //sendbt("S-,Obi");
    plog("V sent");
    sendbt("V");  
    WaitLine();
    
    strcpy(btversion, GetRxLine());
    ReleaseRxLine();
    plog("RN4020 version %s", btversion);
    
    if(strcmp(btversion, "MCHP BTLE v1.10.09 06/09/2014")==0 ||
       strcmp(btversion, "MCHP BTLE v1.20 12/09/2014")==0) {
        plog("RN4020 needs upgrade =================================");
        WakeRN4020();
        __delay_ms(1000);
        
        //greenpattern = 0b0000000000000000000000001010101;
        //greenpattern = 0b0001000000100000010000001010101;
        LED_On(RED);
        LED_On(GREEN);
        
        RN4020OTA();
        
        int connected = 0;
        int lastcmd = 0;
        
        while(1) {
            char *b = WaitLine();
            if(b == NULL) continue;
            plog("%s", b);

            if(strcmp(b,"Connected")==0) connected = 1;
            
            if(connected && strcmp(b,"CMD")==0) {
                plog("Upgrade OK -----------------");
                asm ("RESET");                
            }
            
            ReleaseRxLine();
        }
        
        //while(1);
        
    }
    
    CHGCUR_TRIS = 0;
    CHGCUR = 1; // 1 = 500mA, 0 = 100mA
    
    //DormantRN4020();
    //while(1);
        
    // if vdd is ok, we continue
    // if vdd is not ok we wait in loop and then do a system reset!
    BatCheck();

    // we should read flash memory JEDEC (vendor and id)
    // microchip 25vf... is end-of-life! The others are all page write type
    // need to rewrite write operation to page program

    // Check if we can read from flash memory    
    initflashspi();    
    releasepowerdown();
    flashreset();
    unsigned char manufacturer, device;
    
    readjedecid(&manufacturer, &device);
    if(manufacturer == 1 && device == 0x16) {
        plog("Flash OK");
    } else {
        plog("JEDEC ERROR manuf: %x device %x", (int)manufacturer, (int) device);
        
        LED_On(RED);
        LED_On(GREEN);
        //while(1);
    }

//    testflash();

    ReadConfigAll();
   
    plog("ReadConf OK");    
//    while(1);        

    memptr = findmem();
    plog("Memptr %lu", memptr);
    
    initadcspi();
        
    plog("USB %u\n", USB_BUS_SENSE);
    
    unsigned long maxG = 0;  
    unsigned long minG = 1000000;
    int nG=0;
    
    //pageprogram(50000, page, 1);
    
    //while(1);

    //RtccInitClock();

    // Set RTCC to some initial value
    NVMKEY = 0x55;              // 55/AA sequence to enable write to RTCWREN
    NVMKEY = 0xAA;
    RCFGCALbits.RTCWREN = 1;
    RCFGCALbits.RTCPTR = 3;         // RTCC Value Register Window Pointer bits
    RTCVAL = 0x0010;                     // Set Year (#0x00YY)
    RTCVAL = 0x0101;// Set Month and Day (#0xMMDD)
    RTCVAL = 0x0101;   // Set Weekday and Hour (#0x0WHH). Weekday from 0 to 6
    RTCVAL = 0x0101;  // Set Minute and Second (#0xMMSS)

    //GetTimeStamp();

    memptr = findmem();
    plog("Memptr %lu", memptr);
            
    if(memptr<65536) {
        plog("Empty memory, jump over config area");
        memptr = 65536;
    }

    PowerOpamp(true);

 
    Blink(RED, 1);

    //SetRTC();

    int res = triggeradc();
    if(res ==0) {
        plog("Unexpected, retrigger");
        res = triggeradc();
        if(res == 0) plog("RetrX failed");
    }

    //initcontadc();
    
    rtcintflag = 0;
    t1intflag = 0;

    if(waitbusy()) plog("BUSY");
    //clearblockprotection();

    // write BOOT record to flash
    //logflash("BOOT DT %u", DT);

    btresp == AOK;
    
    int ninvalid=0;
    int nadcerr=0;

    // Start USB
    SYSTEM_Initialize(SYSTEM_STATE_USB_START);   
    USBDeviceInit();

    plog("Start loop");
    
    while(1)
    {
        int r = 0;
        
        //U1OTGSTATbits.SESVD
        ProcRx();
        
        //if(usb_on) USBStuff();
        USBStuff();
                
        if(btresp == NFAIL) {
            btresp = NONE;
            sendbt("R,1");
        }
                
        //------------------------------ timer triggered

        if(USB_BUS_SENSE) {
            USBDeviceAttach();            
        }
                
        if(erasing) {
                        
            if(!eraseReady()) continue;
                        
            plog("blockerase %lu", eraseMem);

            if(eraseMem<8L*1024L*1024L) {
                unsigned char data;
                readmem(eraseMem, &data, 1);

                plog("DATA %u", data);
                if(data != 255) blockerase(eraseMem);
                else plog("skip");
                
                eraseMem += 65536;
                
                continue;
            }
            
            plog("Erase finished ...");       
            
            ReadConfigAll(); // re-read config just in case
            
            erasing = 0;
            memptr = 65536;
            npage = 0;           

        }

        
        if(!t1intflag) {
            if(USB_BUS_SENSE==0) {
                //log("RI %u %u",U2STAbits.RIDLE, U2STAbits.URXDA);
                //U2MODEbits.WAKE = 1;
                if(RxIdle()) {
                    //LED1_LAT=0; // led off
                    Idle();
                    //LED1_LAT=1; // led on                    
                }
            }
            
            continue;
        }

        ProcLeds();   
        
        if(role == ROLE_SYNC_SOURCE && tick % 32768 == 0 && lastSync > 0) {
            SendTimestamp();
        }

        //log("-");

        if(t1intflag>1) plog("*");

        t1intflag=0;

        bool valid = false;
        
        r = getadc();
        //r = getcontadc();
        //log("G %llu %u", tick, r);
        
        if(r==0) {
            plog("ADCERR");
            triggeradc();
            nadcerr++;
            //continue;
        } else {

            G = CalcGsr(hres);                        

            if(G>50 && G<140000) {
                valid = true;
                ninvalid = 0;              
                
                if(G>maxG) maxG = G;
                if(G<minG) minG = G;
                nG ++;
            } else {
                ninvalid++;
                //log("ninvalid %u %lu", ninvalid, G);
                G = 0;
            } //log("G %lu", G);
            
        }

        if(tick % (32768*8) == 32768*2) SendStat();
        else if(valid) if (tick % (32768/2) == 0)SendGsr(G);
                
        if(valid && USB_BUS_SENSE==0 && needSync && (role == ROLE_SYNC_DEST)) {
            WaitForTimestamp();  
        }
                
        if(USB_BUS_SENSE==0 && ninvalid>40 && role != ROLE_SYNC_SOURCE) {           
            npage = 0;

            SleepObi();
            needSync = 1;
            ninvalid=0;
            //lastSync = 0; // reset sync after sleep!
            //log("Reset sync");
        }
        
        sleeping = 0;

        r = triggeradc();
        if(res ==0) {
           plog("Unexpected, retrigger");
           res = triggeradc();
           if(res == 0) plog("Retr failed");
        }

        if(t1intflag) plog("?");
        
        if(valid) {
            uptime_meas++;
                       
            // write a header
            if(npage == 0) {
               //unsigned long long x = 6;
               //log("T %lu", memptr);
               plog("Write header %llu", tick);
                          
               memcpy(page+npage, (unsigned char*) (&tick), 8);
               npage += 8;
            }

            //waitbusy();

            // write gsr data to flash
            memcpy(page+npage, (unsigned char*)(&G), 4);
            npage += 4;
            
            // just filled a page
            if(npage==256) {               
                unsigned char *paddr = (unsigned char *)(&memptr);

                plog("Page write %lu H %x %x %x %x", memptr, paddr[3], paddr[2], paddr[1], paddr[0]);

                if(memptr < MEMSIZE) {
                    waitbusy();
                    pageprogram(memptr, page, 256);                                
                    memptr += 256;
                    
                }
                npage = 0;
            }
            
            //waitbusy();
        }
        
        //LED_Off(GREEN);
        // Flash every 5 sec
        //if(tick<31360154599424LL && n_flash>DT/4 || valid && n_flash>=DT*2 || n_flash>DT*10) {
        //if(tick<31360154599424LL && n_flash>DT/4 || valid && ((tick & (32768-1)) == 0) || ((tick & (32768*8-1)) == 0)) {
        if(tick % (32768*4) == 0) {

            BatCheck();

            plog("VDD %.2f VBAT %.2f CHGSTAT %u USB %u G:%lu", vdd, vbat, CHGSTAT, USB_BUS_SENSE, G);
            plog("memptr %lu npage %u", memptr, npage);
            //plog("BT conn %u state %u nsent %u ", btconnected, btstate, nsent);
            //plog("calib %i %llu", adjustWith, adjust);
            if(nadcerr>0) {
                plog("nadcerr %i", nadcerr);
                nadcerr=0;
            }
            //WriteLogFlash();
            
            minG = 10000000;
            maxG = 0;
            nG = 0;
            
            if(USB_BUS_SENSE) {
                uptime = 0;
                uptime_meas = 0;
                last_charge_bat = 0;
            } else if(last_charge_bat == 0) last_charge_bat = vbat; // we only upgrade last_charge_but when we unplugged the charger
            
            SetLedPattern();
        
        }

        if(t1intflag>1) plog("#");

        t1intflag=0;


    }//end while
}//end main

unsigned long long NextStatTime() {
    return tick + (rand() % (8 * 32768)) + 12 * 32768 ; // somewhere between 12 + 20 sec    
}

void SleepObi() {
    
    plog("SleepObi ======");
    // GO TO SLEEP        
    int nsleep = 0;
    //int myoffset = rand() % 16;        
    
    LED_Off(RED);
    LED_Off(GREEN);

    PowerOpamp(false);
    DormantRN4020();
    
    USBOff();
    
    unsigned long long nextstat = NextStatTime();
        
    while(1) {
        ProcRx();
            
        nsleep ++;
        sleeping = 1;
        
        if(USB_BUS_SENSE) {
            WakeRN4020();

            LED_On(RED);
            __delay_ms(100);
            LED_Off(RED);
            __delay_ms(100);
            LED_On(RED);
            __delay_ms(100);
            LED_Off(RED);
            
            break;
        }
            
        //DT = 1;
        Sleep();

        if(tick % (32768*8) == 0) { // every 8 sec
            //log("Check %llu", tick);
            
            LED_On(RED);
            __delay_ms(10);
            LED_Off(RED);
        }
            
        if(tick > nextstat) { // every X sec
            //log("Check %llu", tick);

            DT = 8;
            
            SetLedPattern();
            
            plog("Wake RN");
            WakeRN4020();
            
            /*
            char *b=WaitLine();
            log("---- '%s'", b);
            if(b!=NULL) ReleaseRxLine();
            */
            
            unsigned long long t1 = tick;
            while(tick - t1 < 50000) Sleep();
            
            WriteLogFlash();
            
            PowerOpamp(true);
                
            int r = triggeradc();
            Sleep();
            t1intflag = 0;                
            r = getadc();
                                
            if(r==0) {
                plog("ADCERR?");
                G = 0;
            } else {
                G = CalcGsr(hres);                
            } 
            
            PowerOpamp(false);
            
            ReadVoltage(); 
            if(last_charge_bat == 0) last_charge_bat = vbat;

            BatCheck();

            SendStat();
                        
            
            unsigned last = tick;
            while(tick < last + 32768) {
                Sleep();
                ProcRx();
            }        
            //SendName();                            
            //Sleep();
            //SendBatMem();
            //Sleep();
            //SendBuild();
                                               
            if(G>50 && G<140000) {
                plog("GSR OK exit sleep");
                break;
            }
            
            Sleep();
            
            plog("Dormant RN");
            DormantRN4020();
                
            //log("Continue sleep"); 
            
            nextstat = NextStatTime();
    
        }            
           
    }    
    
    DT=8;
    USBOn();

}

bool USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, uint16_t size)
{
    switch( (int) event )
    {
        case EVENT_TRANSFER:
            break;

        case EVENT_SOF:
            /* We are using the SOF as a timer to time the LED indicator.  Call
             * the LED update function here. */
            //APP_LEDUpdateUSBStatus();
            break;

        case EVENT_SUSPEND:
            /* Update the LED status for the suspend event. */
            //log("SUSPEND");
            //APP_LEDUpdateUSBStatus();
            break;

        case EVENT_RESUME:
            /* Update the LED status for the resume event. */
            //APP_LEDUpdateUSBStatus();
            break;

        case EVENT_CONFIGURED:
            /* When the device is configured, we can (re)initialize the 
             * demo code. */
            //log("CONFIGURED");
            APP_DeviceCDCBasicDemoInitialize();
            break;

        case EVENT_SET_DESCRIPTOR:
            break;

        case EVENT_EP0_REQUEST:
            /* We have received a non-standard USB request.  The HID driver
             * needs to check to see if the request was for it. */
            USBCheckCDCRequest();
            break;

        case EVENT_BUS_ERROR:
            break;

        case EVENT_TRANSFER_TERMINATED:
            break;

        default:
            break;
    }
    return true;
}

/*******************************************************************************
 End of File
*/
