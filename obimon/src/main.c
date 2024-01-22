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

   

const char __attribute__((space(prog), address(0x2000))) CCC[] = __DATE__;

const char CompileDate[] =__DATE__ ;
const char CompileTime[]=__TIME__;
 
int dump=0;
unsigned long dumpaddr=0;

void SleepObi();
void WriteGsr(uint64_t ts, uint32_t G, uint32_t acc);

extern unsigned int acc;
extern unsigned int maxacc;
extern bool opamp;
extern unsigned int training;
extern int measuring ;

extern char handle_gsr[5];

extern char tmp[256];

extern int n_blocks_req;
extern char* ch_data;
extern char *ch_name;
extern char *ch_tick;
extern char *ch_status;
extern char *ch_version;

extern uint32_t sessionid;
extern unsigned long dumping;

uint64_t next_sample = 0;

uint32_t cumulative_a = 0;
uint8_t cumulative_n = 0;

extern uint64_t tick;
extern uint64_t lastKeepAlive;

extern uint32_t rawadc;

char is=0;

unsigned char lis = 0;

extern unsigned int vddraw, vbatraw; 

//extern char usblog[256];

uint16_t SLEEPRATE = 128;
uint16_t SAMPLERATE = 8;

unsigned int noadc=0;
int btv = 0;

void __attribute__((interrupt,auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0; //Reset Timer1 interrupt flag

    tick += PR1 + 1; // update time first
    
    if(SLEEPRATE==1) {
        plog("XXX %u", PR1);
    }

    //unsigned long long remainder = tick % (32768 / DT);

    //PR1 = 32768 / DT - 1;
    PR1 = 32768 / SLEEPRATE - 1;

    uint32_t a;
    
    SAMPLERATE = 8;
    
    // check if we have new data
    if(opamp) {
        if (getcontadc(&a)) {

            noadc=0;
            //plog("A %lu", a);

            // if no subsampling
            if (SAMPLERATE == 0) {
                adc_put(a, tick);

            } else {

                cumulative_a += a;
                cumulative_n++;

                // have we passed the next sample time
                if (tick >= next_sample) {

                    a = cumulative_a / cumulative_n;

                    next_sample += 32768 / SAMPLERATE;

                    if(next_sample < tick) {
                        plog("              JUMP");
                        next_sample = tick + 32768 / SAMPLERATE;
                    } else {                    
                        adc_put(a, next_sample);
                    }

                    cumulative_a = 0;
                    cumulative_n = 0;


                }
            }
        } else noadc++;
        
        if(noadc>50) {
            enable_cont_adc();
            noadc = 0;
        }

    }


    //if(remainder>0) PR1 += 32768/DT - remainder;

    //unsigned int tmr1 = TMR1;
    // log("%u", tmr1);

    //log("R %llu %u", remainder, PR1);

    //ProcLeds();

    //uptime++;

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
            
            //plog("N: %i", b)
            if(rxn>0) {
                
                rxbuf[wptr][rxn]=0;
                rxn = 0;
                
                int newwptr = wptr + 1;
                
                if(newwptr >= RXL) {
                    newwptr = 0;
                }
                
                if(newwptr == rptr) {
                    plog("rxl overflow");                    
                } else wptr = newwptr;
                                  
            }

            //rxn = 0;

            continue;
        }
        
        //plog("N: %c", b)


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
    
    double alpha;

    //if(lis) alpha = 0.2 / 3.3; //100.0 / (100.0 + 3000.0);
    // if(lis) alpha = 0.2 / 3.3; 
    if(lis) alpha = 0.4166 / 3.3; 
    else alpha = 100.0/(100+680.0);
    
    
    // TEST =======================

    double R = 100e3;
    //double R = 100e3;

    double A = 1.0*h / 2097152.0;
    double gsr = A / alpha / R * 1e6;

    if(gsr>140) gsr=0;

    return (unsigned long)(gsr*1000);
}

//void printstatusreg() {
//    int s1 = readstatus();
//    int s2 = readstatus2();
//    int s3 = readstatus3();
//
//    if(s1 & 1) plog("BUSY");
//    if(s1 & 2) plog("WEL");
//    if(s1 & 4) plog("BP0");
//    if(s1 & 8) plog("BP2");
//    if(s1 & 16) plog("BP3");
//    if(s1 & 32) plog("TB protect from bottom");
//    if(s1 & 64) plog("4kB protection") else plog("64kB protection")
//    if(s1 & 128) plog("SRP0=1 WP input can protect SR or OTP lockdown") else plog("SRP0=0 WP input has no effect or power supply lock down mode");
//    
//    if(s2 & 1) plog("SRP1=1 WP input protects status register") else  plog("SRP1=0 Power Supply Lock Down or OTP Lock Down");
//    if(s2 & 2) plog("QE");
//    if(s2 & 4) plog("LB0");
//    if(s2 & 8) plog("LB1");
//    if(s2 & 16) plog("LB2");
//    if(s2 & 32) plog("LB3");
//    if(s2 & 64) plog("CMP Inverted protection map");
//    if(s2 & 128) plog("SUS Erase/Program suspended");
//
//    if(s3 & 1) plog("LC0");
//    if(s3 & 2) plog("LC0");
//    if(s3 & 4) plog("LC0");
//    if(s3 & 8) plog("LC0");
//    if(s3 & 16) plog("Burst wrap disabled") else plog("Burst wrap enabled");
//    if(s3 & 32) plog("W4");
//    if(s3 & 64) plog("W5");
//    
//}


// ===========================================================================
// ========================================= MAIN ============================
// ===========================================================================
MAIN_RETURN main(void)
{
    //usblog[0]=0;
    
    apiversion = 2;
    
    //uptime = 0;
        
    //tick = 0x100000000LL;
    //tick = 0x1000000LL;
    
    InitPorts(); // Set TRIS and set all CS, power etc to off state
    
    LED_On(GREEN);
    __delay_ms(100);
    LED_Off(GREEN);
    
    protect[0]=0x11;
    protect[1]=0x22;
    protect[2]=0x33;
    protect[3]=0x44;
    

    
    LED_On(RED);
    __delay_ms(100);
    LED_Off(RED);

//    { // TEST THE RTC CLOCK
//        TRISCbits.TRISC6 = 0;
//        PORTCbits.RC6 = 1;
//        
//        // timer setup
//        T1CON = 0x00; //Stops the Timer1 and reset control reg.
//        TMR1 = 0x00; //Clear contents of the timer register
//        PR1 = 32768 / DT; //Load the Period register
//        IPC0bits.T1IP = 7; // Prioty 7 highest
//
//        IFS0bits.T1IF = 0; //Clear the Timer1 interrupt status flag
//        IEC0bits.T1IE = 0; //Disable Timer1 interrupts
//        T1CONbits.TCS = 1; // external clock source
//        T1CONbits.TCKPS = 0b00;
//        T1CONbits.TSYNC = 0;
//            
//        T1CONbits.TON = 1; //Start Timer1 
//    
//        unsigned int t;
//        while(1) {
//            while(t == TMR1);
//            
//            t = TMR1;
//            
//            PORTCbits.RC6 = 1;
//            __delay_us(10);
//            PORTCbits.RC6 = 0;
//        }
//    }
    
    //InitUART2(9600, FCY);
    InitUART1(115200, FCY); // used for logging
    plog("-------------")
    plog("Boot");    
    
    
    
//    // TEST
//    enable_cont_adc();
//    uint32_t a;
//    int r;
//    while(1) {
//        r = getcontadc(&a);
//        if(r==1) plog("%lu", a);
//    }

    
    initflashspi();    

    
    //InitSPI1_TEST(0,0,1,0);
    
    if(lis_whoami()==51) {
        plog("LIS detected");
        lis = 1;
        
        lis_cfg();
        lis_reg23();
        lis_hpfilter();
        
        lis_click_i1click();
        lis_click_latch();
        lis_click_cgf();
        lis_click_ths();
        lis_click_timelimit();
        
        lis_fifo_en();
        lis_fifo_ctrl();
    } else {
        plog("LIS NOT detected");

    }
    
    //while(1) {
    //    lis_readacc();
    //    __delay_ms(100);
    //}

    BuildToHex();

    InitT1();
    
    // Wait for 32khz clock to stabilize
    
    int ii=0;
    while(1) {
        
        unsigned long long t = tick;
        __delay_ms(100);
        ii++;
        
        unsigned long dt = tick - t;        
        plog("RTC init %lu ii %u", dt, ii);

        if(dt > 32768 / 10 / 2) break;
        
        if(ii>50) {
            // 5 sec passed still no clock
            plog("RTC failed, reset timer!");
            InitT1();
            ii = 0;

        }
    } 
    
    
    PowerOpamp(true);
    __delay_ms(100);
    enable_cont_adc();
    clear_adc_buf();
            
    InitInternalADC();
        
    //InitT2();
    
    //initadcspi();
    //test_cont();

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
    send("V");  
    WaitLine();
    
    strcpy(tmp, GetRxLine());
    ReleaseRxLine();
    plog("RN4020 version %s", tmp);
    
    if(strcmp(tmp, "MCHP BTLE v1.10.09 06/09/2014")==0) btv=110;
    if(strcmp(tmp, "MCHP BTLE v1.20 12/09/2014")==0) btv = 120; 
    if(strcmp(tmp, "MCHP BTLE v1.23.5 8/7/2015")==0) btv = 123;
    if(strcmp(tmp, "MCHP BTLE v1.33.4 BEC 11/24/2015")==0) btv = 133;
    
//    if(btv<123) {
//    
//        UpgradeRN();
//        //while(1);
//        
//    }
    
    //if(btv<133) ConfRN4020();
    //else 
       

    // TEST
//    unsigned char ii=0;
//    while(1) {
//        
//        //__delay_ms(1000);
//        WaitResp(); // wait for BT event up to 1 sec
//                
//        if(n_blocks_req>0) {
//            plog("----------------- SEND BLOCK");
//            int i=0;
//            for(i=0; i<n_blocks_req; i++) {
//                //send("SHW,%s,%02x", handle_gsr, ii);   
//                send("SUW,%s,000102030405060708090a0b0c0d0e0f%04x", ch_data, i); // here we use long format and not the handle! We should change it to handle
//                WaitResp();
//                if(btresp != AOK) {
//                   plog("ERR resp");
//                }
//                
//                __delay_ms(10);
//            }
//            
//            n_blocks_req = 0;
//        }                
//                
//        plog("X");
//        
//        send("Y"); 
//        WaitResp();
//        if(btresp != AOK) {
//           plog("ERR resp");
//        }
//
//        send("NZ"); 
//        WaitResp();
//        if(btresp != AOK) {
//           plog("ERR resp");
//        }
//        
//        send("NA,ff%02x%02x%02x%02x", ii, ii+1, ii+2, ii+3); 
//        send("NA,0944617665"); 
//
//        WaitResp();
//        if(btresp != AOK) {
//           plog("ERR resp");
//        }
//
//        
//        nsent++;
//        //send("A,0050,07d0"); // interval 80 msec, duration 2 sec 
//        send("A");
//        WaitResp();
//        if(btresp != AOK) {
//           plog("ERR resp");
//        }
//
//        
//        send("SHW,%s,%02x", handle_gsr, ii);        
//        WaitResp();
//        if(btresp != AOK) {
//           plog("ERR resp");
//        }
//        
//        send("Q,1"); // BT bonding statusß
//        WaitResp();
//        if(btresp != AOK) {
//           plog("Warning command failed");
//        }
//    
//        ii++;
//        
//        
//
//    }
    
    CHGCUR_TRIS = 0;
    CHGCUR = 1; // 1 = 500mA, 0 = 100mA
    
    //DormantRN4020();
    //while(1);
    
    ReadVoltage();     
    // if vdd is ok, we continue
    // if vdd is not ok we wait in loop and then do a system reset!
    //BatCheck();
    

    // we should read flash memory JEDEC (vendor and id)
    // microchip 25vf... is end-of-life! The others are all page write type
    // need to rewrite write operation to page program

    // Check if we can read from flash memory    
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

    ConfRN4020_new();

    // = findmem();
    //plog("Memptr %lu", memptr);
    
        
    plog("USB %u\n", USB_BUS_SENSE);

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


 
    Blink(RED, 1);

    //SetRTC();

//    int res = triggeradc();
//    if(res ==0) {
//        plog("Unexpected, retrigger");
//        res = triggeradc();
//        if(res == 0) plog("RetrX failed");
//    }

    //initcontadc();
    
    rtcintflag = 0;

    if(waitbusy()) plog("BUSY");
    //clearblockprotection();

    // write BOOT record to flash
    //logflash("BOOT DT %u", DT);

    btresp == AOK;
    
    int ninvalid=0;
    int nadcerr=0;
    measuring = 0;

    // Start USB
    SYSTEM_Initialize(SYSTEM_STATE_USB_START);   
    USBDeviceInit();

    plog("Start loop");
    
    unsigned long long last_stat = 0;
    unsigned long long last_bat_check = 0;
    
    // create seed for random:
    unsigned int ss = (unsigned int) rawadc;
    unsigned int ss2 = (unsigned int) tick;
    unsigned int seed = ss + ss2 + vddraw + vbatraw;
    plog("SEED %u", seed);
    srand(seed);
    
    //unsigned int seed = ()
    CreateNewSession(); // we maintain the same sessionid until the device is powered up
    SetCharData();
    
    uint64_t lastwrite=0;
    
    while(1)
    {        
        //U1OTGSTATbits.SESVD
        ProcRx();
        
        //if(usb_on) USBStuff();
        USBStuff();
                
        if(btresp == NFAIL) {
            btresp = NONE;
            send("R,1");
        }
                
        //------------------------------ timer triggered

        if(USB_BUS_SENSE) {
            USBDeviceAttach();            
        }
        
        if(dumping) {
            if(USB_BUS_SENSE ==0) dumping = 0; // cancel dumping if disconnected
            else continue; // else do not do the usual stuff, only dump!
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

        
//        if(!t1intflag) {
//            if(USB_BUS_SENSE==0) {
//                //log("RI %u %u",U2STAbits.RIDLE, U2STAbits.URXDA);
//                //U2MODEbits.WAKE = 1;
//                if(RxIdle()) {
//                    //LED1_LAT=0; // led off
//                    Idle();
//                    //LED1_LAT=1; // led on                    
//                }
//            }
//            
//            continue;
//        }

        ProcLeds();   

        // log("-");

        //if(t1intflag>1) plog("*");

        //t1intflag=0;

        bool valid = false;
        
        uint32_t a;
        uint64_t ts;
        
        if(adc_pop(&a, &ts)) {
        
            G = CalcGsr(a);                        

            //plog("G %llu %lu measuring %u", ts, G, measuring);
            
            if (lis) {
                acc = 0;
               //lis_click_read();
               
               unsigned int nacc = lis_read_fifo_src(); 
               if(nacc>0) {
                  //plog("NACC %u", nacc);
                   acc = lis_read_fifo(nacc);
                   if (acc > maxacc) maxacc = acc;
                   if(maxacc>255) maxacc=255;
                   
                   //plog("X %u %u", acc, maxacc);

               }
               
            }
            
            if (G > 50 && G < 140000) {
                valid = true;
                ninvalid = 0;
                measuring = 1;
                
            } else {
                ninvalid++;
            } 
                                       
            if (measuring) {
                
                //plog("ACC %u", maxacc);
                SendGsr(G,maxacc);            
                WriteGsr(ts, G, maxacc);
                                
//                int dt = ts - lastwrite;
//                //if(ts-lastwrite>4096) 
//                plog("                DTS %llu %llu %u", ts, lastwrite, dt);
//                lastwrite = ts;
            }

            // we only switch to no measuring, when a page is just written, so that all data are flushed
            if(ninvalid > 40 && npage == 0) measuring = 0;

        }
        
        if(tick - last_stat > 7*32768L) {
            if(training == 0) SendStat();
            else {
                // send battery and usbconnected 
                
                unsigned int b = (unsigned int) (vbat*100.0);
                unsigned char u = (unsigned char) USB_BUS_SENSE;

                send("SUW,%s,%04x%02x", ch_status, b, u); // here we use long format and not the handle! We should change it to handle
                WaitLine();
            }
            last_stat = tick;
            
            Hack(); // Reset every 4 hours if not measuring
        } 
        

                               
        if(USB_BUS_SENSE==0 && measuring == 0) {
            
            SleepObi();
            
            ninvalid=0;
            //measuring = 1;
            
            //lastSync = 0; // reset sync after sleep!
            //log("Reset sync");            
        }
        

//        r = triggeradc();
//        if(res ==0) {
//           plog("Unexpected, retrigger");
//           res = triggeradc();
//           if(res == 0) plog("Retr failed");
//        }
        
        if(btconnected && tick - lastKeepAlive > 60 *32768L) {
            plog("\t\t\tBT TIMEOUT");
            sendbt("K",1); // kill konnection
            WaitResp(); // AAA

        }

        
        //LED_Off(GREEN);
        // Flash every 5 sec
        //if(tick<31360154599424LL && n_flash>DT/4 || valid && n_flash>=DT*2 || n_flash>DT*10) {
        //if(tick<31360154599424LL && n_flash>DT/4 || valid && ((tick & (32768-1)) == 0) || ((tick & (32768*8-1)) == 0)) {
        if(tick - last_bat_check > 4 *32768L) {
            
            plog("CHECK CONN");
            sendbt("Q",1); // check connnection status
            WaitResp(); // AAA


            // sendbt("GT",1); // check connnection status
            SetCharData();
            
            last_bat_check = tick;

            //BatCheck();
            ReadVoltage(); 

            plog("VDD %.2f VBAT %.2f CHGSTAT %u USB %u G:%lu", vdd, vbat, CHGSTAT, USB_BUS_SENSE, G);
            plog("measuring %u memptr %lu npage %u", measuring, memptr, npage);
            //plog("BT conn %u state %u nsent %u ", btconnected, btstate, nsent);
            if(nadcerr>0) {
                plog("nadcerr %i", nadcerr);
                nadcerr=0;
            }
            //WriteLogFlash();
            
            //minG = 10000000;
            //maxG = 0;
            //nG = 0;
            
            if(USB_BUS_SENSE) {
                //uptime = 0;
                last_charge_bat = 0;
            } else if(last_charge_bat == 0) last_charge_bat = vbat; // we only upgrade last_charge_but when we unplugged the charger
            
            SetLedPattern();
        
        }


    }//end while
}//end main

void WriteGsr(uint64_t ts, uint32_t G, uint32_t acc) {
    //plog("GG %i %lu", acc, G);
    unsigned long d = G | (((unsigned long) acc) << 24);

    if (memptr == MEMSIZE) {
        plog("MEMFULL\n");
        return;
    }
    
    if(protect[0]!=0x11 || protect[1]!=0x22 || protect[2]!=0x33 || protect[3]!=0x44) plog("+++++++++++++++ PROTECTION ERR");
    
    // write a header
    if (npage == 0) {
        //unsigned long long x = 6;
        //log("T %lu", memptr);
        //plog("Write header %llu %lu", ts, sessionid);
        plog("Write header %08lx ts %llx", sessionid, ts);

        memcpy(page + npage, (unsigned char*) (&ts), 8);
        npage += 8;

        // write sessionid to flash
        memcpy(page + npage, (unsigned char*) (&sessionid), 4);
        npage += 4;
    }

    //waitbusy();
    // write gsr data to flash
    memcpy(page + npage, (unsigned char*) (&d), 4);
    npage += 4;

    // just filled a page
    if (npage == 256) {
        //unsigned char *paddr = (unsigned char *) (&memptr);

        //plog("Page write %lu H %x %x %x %x", memptr, paddr[3], paddr[2], paddr[1], paddr[0]);

        waitbusy();
        pageprogram(memptr, page, 256);
        memptr += 256;
        
        npage = 0;
    }
}

////8388352


unsigned long long NextStatTime() {
    return tick + (rand() % (2 * 32768)) + 5 * 32768 ; // somewhere between 12 + 20 sec    
}

void SleepObi() {
    
    plog("SleepObi ======");
    // GO TO SLEEP        
    int nsleep = 0;
    //int myoffset = rand() % 16;    

    sleeping =1;

    if(btconnected) {
        send("K"); //  Kill current connection is any
        WaitResp();
    }
    
    
    LED_Off(RED);
    LED_Off(GREEN);

    PowerOpamp(false);
    DormantRN4020();
    
    USBOff();
    
    disable_cont_adc();

    
    unsigned long long nextstat = NextStatTime();
        
    int n= 0;
    
    while(1) {
        
        plog("%llu SLEEP LOOP ----", tick);

        ProcRx();
            
        nsleep ++;
                
        if(USB_BUS_SENSE) {
            WakeRN4020();

            LED_On(RED);
            __delay_ms(10);
            LED_Off(RED);
            __delay_ms(10);
            LED_On(RED);
            __delay_ms(10);
            LED_Off(RED);
            
            break;
        }
            
        SLEEPRATE = 1;
        Sleep();
        //plog("T1 %llu", tick);
        //unsigned long long sleepstart = tick;

        n++;
        
        if(n % 8 == 0) { // every 8 sec
            //log("Check %llu", tick);
            Hack(); // Reset every 4 hours if not measuring

            if(vbat < 3.3) {
                LED_On(RED);
                __delay_ms(20);
                LED_Off(RED);
            } else {
                LED_On(GREEN);
                __delay_ms(20);
                LED_Off(GREEN);                
            }
        }
            
        if(tick > nextstat) { // every X sec
            //log("Check %llu", tick);

            PowerOpamp(true);
            enable_cont_adc();
            
            clear_adc_buf();
                        
            // SetLedPattern();
            
            if(training==0) {
                plog("Wake RN");
                WakeRN4020();
            }
                        
            SLEEPRATE = 128;
                        
            uint64_t lasts = tick;
            while(tick < lasts + 32768*1) Sleep();
                            
            clear_adc_buf(); // clear early data
                        
            uint32_t a;
            uint64_t ts;
            while(adc_pop(&a, &ts)==0); // wait for first data
                                
            G = CalcGsr(a);
                        
            ReadVoltage(); 
            if(last_charge_bat == 0) last_charge_bat = vbat;

            if(training == 0) {
                SendStat();
                                   
                uint64_t last = tick;
                while(tick < last + 32768) {
                    Sleep();
                    ProcRx();
                }        
                
            }
                                               
            if(G>50 && G<140000) {
                plog("GSR %lu OK exit sleep", G);
                break;
            }
            plog("GSR %lu NOT DETECTED", G);

            disable_cont_adc();
            PowerOpamp(false);
            
            Sleep();
            
            if(training==0) {
                plog("%llu Dormant RN", tick);
                DormantRN4020();
            }
                
            plog("%llu Continue SLEEP", tick);

            nextstat = NextStatTime();

        }            
           
    }    
    
    if(training) {
        // if training, we switch on BT now
        plog("Wake RN");
        WakeRN4020();
    }

    PowerOpamp(true);
    SLEEPRATE = 128;
    USBOn();

    sleeping = 0;
    
    SetCharData();


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
