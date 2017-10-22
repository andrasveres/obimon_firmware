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

#include <p24FJ64GB004.h>
#include <system.h>
#include <system_config.h>
#include <usb/usb.h>
//#include <adc.h>

/** CONFIGURATION Bits **********************************************/
//_CONFIG1(
//    WDTPS_PS1 &
//    FWPSA_PR32 &
//    WINDIS_OFF &
//    FWDTEN_OFF &
//    ICS_PGx1 &
//    GWRP_OFF &
//    GCP_OFF &
//    JTAGEN_OFF
//);
//
//_CONFIG2(
//    POSCMOD_HS &
//    I2C1SEL_PRI &
//    IOL1WAY_OFF &
//    OSCIOFNC_ON &
//    FCKSM_CSDCMD &
//    FNOSC_PRIPLL &
//    PLL96MHZ_ON &
//    PLLDIV_DIV2 &
//    IESO_OFF
//);
//
//_CONFIG3(
//    WPFP_WPFP0 &
//    SOSCSEL_SOSC &
//    WUTSEL_LEG &
//    WPDIS_WPDIS &
//    WPCFG_WPCFGDIS &
//    WPEND_WPENDMEM
//);
//
//_CONFIG4(
//    DSWDTPS_DSWDTPS3 &
//    DSWDTOSC_LPRC &
//    RTCOSC_SOSC &
//    DSBOREN_OFF &
//    DSWDTEN_OFF
//);

/** CONFIGURATION Bits **********************************************/

#pragma config DSWDTPS = DSWDTPS3       // DSWDT Postscale Select (1:128 (132 ms))
#pragma config DSWDTOSC = SOSC          // Deep Sleep Watchdog Timer Oscillator Select (DSWDT uses Secondary Oscillator (SOSC))
#pragma config RTCOSC = SOSC            // RTCC Reference Oscillator  Select (RTCC uses Secondary Oscillator (SOSC))
#pragma config DSBOREN = ON             // Deep Sleep BOR Enable bit (BOR enabled in Deep Sleep)
#pragma config DSWDTEN = OFF            // Deep Sleep Watchdog Timer (DSWDT disabled)

// CONFIG3
#pragma config WPFP = WPFP0             // Write Protection Flash Page Segment Boundary (Page 0 (0x0))
#pragma config SOSCSEL = SOSC           // Secondary Oscillator Pin Mode Select (SOSC pins in Default (high drive-strength) Oscillator Mode)
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

int bootloader_found ;

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
            //Switch to alternate interrupt vector table for bootloader
            INTCON2bits.ALTIVT = 1;
            //BUTTON_Enable(BUTTON_USB_DEVICE_HID_CUSTOM);

            // ETHAVS
            //if((BUTTON_IsPressed(BUTTON_USB_DEVICE_HID_CUSTOM)==false) && ((RCON & 0x83) != 0))
            //{
                //Switch to app standare IVT for non boot mode
            //    INTCON2bits.ALTIVT = 0;
            //    __asm__("goto 0x1800");
            //} 
            
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
            //LED_Enable(LED_USB_DEVICE_HID_CUSTOM);
            
            break;
            
        case SYSTEM_STATE_USB_SUSPEND:
            break;
            
        case SYSTEM_STATE_USB_RESUME:
            break;
    }
}

#if defined(USB_INTERRUPT)
void __attribute__((interrupt,auto_psv)) _USB1Interrupt()
{
    USBDeviceTasks();
}
#endif

