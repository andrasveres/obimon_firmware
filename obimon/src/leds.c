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

#include <xc.h>
#include <leds.h>
#include <stdbool.h>
#include <system.h>
#include <libpic30.h>


#define LED_ON  0
#define LED_OFF 1

#define INPUT  1
#define OUTPUT 0

/*********************************************************************
 * Function: void LED_On(LED led);
 *
 * Overview: Turns requested LED on
 *
 * PreCondition: LED configured via LED_Configure()
 *
 * Input: LED led - enumeration of the LEDs available in this
 *        demo.  They should be meaningful names and not the names of
 *        the LEDs on the silkscreen on the board (as the demo code may
 *        be ported to other boards).
 *         i.e. - LED_On(LED_CONNECTION_DETECTED);
 *
 * Output: none
 *
 ********************************************************************/
void LED_On ( LED led )
{
    switch (led)
    {
        case LED1:
        case GREEN:
            LED1_LAT = LED_ON ;
            break ;

        case LED2:
        case RED:
            LED2_LAT = LED_ON ;
            break ;

        case LED_NONE:
            break ;
    }
}
/*********************************************************************
 * Function: void LED_Off(LED led);
 *
 * Overview: Turns requested LED off
 *
 * PreCondition: LED configured via LEDConfigure()
 *
 * Input: LED led - enumeration of the LEDs available in this
 *        demo.  They should be meaningful names and not the names of
 *        the LEDs on the silkscreen on the board (as the demo code may
 *        be ported to other boards).
 *         i.e. - LED_Off(LED_CONNECTION_DETECTED);
 *
 * Output: none
 *
 ********************************************************************/
void LED_Off ( LED led )
{
    switch (led)
    {
        case LED1:
        case GREEN:
            LED1_LAT = LED_OFF ;
            break ;

        case LED2:
        case RED:
            LED2_LAT = LED_OFF ;
            break ;

        case LED_NONE:
            break ;
    }
}
/*********************************************************************
 * Function: void LED_Toggle(LED led);
 *
 * Overview: Toggles the state of the requested LED
 *
 * PreCondition: LED configured via LEDConfigure()
 *
 * Input: LED led - enumeration of the LEDs available in this
 *        demo.  They should be meaningful names and not the names of
 *        the LEDs on the silkscreen on the board (as the demo code may
 *        be ported to other boards).
 *         i.e. - LED_Toggle(LED_CONNECTION_DETECTED);
 *
 * Output: none
 *
 ********************************************************************/
void LED_Toggle ( LED led )
{
    switch (led)
    {
        case LED1:
        case GREEN:
            LED1_LAT ^= 1 ;
            break ;

        case LED2:
        case RED:
            LED2_LAT ^= 1 ;
            break ;

        case LED_NONE:
            break ;
    }
}

/*********************************************************************
 * Function: void LED_Enable(LED led);
 *
 * Overview: Configures the LED for use by the other LED API
 *
 * PreCondition: none
 *
 * Input: LED led - enumeration of the LEDs available in this
 *        demo.  They should be meaningful names and not the names of
 *        the LEDs on the silkscreen on the board (as the demo code may
 *        be ported to other boards).
 *
 * Output: none
 *
 ********************************************************************/
void LED_Enable ( LED led )
{
    switch (led)
    {
        case LED1:
        case GREEN:
            LED1_TRIS = OUTPUT ;
            break ;

        case LED2:
        case RED:
            LED2_TRIS = OUTPUT ;
            break ;

        case LED_NONE:
            break ;
    }

    LED_Off(led);
}

void Blink(LED led, unsigned int number_of_blink)
{
   unsigned int n=0;

   for(n=0;n<number_of_blink;n++) {
            LED_On(led);
            __delay_ms(100);
            LED_Off(led);
            __delay_ms(100);

   }
}