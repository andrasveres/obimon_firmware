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

/** INCLUDES *******************************************************/
#include "system.h"

#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

#include <usb/usb.h>

//#include <app_led_usb_status.h>
#include <app_device_cdc_basic.h>
#include <usb_config.h>

/** VARIABLES ******************************************************/

static uint8_t readBuffer[CDC_DATA_OUT_EP_SIZE];

//static uint8_t writeBuffer[CDC_DATA_IN_EP_SIZE];
static uint8_t writeBuffer[256];

unsigned long long lastUsb; // last tick when usb was active
unsigned long dumping = 0;

/*********************************************************************
* Function: void APP_DeviceCDCBasicDemoInitialize(void);
*
* Overview: Initializes the demo code
*
* PreCondition: None
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceCDCBasicDemoInitialize()
{
    CDCInitEP();

    
    line_coding.bCharFormat = 0;
    line_coding.bDataBits = 8;
    line_coding.bParityType = 0;
    line_coding.dwDTERate = 9600;

}

/*********************************************************************
* Function: void APP_DeviceCDCBasicDemoTasks(void);
*
* Overview: Keeps the demo running.
*
* PreCondition: The demo should have been initialized and started via
*   the APP_DeviceCDCBasicDemoInitialize() and APP_DeviceCDCBasicDemoStart() demos
*   respectively.
*
* Input: None
*
* Output: None
*
********************************************************************/



void APP_DeviceCDCBasicDemoTasks()
{

    /*
    if(false)
    {
        if(buttonPressed == false)
        {
            if(mUSBUSARTIsTxTrfReady() == true)
            {
                putrsUSBUSART(buttonMessage);
                buttonPressed = true;
            }
        }
    }
*/
            
    /* Check to see if there is a transmission in progress, if there isn't, then
     * we can see about performing an echo response to data received.
     */
    if( USBUSARTIsTxTrfReady() == true)
    {
        uint8_t numBytesRead;

        numBytesRead = getsUSBUSART(readBuffer, sizeof(readBuffer));

        /* For every byte that was read... */
        if(numBytesRead>0) {
           readBuffer[numBytesRead]=0;
           
           lastUsb = tick;
           
           if(dumping) {
                dumping = 0 ; // cancel dump
                plog("Dump canceled");
           }

           switch(readBuffer[0])
            {
                /* If we receive new line or line feed commands, just echo
                 * them direct.
                 */

                // read status register 1
                case '1': {                    
                    char s1 = readstatus();
                    char s2 = readstatus2();
                    char s3 = readstatus3();
                    
                    sprintf(writeBuffer, "1 %u %u %u", s1, s2, s3);

                    putUSBUSART(writeBuffer,strlen(writeBuffer));

                    break;
                }
                
                case '2': {                    
                    sprintf(writeBuffer, "2 %s", btversion);
                    putUSBUSART(writeBuffer,strlen(writeBuffer));

                    break;
                }
                
                // get role -- NOTE capital R!
                case 'R': {
                    sprintf(writeBuffer, "R 0");
                    putUSBUSART(writeBuffer,strlen(writeBuffer));

                    break;
                }

                
                // get api version
                case 'a': {
                    sprintf(writeBuffer, "a %u", apiversion);
                    putUSBUSART(writeBuffer,strlen(writeBuffer));

                    break;
                }

                // get gsr value
                case 'g': {
                    sprintf(writeBuffer, "g %lu", G);

                    putUSBUSART(writeBuffer,strlen(writeBuffer));

                    break;
                }
                
               case 'c': {
                    sprintf(writeBuffer, "c %s", CompileDate);
                    putUSBUSART(writeBuffer,strlen(writeBuffer));
                    break;                   
                }

                // get memptr
                case 'm': {
                    sprintf(writeBuffer, "m %lu", memptr);

                    putUSBUSART(writeBuffer,strlen(writeBuffer));

                    break;
                }

                // erase
                case 'e': {
                    if(numBytesRead>2) {
                        if(erasing==0) {
                            plog("CMD: start erase");
                            erasing = 1;
                            eraseMem = atol(readBuffer+2); 
                            sprintf(writeBuffer, "e %lu", eraseMem);
                        } else {
                            sprintf(writeBuffer, "e ERR");
                        }                            
                    } else {                        
                        if(erasing == 1) {
                            sprintf(writeBuffer, "e %lu", eraseMem);
                        } else {
                            sprintf(writeBuffer, "e READY");                        
                        }
                    }
                    
                    plog("%s", writeBuffer);                                            
                    putUSBUSART(writeBuffer,strlen(writeBuffer));

                    break;
                }

                // get/set time
                case 't': {
                    sprintf(writeBuffer, "t %llu 0", tick + TMR1);
                    
                    putUSBUSART(writeBuffer,strlen(writeBuffer));

                    break;
                }

                // get/set name
                case 'n': {
                    if(numBytesRead>3) {
                        ChangeName(readBuffer+2);                        
                    }

                    sprintf(writeBuffer, "n %s", name);

                    putUSBUSART(writeBuffer,strlen(writeBuffer));

                    break;
                }

                // get/set group
                case 'o': {
                    if(numBytesRead>3) {
                        ChangeGroup(readBuffer+2);                        
                    }

                    sprintf(writeBuffer, "o %s", group);

                    putUSBUSART(writeBuffer,strlen(writeBuffer));

                    break;
                }
                
                case 'r': {
                    asm ("RESET");

                    break;
                }

                case 'b': {
                    sprintf(writeBuffer, "b %u %f %f", 1-CHGSTAT, vbat, vdd);

                    putUSBUSART(writeBuffer,strlen((char*)writeBuffer));

                    break;
                }

                case 'd': {
                    unsigned long dumpaddr = atol(readBuffer+2);

                    //writeBuffer[i] = readBuffer[i];
                    //sprintf(writeBuffer, "m %lu", memptr);

                    writeBuffer[0]='d';

                    waitbusy();
                    readmem(dumpaddr, writeBuffer+1, 64);
                    
                    putUSBUSART(writeBuffer,65);
                    break;
                }

                case 'D': {
                    //writeBuffer[0]='D';
                    
                    dumping = 65536;
                    
                    //putUSBUSART(writeBuffer,1);
                    break;
                }

                /* If we receive something else, then echo it plus one
                 * so that if we receive 'a', we echo 'b' so that the
                 * user knows that it isn't the echo enabled on their
                 * terminal program.
                 */
                default:
                    //writeBuffer[i] = readBuffer[i] + 1;
                    break;
            }
        } else {
            if(dumping) {
                    int plen = 128;
                    
                    waitbusy();
                    readmem(dumping, writeBuffer, plen);
                    
                    dumping += plen;
                    if(dumping >= memptr) {
                        dumping = 0 ;
                    }
                    
                    putUSBUSART(writeBuffer,plen); 
                    lastUsb = tick;

            }
            
            if(tick - lastUsb > 32768) {
                sprintf(writeBuffer, "Obimon %s", name);
                putUSBUSART(writeBuffer,strlen(writeBuffer));
                lastUsb = tick;
            }
        
//#define TESTADC            
#ifdef TESTADC
            if(G!=0) {
                sprintf(writeBuffer, "\nTESTADC %lu\n", G);
                putUSBUSART(writeBuffer,strlen(writeBuffer));
                
                G=0;
            }
#endif
        }
    }

    CDCTxService();
}