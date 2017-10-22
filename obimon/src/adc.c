
#include <xc.h>
#include <adc.h>
#include <system.h>

#include <libpic30.h>

unsigned long hres;

#define DADC 10

void initadcspi() {
    InitSPI2(3,3);
}


void initcontadc() {
    __delay_us(DADC);    
    ADC_CS = 1;
    __delay_us(DADC);    
    ADC_CS = 0;
    __delay_us(DADC);
    __delay_ms(100);

}

int getcontadc() {
    //if(!SDI2) {
        clockout();
        return 1;
    //} else return 0;
    
    //Turn off the SPI 
    SPI2STATbits.SPIEN=0; 
    __delay_us(DADC);    
    //Clock the line low 
    TRISCbits.TRISC0=0;
    ADC_SCK=0; 
    __delay_us(DADC);    
    //clock the line high 
    ADC_SCK=1; 
    //turn the SSP back on 
    __delay_us(DADC);    
    SPI2STATbits.SPIEN=1; 
    __delay_us(DADC);    
    
}

void clockout() {
            unsigned char c[4];

            SPI2BUF = 0;
            while(SPI2STATbits.SPIRBF==0);
            c[0] = SPI2BUF;

            SPI2BUF = 0;
            while(SPI2STATbits.SPIRBF==0);
            c[1] = SPI2BUF;

            SPI2BUF = 0;
            while(SPI2STATbits.SPIRBF==0);
            c[2] = SPI2BUF;

            c[3] = 0;
            //getsSPI2(3, c, 5000);
            //hres = c[2]+c[1]*256+c[0]*65536;

            hres = c[0];
            hres *= 256;
            hres += c[1];
            hres *= 256;
            hres += c[2];
}

int triggeradc() {
    int res = 0;

        ADC_CS = 1;
        __delay_us(DADC);
        ADC_CS = 0;
        __delay_us(DADC);

        if(!SDI2) {
            clockout();
            ADC_CS = 1;
            __delay_us(DADC);
            ADC_CS = 0;
            __delay_us(DADC);

            res = 0;
        } else res = 1;

        ADC_CS = 1;
        __delay_us(DADC);

        return res;
}

int getadc() {
        int res = 0;

        __delay_us(DADC);
        ADC_CS = 1;
        __delay_us(DADC);

        ADC_CS = 0;
        __delay_us(DADC);

        if(!SDI2) {

            __delay_us(DADC);

            clockout();
            res = 1;

        } else {
            hres = 0;
        }

        __delay_us(DADC);
        ADC_CS = 1;

        return res;
}
