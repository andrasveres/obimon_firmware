
#include <xc.h>
#include <adc.h>
#include <system.h>

#include <libpic30.h>

unsigned long hres;
uint32_t rawadc=0;

#define DADC 100

//void initadcspi() {
//    InitSPI2(3,3);
//}

#define ADCBUFLEN 50
uint32_t adc_buf[ADCBUFLEN];
uint64_t adc_buf_ts[ADCBUFLEN];

volatile uint16_t adc_write_ptr = 0; // next free slot
volatile uint16_t adc_read_ptr = 0; // next data to read

void clear_adc_buf() {
    adc_read_ptr = adc_write_ptr;
}

int adc_put(uint32_t a, uint64_t ts) {    
    
    int nextptr = (adc_write_ptr + 1) % ADCBUFLEN;
    
    if(nextptr == adc_read_ptr) {
        // full
        //plog("ADC buf is full!");
        return 0;
    }

    adc_buf[adc_write_ptr] = a;
    adc_buf_ts[adc_write_ptr] = ts;
    
    adc_write_ptr = nextptr;

    return 1;
}

int adc_pop(uint32_t *a, uint64_t *ts) {
    
    if(adc_read_ptr == adc_write_ptr) {
        // empty
        return 0;
    }
    
    *a = adc_buf[adc_read_ptr];
    *ts = adc_buf_ts[adc_read_ptr];
    
    adc_read_ptr = (adc_read_ptr + 1) % ADCBUFLEN;
    
    return 1;
}

int getcontadc(uint32_t *d) {    
    if(SDI2 == 1) {
        //plog("n");
        return 0;
    }
    //plog("g");

    int i;

    ADC_SCK = 0;
    __delay_us(1);        
    
    uint32_t a=0;

    for(i=0; i<24; i++) {
        ADC_SCK = 1;
        asm("nop");
        a = (a << 1) | SDI2;

        ADC_SCK = 0;
        //__delay_us(1);                
        asm("nop");

    }

    ADC_SCK = 1;
    __delay_us(1);                
    
    rawadc = a;

    if(a & 0xe00000) a = 0;
    
    *d = a;

    //if(a & (1L<<23)) plog("OVL");
    //if(a & (1L<<22)) plog("OVH");
    //if(a & (1L<<21)) plog("NEG");

    // unsigned long g = CalcGsr(a);

    // plog("A %llu %lu", tick, g);
    
    return 1;
                
}


void test_cont() {
    // FCY = 16Mhz
    
    plog("test_cont %lu", FCY);
    
    enable_cont_adc();
    
    while(1) {
        uint32_t a;
        uint64_t ts;
        
        if(adc_pop(&a, &ts)) {
            unsigned long g = CalcGsr(a);
            plog("%llu %lu", ts, g);            
        }

        // __delay_ms(1);

    }
    
}

// continuous adc, move 
void enable_cont_adc() {
    plog("Enabling cont adc");
    
    ADC_CS_TRIS = 0;
    ADC_SCK_TRIS = 0;    
    SDI2_TRIS = 1;

    ADC_SCK = 1;
    __delay_us(DADC);


    ADC_CS = 1;
    __delay_us(DADC);
    ADC_CS = 0;
    __delay_us(DADC);
}

// continuous adc, move 
void disable_cont_adc() {
    plog("Disabling cont adc");

    ADC_CS = 1;
    __delay_us(DADC);
}



//void clockout() {
//    unsigned char c[4];
//
//    SPI2BUF = 0;
//    while(SPI2STATbits.SPIRBF==0);
//    c[0] = SPI2BUF;
//
//    SPI2BUF = 0;
//    while(SPI2STATbits.SPIRBF==0);
//    c[1] = SPI2BUF;
//
//    SPI2BUF = 0;
//    while(SPI2STATbits.SPIRBF==0);
//    c[2] = SPI2BUF;
//
//    c[3] = 0;
//    //getsSPI2(3, c, 5000);
//    //hres = c[2]+c[1]*256+c[0]*65536;
//
//    hres = c[0];
//    hres *= 256;
//    hres += c[1];
//    hres *= 256;
//    hres += c[2];
//}
//
//int triggeradc() {
//    int res = 0;
//
//    ADC_CS = 1;
//    __delay_us(DADC);
//    ADC_CS = 0;
//    __delay_us(DADC);
//
//    if(!SDI2) {
//        clockout();
//        ADC_CS = 1;
//        __delay_us(DADC);
//        ADC_CS = 0;
//        __delay_us(DADC);
//
//        res = 0;
//    } else res = 1;
//
//    ADC_CS = 1;
//    __delay_us(DADC);
//
//    return res;
//}
//
//int getadc() {
//        int res = 0;
//
//        __delay_us(DADC);
//        ADC_CS = 1;
//        __delay_us(DADC);
//
//        ADC_CS = 0;
//        __delay_us(DADC);
//
//        if(!SDI2) {
//
//            __delay_us(DADC);
//
//            clockout();
//            res = 1;
//
//        } else {
//            hres = 0;
//        }
//
//        __delay_us(DADC);
//        ADC_CS = 1;
//
//        return res;
//}
