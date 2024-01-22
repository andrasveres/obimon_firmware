
#include <xc.h>

#include "mem_s25fl164k.h"
#include <system.h>
#include <libpic30.h>
#include <stdint.h>

#define D 10

extern uint8_t acc_hist[16];

void initflashspi() {
    InitSPI1();
    //InitSPI1(3,3);
}

unsigned long findmem() {
   unsigned char data;
   unsigned long low = 0;
   unsigned long step = MEMSIZE / 2;

   unsigned long p;

   plog("Findmem");
   
   do {
      p = low + step;
      readmem(p, &data, 1);

      //log("[%lu]=%u", p, data);
      
      if(data!=255) low = p;

      if(step == 256) return low;

      step = step / 2;

    } while(1);
}

void startcs() {
    __delay_us(D);
    MEM_CS = 1;
    __delay_us(D);

    MEM_CS = 0;
    __delay_us(D);
}

void endcs() {
    __delay_us(D);
    MEM_CS = 1;    
}

unsigned char bytetrx(char b) {
    
    SPI1BUF = b;
    while(SPI1STATbits.SPIRBF==0);
    b = SPI1BUF;

    return b;       
}


void releasepowerdown() {
    plog("release power down");
    startcs();
    bytetrx(0xab);
    endcs();    

    __delay_us(100);
}

void flashreset() {
    plog("flash reset");

    startcs();
    bytetrx(0x66);
    bytetrx(0x99);
    endcs();
    
    __delay_us(100);
}

void pageprogram(unsigned long addr, unsigned char *p, unsigned int n) {

    // write 1 page to memory

    int i;
    
    writeenable();
    
    unsigned char b;
    unsigned char *paddr;

    paddr = (unsigned char *)(&addr);

    startcs();

    bytetrx(0x02);
    bytetrx(paddr[2]);
    bytetrx(paddr[1]);
    bytetrx(paddr[0]);

    for(i=0; i<n; i++) {
       b = bytetrx(p[i]);
    }

    endcs();
    
    waitbusy();
}

void readmem(unsigned long addr, unsigned char* data, int n) {
        unsigned char b;
        int i=0;
        unsigned char *p;

        p = (unsigned char *)(&addr);
        
        //plog("ADDR %x %x %x", p[2], p[1], p[0]);

        startcs();

        //LED_Toggle(LED_TEST);
        bytetrx(0x03);
        bytetrx(p[2]);
        bytetrx(p[1]);
        bytetrx(p[0]);

        for(i = 0; i<n; i++) {
           b =bytetrx(0);
           data[i] = b;
        }

        endcs();
        
        waitbusy();
}

void readjedecid(unsigned char *manufacturer, unsigned char *device) {
        unsigned char b;
        
        startcs();
        
        bytetrx(0x90);
        b = bytetrx(0x00);
        b = bytetrx(0x00);
        b = bytetrx(0x00);

         (*manufacturer) = bytetrx(0x00);
         (*device) = bytetrx(0x00);
         
        endcs();
                 
        return;
}

void writeenable() {
        startcs();
        bytetrx(0x06);
        endcs();
        
        int s = readstatus();
        if((s & 2) == 0) plog("NOT WRITE ENABLED");
        //plog("WE %x", s);
}

int waitbusy() {
    int waited = false;
    while((readstatus()&0x01) == 0x01) {
        __delay_us(100);
        waited = true;
    }

    return waited;
}


unsigned char readstatus() {
    unsigned char b;
    startcs();            
    bytetrx(0x05);
    b = bytetrx(0);
    endcs();
        
    return b;
}

unsigned char readstatus2() {
    unsigned char b;
    startcs();            
    bytetrx(0x35);
    b = bytetrx(0);
    endcs();

    return b;
}

unsigned char readstatus3() {
    unsigned char b;
    startcs();            
    bytetrx(0x33);
    b = bytetrx(0);
    endcs();

    return b;    
}

void writestatusregs(char s1, char s2, char s3) {
    writeenable(); // to write non-volatile registers    

    startcs();            
    bytetrx(0x01);
    bytetrx(s1);
    bytetrx(s2);
    bytetrx(s3);
    endcs();

    waitbusy();    
}

void protectlower32k() {
    char s1=readstatus();
    char s2=readstatus2();
    char s3=readstatus3();

    // SEC=64 TB=32 BP2=0 BP1=8 BP0=4
    s1 = 64+32+8+4;
    
    writestatusregs(s1,s2,s3);        
}

void allowlower32k() {
    char s1=readstatus();
    char s2=readstatus2();
    char s3=readstatus3();

    // SEC=64 TB=32 BP2=0 BP1=8 BP0=4
    s1 = 0;
    
    writestatusregs(s1,s2,s3);        
}


void blockerase(unsigned long addr) {

    // erase 64kbyte
    writeenable();
    
    unsigned char *paddr;
    paddr = (unsigned char *)(&addr);

    startcs();
    bytetrx(0xd8);
    bytetrx(paddr[2]);
    bytetrx(paddr[1]);
    bytetrx(paddr[0]);
    endcs();    
    
    waitbusy();
    
}

void sectorerase(unsigned long addr) {
    // erase 4kbyte
    
    writeenable();
    
    unsigned char *paddr;
    paddr = (unsigned char *)(&addr);

    startcs();
    bytetrx(0x20);
    bytetrx(paddr[2]);
    bytetrx(paddr[1]);
    bytetrx(paddr[0]);
    endcs();
    
    waitbusy();    
}

void eraseDevice() {
    waitbusy();
    allowlower32k(); // this is necessary!
    writeenable();
        
    startcs();        
    bytetrx(0x60);
    endcs();

    waitbusy();
    //while((readstatus()&0x01) == 0);
}

int eraseReady() {
    // check status bit
    
    return (readstatus()&0x01)==0; 
}

unsigned char lis_whoami() {
    __delay_us(D);
    LIS_CS = 1;
    __delay_us(D);

    LIS_CS = 0;
    __delay_us(D);

    unsigned char b;
    unsigned char cmd = 0x0f | 128;
    bytetrx(cmd);
    b= bytetrx(0);
    __delay_us(D);
    LIS_CS = 1;
        
    return b;

}

void lis_cfg() {
    __delay_us(D);
    LIS_CS = 1;
    __delay_us(D);

    LIS_CS = 0;
    __delay_us(D);
    
    __delay_ms(10);


    bytetrx(0x20);
    // bytetrx(0b00100111); // 0010 = 10Hz, 0 - not low power, 111 = all axes enabled
    bytetrx(0b01110111); // 0111 = 400Hz, 0 - not low power, 111 = all axes enabled

    __delay_us(D);
    LIS_CS = 1;
    
}

void lis_hpfilter() {
    __delay_us(D);
    LIS_CS = 1;
    __delay_us(D);

    LIS_CS = 0;
    __delay_us(D);
    
    __delay_ms(10);


    bytetrx(0x21);
    //bytetrx(0b00001000);
    bytetrx(0b00001100); // HP filter enabled also for Click detection

    __delay_us(D);
    LIS_CS = 1;
    
}

void lis_reg23() {
    __delay_us(D);
    LIS_CS = 1;
    __delay_us(D);

    LIS_CS = 0;
    __delay_us(D);
    
    __delay_ms(10);


    bytetrx(0x23);
    bytetrx(0b00001000); // Full scale +/- 2G, endian + high res (endian must select endian)

    __delay_us(D);
    LIS_CS = 1;
    
}


void lis_click_i1click() {
    __delay_us(D);
    LIS_CS = 1;
    __delay_us(D);

    LIS_CS = 0;
    __delay_us(D);
    
    __delay_ms(1);

    bytetrx(0x22);
    bytetrx(1); // enable single click interrupt

    __delay_us(D);
    LIS_CS = 1;
    
}

void lis_click_latch() {
    __delay_us(D);
    LIS_CS = 1;
    __delay_us(D);

    LIS_CS = 0;
    __delay_us(D);
    
    __delay_ms(1);

    bytetrx(0x24);
    bytetrx(8); // enable interrupt latched

    __delay_us(D);
    LIS_CS = 1;
    
}

void lis_click_cgf() {
    __delay_us(D);
    LIS_CS = 1;
    __delay_us(D);

    LIS_CS = 0;
    __delay_us(D);
    
    __delay_ms(1);

    bytetrx(0x38);
    bytetrx(0b00010101); // enable single click detection on all axes

    __delay_us(D);
    LIS_CS = 1;
    
}

void lis_click_ths() {
    __delay_us(D);
    LIS_CS = 1;
    __delay_us(D);

    LIS_CS = 0;
    __delay_us(D);
    
    __delay_ms(1);

    bytetrx(0x3a);
    bytetrx(80); // click threshold

    __delay_us(D);
    LIS_CS = 1;
    
}

void lis_click_timelimit() {
    __delay_us(D);
    LIS_CS = 1;
    __delay_us(D);

    LIS_CS = 0;
    __delay_us(D);
    
    __delay_ms(1);

    // TLI7 through TLI0 define the maximum time interval that can elapse between the start of
    // the click-detection procedure (the acceleration on the selected channel exceeds the
    //programmed threshold) and when the acceleration falls back below the threshold.
    
    bytetrx(0x3b);
    bytetrx(50); // @400Hz ODR

    __delay_us(D);
    LIS_CS = 1;
    
}

unsigned char lis_click_read() {
    __delay_us(D);
    LIS_CS = 1; 
    __delay_us(D);

    LIS_CS = 0;
    __delay_us(D);

    unsigned char d;
    
    bytetrx(0x39 | 128 );
    d = bytetrx(0); 
        
    //plog("acc %i %i %i %i", x, y, z, a);
    
    unsigned click_xyz = d & 7;
    unsigned click_sign = (d & 8) != 0;
    unsigned single_en = (d & 16) != 0;
    unsigned double_en = (d & 32) != 0;
    unsigned int_act = (d & 64) != 0;

    if(d!=0) {
        plog("CLICK xyz %u sign %u en s:%u d:%u int_act %u", click_xyz, click_sign, single_en,
            double_en, int_act);
    }

    __delay_us(D);
    LIS_CS = 1;
    
    return d;    
}


void lis_fifo_en() {
    __delay_us(D);
    LIS_CS = 1;
    __delay_us(D);

    LIS_CS = 0;
    __delay_us(D);
    __delay_ms(1);

    bytetrx(0x24);
    bytetrx(64); // enable FIFO

    __delay_us(D);
    LIS_CS = 1;
    
}

void lis_fifo_ctrl() {
    __delay_us(D);
    LIS_CS = 1;
    __delay_us(D);

    LIS_CS = 0;
    __delay_us(D);
    __delay_ms(1);

    bytetrx(0x2e);
    bytetrx(128); // stream FIFO mode (and threshold set to 0)

    __delay_us(D);
    LIS_CS = 1;
    
}

int lis_read_fifo_src() {
    __delay_us(D);
    LIS_CS = 1; 
    __delay_us(D);

    LIS_CS = 0;
    __delay_us(D);

    int x,y,z;
    
    bytetrx(0x2f | 128);
    unsigned char d = bytetrx(0); 
    
    d = d & 0b11111;  // number of samples in FIFO
    
    __delay_us(D);
    LIS_CS = 1;
    
    //plog("IMU %d", d);
    
    return d;
    
}

int lis_readacc() {
    __delay_us(D);
    LIS_CS = 1; 
    __delay_us(D);

    LIS_CS = 0;
    __delay_us(D);

    int x,y,z;
    
    bytetrx(0x28 | 128 | 64);
    ((unsigned char *)&x)[0] = bytetrx(0); 
    ((unsigned char *)&x)[1] = bytetrx(0); 
    ((unsigned char *)&y)[0] = bytetrx(0); 
    ((unsigned char *)&y)[1] = bytetrx(0); 
    ((unsigned char *)&z)[0] = bytetrx(0); 
    ((unsigned char *)&z)[1] = bytetrx(0); 
        
    x/=32;
    y/=32;
    z/=32;
    
    int a = x+y+z;
    if(a<0) a=-a;
    if(a>250) a=250;
    
    //plog("acc %i %i %i %i", x, y, z, a);
    // plog("acc %i", a);


    __delay_us(D);
    LIS_CS = 1;
    
    return a;
    
}

int lis_read_fifo(int n) {
    int i, d;
    int x[32],y[32],z[32];
    int maxa=0;

    
    __delay_us(D);
    LIS_CS = 1; 
    __delay_us(D);

    LIS_CS = 0;
    __delay_us(D);

    bytetrx(0x28 | 128 | 64);

    if(n>32) n = 32;
    
    for(i=0; i<n; i++) {
        d = bytetrx(0) + (bytetrx(0)<<8);
        x[i] = abs(d / 32);

        d = bytetrx(0) + (bytetrx(0)<<8);
        y[i] = abs(d / 32);
        
        d = bytetrx(0) + (bytetrx(0)<<8);
        z[i] = abs(d / 32);

        //int a = x+y+z;
        
    }
    
    for(i=0; i<16; i++) acc_hist[i]=0;
    
    for(i=0; i<n; i++) {
        int32_t a = x[i]+y[i]+z[i];
        
        if(a>maxa) {
            maxa = a;
        }
                
        acc_hist[i/2] = +a;
        
        //if(a>0) {
        //    plog("FIFO %u: %i %i %i", i, x[i],y[i],z[i]);
        //}
    }
    
    //plog("acc %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", acc_hist[0], acc_hist[1], acc_hist[2], acc_hist[3], acc_hist[4], acc_hist[5], acc_hist[6], acc_hist[7], acc_hist[8], acc_hist[9],
    //        acc_hist[10], acc_hist[11], acc_hist[12], acc_hist[13], acc_hist[14], acc_hist[15]);

    __delay_us(D);
    LIS_CS = 1;
    
    // plog("FIFO ACC %u", maxa);
    
    return maxa;
    
}