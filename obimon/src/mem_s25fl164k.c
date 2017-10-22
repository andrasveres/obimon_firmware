
#include <xc.h>

#include "mem_s25fl164k.h"
#include <system.h>
#include <libpic30.h>

#define D 10

void initflashspi() {
    InitSPI1(0,0);
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
        if(s & 2 == 0) plog("NOT WRITE ENABLED");
        plog("WE %x", s);
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

