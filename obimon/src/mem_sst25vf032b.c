
#include <xc.h>

#include <mem_sst25vf032b.h>
#include <system.h>
#include <libpic30.h>

#define D 100

void initflashspi() {
    InitSPI1(3,3);
}

unsigned long findmem() {
   unsigned char data;
   unsigned long low = 0;
   unsigned long step = 32768*128 / 2;

   unsigned long p;

   do {
      p = low + step;
      readmem(p, &data, 1);

      if(data!=255) low = p;

      if(step == 256) return low;

      step = step / 2;

    } while(1);
}

void writes(unsigned long addr, unsigned char *p, unsigned char n) {
    int i=0;
    for(i=0; i<n; i++) {
        write(addr+i, p[i]);
    }

}


void write(unsigned long addr, unsigned char c) {

    // write 1 byte to memory

    writeenable();
    
        unsigned char b;
        unsigned char *p;

        p = (unsigned char *)(&addr);

        __delay_us(D);
        MEM_CS = 1;
        __delay_us(D);

        MEM_CS = 0;
        __delay_us(D);

        SPI1BUF = 0x02;
        while(SPI1STATbits.SPIRBF==0);
        b = SPI1BUF;

        SPI1BUF = p[2];
        while(SPI1STATbits.SPIRBF==0);
        b = SPI1BUF;

        SPI1BUF = p[1];
        while(SPI1STATbits.SPIRBF==0);
        b = SPI1BUF;

        SPI1BUF = p[0];
        while(SPI1STATbits.SPIRBF==0);
        b = SPI1BUF;

        SPI1BUF = c;
        while(SPI1STATbits.SPIRBF==0);
        b = SPI1BUF;
        
        __delay_us(D);
        MEM_CS = 1;

        //waitbusy(); // TEST

}

void readmem(unsigned long addr, unsigned char* data, int n) {
        unsigned char b;
        int i=0;
        unsigned char *p;

        p = (unsigned char *)(&addr);

        __delay_us(D);
        MEM_CS = 1;
        __delay_us(D);

        MEM_CS = 0;
        __delay_us(D);

        //LED_Toggle(LED_TEST);

        SPI1BUF = 0x03;
        while(SPI1STATbits.SPIRBF==0);
        b = SPI1BUF;

        SPI1BUF = p[2];
        while(SPI1STATbits.SPIRBF==0);
        b = SPI1BUF;

        SPI1BUF = p[1];
        while(SPI1STATbits.SPIRBF==0);
        b = SPI1BUF;

        SPI1BUF = p[0];
        while(SPI1STATbits.SPIRBF==0);
        b = SPI1BUF;

        for(i = 0; i<n; i++) {
           SPI1BUF = 0;
           while(SPI1STATbits.SPIRBF==0);
           b = SPI1BUF;

           data[i] = b;
        }

        __delay_us(D);
        MEM_CS = 1;
}

unsigned char getmemid() {
        unsigned char b;
        __delay_us(D);
        MEM_CS = 1;
        __delay_us(D);

        MEM_CS = 0;
        __delay_us(D);

         SPI1BUF = 0x90;
         while(SPI1STATbits.SPIRBF==0);
         b = SPI1BUF;

         SPI1BUF = 0;
         while(SPI1STATbits.SPIRBF==0);
         b = SPI1BUF;

         SPI1BUF = 0;
         while(SPI1STATbits.SPIRBF==0);
         b = SPI1BUF;

         SPI1BUF = 0;
         while(SPI1STATbits.SPIRBF==0);
         b = SPI1BUF;

         SPI1BUF = 0;
         while(SPI1STATbits.SPIRBF==0);
         b = SPI1BUF;


        __delay_us(D);
        MEM_CS = 1;

        return b;
}

void writeenable() {
        unsigned char b;

        __delay_us(D);
        MEM_CS = 1;
        __delay_us(D);

        MEM_CS = 0;
        __delay_us(D);

         SPI1BUF = 0x06;
         while(SPI1STATbits.SPIRBF==0);
         b = SPI1BUF;

        __delay_us(D);
        MEM_CS = 1;

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

        __delay_us(D);
        MEM_CS = 1;
        __delay_us(D);

        MEM_CS = 0;
        __delay_us(D);

         SPI1BUF = 0x05;
         while(SPI1STATbits.SPIRBF==0);
         b = SPI1BUF;

         SPI1BUF = 0;
         while(SPI1STATbits.SPIRBF==0);
         b = SPI1BUF;


        __delay_us(D);
        MEM_CS = 1;

        return b;
}

void clearblockprotection() {
        unsigned char b;

        __delay_us(D);
        MEM_CS = 1;
        __delay_us(D);

        MEM_CS = 0;
        __delay_us(D);

         SPI1BUF = 0x50;
         while(SPI1STATbits.SPIRBF==0);
         b = SPI1BUF;

        __delay_us(D);
        MEM_CS = 1;
        __delay_us(D);
        MEM_CS = 0;
        __delay_us(D);

         SPI1BUF = 0x01;
         while(SPI1STATbits.SPIRBF==0);
         b = SPI1BUF;

         SPI1BUF = 0;
         while(SPI1STATbits.SPIRBF==0);
         b = SPI1BUF;


        __delay_us(D);
        MEM_CS = 1;

}

void erase() {
        unsigned char b;

        __delay_us(D);
        MEM_CS = 1;
        __delay_us(D);

        MEM_CS = 0;
        __delay_us(D);

         SPI1BUF = 0x60;
         while(SPI1STATbits.SPIRBF==0);
         b = SPI1BUF;

        __delay_us(D);
        MEM_CS = 1;

        while((readstatus()&0x01) == 0);
}
