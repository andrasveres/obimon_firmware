/* 
 * File:   mem.h
 * Author: av
 *
 * Created on 2014. május 31., 16:59
 */

#ifndef MEM_H32
#define	MEM_H32

void initflashspi();
int waitbusy();
unsigned long findmem();
//void clearblockprotection();
unsigned char readstatus();
void writeenable();
void pageprogram(unsigned long addr, unsigned char *p, unsigned int n);
void readmem(unsigned long addr, unsigned char* data, int n);

void readjedecid(unsigned char *manufacturer, unsigned char *device);
void blockerase(unsigned long addr);
void sectorerase(unsigned long addr);
void eraseDevice();
int eraseReady();
void protectlower32k();
void allowlower32k();
unsigned char readstatus2();
unsigned char readstatus3();

#endif	/* MEM_H */

