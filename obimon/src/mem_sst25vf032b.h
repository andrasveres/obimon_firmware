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
void clearblockprotection();
unsigned char readstatus();
void writeenable();
void write(unsigned long addr, unsigned char c);
void writes(unsigned long addr, unsigned char *p, unsigned char n);
void readmem(unsigned long addr, unsigned char* data, int n);
unsigned char getmemid();
void erase();

#endif	/* MEM_H */

