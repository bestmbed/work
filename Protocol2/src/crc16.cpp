#include<iostream>

#define POLY                0x8408

using namespace std;

unsigned short crc16(unsigned char *data_p, unsigned short length)
 {
	   unsigned short i;
	   unsigned short data;
	   unsigned short crc = 0xffff;
 
	   if (length == 0)
			 return (~crc);
 
	   do
	   {
			 for (i=0, data=(unsigned short)0xff & *data_p++;  i < 8;  i++, data >>= 1)
			 {
				   if ((crc & 0x0001) ^ (data & 0x0001))
						 crc = (crc >> 1) ^ POLY;
				   else  crc >>= 1;
			 }
	   } while (--length);
 
	   crc = ~crc;
	   data = crc;
	   crc = (crc << 8) | (data >> 8 & 0xff);
    
	   return (crc);
 }


unsigned char ping[] = {0xf7, 0xf7, 0xff, 0x00, 0x01, 0x03, 0x00, 0x01};
unsigned char Tcrc[] = {0xf7, 0xf7, 0xff, 0x00, 0x01, 0x0a, 0x00, 0x03, 0x01, 0x7e, 0x00, 0x79, 0xe9, 0xf6, 0x42};


int main(void)
 {
    cout << "crc16 = " << hex << crc16(ping,8) << endl;
    return 0;
 }
