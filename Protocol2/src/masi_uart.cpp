#include"masi_uart.h"
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <strings.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include  <sys/queue.h>

using namespace sep;
using namespace std;

// #define QUEUESIZE 1000
// typedef struct{
// 	unsigned char data[QUEUESIZE];
// 	int queuepthead;
// 	int queuepttail;

// }queuetype;

// // namespace sep{

// class Serialport{


//     public:
//         int fd;
//         queuetype queueHandle;
//         void Open_seport(const char *port_name, int baudrate);
//         void Close_seport(void);
//         void Serial_write(unsigned char *data, int length);
//         int queue_init(queuetype *queue);
//         int Serial_read(unsigned char *buffer, int length);
//         int enqueue(queuetype *queue,unsigned char data);
//         int dequeue(queuetype *queue,unsigned char *data);
    
//     private:

//         struct termios tio;
//         struct termios stdio;
//         struct termios old_stdio;
//         int queue_empty(queuetype *queue);
//         int queue_full(queuetype *queue);
// };

// // }

void Serialport::Open_seport(const char *port_name, int baudrate)
{
        int speed = 0;

        tcgetattr(STDOUT_FILENO, &old_stdio);
        memset(&stdio, 0, sizeof(stdio));

        stdio.c_iflag=0;
        stdio.c_oflag=0;
        stdio.c_cflag=0;
        stdio.c_lflag=0;
        stdio.c_cc[VMIN]=1;
        stdio.c_cc[VTIME]=0;
        tcsetattr(STDOUT_FILENO,TCSANOW,&stdio);
        tcsetattr(STDOUT_FILENO,TCSAFLUSH,&stdio);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking

        memset(&tio, 0, sizeof(tio));
        tio.c_iflag=0;
        tio.c_oflag=0;
        tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
        tio.c_lflag=0;
        tio.c_cc[VMIN]=1;
        tio.c_cc[VTIME]=5;

        Serialport::fd = open(port_name, O_RDWR|O_NONBLOCK);

        switch(baudrate)
        {
                case 9600:      speed = B9600;break;
                case 57600:     speed = B57600;break;
                case 115200:    speed = B115200;break;
                default :       speed = B115200;break;
        }

        cfsetospeed(&tio, speed);
        cfsetispeed(&tio, speed);
        tcsetattr(Serialport::fd, TCSANOW, &tio);
        
}

void Serialport::Close_seport(void)
{
        close(fd);
        tcsetattr(STDOUT_FILENO, TCSANOW, &old_stdio);

}



int Serialport::Serial_read(unsigned char *buffer, int length)
{
        int err=0;
	  
	  if(read(fd,buffer,length)>0)
	  {
                  cout <<buffer << endl;
		enqueue(&queueHandle,*buffer);  
                
	  }
	  return err;
	  

}
void Serialport::Serial_write(unsigned char *data, int length)
{
        write(fd, data, length);
}

int Serialport::queue_init(queuetype *queue)
{
	queue->queuepthead= queue->queuepttail= 0;
	return 1;

}

int Serialport::queue_empty(queuetype *queue)
{

	return  queue->queuepthead==queue->queuepttail;
}

int Serialport::queue_full(queuetype *queue)
{
	return ((queue->queuepttail+1)%QUEUESIZE)==queue->queuepthead;
}

int Serialport::enqueue(queuetype *queue,unsigned char data)
{
	if(queue_full(queue))
		return 0;

	queue->data[queue->queuepttail]=data;
	queue->queuepttail=(queue->queuepttail+1)%QUEUESIZE;
	return 1;
}

int Serialport::dequeue(queuetype *queue,unsigned char *data)
{
     if(queue_empty(queue))
    	return 0;
}


        
// Serialport  serial_1, serial_2;


// unsigned char datavelo[9] = {0xF7, 0xF7, 0xFF, 0x00, 0x01, 0x11, 0x00, 0x1E, 0x00};


// int main(int argc, char **argv)
// {
//     serial_1.Open_seport("/dev/ttyUSB0", 115200);
//     serial_1.queue_init(&serial_1.queueHandle);
//     // seruab.PingPacket(1,&driver_1.packetHandle_servo);
//     serial_1.Serial_write(datavelo,sizeof(datavelo));
//     serial_1.Close_seport();
//     return 0;
// }