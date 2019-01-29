

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
#include "serial_robotic.h"
#include  <sys/queue.h>


  void SemiRS232::SerialPort:: semi_open(const char *port_name,int baud_rate)
  {
	  
	  int speed=0;
   tcgetattr(STDOUT_FILENO, &old_stdio);
   memset(&stdio,0,sizeof(stdio));
   
  stdio.c_cflag = 0;
  stdio.c_iflag = 0;
  stdio.c_oflag = 0;
  stdio.c_lflag = 0;

  stdio.c_cc[VTIME] = 0;
  stdio.c_cc[VMIN]  = 1;
  tcsetattr(STDOUT_FILENO, TCSANOW, &stdio);
  tcsetattr(STDOUT_FILENO, TCSAFLUSH, &stdio);
  fcntl(STDOUT_FILENO, F_SETFL, O_NONBLOCK);
 
                  
  memset(&tio,0,sizeof(tio));
  tio.c_cflag =CS8|CREAD|CLOCAL;
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_lflag = 0;
  tio.c_cc[VTIME] = 0;
  tio.c_cc[VMIN]  = 0;

  fd= open(port_name, O_RDWR|O_NONBLOCK);
 
  
   switch(baud_rate)
   {
	 case 9600: speed= B9600;break;
	 case 57600: speed=B57600;break;
	 case 115200:speed= B115200;break;
	 default : speed=B57600;break;  
	   
	   
   }
  
  cfsetospeed(&tio,speed);
  cfsetispeed(&tio,speed);

  tcsetattr(fd, TCSANOW, &tio);
	  
  }
  
  void SemiRS232::SerialPort :: semi_close(void)
  {
	 
	  close(fd);
      tcsetattr(STDOUT_FILENO, TCSANOW, &old_stdio); 
  }
  
  int SemiRS232::SerialPort:: semi_read(unsigned char *buffer, int length)
  {
	  
	  int err=0;
	  
	  if(read(fd,buffer,length)>0)
	  {
		enqueue(&queueHandle,*buffer);  
	  }
	  return err;
	  
  }
  
   void SemiRS232::SerialPort::semi_write(unsigned char *data, int length )
   {
	   
	  write(fd,data,length);
	   
   }
   
   
  int SemiRS232::SerialPort::queue_init(queuetype *queue)
{
	queue->queuepthead= queue->queuepttail= 0;
	return 1;

}

int SemiRS232::SerialPort:: queue_empty(queuetype *queue)
{

	return  queue->queuepthead==queue->queuepttail;
}

int SemiRS232::SerialPort:: queue_full(queuetype *queue)
{

	return ((queue->queuepttail+1)%QUEUESIZE)==queue->queuepthead;
}

int SemiRS232::SerialPort:: enqueue(queuetype *queue,unsigned char data)
{
	if(queue_full(queue))
		return 0;

	queue->data[queue->queuepttail]=data;
	queue->queuepttail=(queue->queuepttail+1)%QUEUESIZE;
	return 1;
}

int SemiRS232::SerialPort:: dequeue(queuetype *queue,unsigned char *data)
{
     if(queue_empty(queue))
    	return 0;

     *data= queue->data[queue->queuepthead];
     queue->queuepthead = (queue->queuepthead+1)%QUEUESIZE;

     return 1;


}


