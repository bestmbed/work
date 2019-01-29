

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
#include <sys/queue.h>



#define QUEUESIZE 1000
typedef struct{
	unsigned char data[QUEUESIZE];
	int queuepthead;
	int queuepttail;

}queuetype;


namespace SemiRS232{
	
	
	class SerialPort{
		 
		   public:
		     int fd;
		   void  semi_open(const char *port_name,int baud_rate=115200);
		   void  semi_close(void);
		   int  semi_read(unsigned char *buffer, int length);
		   void semi_write(unsigned char *data, int length );
		   queuetype queueHandle;
           int queue_init(queuetype *queue);
		    int  enqueue(queuetype *queue,unsigned char data);
			int dequeue(queuetype *queue,unsigned char *data);
			
		   private:
		   
            struct termios stdio, old_stdio;
            struct termios tio;
           
			
			int queue_empty(queuetype *queue);
			int queue_full(queuetype *queue);
			

         
           
    
    };	
	
	
	
	
}
