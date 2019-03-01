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
using namespace std;
namespace sep{

    
class Serialport{


    public:
        int fd;
        queuetype queueHandle;
        void Open_seport(const char *port_name, int baudrate);
        void Close_seport(void);
        void Serial_write(unsigned char *data, int length);
        int queue_init(queuetype *queue);
        int Serial_read(unsigned char *buffer, int length);
        int enqueue(queuetype *queue,unsigned char data);
        int dequeue(queuetype *queue,unsigned char *data);
    
    private:

        struct termios tio;
        struct termios stdio;
        struct termios old_stdio;
        int queue_empty(queuetype *queue);
        int queue_full(queuetype *queue);
};

}