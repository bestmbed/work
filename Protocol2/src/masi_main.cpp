#include "masi_uart.h"
#include "masi_proto2.h"
#include <iostream>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>


// masi_proto2 driver_1, driver_2;

using namespace std;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_1 = PTHREAD_MUTEX_INITIALIZER;


pthread_t device_st;


unsigned char rx ,tx,  x;
    using namespace std;
    using namespace sep;


    masi_proto2 driver_1;
    Serialport  serial_1, serial_2;


unsigned char datavelo[9] = {0xF7, 0xF7, 0xFF, 0x00, 0x01, 0x03, 0x00, 0x01, 0x00};
unsigned char data_que;

void *driver_motor_st(void *arg)
{
    while(1){
    serial_1.Serial_read(&rx,1);    //Serial read @port 

    if(rx == 'q')
    {
        break;
    }
        if(read(STDOUT_FILENO,&rx,1)>0){
            // cout << hex << rx ;
            
            serial_1.dequeue(&serial_1.queueHandle,&data_que); //data que to buffer for decode
            driver_1.inspacket(&data_que,&driver_1.insobj_t_device); //decode hearder from packet data
            // cout << "work" << endl;
                if(driver_1.insobj_t_device.end_packet == true)
                {
                    driver_1.masi_updatepacket(&driver_1.insobj_t_device, &driver_1.packetHandle_device);
                    driver_1.insobj_t_device.end_packet = false;
                }
        }
    }
    // pthread_exit(NULL);    
}


int main(int argc, char **argv)
{
serial_1.Open_seport("/dev/ttyUSB0", 115200);
serial_1.queue_init(&serial_1.queueHandle);

    driver_1.tableHandle_device.id = 1;
    driver_1.insobj_t_device.id_device = 1;
pthread_create(&device_st, NULL, driver_motor_st,NULL);
pthread_join(device_st,NULL);





// if(rx == 'w')
// {
    // driver_1.PingPacket(driver_1.tableHandle_device.id, &driver_1.packetHandle_device);
// }



// cout << &driver_1.packetHandle_device << endl;


// if(read(STDOUT_FILENO,&rx,1)>0){}


 
//     driver_1.PingPacket(1,&driver_1.packetHandle_device);
//     serial_1.Serial_write(driver_1.packetHandle_device.ins_packet,driver_1.packetHandle_device.length);
    
//     while(rx != 'q')
//     {
//         // serial_1.Serial_read(&rx,1);
       
//         // cout << rx << endl;
//     // count << "Srialprint " << &rx << end;
        
//    // driver_1.masi_read(driver_1.tableHandle_device.id, ADD_ENABLE_ALL,24, ReadBlockCommand, 0, &driver_1.packetHandle_device);
//    // serial_1.Serial_write(driver_1.packetHandle_device.ins_packet,driver_1.packetHandle_device.length);
   
//     } 
//     serial_1.Close_seport();

    return 0;
}

