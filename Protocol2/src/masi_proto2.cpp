#include<iostream>
#include"masi_proto2.h"
#include"masi_uart.h"
#include <string>
// #include <boost/asio.hpp>
#include <boost/array.hpp>

#define HEADER_1            0xF7
#define HEADER_2            0xF7
#define HEADER_3            0xFF
#define RESERVED            0x00
#define BROADCAST_ID        0xFB
#define POLY                0x8408

//address of station control
#define ADD_MODEL_NUMBER                        0 
#define ADD_VERSION_FIRMWARE                    2            
#define ADD_ID                                  6
#define ADD_BAUD_RATE                           7
#define ADD_RETURN_DELAY_TIME                   11
#define ADD_OPERATION_MODE                      12
#define ADD_ENABLE_ALL                          13
#define ADD_WHEEEL_MODE                         14
#define ADD_GOAL_JOG                            15
#define ADD_MOTOTFLIPPER                        23
#define ADD_YAW_AXIS                            27
#define ADD_PITCH_AXIS                          31
#define ADD_ROLL_AXIS                           35
#define ADD_PPOS_EMCODER_FLIP                   39
#define ADD_PPOS_ENCODER_ML                     43
#define ADD_PPOS_ENCODER_MR                     47
#define ADD_TEMPERATURE_1                       51
#define ADD_TEMPERATURE_2                       53
#define ADD_TEMPERATURE_3                       55
#define ADD_TEMPERATURE_4                       57
#define ADD_TEMPERATURE_5                       59
#define ADD_LED_1                               61
#define ADD_LED_2                               62
#define ADD_LED_3                               63

/**********************************************************************/

//address of driver motor
/*


teseCRC**  
unsigned char dataTCRC[] = {0xF7,0xF7,0xFF,0x00,0x01,0x0A,0x00,0x03,0x01,0x7E,0x00,0x79,0xE9,0xF6,0x42,0xCC,0x41}

*/

/**********************************************************************/


// struct _PB_PACKET_
// {
//     unsigned short length;
//     unsigned short start_address;
//     unsigned char  ins_packet[250];
//     unsigned char  enable_send;
//     unsigned char user_command;
//     unsigned char  continous_flag;
//     unsigned char  error_return;
//     unsigned char  data_length;
// };


// struct _PB_INSTRUCT_
// {
//     long process_index;
//     long count_pass;
//     long count_fail;
//     unsigned short  count_packet;
//     unsigned short  count_timeout;       
//     unsigned char   end_packet;
//     unsigned char   start_timeout;
//     unsigned char   id;
//     unsigned char   length_low;
//     unsigned char   length_hight;
//     unsigned char   flash_end_header;
//     unsigned char   end_of_data;
//     unsigned char   end_of_header;
//     unsigned char   flag_header;
//     unsigned char   instruction;
//     unsigned char   lock_length;
//     unsigned char   length;
//     unsigned char   buffer_packet[250];
//     unsigned char   id_servo;
// };

// enum{
//    disable_ta_packet=0,
//    enable_tx_packet=1	

// };
// enum
// {
//     Ping,                   //1
//     Read,                   //2
//     Write,                  //3
//     Status,                 //4
//     Factoryvalue,           //5
//     Restart,                //6
//     Writeblock              //7
// };

// typedef enum{
//    DisableCommand=0,
//    PingCommand=1,
//    WriteCommand=2,
//    ReadCommand=3,
//    ResetFactoryCommand=4,
//    RestartCommand=5,
//    WriteRegisterCommand=6,
//    ReadContinuousCommand=7,
//    WriteBlockCommand=8,
//    ReadBlockCommand=9,
//    DataFail=10
   

// }USER_COMMAN;


// class masi_proto2{

//     public:
//         unsigned char PingPacket(unsigned char id,struct _PB_PACKET_ *packetHandle);
//         int masi_write(uint8_t id, uint16_t address, uint16_t length, uint8_t *data);
//         int masi_read(uint8_t id, uint16_t address, uint16_t length, uint8_t *data);
//         struct _PB_PACKET_ packetHandle_servo;

//     private:
//         int inspacket(unsigned char *rx_pack, struct _PB_INSTRUCT_ *insobj_t);
//         int transmission(int portUSB);
//         unsigned short crc16(unsigned char *data_p, unsigned short length);
// };

using namespace std;


int masi_proto2::inspacket(unsigned char *rx_pack, struct _PB_INSTRUCT_ *insobj_t)
{
/************************* check header *************************/

    unsigned char ch; //variable local use to 
    unsigned int count = 0;
    unsigned char Llock_length = 0;
    char flag_packet_found = 0;

    // cout << insobj_t->flag_header << endl;

    if(*rx_pack == HEADER_1 && insobj_t->flag_header == 0)
    {
        insobj_t->flag_header = 1;
        cout << "H1" << endl;
    }
    else if(*rx_pack == HEADER_2 && insobj_t->flag_header == 1)
    {
        insobj_t->flag_header = 2;
        cout << "H2" << endl;
    }
    else if(*rx_pack == HEADER_3 && insobj_t->flag_header == 2)
    {
        insobj_t->flag_header = 3;
        cout << "H3" << endl;
    }
    else if(*rx_pack == RESERVED && insobj_t->flag_header == 3)
    {
        insobj_t->end_of_header = 1;
        insobj_t->flag_header = 0;
        cout << insobj_t->end_of_header << endl;
    }
    
    else
    {
        insobj_t->flag_header = 0;
    }

/************************* Start collecting data *************************/

    if(insobj_t->end_of_header == 1 && insobj_t->end_of_data == 0)
    {
        insobj_t->end_of_data = 1;
    }
    else if(insobj_t->end_of_data == 1)
    {
        count = insobj_t->count_packet;
        ch = *rx_pack;
        insobj_t->buffer_packet[4+count] = ch;
        Llock_length = insobj_t->lock_length;

        switch(Llock_length)
        {
            case 0: 
                    if(count == 1)
                    {
                        insobj_t->length_low = insobj_t->buffer_packet[5];

                    }
                    else if(count == 2)
                    {   
                        insobj_t->length_hight = insobj_t->buffer_packet[6];
                        insobj_t->length = insobj_t->length_hight | insobj_t->length_low;
                        Llock_length = 1;
                    }break;  
                    
            case 1:
                    if(count >= (insobj_t->length+2))
                    {
                        insobj_t->buffer_packet[0] = HEADER_1;
                        insobj_t->buffer_packet[1] = HEADER_2;
                        insobj_t->buffer_packet[2] = HEADER_3;
                        insobj_t->buffer_packet[3] = RESERVED;
                        insobj_t->end_of_data = 0;
                        insobj_t->lock_length = 0;

                        for(int i = 0; i<4; i++)
                        {
                            cout << insobj_t->buffer_packet[i];
                        }
                    }break;

        }
        
        if(insobj_t->end_of_data == 0)
        {
            insobj_t->end_packet = 1;
            Llock_length = 0;
            count = 0;
        }

        if(flag_packet_found != 0 && insobj_t->end_of_data == 1 )
        {
            count++;
        }
        else if(flag_packet_found == 0 && insobj_t->end_of_data == 1)
        {
            count                   = 0;
            Llock_length            = 0;
            insobj_t->end_of_data   = 0;
            insobj_t->count_packet  = 0;
            insobj_t->lock_length   = 0;
            insobj_t->buffer_packet[0] = 0x00;
            insobj_t->buffer_packet[1] = 0x00;
            insobj_t->buffer_packet[2] = 0x00;
            insobj_t->buffer_packet[3] = 0x00;
            insobj_t->count_fail++;
        }
        insobj_t->lock_length = Llock_length;
        insobj_t->count_packet = count;
    }

    return 0;
}

unsigned short masi_proto2::crc16(unsigned char *data_p, unsigned short length)
 {
	   unsigned short i;
	   unsigned short data;
	   unsigned short crc = 0xffff;
 

	   if (length == 0){
			 return (~crc);
       }
 
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


unsigned char masi_proto2::PingPacket(unsigned char id,struct _PB_PACKET_ *packetHandle)
 {
	unsigned char low_byte=0,high_byte=0;
	unsigned short crc;

	packetHandle->length =10;
	packetHandle->ins_packet[0]=HEADER_1;
	packetHandle->ins_packet[1]=HEADER_2;
	packetHandle->ins_packet[2]=HEADER_3;
	packetHandle->ins_packet[3]= RESERVED;
	packetHandle->ins_packet[4]= id;
	packetHandle->ins_packet[5]=0x03;
	packetHandle->ins_packet[6]=0x00;
	packetHandle->ins_packet[7]=Ping;
	crc = crc16(packetHandle->ins_packet,8);
	low_byte = crc & 0x00FF;
	high_byte = (crc & 0xFF00)>>8;
	packetHandle->ins_packet[8]=low_byte;
	packetHandle->ins_packet[9]=high_byte;
	packetHandle->start_address=0x00;
	packetHandle->user_command=PingCommand;
	packetHandle->enable_send=enable_tx_packet;

	return 0;
}


int masi_proto2::masi_write(unsigned char id,unsigned short address, unsigned short num_byte, unsigned char *data,struct _PB_PACKET_ *packetHandle)
{
    unsigned char i , low_byte = 0 , high_byte = 0;
    unsigned short crc;


    if(num_byte == 1 || num_byte == 2 || num_byte == 4)
    {
    packetHandle->ins_packet[0] = HEADER_1;
    packetHandle->ins_packet[1] = HEADER_2;
    packetHandle->ins_packet[2] = HEADER_3;
    packetHandle->ins_packet[3] = RESERVED;
    packetHandle->ins_packet[4] = id;
    packetHandle->ins_packet[5] = 5+num_byte;
    packetHandle->ins_packet[6] = 0x00;
    packetHandle->ins_packet[7] = Write;
    low_byte = address >> 8 & 0x00ff;
    high_byte = (crc & 0xff00)>>8;
    packetHandle->ins_packet[8] = low_byte;
    packetHandle->ins_packet[9] = high_byte;

    for(i = 0; i<num_byte; i++)
    {
        packetHandle->ins_packet[10+i] = data[i];
    }

   crc = crc16(packetHandle->ins_packet,10+num_byte);
   low_byte = crc & 0x00FF;
   high_byte = (crc & 0xFF00)>>8;
   packetHandle->ins_packet[10+num_byte]=low_byte;
   packetHandle->user_command=WriteCommand;
   packetHandle->enable_send=enable_tx_packet;
   return 0;
   }
   else{
       return 1;
   }
}

int masi_proto2::masi_read(unsigned char id, unsigned short address,unsigned short data_length, 
                                           unsigned char command,unsigned char mode_filed,struct _PB_PACKET_ *packetHandle)
{
    unsigned char low_byte = 0, high_byte = 0;
    unsigned short crc;

    packetHandle->ins_packet[0] = HEADER_1;
    packetHandle->ins_packet[1] = HEADER_2;
    packetHandle->ins_packet[2] = HEADER_3;
    packetHandle->ins_packet[3] = RESERVED;
    packetHandle->ins_packet[4] = id;
    packetHandle->ins_packet[5] = 0x07;
    packetHandle->ins_packet[6] = 0x00;
    packetHandle->ins_packet[7] = Read;
    packetHandle->start_address = address;
    low_byte                    = data_length >> 8 & 0xff;
    high_byte                   = data_length & 0xff;
    packetHandle->ins_packet[8] = low_byte;
    packetHandle->ins_packet[9] = high_byte;
    packetHandle->data_length   = data_length;
    low_byte                    = data_length >> 8 &0xff;
    high_byte                   = data_length >> 8;
    packetHandle->ins_packet[10] = low_byte;
    packetHandle->ins_packet[11] = high_byte;
    crc = crc16(packetHandle->ins_packet,12);
    low_byte = crc & 0x00ff;
    high_byte = (crc & 0xff00)>>8;
    packetHandle->ins_packet[12] = low_byte;
    packetHandle->ins_packet[13] = high_byte;
    packetHandle->user_command   = command;
    packetHandle->enable_send   = enable_tx_packet;
	//packetHandle->mode_filed    = mode_filed;

return 0;

}

int masi_proto2::masi_updatepacket(struct _PB_INSTRUCT_ *insobj_t, struct _PB_PACKET_ *packetHandle)
{
    unsigned short crc = 0;
    unsigned short crc_low = 0;
    unsigned char  length=0;
    unsigned char low_byte = 0;
    unsigned char higbyte = 0;
    unsigned short ID = 0;
    unsigned char INS=0;
    unsigned char ERR=0;
    unsigned short length_data = 0;
    unsigned short crc_cal = 0;



    ID = insobj_t->buffer_packet[4];
	// cout << "Ma" << endl;
	cout << insobj_t->buffer_packet[4] << endl;

    if(ID == tableHandle_device.id)
    {
        crc_low = insobj_t->buffer_packet[(insobj_t->length+7)-2];  
        crc     = insobj_t->buffer_packet[((insobj_t->length+7)-2)+1];
        crc <<= 8;
        crc |= crc_low;
        length = ((insobj_t->length+7)-2);
        length_data = insobj_t->length - 3;
        crc_cal = crc16(insobj_t->buffer_packet, length);
        INS = insobj_t->buffer_packet[7];
        ERR = insobj_t->buffer_packet[8];


            if(crc_cal == crc)
            {
                if (ERR == Pass)
                {
                    if(INS == Status)
                    {
                        if(packetHandle->user_command == ReadCommand || packetHandle->user_command == PingCommand || packetHandle->user_command == ReadBlockCommand)
                        {
                            if(packetHandle->user_command == ReadCommand)
                            {

                            }
                            else if(packetHandle->user_command == PingCommand)
                            {

                                insobj_t->buffer_packet[8] = 0x09;
                                insobj_t->buffer_packet[9] = 0x03;
                                tableHandle_device.model_number = make2uint(insobj_t->buffer_packet[8],insobj_t->buffer_packet[9]);
                                tableHandle_device.version_firmware = insobj_t->buffer_packet[10];
                            }
                            else if(packetHandle->user_command == ReadBlockCommand)
                            {
                                tableHandle_device.Linear_X = make2uint(insobj_t->buffer_packet[14], insobj_t->buffer_packet[15]);
                                tableHandle_device.Linear_Y = make2uint(insobj_t->buffer_packet[16], insobj_t->buffer_packet[17]);
                                tableHandle_device.Linear_Z = make2uint(insobj_t->buffer_packet[18], insobj_t->buffer_packet[19]);
                                tableHandle_device.Angular_X = make2uint(insobj_t->buffer_packet[20], insobj_t->buffer_packet[21]);
                                tableHandle_device.Angular_Y = make2uint(insobj_t->buffer_packet[22], insobj_t->buffer_packet[23]);
                                tableHandle_device.Angular_Z = make2uint(insobj_t->buffer_packet[24], insobj_t->buffer_packet[25]);
                                tableHandle_device.Flipper_Goal_Pos = make2float(insobj_t->buffer_packet[26], insobj_t->buffer_packet[27], insobj_t->buffer_packet[28], insobj_t->buffer_packet[29]);
                                tableHandle_device.Flipper_Goal_Vel = make2float(insobj_t->buffer_packet[30], insobj_t->buffer_packet[31], insobj_t->buffer_packet[32], insobj_t->buffer_packet[33]);
                                tableHandle_device.Yaw_Axis_IMU = make2float(insobj_t->buffer_packet[34], insobj_t->buffer_packet[35], insobj_t->buffer_packet[36], insobj_t->buffer_packet[37]);
                                tableHandle_device.Pitch_Axis_IMU = make2float(insobj_t->buffer_packet[38], insobj_t->buffer_packet[39], insobj_t->buffer_packet[40], insobj_t->buffer_packet[41]);
                                tableHandle_device.Roll_Axis_IMU = make2float(insobj_t->buffer_packet[42], insobj_t->buffer_packet[43], insobj_t->buffer_packet[44], insobj_t->buffer_packet[45]);
                                tableHandle_device.Present_position_encoder_flip = make2float(insobj_t->buffer_packet[46], insobj_t->buffer_packet[47], insobj_t->buffer_packet[48], insobj_t->buffer_packet[49]);
                                tableHandle_device.Present_position_encoder_MotorL = make2float(insobj_t->buffer_packet[50], insobj_t->buffer_packet[51], insobj_t->buffer_packet[52], insobj_t->buffer_packet[53]);
                                tableHandle_device.Present_position_encoder_MotorR = make2float(insobj_t->buffer_packet[54], insobj_t->buffer_packet[55], insobj_t->buffer_packet[56], insobj_t->buffer_packet[57]);
                                tableHandle_device.Arm_Joint_1 = make2float(insobj_t->buffer_packet[58], insobj_t->buffer_packet[59], insobj_t->buffer_packet[60], insobj_t->buffer_packet[61]);
                                tableHandle_device.Arm_Joint_2 = make2float(insobj_t->buffer_packet[62], insobj_t->buffer_packet[63], insobj_t->buffer_packet[64], insobj_t->buffer_packet[65]);
                                tableHandle_device.Arm_Joint_3 = make2float(insobj_t->buffer_packet[66], insobj_t->buffer_packet[67], insobj_t->buffer_packet[68], insobj_t->buffer_packet[69]);
                                tableHandle_device.Arm_Joint_4 = make2float(insobj_t->buffer_packet[70], insobj_t->buffer_packet[71], insobj_t->buffer_packet[72], insobj_t->buffer_packet[73]);
                                tableHandle_device.Arm_Joint_5 = make2float(insobj_t->buffer_packet[74], insobj_t->buffer_packet[75], insobj_t->buffer_packet[76], insobj_t->buffer_packet[77]);
                                tableHandle_device.Arm_Joint_6 = make2float(insobj_t->buffer_packet[78], insobj_t->buffer_packet[79], insobj_t->buffer_packet[80], insobj_t->buffer_packet[81]);
                                tableHandle_device.Gripper = make2float(insobj_t->buffer_packet[82], insobj_t->buffer_packet[83], insobj_t->buffer_packet[84], insobj_t->buffer_packet[85]);
                                tableHandle_device.Arm_XY = make2float(insobj_t->buffer_packet[86], insobj_t->buffer_packet[87], insobj_t->buffer_packet[88], insobj_t->buffer_packet[89]);
                                tableHandle_device.Swing_Arm = make2float(insobj_t->buffer_packet[90], insobj_t->buffer_packet[91], insobj_t->buffer_packet[92], insobj_t->buffer_packet[93]);
                                tableHandle_device.Gripper_Torque_limit = make2float(insobj_t->buffer_packet[94], insobj_t->buffer_packet[95], insobj_t->buffer_packet[96], insobj_t->buffer_packet[97]);
                                tableHandle_device.Arm_Joint_lncrement = make2float(insobj_t->buffer_packet[98], insobj_t->buffer_packet[99], insobj_t->buffer_packet[100], insobj_t->buffer_packet[101]);
                                tableHandle_device.Arm_XY_Increment = make2float(insobj_t->buffer_packet[102], insobj_t->buffer_packet[103], insobj_t->buffer_packet[104], insobj_t->buffer_packet[105]);
                                tableHandle_device.Front_Camera_Pan = make2float(insobj_t->buffer_packet[106], insobj_t->buffer_packet[107], insobj_t->buffer_packet[108], insobj_t->buffer_packet[109]);
                                tableHandle_device.P_Camera = make2float(insobj_t->buffer_packet[110], insobj_t->buffer_packet[111], insobj_t->buffer_packet[112], insobj_t->buffer_packet[113]);
                                tableHandle_device.T_Camera = make2float(insobj_t->buffer_packet[114], insobj_t->buffer_packet[115], insobj_t->buffer_packet[116], insobj_t->buffer_packet[117]);
                                tableHandle_device.Z_Camera = make2float(insobj_t->buffer_packet[118], insobj_t->buffer_packet[119], insobj_t->buffer_packet[120], insobj_t->buffer_packet[121]);
                                
                            }

                            packetHandle->error_return = Pass;
                            
                        }
                    }
                }
            }
            else
            {
                packetHandle->error_return = CRC_USER;
            }
    }
    else
    {		
        packetHandle->error_return = CRC_USER;
    }


return 0;
}

unsigned short masi_proto2::make2uint(unsigned char first_byte, unsigned char second_byte)
{
  unsigned short data;

    data = second_byte;
    data <<=8;
    data |=first_byte;

    return data;
}

unsigned char masi_proto2::uint2byte(unsigned short number, unsigned char *data_byte)
{
   *(data_byte) = number &0x00FF;//low_byte
   *(data_byte+1)= (number &0xFF00)>>8;

   return 0;

}

float masi_proto2::make2float(unsigned char first_byte, unsigned char second_byte, unsigned char third_byte, unsigned char fourth_byte)
{
  union{
   unsigned char data_int[4];
   float   data_float;

  }u;



     u.data_int[0]=first_byte;
     u.data_int[1]=second_byte;
     u.data_int[2]=third_byte;
     u.data_int[3]=fourth_byte;

   return u.data_float;

}


unsigned char masi_proto2::float2byte(float number, unsigned char *data_byte)
{
    unsigned char i;
    union{
      unsigned char data_int[4];
      float   data_float;

     }u;

      u.data_float = number;

      for(i=0;i<4;i++)
       *(data_byte+i)=u.data_int[i];


     return 0;
}
