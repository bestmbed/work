#ifndef _MASI_PROTO2_H_
#define _MASI_PROTO2_H_

#include <iostream>
#include <string>
// #include <boost/asio.hpp>
#include <boost/array.hpp>

using namespace std;
// typedef unsigned char   uint8_t
// typedef unsigned int    uint16_t
   

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

*/

/**********************************************************************/
struct _PB_DEVICE_TABLE_
{
unsigned short	model_number;
long	version_firmware;
unsigned char	id;
long	Baud_rate;
unsigned char	return_delay_time;
unsigned char	operation_mode;
unsigned char	Enable_all;
int	Linear_X;
int	Linear_Y;
int	Linear_Z;
int	Angular_X;
int	Angular_Y;
int	Angular_Z;
float	Flipper_Goal_Pos;
float	Flipper_Goal_Vel;
float	Yaw_Axis_IMU;
float	Pitch_Axis_IMU;
float	Roll_Axis_IMU;
float	Present_position_encoder_flip;
float	Present_position_encoder_MotorL;
float	Present_position_encoder_MotorR;
float	Arm_Joint_1;
float	Arm_Joint_2;
float	Arm_Joint_3;
float	Arm_Joint_4;
float	Arm_Joint_5;
float	Arm_Joint_6;
float	Gripper;
float	Arm_XY;
float	Swing_Arm;
float	Gripper_Torque_limit;
float	Arm_Joint_lncrement;
float	Arm_XY_Increment;
float	Front_Camera_Pan;
float	P_Camera;
float	T_Camera;
float	Z_Camera;
int 	A2D_Port_1;
int 	A2D_Port_2;
int 	A2D_Port_3;
int 	A2D_Port_4;
int 	A2D_Port_5;
unsigned char	Digital_Port_1;
unsigned char	Digital_Port_2;
unsigned char	Digital_Port_3;
unsigned char	Digital_Port_4;
unsigned char	Battery_level;
unsigned char	Wheel_Encoder_Activate;
unsigned char	init_flipper_value;
};

struct _PB_PACKET_
{
    unsigned short length;
    unsigned short start_address;
    unsigned char  ins_packet[250];
    unsigned char  enable_send;
    unsigned char user_command;
    unsigned char  continous_flag;
    unsigned char  error_return;
    unsigned char  data_length;
};


struct _PB_INSTRUCT_
{
    long process_index;
    long count_pass;
    long count_fail;
    unsigned short  count_packet;
    unsigned short  count_timeout;       
    unsigned char   end_packet;
    unsigned char   start_timeout;
    unsigned char   length_low;
    unsigned char   length_hight;
    unsigned char   flash_end_header;
    unsigned char   end_of_data;
    unsigned char   end_of_header;
    unsigned char   flag_header;
    unsigned char   instruction;
    unsigned char   lock_length;
    unsigned char   length;
    unsigned char   buffer_packet[250];
    unsigned char   id_device;
};

enum{
   disable_tx_packet=0,
   enable_tx_packet=1	

};

typedef enum{ 
    Pass=0,
	 CRC_USER=1,
	 ErrorAccress=2,
	 ErrorLimitMemory=3,
	 ErrorRangeData=4,
	 OverWritePage=5,
	 OverReadPage=6,
	 ErrorINS=7,
	 ErrorDataLength=8,
	 VolateMaxError=9,
	 VolateMinError=10,
	 CurrentOverLoad=11,
	 HighTemperature=12,
	 HardwareReset=13,
	 HighLowFrequency=14
    }MessageError;

    
enum
{
    Ping,                   //1
    Read,                   //2
    Write,                  //3
    Status,                 //4
    Factoryvalue,           //5
    Restart,                //6
    Writeblock              //7
};


typedef enum{
   DisableCommand=0,
   PingCommand=1,
   WriteCommand=2,
   ReadCommand=3,
   ResetFactoryCommand = 4,
   RestartCommand = 5,
   WriteRegisterCommand=6,
   ReadContinuousCommand=7,
   WriteBlockCommand=8,
   ReadBlockCommand=9,
   DataFail=10

}USER_COMMAN;


class masi_proto2{

    public:
        unsigned char PingPacket(unsigned char id,struct _PB_PACKET_ *packetHandle);
        int masi_write(unsigned char id,unsigned short address, unsigned short num_byte, unsigned char *data,struct _PB_PACKET_ *packetHandle);
        int masi_read(unsigned char id, unsigned short address,unsigned short data_length, unsigned char command,unsigned char mode_filed,struct _PB_PACKET_ *packetHandle);
        int masi_updatepacket(struct _PB_INSTRUCT_ *insobj_t, struct _PB_PACKET_ *packetHandle);
        int inspacket(unsigned char *rx_pack, struct _PB_INSTRUCT_ *insobj_t);
        struct _PB_PACKET_          packetHandle_device;
        struct _PB_INSTRUCT_        insobj_t_device;
        struct _PB_DEVICE_TABLE_    tableHandle_device;    
         unsigned short make2uint(unsigned char first_byte, unsigned char second_byte);
        unsigned char uint2byte(unsigned short number, unsigned char *data_byte);
        unsigned char float2byte(float number, unsigned char *data_byte);
        float make2float(unsigned char first_byte, unsigned char second_byte, unsigned char third_byte, unsigned char fourth_byte);
    
    private:
        
     
        unsigned short crc16(unsigned char *data_p, unsigned short length);
       
    
};


#endif
