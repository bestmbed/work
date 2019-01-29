/*
 * HAL_DriveBot.c
 *
 *  Created on: 9 �.�. 2560
 *      Author: User
 */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "HAL_DriveBot.h"
#include <iostream>


#define HEADER_1     0xF7
#define HEADER_2     0xF7
#define HEADER_3     0xFF
#define RESERVED     0x00
#define BROADCAST_ID  0xFB //251
#define PI 3.14159265359
#define POLY 0x8408
#define POSTIVE(n)  ((n) > 0 ? 0 + (n) : -(n))



/*********Index Factory reset*********/
//@uint16_b ResetFactoryPacket(uint8_b id, uint8_b index_factory)
#define RESET_FACTORY_ALL 0xF0;
#define RESET_FACTORY_EXP_ID 0xF1
#define RESET_FACTORY_EXP_ID_BAUD 0xF2
/*********************************/




/*********Index Write Register*********/
 //@ uint16_b WriteRegisterPacket(uint8_b id, uint8_b index_reg)
     //@ 0 -  Write Kp Position
     //@ 1 -  Write Ki Position
     //@ 2 -  Write Kd Position
     //@ 3 -  Write Kp Velocity 
     //@ 4 -  Write Ki Velocity 
     //@ 5 -  Write Kd Velocity 
     //@ 6 -  Write Kp Current 
     //@ 7 -  Write Ki Current  
     //@ 8 -  Write Kd Current 
     //@ 9 -  Write All Gain Position
     //@ 10 - Write All Gain Velocity
     //@ 11 - Write All Gain Current
     //@ 12 - Write All Gain PID Loop 

#define REG_KP_POS 0
#define REG_KI_POS 1
#define REG_KD_POS 2
#define REG_KP_VEL 3
#define REG_KI_VEL 4
#define REG_KD_VEL 5
#define REG_KP_CUR 6
#define REG_KI_CUR 7
#define REG_KD_CUR 8
#define REG_ALL_POS 9
#define REG_ALL_VEL 10
#define REG_ALL_CUR 11
#define REG_ALL 12

/*********************************/
/********Variable user define****/



int  state_update_packet=0;



int MyDriverDCServo::InsPacket(unsigned char *rx_byte_ch,struct _STR_INSTRUCTION_ *insObj_t)
{

	unsigned char ch;
	unsigned int count=0;
	unsigned char Llock_length=0;
	char  flag_packet_found=0;

     
		      flag_packet_found= SearchHeader(*rx_byte_ch, insObj_t);
		    //if(flag_packet_found==0){task[0]++;}
		  //   else{task[1]++;}
		      
		  // printf("[%llu] Sink HEADER: %d       FOUND   %llu            MISSING   %llu \n\r",task[3]++,flag_packet_found,task[0],task[1]);
		     
		       if(flag_packet_found==0&&insObj_t->end_of_data==0)// found header packet
		       {
				 insObj_t->end_of_data=1; //Start next  data to buffer xxx 
             
			   }
			 // printf("%d\n\r",insObj_t->end_of_data);
			
			   else if(insObj_t->end_of_data==1)// New found header packet
			   	{ 
				 // printf("loop new found : %llu\n\r",task[4]++);
				   count= insObj_t->count_packet;
				   ch= *rx_byte_ch;
				   insObj_t->buffer_packet[4+count]= ch; //save data to buffer
		           Llock_length= insObj_t->lock_length;
                    switch(Llock_length)
                    {
                      case 0:
                               if(count==1)
                               insObj_t->length_low = insObj_t->buffer_packet[5]; //Low byte of length
                               else if(count==2)
                               {
                                 insObj_t->length  =  insObj_t->buffer_packet[6];  //High byte of length
                                 insObj_t->length  <<= 8;
                                 insObj_t->length |=insObj_t->length_low;
                                 Llock_length=1;
                                 
                                 // printf("length packet : %d     %llu\n\r",insObj_t->length,task[4]++);
              
                   			   } 
                               break;              
                      case 1:
                                if(count >= (insObj_t->length+2)) 
                    			{
							      insObj_t->buffer_packet[0]=HEADER_1;
							      insObj_t->buffer_packet[1]=HEADER_2;
							      insObj_t->buffer_packet[2]=HEADER_3;
							      insObj_t->buffer_packet[3]=RESERVED;
                                  insObj_t->end_of_data  =  0 ;
								  insObj_t->lock_length  =  0;
								
							
				 			    }
 								break;
                      }

 					  
                        if( insObj_t->end_of_data==0)
                          {
							//printf("packet complete:%llu    count: %d\n\r   ",task[4]++,count);
							insObj_t->end_packet =1;
                            Llock_length=0;
                            count=0;
                          }
                         
                         
                         
                         if( flag_packet_found!=0 && insObj_t->end_of_data==1)// true found before and  pass condition
                          {
							   count++;
							 //  insObj_t->end_packet=0;
						  }
						  else if(flag_packet_found==0 && insObj_t->end_of_data==1)// at time false byte data
						  {
							  count=0; 
							  Llock_length=0;
							  insObj_t->end_of_data=0;
							  insObj_t->count_packet=0;
							  insObj_t->lock_length=0;
							  insObj_t->buffer_packet[0]=0x00;
							  insObj_t->buffer_packet[1]=0x00;
							  insObj_t->buffer_packet[2]=0x00;
							  insObj_t->buffer_packet[3]=0x00;
							  insObj_t->count_fail++;
							// printf("Missing packet %llu\n\r",task[5]++);
							  
						  }
                         
                         
                         
                        insObj_t->lock_length = Llock_length;
                        insObj_t->count_packet = count;
		      
		    
	                	  
	             }
	             
	             
            	
	               
	       
		     return 0;
		     

}



char MyDriverDCServo::SearchHeader(unsigned char ch, struct _STR_INSTRUCTION_ *insObj_t)
{
	

		  
                  
		if(ch==HEADER_1 && insObj_t->flag_header==0)
		{
                     
           
		insObj_t->flag_header =1;
				    // insObj_t->buffer_packet[0]= ch; //save data to buffer
				    
		 }

		 else if(ch==HEADER_2 && insObj_t->flag_header==1)
		  {
			    
		  insObj_t->flag_header =2;
			  	      
                   //  insObj_t->buffer_packet[1]= ch; //save data to buffer
                   
					
			}
            else if( ch==HEADER_3 && insObj_t->flag_header==2)
               {
                      insObj_t->flag_header =3;
                    // insObj_t->buffer_packet[2]= ch; //save data to buffer
					
                    
       
             }
            else if(ch == RESERVED && insObj_t->flag_header==3)
               {
                       insObj_t->flag_header =0; //Reset index Search header auto check
                    
                     return 0;
                    	 
             }
                
		   else
		  {
				
                  
			insObj_t->flag_header=0;
				 
	         }
				   	
		return -1;// False
		 	
}





//                                                  16   12   5
// this is the CCITT CRC 16 polynomial X  + X  + X  + 1.

unsigned short MyDriverDCServo::crc16(unsigned char *data_p, unsigned short length)
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





unsigned char MyDriverDCServo::PingPacket(unsigned char id,struct _STR_PACKET_ *packetHandle)
 {
	unsigned char low_byte=0,high_byte=0;
	unsigned short crc;


   // packetHandle->ins_packet = (unsigned char*)malloc(10);
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

unsigned char MyDriverDCServo:: ReadPacket(unsigned char id, unsigned short address,unsigned short data_length, 
                                           unsigned char command,unsigned char mode_filed,struct _STR_PACKET_ *packetHandle)
{

    unsigned char low_byte=0,high_byte=0;
	unsigned short crc;


  //  packetHandle->ins_packet = (unsigned char*)malloc(14);
	packetHandle->length =14;
	packetHandle->ins_packet[0]=HEADER_1;
	packetHandle->ins_packet[1]=HEADER_2;
	packetHandle->ins_packet[2]=HEADER_3;
	packetHandle->ins_packet[3]= RESERVED;
	packetHandle->ins_packet[4]= id;
	packetHandle->ins_packet[5]=0x07;
	packetHandle->ins_packet[6]=0x00;
	packetHandle->ins_packet[7]=Read;
	packetHandle->start_address=address;
	low_byte = address &0x00FF;
	high_byte = address >>8;
	packetHandle->ins_packet[8]=low_byte;
	packetHandle->ins_packet[9]=high_byte;
	packetHandle->data_length=data_length;
	low_byte= data_length &0x00FF;
	high_byte = data_length>>8;
	packetHandle->ins_packet[10]=low_byte;
	packetHandle->ins_packet[11]=high_byte;
	crc = crc16(packetHandle->ins_packet,12);
	low_byte = crc & 0x00FF;
	high_byte = (crc & 0xFF00)>>8;
	packetHandle->ins_packet[12]=low_byte;
	packetHandle->ins_packet[13]=high_byte;
	packetHandle->user_command=command;//ReadCommand;
	packetHandle->enable_send=enable_tx_packet;
	packetHandle->mode_filed = mode_filed;
	
	return 0;


}

unsigned char MyDriverDCServo::WritePacket(unsigned char id,unsigned short address, unsigned short num_byte, unsigned char *data,struct _STR_PACKET_ *packetHandle)
{

   unsigned char i,low_byte=0,high_byte=0;
   unsigned short crc;

   if(num_byte==1||num_byte==2||num_byte==4){
  // packetHandle->ins_packet = (unsigned char*)malloc(12+num_byte);
   packetHandle->length =12+num_byte;
   packetHandle->ins_packet[0]=HEADER_1;
   packetHandle->ins_packet[1]=HEADER_2;
   packetHandle->ins_packet[2]=HEADER_3;
   packetHandle->ins_packet[3]= RESERVED;
   packetHandle->ins_packet[4]= id;
   packetHandle->ins_packet[5]=5+num_byte;
   packetHandle->ins_packet[6]=0x00;
   packetHandle->ins_packet[7]=Write;
   low_byte = address &0x00FF;
   high_byte = address >>8;
   packetHandle->ins_packet[8]=low_byte;
   packetHandle->ins_packet[9]=high_byte;

   for(i=0;i<num_byte;i++)
   	packetHandle->ins_packet[10+i]= data[i];


   crc = crc16(packetHandle->ins_packet,10+num_byte);
   low_byte = crc & 0x00FF;
   high_byte = (crc & 0xFF00)>>8;
   packetHandle->ins_packet[10+num_byte]=low_byte;
   packetHandle->ins_packet[10+num_byte+1]=high_byte;
   packetHandle->user_command=WriteCommand;
   packetHandle->enable_send=enable_tx_packet;
   return 0;
   }
   else{
       return 1;
   }
   
}


unsigned char MyDriverDCServo::WriteBlockPacket(unsigned char id,unsigned short address, unsigned short num_byte, unsigned char *data,struct _STR_PACKET_ *packetHandle)
{

   unsigned char i,low_byte=0,high_byte=0;
   unsigned short crc;


  // packetHandle->ins_packet = (unsigned char*)malloc(12+num_byte);
   packetHandle->length =12+num_byte;
   packetHandle->ins_packet[0]=HEADER_1;
   packetHandle->ins_packet[1]=HEADER_2;
   packetHandle->ins_packet[2]=HEADER_3;
   packetHandle->ins_packet[3]= RESERVED;
   packetHandle->ins_packet[4]= id;
   packetHandle->ins_packet[5]=5+num_byte;
   packetHandle->ins_packet[6]=0x00;
   packetHandle->ins_packet[7]=WriteBlock;
   low_byte = address &0x00FF;
   high_byte = address >>8;
   packetHandle->ins_packet[8]=low_byte;
   packetHandle->ins_packet[9]=high_byte;

   for(i=0;i<num_byte;i++)
    packetHandle->ins_packet[10+i]= data[i];


   crc = crc16(packetHandle->ins_packet,10+num_byte);
   low_byte = crc & 0x00FF;
   high_byte = (crc & 0xFF00)>>8;
   packetHandle->ins_packet[10+num_byte]=low_byte;
   packetHandle->ins_packet[10+num_byte+1]=high_byte;
   //packetHandle->user_command=WriteBlockCommand;
   packetHandle->enable_send=enable_tx_packet;

   return 0;



}

unsigned char MyDriverDCServo::ReadContPacket(unsigned char id, unsigned char mode_filed,  unsigned char group1, unsigned char group2, struct _STR_PACKET_ *packetHandle)
{

   unsigned char low_byte=0,high_byte=0;
   unsigned short crc;


   //packetHandle->ins_packet = (unsigned char*)malloc(12);
   packetHandle->length =12;
   packetHandle->ins_packet[0]=HEADER_1;
   packetHandle->ins_packet[1]=HEADER_2;
   packetHandle->ins_packet[2]=HEADER_3;
   packetHandle->ins_packet[3]= RESERVED;
   packetHandle->ins_packet[4]= id;
   packetHandle->ins_packet[5]= 0x05;
   packetHandle->ins_packet[6]=0x00;
   packetHandle->ins_packet[7]=ReadContinuous;
   packetHandle->ins_packet[8]=group1;//Group 1
   packetHandle->ins_packet[9]=group2;//Group 2

   crc = crc16(packetHandle->ins_packet,10);
   low_byte = crc & 0x00FF;
   high_byte = (crc & 0xFF00)>>8;
   packetHandle->ins_packet[10]=low_byte;
   packetHandle->ins_packet[11]=high_byte;
   packetHandle->mode_filed =mode_filed;//0=Setup,1 Run
   packetHandle->user_command=ReadContinuousCommand;
   packetHandle->continous_flag =ReadContinuousCommand;
   packetHandle->enable_send=enable_tx_packet;

   return 0;



}



unsigned char MyDriverDCServo::ResetFactoryPacket(unsigned char id, unsigned char index_factory,struct _STR_PACKET_ *packetHandle)
{
    unsigned char low_byte=0,high_byte=0;
	unsigned short crc;


  //  packetHandle->ins_packet = (unsigned char*)malloc(11);
	packetHandle->length =11;
	packetHandle->ins_packet[0]=HEADER_1;
	packetHandle->ins_packet[1]=HEADER_2;
	packetHandle->ins_packet[2]=HEADER_3;
	packetHandle->ins_packet[3]= RESERVED;
	packetHandle->ins_packet[4]= id;
	packetHandle->ins_packet[5]=0x04;
	packetHandle->ins_packet[6]=0x00;
	packetHandle->ins_packet[7]=FactoryValue;
	packetHandle->ins_packet[8]=index_factory;
	crc = crc16(packetHandle->ins_packet,9);
	low_byte = crc & 0x00FF;
	high_byte = (crc & 0xFF00)>>8;
	packetHandle->ins_packet[9]=low_byte;
	packetHandle->ins_packet[10]=high_byte;
	packetHandle->user_command=ResetFactoryCommand;
	packetHandle->enable_send=enable_tx_packet;
	
	
	return 0;
}


unsigned char MyDriverDCServo::RestartPacket(unsigned char id,struct _STR_PACKET_ *packetHandle)
{
    unsigned char low_byte=0,high_byte=0;
	unsigned short crc;


  // packetHandle->ins_packet = (unsigned char*)malloc(10);
	packetHandle->length =10;
	packetHandle->ins_packet[0]=HEADER_1;
	packetHandle->ins_packet[1]=HEADER_2;
	packetHandle->ins_packet[2]=HEADER_3;
	packetHandle->ins_packet[3]= RESERVED;
	packetHandle->ins_packet[4]= id;
	packetHandle->ins_packet[5]=0x03;
	packetHandle->ins_packet[6]=0x00;
	packetHandle->ins_packet[7]=Restart;
	crc = crc16(packetHandle->ins_packet,8);
	low_byte = crc & 0x00FF;
	high_byte = (crc & 0xFF00)>>8;
	packetHandle->ins_packet[8]=low_byte;
	packetHandle->ins_packet[9]=high_byte;
	packetHandle->user_command=RestartCommand;
	packetHandle->enable_send=enable_tx_packet;
	

  return 0;
}

unsigned char  MyDriverDCServo:: WriteRegisterPacket(unsigned char id, unsigned char index_reg,struct _STR_PACKET_ *packetHandle)
{
    unsigned char low_byte=0,high_byte=0;
	unsigned short crc;


   // packetHandle->ins_packet = (unsigned char*)malloc(11);
	packetHandle->length =11;
	packetHandle->ins_packet[0]=HEADER_1;
	packetHandle->ins_packet[1]=HEADER_2;
	packetHandle->ins_packet[2]=HEADER_3;
	packetHandle->ins_packet[3]= RESERVED;
	packetHandle->ins_packet[4]= id;
	packetHandle->ins_packet[5]=0x04;
	packetHandle->ins_packet[6]=0x00;
	packetHandle->ins_packet[7]=WriteSpecialReg;
	packetHandle->ins_packet[8]=index_reg;
	crc = crc16(packetHandle->ins_packet,9);
	low_byte = crc & 0x00FF;
	high_byte = (crc & 0xFF00)>>8;
	packetHandle->ins_packet[9]=low_byte;
	packetHandle->ins_packet[10]=high_byte;
	packetHandle->user_command=WriteCommand;
	packetHandle->enable_send=enable_tx_packet;
	
	
	return 0;
}


unsigned char MyDriverDCServo::UpdatePacketJoint(struct _STR_INSTRUCTION_ *insObj_t, struct _STR_PACKET_ *packetHandle )
{


  unsigned short crc=0;
  unsigned char crc_low=0;
  unsigned char  length=0;
  unsigned short crc_cal=0;
  unsigned char ID=0;
  unsigned char INS=0;
  unsigned char ERR=0;
  unsigned short i;
  unsigned char *ptr_var;
  unsigned short length_data=0;
   
    ID = insObj_t->buffer_packet[4]; 

   //  if(insObj_t->end_of_data==1)
     //	{

                   if(ID == servoHandle_servo.id) //1-250 number of rang
             		{
                            
   	 					 crc_low = insObj_t->buffer_packet[(insObj_t->length+7)-2]; //LOW SIDE OF CRC
     					 crc     =  insObj_t->buffer_packet[((insObj_t->length+7)-2)+1]; //HIGH SIDE OF CRC
     					 crc     <<=8;
     					 crc     |=crc_low;
      					 length  = ((insObj_t->length+7)-2);
      					 length_data =insObj_t->length - 3;
        			     crc_cal = crc16(insObj_t->buffer_packet, length);
	   					 INS = insObj_t->buffer_packet[7];//instruction code
	   					 ERR = insObj_t->buffer_packet[8];//instruction error code
      
     					      if(crc_cal == crc)
       						  {
                              
                                 if(ERR == Pass)
                                 	{
                                         insObj_t->count_pass++;	  
                                        //  printf("Make  it   %d !!\n\r",packetHandle->user_command); 
                                     if(INS==Status)
                                     	{ //Read INS
												  
                                           if(packetHandle->user_command==ReadCommand||packetHandle->user_command==PingCommand||packetHandle->user_command==ReadBlockCommand)
                                           	{
												
                                              if(packetHandle->user_command==ReadCommand)
                                              {
                                                   ;

                                              }
                                              else if(packetHandle->user_command==PingCommand)
                                              {
                                                 //---------ping command
                                                  servoHandle_servo.model_number=make2uint(insObj_t->buffer_packet[9],insObj_t->buffer_packet[10]);
                                                  servoHandle_servo.version_firmware=insObj_t->buffer_packet[11];
                                                  
                                              }
                                              else if(packetHandle->user_command==ReadBlockCommand)
                                              {
												 //--------Read Block Command
												  if(packetHandle->mode_filed==0) {//setup All variable
										          servoHandle_servo.moving = insObj_t->buffer_packet[9];
										          servoHandle_servo.process_variable_position = make2float(insObj_t->buffer_packet[11],insObj_t->buffer_packet[12],
										                                                           insObj_t->buffer_packet[13],insObj_t->buffer_packet[14]); 
										          servoHandle_servo.process_variable_velocity = make2float(insObj_t->buffer_packet[15],insObj_t->buffer_packet[16],
										                                                           insObj_t->buffer_packet[17],insObj_t->buffer_packet[18]);
										          servoHandle_servo.process_variable_current  = make2float(insObj_t->buffer_packet[19],insObj_t->buffer_packet[20],
										                                                           insObj_t->buffer_packet[21],insObj_t->buffer_packet[22]) ;
												  servoHandle_servo.OutputControl = make2float(insObj_t->buffer_packet[23],insObj_t->buffer_packet[24],
												                                               insObj_t->buffer_packet[25],insObj_t->buffer_packet[26]) ;
												  servoHandle_servo.process_input_voltage = make2float(insObj_t->buffer_packet[27],insObj_t->buffer_packet[28],
												                                            insObj_t->buffer_packet[29],insObj_t->buffer_packet[30]);
												 
												
												   servoHandle_servo.process_temperature =insObj_t->buffer_packet[31];
												   if( servoHandle_servo.process_temperature<=100)
												   insObj_t->temperature_k1=servoHandle_servo.process_temperature;
												   else
												   servoHandle_servo.process_temperature = insObj_t->temperature_k1;
														
													
												 
												   
												  servoHandle_servo.alarm_drive = insObj_t->buffer_packet[32];
												  insObj_t->state_update_packet=1;
												  float  cal_error =((float)insObj_t->count_pass/((float)insObj_t->count_pass + (float)insObj_t->count_fail))*100.0; 
												 /*printf("ID: %d  [%lu]  POS: %.3f     Speed: %.3f     Current:  %.3f    Out:   %.3f   Volt: %.3f  Err: %d    Move: %d    TEMP: %d     OverLoad:  %d    %%ERR:  %.2f \n\r",
												                                                                              insObj_t->id_servo,
                                                                                                                              insObj_t->process_index++,
                                                                                                                              servoHandle_servo.process_variable_position ,
																															  servoHandle_servo.process_variable_velocity,
																															  servoHandle_servo.process_variable_current,
																															  servoHandle_servo.OutputControl,
                                                                                                                              servoHandle_servo.process_input_voltage,
                                                                                                                              servoHandle_servo.hardware_error,
                                                                                                                              servoHandle_servo.moving,
                                                                                                                              servoHandle_servo.process_temperature,
                                                                                                                              servoHandle_servo.alarm_drive,
                                                                                                                             cal_error);*/
                                                                                                                              
                                                  }
                                                  else if(packetHandle->mode_filed==1)//param control
                                                  {
													  
												  servoHandle_servo.moving = insObj_t->buffer_packet[9];
										          servoHandle_servo.process_variable_position = make2float(insObj_t->buffer_packet[11],insObj_t->buffer_packet[12],
										                                                           insObj_t->buffer_packet[13],insObj_t->buffer_packet[14]); 
										          servoHandle_servo.process_variable_velocity = make2float(insObj_t->buffer_packet[15],insObj_t->buffer_packet[16],
										                                                           insObj_t->buffer_packet[17],insObj_t->buffer_packet[18]);
										          servoHandle_servo.process_variable_current  = make2float(insObj_t->buffer_packet[19],insObj_t->buffer_packet[20],
										                                                           insObj_t->buffer_packet[21],insObj_t->buffer_packet[22]) ;
										           insObj_t->state_update_packet=1;     
										             float  cal_error =((float)insObj_t->count_pass/((float)insObj_t->count_pass + (float)insObj_t->count_fail))*100.0;                                         
										           printf("ID: %d  [%lu]  POS: %.3f     Speed: %.3f     Current:  %.3f   Move: %d    %%ERR %.2f \n\r",
												                                                                              insObj_t->id_servo,
                                                                                                                              insObj_t->process_index++,
                                                                                                                              servoHandle_servo.process_variable_position ,
																															  servoHandle_servo.process_variable_velocity,
																															  servoHandle_servo.process_variable_current,
                                                                                                                              servoHandle_servo.moving,
                                                                                                                              cal_error);
                                                                                                                              
													  
												  }
                                                                                                                              
											  }


											   //  packetHandle->user_command=DisableCommand;
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



unsigned short MyDriverDCServo::make2uint(unsigned char first_byte, unsigned char second_byte)
{
  unsigned short data;

    data = second_byte;
    data <<=8;
    data |=first_byte;

    return data;
}

unsigned char MyDriverDCServo::uint2byte(unsigned short number, unsigned char *data_byte)
{
   *(data_byte) = number &0x00FF;//low_byte
   *(data_byte+1)= (number &0xFF00)>>8;

   return 0;

}

float MyDriverDCServo::make2float(unsigned char first_byte, unsigned char second_byte, unsigned char third_byte, unsigned char fourth_byte)
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


unsigned char MyDriverDCServo::float2byte(float number, unsigned char *data_byte)
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

unsigned char MyDriverDCServo::make2pvta(float position, float speed, float torque, float accleation, unsigned char *data)
{
	unsigned char data_pos[4];
	unsigned char data_speed[4];
	unsigned char data_torque[4];
	unsigned char data_acc[4];
	
	float2byte(position, data_pos);
	float2byte(speed, data_speed);
	float2byte(torque, data_torque);
	float2byte(accleation, data_acc);
	
	*data = data_pos[0];
	*(data+1) = data_pos[1];
	*(data+2) = data_pos[2];
	*(data+3) = data_pos[3];
	*(data+4) = data_speed[0];
	*(data+5) = data_speed[1];
	*(data+6) = data_speed[2];
	*(data+7) = data_speed[3];
	*(data+8) = data_torque[0];
	*(data+9) = data_torque[1];
	*(data+10) = data_torque[2];
	*(data+11) = data_torque[3];
	*(data+12) = data_acc[0];
	*(data+13) = data_acc[1];
	*(data+14) = data_acc[2];
	*(data+15) = data_acc[3];
	
	return 0;
}


