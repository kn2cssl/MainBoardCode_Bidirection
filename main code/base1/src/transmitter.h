/*
 * transmitter.h
 *
 * Created: 02/27/2012 05:52:57 ب.ظ
 *  Author: Milad
 
 
 
 
 */ 


#ifndef TRANSMITTER_H_
#define TRANSMITTER_H_
#include <lcd.h>
#include <stdio.h>

#define Header_Lenght 4
#define Data_Lenght 7
#define Max_Robot 12
#define Max_Packet_Lenght 47 //4 + 6*7 + 1 
#define START_BYTE0 0xA5
#define START_BYTE1 0x5A
#define STOP_BYTE 0x80
#define Buffer_Size Max_Packet_Lenght
uint8_t PCK_Num=0;
uint8_t data_write_index=0,data_read_index=0,Packet_Next_Start=0;
uint16_t PCK_Speed,PCK_Reset=0;
uint8_t Circular_Buffer[Buffer_Size];
extern char Buf_Tx_R[Max_Robot][_Buffer_Size];
struct PCK_Header
{
	uint8_t NOB;  //Counter for Packet Bytes
	uint8_t SIB;
	uint8_t CHK;
};

struct Robot_Data
{
	uint8_t RID;
	signed int M0a;
	signed int M0b;
	signed int M1a;
	signed int M1b;
	signed int M2a;
	signed int M2b;
	signed int M3a;
	signed int M3b;
	uint8_t KCK;
	uint8_t CHP;
	uint8_t ASK;
	unsigned char P;
	unsigned char I;
	unsigned char D;
};

struct PCK_Send_Header
{
	uint8_t SIB;
	uint8_t CHK;
	uint8_t RID;	
};


struct Robot_Send_Data
{
	uint8_t PTP;
	uint8_t EN1;
	uint8_t EN2;
	uint8_t EN3;
	uint8_t EN4;
};

struct PCK_Header PCK_H;
struct Robot_Data Robot_D[Max_Robot], Robot_D_tmp[Max_Robot];

struct PCK_Send_Header PCK_S_H;
struct Robot_Send_Data Robot_S_D, Robot_S_D_tmp, Robot_S_D_tmp2;
uint8_t Robot_Send_PCK[11],Send_cnt=0;
//static void GetNewData(uint8_t data)
//{	
	//if (PCK_Num<Header_Lenght)
	//{
		//switch(PCK_Num)
		//{	
		//case 0:
			//if (data == START_BYTE0) 
			//{
				//PCK_Num++; 
			//}
			//break;
		//case 1:
			//if (data == START_BYTE1) 
				//PCK_Num++;
			//else
			//{
				//PCK_Num = 0;
			//}			
			//break;
		//case 2:
			//PCK_H.SIB = data;
			//PCK_Num++;
			//break;
		//case 3:
			//PCK_H.CHK = data;
			//PCK_Num++;
			//break;
		//}
	//}	
	//else
	//{
		//if (PCK_Num < PCK_H.SIB-1)
		//{
			//
			//switch((PCK_Num-Header_Lenght) % Data_Lenght) //0x07 is Size of Robot Data
			//{
			//case 0:
				//Robot_D_tmp[(PCK_Num-Header_Lenght)/Data_Lenght].RID=data;
				//PCK_H.CHK -= data;
				//break;
			//case 1:
				//Robot_D_tmp[(PCK_Num-Header_Lenght)/Data_Lenght].M1=data;
				//PCK_H.CHK -= data;
				//break;
			//case 2:
				//Robot_D_tmp[(PCK_Num-Header_Lenght)/Data_Lenght].M2=data;
				//PCK_H.CHK -= data;
				//break;
			//case 3:
				//Robot_D_tmp[(PCK_Num-Header_Lenght)/Data_Lenght].M3=data;
				//PCK_H.CHK -= data;
				//break;
			//case 4:
				//Robot_D_tmp[(PCK_Num-Header_Lenght)/Data_Lenght].M4=data;
				//PCK_H.CHK -= data;
				//break;
			//case 5:
				//Robot_D_tmp[(PCK_Num-Header_Lenght)/Data_Lenght].KCK=data;
				//PCK_H.CHK -= data;
				//break;
			//case 6:
				//Robot_D_tmp[(PCK_Num-Header_Lenght)/Data_Lenght].CHP=data;
				//PCK_H.CHK -= data;
				//break;		
			//}
			//PCK_Num++;
		//}		
		//else
		//{
			//if (PCK_H.CHK == 0 && data == 0x80)
			//{	
				//PCK_Reset = 0;
				//PCK_Speed++;
				//for (uint8_t i=0;i<Max_Robot;i++)
				//{
					//Robot_D[i] = Robot_D_tmp[i];
					//Buf_Tx_R[i][0] = Robot_D[i].RID;
					//Buf_Tx_R[i][1] = Robot_D[i].M1;
					//Buf_Tx_R[i][2] = Robot_D[i].M2;
					//Buf_Tx_R[i][3] = Robot_D[i].M3;
					//Buf_Tx_R[i][4] = Robot_D[i].M4;
					//Buf_Tx_R[i][5] = Robot_D[i].KCK;
					//Buf_Tx_R[i][6] = Robot_D[i].CHP;
					//
				//}
				//
					//
					//
				//
				//
				////LED_Green_R_PORT.OUTTGL = LED_Green_R_PIN_bm;
				//
			//}
			//PCK_Num = 0;
		//}					
	//}
//}

 void SendNewData()
{
	Send_cnt = 0;
	
	Robot_Send_PCK[0] = START_BYTE0;
	Robot_Send_PCK[1] = START_BYTE1;
	Robot_Send_PCK[2] = PCK_S_H.SIB;
	Robot_Send_PCK[3] = PCK_S_H.CHK;
	Robot_Send_PCK[4] = PCK_S_H.RID;
	Robot_Send_PCK[5] = Robot_S_D.PTP;
	Robot_Send_PCK[6] = Robot_S_D.EN1;
	Robot_Send_PCK[7] = Robot_S_D.EN2;
	Robot_Send_PCK[8] = Robot_S_D.EN3;
	Robot_Send_PCK[9] = Robot_S_D.EN4;
	Robot_Send_PCK[10] = 0x80;
	usart_set_dre_interrupt_level(&USARTE0,USART_INT_LVL_LO);
	usart_put(&USARTE0,Robot_Send_PCK[Send_cnt]);
	Send_cnt++;
	
}

#endif /* TRANSMITTER_H_ */




////////
////////SOP	// start of packet (2 bytes)
////////SIB	// size in bytes
////////CHK	// ckeck sum
////////-------------------------------------------
////////RID	// robot id (1 bytes)
////////M1	// motor 1
////////M2	// motor 2
////////M3	// motor 3
////////M4	// motor 4
////////KCK	// kick options
////////CHP	// CHIP & SPIN   #SCCCCCCC
////////-------------------------------------------
////////EOP	// end of packet
////////
////////
////////
////////SOP	// start of packet (2 bytes)
////////SIB	// size in bytes
////////CHK	// ckeck sum
////////RID	// robot id (1 byte)
////////PTP	// packet type
////////-------------------------------------------
////////EN1	Batt
////////EN2	Kick
////////EN3
////////EN4
////////-------------------------------------------
////////EOP	// end of packet



