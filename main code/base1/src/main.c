/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include <stdlib.h>
#define F_CPU 32000000UL
#include <util/delay.h>

#include "lcd.h"
#include "initialize.h"
#include "nrf24l01_L.h"
#include "transmitter.h"
#include "Menu.h"


void NRF_init (void) ;
void data_transmission (void);
void driver_packet (void);
void disp_ans(void);

/*! Defining an example slave address. */
#define SLAVE1_ADDRESS    0
#define SLAVE2_ADDRESS    1
#define SLAVE3_ADDRESS    2
#define SLAVE4_ADDRESS    3

/* Global variables */
unsigned char current;
unsigned char current_ov;
int curr_alarm=0,curr_alarm0,curr_alarm1,curr_alarm2,curr_alarm3;
int flg_alarm=0;

int flg=0;
int flg1=0;
char full_charge=0;

int adc =0;
int driverTGL;
int free_wheel=0;
int time_memory = 0 ;
int time_diff = 0; 
int last_check = 0;
int wireless_reset=0;
int Test_Data[8];
uint16_t TX_Time = 0;
uint32_t time_ms=0,kck_time,Buzzer_Time=1,Last_TX_time;
uint16_t Buzzer_Speed;
int8_t m_reset_counter = 0;
int Seg[18] = {Segment_0,Segment_1,Segment_2,Segment_3,Segment_4,Segment_5,Segment_6,Segment_7,Segment_8,Segment_9,
               Segment_10,Segment_11,Segment_12,Segment_13,Segment_14,Segment_15,Segment_Dash};
unsigned char Buf_Rx_L[_Buffer_Size] ;
char Buf_Tx_L[_Buffer_Size] ;
char Address[_Address_Width] = { 0x11, 0x22, 0x33, 0x44, 0x55};//pipe0 {0xE7,0xE7,0xE7,0xE7,0xE7};////
struct _Motor_Param
{
	int8_t restart_times;
};
typedef	struct _Motor_Param Motor_Param;
Motor_Param M[4];

int main (void)
{
    En_RC32M();

    //Enable LowLevel & HighLevel Interrupts
    PMIC_CTRL |= PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm |PMIC_MEDLVLEN_bm;

    PORT_init();
    TimerD0_init();
    TimerC0_init();
    USARTF0_init();
    USARTF1_init();
	USARTE0_init();
    ADCA_init();
	wdt_enable();
	wdt_set_timeout_period(WDT_TIMEOUT_PERIOD_16CLK);
    //LCDInit();

    // Globally enable interrupts
    sei();

    Buzzer_Time		= 2000;	
	Buzzer_Speed	= 150;

//    Address[0]=Address[0] + RobotID ;

	NRF_init () ;

    while(1)
    {
        asm("wdr");
		//////////////////////SHOOT//////////////////////////////////
		PORTC_OUTCLR=KCK_SH_PIN_bm;
		if((KCK_Ch_Limit_PORT.IN & KCK_Ch_Limit_PIN_bm)>>KCK_Ch_Limit_PIN_bp)
		{
			full_charge=1;
			tc_disable_cc_channels(&TCC0,TC_CCAEN);
		}
		else
		{
			if(flg==0)
			{
				tc_enable_cc_channels(&TCC0,TC_CCAEN);
			}
		}
		
		if (full_charge)
		{
			if (Robot_D[RobotID].KCK )
			{
				flg = 1;
			}
		}
		if (KCK_DSH_SW)
		{
			flg = 1;
		}
		//////////////////////////////SHOOT END ///////////////////////
		
		curr_alarm0=((PORTH_IN&PIN4_bm)>>4);
		curr_alarm1=((PORTQ_IN&PIN1_bm)>>1);
		curr_alarm2=((PORTQ_IN&PIN2_bm)>>2);
		curr_alarm3=((PORTC_IN&PIN4_bm)>>4);
		curr_alarm= 0*curr_alarm0 + 1*curr_alarm1 + 2*curr_alarm2 + 3*curr_alarm3 ;
		
		current_ov=curr_alarm0 || curr_alarm1 || curr_alarm2 || curr_alarm3;
		if (curr_alarm0 || curr_alarm1 || curr_alarm2 || curr_alarm3)   /////////  alarm of cuurent ov
		{
			Buzzer_PORT.OUTSET = (flg_alarm>>Buzzer_PIN_bp);
			driverTGL=1;
		}
		else
		driverTGL=0;
		//sending driver packet///////////////////////////////////////////////////////////////// 
		//duration for sending all of the packet : 13 ms 
		//sending every character last about 1 ms
        usart_putchar(&USARTF0,'*');
        usart_putchar(&USARTF0,'~');
		usart_putchar(&USARTF0,Robot_D[RobotID].M0a);//M3.PWM);
		usart_putchar(&USARTF0,Robot_D[RobotID].M0b);//M3.PWM);
		usart_putchar(&USARTF0,Robot_D[RobotID].M1a);//M3.PWM);
		usart_putchar(&USARTF0,Robot_D[RobotID].M1b);//M3.PWM);
		usart_putchar(&USARTF0,Robot_D[RobotID].M2a);//M3.PWM);
		usart_putchar(&USARTF0,Robot_D[RobotID].M2b);//M3.PWM);
		usart_putchar(&USARTF0,Robot_D[RobotID].M3a);//M3.PWM);
		usart_putchar(&USARTF0,Robot_D[RobotID].M3b);//M3.PWM);
		usart_putchar(&USARTF0,Robot_D[RobotID].ASK);
		
		if ((Robot_D[RobotID].M0a == 1) 
		&& (Robot_D[RobotID].M0b == 2) 
		&& (Robot_D[RobotID].M1a==3) 
		&& (Robot_D[RobotID].M1b == 4) || free_wheel>100 || current_ov) 
		{
				driverTGL=1;
		}
		else
		{
				driverTGL=0;
		}
		
		switch (driverTGL)
		{
			case 0:
			usart_putchar(&USARTF0,'^');//end of packet
			break;
			
			case 1:
			usart_putchar(&USARTF0,'%');//free wheel order end packet
			break;
		}
		
		//data_transmission () ;
		
		if (free_wheel > 2000)//2000ms=2s reseting nrf
		{
			//NRF_init();
			
			NRF24L01_L_Flush_TX();
			NRF24L01_L_Flush_RX();
		}
			
		//checking battery voltage////////////////////////////////////////////////////////////
        adc = adc_get_unsigned_result(&ADCA,ADC_CH0);

        if (adc<=1240)
        {
            Buzzer_PORT.OUTSET = Buzzer_PIN_bm;//10.3 volt
			PORTC.OUTSET=PIN2_bm;
        }
		else
		{
			Buzzer_PORT.OUTCLR = Buzzer_PIN_bm;//10.3 volt
			PORTC.OUTCLR=PIN2_bm;
		}
		//////////////////////////////////////////////////////////////////////////////////////
	
		//for showing test data through LCD & FT232
		// this function take time (about 16 ms)
		//so do not use it unless in test cases
		//disp_ans();	

    }
}




ISR(PORTE_INT0_vect)////////////////////////////////////////PTX   IRQ Interrupt Pin
{   
    uint8_t status_L = NRF24L01_L_WriteReg(W_REGISTER | STATUSe, _TX_DS|_MAX_RT|_RX_DR);
    if((status_L & _RX_DR) == _RX_DR)
    {
        LED_White_PORT.OUTTGL = LED_White_PIN_bm;
		wireless_reset=0;
        //1) read payload through SPI,
        NRF24L01_L_Read_RX_Buf(Buf_Rx_L, _Buffer_Size);
		free_wheel=0 ;
        if(Buf_Rx_L[0] == 'L')//RobotID)
        {   
			LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
            Robot_D[RobotID].RID  = Buf_Rx_L[0];
            Robot_D[RobotID].M0a  = Buf_Rx_L[1+ RobotID * 10];
            Robot_D[RobotID].M0b  = Buf_Rx_L[2+ RobotID * 10];
            Robot_D[RobotID].M1a  = Buf_Rx_L[3+ RobotID * 10];
            Robot_D[RobotID].M1b  = Buf_Rx_L[4+ RobotID * 10];
            Robot_D[RobotID].M2a  = Buf_Rx_L[5+ RobotID * 10];
            Robot_D[RobotID].M2b  = Buf_Rx_L[6+ RobotID * 10];
            Robot_D[RobotID].M3a  = Buf_Rx_L[7+ RobotID * 10];
            Robot_D[RobotID].M3b  = Buf_Rx_L[8+ RobotID * 10];
            Robot_D[RobotID].KCK  = Buf_Rx_L[9+ RobotID * 10];
            Robot_D[RobotID].CHP  = Buf_Rx_L[10+ RobotID * 10];
 //           Robot_D[RobotID].ASK = Buf_Rx_L[11];

        }
		
				//calculation of main loop duration///////////////////////////////////////////////////
				if (Buf_Rx_L[6] != last_check)
				{
					last_check = Buf_Rx_L[6];
					time_diff = time_ms - time_memory;
					time_memory = time_ms;
				}
				//////////////////////////////////////////////////////////////////////////////////////


        //2) clear RX_DR IRQ,
        //NRF24L01_R_WriteReg(W_REGISTER | STATUSe, _RX_DR );
        //3) read FIFO_STATUS to check if there are more payloads available in RX FIFO,
        //4) if there are more data in RX FIFO, repeat from step 1).
    }
    if((status_L&_TX_DS) == _TX_DS)
    {   LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
		wireless_reset=0;
        //NRF24L01_R_WriteReg(W_REGISTER | STATUSe, _TX_DS);
    }
    if ((status_L&_MAX_RT) == _MAX_RT)
    {
        LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
        NRF24L01_L_Flush_TX();
        //NRF24L01_R_WriteReg(W_REGISTER | STATUSe, _MAX_RT);
    }
}

char timectrl;
long int t_alarm;

ISR(TCD0_OVF_vect)
{
	TX_Time ++;
	if (TX_Time == 2)
	{
		TX_Time = 0 ;
		if (Last_TX_time + 4 < time_ms)
		{
			//data_transmission () ;
			Last_TX_time = time_ms;
		}
		
	}

    wdt_reset();
	t_alarm++;
	wireless_reset++;
	free_wheel++;// for making wheels free when there is no wireless data
    //timer for 1msTest_RPM
    time_ms++;
	if (t_alarm>=500)
	{
		flg_alarm = ~(flg_alarm);
		t_alarm=0;
		
	}
   
if(flg)
{
	if(kck_time<100)
	{
		kck_time++;
		tc_disable_cc_channels(&TCC0,TC_CCAEN);
		if(((PORTH.IN & PIN6_bm)>>PIN6_bp))
		tc_disable_cc_channels(&TCC0,TC_CCBEN);
		else
		{
			if(KCK_DSH_SW)
			{
				tc_enable_cc_channels(&TCC0,TC_CCBEN);
				KCK_Speed_DIR(KCK_SPEED_HI);
				full_charge=0;
			}
			else if(full_charge==1)
			{
				tc_enable_cc_channels(&TCC0,TC_CCBEN);
				KCK_Speed_DIR(Robot_D[RobotID].KCK);
				full_charge=0;
				// LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
			}
		}
	}
	
	else {
		KCK_Speed_DIR(KCK_SPEED_OFF);//KCK_Charge(KCK_CHARGE_ON);
		tc_enable_cc_channels(&TCC0,TC_CCAEN);
		//tc_disable_cc_channels(&TCC0,TC_CCBEN);
	kck_time=0; flg=0;}
}
    //if(flg1)
    //{
        //if(kck_time<100){kck_time++; KCK_Speed_CHIP(KCK_SPEED_HI); KCK_Charge(KCK_CHARGE_OFF);}
        //else {KCK_Speed_CHIP(KCK_SPEED_OFF);KCK_Charge(KCK_CHARGE_ON); kck_time=0; flg1=0;}
    //}


    if(menu_time == 1)
    {
        Menu_Disp(Menu_Disp_OFF);
        Menu_Display();
        Menu_Reset();
        menu_time--;
    }
    else if (menu_time>1)
    {
        menu_time--;
        menu_check_status();

        if(menu_time<3000)
        {
            Buzzer_Time=menu_time;
            Buzzer_Speed=200;
        }
    }
    else
    {
        Disp_R_PORT.OUT = Seg[RobotID];
        Disp_L_PORT.OUT = Seg[RobotID];
        //PORTJ_OUTSET=0xFF;
        //PORTH_OUTSET=0xFF;
    }
}

ISR(PORTF_INT0_vect)
{
}

ISR(PORTQ_INT0_vect)
{
}

ISR(PORTH_INT0_vect)
{
}

ISR(PORTC_INT0_vect)
{
}

ISR(PORTQ_INT1_vect)
{
}

ISR(PORTH_INT1_vect)
{
	if(menu_time ==0 )
	{
		menu_check_sw((Menu_Set),&Menu_Set_flg);
		menu_check_sw((Menu_Cancel),&Menu_Cancel_flg);
	}
	menu_time = 30000;

	Menu_Disp(Menu_Disp_ON);
	Menu_Display();
}

ISR(PORTK_INT0_vect)
{
	
}

void disp_ans(void)
{
	char str[40];
	LCDGotoXY(0,0);
	//LCDStringRam("salam");
	sprintf(str,"H: %1d",adc);
	LCDStringRam(str);
	

	uint8_t count1;
	char str1[200];
	count1 = sprintf(str1,"%d,%d,%d,%d\r",Test_Data[0],Test_Data[1],Test_Data[2],Test_Data[3]);
																			  
	for (uint8_t i=0;i<count1;i++)
	{
		usart_putchar(&USARTE0,str1[i]);	
	}
	
}


int ask_cnt0=0;
int ask_cnt1=0;

int F0_buff_tmp0;
int F0_buff_tmp1;
int F0_buff_tmp2;
int F0_buff_tmp3;

int F1_buff_tmp0;
int F1_buff_tmp1;
int F1_buff_tmp2;
int F1_buff_tmp3;

int buff_reply_tmp1;
int buff_p_temp;
int buff_i_temp;
int buff_d_temp;
int buff_u_temp;
unsigned char reply2_tmp;


ISR(USARTF0_RXC_vect)   ///////////Driver  M.2  &  M.3
{
	
	//char buff_reply [16];
	unsigned char data;
	data=USARTF0_DATA;

	switch(ask_cnt0)
	{
		case 0:
		if (data== '*')
		{
			ask_cnt0++;
		}
		break;

		case 1:
		F0_buff_tmp0=(data<<8)&0x0ff00;
		ask_cnt0++;
		break;

		case 2:
		F0_buff_tmp0|=data&0x00ff;
		ask_cnt0++;
		break;
		
		case 3:
		F0_buff_tmp1=(data<<8)&0x0ff00;
		ask_cnt0++;
		break;

		case 4:
		F0_buff_tmp1|=data&0x0ff;
		ask_cnt0++;
		break;
		
		case 5:
		F0_buff_tmp2=(data<<8)&0x0ff00;
		ask_cnt0++;
		break;

		case 6:
		F0_buff_tmp2|=data&0x0ff;
		ask_cnt0++;
		break;
		
		case 7:
		F0_buff_tmp3=(data<<8)&0x0ff00;
		ask_cnt0++;
		break;

		case 8:
		F0_buff_tmp3|=data&0x0ff;
		ask_cnt0++;
		break;

		case 9:
		if (data=='#')
		{
			Test_Data[0]=F0_buff_tmp0;
			Test_Data[1]=F0_buff_tmp1;
			Test_Data[2]=F0_buff_tmp2;
			Test_Data[3]=F0_buff_tmp3;
			
			if (Test_Data[0]=='1' && Test_Data[1]=='2' && Test_Data[2]=='3' && Test_Data[3]=='4')
			{
				m_reset_counter++;
				if ( m_reset_counter == 1 )
				{
					M[Robot_D[RobotID].ASK].restart_times++;
				}
					
			}
			else
			{
				m_reset_counter = 0 ;
			}

			ask_cnt0=0;
		}
		ask_cnt0=0;
		break;
	}
}

ISR(USARTF1_RXC_vect)   ////////// Driver  M.0  &  M.1
{
	unsigned char data;
	data=USARTF1_DATA;
	
	switch(ask_cnt1)
	{
		case 0:
		if (data== '*')
		{
			
			ask_cnt1++;
		}
		break;

		case 1:
		F1_buff_tmp0 =(data<<8)&0x0ff00;
		ask_cnt1++;
		break;
		
		case 2:
		F1_buff_tmp0|=data&0x00ff;
		ask_cnt1++;
		break;
		
		case 3:
		F1_buff_tmp1=(data<<8)&0x0ff00;
		ask_cnt1++;
		break;

		case 4:
		F1_buff_tmp1|=data&0x00ff;
		ask_cnt1++;
		break;
		
		case 5:
		F1_buff_tmp2=(data<<8)&0x0ff00;
		ask_cnt1++;
		break;
		
		case 6:
		F1_buff_tmp2|=data&0x00ff;
		ask_cnt1++;
		break;

		case 7:
		F1_buff_tmp3=(data<<8)&0x0ff00;
		ask_cnt1++;
		break;
		
		case 8:
		F1_buff_tmp3|=data&0x00ff;
		ask_cnt1++;
		break;

		case 9:
		if (data=='#')
		{
			Test_Data[0]=F1_buff_tmp0;
			Test_Data[1]=F1_buff_tmp1;
			Test_Data[2]=F1_buff_tmp2;
			Test_Data[3]=F1_buff_tmp3;
			
			if (Test_Data[0]=='1' && Test_Data[1]=='2' && Test_Data[2]=='3' && Test_Data[3]=='4')
			{
				m_reset_counter++;
				if ( m_reset_counter == 1 )
				{
					M[Robot_D[RobotID].ASK].restart_times++;
				}
				
			}
			else
			{
				m_reset_counter = 0 ;
			}
			
			ask_cnt1=0;
		}

		ask_cnt1=0;
		break;
	}
	
}

ISR(USARTE0_RXC_vect)
{
	LED_Green_PORT.OUTTGL = LED_Green_PIN_bm;
}

void NRF_init (void)
{
	    NRF24L01_L_CE_LOW;       //disable transceiver modes

	    SPI_Init();

	    _delay_us(10);
	    _delay_ms(11);      //power on reset delay needs 10.3ms//amin changed 100ms to 11ms
	    NRF24L01_L_Clear_Interrupts();
	    NRF24L01_L_Flush_TX();
	    NRF24L01_L_Flush_RX();
	    NRF24L01_L_CE_LOW;
// 	    if (RobotID < 3)
// 	    NRF24L01_L_Init_milad(_TX_MODE, _CH_0, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
// 	    else if(RobotID > 2 && RobotID < 6)
// 	    NRF24L01_L_Init_milad(_TX_MODE, _CH_1, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
// 	    else if (RobotID > 5 && RobotID < 9)
// 	    NRF24L01_L_Init_milad(_TX_MODE, _CH_2, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
// 	    else
// 	    NRF24L01_L_Init_milad(_TX_MODE, _CH_3, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
		NRF24L01_L_Init_milad(_RX_MODE, _CH_1, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
 	    NRF24L01_L_WriteReg(W_REGISTER | DYNPD,0x01);
	    NRF24L01_L_WriteReg(W_REGISTER | FEATURE,0x06);

	    NRF24L01_L_CE_HIGH;
	    _delay_us(130);
}

void data_transmission (void)
{
		//transmitting data to wireless board/////////////////////////////////////////////////
		Test_Data[0] = time_diff;
		
		Buf_Tx_L[0]  = (Test_Data[0]>> 8) & 0xFF;	//drive test data
		Buf_Tx_L[1]  = Test_Data[0] & 0xFF;			//drive test data
		Buf_Tx_L[2]  = (Test_Data[1]>> 8) & 0xFF;	//drive test data
		Buf_Tx_L[3]  = Test_Data[1] & 0xFF;			//drive test data
		Buf_Tx_L[4]  = (Test_Data[2]>> 8) & 0xFF;	//drive test data
		Buf_Tx_L[5]  = Test_Data[2] & 0xFF;			//drive test data
		Buf_Tx_L[6]  = (Test_Data[3]>> 8) & 0xFF;	//drive test data
		Buf_Tx_L[7]  = Test_Data[3] & 0xFF;			//drive test data
		Buf_Tx_L[8]  = (Test_Data[4]>> 8) & 0xFF;	// unused
		Buf_Tx_L[9]  = Test_Data[4] & 0xFF;			// unused
		Buf_Tx_L[10] = (Test_Data[5]>> 8) & 0xFF;	// unused
		Buf_Tx_L[11] = Test_Data[5] & 0xFF;			// unused
		Buf_Tx_L[12] = (Test_Data[6]>> 8) & 0xFF;	// unused
		Buf_Tx_L[13] = Test_Data[6] & 0xFF;			// unused
		Buf_Tx_L[14] = (Test_Data[7]>> 8) & 0xFF;	// unused
		Buf_Tx_L[15] = Test_Data[7] & 0xFF;			// unused
		Buf_Tx_L[16] = adc/12;						//battery voltage
		

		//LED_Red_PORT.OUTTGL = LED_Red_PIN_bm;
		NRF24L01_L_Write_TX_Buf(Buf_Tx_L, _Buffer_Size);
		NRF24L01_L_RF_TX();
}
void driver_packet (void);