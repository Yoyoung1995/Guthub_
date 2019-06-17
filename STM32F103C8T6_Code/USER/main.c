/***************************************************************************************
 * ����    ��W5500�Ķ˿�0�����ڿͻ���ģʽ,�����롶TCP&UDP���Թ��ߡ��ϴ����ķ��������,
 *			 ������500ms��ʱ������ʱ������˷����ַ���"\r\nWelcome To YiXinElec!\r\n",ͬʱ����
 *			 �յ�����˷��������ݻط�������ˡ�
 * ʵ��ƽ̨���û�STM32������ + YIXIN_W5500��̫��(TCP/IP)ģ��
 * Ӳ�����ӣ�  PC5 -> W5500_RST       
 *             PA4 -> W5500_SCS      
 *             PA5 -> W5500_SCK    
 *             PA6 -> W5500_MISO    
 *             PA7 -> W5500_MOSI    
 * ��汾  ��ST_v3.5

 * �Ա�    ��http://yixindianzikeji.taobao.com/
***************************************************************************************/

/*�����������*/
//���أ�192.168.1.1
//����:	255.255.255.0
//�����ַ��0C 29 AB 7C 00 01
//����IP��ַ:192.168.1.199
//�˿�0�Ķ˿ںţ�5000
//�˿�0��Ŀ��IP��ַ��192.168.1.190
//�˿�0��Ŀ�Ķ˿ںţ�6000

#include "stm32f10x.h"		
#include "W5500.h"			
#include "stmflash.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


void RCC_Configuration(void);		//����ϵͳʱ��Ϊ72MHZ(������Ը�����Ҫ��)
void NVIC_Configuration(void);		//STM32�ж�������������
void Timer2_Init_Config(void);		//Timer2��ʼ������
void System_Initialization(void);	//STM32ϵͳ��ʼ������(��ʼ��STM32ʱ�Ӽ�����)
void Delay(unsigned int d);			//��ʱ����(10ms)

extern void Yoyung_GPIO_Init(void);

unsigned int Timer2_Counter=0; //Timer2��ʱ����������(ms)
unsigned int W5500_Send_Delay_Counter=0; //W5500������ʱ��������(*10 ms)
unsigned int W5500_P_Delay_Counter=0; //W5500��������ʱ��������(*10 ms)

typedef struct xxxxbb {
	unsigned short Gateway_IP[4];			
	unsigned short Sub_Mask[4];
	unsigned short Phy_Addr[6];
	unsigned short IP_Addr[4];
	unsigned short S0_Port;
	unsigned short S0_DIP[4];
	unsigned short S0_DPort;
}SET_IP;
SET_IP SetIP_Default={{192,168,1,1},
											{255,255,255,0},
											{0x0c,0x29,0xab,0x7c,0x00,0x01},
											{192,168,1,199},
											5000,
											{192,168,1,100},
											6000
};
SET_IP SetIP_User = {0};
//flash�� 0~99���� SetIP_Default   100~199���� SetIP_User

//Ҫд�뵽STM32 FLASH���ַ�������

char Command[30] = {0};
//char Part[6][10] = {0};
//char * p = NULL;
//unsigned char j=0;
//const u8 TEXT_Buffer[]={"STM32F103 FLASH TEST"};
#define SIZE sizeof(SET_IP)		//���鳤��  * 2 < 1024
#define FLASH_SAVE_ADDR  0X0800FC00		//����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000)

/*******************************************************************************
* ������  : W5500_Initialization
* ����    : W5500��ʼ������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ��
*******************************************************************************/
void W5500_Initialization(void)
{
	W5500_Init();		//��ʼ��W5500�Ĵ�������
	Detect_Gateway();	//������ط����� 
	Socket_Init(0);		//ָ��Socket(0~7)��ʼ��,��ʼ���˿�0
}

/*******************************************************************************
* ������  : Load_Net_Parameters
* ����    : װ���������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ���ء����롢�����ַ������IP��ַ���˿ںš�Ŀ��IP��ַ��Ŀ�Ķ˿ںš��˿ڹ���ģʽ
*******************************************************************************/
void Load_Net_Parameters(void)
{
	Gateway_IP[0] = SetIP_User.Gateway_IP[0]%256;//�������ز���
	Gateway_IP[1] = SetIP_User.Gateway_IP[1]%256;
	Gateway_IP[2] = SetIP_User.Gateway_IP[2]%256;
	Gateway_IP[3] = SetIP_User.Gateway_IP[3]%256;

	Sub_Mask[0]=SetIP_User.Sub_Mask[0]%256;//������������
	Sub_Mask[1]=SetIP_User.Sub_Mask[1]%256;
	Sub_Mask[2]=SetIP_User.Sub_Mask[2]%256;
	Sub_Mask[3]=SetIP_User.Sub_Mask[3]%256;

	Phy_Addr[0]=SetIP_User.Phy_Addr[0]%256;//���������ַ
	Phy_Addr[1]=SetIP_User.Phy_Addr[1]%256;
	Phy_Addr[2]=SetIP_User.Phy_Addr[2]%256;
	Phy_Addr[3]=SetIP_User.Phy_Addr[3]%256;
	Phy_Addr[4]=SetIP_User.Phy_Addr[4]%256;
	Phy_Addr[5]=SetIP_User.Phy_Addr[5]%256;

	IP_Addr[0]=SetIP_User.IP_Addr[0]%256;//���ر���IP��ַ
	IP_Addr[1]=SetIP_User.IP_Addr[1]%256;
	IP_Addr[2]=SetIP_User.IP_Addr[2]%256;
	IP_Addr[3]=SetIP_User.IP_Addr[3]%256;

	S0_Port[0] = SetIP_User.S0_Port/256;//���ض˿�0�Ķ˿ں�5000 
	S0_Port[1] = SetIP_User.S0_Port%256;

	S0_DIP[0]=SetIP_User.S0_DIP[0]%256;//���ض˿�0��Ŀ��IP��ַ
	S0_DIP[1]=SetIP_User.S0_DIP[1]%256;
	S0_DIP[2]=SetIP_User.S0_DIP[2]%256;
	S0_DIP[3]=SetIP_User.S0_DIP[3]%256;
	
	S0_DPort[0] = SetIP_User.S0_DPort/256;//���ض˿�0��Ŀ�Ķ˿ں�6000
	S0_DPort[1] = SetIP_User.S0_DPort%256;

	S0_Mode=TCP_CLIENT;//���ض˿�0�Ĺ���ģʽ,TCP�ͻ���ģʽ
}

/*******************************************************************************
* ������  : W5500_Socket_Set
* ����    : W5500�˿ڳ�ʼ������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : �ֱ�����4���˿�,���ݶ˿ڹ���ģʽ,���˿�����TCP��������TCP�ͻ��˻�UDPģʽ.
*			�Ӷ˿�״̬�ֽ�Socket_State�����ж϶˿ڵĹ������
*******************************************************************************/
void W5500_Socket_Set(void)
{
	if(S0_State==0)//�˿�0��ʼ������
	{
		if(S0_Mode==TCP_SERVER)//TCP������ģʽ 
		{
			if(Socket_Listen(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else if(S0_Mode==TCP_CLIENT)//TCP�ͻ���ģʽ 
		{
			if(Socket_Connect(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else//UDPģʽ 
		{
			if(Socket_UDP(0)==TRUE)
				S0_State=S_INIT|S_CONN;
			else
				S0_State=0;
		}
	}
}

/*******************************************************************************
* ������  : Process_Socket_Data
* ����    : W5500���ղ����ͽ��յ�������
* ����    : s:�˿ں�
* ���    : ��
* ����ֵ  : ��
* ˵��    : �������ȵ���S_rx_process()��W5500�Ķ˿ڽ������ݻ�������ȡ����,
*			Ȼ�󽫶�ȡ�����ݴ�Rx_Buffer������Temp_Buffer���������д���
*			������ϣ������ݴ�Temp_Buffer������Tx_Buffer������������S_tx_process()
*			�������ݡ�
*******************************************************************************/
void Process_Socket_Data(SOCKET s)
{
	unsigned short size;
	size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
	memcpy(Tx_Buffer, Rx_Buffer, size);			
	Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
}

/**
 * ��ʼ�����Ź�
 * prer:???:0~7(??? 3 ???!)
 * ????=4*2^prer.??????? 256!
 * rlr:???????:? 11 ???.
 * ????(??):Tout=((4*2^prer)*rlr)/40 (ms).
 */
void IWDG_Init(u8 prer,u16 rlr)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); /* ??????IWDG_PR?IWDG_RLR????*/
    IWDG_SetPrescaler(prer);    /*??IWDG????:??IWDG????*/
    IWDG_SetReload(rlr);     /*??IWDG????*/
    IWDG_ReloadCounter();    /*??IWDG???????????IWDG???*/
    IWDG_Enable();        /*??IWDG*/
}

/**
 * ι���Ź�
 */
void IWDG_Feed(void)
{
    IWDG_ReloadCounter();    /*reload*/
}

/*******************************************************************************
* ������  : main
* ����    : ���������û������main������ʼ����
* ����    : ��
* ���    : ��
* ����ֵ  : int:����ֵΪһ��16λ������
* ˵��    : ��
*******************************************************************************/
int main(void)
{								
	System_Initialization();	//STM32ϵͳ��ʼ������(��ʼ��STM32ʱ�Ӽ�����)
	
	//��һ�����г���д��һ��flash
	STMFLASH_Read(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);
	if(SetIP_User.Gateway_IP[0]==0xffff && SetIP_User.Gateway_IP[1]==0xffff)
	{
	memcpy((u16*)&SetIP_User,(u16*)&SetIP_Default,SIZE);
	STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)&SetIP_Default,SIZE);
	STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	
	}


//memcpy((u16*)&SetIP_User,(u16*)&SetIP_Default,SIZE);	
  STMFLASH_Read(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	//flash��ȡ�������
	Load_Net_Parameters();		//װ���������	
	W5500_Hardware_Reset();		//Ӳ����λW5500
	W5500_Initialization();		//W5500��ʼ������
	Yoyung_GPIO_Init();				// GPIO/����ϵͳ ��ʼ��
	
	//IP��ַ�ָ�Ĭ��
	if ( 0 == GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8) )   
	{
		memcpy((u16*)&SetIP_User,(u16*)&SetIP_Default,SIZE);
		STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //��¼��flash
	}
	
//	IWDG_Init(4,625);//????????,????64,?????625,???????:64*625/40=1000ms=1s
	IWDG_Init(5,625);   //���ʱ��Ϊ2��
	
	while (1)
	{
		W5500_Socket_Set();//W5500�˿ڳ�ʼ������

		W5500_Interrupt_Process();//W5500�жϴ��������

		if((S0_Data & S_RECEIVE) == S_RECEIVE)//���Socket0���յ�����
		{
			S0_Data&=~S_RECEIVE;
			Process_Socket_Data(0);//W5500���ղ����ͽ��յ�������
			
			//---- Add By Yoyung ---  Ӧ��Э��
			switch(Rx_Buffer[0])
			{
				case 1: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_1);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_1);  //��
								break;
				case 2: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_2);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_2);  //��
								break;		
				case 3: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_3);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_3);  //��
								break;
				case 4: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_8);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_8);  //��
								break;		
				case 5: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_9);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_9);  //��
								break;
				case 6: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_10);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_10);  //��
								break;		
				case 7: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_11);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_11);  //��
								break;
				case 8: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_12);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_12);  //��
								break;
				case 9: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_15);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_15);  //��
								break;
				case 10: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_3);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_3);  //��
								break;							
				case 11: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_4);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_4);  //��
								break;								
				case 12: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_5);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_5);  //��
								break;							
				case 13: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_6);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_6);  //��
								break;								
				case 14: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_7);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_7);  //��
								break;					
				case 15: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_10);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_10);  //��
								break;					
				case 16: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_11);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_11);  //��
								break;					
				case 17: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_12);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_12);  //��
								break;					
				case 18: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_13);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_13);  //��
								break;					
				case 19: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_14);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_14);  //��
								break;					
				case 20: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_15);  //��
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_15);  //��
								break;	
								
				case 'S':{
									/***** ���ַ������ɼ���С�ַ��� ******/
//									char Command[20] = "Set xxxxxxx";
									char Part[6][10] = {0};
									char * p = NULL;
									unsigned char j=0;								
									
									p = strtok((char *)Rx_Buffer,":");
									memcpy(Command,p,strlen(p)+1);
									while( p != NULL ) 
									{	
										p = strtok(NULL, ".");
										memcpy(Part[j],p,strlen(p)+1);
										j++;
									}
									
									if (0 == strncmp(Command,"Set Gateway_IP",strlen(Command))  )
									{
										SetIP_User.Gateway_IP[0] = atoi(Part[0])%65536;
										SetIP_User.Gateway_IP[1] = atoi(Part[1])%65536;
										SetIP_User.Gateway_IP[2] = atoi(Part[2])%65536;
										SetIP_User.Gateway_IP[3] = atoi(Part[3])%65536;
										STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //��¼��flash
									}
									else if (0 == strncmp(Command,"Set Sub_Mask",strlen(Command))  )
									{
										SetIP_User.Sub_Mask[0] = atoi(Part[0])%65536;
										SetIP_User.Sub_Mask[1] = atoi(Part[1])%65536;
										SetIP_User.Sub_Mask[2] = atoi(Part[2])%65536;
										SetIP_User.Sub_Mask[3] = atoi(Part[3])%65536;
										STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //��¼��flash
									}
									else if (0 == strncmp(Command,"Set Phy_Addr",strlen("Set Phhy_Addr"))  )
									{
										SetIP_User.Phy_Addr[0] = atoi(Part[0])%65536;
										SetIP_User.Phy_Addr[1] = atoi(Part[1])%65536;
										SetIP_User.Phy_Addr[2] = atoi(Part[2])%65536;
										SetIP_User.Phy_Addr[3] = atoi(Part[3])%65536;
										SetIP_User.Phy_Addr[4] = atoi(Part[4])%65536;
										SetIP_User.Phy_Addr[5] = atoi(Part[5])%65536;
										STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //��¼��flash
									}
									else if (0 == strncmp(Command,"Set IP_Addr",strlen(Command))  )
									{
										SetIP_User.IP_Addr[0] = atoi(Part[0])%65536;
										SetIP_User.IP_Addr[1] = atoi(Part[1])%65536;
										SetIP_User.IP_Addr[2] = atoi(Part[2])%65536;
										SetIP_User.IP_Addr[3] = atoi(Part[3])%65536;
										STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //��¼��flash
									}
									else if (0 == strncmp(Command,"Set S0_Port",strlen(Command))  )
									{
										SetIP_User.S0_Port = atoi(Part[0])%65536;
										STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //��¼��flash
									}
									else if (0 == strncmp(Command,"Set S0_DIP",strlen(Command))  )
									{
										SetIP_User.S0_DIP[0] = atoi(Part[0])%65536;
										SetIP_User.S0_DIP[1] = atoi(Part[1])%65536;
										SetIP_User.S0_DIP[2] = atoi(Part[2])%65536;
										SetIP_User.S0_DIP[3] = atoi(Part[3])%65536;			
										STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //��¼��flash										
									}									
									else if (0 == strncmp(Command,"Set S0_DPort_",strlen(Command))  )
									{
										SetIP_User.S0_DPort = atoi(Part[0])%65536;
										STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //��¼��flash
									}
									else if (0 == strncmp(Command,"Set Watch",strlen(Command))  )
									{
										snprintf( (char *)Tx_Buffer,sizeof(Tx_Buffer),
																							"������Ϣ: \r\n����IP��ַ:%hd.%hd.%hd.%hd \r\n��������:%hd.%hd.%hd.%hd \r\n�����ַMAC:%x.%x.%x.%x.%x.%x \r\n����IP��ַ:%hd.%hd.%hd.%hd \r\n�˿�0�Ķ˿ں�:%hd \r\n�˿�0Ŀ��IP��ַ:%hd.%hd.%hd.%hd \r\n�˿�0Ŀ�Ķ˿ں�:%hd \r\n",
																							SetIP_User.Gateway_IP[0],SetIP_User.Gateway_IP[1],SetIP_User.Gateway_IP[2],SetIP_User.Gateway_IP[3],
																							SetIP_User.Sub_Mask[0],SetIP_User.Sub_Mask[1],SetIP_User.Sub_Mask[2],SetIP_User.Sub_Mask[3],
																							SetIP_User.Phy_Addr[0],SetIP_User.Phy_Addr[1],SetIP_User.Phy_Addr[2],SetIP_User.Phy_Addr[3],SetIP_User.Phy_Addr[4],SetIP_User.Phy_Addr[5],
																							SetIP_User.IP_Addr[0],SetIP_User.IP_Addr[1],SetIP_User.IP_Addr[2],SetIP_User.IP_Addr[3],
																							SetIP_User.S0_Port,
																							SetIP_User.S0_DIP[0],SetIP_User.S0_DIP[1],SetIP_User.S0_DIP[2],SetIP_User.S0_DIP[3],
																							SetIP_User.S0_DPort
																							);
										Write_SOCK_Data_Buffer(0, Tx_Buffer, strlen((char *)Tx_Buffer)+1 );
									}
									
//									STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //��¼��flash
									//�����ϵ���� �������õ�W5500ģ��
									
									}
									break;
									
				case 'P' :   //������ ,����
									Rx_Buffer[0] = 0;
									
									W5500_P_Delay_Counter = 0;
									break;
				default : break;										
			}
			
			
		}
		else if(W5500_Send_Delay_Counter >= 50)//��ʱ�����ַ���
		{
			if(S0_State == (S_INIT|S_CONN))
			{
				S0_Data&=~S_TRANSMITOK;
				memcpy(Tx_Buffer, "\r\nWelcome To YiXinElec!\r\n", 23);	
				Write_SOCK_Data_Buffer(0, Tx_Buffer, 23);//ָ��Socket(0~7)�������ݴ���,�˿�0����23�ֽ�����
			}
			W5500_Send_Delay_Counter=0;
		}
		
		/****30S�ڽ��ղ��������� �Ĵ��� *******/
		if(W5500_P_Delay_Counter>3000)
		{
			System_Initialization();
			STMFLASH_Read(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	//flash��ȡ�������
			Load_Net_Parameters();		//װ���������	
			W5500_Hardware_Reset();		//Ӳ����λW5500
			W5500_Initialization();		//W5500��ʼ������
			W5500_Socket_Set();//W5500�˿ڳ�ʼ������
			W5500_Interrupt_Process();//W5500�жϴ��������
			
			
			W5500_P_Delay_Counter = 0;
		}
		
		
		IWDG_Feed();
	}
}

/*******************************************************************************
* ������  : RCC_Configuration
* ����    : ����ϵͳʱ��Ϊ72MHZ(������Ը�����Ҫ��)
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : STM32F107x��STM32F105xϵ��MCU��STM32F103xϵ��MCUʱ������������ͬ
*******************************************************************************/
void RCC_Configuration(void)
{
  ErrorStatus HSEStartUpStatus;               //�ⲿ����ʱ��(HSE)�Ĺ���״̬����
  
  RCC_DeInit();                               //��������ʱ����صļĴ�������ΪĬ��ֵ
  RCC_HSEConfig(RCC_HSE_ON);                  //�����ⲿ����ʱ��HSE 
  HSEStartUpStatus = RCC_WaitForHSEStartUp(); //�ȴ��ⲿ����ʱ��(HSE)�ȶ�

  if(SUCCESS == HSEStartUpStatus)             //����ⲿ����ʱ���Ѿ��ȶ�
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); //Flash����
    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    
  
    RCC_HCLKConfig(RCC_SYSCLK_Div1); //����AHBʱ�ӵ���ϵͳʱ��(1��Ƶ)/72MHZ
    RCC_PCLK2Config(RCC_HCLK_Div1);  //����APB2ʱ�Ӻ�HCLKʱ�����/72MHz(���Ϊ72MHz)
    RCC_PCLK1Config(RCC_HCLK_Div2);  //����APB1ʱ����HCLKʱ�ӵ�2��Ƶ/36MHz(���Ϊ36MHz)
  
#ifndef STM32F10X_CL                 //���ʹ�õĲ���STM32F107x��STM32F105xϵ��MCU,PLL��������  
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); //PLLCLK = 8MHz * 9 = 72 MHz 
#else                                //���ʹ�õ���STM32F107x��STM32F105xϵ��MCU,PLL��������
    /***** ����PLLx *****/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    RCC_PREDIV2Config(RCC_PREDIV2_Div5);
    RCC_PLL2Config(RCC_PLL2Mul_8);

    RCC_PLL2Cmd(ENABLE); //ʹ��PLL2 
    while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET);//�ȴ�PLL2�ȶ�

    /* PLL configuration: PLLCLK = (PLL2 / 5) * 9 = 72 MHz */ 
    RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);
#endif

    RCC_PLLCmd(ENABLE); //ʹ��PLL
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); //�ȴ�PLL�ȶ�

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);          //����ϵͳʱ�ӵ�ʱ��ԴΪPLL

    while(RCC_GetSYSCLKSource() != 0x08);               //���ϵͳ��ʱ��Դ�Ƿ���PLL
    RCC_ClockSecuritySystemCmd(ENABLE);                 //ʹ��ϵͳ��ȫʱ�� 

	/* Enable peripheral clocks --------------------------------------------------*/
  	/* Enable I2C1 and I2C1 clock */
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  	/* Enable GPIOA GPIOB SPI1 and USART1 clocks */
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB
					| RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD
					| RCC_APB2Periph_AFIO, ENABLE);    
  }
}

/*******************************************************************************
* ������  : NVIC_Configuration
* ����    : STM32�ж�������������
* ����    : ��
* ���    : ��
* ����ֵ  : ��
* ˵��    : ����KEY1(PC11)���ж�������
*******************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;						//����NVIC��ʼ���ṹ��

  	/* Set the Vector Table base location at 0x08000000 */
  	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);				//�����ж����ȼ���Ϊ1��������(����0��4λ)
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;				//�����ж�������
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//�����������ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//������Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ��NVIC
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* ������  : Timer2_Init_Config
* ����    : Timer2��ʼ������
* ����    : ��
* ���    : ��
* ����    : �� 
* ˵��    : ��
*******************************************************************************/
void Timer2_Init_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		//ʹ��Timer2ʱ��
	
	TIM_TimeBaseStructure.TIM_Period = 99;						//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ(������10Ϊ1ms)
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;					//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ(10KHz�ļ���Ƶ��)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//����ʱ�ӷָ�:TDTS = TIM_CKD_DIV1
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);				//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	 
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE ); 				//ʹ��TIM2ָ�����ж�
	
	TIM_Cmd(TIM2, ENABLE);  									//ʹ��TIMx����
}

/*******************************************************************************
* ������  : TIM2_IRQHandler
* ����    : ��ʱ��2�ж϶Ϸ�����
* ����    : ��
* ���    : ��
* ����    : �� 
* ˵��    : ��
*******************************************************************************/
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		Timer2_Counter++;
		W5500_Send_Delay_Counter++;
		W5500_P_Delay_Counter++;
	}
}

/*******************************************************************************
* ������  : System_Initialization
* ����    : STM32ϵͳ��ʼ������(��ʼ��STM32ʱ�Ӽ�����)
* ����    : ��
* ���    : ��
* ����    : �� 
* ˵��    : ��
*******************************************************************************/
void System_Initialization(void)
{
	RCC_Configuration();		//����ϵͳʱ��Ϊ72MHZ(������Ը�����Ҫ��)
  	NVIC_Configuration();		//STM32�ж�������������
	SPI_Configuration();		//W5500 SPI��ʼ������(STM32 SPI1)
	Timer2_Init_Config();		//Timer2��ʼ������
	W5500_GPIO_Configuration();	//W5500 GPIO��ʼ������	
}

/*******************************************************************************
* ������  : Delay
* ����    : ��ʱ����(ms)
* ����    : d:��ʱϵ������λΪ10����
* ���    : ��
* ����    : �� 
* ˵��    : ��ʱ������Timer2��ʱ��������10����ļ�����ʵ�ֵ�
*******************************************************************************/
void Delay(unsigned int d)
{
	Timer2_Counter=0; 
	while(Timer2_Counter < d);
}
