/***************************************************************************************
 * 描述    ：W5500的端口0工作在客户端模式,主动与《TCP&UDP测试工具》上创建的服务端连接,
 *			 并且以500ms的时间间隔定时给服务端发送字符串"\r\nWelcome To YiXinElec!\r\n",同时将接
 *			 收到服务端发来的数据回发给服务端。
 * 实验平台：用户STM32开发板 + YIXIN_W5500以太网(TCP/IP)模块
 * 硬件连接：  PC5 -> W5500_RST       
 *             PA4 -> W5500_SCS      
 *             PA5 -> W5500_SCK    
 *             PA6 -> W5500_MISO    
 *             PA7 -> W5500_MOSI    
 * 库版本  ：ST_v3.5

 * 淘宝    ：http://yixindianzikeji.taobao.com/
***************************************************************************************/

/*例程网络参数*/
//网关：192.168.1.1
//掩码:	255.255.255.0
//物理地址：0C 29 AB 7C 00 01
//本机IP地址:192.168.1.199
//端口0的端口号：5000
//端口0的目的IP地址：192.168.1.190
//端口0的目的端口号：6000

#include "stm32f10x.h"		
#include "W5500.h"			
#include "stmflash.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


void RCC_Configuration(void);		//设置系统时钟为72MHZ(这个可以根据需要改)
void NVIC_Configuration(void);		//STM32中断向量表配配置
void Timer2_Init_Config(void);		//Timer2初始化配置
void System_Initialization(void);	//STM32系统初始化函数(初始化STM32时钟及外设)
void Delay(unsigned int d);			//延时函数(10ms)

extern void Yoyung_GPIO_Init(void);

unsigned int Timer2_Counter=0; //Timer2定时器计数变量(ms)
unsigned int W5500_Send_Delay_Counter=0; //W5500发送延时计数变量(*10 ms)
unsigned int W5500_P_Delay_Counter=0; //W5500心跳包延时计数变量(*10 ms)

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
//flash中 0~99半字 SetIP_Default   100~199半字 SetIP_User

//要写入到STM32 FLASH的字符串数组

char Command[30] = {0};
//char Part[6][10] = {0};
//char * p = NULL;
//unsigned char j=0;
//const u8 TEXT_Buffer[]={"STM32F103 FLASH TEST"};
#define SIZE sizeof(SET_IP)		//数组长度  * 2 < 1024
#define FLASH_SAVE_ADDR  0X0800FC00		//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)

/*******************************************************************************
* 函数名  : W5500_Initialization
* 描述    : W5500初始货配置
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 无
*******************************************************************************/
void W5500_Initialization(void)
{
	W5500_Init();		//初始化W5500寄存器函数
	Detect_Gateway();	//检查网关服务器 
	Socket_Init(0);		//指定Socket(0~7)初始化,初始化端口0
}

/*******************************************************************************
* 函数名  : Load_Net_Parameters
* 描述    : 装载网络参数
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 网关、掩码、物理地址、本机IP地址、端口号、目的IP地址、目的端口号、端口工作模式
*******************************************************************************/
void Load_Net_Parameters(void)
{
	Gateway_IP[0] = SetIP_User.Gateway_IP[0]%256;//加载网关参数
	Gateway_IP[1] = SetIP_User.Gateway_IP[1]%256;
	Gateway_IP[2] = SetIP_User.Gateway_IP[2]%256;
	Gateway_IP[3] = SetIP_User.Gateway_IP[3]%256;

	Sub_Mask[0]=SetIP_User.Sub_Mask[0]%256;//加载子网掩码
	Sub_Mask[1]=SetIP_User.Sub_Mask[1]%256;
	Sub_Mask[2]=SetIP_User.Sub_Mask[2]%256;
	Sub_Mask[3]=SetIP_User.Sub_Mask[3]%256;

	Phy_Addr[0]=SetIP_User.Phy_Addr[0]%256;//加载物理地址
	Phy_Addr[1]=SetIP_User.Phy_Addr[1]%256;
	Phy_Addr[2]=SetIP_User.Phy_Addr[2]%256;
	Phy_Addr[3]=SetIP_User.Phy_Addr[3]%256;
	Phy_Addr[4]=SetIP_User.Phy_Addr[4]%256;
	Phy_Addr[5]=SetIP_User.Phy_Addr[5]%256;

	IP_Addr[0]=SetIP_User.IP_Addr[0]%256;//加载本机IP地址
	IP_Addr[1]=SetIP_User.IP_Addr[1]%256;
	IP_Addr[2]=SetIP_User.IP_Addr[2]%256;
	IP_Addr[3]=SetIP_User.IP_Addr[3]%256;

	S0_Port[0] = SetIP_User.S0_Port/256;//加载端口0的端口号5000 
	S0_Port[1] = SetIP_User.S0_Port%256;

	S0_DIP[0]=SetIP_User.S0_DIP[0]%256;//加载端口0的目的IP地址
	S0_DIP[1]=SetIP_User.S0_DIP[1]%256;
	S0_DIP[2]=SetIP_User.S0_DIP[2]%256;
	S0_DIP[3]=SetIP_User.S0_DIP[3]%256;
	
	S0_DPort[0] = SetIP_User.S0_DPort/256;//加载端口0的目的端口号6000
	S0_DPort[1] = SetIP_User.S0_DPort%256;

	S0_Mode=TCP_CLIENT;//加载端口0的工作模式,TCP客户端模式
}

/*******************************************************************************
* 函数名  : W5500_Socket_Set
* 描述    : W5500端口初始化配置
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 分别设置4个端口,根据端口工作模式,将端口置于TCP服务器、TCP客户端或UDP模式.
*			从端口状态字节Socket_State可以判断端口的工作情况
*******************************************************************************/
void W5500_Socket_Set(void)
{
	if(S0_State==0)//端口0初始化配置
	{
		if(S0_Mode==TCP_SERVER)//TCP服务器模式 
		{
			if(Socket_Listen(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else if(S0_Mode==TCP_CLIENT)//TCP客户端模式 
		{
			if(Socket_Connect(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else//UDP模式 
		{
			if(Socket_UDP(0)==TRUE)
				S0_State=S_INIT|S_CONN;
			else
				S0_State=0;
		}
	}
}

/*******************************************************************************
* 函数名  : Process_Socket_Data
* 描述    : W5500接收并发送接收到的数据
* 输入    : s:端口号
* 输出    : 无
* 返回值  : 无
* 说明    : 本过程先调用S_rx_process()从W5500的端口接收数据缓冲区读取数据,
*			然后将读取的数据从Rx_Buffer拷贝到Temp_Buffer缓冲区进行处理。
*			处理完毕，将数据从Temp_Buffer拷贝到Tx_Buffer缓冲区。调用S_tx_process()
*			发送数据。
*******************************************************************************/
void Process_Socket_Data(SOCKET s)
{
	unsigned short size;
	size=Read_SOCK_Data_Buffer(s, Rx_Buffer);
	memcpy(Tx_Buffer, Rx_Buffer, size);			
	Write_SOCK_Data_Buffer(s, Tx_Buffer, size);
}

/**
 * 初始化看门狗
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
 * 喂看门狗
 */
void IWDG_Feed(void)
{
    IWDG_ReloadCounter();    /*reload*/
}

/*******************************************************************************
* 函数名  : main
* 描述    : 主函数，用户程序从main函数开始运行
* 输入    : 无
* 输出    : 无
* 返回值  : int:返回值为一个16位整形数
* 说明    : 无
*******************************************************************************/
int main(void)
{								
	System_Initialization();	//STM32系统初始化函数(初始化STM32时钟及外设)
	
	//第一次运行程序，写入一次flash
	STMFLASH_Read(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);
	if(SetIP_User.Gateway_IP[0]==0xffff && SetIP_User.Gateway_IP[1]==0xffff)
	{
	memcpy((u16*)&SetIP_User,(u16*)&SetIP_Default,SIZE);
	STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)&SetIP_Default,SIZE);
	STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	
	}


//memcpy((u16*)&SetIP_User,(u16*)&SetIP_Default,SIZE);	
  STMFLASH_Read(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	//flash读取网络参数
	Load_Net_Parameters();		//装载网络参数	
	W5500_Hardware_Reset();		//硬件复位W5500
	W5500_Initialization();		//W5500初始货配置
	Yoyung_GPIO_Init();				// GPIO/开关系统 初始化
	
	//IP地址恢复默认
	if ( 0 == GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8) )   
	{
		memcpy((u16*)&SetIP_User,(u16*)&SetIP_Default,SIZE);
		STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //记录进flash
	}
	
//	IWDG_Init(4,625);//????????,????64,?????625,???????:64*625/40=1000ms=1s
	IWDG_Init(5,625);   //溢出时间为2秒
	
	while (1)
	{
		W5500_Socket_Set();//W5500端口初始化配置

		W5500_Interrupt_Process();//W5500中断处理程序框架

		if((S0_Data & S_RECEIVE) == S_RECEIVE)//如果Socket0接收到数据
		{
			S0_Data&=~S_RECEIVE;
			Process_Socket_Data(0);//W5500接收并发送接收到的数据
			
			//---- Add By Yoyung ---  应用协议
			switch(Rx_Buffer[0])
			{
				case 1: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_1);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_1);  //关
								break;
				case 2: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_2);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_2);  //关
								break;		
				case 3: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_3);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_3);  //关
								break;
				case 4: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_8);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_8);  //关
								break;		
				case 5: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_9);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_9);  //关
								break;
				case 6: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_10);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_10);  //关
								break;		
				case 7: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_11);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_11);  //关
								break;
				case 8: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_12);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_12);  //关
								break;
				case 9: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOA, GPIO_Pin_15);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOA, GPIO_Pin_15);  //关
								break;
				case 10: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_3);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_3);  //关
								break;							
				case 11: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_4);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_4);  //关
								break;								
				case 12: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_5);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_5);  //关
								break;							
				case 13: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_6);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_6);  //关
								break;								
				case 14: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_7);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_7);  //关
								break;					
				case 15: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_10);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_10);  //关
								break;					
				case 16: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_11);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_11);  //关
								break;					
				case 17: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_12);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_12);  //关
								break;					
				case 18: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_13);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_13);  //关
								break;					
				case 19: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_14);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_14);  //关
								break;					
				case 20: if(Rx_Buffer[1]==1)  GPIO_ResetBits(GPIOB, GPIO_Pin_15);  //开
								else if (Rx_Buffer[1]==2)  GPIO_SetBits(GPIOB, GPIO_Pin_15);  //关
								break;	
								
				case 'S':{
									/***** 将字符串拆解成几个小字符串 ******/
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
										STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //记录进flash
									}
									else if (0 == strncmp(Command,"Set Sub_Mask",strlen(Command))  )
									{
										SetIP_User.Sub_Mask[0] = atoi(Part[0])%65536;
										SetIP_User.Sub_Mask[1] = atoi(Part[1])%65536;
										SetIP_User.Sub_Mask[2] = atoi(Part[2])%65536;
										SetIP_User.Sub_Mask[3] = atoi(Part[3])%65536;
										STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //记录进flash
									}
									else if (0 == strncmp(Command,"Set Phy_Addr",strlen("Set Phhy_Addr"))  )
									{
										SetIP_User.Phy_Addr[0] = atoi(Part[0])%65536;
										SetIP_User.Phy_Addr[1] = atoi(Part[1])%65536;
										SetIP_User.Phy_Addr[2] = atoi(Part[2])%65536;
										SetIP_User.Phy_Addr[3] = atoi(Part[3])%65536;
										SetIP_User.Phy_Addr[4] = atoi(Part[4])%65536;
										SetIP_User.Phy_Addr[5] = atoi(Part[5])%65536;
										STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //记录进flash
									}
									else if (0 == strncmp(Command,"Set IP_Addr",strlen(Command))  )
									{
										SetIP_User.IP_Addr[0] = atoi(Part[0])%65536;
										SetIP_User.IP_Addr[1] = atoi(Part[1])%65536;
										SetIP_User.IP_Addr[2] = atoi(Part[2])%65536;
										SetIP_User.IP_Addr[3] = atoi(Part[3])%65536;
										STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //记录进flash
									}
									else if (0 == strncmp(Command,"Set S0_Port",strlen(Command))  )
									{
										SetIP_User.S0_Port = atoi(Part[0])%65536;
										STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //记录进flash
									}
									else if (0 == strncmp(Command,"Set S0_DIP",strlen(Command))  )
									{
										SetIP_User.S0_DIP[0] = atoi(Part[0])%65536;
										SetIP_User.S0_DIP[1] = atoi(Part[1])%65536;
										SetIP_User.S0_DIP[2] = atoi(Part[2])%65536;
										SetIP_User.S0_DIP[3] = atoi(Part[3])%65536;			
										STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //记录进flash										
									}									
									else if (0 == strncmp(Command,"Set S0_DPort_",strlen(Command))  )
									{
										SetIP_User.S0_DPort = atoi(Part[0])%65536;
										STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //记录进flash
									}
									else if (0 == strncmp(Command,"Set Watch",strlen(Command))  )
									{
										snprintf( (char *)Tx_Buffer,sizeof(Tx_Buffer),
																							"配置信息: \r\n网关IP地址:%hd.%hd.%hd.%hd \r\n子网掩码:%hd.%hd.%hd.%hd \r\n物理地址MAC:%x.%x.%x.%x.%x.%x \r\n本机IP地址:%hd.%hd.%hd.%hd \r\n端口0的端口号:%hd \r\n端口0目的IP地址:%hd.%hd.%hd.%hd \r\n端口0目的端口号:%hd \r\n",
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
									
//									STMFLASH_Write(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	 //记录进flash
									//重新上电完成 更新配置到W5500模块
									
									}
									break;
									
				case 'P' :   //心跳包 ,接收
									Rx_Buffer[0] = 0;
									
									W5500_P_Delay_Counter = 0;
									break;
				default : break;										
			}
			
			
		}
		else if(W5500_Send_Delay_Counter >= 50)//定时发送字符串
		{
			if(S0_State == (S_INIT|S_CONN))
			{
				S0_Data&=~S_TRANSMITOK;
				memcpy(Tx_Buffer, "\r\nWelcome To YiXinElec!\r\n", 23);	
				Write_SOCK_Data_Buffer(0, Tx_Buffer, 23);//指定Socket(0~7)发送数据处理,端口0发送23字节数据
			}
			W5500_Send_Delay_Counter=0;
		}
		
		/****30S内接收不到心跳包 的处理 *******/
		if(W5500_P_Delay_Counter>3000)
		{
			System_Initialization();
			STMFLASH_Read(FLASH_SAVE_ADDR+200,(u16*)&SetIP_User,SIZE);	//flash读取网络参数
			Load_Net_Parameters();		//装载网络参数	
			W5500_Hardware_Reset();		//硬件复位W5500
			W5500_Initialization();		//W5500初始货配置
			W5500_Socket_Set();//W5500端口初始化配置
			W5500_Interrupt_Process();//W5500中断处理程序框架
			
			
			W5500_P_Delay_Counter = 0;
		}
		
		
		IWDG_Feed();
	}
}

/*******************************************************************************
* 函数名  : RCC_Configuration
* 描述    : 设置系统时钟为72MHZ(这个可以根据需要改)
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : STM32F107x和STM32F105x系列MCU与STM32F103x系列MCU时钟配置有所不同
*******************************************************************************/
void RCC_Configuration(void)
{
  ErrorStatus HSEStartUpStatus;               //外部高速时钟(HSE)的工作状态变量
  
  RCC_DeInit();                               //将所有与时钟相关的寄存器设置为默认值
  RCC_HSEConfig(RCC_HSE_ON);                  //启动外部高速时钟HSE 
  HSEStartUpStatus = RCC_WaitForHSEStartUp(); //等待外部高速时钟(HSE)稳定

  if(SUCCESS == HSEStartUpStatus)             //如果外部高速时钟已经稳定
  {
    /* Enable Prefetch Buffer */
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); //Flash设置
    /* Flash 2 wait state */
    FLASH_SetLatency(FLASH_Latency_2);
    
  
    RCC_HCLKConfig(RCC_SYSCLK_Div1); //设置AHB时钟等于系统时钟(1分频)/72MHZ
    RCC_PCLK2Config(RCC_HCLK_Div1);  //设置APB2时钟和HCLK时钟相等/72MHz(最大为72MHz)
    RCC_PCLK1Config(RCC_HCLK_Div2);  //设置APB1时钟是HCLK时钟的2分频/36MHz(最大为36MHz)
  
#ifndef STM32F10X_CL                 //如果使用的不是STM32F107x或STM32F105x系列MCU,PLL以下配置  
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); //PLLCLK = 8MHz * 9 = 72 MHz 
#else                                //如果使用的是STM32F107x或STM32F105x系列MCU,PLL以下配置
    /***** 配置PLLx *****/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    RCC_PREDIV2Config(RCC_PREDIV2_Div5);
    RCC_PLL2Config(RCC_PLL2Mul_8);

    RCC_PLL2Cmd(ENABLE); //使能PLL2 
    while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET);//等待PLL2稳定

    /* PLL configuration: PLLCLK = (PLL2 / 5) * 9 = 72 MHz */ 
    RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);
#endif

    RCC_PLLCmd(ENABLE); //使能PLL
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); //等待PLL稳定

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);          //设置系统时钟的时钟源为PLL

    while(RCC_GetSYSCLKSource() != 0x08);               //检查系统的时钟源是否是PLL
    RCC_ClockSecuritySystemCmd(ENABLE);                 //使能系统安全时钟 

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
* 函数名  : NVIC_Configuration
* 描述    : STM32中断向量表配配置
* 输入    : 无
* 输出    : 无
* 返回值  : 无
* 说明    : 设置KEY1(PC11)的中断优先组
*******************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;						//定义NVIC初始化结构体

  	/* Set the Vector Table base location at 0x08000000 */
  	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);				//设置中断优先级组为1，优先组(可设0～4位)
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;				//设置中断向量号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//设置抢先优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//设置响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能NVIC
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* 函数名  : Timer2_Init_Config
* 描述    : Timer2初始化配置
* 输入    : 无
* 输出    : 无
* 返回    : 无 
* 说明    : 无
*******************************************************************************/
void Timer2_Init_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);		//使能Timer2时钟
	
	TIM_TimeBaseStructure.TIM_Period = 99;						//设置在下一个更新事件装入活动的自动重装载寄存器周期的值(计数到10为1ms)
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;					//设置用来作为TIMx时钟频率除数的预分频值(10KHz的计数频率)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//设置时钟分割:TDTS = TIM_CKD_DIV1
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);				//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	 
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE ); 				//使能TIM2指定的中断
	
	TIM_Cmd(TIM2, ENABLE);  									//使能TIMx外设
}

/*******************************************************************************
* 函数名  : TIM2_IRQHandler
* 描述    : 定时器2中断断服务函数
* 输入    : 无
* 输出    : 无
* 返回    : 无 
* 说明    : 无
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
* 函数名  : System_Initialization
* 描述    : STM32系统初始化函数(初始化STM32时钟及外设)
* 输入    : 无
* 输出    : 无
* 返回    : 无 
* 说明    : 无
*******************************************************************************/
void System_Initialization(void)
{
	RCC_Configuration();		//设置系统时钟为72MHZ(这个可以根据需要改)
  	NVIC_Configuration();		//STM32中断向量表配配置
	SPI_Configuration();		//W5500 SPI初始化配置(STM32 SPI1)
	Timer2_Init_Config();		//Timer2初始化配置
	W5500_GPIO_Configuration();	//W5500 GPIO初始化配置	
}

/*******************************************************************************
* 函数名  : Delay
* 描述    : 延时函数(ms)
* 输入    : d:延时系数，单位为10毫秒
* 输出    : 无
* 返回    : 无 
* 说明    : 延时是利用Timer2定时器产生的10毫秒的计数来实现的
*******************************************************************************/
void Delay(unsigned int d)
{
	Timer2_Counter=0; 
	while(Timer2_Counter < d);
}
