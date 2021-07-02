/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : tech18
* Version            : V1.0
* Date               : 2019/06/22
* Description        : CH9434操作示例，基于CH32F103例程编写
*******************************************************************************/

/******************************************************************************/
/* 头文件包含 */
#include "debug.h"
#include "string.h"
#include "CH9434.h"


uint8_t uart_idx;
uint8_t uart_iir;
uint8_t uart_lsr;
uint8_t uart_msr;

uint16_t rec_buf_cnt = 0;
uint8_t uart_rec_buf[512];

uint32_t uart_rec_total_cnt[4] = {0,0,0,0};

#define dg_log              printf


/* 打印串口初始化 */
void UARTPrintfInit(void)
{
	USART_Printf_Init(460800);
}

/* 定义CH9434接口函数 */
/* US延时函数 */
void CH9434_US_DELAY(void)
{
	Delay_Us(1);
}
 
/* SPI接口SCS引脚控制，0：低电平  1：高电平 */ 
void CH9434_SPI_SCS_OP(uint8_t dat)
{
	if(dat)  GPIO_SetBits(GPIOA, GPIO_Pin_4);  //SCS置高
	else     GPIO_ResetBits(GPIOA, GPIO_Pin_4);//SCS置低
}

/* SPI交换一个字节接口 */
uint8_t CH9434_SPI_WRITE_BYTE(uint8_t dat)
{
	SPI_I2S_SendData(SPI1, (uint16_t)dat);
	while(SPI_I2S_GetFlagStatus( SPI1, SPI_I2S_FLAG_TXE ) == RESET);
	return ( (uint8_t)SPI_I2S_ReceiveData( SPI1 ) );
}

/* SPI主机初始化 */
void SPIHostInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE );	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//SPI1_NSS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		
	GPIO_Init( GPIOA, &GPIO_InitStructure );
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//SPI1_SCK
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_InitStructure );	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//SPI1_MISO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;		
	GPIO_Init( GPIOA, &GPIO_InitStructure );	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//SPI1_MOSI
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_InitStructure );
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	
	
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;	

	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init( SPI1, &SPI_InitStructure );

	SPI_Cmd( SPI1, ENABLE );
}

/* INT#引脚初始化 */
void InitIntGPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PA0--INT#
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		
	GPIO_Init( GPIOA, &GPIO_InitStructure );
}
 
/*******************************************************************************
* Function Name  : main
* Description    : 主函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void) 
{
	uint32_t i;
	uint32_t test_bps;
	uint32_t tim_cnt = 0;
	uint8_t  pin_val = 0;
	
	Delay_Init();
	
	/* 延时一会 */
	Delay_Ms(50);
	
	/* 初始化一个串口用作打印 */
	UARTPrintfInit();
	
	//时钟初始化
	dg_log("CH9434 TEST START %s %s \r\n",__TIME__,__DATE__);
	
	SPIHostInit();
	
	/* SPI传输测试 */
	/* 初始化CH9434 */
	CH9434InitClkMode(CH9434_ENABLE, //外部晶振  
	                  CH9434_ENABLE, //开启倍频功能
                      13);//分频系数
	Delay_Ms(50);			  
					  
	/* 初始化串口 */				  
	test_bps = 115200;
	
	//初始化串口1
	CH9434UARTxParaSet(CH9434_UART_IDX_0,
	                   test_bps,
					   CH9434_UART_8_BITS_PER_CHAR,
					   CH9434_UART_ONE_STOP_BIT,
					   CH9434_UART_NO_PARITY);
	CH9434UARTxFIFOSet(CH9434_UART_IDX_0,
	                   CH9434_ENABLE,
					   CH9434_UART_FIFO_MODE_1280);	   
	CH9434UARTxFlowSet(CH9434_UART_IDX_0,
	                   CH9434_ENABLE);
	CH9434UARTxIrqSet(CH9434_UART_IDX_0,
	                  CH9434_DISABLE,    //modem信号中断
					  CH9434_ENABLE,    //线路状态中断
					  CH9434_ENABLE,    //发送中断
					  CH9434_ENABLE);   //接收中断
	CH9434UARTxIrqOpen(CH9434_UART_IDX_0);
	CH9434UARTxRtsDtrPin(CH9434_UART_IDX_0,
	                     CH9434_ENABLE,   //RTS引脚电平状态
						 CH9434_ENABLE);  //DTR引脚电平状态


	//初始化串口1
	CH9434UARTxParaSet(CH9434_UART_IDX_1,
	                   test_bps,
					   CH9434_UART_8_BITS_PER_CHAR,
					   CH9434_UART_ONE_STOP_BIT,
					   CH9434_UART_NO_PARITY);
	CH9434UARTxFIFOSet(CH9434_UART_IDX_1,
	                   CH9434_ENABLE,
					   CH9434_UART_FIFO_MODE_1280);	   
	CH9434UARTxFlowSet(CH9434_UART_IDX_1,
	                   CH9434_ENABLE);
	CH9434UARTxIrqSet(CH9434_UART_IDX_1,
	                  CH9434_DISABLE,    //modem信号中断
					  CH9434_ENABLE,    //线路状态中断
					  CH9434_ENABLE,    //发送中断
					  CH9434_ENABLE);   //接收中断
	CH9434UARTxIrqOpen(CH9434_UART_IDX_1);
	CH9434UARTxRtsDtrPin(CH9434_UART_IDX_1,
	                     CH9434_ENABLE,   //RTS引脚电平状态
						 CH9434_ENABLE);  //DTR引脚电平状态
						 
	//初始化串口2
	CH9434UARTxParaSet(CH9434_UART_IDX_2,
	                   test_bps,
					   CH9434_UART_8_BITS_PER_CHAR,
					   CH9434_UART_ONE_STOP_BIT,
					   CH9434_UART_NO_PARITY);
	CH9434UARTxFIFOSet(CH9434_UART_IDX_2,
	                   CH9434_ENABLE,
					   CH9434_UART_FIFO_MODE_1280);	   
	CH9434UARTxFlowSet(CH9434_UART_IDX_2,
	                   CH9434_ENABLE);
	CH9434UARTxIrqSet(CH9434_UART_IDX_2,
	                  CH9434_DISABLE,    //modem信号中断
					  CH9434_ENABLE,    //线路状态中断
					  CH9434_ENABLE,    //发送中断
					  CH9434_ENABLE);   //接收中断
	CH9434UARTxIrqOpen(CH9434_UART_IDX_2);
	CH9434UARTxRtsDtrPin(CH9434_UART_IDX_2,
	                     CH9434_ENABLE,   //RTS引脚电平状态
						 CH9434_ENABLE);  //DTR引脚电平状态					 
						 
	//初始化串口3
	CH9434UARTxParaSet(CH9434_UART_IDX_3,
	                   test_bps,
					   CH9434_UART_8_BITS_PER_CHAR,
					   CH9434_UART_ONE_STOP_BIT,
					   CH9434_UART_NO_PARITY);
	CH9434UARTxFIFOSet(CH9434_UART_IDX_3,
	                   CH9434_ENABLE,
					   CH9434_UART_FIFO_MODE_1280);	   
	CH9434UARTxFlowSet(CH9434_UART_IDX_3,
	                   CH9434_ENABLE);
	CH9434UARTxIrqSet(CH9434_UART_IDX_3,
	                  CH9434_DISABLE,    //modem信号中断
					  CH9434_ENABLE,    //线路状态中断
					  CH9434_ENABLE,    //发送中断
					  CH9434_ENABLE);   //接收中断
	CH9434UARTxIrqOpen(CH9434_UART_IDX_3);
	CH9434UARTxRtsDtrPin(CH9434_UART_IDX_3,
	                     CH9434_ENABLE,   //RTS引脚电平状态
						 CH9434_ENABLE);  //DTR引脚电平状态

	while(1)
	{
		/* 串口线程处理 */
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==Bit_RESET) //INT电平为低
		{
			for(uart_idx=0; uart_idx<4; uart_idx++)
			{
				uart_iir = CH9434UARTxReadIIR(uart_idx);
				dg_log("idx:%d uart_iir:%02x\r\n",uart_idx,uart_iir);
				switch(uart_iir & 0x0f)
				{
					case 0x01://没有中断
						break;
					case 0x06://接收线路状态
					{
						uart_lsr = CH9434UARTxReadLSR(uart_idx);
						dg_log("uart_lsr:%02x\r\n",uart_lsr);
						rec_buf_cnt = CH9434UARTxGetRxFIFOLen(uart_idx);
						if(rec_buf_cnt)
						{
							CH9434UARTxGetRxFIFOData(uart_idx,uart_rec_buf,rec_buf_cnt);
							uart_rec_total_cnt[uart_idx] += rec_buf_cnt;
							dg_log("idx:%d rec:%d total:%d\r\n",uart_idx,rec_buf_cnt,(int)uart_rec_total_cnt[uart_idx]);
							CH9434UARTxSetTxFIFOData(uart_idx,uart_rec_buf,rec_buf_cnt);
						}	
						break;
					}
					case 0x04://接收数据可用	
					{
						rec_buf_cnt = CH9434UARTxGetRxFIFOLen(uart_idx);
						if(rec_buf_cnt)
						{
							CH9434UARTxGetRxFIFOData(uart_idx,uart_rec_buf,rec_buf_cnt);
							uart_rec_total_cnt[uart_idx] += rec_buf_cnt;
							dg_log("idx:%d rec:%d total:%d\r\n",uart_idx,rec_buf_cnt,(int)uart_rec_total_cnt[uart_idx]);
							CH9434UARTxSetTxFIFOData(uart_idx,uart_rec_buf,rec_buf_cnt);
						}	
						break;
					}
					case 0x0C://接收数据超时
					{
						rec_buf_cnt = CH9434UARTxGetRxFIFOLen(uart_idx);
						if(rec_buf_cnt)
						{
							CH9434UARTxGetRxFIFOData(uart_idx,uart_rec_buf,rec_buf_cnt);
							uart_rec_total_cnt[uart_idx] += rec_buf_cnt;
							dg_log("idx:%d rec:%d total:%d\r\n",uart_idx,rec_buf_cnt,(int)uart_rec_total_cnt[uart_idx]);
							CH9434UARTxSetTxFIFOData(uart_idx,uart_rec_buf,rec_buf_cnt);
						}	
						break;
					}
					case 0x02://THR寄存器空
						break;
					case 0x00://modem信号变化
					{
						uart_msr = CH9434UARTxReadMSR(uart_idx);
						dg_log("uart_msr:%02x\r\n",uart_msr);
						break;
					}
				}
			}
		}
	}
}

/*************************************************************************************************
**************************************************************************************************/
