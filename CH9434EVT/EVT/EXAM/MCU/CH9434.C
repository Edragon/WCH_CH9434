/********************************** (C) COPYRIGHT *******************************
* File Name          : CH9434.c
* Author             : tech18
* Version            : V1.0
* Date               : 2020/05/08
* Description        : SPI转串口芯片CH9434操作接口
*******************************************************************************/

#include "CH9434.h"

/*
一、芯片时钟配置相关说明：
1.外部晶振：32M
2.内部时钟频率：32M
3.开启倍频系数：15（固定）
4.芯片设置最高时钟频率不超过40M
5.串口基准时钟为：不开启倍频：32MHz  开启倍频：32MHz*15/分频系数

二、推荐常见波特率的计算方式
1.32M -> 串口基准时钟计算波特率：32M/8/波特率
2.32*15/13=36.923M -> 串口基准时钟计算波特率：36.923M/8/波特率 （如波特率：921600）

三、变量存储定义
1.osc_xt_frequency：记录外部晶振频率，当使用外部晶振时记录，调用CH9434OscXtFreqSet修改
2.sys_frequency：根据配置的时钟模式，得出的CH9434串口基准时钟，用于后面计算波特率
3.lower_power_reg：记录CH9434低功耗状态
4.ch9434_gpio_x_val：CH9434通用GPIO的输出电平值，按位定义，控制引脚电平函数：CH9434GPIOPinOut
*/

//定义外部晶振频率
u32_t osc_xt_frequency = 32000000;

//定义当前串口基准时钟频率
u32_t sys_frequency = 32000000;  //芯片默认为内部32M

//睡眠模式
u8_t  lower_power_reg = 0;

//GPIO的输出电平值 共24个GPIO
u32_t ch9434_gpio_x_val = 0;

/*******************************************************************************
* Function Name  : CH9434OscXtFreqSet
* Description    : 外部晶振频率记录
* Input          : x_freq：当前芯片连接的晶振频率
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434OscXtFreqSet(u32_t x_freq)
{
	osc_xt_frequency = x_freq;
}

/*******************************************************************************
* Function Name  : CH9434InitClkMode
* Description    : CH9434芯片时钟模式设置
* Input          : xt_en：外部晶振使能
                   freq_mul_en：倍频功能使能
                   div_num：分频系数
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434InitClkMode(u8_t xt_en,u8_t freq_mul_en,u8_t div_num)
{
	u8_t  clk_ctrl_reg;
	u16_t i;
	
	clk_ctrl_reg = 0;
	if(freq_mul_en) clk_ctrl_reg |= (1<<7);
	if(xt_en) clk_ctrl_reg |= (1<<6);
	clk_ctrl_reg |= (div_num&0x1f);
	
	/* 计算当前的串口基准时钟 */
	//sys_frequency
	switch(clk_ctrl_reg&0xc0)
	{
		case 0x00: //内部32M提供时钟
			sys_frequency = 32000000;
			break;
		case 0x40: //外部晶振提供时钟
			if((osc_xt_frequency>36000000)||(osc_xt_frequency<24000000)) //时钟错误
			{
				return;
			}
			sys_frequency = osc_xt_frequency;
			break;
		case 0x80: //使用内部32M，并开启倍频
			sys_frequency = 480000000/(div_num&0x1f);
			if(sys_frequency>40000000) //时钟错误
			{
				sys_frequency = 32000000;
				return;
			}
			break;
		case 0xc0: //使用外部晶振，并开启倍频
			if((osc_xt_frequency>36000000)||(osc_xt_frequency<24000000)) //时钟错误
			{
				return;
			}
			sys_frequency = osc_xt_frequency*15/(div_num&0x1f);
			if(sys_frequency>40000000) //时钟错误
			{
				sys_frequency = 32000000;
				return;
			}
			break;
	}
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_CLK_CTRL_CFG_ADD);
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(clk_ctrl_reg);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
	for(i=0; i<50000; i++) CH9434_US_DELAY();  //切换时钟需要延时
}

/*******************************************************************************
* Function Name  : CH9434UARTxParaSet
* Description    : 串口参数设置
* Input          : uart_idx：串口号
                   bps：串口的波特率
                   data_bits：数据位
                   stop_bits：停止位
                   veri_bits：校验位
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434UARTxParaSet(u8_t uart_idx,u32_t bps,u8_t data_bits,u8_t stop_bits,u8_t veri_bits)
{
	u8_t  uart_reg_dll;
	u8_t  uart_reg_dlm;
	u32_t x;
	u8_t  uart_reg_lcr;
	
	x = 10 * sys_frequency / 8 / bps;
	x = ( x + 5 ) / 10;  	
	
	uart_reg_dll = x&0xff;
	uart_reg_dlm = (x>>8)&0xff;
	
	//DLAB置位 设置LCR寄存器
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_UARTx_LCR_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	uart_reg_lcr = CH9434_SPI_WRITE_BYTE(0xff);
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
	
	uart_reg_lcr |= CH9434_UARTx_BIT_DLAB;
	//数据位
	uart_reg_lcr &= ~0x03;
	switch(data_bits)
	{
		case CH9434_UART_5_BITS_PER_CHAR:
			break;
		case CH9434_UART_6_BITS_PER_CHAR:
			uart_reg_lcr |= 0x01;
			break;
		case CH9434_UART_7_BITS_PER_CHAR:
			uart_reg_lcr |= 0x02;
			break;
		case CH9434_UART_8_BITS_PER_CHAR:
			uart_reg_lcr |= 0x03;
			break;
		default:
			uart_reg_lcr |= 0x03;
			break;	
	}
	//停止位
	uart_reg_lcr &= ~(1<<2);
	if(stop_bits == CH9434_UART_TWO_STOP_BITS)
	{
		uart_reg_lcr |= (1<<2);
	}
	//校验位
	uart_reg_lcr &= ~(1<<3);
	uart_reg_lcr &= ~(3<<4);
	switch(veri_bits)
	{
		case CH9434_UART_NO_PARITY:
			break;
		case CH9434_UART_ODD_PARITY:
			uart_reg_lcr |= (1<<3);
			break;
		case CH9434_UART_EVEN_PARITY:
			uart_reg_lcr |= (1<<3);
			uart_reg_lcr |= (1<<4);
			break;
		case CH9434_UART_MARK_PARITY:
			uart_reg_lcr |= (1<<3);
			uart_reg_lcr |= (2<<4);
			break;
		case CH9434_UART_SPACE_PARITY:
			uart_reg_lcr |= (1<<3);
			uart_reg_lcr |= (3<<4);
			break;
		default:
			break;	
	}

	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_UARTx_LCR_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(uart_reg_lcr);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
	
	//设置DLL DLM
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_UARTx_DLL_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(uart_reg_dll);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_UARTx_DLM_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(uart_reg_dlm);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
	
	//DLAB清0
	uart_reg_lcr &= ~CH9434_UARTx_BIT_DLAB;
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_UARTx_LCR_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(uart_reg_lcr);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
}

/*******************************************************************************
* Function Name  : CH9434UARTxFIFOSet
* Description    : 串口FIFO设置
* Input          : uart_idx：串口号
                   fifo_en：FIFO功能使能
                   fifo_level：FIFO触发等级
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434UARTxFIFOSet(u8_t uart_idx,u8_t fifo_en,u8_t fifo_level)
{
	u8_t  uart_reg_fcr;
	
	uart_reg_fcr = 0;
	if(fifo_en)
	{
		uart_reg_fcr |= 0x01;
		uart_reg_fcr |= fifo_level<<6;
	}
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_UARTx_FCR_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(uart_reg_fcr);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
}

/*******************************************************************************
* Function Name  : CH9434UARTxIrqSet
* Description    : 串口中断设置
* Input          : uart_idx：串口号
                   modem：modem信号中断
                   line：线路状态中断
                   tx：发送中断
                   rx：接收中断
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434UARTxIrqSet(u8_t uart_idx,u8_t modem,u8_t line,u8_t tx,u8_t rx)
{
	u8_t  uart_reg_ier;
	
	uart_reg_ier = 0;
	if(modem) uart_reg_ier |= (1<<3);
	if(line)	uart_reg_ier |= (1<<2);
	if(tx)	  uart_reg_ier |= (1<<1);
	if(rx)	  uart_reg_ier |= (1<<0);
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_UARTx_IER_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(uart_reg_ier);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
}

/* 流控功能和引脚设置 */
/*******************************************************************************
* Function Name  : CH9434UARTxFlowSet
* Description    : 流控设置
* Input          : uart_idx：串口号
                   flow_en：流控使能
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434UARTxFlowSet(u8_t uart_idx,u8_t flow_en)
{
	u8_t  uart_reg_mcr;
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_UARTx_MCR_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	uart_reg_mcr = CH9434_SPI_WRITE_BYTE(0xff);
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);	
	
	uart_reg_mcr &=~(1<<5);
	if(flow_en) uart_reg_mcr |= (1<<5);
		
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_UARTx_MCR_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(uart_reg_mcr);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);	
}

/*******************************************************************************
* Function Name  : CH9434UARTxIrqOpen
* Description    : 开启中断串口请求
* Input          : uart_idx：串口号
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434UARTxIrqOpen(u8_t uart_idx)
{
	u8_t  uart_reg_mcr;
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_UARTx_MCR_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	uart_reg_mcr = CH9434_SPI_WRITE_BYTE(0xff);
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);	
	
	uart_reg_mcr |= (1<<3);
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_UARTx_MCR_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(uart_reg_mcr);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
}

/*******************************************************************************
* Function Name  : CH9434UARTxRtsDtrPin
* Description    : 设置串口RTS、DTR引脚
* Input          : uart_idx：串口号
                   rts_val：RTS引脚电平状态
                   dtr_val：DTR引脚电平状态
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434UARTxRtsDtrPin(u8_t uart_idx,u8_t rts_val,u8_t dtr_val)
{
	u8_t  uart_reg_mcr;
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_UARTx_MCR_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	uart_reg_mcr = CH9434_SPI_WRITE_BYTE(0xff);
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);	
	
	if(rts_val) uart_reg_mcr |= (1<<1);
	else        uart_reg_mcr &= ~(1<<1);
	if(dtr_val) uart_reg_mcr |= (1<<0);
	else        uart_reg_mcr &= ~(1<<0);
		
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_UARTx_MCR_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(uart_reg_mcr);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
}

/*******************************************************************************
* Function Name  : CH9434UARTxWriteSRC
* Description    : SRC寄存器写操作
* Input          : uart_idx：串口号
                   src_val：SRC寄存器值
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434UARTxWriteSRC(u8_t uart_idx,u8_t src_val)
{
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_UARTx_SCR_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(src_val);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
}

/*******************************************************************************
* Function Name  : CH9434UARTxReadSRC
* Description    : SRC寄存器读操作
* Input          : uart_idx：串口号
* Output         : None
* Return         : SRC寄存器值
*******************************************************************************/
u8_t CH9434UARTxReadSRC(u8_t uart_idx)
{
	u8_t  uart_reg_src = 0;
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_UARTx_SCR_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	uart_reg_src = CH9434_SPI_WRITE_BYTE(0xff);
	CH9434_SPI_SCS_OP(CH9434_ENABLE);	
	
	return uart_reg_src;
}

/*******************************************************************************
* Function Name  : CH9434UARTxReadIIR
* Description    : 串口中断码查询
* Input          : uart_idx：串口号
* Output         : None
* Return         : IIR寄存器值
*******************************************************************************/
u8_t CH9434UARTxReadIIR(u8_t uart_idx)
{
	u8_t  uart_reg_iir = 0;
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_UARTx_IIR_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	uart_reg_iir = CH9434_SPI_WRITE_BYTE(0xff);
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
	
	return uart_reg_iir;
}

/*******************************************************************************
* Function Name  : CH9434UARTxReadLSR
* Description    : 串口LSR寄存器读取
* Input          : uart_idx：串口号
* Output         : None
* Return         : LSR寄存器值
*******************************************************************************/
u8_t CH9434UARTxReadLSR(u8_t uart_idx)
{
	u8_t  uart_reg_lsr = 0;
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_UARTx_LSR_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	uart_reg_lsr = CH9434_SPI_WRITE_BYTE(0xff);
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
	
	return uart_reg_lsr;
}

/*******************************************************************************
* Function Name  : CH9434UARTxReadMSR
* Description    : 串口MSR寄存器读取
* Input          : uart_idx：串口号
* Output         : None
* Return         : MSR寄存器值
*******************************************************************************/
u8_t CH9434UARTxReadMSR(u8_t uart_idx)
{
	u8_t  uart_reg_msr = 0;
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_UARTx_MSR_ADD+0x10*uart_idx);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	uart_reg_msr = CH9434_SPI_WRITE_BYTE(0xff);
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
	
	return uart_reg_msr;
}

/* 串口收发数据 */
/*******************************************************************************
* Function Name  : CH9434UARTxGetRxFIFOLen
* Description    : 获取串口接收数据长度
* Input          : uart_idx：串口号
* Output         : None
* Return         : 串口接收FIFO的大小
*******************************************************************************/
u16_t CH9434UARTxGetRxFIFOLen(u8_t uart_idx)
{
	u8_t  uart_fifo_ctrl = 0;
	u8_t  uart_fifo_cnt_l;
	u8_t  uart_fifo_cnt_h;
	u16_t uart_fifo_cnt = 0;
	
	uart_fifo_ctrl |= uart_idx;
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_FIFO_CTRL_ADD);
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(uart_fifo_ctrl);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	//CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);	
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_FIFO_CTRL_L_ADD);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	//CH9434_US_DELAY();
	uart_fifo_cnt_l = CH9434_SPI_WRITE_BYTE(0xff);
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_FIFO_CTRL_H_ADD);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	//CH9434_US_DELAY();
	uart_fifo_cnt_h = CH9434_SPI_WRITE_BYTE(0xff);
	CH9434_SPI_SCS_OP(CH9434_ENABLE);

	uart_fifo_cnt = uart_fifo_cnt_h;
	uart_fifo_cnt = (uart_fifo_cnt<<8) | uart_fifo_cnt_l;

	return uart_fifo_cnt;
}

/*******************************************************************************
* Function Name  : CH9434UARTxGetRxFIFOData
* Description    : 读取串口接收数据
* Input          : uart_idx：串口号
                   p_data：数据存储指针
                   read_len：读取的数据长度
* Output         : None
* Return         : 空
*******************************************************************************/
u8_t CH9434UARTxGetRxFIFOData(u8_t uart_idx,u8_t *p_data,u16_t read_len)
{
	u16_t i;
	u8_t  *p_sv_data;
	u8_t  uart_reg_add;
	
	uart_reg_add = CH9434_REG_OP_READ|CH9434_UARTx_RBR_ADD+0x10*uart_idx;
	p_sv_data = p_data;
	for(i=0; i<read_len; i++)
	{
		CH9434_SPI_SCS_OP(CH9434_DISABLE);
		CH9434_SPI_WRITE_BYTE(uart_reg_add);
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		*p_sv_data++ = CH9434_SPI_WRITE_BYTE(0xff);
		CH9434_US_DELAY();
		CH9434_SPI_SCS_OP(CH9434_ENABLE);
	}
	
	return 0;
}

/*******************************************************************************
* Function Name  : CH9434UARTxGetTxFIFOLen
* Description    : 获取串口发送FIFO长度
* Input          : uart_idx：串口号
* Output         : None
* Return         : 当前串口的接收数据长度
*******************************************************************************/
u16_t CH9434UARTxGetTxFIFOLen(u8_t uart_idx)
{
	u8_t  uart_fifo_ctrl = 0;
	u8_t  uart_fifo_cnt_l;
	u8_t  uart_fifo_cnt_h;
	u16_t uart_fifo_cnt = 0;
	
	uart_fifo_ctrl |= CH9434_FIFO_CTRL_TR;
	uart_fifo_ctrl |= uart_idx;
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_FIFO_CTRL_ADD);
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(uart_fifo_ctrl);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);	
		
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_FIFO_CTRL_L_ADD);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	uart_fifo_cnt_l = CH9434_SPI_WRITE_BYTE(0xff);
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_FIFO_CTRL_H_ADD);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	uart_fifo_cnt_h = CH9434_SPI_WRITE_BYTE(0xff);
	CH9434_SPI_SCS_OP(CH9434_ENABLE);

	uart_fifo_cnt = uart_fifo_cnt_h;
	uart_fifo_cnt = (uart_fifo_cnt<<8) | uart_fifo_cnt_l;

	return uart_fifo_cnt;
}

/*******************************************************************************
* Function Name  : CH9434UARTxSetTxFIFOData
* Description    : 串口填入发送数据
* Input          : uart_idx：串口号
                   p_data：发送数据指针
                   send_len：发送的数据长度
* Output         : None
* Return         : 空
*******************************************************************************/
u8_t CH9434UARTxSetTxFIFOData(u8_t uart_idx,u8_t *p_data,u16_t send_len)
{
	u16_t i;
	u8_t  *p_sv_data;
	u8_t  uart_reg_add;
	
	uart_reg_add = CH9434_REG_OP_WRITE|CH9434_UARTx_RBR_ADD+0x10*uart_idx;
	p_sv_data = p_data;
	for(i=0; i<send_len; i++)
	{
		CH9434_SPI_SCS_OP(CH9434_DISABLE);
		CH9434_SPI_WRITE_BYTE(uart_reg_add);
		CH9434_US_DELAY();
		CH9434_SPI_WRITE_BYTE(*p_sv_data++);
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_SPI_SCS_OP(CH9434_ENABLE);
	}
	
	return 0;
}

/*******************************************************************************
* Function Name  : CH9434UARTxTnowSet
* Description    : 串口485切换引脚设置
* Input          : uart_idx：串口号
                   tnow_en：串口tnow使能状态
                   polar：极性反向设置
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434UARTxTnowSet(u8_t uart_idx,u8_t tnow_en,u8_t polar)
{
	u8_t  tnow_ctrl_reg;
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_TNOW_CTRL_CFG_ADD);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	tnow_ctrl_reg = CH9434_SPI_WRITE_BYTE(0xff);
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
	
	if(tnow_en) tnow_ctrl_reg |= (1<<uart_idx);
	else        tnow_ctrl_reg &=~(1<<uart_idx);
		
	if(polar) tnow_ctrl_reg |= (1<<(uart_idx+4));
	else      tnow_ctrl_reg &=~(1<<(uart_idx+4));
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_TNOW_CTRL_CFG_ADD);
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(tnow_ctrl_reg);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
}

/*******************************************************************************
* Function Name  : CH9434LowerPowerModeSet
* Description    : CH9434芯片低功耗设置
* Input          : mode：低功耗模式
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434LowerPowerModeSet(u8_t mode)
{
	lower_power_reg = mode;
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_SLEEP_MOD_CFG_ADD);
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(lower_power_reg);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
}

/*******************************************************************************
* Function Name  : CH9434WakeUp
* Description    : CH9434唤醒操作，从低功耗模式中唤醒，也可操作SPI进行唤醒
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434WakeUp(void)
{
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
}

/*******************************************************************************
* Function Name  : CH9434GPIOFuncSet
* Description    : GPIO功能设置
* Input          : gpio_idx：GPIO序号
                   en：使能状态
                   dir：GPIO方向
                   pu：上拉设置
                   pd：下拉设置
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434GPIOFuncSet(u8_t gpio_idx,u8_t en,u8_t dir,u8_t pu,u8_t pd)
{
	u8_t gpio_func_reg;   //  GPIO_FUNC
	u8_t gpio_dir_reg;
	u8_t gpio_pu_reg;
	u8_t gpio_pd_reg;
	
	if(en)
	{
		//GPIO使能
		CH9434_SPI_SCS_OP(CH9434_DISABLE);
		CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_GPIO_FUNC_EN_0+(gpio_idx/8));
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		gpio_func_reg = CH9434_SPI_WRITE_BYTE(0xff);
		CH9434_SPI_SCS_OP(CH9434_ENABLE);
		
		gpio_func_reg |= (1<<(gpio_idx%8));
		
		CH9434_SPI_SCS_OP(CH9434_DISABLE);
		CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_GPIO_FUNC_EN_0+(gpio_idx/8));
		CH9434_US_DELAY();
		CH9434_SPI_WRITE_BYTE(gpio_func_reg);
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_SPI_SCS_OP(CH9434_ENABLE);
		
		//GPIO方向设置
		CH9434_SPI_SCS_OP(CH9434_DISABLE);
		CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_GPIO_DIR_MOD_0+(gpio_idx/8));
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		gpio_dir_reg = CH9434_SPI_WRITE_BYTE(0xff);
		CH9434_SPI_SCS_OP(CH9434_ENABLE);
		
		if(dir) gpio_dir_reg |= (1<<(gpio_idx%8));
		else    gpio_dir_reg &=~(1<<(gpio_idx%8));
		
		CH9434_SPI_SCS_OP(CH9434_DISABLE);
		CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_GPIO_DIR_MOD_0+(gpio_idx/8));
		CH9434_US_DELAY();
		CH9434_SPI_WRITE_BYTE(gpio_dir_reg);
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_SPI_SCS_OP(CH9434_ENABLE);
		
		//上拉设置
		CH9434_SPI_SCS_OP(CH9434_DISABLE);
		CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_GPIO_PU_MOD_0+(gpio_idx/8));
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		gpio_pu_reg = CH9434_SPI_WRITE_BYTE(0xff);
		CH9434_SPI_SCS_OP(CH9434_ENABLE);
		
		if(pu) gpio_pu_reg |= (1<<(gpio_idx%8));
		else   gpio_pu_reg &=~(1<<(gpio_idx%8));
		
		CH9434_SPI_SCS_OP(CH9434_DISABLE);
		CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_GPIO_PU_MOD_0+(gpio_idx/8));
		CH9434_US_DELAY();
		CH9434_SPI_WRITE_BYTE(gpio_pu_reg);
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_SPI_SCS_OP(CH9434_ENABLE);
		
		//下拉设置
		CH9434_SPI_SCS_OP(CH9434_DISABLE);
		CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_GPIO_PD_MOD_0+(gpio_idx/8));
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		gpio_pd_reg = CH9434_SPI_WRITE_BYTE(0xff);
		CH9434_SPI_SCS_OP(CH9434_ENABLE);
		
		if(pd) gpio_pd_reg |= (1<<(gpio_idx%8));
		else   gpio_pd_reg &=~(1<<(gpio_idx%8));
		
		CH9434_SPI_SCS_OP(CH9434_DISABLE);
		CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_GPIO_PD_MOD_0+(gpio_idx/8));
		CH9434_US_DELAY();
		CH9434_SPI_WRITE_BYTE(gpio_pd_reg);
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_SPI_SCS_OP(CH9434_ENABLE);
	}
	else
	{
		CH9434_SPI_SCS_OP(CH9434_DISABLE);
		CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_GPIO_FUNC_EN_0+(gpio_idx/8));
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		gpio_func_reg = CH9434_SPI_WRITE_BYTE(0xff);
		CH9434_SPI_SCS_OP(CH9434_ENABLE);
		
		gpio_func_reg &= ~(1<<(gpio_idx%8));
		
		CH9434_SPI_SCS_OP(CH9434_DISABLE);
		CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_GPIO_FUNC_EN_0+(gpio_idx/8));
		CH9434_US_DELAY();
		CH9434_SPI_WRITE_BYTE(gpio_func_reg);
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_US_DELAY();
		CH9434_SPI_SCS_OP(CH9434_ENABLE);
	}
}

/*******************************************************************************
* Function Name  : CH9434GPIOPinOut
* Description    : GPIO输出电平设置
* Input          : gpio_idx：GPIO序号
                   out_val：输出电平设置
* Output         : None
* Return         : None
*******************************************************************************/
void CH9434GPIOPinOut(u8_t gpio_idx,u8_t out_val)
{
	u8_t pin_val_reg;
	
	if(out_val) ch9434_gpio_x_val |= (1<<gpio_idx);
	else        ch9434_gpio_x_val &= ~(1<<gpio_idx);
	
	pin_val_reg = (u8_t)(ch9434_gpio_x_val>>((gpio_idx/8)*8));
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_WRITE|CH9434_GPIO_PIN_VAL_0+(gpio_idx/8));
	CH9434_US_DELAY();
	CH9434_SPI_WRITE_BYTE(pin_val_reg);
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_SPI_SCS_OP(CH9434_ENABLE);
}

/*******************************************************************************
* Function Name  : CH9434GPIOPinVal
* Description    : GPIO电平读取
* Input          : gpio_idx：GPIO序号
* Output         : None
* Return         : 电平状态：1：高电平 0：低电平
*******************************************************************************/
u8_t CH9434GPIOPinVal(u8_t gpio_idx)
{
	u8_t pin_val_reg;
	
	CH9434_SPI_SCS_OP(CH9434_DISABLE);
	CH9434_SPI_WRITE_BYTE(CH9434_REG_OP_READ|CH9434_GPIO_PIN_VAL_0+(gpio_idx/8));
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	CH9434_US_DELAY();
	pin_val_reg = CH9434_SPI_WRITE_BYTE(0xff);
	CH9434_SPI_SCS_OP(CH9434_ENABLE);	
	
	if(pin_val_reg & (1<<(gpio_idx%8))) return 1;
	else                                return 0;
}
