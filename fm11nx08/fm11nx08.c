/************************************************************
 Shanghai Fudan Microelectronics Group Company Limited

 file name:FM11_demo.c

 version:v0.10

 project:YX1323
*************************************************************/


#include "fm11nx08.h"
#include "bsp_spi.h"




uint8_t FlagFirstFrame = OFF;			//卡片首帧标识
uint32_t FSDI = 64-2;		//-4帧长度PCD
uint8_t CID = 0;
uint8_t block_num = 1;
uint8_t irq_data_in = 0;		//非接数据接收终端标识
uint8_t irq_rxdone = 0;
uint8_t irq_txdone = 0;
uint8_t rfLen;
uint8_t rfBuf[255];
uint8_t irq_data_wl = 0;

void pt_delay_ms(uint32_t delayMs)
{
    uint32_t j;
    /* by experimination, this is in ms (approx) */
    for (j = 0; j < 4010 * delayMs; j++)
    {
        asm(" NOP");
    }
}

void pt_delay_us(uint32_t delayUs)
{
    uint32_t j;
    for (j = 0; j < 4 * delayUs; j++)
    {
        asm(" NOP");
    }
}


#define Delay_10us(val)        {pt_delay_us(val*10);}



/*********************************************************************************************************
** 函数名称:	FM11_RF_Tx
** 函数功能:	RF数据回发
** 输入参数:   len:回发数据长度
** 			buf:回发的数据
** 输出参数:   无
** 返回值:   无
*********************************************************************************************************/
void FM11_RF_Tx(uint32_t len,uint8_t *txbuf)
{
    uint32_t slen;
    uint8_t *sbuf;

    slen = len;
    sbuf = txbuf;
// 	cnt = 0;

    if(slen <= 32)
    {
        FM11_Serial_Write_FIFO(sbuf,slen);		//write fifo	有多少发多少
        slen = 0;
    }
    else
    {
        FM11_Serial_Write_FIFO(sbuf,32);			//write fifo	先发32字节进fifo

        slen -= 32;		//待发长度－32
        sbuf += 32;		//待发数据指针+32
    }

    FM11_Serial_WriteReg(RF_TXEN,0x55);	//写0x55时触发非接触口回发数据
    while(slen>0)
    {
        if((FM11_Serial_ReadReg(FIFO_WORDCNT) & 0x3F )<=8)
        {
            if(slen<=24)
            {
                FM11_Serial_Write_FIFO(sbuf,slen);			//write fifo	先发32字节进fifo
                slen = 0;
            }
            else
            {
                FM11_Serial_Write_FIFO(sbuf,24);			//write fifo	先发32字节进fifo

                slen -= 24; 	//待发长度－24
                sbuf += 24; 	//待发数据指针+24
            }
        }
    }

    //while(FM11_ChkIrqInfo(FIFO_IRQ_EMPTY,FIFO_IRQ)==OFF);
    //FM11_ReadReg(MAIN_IRQ);

    //while((FM11_ReadReg(FIFO_WORDCNT) & 0x3F )> 0);	//等待发送完成标志位置起
    irq_txdone = 0;
}

/*********************************************************************************************************
** 函数名称:	FM11_RF_Rx
** 函数功能:	写FIFO
** 输入参数:    rbuf:读取数据
** 输出参数:    无
** 返回值:      读取的数据长度
*********************************************************************************************************/
#if  0
uint32_t FM11_RF_Rx(uint8_t *rbuf)
{
    uint32_t rlen,temp;
    rlen = 0;
    temp = 0;
    do
    {
        if((FM11_Serial_ReadReg(FIFO_WORDCNT) & 0x3F )>=24)	//查fifo是否到24字节
        {
            FM11_Serial_Read_FIFO(24,&rbuf[rlen]);		//渐满之后读取24字节
            rlen += 24;
            //printf("rlen %02x \n",rlen);
        }
        if( ( (FM11_Serial_ReadReg(FIFO_WORDCNT) & 0x3F ) ==0 )&&(rlen==0) && (irq_rxdone == 0))
        {
            return 0;
        }
    }
    while(irq_rxdone == 0);
    //while((irq_rxdone == 0)&&(FlagErrIrq == OFF));
    irq_rxdone = 0;
    /*
    	if(FlagErrIrq == ON)
    	{
    		FlagErrIrq = OFF;
    		return 0;
    	}
    */

    temp =(uint32_t)( FM11_Serial_ReadReg(FIFO_WORDCNT) & 0x3F);	//接收完全之后，查fifo有多少字节

    FM11_Serial_Read_FIFO(temp,&rbuf[rlen]);		//读最后的数据
    rlen += temp;

#if DEBUG_PRINT ==1
    printf("temp %02x \n",temp);
    printf("rlen %02x \n",rlen);
#endif

    if(rlen <= 2)
        return 0;
    rlen -= 2;	//2字节crc校验
    return rlen;
}
#else
uint32_t FM11_RF_Rx(uint8_t *rbuf)
{
  uint32_t rlen=0,temp=0;
    while(1)
    {
        if( irq_data_wl ==1 )   //
//      if((irq_data_wl ==1)&&(irq_data_in == 1))   //
        {
            FM11_Serial_Read_FIFO(24,&rbuf[rlen]);      //
            rlen += 24;
            irq_data_wl = 0;
        }

    if(irq_rxdone==1)
        {
            break;
        }
    }
    //while((irq_rxdone == 0)&&(FlagErrIrq == OFF));
    irq_rxdone = 0;

    temp =(uint32_t)( FM11_Serial_ReadReg(FIFO_WORDCNT) & 0x3F);   //

    FM11_Serial_Read_FIFO(temp,&rbuf[rlen]);        //
    rlen += temp;

    if(rlen <= 2)           return 0;
    rlen -= 2;  //
    return rlen;
}

#endif

/*********************************************************************************************************
** 函数名称:	FM11_Set_RatsCfg
** 函数功能:	配置卡片rats相关数据
** 输入参数:    rats 参数
** 输出参数:    无
** 返回值:      无
*********************************************************************************************************/
void FM11_Set_RatsCfg(uint8_t rats)
{
    uint8_t temp;

    CID = rats & 0x0F;
    temp = (rats >> 4) & 0x0F;

    if(temp < 5)
        FSDI = 8*(temp+2);
    else if((temp >= 5)&&(temp <= 7))
        FSDI = 32*(temp-3);
    else
        FSDI = 256;

    FSDI -= 2;	//减去2字节EDC
    block_num = 0x01;	//初始为1，调用前，一直为上一帧的块号
}



/*********************************************************************************************************
** 函数名称:    FM11_CS_ON
** 函数功能:    FM11的spi口片选ON
** 输入参数:    无
** 输出参数:    无
** 返回值:      无
*********************************************************************************************************/
void FM11_CS_ON(void)
{
    //__disable_irq();
    BSP_FM11_Sel();        //输出低有效
    Delay_10us(100);
}

/*********************************************************************************************************
** 函数名称:    FM11_CS_OFF
** 函数功能:    FM11的spi口片选OFF
** 输入参数:    无
** 输出参数:    无
** 返回值:      无
*********************************************************************************************************/
void FM11_CS_OFF(void)
{
    //return;
    BSP_FM11_DeSel()   //输出高无效
    //__enable_irq();
}


/**********************************************************************************************************
** 函数名称：FM11读写一个字节(SPI)
*********************************************************************************************************
uint8_t FM11_Single_RW(uint8_t wData)
{
    //发送数据
    while (SPI_I2S_GetFlagStatus(FM11_SPI, SPI_I2S_FLAG_TXE) == RESET);

    SPI_I2S_SendData(FM11_SPI,wData);

    while (SPI_I2S_GetFlagStatus(FM11_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    //接收数据
    return SPI_I2S_ReceiveData(FM11_SPI);
}
*/

/**********************************************************************************************************
** 函数名称：FM11 写多个字节 <=16个字节
********************************************************************************************************
void SPI_WriteByte(uint8_t *pData, uint32_t length)
{

  for(; length>0; length--)
    {
        FM11_SPI_SingleRW(*pData);
        pData++;
    }


}
**/
/**********************************************************************************************************
** 函数名称：FM11读多个字节 <=16个字节
********************************************************************************************************
void  SPI_readByte(uint8_t *pData, uint32_t length)
{

  while(length--)
        {
                *pData = FM11_SPI_SingleRW(0xFF);
          pData++;
        }

}
**/

/*********************************************************************************************************
** 函数名称:    FM11_Write_E2_Enable
** 函数功能:    FM11的spi口E2写使能
** 输入参数:    无
** 输出参数:    无
** 返回值:      无
*********************************************************************************************************/
void FM11_Write_E2_Enable(void)
{
    FM11_CS_ON();
    Delay_10us(CS_DELAY);         //延时确保FM11上电完成
    BSP_SPI_RW_One(0xCE); //使能E2特殊指令，先发送CE
    BSP_SPI_RW_One(0x55); //使能E2特殊指令，再发送55，见文档第30页
    FM11_CS_OFF();
}

/*********************************************************************************************************
** 函数名称:    FM11_Write_E2_Disable
** 函数功能:    FM11的spi口E2写禁止
** 输入参数:    无
** 输出参数:    无
** 返回值:      无
*********************************************************************************************************/
void FM11_Write_E2_Disable(void)
{
    FM11_CS_ON();
    Delay_10us(CS_DELAY);         //延时确保FM11上电完成
    BSP_SPI_RW_One(0xCE); //禁止E2特殊指令
    BSP_SPI_RW_One(0xAA); //禁止E2特殊指令
    FM11_CS_OFF();
}


/*********************************************************************************************************
** 函数名称:    FM11_Serial_WriteReg
** 函数功能:    写FM11寄存器
** 输入参数:    reg:寄存器地址
**                  val:写入的参数
** 输出参数:    无
** 返回值:      无
*********************************************************************************************************/
void FM11_Serial_WriteReg(uint8_t reg,uint8_t val)
{
    uint8_t mode;
    mode = reg & 0x0F;
    FM11_CS_ON();
    Delay_10us(CS_DELAY);
    BSP_SPI_RW_One(mode);    //指令见手册第28页
    BSP_SPI_RW_One(val);
    FM11_CS_OFF();
}


/*********************************************************************************************************
** 函数名称:    FM11_Serial_ReadReg
** 函数功能:    读寄存器值
** 输入参数:    reg:寄存器地址
** 输出参数:    无
** 返回值:      val,读出的寄存器值
*********************************************************************************************************/
uint8_t FM11_Serial_ReadReg(uint8_t reg)
{
    uint8_t mode;
    uint8_t val;

    mode = reg&0x0F; //指令见手册第28页
    mode = mode | 0x20; //指令见手册第28页

    FM11_CS_ON();
    BSP_SPI_RW_One(mode);
    val = BSP_SPI_RW_One(0xFF);
    FM11_CS_OFF();
    return val;
}

/*********************************************************************************************************
** 函数名称:    FM11_Serial_Write_E2Page
** 函数功能:    写E2数据
** 输入参数:    addr:E2地址
**           len:写入的数据长度(<=16)
**           *buf:写入的数据
** 输出参数:    无
** 返回值:     无
*********************************************************************************************************/
uint8_t w_len=0;
uint8_t w_cnt=0;
void FM11_Serial_Write_E2Page(uint16_t addr,uint32_t len,uint8_t *buf)
{
    uint8_t cmd[2];
    cmd[0] = (addr >> 8) & 0x03| 0x40; //此处确认一下，需不需要再次发送写ee的第一个字节的特殊指令
    cmd[1] = addr & 0xFF;
//FM11_Write_E2_Enable();
//  printf("%s: adr = 0x%3x, len = %d, ibuf[0] = 0x%2x\r\n", __func__, adr, len, ibuf[0]);
    FM11_CS_ON();
    Delay_10us(10);
    BSP_SPI_RW_One(cmd[0]);
    BSP_SPI_RW_One(cmd[1]); //E2地址为10bit，这个字节只占8bit，还有两位在addr0
    //w_len=len;
	//w_cnt++;
    BSP_SPI_Write(buf,len);
    FM11_CS_OFF();
	//cmd[1]=cmd[1];
    Delay_10us(2000);          //此处必须要加延时，见手册第30页
 //   FM11_Init(BSP_FM11_SPI_BIT_RATE, BSP_FM11_SPI_CLK);
}

/*********************************************************************************************************
** 函数名称:    FM11_Serial_Write_Eeprom
** 函数功能:    写E2数据
** 输入参数:    addr:E2地址
**           len:写入的数据长度可以大于16字节
**           *wbuf:写入的数据
** 输出参数:    无
** 返回值:     无
*********************************************************************************************************/
int FM11_Serial_Write_Eeprom(uint16_t addr,uint32_t len,uint8_t *wbuf)
{
    uint8_t offset;
   FM11_Write_E2_Enable();

//  printf("%s: adr = 0x%3x, len = %d, wbuf[0] = 0x%2x\r\n", __func__, adr, len, wbuf[0]);
    if(addr < FM11_E2_USER_ADDR || addr >= FM11_E2_MANUF_ADDR)
    {
        return -1;
    }
    if(addr % FM11_E2_BLOCK_SIZE)
    {
        offset = FM11_E2_BLOCK_SIZE - (addr % FM11_E2_BLOCK_SIZE);
        if(len > offset)
        {
            FM11_Serial_Write_E2Page(addr,offset,wbuf);
            addr += offset;
            wbuf += offset;
            len -= offset;
        }
        else
        {
            FM11_Serial_Write_E2Page(addr,len,wbuf);
            len = 0;
        }
    }
    while(len)
    {
        if(len >= FM11_E2_BLOCK_SIZE)
        {
            FM11_Serial_Write_E2Page(addr,FM11_E2_BLOCK_SIZE,wbuf);
            addr += FM11_E2_BLOCK_SIZE;
            wbuf += FM11_E2_BLOCK_SIZE;
            len -= FM11_E2_BLOCK_SIZE;
        }
        else
        {
            FM11_Serial_Write_E2Page(addr,len,wbuf);
            len = 0;
        }
    }
    return 0;
}


/*********************************************************************************************************
** 函数名称:    FM11_Serial_Read_Eeprom
** 函数功能:    读取的E2数据
** 输入参数:    addr:E2地址
**           len:读取的数据长度，可以大于16个字节
**           *rbuf:读取的数据
** 输出参数:    无
** 返回值:     无
*********************************************************************************************************/
void FM11_Serial_Read_Eeprom(uint16_t addr,uint32_t len,uint8_t *rbuf)
{
    uint8_t buf[2];
    buf[0] = ((addr >> 8) & 0x03)| 0x60; //指令见手册第28页
    buf[1] = addr & 0xFF;

    FM11_CS_ON();
    Delay_10us(CS_DELAY);         //延时确保FM11上电完成
    BSP_SPI_RW_One(buf[0]);
    BSP_SPI_RW_One(buf[1]);
    BSP_SPI_Read(rbuf,len);
    FM11_CS_OFF();
}


/*********************************************************************************************************
** 函数名称:    FM11_Serial_Write_FIFO
** 函数功能:    写FIFO
** 输入参数:    wlen:写数据长度(<=32 fifo空间)
**           wbuf:写的数据
** 输出参数:    无
** 返回值:      无
*********************************************************************************************************/
void FM11_Serial_Write_FIFO(uint8_t *wbuf,uint32_t wlen)
{
    FM11_CS_ON();
    BSP_SPI_RW_One(0x80);
    BSP_SPI_Write(wbuf,wlen);
    FM11_CS_OFF();
}

/*********************************************************************************************************
** 函数名称:    FM11_Serial_Read_FIFO
** 函数功能:    发送NAK帧
** 输入参数:    *rlen:待读取的数据长度
** 输出参数:    *rbuf:读取的数据
** 返回值:      读取的数据长度
*********************************************************************************************************/
void FM11_Serial_Read_FIFO(uint32_t rlen,uint8_t *rbuf)
{
    FM11_CS_ON();
    BSP_SPI_RW_One(0xA0);
    BSP_SPI_Read(rbuf,rlen);
    FM11_CS_OFF();
}


/*********************************************************************************************************
** 函数名称:    FM11_Init
** 函数功能:    FM11的spi口初始化程序
** 输入参数:    无
** 输出参数:    无
** 返回值:      无
*********************************************************************************************************/
uint8_t test_WriteBuf[16]= {1,2,3,4,5,6,7,8,};
uint8_t test_ReadBuf[16]= {0};
uint8_t g_reg=0;
void FM11_Init(uint32_t bitRate, uint32_t clkPin)
{
//    uint8_t eebuf[20]= {1};
//    uint8_t reg;
//    uint8_t i;
    BSP_SPI_Open(bitRate,clkPin);

    FM11_CS_OFF();
    FM11_Serial_WriteReg(FIFO_FLUSH,0xFF);     //清fifo寄存器

#ifdef FIFO_TEST
    while(1)
    {
        FM11_Serial_Write_FIFO(test_WriteBuf, 8);
        FM11_Serial_Read_FIFO(8, test_ReadBuf);
        FM11_Serial_ReadReg(NFC_CFG);
    }
#endif

#ifdef REG_TEST
    FM11_Serial_WriteReg(NFC_CFG,0x3);
    g_reg=FM11_Serial_ReadReg(NFC_CFG);

    FM11_Serial_WriteReg(NFC_CFG,0x2);
    g_reg=FM11_Serial_ReadReg(NFC_CFG);
#endif

}


