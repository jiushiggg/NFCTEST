/************************************************************
 Shanghai Fudan Microelectronics Group Company Limited

 file name:FM11_demo.c

 version:v0.10

 project:YX1323
*************************************************************/


#include "fm11nx08.h"
#include "bsp_spi.h"




uint8_t FlagFirstFrame = OFF;			//��Ƭ��֡��ʶ
uint32_t FSDI = 64-2;		//-4֡����PCD
uint8_t CID = 0;
uint8_t block_num = 1;
uint8_t irq_data_in = 0;		//�ǽ����ݽ����ն˱�ʶ
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
** ��������:	FM11_RF_Tx
** ��������:	RF���ݻط�
** �������:   len:�ط����ݳ���
** 			buf:�ط�������
** �������:   ��
** ����ֵ:   ��
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
        FM11_Serial_Write_FIFO(sbuf,slen);		//write fifo	�ж��ٷ�����
        slen = 0;
    }
    else
    {
        FM11_Serial_Write_FIFO(sbuf,32);			//write fifo	�ȷ�32�ֽڽ�fifo

        slen -= 32;		//�������ȣ�32
        sbuf += 32;		//��������ָ��+32
    }

    FM11_Serial_WriteReg(RF_TXEN,0x55);	//д0x55ʱ�����ǽӴ��ڻط�����
    while(slen>0)
    {
        if((FM11_Serial_ReadReg(FIFO_WORDCNT) & 0x3F )<=8)
        {
            if(slen<=24)
            {
                FM11_Serial_Write_FIFO(sbuf,slen);			//write fifo	�ȷ�32�ֽڽ�fifo
                slen = 0;
            }
            else
            {
                FM11_Serial_Write_FIFO(sbuf,24);			//write fifo	�ȷ�32�ֽڽ�fifo

                slen -= 24; 	//�������ȣ�24
                sbuf += 24; 	//��������ָ��+24
            }
        }
    }

    //while(FM11_ChkIrqInfo(FIFO_IRQ_EMPTY,FIFO_IRQ)==OFF);
    //FM11_ReadReg(MAIN_IRQ);

    //while((FM11_ReadReg(FIFO_WORDCNT) & 0x3F )> 0);	//�ȴ�������ɱ�־λ����
    irq_txdone = 0;
}

/*********************************************************************************************************
** ��������:	FM11_RF_Rx
** ��������:	дFIFO
** �������:    rbuf:��ȡ����
** �������:    ��
** ����ֵ:      ��ȡ�����ݳ���
*********************************************************************************************************/
#if  0
uint32_t FM11_RF_Rx(uint8_t *rbuf)
{
    uint32_t rlen,temp;
    rlen = 0;
    temp = 0;
    do
    {
        if((FM11_Serial_ReadReg(FIFO_WORDCNT) & 0x3F )>=24)	//��fifo�Ƿ�24�ֽ�
        {
            FM11_Serial_Read_FIFO(24,&rbuf[rlen]);		//����֮���ȡ24�ֽ�
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

    temp =(uint32_t)( FM11_Serial_ReadReg(FIFO_WORDCNT) & 0x3F);	//������ȫ֮�󣬲�fifo�ж����ֽ�

    FM11_Serial_Read_FIFO(temp,&rbuf[rlen]);		//����������
    rlen += temp;

#if DEBUG_PRINT ==1
    printf("temp %02x \n",temp);
    printf("rlen %02x \n",rlen);
#endif

    if(rlen <= 2)
        return 0;
    rlen -= 2;	//2�ֽ�crcУ��
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
** ��������:	FM11_Set_RatsCfg
** ��������:	���ÿ�Ƭrats�������
** �������:    rats ����
** �������:    ��
** ����ֵ:      ��
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

    FSDI -= 2;	//��ȥ2�ֽ�EDC
    block_num = 0x01;	//��ʼΪ1������ǰ��һֱΪ��һ֡�Ŀ��
}



/*********************************************************************************************************
** ��������:    FM11_CS_ON
** ��������:    FM11��spi��ƬѡON
** �������:    ��
** �������:    ��
** ����ֵ:      ��
*********************************************************************************************************/
void FM11_CS_ON(void)
{
    //__disable_irq();
    BSP_FM11_Sel();        //�������Ч
    Delay_10us(100);
}

/*********************************************************************************************************
** ��������:    FM11_CS_OFF
** ��������:    FM11��spi��ƬѡOFF
** �������:    ��
** �������:    ��
** ����ֵ:      ��
*********************************************************************************************************/
void FM11_CS_OFF(void)
{
    //return;
    BSP_FM11_DeSel()   //�������Ч
    //__enable_irq();
}


/**********************************************************************************************************
** �������ƣ�FM11��дһ���ֽ�(SPI)
*********************************************************************************************************
uint8_t FM11_Single_RW(uint8_t wData)
{
    //��������
    while (SPI_I2S_GetFlagStatus(FM11_SPI, SPI_I2S_FLAG_TXE) == RESET);

    SPI_I2S_SendData(FM11_SPI,wData);

    while (SPI_I2S_GetFlagStatus(FM11_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    //��������
    return SPI_I2S_ReceiveData(FM11_SPI);
}
*/

/**********************************************************************************************************
** �������ƣ�FM11 д����ֽ� <=16���ֽ�
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
** �������ƣ�FM11������ֽ� <=16���ֽ�
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
** ��������:    FM11_Write_E2_Enable
** ��������:    FM11��spi��E2дʹ��
** �������:    ��
** �������:    ��
** ����ֵ:      ��
*********************************************************************************************************/
void FM11_Write_E2_Enable(void)
{
    FM11_CS_ON();
    Delay_10us(CS_DELAY);         //��ʱȷ��FM11�ϵ����
    BSP_SPI_RW_One(0xCE); //ʹ��E2����ָ��ȷ���CE
    BSP_SPI_RW_One(0x55); //ʹ��E2����ָ��ٷ���55�����ĵ���30ҳ
    FM11_CS_OFF();
}

/*********************************************************************************************************
** ��������:    FM11_Write_E2_Disable
** ��������:    FM11��spi��E2д��ֹ
** �������:    ��
** �������:    ��
** ����ֵ:      ��
*********************************************************************************************************/
void FM11_Write_E2_Disable(void)
{
    FM11_CS_ON();
    Delay_10us(CS_DELAY);         //��ʱȷ��FM11�ϵ����
    BSP_SPI_RW_One(0xCE); //��ֹE2����ָ��
    BSP_SPI_RW_One(0xAA); //��ֹE2����ָ��
    FM11_CS_OFF();
}


/*********************************************************************************************************
** ��������:    FM11_Serial_WriteReg
** ��������:    дFM11�Ĵ���
** �������:    reg:�Ĵ�����ַ
**                  val:д��Ĳ���
** �������:    ��
** ����ֵ:      ��
*********************************************************************************************************/
void FM11_Serial_WriteReg(uint8_t reg,uint8_t val)
{
    uint8_t mode;
    mode = reg & 0x0F;
    FM11_CS_ON();
    Delay_10us(CS_DELAY);
    BSP_SPI_RW_One(mode);    //ָ����ֲ��28ҳ
    BSP_SPI_RW_One(val);
    FM11_CS_OFF();
}


/*********************************************************************************************************
** ��������:    FM11_Serial_ReadReg
** ��������:    ���Ĵ���ֵ
** �������:    reg:�Ĵ�����ַ
** �������:    ��
** ����ֵ:      val,�����ļĴ���ֵ
*********************************************************************************************************/
uint8_t FM11_Serial_ReadReg(uint8_t reg)
{
    uint8_t mode;
    uint8_t val;

    mode = reg&0x0F; //ָ����ֲ��28ҳ
    mode = mode | 0x20; //ָ����ֲ��28ҳ

    FM11_CS_ON();
    BSP_SPI_RW_One(mode);
    val = BSP_SPI_RW_One(0xFF);
    FM11_CS_OFF();
    return val;
}

/*********************************************************************************************************
** ��������:    FM11_Serial_Write_E2Page
** ��������:    дE2����
** �������:    addr:E2��ַ
**           len:д������ݳ���(<=16)
**           *buf:д�������
** �������:    ��
** ����ֵ:     ��
*********************************************************************************************************/
uint8_t w_len=0;
uint8_t w_cnt=0;
void FM11_Serial_Write_E2Page(uint16_t addr,uint32_t len,uint8_t *buf)
{
    uint8_t cmd[2];
    cmd[0] = (addr >> 8) & 0x03| 0x40; //�˴�ȷ��һ�£��費��Ҫ�ٴη���дee�ĵ�һ���ֽڵ�����ָ��
    cmd[1] = addr & 0xFF;
//FM11_Write_E2_Enable();
//  printf("%s: adr = 0x%3x, len = %d, ibuf[0] = 0x%2x\r\n", __func__, adr, len, ibuf[0]);
    FM11_CS_ON();
    Delay_10us(10);
    BSP_SPI_RW_One(cmd[0]);
    BSP_SPI_RW_One(cmd[1]); //E2��ַΪ10bit������ֽ�ֻռ8bit��������λ��addr0
    //w_len=len;
	//w_cnt++;
    BSP_SPI_Write(buf,len);
    FM11_CS_OFF();
	//cmd[1]=cmd[1];
    Delay_10us(2000);          //�˴�����Ҫ����ʱ�����ֲ��30ҳ
 //   FM11_Init(BSP_FM11_SPI_BIT_RATE, BSP_FM11_SPI_CLK);
}

/*********************************************************************************************************
** ��������:    FM11_Serial_Write_Eeprom
** ��������:    дE2����
** �������:    addr:E2��ַ
**           len:д������ݳ��ȿ��Դ���16�ֽ�
**           *wbuf:д�������
** �������:    ��
** ����ֵ:     ��
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
** ��������:    FM11_Serial_Read_Eeprom
** ��������:    ��ȡ��E2����
** �������:    addr:E2��ַ
**           len:��ȡ�����ݳ��ȣ����Դ���16���ֽ�
**           *rbuf:��ȡ������
** �������:    ��
** ����ֵ:     ��
*********************************************************************************************************/
void FM11_Serial_Read_Eeprom(uint16_t addr,uint32_t len,uint8_t *rbuf)
{
    uint8_t buf[2];
    buf[0] = ((addr >> 8) & 0x03)| 0x60; //ָ����ֲ��28ҳ
    buf[1] = addr & 0xFF;

    FM11_CS_ON();
    Delay_10us(CS_DELAY);         //��ʱȷ��FM11�ϵ����
    BSP_SPI_RW_One(buf[0]);
    BSP_SPI_RW_One(buf[1]);
    BSP_SPI_Read(rbuf,len);
    FM11_CS_OFF();
}


/*********************************************************************************************************
** ��������:    FM11_Serial_Write_FIFO
** ��������:    дFIFO
** �������:    wlen:д���ݳ���(<=32 fifo�ռ�)
**           wbuf:д������
** �������:    ��
** ����ֵ:      ��
*********************************************************************************************************/
void FM11_Serial_Write_FIFO(uint8_t *wbuf,uint32_t wlen)
{
    FM11_CS_ON();
    BSP_SPI_RW_One(0x80);
    BSP_SPI_Write(wbuf,wlen);
    FM11_CS_OFF();
}

/*********************************************************************************************************
** ��������:    FM11_Serial_Read_FIFO
** ��������:    ����NAK֡
** �������:    *rlen:����ȡ�����ݳ���
** �������:    *rbuf:��ȡ������
** ����ֵ:      ��ȡ�����ݳ���
*********************************************************************************************************/
void FM11_Serial_Read_FIFO(uint32_t rlen,uint8_t *rbuf)
{
    FM11_CS_ON();
    BSP_SPI_RW_One(0xA0);
    BSP_SPI_Read(rbuf,rlen);
    FM11_CS_OFF();
}


/*********************************************************************************************************
** ��������:    FM11_Init
** ��������:    FM11��spi�ڳ�ʼ������
** �������:    ��
** �������:    ��
** ����ֵ:      ��
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
    FM11_Serial_WriteReg(FIFO_FLUSH,0xFF);     //��fifo�Ĵ���

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


