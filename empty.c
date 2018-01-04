/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== empty.c ========
 */

/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>

/* Board Header file */
#include "Board.h"
#include ".\fm11nx08\fm11nx08.h"

uint8_t rbuf[35] =
{
    0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,
    0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,
    0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,
    0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,
    0xAA,0xAA,0xAA
};

uint8_t wbuf[35] =
{
    0x06,0x18,0xFF,0xFF,0x09,0x2F,0x3F,0x5D,
    0x16,0x17,0xFF,0x09,0x2F,0x3F,0x33,0x06,
    0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,
    0x18,0x17,0xFF,0xFF,0x09,0x2F,0x3F,0x5D,
    0x09,0x2F,0x3F
};
#define LAUNCHPAD
#if defined(LAUNCHPAD)
#define BSP_LED_R          IOID_6
#define BSP_LED_G          IOID_7
#define LED_ON(n)    GPIO_setDio(n)
#define LED_OFF(n)   GPIO_clearDio(n)
#else
#define BSP_LED_R          IOID_1
#define BSP_LED_B          IOID_0
#define BSP_LED_G          IOID_14
#define LED_ON(n)    GPIO_clearDio(n)
#define LED_OFF(n)   GPIO_setDio(n)
#endif

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on Board_GPIO_BUTTON0.
 */
#if 1
void gpioButtonFxn0(uint_least8_t index)
{
        uint8_t ret = 0, B=0 ;

        ret = FM11_Serial_ReadReg(MAIN_IRQ);
        B = FM11_Serial_ReadReg(MAIN_IRQ_MASK);
        if(ret & MAIN_IRQ_RX_START)  {
            irq_data_in = 1;
        }
        if(ret & MAIN_IRQ_RX_DONE){
            irq_rxdone = 1;
        }
        if(ret & MAIN_IRQ_TX_DONE){
            irq_txdone = 1;
        }
        if(ret & MAIN_IRQ_ACTIVE){
            FlagFirstFrame = ON;
        }
        if(ret & MAIN_IRQ_FIFO){
            FM11_Serial_ReadReg(FIFO_IRQ); //
        }
        if(ret & MAIN_IRQ_AUX){
            FlagErrIrq = ON;
            ret =FM11_Serial_ReadReg(AUX_IRQ);
            if((ret & 0x20) == 0x20){ // parity error
                parity_err = 1;
             }
            if((ret & 0x10) == 0x10) {// crc error
                crc_err = 1;
            }
            if((ret & 0x08) == 0x08){ // frame error
                frame_err = 1;
            }
        }
}
#else
void gpioButtonFxn0(uint_least8_t index)
{
  uint8_t ret = 0;
    uint8_t reg = 0 ;
//  uint8_t aux_irq_status = 0;

    ret = FM11_Serial_ReadReg(MAIN_IRQ);
    if(ret & MAIN_IRQ_RX_START)
    {
        irq_data_wl = 0;
        irq_data_in = 1;
    }

    if(ret & MAIN_IRQ_RX_DONE){
        irq_rxdone = 1;
    }

    if(ret & MAIN_IRQ_TX_DONE){
        irq_txdone = 1;
    }
    if(ret & MAIN_IRQ_RF_PWON)
    {
        FM11_Serial_WriteReg (MAIN_IRQ_MASK,0x04);  //
        FM11_Serial_WriteReg (FIFO_IRQ_MASK,0x03); //
        FlagFirstFrame = ON;
    }
    if(ret & MAIN_IRQ_FIFO)
     {
        reg = FM11_Serial_ReadReg(FIFO_IRQ);    //
        if(reg & FIFO_IRQ_WL)
        {
            irq_data_wl = 1;
        }
//          FM11_Sreial_ReadReg(MAIN_IRQ_MASK);
      }
    if(ret & MAIN_IRQ_AUX)
    {
//        FlagErrIrq = ON;
        FM11_Serial_ReadReg(AUX_IRQ);//
    }
}
#endif


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    uint8_t ret = 0;
    /* 1 second delay */

    /* Call driver init functions */
    GPIO_init();
    // I2C_init();
    // SDSPI_init();
    //SPI_init();
    // UART_init();
    // Watchdog_init();

    /* install Button callback */
    GPIO_setCallback(Board_GPIO_BUTTON0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(Board_GPIO_BUTTON0);

    FM11_Init(BSP_FM11_SPI_BIT_RATE, BSP_FM11_SPI_CLK);
    ret = FM11_Serial_ReadReg(NFC_CFG);

#if 0
    while(1)
    {
        FM11_Serial_Write_Eeprom(0x18, 35, wbuf);
        FM11_Serial_Read_Eeprom(0x18, 35, rbuf);
    }
#endif	

    while(1)
    {
        if(irq_data_in)
        {
            rfLen = FM11_RF_Rx(fm327_fifo);

            if(rfLen > 0)
            {
                //FM11_RF_Tx(rfLen,rfBuf);
                FM11T4T();
            }
            irq_data_in = 0;
        }
    }
	
}



