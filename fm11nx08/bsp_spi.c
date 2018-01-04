/******************************************************************************

 @file       bsp_spi.c

 @brief Common API for SPI access. Driverlib implementation.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2014-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_1_40_00_45
 Release Date: 2017-07-20 17:16:59
 *****************************************************************************/
//#include <driverlib/ssi.h>
//#include <driverlib/gpio.h>
//#include <driverlib/prcm.h>
//#include <driverlib/ioc.h>
//#include <driverlib/rom.h>
//#include <driverlib/ssi.h>
//#include "bsp.h"
#include "bsp_spi.h"

#include <ti/devices/cc26x0r2/driverlib/ssi.h>
#include <ti/devices/cc26x0r2/driverlib/gpio.h>
#include <ti/devices/cc26x0r2/driverlib/prcm.h>
#include <ti/devices/cc26x0r2/driverlib/ioc.h>
#include <ti/devices/cc26x0r2/driverlib/rom.h>

/* Board specific settings for CC26xx SensorTag, PCB version 1.01
 *
 * Note that since this module is an experimental implementation,
 * board specific settings are directly hard coded here.
 */




/**
 *
 * Write one byte and read one byte
 */
uint8_t BSP_SPI_RW_One(uint8_t Data)
{
    uint32_t ul,buf;
    SSIDataPut(BSP_FM11_SPI_BASE,Data);
	//    
	// Wait until the transmit FIFO is empty.   
	//
    while (SSIDataGetNonBlocking(BSP_FM11_SPI_BASE, &buf));

    while(!HWREG(BSP_FM11_SPI_BASE + SSI_O_SR) & SSI_SR_TFE)
     {
     }
     //
     // Wait until the interface is not busy.
     //
     while(HWREG(BSP_FM11_SPI_BASE + SSI_O_SR) & SSI_SR_BSY)
     {
     }

    SSIDataGet(BSP_FM11_SPI_BASE, &ul);

    return ul&0xFF;
}






/**
 * Write a command to SPI
 */
int BSP_SPI_Write(const uint8_t *buf, size_t len)
{
    while (len > 0)
    {
        uint32_t ul;
        SSIDataPut(BSP_FM11_SPI_BASE, *buf);
        SSIDataGet(BSP_FM11_SPI_BASE, &ul);
        len--;
        buf++;
    }
    return 0;
}

/**
 * Read from SPI
 */
int BSP_SPI_Read(uint8_t *buf, size_t len)
{
    while (len > 0)
    {
        uint32_t ul;
        if (!SSIDataPutNonBlocking(BSP_FM11_SPI_BASE, 0))
        {
            /* Error */
            return -1;
        }
        SSIDataGet(BSP_FM11_SPI_BASE, &ul);
        *buf = (uint8_t) ul;
        len--;
        buf++;
    }
    return 0;
}


/* See bsp_spi.h file for description */
void BSP_SPI_Flush(void)
{
  uint32_t ul;

  while (SSIDataGetNonBlocking(BSP_FM11_SPI_BASE, &ul));
}


/* See bsp_spi.h file for description */
void BSP_SPI_Open(uint32_t bitRate, uint32_t clkPin)
{
  /* GPIO power && SPI power domain */
  PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH | PRCM_DOMAIN_SERIAL);
  while (PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH | PRCM_DOMAIN_SERIAL)
         != PRCM_DOMAIN_POWER_ON);

  /* GPIO power */
  PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
  PRCMLoadSet();
  while (!PRCMLoadGet());

  /* SPI power */
  PRCMPeripheralRunEnable(PRCM_PERIPH_SSI1);
  PRCMLoadSet();
  while (!PRCMLoadGet());

  /* SPI configuration */
  SSIIntDisable(BSP_FM11_SPI_BASE, SSI_RXOR | SSI_RXFF | SSI_RXTO | SSI_TXFF);
  SSIIntClear(BSP_FM11_SPI_BASE, SSI_RXOR | SSI_RXTO);
  SSIConfigSetExpClk(BSP_FM11_SPI_BASE,
                         BSP_CPU_FREQ, /* CPU rate */
                         SSI_FRF_MOTO_MODE_1, /* frame format */
                         SSI_MODE_MASTER, /* mode */
                         bitRate, /* bit rate */
                         8); /* data size */
  IOCPinTypeSsiMaster(BSP_FM11_SPI_BASE, BSP_FM11_SPI_MISO, BSP_FM11_SPI_MOSI,
                          IOID_UNUSED /* csn */, clkPin);
  SSIEnable(BSP_FM11_SPI_BASE);

  {
    /* Get read of residual data from SSI port */
    uint32_t buf;

    while (SSIDataGetNonBlocking(BSP_FM11_SPI_BASE, &buf));
  }

}

/* See bsp_spi.h file for description */
void BSP_SPI_Close(void)
{
  // Power down SSI1
  PRCMPeripheralRunDisable(PRCM_PERIPH_SSI1);
  PRCMLoadSet();
  while (!PRCMLoadGet());

  PRCMPeripheralRunDisable(PRCM_PERIPH_GPIO);
  PRCMLoadSet();
  while (!PRCMLoadGet());

  PRCMPowerDomainOff(PRCM_DOMAIN_SERIAL | PRCM_DOMAIN_PERIPH);
  while (PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL | PRCM_DOMAIN_PERIPH)
         != PRCM_DOMAIN_POWER_OFF);

}



