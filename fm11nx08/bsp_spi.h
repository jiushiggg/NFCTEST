/******************************************************************************

 @file       bsp_spi.h

 @brief Common API for SPI access

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
#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include <ti/devices/cc26x0r2/driverlib/ioc.h>
#include <ti/devices/cc26x0r2/driverlib/gpio.h>
#include <ti/sysbios/knl/Task.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define BLS_SPI_BASE           SSI0_BASE
#define BSP_CPU_FREQ           48000000ul

// Board external flash defines
#define BSP_IOID_FLASH_CS       IOID_27
#define BSP_SPI_MOSI            IOID_24
#define BSP_SPI_MISO            IOID_26
#define BSP_SPI_CLK_FLASH       IOID_25


#define BSP_FM11_SPI_BIT_RATE   4000000
#define BSP_FM11_SPI_BASE       SSI1_BASE
/*Board NFC GPIO defines*/

#define LAUNCHPAD
#ifdef LAUNCHPAD
#define BSP_FM11_SPI_CS         IOID_15
#define BSP_FM11_SPI_MOSI       IOID_9
#define BSP_FM11_SPI_MISO       IOID_8
#define BSP_FM11_SPI_CLK        IOID_10
#define BSP_FM11_CHIP_INT       IOID_25
#else
#define BSP_FM11_SPI_CS         IOID_4
#define BSP_FM11_SPI_MOSI       IOID_5
#define BSP_FM11_SPI_MISO       IOID_16
#define BSP_FM11_SPI_CLK        IOID_6
#define BSP_FM11_CHIP_INT       IOID_13
#endif




#define BSP_FM11_Sel()         {GPIO_clearDio(BSP_FM11_SPI_CS);}
#define BSP_FM11_DeSel()       {GPIO_setDio(BSP_FM11_SPI_CS);}
//#define Delay_10us(val)        {Task_sleep(val);}
//#define Delay_10us(val)        {pt_delay_us(val*10);}



/**
* Initialize SPI interface
*
* @return none
*/
extern void BSP_SPI_Open(uint32_t bitRate, uint32_t clkPin);

/**
* Close SPI interface
*
* @return True when successful.
*/
extern void BSP_SPI_Close(void);

/**
* Clear data from SPI interface
*
* @return none
*/
extern void BSP_SPI_Flush(void);



/**
 *
 * Write one byte and read one byte
 */
extern uint8_t BSP_SPI_RW_One(uint8_t Data);


/**
* Read from an SPI device
*
* @return True when successful.
*/
extern int BSP_SPI_Read(uint8_t *buf, size_t len);

/**
* Write to an SPI device
*
* @return True when successful.
*/
extern int BSP_SPI_Write(const uint8_t *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SPI_H */
