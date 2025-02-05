/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 3 $
 * $Date: 17/05/12 10:13a $
 * @brief    USB CCID smart card reader sample code.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC029xGE.h"
#include "ccid.h"
#include "ccid_if.h"
#include "sclib.h"


#define PLL_CLOCK       72000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define INT_BUFFER_SIZE     64    /* Interrupt message buffer size */
#define BULK_BUFFER_SIZE    512   /* bulk message buffer size */

uint8_t UsbIntMessageBuffer[INT_BUFFER_SIZE];
uint8_t UsbMessageBuffer[BULK_BUFFER_SIZE];

uint8_t volatile gu8IsDeviceReady;
uint8_t volatile gu8AbortRequestFlag;
uint8_t volatile gu8IsBulkOutReady;
uint8_t volatile gu8IsBulkInReady;

uint8_t *pu8IntInBuf;
uint8_t *pUsbMessageBuffer;
uint32_t volatile u32BulkSize;

int32_t volatile gi32UsbdMessageLength;

/*---------------------------------------------------------------------------------------------------------*/
/* The interrupt services routine of smartcard port 0                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void SC01_IRQHandler(void)
{
    /* Please don't remove any of the function calls below */
    if(SCLIB_CheckCDEvent(0))
    {
        RDR_to_PC_NotifySlotChange();
        USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP4)), pu8IntInBuf, 2);
        USBD_SET_PAYLOAD_LEN(EP4, 2);
        return; // Card insert/remove event occurred, no need to check other event...
    }

    SCLIB_CheckTimeOutEvent(0);
    SCLIB_CheckTxRxEvent(0);
    SCLIB_CheckErrorEvent(0);

    return;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PLL to Power-down mode and PLLSTB bit in CLK_STATUS register will be cleared by hardware.*/
    CLK_DisablePLL();

    /* Enable HXT clock (external XTAL 12MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Select module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HXT, CLK_CLKDIV0_UART(1));
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_HXT, CLK_CLKDIV1_SC0(3));
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_PLL, CLK_CLKDIV0_USB(3));

    /* Enable module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(SC0_MODULE);
    CLK_EnableModuleClock(USBD_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA3MFP_UART0_RXD | SYS_GPA_MFPL_PA2MFP_UART0_TXD);

    /* Set PC.0 ~ PC.3 and PB.2 for SC0 interface */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk |
                       SYS_GPC_MFPL_PC1MFP_Msk |
                       SYS_GPC_MFPL_PC2MFP_Msk |
                       SYS_GPC_MFPL_PC3MFP_Msk);
    SYS->GPB_MFPL &= ~SYS_GPB_MFPL_PB2MFP_Msk;
    SYS->GPC_MFPL |= (SC0_DAT_PC0 |
                      SC0_CLK_PC1 |
                      SC0_RST_PC2 |
                      SC0_PWR_PC3);
    SYS->GPB_MFPL |= SC0_nCD_PB2;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------------------------+\n");
    printf("|    NuMicro USB CCID SmartCard Reader Sample Code    |\n");
    printf("+-----------------------------------------------------+\n");

    /* Open smartcard interface 0. CD pin state low indicates card insert and PWR pin low raise VCC pin to card */
    SC_Open(SC0, SC_PIN_STATE_LOW, SC_PIN_STATE_HIGH);
    NVIC_EnableIRQ(SC01_IRQn);

    SC0->INTEN = SC_INTEN_CDIEN_Msk;

    USBD_Open(&gsInfo, CCID_ClassRequest, NULL);

    /* Endpoint configuration */
    CCID_Init();

    /* Set priority is a must under current architecture. Otherwise smartcard interrupt will be blocked by USBD interrupt */
    NVIC_SetPriority(USBD_IRQn, (1 << __NVIC_PRIO_BITS) - 2);

    NVIC_EnableIRQ(USBD_IRQn);
    USBD_Start();

    while(1);
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
