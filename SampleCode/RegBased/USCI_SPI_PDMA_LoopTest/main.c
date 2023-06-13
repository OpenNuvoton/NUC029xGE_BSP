/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate USCI_SPI data transfer with PDMA.
 *           USCI_SPI0 will be configured as Master mode and USCI_SPI1 will be configured as Slave mode.
 *           Both TX PDMA function and RX PDMA function will be enabled.
 *
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC029xGE.h"

#define USPI_MASTER_TX_DMA_CH 0
#define USPI_MASTER_RX_DMA_CH 1
#define USPI_SLAVE_TX_DMA_CH  2
#define USPI_SLAVE_RX_DMA_CH  3

#define TEST_COUNT  64

/* Function prototype declaration */
void SYS_Init(void);
void UART_Init(void);
void USCI_SPI_Init(void);
void UsciSpiLoopTest_WithPDMA(void);

/* Global variable declaration */
static uint16_t s_au16MasterToSlaveTestPattern[TEST_COUNT];
static uint16_t s_au16SlaveToMasterTestPattern[TEST_COUNT];
static uint16_t s_au16MasterRxBuffer[TEST_COUNT];
static uint16_t s_au16SlaveRxBuffer[TEST_COUNT];

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for print message */
    UART_Init();

    /* Init USCI_SPI */
    USCI_SPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------------+\n");
    printf("|                  USCI_SPI + PDMA Sample Code                 |\n");
    printf("+--------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure USCI_SPI0 as a master and USCI_SPI1 as a slave.\n");
    printf("Bit length of a transaction: 16\n");
    printf("The I/O connection for USCI_SPI0/USCI_SPI1 loopback:\n");
    printf("    USCI_SPI0_SS  (PC3) <--> USCI_SPI1_SS  (PD12)\n    USCI_SPI0_CLK (PC4) <--> USCI_SPI1_CLK (PD15)\n");
    printf("    USCI_SPI0_MISO(PC6) <--> USCI_SPI1_MISO(PD13)\n    USCI_SPI0_MOSI(PC5) <--> USCI_SPI1_MOSI(PD14)\n\n");
    printf("Please connect USCI_SPI0 with USCI_SPI1, and press any key to start transmission ...");
    getchar();
    printf("\n");

    UsciSpiLoopTest_WithPDMA();

    printf("\n\nExit USCI_SPI driver sample code.\n");

    /* Disable USCI_SPI0 function mode */
    USPI0->CTL &= ~USPI_CTL_FUNMODE_Msk;
    /* Disable USCI_SPI1 function mode */
    USPI1->CTL &= ~USPI_CTL_FUNMODE_Msk;
    while(1);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable external 12MHz XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Select HXT as the clock source of HCLK */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HXT;

    /* Select HXT as the clock source of UART */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UARTSEL_Msk)) | CLK_CLKSEL1_UARTSEL_HXT;

    /* Enable 48MHz HIRC */
    CLK->PWRCTL |= CLK_PWRCTL_HIRC48EN_Msk;

    /* Waiting for 48MHz clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRC48STB_Msk));

    /* HCLK Clock source from HIRC48 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC48;

    /* Enable UART clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Enable USCI0 peripheral clock */
    CLK->APBCLK1 |= CLK_APBCLK1_USCI0CKEN_Msk;

    /* Enable USCI1 peripheral clock */
    CLK->APBCLK1 |= CLK_APBCLK1_USCI1CKEN_Msk;

    /* Enable PDMA clock source */
    CLK->AHBCLK |= CLK_AHBCLK_PDMACKEN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFPL &= ~(SYS_GPA_MFPL_PA3MFP_Msk | SYS_GPA_MFPL_PA2MFP_Msk);
    SYS->GPA_MFPL |= (SYS_GPA_MFPL_PA3MFP_UART0_RXD | SYS_GPA_MFPL_PA2MFP_UART0_TXD);

    /* Set USCI0_SPI0 multi-function pins */
    SYS->GPC_MFPL = SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC3MFP_Msk | SYS_GPC_MFPL_PC4MFP_Msk | SYS_GPC_MFPL_PC5MFP_Msk | SYS_GPC_MFPL_PC6MFP_Msk);
    SYS->GPC_MFPL = SYS->GPC_MFPL | (SYS_GPC_MFPL_PC3MFP_USCI0_CTL0 | SYS_GPC_MFPL_PC4MFP_USCI0_CLK | SYS_GPC_MFPL_PC5MFP_USCI0_DAT0 | SYS_GPC_MFPL_PC6MFP_USCI0_DAT1);

    /* Set USCI_SPI1 multi-function pins */
    SYS->GPD_MFPH &= ~(SYS_GPD_MFPH_PD12MFP_Msk | SYS_GPD_MFPH_PD13MFP_Msk | SYS_GPD_MFPH_PD14MFP_Msk | SYS_GPD_MFPH_PD15MFP_Msk);
    SYS->GPD_MFPH |= (SYS_GPD_MFPH_PD12MFP_USCI1_CTL0 | SYS_GPD_MFPH_PD13MFP_USCI1_DAT1 | SYS_GPD_MFPH_PD14MFP_USCI1_DAT0 | SYS_GPD_MFPH_PD15MFP_USCI1_CLK);
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void USCI_SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI_SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Select USCI_SPI0 protocol */
    USPI0->CTL &= ~USPI_CTL_FUNMODE_Msk;
    USPI0->CTL = 1 << USPI_CTL_FUNMODE_Pos;
    /* Configure USCI_SPI0 as a master, clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI0->PROTCTL = USPI_MASTER | USPI_MODE_0;
    USPI0->LINECTL = 0;
    /* Set USCI_SPI0 clock rate = f_PCLK1 / 2*(5+1) */
    USPI0->BRGEN = (USPI0->BRGEN & (~USPI_BRGEN_CLKDIV_Msk)) | (5 << USPI_BRGEN_CLKDIV_Pos);
    /* Enable the automatic hardware slave selection function and configure USCI_SPI_SS pin as low-active. */
    USPI0->LINECTL = (USPI0->LINECTL & (~USPI_LINECTL_CTLOINV_Msk)) | USPI_SS_ACTIVE_LOW;
    USPI0->PROTCTL |= USPI_PROTCTL_AUTOSS_Msk;
    /* Enable USCI_SPI0 protocol */
    USPI0->PROTCTL |= USPI_PROTCTL_PROTEN_Msk;

    /* Select USCI_SPI1 protocol */
    USPI1->CTL &= ~USPI_CTL_FUNMODE_Msk;
    USPI1->CTL = 1 << USPI_CTL_FUNMODE_Pos;
    /* Configure USCI_SPI1 as a slave, clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI1->PROTCTL = USPI_SLAVE | USPI_MODE_0;
    USPI1->LINECTL = 0;
    /* Set USCI_SPI1 clock rate = f_PCLK1 */
    USPI1->BRGEN = USPI1->BRGEN & (~USPI_BRGEN_CLKDIV_Msk);
    /* Configure USCI_SPI_SS pin as low-active. */
    USPI1->CTLIN0 = (USPI1->CTLIN0 & (~USPI_CTLIN0_ININV_Msk)) | USPI_CTLIN0_ININV_Msk;
    /* Enable USCI_SPI1 protocol */
    USPI1->PROTCTL |= USPI_PROTCTL_PROTEN_Msk;
}

void UsciSpiLoopTest_WithPDMA(void)
{
    uint16_t u16DataCount, u32TestCycle;
    uint32_t u32RegValue, u32Abort;
    int32_t i32Err;

    USPI_T *UspiMaster = USPI0;
    USPI_T *UspiSlave  = USPI1;


    printf("\nUSCI_SPI0/1 Loop test with PDMA ");

    /* Source data initiation */
    for(u16DataCount = 0; u16DataCount < TEST_COUNT; u16DataCount++)
    {
        s_au16MasterToSlaveTestPattern[u16DataCount] = 0x5500 | (u16DataCount + 1);
        s_au16SlaveToMasterTestPattern[u16DataCount] = 0xAA00 | (u16DataCount + 1);
    }

    /* USCI_SPI master PDMA TX channel configuration */
    PDMA->CHCTL |= (1 << USPI_MASTER_TX_DMA_CH); /* Enable PDMA channel */
    PDMA->DSCT[USPI_MASTER_TX_DMA_CH].CTL =
        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_16 | /* Transfer width 16 bits */
        PDMA_DAR_FIX  | /* Fixed destination address */
        PDMA_SAR_INC  | /* Increment source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_BURST_128   | /* Burst size 128 transfers. No effect in single request type. */
        PDMA_REQ_SINGLE  | /* Single request type */
        PDMA_OP_BASIC;     /* Basic mode */
    PDMA->DSCT[USPI_MASTER_TX_DMA_CH].SA = (uint32_t)s_au16MasterToSlaveTestPattern;
    PDMA->DSCT[USPI_MASTER_TX_DMA_CH].DA = (uint32_t)&UspiMaster->TXDAT;
#if(USPI_MASTER_TX_DMA_CH<=3)
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~(0x3Ful << (8 * (USPI_MASTER_TX_DMA_CH % 4))))) |
                      ((0 + 10) << (8 * (USPI_MASTER_TX_DMA_CH % 4))); /* USPIx_TX */
#else
    PDMA->REQSEL4 = (PDMA->REQSEL4 & (~(0x3Ful << (8 * (USPI_MASTER_TX_DMA_CH % 4))))) |
                    ((0 + 10) << (8 * (USPI_MASTER_TX_DMA_CH % 4))); /* USPIx_TX */
#endif

    /* USCI_SPI master PDMA RX channel configuration */
    PDMA->CHCTL |= (1 << USPI_MASTER_RX_DMA_CH); /* Enable PDMA channel */
    PDMA->DSCT[USPI_MASTER_RX_DMA_CH].CTL =
        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_16 | /* Transfer width 16 bits */
        PDMA_DAR_INC  | /* Increment destination address */
        PDMA_SAR_FIX  | /* Fixed source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_BURST_128  | /* Burst size 128 transfers. No effect in single request type. */
        PDMA_REQ_SINGLE | /* Single request type */
        PDMA_OP_BASIC;    /* Basic mode */
    PDMA->DSCT[USPI_MASTER_RX_DMA_CH].SA = (uint32_t)&UspiMaster->RXDAT;
    PDMA->DSCT[USPI_MASTER_RX_DMA_CH].DA = (uint32_t)s_au16MasterRxBuffer;
#if(USPI_MASTER_RX_DMA_CH<=3)
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~(0x3Ful << (8 * (USPI_MASTER_RX_DMA_CH % 4))))) |
                      ((0 + 11) << (8 * (USPI_MASTER_RX_DMA_CH % 4))); /* USPIx_RX */
#else
    PDMA->REQSEL4 = (PDMA->REQSEL4 & (~(0x3Ful << (8 * (USPI_MASTER_RX_DMA_CH % 4))))) |
                    ((0 + 11) << (8 * (USPI_MASTER_RX_DMA_CH % 4))); /* USPIx_RX */
#endif

    /* USCI_SPI slave PDMA RX channel configuration */
    PDMA->CHCTL |= (1 << USPI_SLAVE_RX_DMA_CH); /* Enable PDMA channel */
    PDMA->DSCT[USPI_SLAVE_RX_DMA_CH].CTL =
        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_16 | /* Transfer width 16 bits */
        PDMA_DAR_INC  | /* Increment destination address */
        PDMA_SAR_FIX  | /* Fixed source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_BURST_128  | /* Burst size 128 transfers. No effect in single request type. */
        PDMA_REQ_SINGLE | /* Single request type */
        PDMA_OP_BASIC;    /* Basic mode */
    PDMA->DSCT[USPI_SLAVE_RX_DMA_CH].SA = (uint32_t)&UspiSlave->RXDAT;
    PDMA->DSCT[USPI_SLAVE_RX_DMA_CH].DA = (uint32_t)s_au16SlaveRxBuffer;
#if(USPI_SLAVE_RX_DMA_CH<=3)
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~(0x3Ful << (8 * (USPI_SLAVE_RX_DMA_CH % 4))))) |
                      ((2 + 11) << (8 * (USPI_SLAVE_RX_DMA_CH % 4))); /* USPIx_RX */
#else
    PDMA->REQSEL4 = (PDMA->REQSEL4 & (~(0x3Ful << (8 * (USPI_SLAVE_RX_DMA_CH % 4))))) |
                    ((2 + 11) << (8 * (USPI_SLAVE_RX_DMA_CH % 4))); /* USPIx_RX */
#endif

    /* USCI_SPI slave PDMA TX channel configuration */
    PDMA->CHCTL |= (1 << USPI_SLAVE_TX_DMA_CH); /* Enable PDMA channel */
    PDMA->DSCT[USPI_SLAVE_TX_DMA_CH].CTL =
        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
        PDMA_WIDTH_16 | /* Transfer width 16 bits */
        PDMA_DAR_FIX  | /* Fixed destination address */
        PDMA_SAR_INC  | /* Increment source address */
        PDMA_DSCT_CTL_TBINTDIS_Msk  | /* Table interrupt disabled */
        PDMA_BURST_128  | /* Burst size 128 transfers. No effect in single request type. */
        PDMA_REQ_SINGLE | /* Single request type */
        PDMA_OP_BASIC;    /* Basic mode */
    PDMA->DSCT[USPI_SLAVE_TX_DMA_CH].SA = (uint32_t)s_au16SlaveToMasterTestPattern;
    PDMA->DSCT[USPI_SLAVE_TX_DMA_CH].DA = (uint32_t)&UspiSlave->TXDAT;
#if(USPI_SLAVE_TX_DMA_CH<=3)
    PDMA->REQSEL0_3 = (PDMA->REQSEL0_3 & (~(0x3Ful << (8 * (USPI_SLAVE_TX_DMA_CH % 4))))) |
                      ((2 + 10) << (8 * (USPI_SLAVE_TX_DMA_CH % 4))); /* USPIx_TX */
#else
    PDMA->REQSEL4 = (PDMA->REQSEL4 & (~(0x3Ful << (8 * (USPI_SLAVE_TX_DMA_CH % 4))))) |
                    ((2 + 10) << (8 * (USPI_SLAVE_TX_DMA_CH % 4))); /* USPIx_TX */
#endif

    /* Enable USCI_SPI slave PDMA function */
    UspiSlave->PDMACTL = (USPI_PDMACTL_TXPDMAEN_Msk | USPI_PDMACTL_RXPDMAEN_Msk | USPI_PDMACTL_PDMAEN_Msk);

    /* Enable USCI_SPI master PDMA function */
    UspiMaster->PDMACTL = (USPI_PDMACTL_TXPDMAEN_Msk | USPI_PDMACTL_RXPDMAEN_Msk | USPI_PDMACTL_PDMAEN_Msk);

    i32Err = 0;
    for(u32TestCycle = 0; u32TestCycle < 10000; u32TestCycle++)
    {
        if((u32TestCycle & 0x1FF) == 0)
            putchar('.');

        while(1)
        {
            /* Get interrupt status */
            u32RegValue = PDMA->INTSTS;
            /* Check the PDMA transfer done interrupt flag */
            if(u32RegValue & PDMA_INTSTS_TDIF_Msk)
            {
                /* Check the PDMA transfer done flags */
                if((PDMA_GET_TD_STS() & ((1 << USPI_MASTER_TX_DMA_CH) | (1 << USPI_MASTER_RX_DMA_CH) | (1 << USPI_SLAVE_TX_DMA_CH) | (1 << USPI_SLAVE_RX_DMA_CH))) ==
                        ((1 << USPI_MASTER_TX_DMA_CH) | (1 << USPI_MASTER_RX_DMA_CH) | (1 << USPI_SLAVE_TX_DMA_CH) | (1 << USPI_SLAVE_RX_DMA_CH)))
                {
                    /* Clear the PDMA transfer done flags */
                    PDMA->TDSTS = ((1 << USPI_MASTER_TX_DMA_CH) | (1 << USPI_MASTER_RX_DMA_CH) | (1 << USPI_SLAVE_TX_DMA_CH) | (1 << USPI_SLAVE_RX_DMA_CH));
                    /* Disable USCI_SPI master's PDMA transfer function */
                    UspiMaster->PDMACTL &= ~(USPI_PDMACTL_TXPDMAEN_Msk | USPI_PDMACTL_RXPDMAEN_Msk);
                    /* Check the transfer data */
                    for(u16DataCount = 0; u16DataCount < TEST_COUNT; u16DataCount++)
                    {
                        if(s_au16MasterToSlaveTestPattern[u16DataCount] != s_au16SlaveRxBuffer[u16DataCount])
                        {
                            i32Err = 1;
                            break;
                        }
                        if(s_au16SlaveToMasterTestPattern[u16DataCount] != s_au16MasterRxBuffer[u16DataCount])
                        {
                            i32Err = 1;
                            break;
                        }
                    }

                    if(u32TestCycle >= 10000)
                        break;

                    /* Source data initiation */
                    for(u16DataCount = 0; u16DataCount < TEST_COUNT; u16DataCount++)
                    {
                        s_au16MasterToSlaveTestPattern[u16DataCount]++;
                        s_au16SlaveToMasterTestPattern[u16DataCount]++;
                    }
                    /* Re-trigger */
                    /* Slave PDMA TX channel configuration */
                    PDMA->CHCTL |= (1 << USPI_SLAVE_TX_DMA_CH); /* Enable PDMA channel */
                    PDMA->DSCT[USPI_SLAVE_TX_DMA_CH].CTL =
                        (PDMA->DSCT[USPI_SLAVE_TX_DMA_CH].CTL & (~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk))) |
                        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
                        1 << PDMA_DSCT_CTL_OPMODE_Pos;
                    /* Slave PDMA RX channel configuration */
                    PDMA->CHCTL |= (1 << USPI_SLAVE_RX_DMA_CH); /* Enable PDMA channel */
                    PDMA->DSCT[USPI_SLAVE_RX_DMA_CH].CTL =
                        (PDMA->DSCT[USPI_SLAVE_RX_DMA_CH].CTL & (~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk))) |
                        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
                        1 << PDMA_DSCT_CTL_OPMODE_Pos;
                    /* Master PDMA TX channel configuration */
                    PDMA->CHCTL |= (1 << USPI_MASTER_TX_DMA_CH); /* Enable PDMA channel */
                    PDMA->DSCT[USPI_MASTER_TX_DMA_CH].CTL =
                        (PDMA->DSCT[USPI_MASTER_TX_DMA_CH].CTL & (~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk))) |
                        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
                        1 << PDMA_DSCT_CTL_OPMODE_Pos;
                    /* Master PDMA RX channel configuration */
                    PDMA->CHCTL |= (1 << USPI_MASTER_RX_DMA_CH); /* Enable PDMA channel */
                    PDMA->DSCT[USPI_MASTER_RX_DMA_CH].CTL =
                        (PDMA->DSCT[USPI_MASTER_RX_DMA_CH].CTL & (~(PDMA_DSCT_CTL_TXCNT_Msk | PDMA_DSCT_CTL_OPMODE_Msk))) |
                        (TEST_COUNT - 1) << PDMA_DSCT_CTL_TXCNT_Pos | /* Transfer count */
                        1 << PDMA_DSCT_CTL_OPMODE_Pos;

                    /* Enable master's PDMA transfer function */
                    UspiMaster->PDMACTL = (USPI_PDMACTL_TXPDMAEN_Msk | USPI_PDMACTL_RXPDMAEN_Msk | USPI_PDMACTL_PDMAEN_Msk);
                    break;
                }
            }
            /* Check the PDMA transfer abort interrupt flag */
            if(u32RegValue & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = PDMA->ABTSTS;
                /* Clear the target abort flag */
                PDMA->ABTSTS = u32Abort;
                i32Err = 1;
                break;
            }
            /* Check the PDMA time-out interrupt flag */
            if(u32RegValue & 0x00000300)
            {
                /* Clear the time-out flag */
                PDMA->INTSTS = u32RegValue & 0x00000300;
                i32Err = 1;
                break;
            }
        }

        if(i32Err)
            break;
    }

    /* Disable all PDMA channels */
    PDMA->CHCTL = 0;

    if(i32Err)
    {
        printf(" [FAIL]\n");
    }
    else
    {
        printf(" [PASS]\n");
    }

    return;
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

