/***************************************************************************//**
 * @file     targetdev.h
 * @brief    ISP support function header file
 * @version  0x34
 *
 * @note
 * Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "NUC029xGE.h"
#include "isp_user.h"
#include "uart_transfer.h"

#define DetectPin   			PA3

/* rename for uart_transfer.c */
#define UART_T					UART0
#define UART_T_IRQHandler		UART02_IRQHandler
#define UART_T_IRQn				UART02_IRQn

#define CONFIG_SIZE 8 // in bytes

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
