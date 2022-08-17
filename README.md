# NUC029xGE Series CMSIS BSP

This BSP folder

## .\Document\


- CMSIS.html<br>
	Introduction of CMSIS version 4.5.0. CMSIS components included CMSIS-CORE, CMSIS-Driver, CMSIS-DSP, etc.

- NuMicro NUC029xGE Series CMSIS BSP Revision History.pdf<br>
	The revision history of NUC029xGE Series BSP.

- NuMicro NUC029xGE Series Driver Reference Guide.chm<br>
	The usage of drivers in NUC029xGE Series BSP.

## .\Library\


- CMSIS<br>
	Cortex® Microcontroller Software Interface Standard (CMSIS) V4.5.0 definitions by ARM® Corp.

- Device<br>
	CMSIS compliant device header file.

- SmartcardLib<br>
	Library for accessing a smartcard.

- StdDriver<br>
	All peripheral driver header and source files.

## .\Sample Code\


- CardReader<br>
	CCID (Circuit card interface device) sample code for smart card interface.

- Hard\_Fault\_Sample<br>
	Show hard fault information when hard fault happened. The hard fault handler show some information included program counter, which is the address where the processor was executing when the hard fault occur. The listing file (or map file) can show what function and instruction that was. It also shows the Link Register (LR), which contains the return address of the last function call. It can show the status where CPU comes from to get to this point.

- ISP<br>
	Sample codes for In-System-Programming.

- NuMaker<br>
	Simple demo for NuMaker board to show message from UART0 to ICE VCOM and show LED.

- Semihost<br>
	Show how to print and get character through IDE console window.

- RegBased<br>
	The sample codes which access control registers directly.

- StdDriver<br>
	Demonstrate the usage of NUC029xGE series MCU peripheral driver APIs.

- Template<br>
	A project template for NUC029xGE series MCU.


# Licesne

**SPDX-License-Identifier: Apache-2.0**

Copyright in some of the content available in this BSP belongs to third parties.
Third parties license is specified in a file header or license file.
NUC029xGE BSP files are provided under the Apache-2.0 license.

