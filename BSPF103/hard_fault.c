/************************************************************************** 
 * 模块名称 : 
 * FilePath: \2.3.0c:\Users\Administrator\Desktop\MICRO\FAtfs\USBCAN\SETUP 4 USB+FILESYSTEM+CAN+FILEWRITE\BSPF103\hard_fault.c
 * Date: 2021-10-28 18:46:22
 * LastEditTime: 2021-10-29 01:14:10
 * 说     明: 
 **************************************************************************/
#include "include.h"

#define SCB_CFSR_MMFSR_IACCVIOL (0x01 << 0)
#define SCB_CFSR_MMFSR_DACCVIOL (0x01 << 1)
#define SCB_CFSR_MMFSR_MUNSTKERR (0x01 << 3)
#define SCB_CFSR_MMFSR_MSTKERR (0x01 << 4)
#define SCB_CFSR_MMFSR_MMARVALID (0x01 << 7)

#define SCB_CFSR_BFSR_IBUSERR (0x01 << (0 + 8))
#define SCB_CFSR_BFSR_PRECISERR (0x01 << (0 + 8))
#define SCB_CFSR_BFSR_IMPRECISERR (0x01 << (2 + 8))
#define SCB_CFSR_BFSR_UNSTKERR (0x01 << (3 + 8))
#define SCB_CFSR_BFSR_STKERR (0x01 << (4 + 8))
#define SCB_CFSR_BFSR_BFARVALID (0x01 << (7 + 8))

#define SCB_CFSR_UFSR_UNDEFINSTR (0x01 << (0 + 16))
#define SCB_CFSR_UFSR_INVSTATE (0x01 << (1 + 16))
#define SCB_CFSR_UFSR_INVPC (0x01 << (2 + 16))
#define SCB_CFSR_UFSR_NOCP (0x01 << (3 + 16))
#define SCB_CFSR_UFSR_UNALIGNED (0x01 << (8 + 16))
#define SCB_CFSR_UFSR_DIVBYZERO (0x01 << (9 + 16))

void FAULT_PrintFaultRegs(void)
{
  DBG_SendPolling("MMFSR:0xx[%s%s%s%s%s]\r\n",
                  SCB->CFSR & SCB_CFSR_MEMFAULTSR_Msk,
                  (0 == (SCB_CFSR_MMFSR_IACCVIOL & SCB->CFSR)) ? "" : "IACCVIOL|",
                  (0 == (SCB_CFSR_MMFSR_DACCVIOL & SCB->CFSR)) ? "" : "DACCVIOL|",
                  (0 == (SCB_CFSR_MMFSR_MUNSTKERR & SCB->CFSR)) ? "" : "MUNSTKERR|",
                  (0 == (SCB_CFSR_MMFSR_MSTKERR & SCB->CFSR)) ? "" : "MSTKERR|",
                  (0 == (SCB_CFSR_MMFSR_MMARVALID & SCB->CFSR)) ? "" : "MMARVALID|");

  DBG_SendPolling("BFSR:0xx[%s%s%s%s%s%s]\r\n",
                  SCB->CFSR & SCB_CFSR_BUSFAULTSR_Msk,
                  (0 == (SCB_CFSR_BFSR_IBUSERR & SCB->CFSR)) ? "" : "IBUSERR|",
                  (0 == (SCB_CFSR_BFSR_PRECISERR & SCB->CFSR)) ? "" : "PRECISERR|",
                  (0 == (SCB_CFSR_BFSR_IMPRECISERR & SCB->CFSR)) ? "" : "IMPRECISERR|",
                  (0 == (SCB_CFSR_BFSR_UNSTKERR & SCB->CFSR)) ? "" : "UNSTKERR|",
                  (0 == (SCB_CFSR_BFSR_STKERR & SCB->CFSR)) ? "" : "STKERR|",
                  (0 == (SCB_CFSR_BFSR_BFARVALID & SCB->CFSR)) ? "" : "BFARVALID|");

  DBG_SendPolling("UFSR:0xx[%s%s%s%s%s%s]\r\n",
                  SCB->CFSR & SCB_CFSR_USGFAULTSR_Msk,
                  (0 == (SCB_CFSR_UFSR_UNDEFINSTR & SCB->CFSR)) ? "" : "UNDEFINSTR",
                  (0 == (SCB_CFSR_UFSR_INVSTATE & SCB->CFSR)) ? "" : "INVSTATE|",
                  (0 == (SCB_CFSR_UFSR_INVPC & SCB->CFSR)) ? "" : "INVPC|",
                  (0 == (SCB_CFSR_UFSR_NOCP & SCB->CFSR)) ? "" : "NOCP|",
                  (0 == (SCB_CFSR_UFSR_UNALIGNED & SCB->CFSR)) ? "" : "UNALIGNED|",
                  (0 == (SCB_CFSR_UFSR_DIVBYZERO & SCB->CFSR)) ? "" : "DIVBYZERO|");

  DBG_SendPolling("HFSR:0xx[%s%s%s]\r\n",
                  SCB->HFSR,
                  (0 == (SCB_HFSR_DEBUGEVT_Msk & SCB->HFSR)) ? "" : "DEBUGEVT|",
                  (0 == (SCB_HFSR_FORCED_Msk & SCB->HFSR)) ? "" : "FORCED|",
                  (0 == (SCB_HFSR_VECTTBL_Msk & SCB->HFSR)) ? "" : "VECTTBL|");
  DBG_SendPolling("DFSR:0xx\r\n", SCB->DFSR);
  DBG_SendPolling("MMFAR:Oxx\r\n", SCB->MMFAR);
  DBG_SendPolling("BFAR:0xx\r\n", SCB->BFAR);

  DBG_SendPolling("\r\n");
  return;
}

void FAULT_PrintGeneralRegs(uint32_t *stackaddr)
{
  DBG_SendPolling("\r\nExceptions:\r\n");

  DBG_SendPolling("r0:0xx\r\n",stackaddr[14]);
  DBG_SendPolling("r1:0xx\r\n",stackaddr[15]);
  DBG_SendPolling("r2:0xx\r\n",stackaddr[16]);
  DBG_SendPolling("r3:0xx\r\n",stackaddr[17]);

  DBG_SendPolling("r4:0xx\r\n",stackaddr[6]);
  DBG_SendPolling("r5:0xx\r\n",stackaddr[7]);
  DBG_SendPolling("r6:0xx\r\n",stackaddr[8]);
  DBG_SendPolling("r7:0xx\r\n",stackaddr[9]);
  DBG_SendPolling("r8:0xx\r\n",stackaddr[10]);
  DBG_SendPolling("r9:0xx\r\n",stackaddr[11]);
  DBG_SendPolling("r10:0xx\r\n",stackaddr[12]);
  DBG_SendPolling("r11:0xx\r\n",stackaddr[13]);

  DBG_SendPolling("r12:0xx\r\n",stackaddr[18]);
  DBG_SendPolling("lr:0xx\r\n",stackaddr[19]);
  DBG_SendPolling("pc:0xx\r\n",stackaddr[20]);
  DBG_SendPolling("xpsr:0xx\r\n",stackaddr[21]);

  DBG_SendPolling("current xpsr:0xx\r\n",stackaddr[5]);
  DBG_SendPolling("current lr:0xx\r\n",stackaddr[4]);
  DBG_SendPolling("primask:0xx\r\n",stackaddr[3]);
  DBG_SendPolling("basepri:0xx\r\n",stackaddr[2]);
  DBG_SendPolling("faultmask:0xx\r\n",stackaddr[1]);
  DBG_SendPolling("control:0xx\r\n",stackaddr[0]);

  DBG_SendPolling("\r\n");

  return;
}

__ASM void HardFault_Handler(void)
{
  IMPORT FAULT_PrintGeneralRegs
  IMPORT FAULT_PrintFaultRegs

  PRESERVE8

  push {r4,r5,r6,r7,r8,r9,r10,r11}

  mrs r0,xpsr
  push {r0}
  mov r0,lr
  push {r0}
  mrs r0,primask
  push {r0}
  mrs r0,basepri
  push {r0}
  mrs r0,faultmask
  push {r0}
  mrs r0,control
  push{r0}

  mov r0,sp
  ldr r1,=FAULT_PrintGeneralRegs
  blx r1
  ldr r1,=FAULT_PrintFaultRegs 
  blx r1

  b.
  nop
}