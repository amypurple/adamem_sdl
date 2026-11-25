/** ADAMEm: Coleco ADAM emulator ********************************************/
/**                                                                        **/
/**                                 F18A.h                                 **/
/**                                                                        **/
/** F18A video chip (TMS9918A-compatible FPGA VDP by Matthew Hagerty)      **/
/** emulation: unlock, 64 registers, 64-colour palette, ECM tile and       **/
/** sprite modes, two tile layers, bitmap layer, scrolling, 30 rows,       **/
/** 80 columns, line interrupt, the status registers, and the GPU with    **/
/** PIX and DMA. Drawn one scanline at a time in step with the Z80.         **/
/****************************************************************************/
#ifndef F18A_H
#define F18A_H

#include <stdio.h>
#include "Z80.h"

#define F18A_WIDTH  512                 /* output frame: 512x240, RGB565    */
#define F18A_HEIGHT 240

extern int F18A_On;                     /* 1 if -video f18a                 */

void F18A_Reset (void);                 /* power-on state                   */
void F18A_WriteData (byte v);           /* port A0 (even) write             */
void F18A_WriteCtrl (byte v);           /* port A1 (odd) write              */
byte F18A_ReadData (void);              /* port A0 read                     */
byte F18A_ReadStatus (void);            /* port A1 read                     */
void F18A_Sync (void);                  /* catch up to the Z80's clock      */
int  F18A_Event (void);                 /* called by the Z80 core when the  */
                                        /* armed event time is reached      */
const word *F18A_Screen (void);         /* last complete frame, RGB565      */
int  F18A_ScreenChanged (void);         /* 1 once per new frame             */
int  F18A_SaveState (FILE *f);          /* 0 on success                     */
int  F18A_LoadState (FILE *f);          /* 0 on success                     */

/* Test hooks */
byte F18A_GetReg (int r);
byte F18A_GetVRAM (int a);
word F18A_GetPalette (int i);
int  F18A_Unlocked (void);

#endif
