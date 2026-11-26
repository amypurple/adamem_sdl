/** ADAMEm: Coleco ADAM emulator ********************************************/
/**                                                                        **/
/**                                 F18A.h                                 **/
/**                                                                        **/
/** Line-by-line video chip: a TMS9918A, or the F18A (TMS9918A-compatible **/
/** FPGA VDP by Matthew Hagerty) with all of its extensions:               **/
/** unlock, 64 registers, 64-colour palette, ECM tile and       **/
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

#define F18A_CHIP_TMS9918 0             /* only what a TMS9918A has         */
#define F18A_CHIP_F18A    1

extern int F18A_On;                     /* 1: this module drives the VDP    */
extern int F18A_Chip;                   /* F18A_CHIP_TMS9918 or _F18A       */
extern int F18A_VramTiming;             /* 1: TMS9918A VRAM access slots    */
unsigned F18A_DroppedAccesses (void);   /* bytes lost to fast CPU access    */
#define F18A_FULL (F18A_On && F18A_Chip==F18A_CHIP_F18A)

void F18A_Reset (void);                 /* power-on state                   */
void F18A_WriteData (byte v);           /* port A0 (even) write             */
void F18A_WriteCtrl (byte v);           /* port A1 (odd) write              */
byte F18A_ReadData (void);              /* port A0 read                     */
byte F18A_ReadStatus (void);            /* port A1 read                     */
void F18A_Sync (void);                  /* catch up to the Z80's clock      */
int  F18A_Event (void);                 /* called by the Z80 core when the  */
                                        /* armed event time is reached      */
const word *F18A_Screen (void);         /* last complete frame, RGB565      */
const byte *F18A_IndexScreen (void);    /* TMS9918A: 256x192 colour indices */
void F18A_SetVRAM (byte *p);            /* share the emulator's 16K VRAM    */
byte F18A_GetStatus (void);
word F18A_GetAddr (void);
void F18A_ImportTMS (const int *reg,int status,int addr);
int  F18A_ScreenChanged (void);         /* 1 once per new frame             */
int  F18A_SaveState (FILE *f);          /* 0 on success                     */
int  F18A_LoadState (FILE *f);          /* 0 on success                     */

/* Test hooks */
byte F18A_GetReg (int r);
byte F18A_GetVRAM (int a);
word F18A_GetPalette (int i);
int  F18A_Unlocked (void);

#endif
