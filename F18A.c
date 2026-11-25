/** ADAMEm: Coleco ADAM emulator ********************************************/
/**                                                                        **/
/**                                 F18A.c                                 **/
/**                                                                        **/
/** F18A video chip emulation. Written for ADAMEm from the F18A register   **/
/** documentation; its behaviour was checked against Gearcoleco's F18A     **/
/** core, which was used as a test reference only.                          **/
/**                                                                        **/
/** Timing: a frame is Z80_IPeriod*1000/IFreq T-states (the same as the    **/
/** rest of ADAMEm) split into 262 (60 Hz) or 313 (50 Hz) lines. Line n is **/
/** drawn when the Z80 clock passes the end of line n, so register and     **/
/** VRAM writes made in the middle of the frame show up on the lines that  **/
/** follow them. The Z80 core calls F18A_Event() at the next time an       **/
/** interrupt can rise (the line interrupt or the vertical blank), so      **/
/** the NMI arrives on the right line.                                      **/
/****************************************************************************/

#include <string.h>
#include "F18A.h"

extern int IFreq;                       /* Coleco.c: 50 or 60 Hz            */
extern int Verbose;

int F18A_On=0;

/* Status register 0 bits */
#define ST_F   0x80                     /* vertical blank                   */
#define ST_5S  0x40                     /* fifth sprite                     */
#define ST_C   0x20                     /* sprite collision                 */

/* Line-buffer pixel flags */
#define PX_OPAQUE   1
#define PX_PRIORITY 2

/* Hblank start offset used for status register 1, in 1/228 of a line */
#define HBLANK_PART 37

/* F18A power-on palette (12-bit RGB): 0-15 TMS9918A colours, 16-31 the   */
/* ECM1 set, 32-47 a 16-colour ECM2 set, 48-63 a grey/ECM3 set.           */
static const word DefaultPalette[64]=
{
 0x000,0x000,0x2C3,0x5D6,0x54F,0x76F,0xD54,0x4EF,
 0xF54,0xF76,0xDC3,0xED6,0x2B2,0xC5C,0xCCC,0xFFF,
 0x000,0x2C3,0x000,0x54F,0x000,0xD54,0x000,0x4EF,
 0x000,0xCCC,0x000,0xDC3,0x000,0xC5C,0x000,0xFFF,
 0x000,0x00A,0x0A0,0x0AA,0xA00,0xA0A,0xA50,0xAAA,
 0x555,0x55F,0x5F5,0x5FF,0xF55,0xF5F,0xFF5,0xFFF,
 0x000,0x555,0x000,0x00A,0x000,0x0A0,0x000,0x0AA,
 0x000,0xA00,0x000,0xA0A,0x000,0xA50,0x000,0xFFF
};

static struct
{
 byte VRAM[0x4000];
 byte Reg[64];
 byte Status;                           /* status register 0                */
 word Addr;                             /* VRAM address                     */
 byte ReadBuf;                          /* read-ahead buffer                */
 byte RegRead;                          /* status register 15               */
 int  First;                            /* 1: next control byte is first    */
 int  Unlocked,UnlockStep;
 int  DPM,PalSecond;                    /* palette data-port mode           */
 byte PalIdx,PalHi;
 word Pal[64];                          /* 12-bit colours                   */
 word Pal565[64];
 int  LinePending;                      /* status 1 bit 0                   */
 int  IntOut;                           /* interrupt line level             */
 int  NMIReq;                           /* rising edge not yet delivered    */
 /* timing */
 unsigned T0;                           /* T-state of the start of line 0   */
 int  FC,NL;                            /* frame length, lines per frame    */
 int  Line;                             /* line being displayed             */
 int  H;                                /* active lines of this frame       */
 /* counters (status 4-11) */
 unsigned long long CntNs,CntSnap;
 unsigned CntT;
 unsigned long long CntRem;
 /* frame */
 int  NewFrame;
 /* GPU */
 byte GRAM[0x800];                      /* GPU RAM at 4000H                 */
 word R[16],PC,ST;                      /* registers, program counter, flags*/
 int  Run;                              /* 1 while the GPU executes         */
 byte User;                             /* status register 2 bits 6-0       */
 long long Bal;                         /* GPU clocks owed                  */
 unsigned GpuT;                         /* Z80 T-state the GPU has reached  */
 unsigned long long GpuRem;
 word DmaSrc,DmaDst;
 byte DmaW,DmaH,DmaStride,DmaCtl;
} F;

static word Work[F18A_HEIGHT][F18A_WIDTH];  /* frame being drawn            */
static word Shown[F18A_HEIGHT][F18A_WIDTH]; /* last complete frame          */

static byte TileCol[F18A_WIDTH],TileFlg[F18A_WIDTH];
static byte SprCol[256],SprFlg[256],SprOcc[256];

static byte VR (int a) { return F.VRAM[a&0x3FFF]; }

static void SetPal565 (int i)
{
 int c=F.Pal[i&63],r=(c>>8)&15,g=(c>>4)&15,b=c&15;
 F.Pal565[i&63]=(word)((((r<<1)|(r>>3))<<11)|(((g<<2)|(g>>2))<<5)|((b<<1)|(b>>3)));
}

/****************************************************************************/
/** Registers                                                              **/
/****************************************************************************/
static void ResetRegisters (void)
{
 memset (F.Reg,0,sizeof(F.Reg));
 F.Reg[1]=0x40; F.Reg[3]=0x10; F.Reg[4]=0x01; F.Reg[5]=0x0A;
 F.Reg[6]=0x02; F.Reg[7]=0x1F;
 F.Reg[30]=0x1F;                        /* 32 sprites per line              */
 F.Reg[48]=0x01;                        /* address increment                */
 F.Reg[51]=0x20;                        /* no stop sprite                   */
 F.Reg[54]=0x40;                        /* GPU program address 4000H        */
 F.Unlocked=F.UnlockStep=0;
}

static void ArmEvent (void);

/* Recompute the interrupt line; a rising edge requests an NMI */
static void UpdateInt (void)
{
 int out=((F.Status&ST_F) && (F.Reg[1]&0x20)) ||
         (F.LinePending && (F.Reg[0]&0x10));
 if (out && !F.IntOut) F.NMIReq=1;
 F.IntOut=out;
}

static int SpriteLimit (void)
{
 int n=F.Reg[30]&0x1F;
 return (n==0x1F || n==0)? 32:n;
}

static unsigned long long CyclesToNs (unsigned dt)
{
 /* ns = T * 1e9 / 3579545, keeping the remainder between calls */
 unsigned long long num=(unsigned long long)dt*1000000000ULL+F.CntRem;
 F.CntRem=num%3579545ULL;
 return num/3579545ULL;
}

static void UpdateCounter (void)
{
 unsigned now=Z80_CyclesNow();
 if (F.Reg[15]&0x10) F.CntNs+=CyclesToNs (now-F.CntT);
 F.CntT=now;
}

static void GpuLoad (int run);
static void WriteReg (int r,byte v,int gpu)
{
 switch (r)
 {
  case 0:  F.Reg[0]=v&0x16; break;
  case 1:  F.Reg[1]=v&0x7B; break;
  case 2:  F.Reg[2]=v&0x0F; break;
  case 4:  F.Reg[4]=v&0x07; break;
  case 5:  F.Reg[5]=v&0x7F; break;
  case 6:  F.Reg[6]=v&0x07; break;
  case 10: F.Reg[10]=v&0x0F; break;
  case 15:
   UpdateCounter ();
   if (v&0x20) F.CntSnap=F.CntNs;
   if (v&0x40) { F.CntNs=0; F.CntRem=0; }
   F.Reg[15]=v&0x1F;
   break;
  case 19:
   if (v && v!=F.Reg[19] && v==F.Line) F.LinePending=1;
   F.Reg[19]=v;
   break;
  case 24: F.Reg[24]=v&0x3F; break;
  case 30: F.Reg[30]=v&0x1F; if (!F.Reg[30]) F.Reg[30]=0x1F; break;
  case 47:
   F.Reg[47]=v;
   if (gpu) break;
   F.PalIdx=v&0x3F;
   F.DPM=(v&0x80)!=0;
   F.PalSecond=0;
   break;
  case 49: F.Reg[49]=v&0xFB; break;
  case 50:
   if (v&0x80)
   {
    ResetRegisters ();
    F.DPM=F.PalSecond=0;
   }
   else F.Reg[50]=v&0x7F;
   break;
  case 51: F.Reg[51]=v&0x3F; break;
  case 54: F.Reg[54]=v; break;
  case 55:                              /* load the PC and start            */
   F.Reg[55]=v;
   if (!gpu) GpuLoad (1);
   break;
  case 56:                              /* bit 0: 1 run, 0 stop at VR54-55  */
   if (gpu) break;
   if (v&1) F.Run=1; else GpuLoad (0);
   break;
  default: F.Reg[r]=v; break;
 }
 UpdateInt ();
}

/* VR57 is the unlock register: 0x1C (bits 1-0 ignored) written twice in a */
/* row unlocks, any other value locks. While locked, register numbers      */
/* above 7 fold onto 0-7 as on a TMS9918A, unless 80 columns are selected. */
static void RegisterWrite (int r,byte v)
{
 if (r==57)
 {
  int valid=(v&0xFC)==0x1C;
  F.Unlocked=valid && F.UnlockStep;
  F.UnlockStep=valid;
  return;
 }
 F.UnlockStep=0;
 if (!F.Unlocked && r>7)
 {
  if (F.Reg[0]&0x04) return;
  r&=7;
 }
 WriteReg (r,v,0);
}

static byte RegisterRead (int r)
{
 r&=0x3F;
 return (r==56 || r==57)? 0:F.Reg[r];
}

/****************************************************************************/
/** Scanline rendering                                                     **/
/****************************************************************************/
static int Backdrop (void) { return ((F.Reg[24]&3)<<4)|(F.Reg[7]&15); }

/* One pattern pixel with ecm extra bit planes 'off' bytes apart */
static int PatPixel (int a,int x,int ecm,int off)
{
 int bit=7-(x&7),p=(VR(a)>>bit)&1;
 if (ecm>=2) p|=((VR(a+off)>>bit)&1)<<1;
 if (ecm>=3) p|=((VR(a+off*2)>>bit)&1)<<2;
 return p;
}

static const int PlaneOffset[4]={2048,1024,512,256};

/* Draw tile layer 'l' (0=TL1, 1=TL2) of line y into TileCol/TileFlg.     */
/* gw is the grid width: 256 (32 columns, text40 at x 8-247) or 512        */
/* (text80 at x 16-495); 256-wide pixels are written twice.               */
static void TileLine (int y,int l,int rows,int t40,int t80,int mc,int g2,int ecm)
{
 int ntb=l? F.Reg[10]:F.Reg[2];
 int ctb=l? F.Reg[11]:F.Reg[3];
 int hs =l? F.Reg[25]:F.Reg[27];
 int vs =l? F.Reg[26]:F.Reg[28];
 int hsz=(F.Reg[29]>>(l?5:1))&1;
 int vsz=(F.Reg[29]>>(l?4:0))&1;
 int ps =l? (F.Reg[24]>>2)&3:F.Reg[24]&3;
 int off=PlaneOffset[(F.Reg[29]>>2)&3];
 int posattr=(F.Reg[50]&0x02)!=0;
 int text=t40||t80;
 int cols=t80? 80:(t40? 40:32);
 int x0=t80? 16:(t40? 8:0), xw=text? cols*6:256;
 int sy=y+vs,vtot=rows*(vsz?2:1),trow,vpage,x;
 trow=(sy>>3)%vtot;
 vpage=((ntb>>1)&1)^(trow>=rows);
 if (trow>=rows) trow-=rows;
 for (x=0;x<xw;++x)
 {
  int col,hpage,px,py,name,offs,attr,pa,pix,colour,opaque;
  if (text)
  {
   int sx=(x+hs*(t80?2:1))%(cols*6);
   col=sx/6; px=sx%6; hpage=0;
   offs=trow*cols+col;
   name=VR(((t80 && !F.Unlocked)? (ntb&0x0C):ntb)*1024+offs);
   attr=VR(ctb*64+(posattr? offs:name));
  }
  else
  {
   int sx=x+hs,htot=32*(hsz?2:1);
   col=(sx>>3)%htot; px=sx&7;
   hpage=(ntb&1)^(col>=32); col&=31;
   offs=(vpage<<11)|(hpage<<10)|(trow<<5)|col;
   name=VR(((ntb&0x0C)<<10)|offs);
   attr=VR(ctb*64+(posattr? offs:name));
  }
  py=sy&7;
  if (ecm)
  {
   if (attr&0x40) px=7-px;
   if (attr&0x20) py=7-py;
  }
  if (g2)
  {
   int region=(trow&0x18)<<5,mask=((F.Reg[4]&3)<<8)|0xFF;
   pa=((F.Reg[4]&4)<<11)+(((name+region)&mask)<<3)+py;
  }
  else if (mc)
   pa=(F.Reg[4]<<11)+(name<<3)+((trow&3)<<1)+((sy>>2)&1);
  else
   pa=(F.Reg[4]<<11)+(name<<3)+py;
  if (!ecm)
  {
   int cb;
   if (text)    { cb=posattr? attr:F.Reg[7]; pix=PatPixel (pa,px,0,0); }
   else if (mc) { cb=VR(pa); pix=px<4; }
   else if (g2)
   {
    int region=(trow&0x18)<<5,mask=((F.Reg[3]&0x7F)<<3)|7;
    cb=VR(((F.Reg[3]&0x80)<<6)+(((name+region)&mask)<<3)+py);
    pix=PatPixel (pa,px,0,0);
   }
   else         { cb=VR(ctb*64+(name>>3)); pix=PatPixel (pa,px,0,0); }
   colour=pix? cb>>4:cb&15;
   opaque=colour!=0;
   colour|=ps<<4;
  }
  else
  {
   pix=PatPixel (pa,px,ecm,off);
   opaque=!(attr&0x10) || pix;
   if (ecm==1)      colour=((ps>>1)<<5)|((attr&15)<<1)|pix;
   else if (ecm==2) colour=((attr&15)<<2)|pix;
   else             colour=((attr&14)<<2)|pix;
  }
  if (opaque)
  {
   int flg=PX_OPAQUE;
   if ((ecm && (attr&0x80)) || (l && !(F.Reg[50]&1))) flg|=PX_PRIORITY;
   if (t80)
   { TileCol[x0+x]=(byte)colour; TileFlg[x0+x]=(byte)flg; }
   else
   {
    int o=(x0+x)*2;
    TileCol[o]=TileCol[o+1]=(byte)colour;
    TileFlg[o]=TileFlg[o+1]=(byte)flg;
   }
  }
 }
}

/* Bitmap layer, on the 256-pixel grid */
static void BitmapLine (int y)
{
 int ly=y-F.Reg[34],w,stride,base,fat,trans,prio,x;
 if (!(F.Reg[31]&0x80) || ly<0 || ly>=F.Reg[36]) return;
 w=F.Reg[35]? F.Reg[35]:256;
 stride=(w+3)>>2;
 base=F.Reg[32]<<6;
 fat=(F.Reg[31]&0x10)!=0; trans=(F.Reg[31]&0x20)!=0; prio=(F.Reg[31]&0x40)!=0;
 for (x=0;x<256;++x)
 {
  int lx=x-F.Reg[33],d,pix,c,o;
  if (lx<0 || lx>=w) continue;
  d=VR(base+ly*stride+(lx>>2));
  if (fat) { pix=(d>>(((lx>>1)&1)? 0:4))&15; c=((F.Reg[31]&0x0C)<<2)|pix; }
  else     { pix=(d>>(6-((lx&3)<<1)))&3;    c=((F.Reg[31]&0x0F)<<2)|pix; }
  if (trans && !pix) continue;
  o=x*2;
  if (prio || !(TileFlg[o]&PX_OPAQUE))
  {
   TileCol[o]=TileCol[o+1]=(byte)(c&63);
   TileFlg[o]=TileFlg[o+1]=PX_OPAQUE;
  }
 }
}

/* Sprites of line y into SprCol/SprFlg (256-pixel grid) */
static void SpriteLine (int y,int rows30,int secm)
{
 int sat=(F.Reg[5]&0x7F)<<7,spg=(F.Reg[6]&7)<<11;
 int mag=F.Reg[1]&1,big=(F.Reg[1]&2)!=0;
 int off=PlaneOffset[(F.Reg[29]>>6)&3];
 int limit=SpriteLimit (),stop=F.Reg[51]&0x3F,count=0,s;
 for (s=0;s<32;++s)
 {
  int a=sat+s*4,Y,X,name,tag,s16,src,disp,top,dy,dx;
  if (s==stop) break;
  Y=VR(a);
  if (!rows30 && Y==0xD0) break;
  tag=VR(a+3);
  s16=big || (F.Unlocked && (tag&0x10));
  src=s16? 16:8; disp=src<<mag;
  top=(F.Reg[49]&0x08)? Y:((Y+1)&0xFF);
  dy=(y-top)&0xFF;
  if (dy>=disp) continue;
  if (count==limit)
  {
   if ((F.Reg[50]&0x08) && !(F.Status&ST_5S))
    F.Status=(byte)((F.Status&0xA0)|ST_5S|s);
   break;
  }
  if (++count==5 && !(F.Reg[50]&0x08) && !(F.Status&ST_5S))
   F.Status=(byte)((F.Status&0xA0)|ST_5S|s);
  X=VR(a+1)-((tag&0x80)? 32:0);
  name=VR(a+2); if (s16) name&=0xFC;
  for (dx=0;dx<disp;++dx)
  {
   int gx=X+dx,px,py,pix,c;
   if (gx<0 || gx>255) continue;
   px=dx>>mag; py=dy>>mag;
   if (F.Unlocked)
   {
    if (tag&0x40) px=src-1-px;
    if (tag&0x20) py=src-1-py;
   }
   pix=PatPixel (spg+(name<<3)+py+((px&8)? 16:0),px,secm,off);
   if (!pix) continue;
   if (SprOcc[gx]) F.Status|=ST_C; else SprOcc[gx]=1;
   if (!secm)
   {
    if (!(tag&15)) continue;
    c=(((F.Reg[24]>>4)&3)<<4)|(tag&15);
   }
   else if (secm==1) c=(((F.Reg[24]>>5)&1)<<5)|((tag&15)<<1)|pix;
   else if (secm==2) c=((tag&15)<<2)|pix;
   else              c=((tag&14)<<2)|pix;
   if (!SprFlg[gx]) { SprCol[gx]=(byte)(c&63); SprFlg[gx]=PX_OPAQUE; }
  }
 }
}

/* Draw active line y (0..F.H-1) of the frame */
static void RenderLine (int y)
{
 int m1=F.Reg[1]&0x10,m2=F.Reg[1]&0x08,m3=F.Reg[0]&0x02,m4=F.Reg[0]&0x04;
 int t40=m1 && !m4,t80=m1 && m4,mc=!m1 && m2,g2=!m1 && !m2 && m3;
 int ecm=F.Unlocked? (F.Reg[49]>>4)&3:0;
 int secm=F.Unlocked? F.Reg[49]&3:0;
 int rows30=F.H==240,bd=Backdrop (),x;
 word *out=Work[y+(F18A_HEIGHT-F.H)/2];
 memset (TileCol,bd,sizeof(TileCol));
 memset (TileFlg,0,sizeof(TileFlg));
 memset (SprFlg,0,sizeof(SprFlg));
 memset (SprOcc,0,sizeof(SprOcc));
 if (F.Reg[1]&0x40)
 {
  if (!(F.Reg[50]&0x10)) TileLine (y,0,rows30? 30:24,t40,t80,mc,g2,ecm);
  BitmapLine (y);
  if (F.Unlocked && (F.Reg[49]&0x80)) TileLine (y,1,rows30? 30:24,t40,t80,mc,g2,ecm);
  if (!(t40||t80) || F.Unlocked) SpriteLine (y,rows30,secm);
 }
 for (x=0;x<F18A_WIDTH;++x)
 {
  int c=TileCol[x];
  if (SprFlg[x>>1] && (TileFlg[x]&(PX_OPAQUE|PX_PRIORITY))!=(PX_OPAQUE|PX_PRIORITY))
   c=SprCol[x>>1];
  out[x]=F.Pal565[c&63];
 }
}

/* Fill the border lines of a 192-line frame and publish it */
static void FinishFrame (void)
{
 int y,b=(F18A_HEIGHT-F.H)/2,x;
 word bd=F.Pal565[Backdrop ()];
 for (y=0;y<b;++y)
  for (x=0;x<F18A_WIDTH;++x) Work[y][x]=Work[F18A_HEIGHT-1-y][x]=bd;
 memcpy (Shown,Work,sizeof(Shown));
 F.NewFrame=1;
}

/****************************************************************************/
/** GPU: a TMS9900-style processor running at 100 MHz, with its registers  **/
/** inside the chip (no workspace pointer). F18A changes: CALL, RET, PUSH, **/
/** POP (R15 is the stack pointer), SLC, and PIX in place of XOP; no CRU.  **/
/** Memory map: 0000-3FFF VRAM, 4000-47FF GPU RAM, 5000 palette (words),   **/
/** 6000 VDP registers (write), 7000 scanline, 7001 blanking, 8000 DMA,    **/
/** A000 version, B000 status register 2 bits 6-0 (write).                 **/
/****************************************************************************/
#define GPU_HZ 100000000ULL

#define F_LGT 0x8000
#define F_AGT 0x4000
#define F_EQ  0x2000
#define F_C   0x1000
#define F_OV  0x0800
#define F_OP  0x0400

static int Clk;                         /* clocks of the current instruction*/

static void Dma (void);

static byte GRead (word a)
{
 switch (a>>12)
 {
  case 0: case 1: case 2: case 3: return F.VRAM[a&0x3FFF];
  case 4:  return F.GRAM[a&0x7FF];
  case 5:  { word c=F.Pal[(a>>1)&63]; return (byte)((a&1)? c:(c>>8)); }
  case 6:  return RegisterRead (a&0x3F);
  case 7:  return (byte)((a&1)? (F.Line>=F.H):(F.Line<F.H? F.Line:0));
  case 8:
   switch (a&15)
   {
    case 0: return (byte)(F.DmaSrc>>8);
    case 1: return (byte)F.DmaSrc;
    case 2: return (byte)(F.DmaDst>>8);
    case 3: return (byte)F.DmaDst;
    case 4: return F.DmaW;
    case 5: return F.DmaH;
    case 6: return F.DmaStride;
    case 7: return F.DmaCtl;
   }
   return 0;
  case 10: return 0x19;
 }
 return 0;
}

static void GWrite (word a,byte v)
{
 switch (a>>12)
 {
  case 0: case 1: case 2: case 3: F.VRAM[a&0x3FFF]=v; break;
  case 4: F.GRAM[a&0x7FF]=v; break;
  case 5:
  {
   int i=(a>>1)&63;
   F.Pal[i]=(word)((a&1)? ((F.Pal[i]&0xF00)|v):((F.Pal[i]&0xFF)|((v&15)<<8)));
   SetPal565 (i);
   break;
  }
  case 6: F.UnlockStep=0; WriteReg (a&0x3F,v,1); break;
  case 8:
   switch (a&15)
   {
    case 0: F.DmaSrc=(word)((F.DmaSrc&0xFF)|(v<<8)); break;
    case 1: F.DmaSrc=(word)((F.DmaSrc&0xFF00)|v); break;
    case 2: F.DmaDst=(word)((F.DmaDst&0xFF)|(v<<8)); break;
    case 3: F.DmaDst=(word)((F.DmaDst&0xFF00)|v); break;
    case 4: F.DmaW=v; break;
    case 5: F.DmaH=v; break;
    case 6: F.DmaStride=v; break;
    case 7: F.DmaCtl=v; break;
    case 8: Dma (); break;
   }
   break;
  case 11: F.User=v&0x7F; break;
 }
}

static word GReadW (word a)
{
 a&=0xFFFE;
 Clk+=2;
 if ((a>>12)==5) return F.Pal[(a>>1)&63];
 return (word)((GRead (a)<<8)|GRead ((word)(a+1)));
}

static void GWriteW (word a,word v)
{
 a&=0xFFFE;
 Clk+=2;
 if ((a>>12)==5)
 {
  F.Pal[(a>>1)&63]=v&0xFFF;
  SetPal565 ((a>>1)&63);
  return;
 }
 GWrite (a,(byte)(v>>8));
 GWrite ((word)(a+1),(byte)v);
}

/* DMA: width x height bytes, rows 'stride' apart; bit 1 of the control  */
/* byte counts down, bit 0 fills with the first source byte.            */
static void Dma (void)
{
 int w=F.DmaW? F.DmaW:256,h=F.DmaH? F.DmaH:256,x,y;
 int dir=(F.DmaCtl&2)? -1:1,fill=F.DmaCtl&1;
 word src=F.DmaSrc,dst=F.DmaDst;
 byte v=GRead (src);
 for (y=0;y<h;++y)
 {
  word s=src,d=dst;
  for (x=0;x<w;++x)
  {
   GWrite (d,fill? v:GRead (s));
   s=(word)(s+dir); d=(word)(d+dir);
  }
  src=(word)(src+dir*F.DmaStride);
  dst=(word)(dst+dir*F.DmaStride);
 }
 F.Bal-=(fill? w*h:w*h*2)+4;
}

static word Fetch (void)
{
 word v=GReadW (F.PC);
 F.PC+=2;
 return v;
}

/* Operand: register n (r>=0) or memory address a */
typedef struct { int r; word a; } Op;

static Op Ea (int ts,int n,int b)
{
 Op o;
 o.r=-1; o.a=0;
 switch (ts)
 {
  case 0: o.r=n; o.a=(word)(n<<1); Clk+=1; break;
  case 1: o.a=F.R[n]; break;
  case 2: o.a=Fetch (); if (n) o.a+=F.R[n]; break;
  case 3: o.a=F.R[n]; F.R[n]+=b? 1:2; Clk+=1; break;
 }
 return o;
}

static word Get (Op o,int b)
{
 if (o.r>=0) return b? (word)(F.R[o.r]&0xFF00):F.R[o.r];
 if (b) { Clk+=1; return (word)(GRead (o.a)<<8); }
 return GReadW (o.a);
}

static void Put (Op o,word v,int b)
{
 if (o.r>=0)
 {
  F.R[o.r]=b? (word)((v&0xFF00)|(F.R[o.r]&0xFF)):v;
  Clk+=1;
 }
 else if (b) { GWrite (o.a,(byte)(v>>8)); Clk+=1; }
 else GWriteW (o.a,v);
}

static void Flag (int f,int on) { if (on) F.ST|=f; else F.ST&=~f; }

static void Parity (word v)
{
 int x=v>>8;
 x^=x>>4; x^=x>>2; x^=x>>1;
 Flag (F_OP,x&1);
}

/* L>, A>, EQ from comparing a result with zero */
static void Zero (word v,int b)
{
 if (b)
 {
  int x=v>>8;
  Flag (F_LGT,x!=0); Flag (F_AGT,x && !(x&0x80)); Flag (F_EQ,x==0);
  Parity (v);
 }
 else
 {
  Flag (F_LGT,v!=0); Flag (F_AGT,v && !(v&0x8000)); Flag (F_EQ,v==0);
 }
}

/* C S,D: L> and A> when S is greater than D */
static void Compare (word s,word d,int b)
{
 if (b) { s&=0xFF00; d&=0xFF00; Parity (s); }
 Flag (F_LGT,s>d);
 Flag (F_AGT,(short)s>(short)d);
 Flag (F_EQ,s==d);
}

static word Add (word d,word s)
{
 word r=(word)(d+s);
 Flag (F_C,(unsigned)d+s>0xFFFF);
 Flag (F_OV,(~(d^s)&(d^r)&0x8000)!=0);
 return r;
}

static word Sub (word d,word s)
{
 word r=(word)(d-s);
 Flag (F_C,d>=s);
 Flag (F_OV,((d^s)&(d^r)&0x8000)!=0);
 return r;
}

/* PIX Rs,Rd: Rs = X (MSB) and Y (LSB); Rd holds the options */
static void Pix (word xy,int d)
{
 word opt=F.R[d],a;
 int x=xy>>8,y=xy&0xFF,shift,old;
 byte data;
 if (opt&0x8000)                        /* Graphics II address              */
  a=(word)(((F.Reg[4]&4)<<11)+((y>>3)<<8)+(x&0xF8)+(y&7));
 else                                   /* bitmap layer address             */
 {
  int w=F.Reg[35]? F.Reg[35]:256;
  a=(word)(((F.Reg[32]<<6)+y*((w+3)>>2)+(x>>2))&0x3FFF);
 }
 if (opt&0xC000) { F.R[d]=a; Clk+=1; return; }
 data=F.VRAM[a];
 Clk+=1;
 shift=6-((x&3)<<1);
 old=(data>>shift)&3;
 if (opt&0x0800) F.R[d]=(word)((opt&0xFFFC)|old);
 if (!(opt&0x0400))
 {
  int write=1;
  if (opt&0x0200)
  {
   int eq=old==((opt>>4)&3);
   write=(opt&0x0100)? eq:!eq;
  }
  if (write)
  {
   F.VRAM[a]=(byte)((data&~(3<<shift))|((opt&3)<<shift));
   Clk+=1;
  }
 }
}

static void Exec (word op);

static void Shift (word op,int kind)
{
 int n=(op>>4)&15,r=op&15,i,c=0,ov=0;
 word v=F.R[r];
 if (!n) { n=F.R[0]&15; if (!n) n=16; }
 for (i=0;i<n;++i)
 {
  switch (kind)
  {
   case 0: c=v&1; v=(word)((v>>1)|(v&0x8000)); break;       /* SRA */
   case 1: c=v&1; v>>=1; break;                              /* SRL */
   case 2: c=v>>15; if (((v^(v<<1))&0x8000)) ov=1; v=(word)(v<<1); break; /* SLA */
   case 3: c=v&1; v=(word)((v>>1)|(c<<15)); break;           /* SRC */
   default: c=v>>15; v=(word)((v<<1)|c); break;              /* SLC */
  }
 }
 F.R[r]=v;
 Zero (v,0);
 Flag (F_C,c);
 if (kind==2) Flag (F_OV,ov);
 Clk+=n+3;
}

static void Exec (word op)
{
 if (op>=0x4000)                        /* two operands                     */
 {
  int b=(op>>12)&1;
  Op so=Ea ((op>>4)&3,op&15,b);
  word s=Get (so,b),d,r;
  Op dop=Ea ((op>>10)&3,(op>>6)&15,b);
  switch (op>>13)
  {
   case 2: d=Get (dop,b); r=(word)(d&~s); Zero (r,b); Put (dop,r,b); return; /* SZC */
   case 3: d=Get (dop,b); r=Sub (d,s); Zero (r,b); Put (dop,r,b); return;    /* S   */
   case 4: d=Get (dop,b); Compare (s,d,b); return;                            /* C   */
   case 5: d=Get (dop,b); r=Add (d,s); Zero (r,b); Put (dop,r,b); return;    /* A   */
   case 6: Zero (s,b); Put (dop,s,b); return;                                 /* MOV */
   default: d=Get (dop,b); r=d|s; Zero (r,b); Put (dop,r,b); return;          /* SOC */
  }
 }
 if (op>=0x2000)                        /* source operand, register dest    */
 {
  int d=(op>>6)&15;
  Op so=Ea ((op>>4)&3,op&15,0);
  word s=Get (so,0);
  switch ((op>>10)&7)
  {
   case 0: Flag (F_EQ,(s&~F.R[d])==0); break;                    /* COC */
   case 1: Flag (F_EQ,(s&F.R[d])==0); break;                     /* CZC */
   case 2: F.R[d]^=s; Zero (F.R[d],0); break;                    /* XOR */
   case 3: Pix (s,d); break;                                     /* PIX */
   case 6:                                                       /* MPY */
   {
    unsigned long p=(unsigned long)s*F.R[d];
    F.R[d]=(word)(p>>16); F.R[(d+1)&15]=(word)p;
    Clk+=3;
    break;
   }
   case 7:                                                       /* DIV */
    if (s<=F.R[d]) Flag (F_OV,1);
    else
    {
     unsigned long n=((unsigned long)F.R[d]<<16)|F.R[(d+1)&15];
     F.R[d]=(word)(n/s); F.R[(d+1)&15]=(word)(n%s);
     Flag (F_OV,0);
     Clk+=16;
    }
    break;
  }
  return;
 }
 if (op>=0x1000)                        /* jumps                            */
 {
  int j=0,st=F.ST;
  switch ((op>>8)&15)
  {
   case 0:  j=1; break;                                       /* JMP */
   case 1:  j=!(st&F_AGT) && !(st&F_EQ); break;               /* JLT */
   case 2:  j=!(st&F_LGT) || (st&F_EQ); break;                /* JLE */
   case 3:  j=(st&F_EQ)!=0; break;                            /* JEQ */
   case 4:  j=(st&F_LGT) || (st&F_EQ); break;                 /* JHE */
   case 5:  j=(st&F_AGT)!=0; break;                           /* JGT */
   case 6:  j=!(st&F_EQ); break;                              /* JNE */
   case 7:  j=!(st&F_C); break;                               /* JNC */
   case 8:  j=(st&F_C)!=0; break;                             /* JOC */
   case 9:  j=!(st&F_OV); break;                              /* JNO */
   case 10: j=!(st&F_LGT) && !(st&F_EQ); break;               /* JL  */
   case 11: j=(st&F_LGT) && !(st&F_EQ); break;                /* JH  */
   case 12: j=(st&F_OP)!=0; break;                            /* JOP */
  }
  if (j) F.PC=(word)(F.PC+2*(signed char)(op&0xFF));
  Clk+=1;
  return;
 }
 if (op>=0x0800)                        /* shifts and the F18A additions    */
 {
  switch ((op>>8)&7)
  {
   case 0: case 1: case 2: case 3: Shift (op,(op>>8)&3); return;
   case 4:
    if (op&0x80)                                               /* CALL */
    {
     Op o=Ea ((op>>4)&3,op&15,0);
     GWriteW (F.R[15],F.PC); F.R[15]-=2;
     F.PC=o.a;
    }
    else { F.R[15]+=2; F.PC=GReadW (F.R[15]); }                /* RET  */
    Clk+=2;
    return;
   case 5:                                                     /* PUSH */
   {
    Op o=Ea ((op>>4)&3,op&15,0);
    GWriteW (F.R[15],Get (o,0)); F.R[15]-=2;
    return;
   }
   case 6: Shift (op,4); return;                               /* SLC  */
   default:                                                    /* POP  */
   {
    word v;
    F.R[15]+=2; v=GReadW (F.R[15]);
    Put (Ea ((op>>4)&3,op&15,0),v,0);
    return;
   }
  }
 }
 if (op>=0x0400)                        /* one operand                      */
 {
  Op o=Ea ((op>>4)&3,op&15,0);
  word v,r;
  switch ((op>>6)&15)
  {
   case 1: F.PC=o.a; Clk+=1; return;                              /* B    */
   case 2: Exec (Get (o,0)); return;                              /* X    */
   case 3: Put (o,0,0); return;                                   /* CLR  */
   case 4: v=Get (o,0); r=(word)(0-v); Zero (r,0);                /* NEG  */
           Flag (F_C,v==0); Flag (F_OV,v==0x8000); Put (o,r,0); return;
   case 5: r=(word)~Get (o,0); Zero (r,0); Put (o,r,0); return;   /* INV  */
   case 6: r=Add (Get (o,0),1); Zero (r,0); Put (o,r,0); return;  /* INC  */
   case 7: r=Add (Get (o,0),2); Zero (r,0); Put (o,r,0); return;  /* INCT */
   case 8: r=Sub (Get (o,0),1); Zero (r,0); Put (o,r,0); return;  /* DEC  */
   case 9: r=Sub (Get (o,0),2); Zero (r,0); Put (o,r,0); return;  /* DECT */
   case 10: F.R[11]=F.PC; F.PC=o.a; Clk+=2; return;               /* BL   */
   case 11: v=Get (o,0); Put (o,(word)((v<<8)|(v>>8)),0); return; /* SWPB */
   case 12: Put (o,0xFFFF,0); return;                             /* SETO */
   case 13: v=Get (o,0); Zero (v,0);                              /* ABS  */
            Flag (F_C,0); Flag (F_OV,v==0x8000);
            if (v&0x8000) Put (o,(word)(0-v),0);
            return;
  }
  return;                                                         /* BLWP */
 }
 if (op>=0x0200)                        /* immediates and control           */
 {
  int r=op&15;
  switch (op&0xFFE0)
  {
   case 0x0200: F.R[r]=Fetch (); Zero (F.R[r],0); break;          /* LI   */
   case 0x0220: F.R[r]=Add (F.R[r],Fetch ()); Zero (F.R[r],0); break; /* AI */
   case 0x0240: F.R[r]&=Fetch (); Zero (F.R[r],0); break;         /* ANDI */
   case 0x0260: F.R[r]|=Fetch (); Zero (F.R[r],0); break;         /* ORI  */
   case 0x0280: Compare (F.R[r],Fetch (),0); break;               /* CI   */
   case 0x02A0: F.R[r]=0; break;                                  /* STWP */
   case 0x02C0: F.R[r]=F.ST&0xFC00; break;                        /* STST */
   case 0x02E0: case 0x0300: Fetch (); break;                     /* LWPI, LIMI */
   case 0x0340: F.Run=0; break;                                   /* IDLE */
   case 0x0380: F.PC=F.R[14]; F.ST=F.R[15]&0xFC00; break;         /* RTWP */
  }
  Clk+=2;
 }
}

/* Run the GPU up to Z80 T-state t */
static void GpuRunTo (unsigned t)
{
 unsigned dt=t-F.GpuT;
 unsigned long long den=(unsigned long long)F.FC*(unsigned)(IFreq>0? IFreq:60),num;
 if ((int)dt<=0) return;
 F.GpuT=t;
 num=(unsigned long long)dt*GPU_HZ+F.GpuRem;
 F.GpuRem=num%den;
 if (!F.Run) { F.Bal=0; return; }
 F.Bal+=(long long)(num/den);
 while (F.Run && F.Bal>0)
 {
  Clk=5;
  Exec (Fetch ());
  F.Bal-=Clk;
 }
}

static void GpuLoad (int run)
{
 F.PC=(word)((F.Reg[54]<<8)|F.Reg[55]);
 F.Run=run;
 F.Bal=0;
}

/****************************************************************************/
/** Timing                                                                 **/
/****************************************************************************/
static void StartFrame (void)
{
 F.NL=(IFreq==50)? 313:262;
 F.FC=(int)(((long long)Z80_IPeriod*1000)/(IFreq>0? IFreq:60));
 if (F.FC<F.NL) F.FC=F.NL;
 F.H=(F.Unlocked && (F.Reg[49]&0x40))? 240:192;
 F.Line=0;
}

/* T-state at which line n ends */
static unsigned LineEnd (int n)
{
 return F.T0+(unsigned)(((long long)(n+1)*F.FC)/F.NL);
}

/* End of line n: draw it, raise the flags, move to line n+1 */
static void EndOfLine (void)
{
 int n=F.Line;
 if (n<F.H) RenderLine (n);
 if (n==F.H-1) FinishFrame ();
 if (n==F.H)
 {
  F.Status|=ST_F;
  if (F.Reg[50]&0x20) F.Run=1;          /* GPU vsync trigger                */
 }
 if (F.Reg[50]&0x40) F.Run=1;           /* GPU hsync trigger                */
 if (++F.Line>=F.NL)
 {
  F.T0=LineEnd (F.NL-1);
  StartFrame ();
 }
 if (F.Reg[19] && F.Line==F.Reg[19]) F.LinePending=1;
 UpdateInt ();
}

void F18A_Sync (void)
{
 unsigned now=Z80_CyclesNow();
 while ((int)(now-LineEnd(F.Line))>=0)
 {
  GpuRunTo (LineEnd (F.Line));
  EndOfLine ();
 }
 GpuRunTo (now);
 ArmEvent ();
}

/* Next time something can raise the interrupt line */
static void ArmEvent (void)
{
 int n,target=-1;
 if (F.NMIReq) { Z80_EventAt=Z80_CyclesNow(); Z80_EventOn=1; return; }
 /* vblank: end of line H; line interrupt: end of line VR19-1 */
 for (n=F.Line;n<F.NL;++n)
 {
  if (n==F.H || (F.Reg[19] && n+1==F.Reg[19])) { target=n; break; }
 }
 if (target<0) target=F.NL-1;           /* wrap: re-arm at frame end        */
 Z80_EventAt=LineEnd (target);
 Z80_EventOn=1;
}

int F18A_Event (void)
{
 F18A_Sync ();
 if (F.NMIReq)
 {
  F.NMIReq=0;
  ArmEvent ();
  return Z80_NMI_INT;
 }
 return Z80_IGNORE_INT;
}

/****************************************************************************/
/** Ports                                                                  **/
/****************************************************************************/
static void Advance (void)
{
 F.Addr=(word)((F.Addr+(signed char)F.Reg[48])&0x3FFF);
}

void F18A_WriteData (byte v)
{
 F18A_Sync ();
 F.First=1;
 if (F.DPM)
 {
  if (!F.PalSecond) { F.PalHi=v&15; F.PalSecond=1; return; }
  F.Pal[F.PalIdx&63]=(word)((F.PalHi<<8)|v);
  SetPal565 (F.PalIdx);
  F.PalSecond=0;
  if (F.Reg[47]&0x40)
  {
   if (F.PalIdx==63) F.DPM=0;
   F.PalIdx=(byte)((F.PalIdx+1)&63);
  }
  else F.DPM=0;
  return;
 }
 F.VRAM[F.Addr]=v;
 F.ReadBuf=v;
 Advance ();
}

byte F18A_ReadData (void)
{
 byte r;
 F18A_Sync ();
 F.First=1;
 r=F.ReadBuf;
 F.ReadBuf=F.VRAM[F.Addr];
 Advance ();
 return r;
}

void F18A_WriteCtrl (byte v)
{
 F18A_Sync ();
 if (F.First)
 {
  F.First=0;
  F.Addr=(word)((F.Addr&0x3F00)|v);
  return;
 }
 F.First=1;
 {
  byte lo=(byte)F.Addr;
  F.Addr=(word)(((v&0x3F)<<8)|lo);
  if (v&0x80)
   RegisterWrite (v&0x3F,lo);
  else if (!(v&0x40))
  {
   F.ReadBuf=F.VRAM[F.Addr];
   F.RegRead=RegisterRead (F.Addr>>8);
   Advance ();
  }
 }
 ArmEvent ();
}

static byte StatusReg (int n)
{
 unsigned long long ns=F.CntSnap;
 switch (n&15)
 {
  case 0: return F.Status;
  case 1:
  {
   unsigned now=Z80_CyclesNow(),start=F.Line? LineEnd (F.Line-1):F.T0;
   int part=(int)(((long long)(now-start)*228*F.NL)/F.FC);
   int blank=F.Line>=F.H || part<HBLANK_PART;
   return (byte)(0xE0|(blank? 2:0)|(F.LinePending? 1:0));
  }
  case 2: return (byte)((F.Run? 0x80:0)|F.User);
  case 3: return (byte)(F.Line<F.H? F.Line:0);
  case 4: return (byte)(ns%1000);
  case 5: return (byte)(((ns%1000)>>8)&3);
  case 6: return (byte)((ns/1000)%1000);
  case 7: return (byte)((((ns/1000)%1000)>>8)&3);
  case 8: return (byte)((ns/1000000)%1000);
  case 9: return (byte)((((ns/1000000)%1000)>>8)&3);
  case 10: return (byte)(ns/1000000000ULL);
  case 11: return (byte)((ns/1000000000ULL)>>8);
  case 14: return 0x19;                 /* firmware version 1.9             */
  case 15: return F.RegRead;
 }
 return 0;
}

byte F18A_ReadStatus (void)
{
 int n=F.Reg[15]&15;
 byte r;
 F18A_Sync ();
 F.First=1;
 r=StatusReg (n);
 if (n==0) F.Status&=0x1F;
 else if (n==1) F.LinePending=0;
 F.DPM=F.PalSecond=0;
 UpdateInt ();
 ArmEvent ();
 return r;
}

/****************************************************************************/
/** Reset, frame access and save states                                    **/
/****************************************************************************/
void F18A_Reset (void)
{
 int i;
 memset (&F,0,sizeof(F));
 ResetRegisters ();
 for (i=0;i<64;++i) { F.Pal[i]=DefaultPalette[i]; SetPal565 (i); }
 F.First=1;
 F.T0=Z80_CyclesNow();
 F.CntT=F.GpuT=F.T0;
 F.Reg[54]=0x40;                        /* GPU idle, PC at GPU RAM          */
 GpuLoad (0);
 StartFrame ();
 memset (Work,0,sizeof(Work));
 memset (Shown,0,sizeof(Shown));
 if (F18A_On) ArmEvent ();
}

const word *F18A_Screen (void) { return &Shown[0][0]; }

int F18A_ScreenChanged (void)
{
 int r=F.NewFrame;
 F.NewFrame=0;
 return r;
}

/* The saved block stores the timing as an offset into the frame, since   */
/* the Z80 clock restarts after a load.                                    */
#define F18A_STATE_TAG "F18A0002"

int F18A_SaveState (FILE *f)
{
 unsigned now=Z80_CyclesNow(),t0=F.T0;
 int ok;
 UpdateCounter ();
 F.T0=now-t0;                           /* store elapsed frame time         */
 ok=fwrite (F18A_STATE_TAG,1,8,f)==8 && fwrite (&F,sizeof(F),1,f)==1;
 F.T0=t0;
 return ok? 0:-1;
}

int F18A_LoadState (FILE *f)
{
 char tag[8];
 unsigned now=Z80_CyclesNow();
 if (fread (tag,1,8,f)!=8 || memcmp (tag,F18A_STATE_TAG,8)) return -1;
 if (fread (&F,sizeof(F),1,f)!=1) return -1;
 F.T0=now-F.T0;
 F.CntT=F.GpuT=now;
 F.NMIReq=0;
 if (F18A_On) ArmEvent ();
 return 0;
}

byte F18A_GetReg (int r)    { return F.Reg[r&63]; }
byte F18A_GetVRAM (int a)   { return F.VRAM[a&0x3FFF]; }
word F18A_GetPalette (int i){ return F.Pal[i&63]; }
int  F18A_Unlocked (void)   { return F.Unlocked; }
