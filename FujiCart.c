/****************************************************************************/
/**                                                                        **/
/**                               FujiCart.c                               **/
/**                                                                        **/
/** FujiNet ColecoVision cartridge emulation.                              **/
/**                                                                        **/
/** The real cartridge (an RP2040 serving the 32K window, plus an ESP32-S3 **/
/** FujiNet) sees only address lines and chip selects: the console talks   **/
/** by *reading* hotspot addresses, and the cartridge answers by repainting**/
/** the ROM it serves. Layout (console addresses):                         **/
/**                                                                        **/
/**   F800-FBFF  reply window (1K)                                         **/
/**   FC00-FC0C  status bytes; FCFC-FCFF the image's "FUJI" claim          **/
/**   FD00+r     arm register r (r < 80H)                                  **/
/**   FD80+n     APPBANK: map image page n at C000H-DFFFH                  **/
/**   FDFE       swap to the staged boot image (BOOTLOCK-armed only)       **/
/**   FE00+v     armed register = v, then disarm                           **/
/**   FF00+v     append v to the TX stream (max 320)                       **/
/**                                                                        **/
/** A committed transaction becomes one FujiBus packet sent to fujinet-pc  **/
/** over Bus-over-IP (raw TCP, SLIP-framed, byte-identical to the USB link)**/
/** The reply is painted into the window, FN_ACKSEQ last.                  **/
/****************************************************************************/

#include "FujiCart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "SDL.h"

#ifdef _WIN32
# include <winsock2.h>
# include <ws2tcpip.h>
# pragma comment(lib, "Ws2_32.lib")
typedef SOCKET fc_socket_t;
# define FC_INVALID INVALID_SOCKET
# define fc_close(s)  closesocket(s)
# define fc_errno()   WSAGetLastError()
# define FC_EWOULDBLOCK WSAEWOULDBLOCK
# define FC_EINPROGRESS WSAEWOULDBLOCK
#else
# include <unistd.h>
# include <fcntl.h>
# include <sys/types.h>
# include <sys/socket.h>
# include <sys/select.h>
# include <netinet/in.h>
# include <netinet/tcp.h>
# include <netdb.h>
typedef int fc_socket_t;
# define FC_INVALID (-1)
# define fc_close(s)  close(s)
# define fc_errno()   errno
# define FC_EWOULDBLOCK EWOULDBLOCK
# define FC_EINPROGRESS EINPROGRESS
#endif

typedef unsigned char byte;

/* From Coleco.c */
extern byte *CART;
extern int   CartSize;
extern byte *AddrTabl[256];
extern int   Verbose;
int Coleco_InstallCart (const byte *image,int size);

/* ---- window offsets (console address - F800H) ------------------------- */
#define W_REPLY      0x000
#define W_ACKSEQ     0x400
#define W_STATUS     0x401
#define W_ERR        0x402
#define W_REPLYCMD   0x403
#define W_RXLEN_LO   0x404
#define W_RXLEN_HI   0x405
#define W_BOOTSTAT   0x406
#define W_BOOTPCT    0x407
#define W_BOOTERR    0x408
#define W_MAGIC0     0x409
#define W_MAGIC1     0x40A
#define W_PROTOVER   0x40B
#define W_SLICE      0x40C

#define REPLY_MAX    1024
#define TX_MAX       320
#define NPARAM_MAX   8

/* registers */
#define R_DEVICE     0x00
#define R_COMMAND    0x01
#define R_NPARAM     0x02
#define R_DATA_RST   0x05
#define R_RXSLICE    0x06
#define R_SEQ        0x10
#define R_BOOTLOCK   0x11
#define R_BOOTSEL1   0x12
#define R_BOOTSEL2   0x13

/* FN_ERR */
#define E_OK         0
#define E_NOLINK     1
#define E_TIMEOUT    2
#define E_BADFRAME   3
#define E_TOOBIG     4

/* FN_BOOTSTAT / FN_BOOTERR */
#define BS_IDLE      0
#define BS_XFER      1
#define BS_READY     2
#define BS_FAILED    0x80
#define BE_TOOBIG    1
#define BE_TRUNC     2
#define BE_NOMAP     3

/* FujiBus */
#define DEV_FUJI     0x70
#define DEV_DBC      0xFF               /* the cartridge's companion MCU    */
#define CMD_MOUNT_IMAGE 0xF8
#define CMD_ACK      0x06
#define CMD_NAK      0x15
#define NET_OPEN     'O'
#define NET_CLOSE    'C'
#define NET_WRITE    'W'

#define SL_END       0xC0
#define SL_ESC       0xDB
#define SL_ESCEND    0xDC
#define SL_ESCESC    0xDD

#define TIMEOUT_MS        5000
#define TIMEOUT_MOUNT_MS  60000
#define RECONNECT_MS      2000
#define IMAGE_MAX         (128*1024)
#define CFG_MAX           (4*1024)

int FujiCart_On=0;

static char fc_host[128]=FUJICART_DEFAULT_HOST;
static int  fc_port=FUJICART_DEFAULT_PORT;
static int  fc_disabled=0;

static byte fc_win[0x800];              /* what the console sees at F800H   */
static byte fc_zero_page[0x2000];       /* past the image's last page       */

/* mailbox registers */
static int  fc_armed=-1;
static byte fc_device,fc_command,fc_nparam;
static int  fc_bootlock=0;
static byte fc_tx[TX_MAX];
static int  fc_txlen=0,fc_txover=0;

/* transaction */
static int    fc_pending=0;
static byte   fc_pendseq;
static Uint32 fc_deadline;

/* APPBANK */
static int fc_page=0;

/* link */
static fc_socket_t fc_sock=FC_INVALID;
static int    fc_connecting=0;
#ifdef _WIN32
static int    fc_wsa=0;
#endif
static Uint32 fc_next_try=0;

/* SLIP receive */
static byte fc_rx[4096];
static int  fc_rxlen=0,fc_rxesc=0,fc_rxover=0;

/* DBC push (MOUNT_IMAGE): stream 0 the ROM, stream 1 its .cfg */
static byte *fc_stage[2];
static int   fc_stsize[2],fc_stgot[2],fc_stopen[2];

/* ======================================================================= */
/* helpers                                                                 */
/* ======================================================================= */
static void link_down (void);

static unsigned fold (unsigned chk,const byte *p,int n)
{
 while (n-->0)
 {
  chk+=*p++;
  chk=(chk>>8)+(chk&0xFF);
 }
 return chk;
}

static void paint_status (void)
{
 fc_win[W_STATUS]=(fc_sock!=FC_INVALID && !fc_connecting)? 0x01:0x00;
 fc_win[W_MAGIC0]='F';
 fc_win[W_MAGIC1]='N';
 fc_win[W_PROTOVER]=1;                  /* 1 serves flat and APPBANK images */
 fc_win[W_SLICE]=0;
}

/* The transaction's verdict; ACKSEQ is painted last (the interlock). */
static void finish (byte seq,int err,byte replycmd,const byte *data,int len)
{
 if (err==E_OK)
 {
  if (len>REPLY_MAX) { len=REPLY_MAX; err=E_TOOBIG; }
  if (len>0) memcpy (fc_win+W_REPLY,data,len);
  fc_win[W_REPLYCMD]=replycmd;
 }
 else
 {
  fc_win[W_REPLYCMD]=0;
  len=0;
 }
 fc_win[W_ERR]=(byte)err;
 fc_win[W_RXLEN_LO]=(byte)(len&0xFF);
 fc_win[W_RXLEN_HI]=(byte)(len>>8);
 paint_status ();
 fc_win[W_ACKSEQ]=seq;
 fc_pending=0;
}

/* ======================================================================= */
/* the link to fujinet-pc                                                  */
/* ======================================================================= */
static void set_nonblock (fc_socket_t s)
{
#ifdef _WIN32
 u_long mode=1;
 ioctlsocket (s,FIONBIO,&mode);
#else
 fcntl (s,F_SETFL,fcntl(s,F_GETFL,0)|O_NONBLOCK);
#endif
}

static void start_connect (void)
{
 struct addrinfo hints,*res=NULL;
 char port[16];
 int on=1,r;

#ifdef _WIN32
 if (!fc_wsa)
 {
  WSADATA wsa;
  if (WSAStartup (MAKEWORD(2,2),&wsa)!=0) return;
  fc_wsa=1;
 }
#endif
 memset (&hints,0,sizeof(hints));
 hints.ai_family=AF_INET;
 hints.ai_socktype=SOCK_STREAM;
 sprintf (port,"%d",fc_port);
 if (getaddrinfo (fc_host,port,&hints,&res)!=0 || !res) return;
 fc_sock=socket (res->ai_family,res->ai_socktype,res->ai_protocol);
 if (fc_sock==FC_INVALID) { freeaddrinfo (res); return; }
 setsockopt (fc_sock,IPPROTO_TCP,TCP_NODELAY,(const char *)&on,sizeof(on));
 set_nonblock (fc_sock);
 r=connect (fc_sock,res->ai_addr,(int)res->ai_addrlen);
 freeaddrinfo (res);
 if (r==0) fc_connecting=0;
 else if (fc_errno()==FC_EINPROGRESS || fc_errno()==FC_EWOULDBLOCK) fc_connecting=1;
 else { fc_close (fc_sock); fc_sock=FC_INVALID; }
}

/* Nonblocking connect finished? */
static void check_connect (void)
{
 fd_set wf,ef;
 struct timeval tv={0,0};
 int err=0;
 socklen_t len=sizeof(err);
 FD_ZERO (&wf); FD_ZERO (&ef);
 FD_SET (fc_sock,&wf); FD_SET (fc_sock,&ef);
 if (select ((int)(fc_sock+1),NULL,&wf,&ef,&tv)<=0) return;
 if (FD_ISSET (fc_sock,&ef) ||
     getsockopt (fc_sock,SOL_SOCKET,SO_ERROR,(char *)&err,&len)!=0 || err)
 {
  fc_close (fc_sock); fc_sock=FC_INVALID; fc_connecting=0;
  return;
 }
 fc_connecting=0;
 fc_rxlen=0; fc_rxesc=0; fc_rxover=0;
 if (Verbose) printf ("FujiCart: connected to fujinet-pc at %s:%d\n",fc_host,fc_port);
}

static void link_down (void)
{
 if (fc_sock!=FC_INVALID)
 {
  fc_close (fc_sock);
  fc_sock=FC_INVALID;
  if (Verbose) printf ("FujiCart: link to fujinet-pc lost\n");
 }
 fc_connecting=0;
 fc_next_try=SDL_GetTicks ()+RECONNECT_MS;
 if (fc_pending) finish (fc_pendseq,E_NOLINK,0,NULL,0);
 else paint_status ();
}

/* Send all of buf (a few hundred bytes); short waits only. */
static int send_all (const byte *buf,int n)
{
 int off=0,tries=0;
 while (off<n)
 {
  int r=(int)send (fc_sock,(const char *)buf+off,n-off,0);
  if (r>0) { off+=r; continue; }
  if (r<0 && (fc_errno()==FC_EWOULDBLOCK) && ++tries<200)
  {
   SDL_Delay (1);
   continue;
  }
  return 0;
 }
 return 1;
}

/* One FujiBus packet, SLIP-framed. */
static int send_packet (const byte *pkt,int n)
{
 static byte out[2*1400+2];
 int i,o=0;
 if (fc_sock==FC_INVALID || fc_connecting) return 0;
 out[o++]=SL_END;
 for (i=0;i<n;++i)
 {
  if (pkt[i]==SL_END)      { out[o++]=SL_ESC; out[o++]=SL_ESCEND; }
  else if (pkt[i]==SL_ESC) { out[o++]=SL_ESC; out[o++]=SL_ESCESC; }
  else out[o++]=pkt[i];
 }
 out[o++]=SL_END;
 if (!send_all (out,o)) { link_down (); return 0; }
 return 1;
}

/* A reply-less packet (header only) with command cmd. */
static void send_simple (byte dev,byte cmd)
{
 byte p[6];
 p[0]=dev; p[1]=cmd; p[2]=6; p[3]=0; p[4]=0; p[5]=0;
 p[4]=(byte)fold (0,p,6);
 send_packet (p,6);
}

/* ======================================================================= */
/* transactions                                                            */
/* ======================================================================= */

/* Descriptor value for a run of fields: 1-byte x1..4 = 1..4, 2-byte x1..2 =
   5..6, 4-byte x1 = 7 (FEP-004). */
static int field_run (int size,int count,int *used)
{
 switch (size)
 {
  case 1: *used=(count>4)? 4:count; return *used;
  case 2: *used=(count>2)? 2:count; return 4+*used;
  case 4: *used=1; return 7;
 }
 *used=0;
 return -1;
}

static void commit (byte seq)
{
 static byte pkt[16+TX_MAX];
 byte sizes[NPARAM_MAX],desc[NPARAM_MAX];
 const byte *fields;
 int np,pos,i,nd=0,fieldlen,datalen,n;

 if (fc_sock==FC_INVALID || fc_connecting) { finish (seq,E_NOLINK,0,NULL,0); return; }
 if (fc_txover) { finish (seq,E_TOOBIG,0,NULL,0); return; }

 /* the TX stream: NPARAM x (size, value...), then the payload */
 np=fc_nparam; if (np>NPARAM_MAX) np=NPARAM_MAX;
 pos=0;
 for (i=0;i<np;++i)
 {
  if (pos>=fc_txlen) { finish (seq,E_BADFRAME,0,NULL,0); return; }
  sizes[i]=fc_tx[pos];
  if ((sizes[i]!=1 && sizes[i]!=2 && sizes[i]!=4) || pos+1+sizes[i]>fc_txlen)
  { finish (seq,E_BADFRAME,0,NULL,0); return; }
  pos+=1+sizes[i];
 }
 /* descriptors: runs of equal-sized fields */
 for (i=0;i<np;)
 {
  int run=1,used;
  while (i+run<np && sizes[i+run]==sizes[i]) ++run;
  desc[nd++]=(byte)field_run (sizes[i],run,&used);
  i+=used;
 }
 if (nd==0) desc[nd++]=0;
 for (i=0;i<nd-1;++i) desc[i]|=0x80;   /* more descriptors follow */

 /* header, extra descriptors, field values, payload */
 n=0;
 pkt[n++]=fc_device;
 pkt[n++]=fc_command;
 n+=2;                                  /* length, below */
 pkt[n++]=0;                            /* checksum, below */
 for (i=0;i<nd;++i) pkt[n++]=desc[i];
 fieldlen=0;
 for (i=0,pos=0;i<np;++i)
 {
  memcpy (pkt+n,fc_tx+pos+1,sizes[i]);
  n+=sizes[i]; fieldlen+=sizes[i];
  pos+=1+sizes[i];
 }
 fields=fc_tx+pos;
 datalen=fc_txlen-pos;
 memcpy (pkt+n,fields,datalen);
 n+=datalen;
 pkt[2]=(byte)(n&0xFF);
 pkt[3]=(byte)(n>>8);
 pkt[4]=(byte)fold (0,pkt,n);
 (void)fieldlen;

 fc_pending=1;
 fc_pendseq=seq;
 fc_deadline=SDL_GetTicks ()+((fc_device==DEV_FUJI && fc_command==CMD_MOUNT_IMAGE)?
                              TIMEOUT_MOUNT_MS:TIMEOUT_MS);
 if (!send_packet (pkt,n) && fc_pending) finish (seq,E_NOLINK,0,NULL,0);
}

/* ======================================================================= */
/* DBC push: the ESP32 streams a MOUNT_IMAGE'd file to the "RP2040"        */
/* ======================================================================= */
static void boot_fail (int err)
{
 fc_win[W_BOOTERR]=(byte)err;
 fc_win[W_BOOTSTAT]=BS_FAILED;
}

static void dbc_frame (byte cmd,const unsigned *fv,int nf,const byte *data,int len)
{
 int sid=(nf>0)? (int)(fv[0]&0xFF):0;
 if (sid<0 || sid>1) sid=0;
 switch (cmd)
 {
  case NET_OPEN:
  {
   unsigned size=(nf>1)? fv[1]:
                 (len>=4)? (unsigned)(data[0]|(data[1]<<8)|(data[2]<<16)|((unsigned)data[3]<<24)):0;
   int max=(sid==0)? IMAGE_MAX:CFG_MAX;
   if (size==0 || size>(unsigned)max)
   {
    if (sid==0) boot_fail (BE_TOOBIG);
    send_simple (DEV_DBC,CMD_NAK);
    return;
   }
   free (fc_stage[sid]);
   fc_stage[sid]=(byte *)malloc (size);
   if (!fc_stage[sid]) { send_simple (DEV_DBC,CMD_NAK); return; }
   fc_stsize[sid]=(int)size; fc_stgot[sid]=0; fc_stopen[sid]=1;
   if (sid==0)
   {
    fc_win[W_BOOTERR]=0;
    fc_win[W_BOOTPCT]=0;
    fc_win[W_BOOTSTAT]=BS_XFER;
   }
   break;
  }
  case NET_WRITE:
   if (!fc_stopen[sid]) { send_simple (DEV_DBC,CMD_NAK); return; }
   if (fc_stgot[sid]+len>fc_stsize[sid])
   {
    if (sid==0) boot_fail (BE_TOOBIG);
    fc_stopen[sid]=0;
    send_simple (DEV_DBC,CMD_NAK);
    return;
   }
   memcpy (fc_stage[sid]+fc_stgot[sid],data,len);
   fc_stgot[sid]+=len;
   if (sid==0) fc_win[W_BOOTPCT]=(byte)((fc_stgot[0]*100)/fc_stsize[0]);
   break;
  case NET_CLOSE:
   if (!fc_stopen[sid]) { send_simple (DEV_DBC,CMD_NAK); return; }
   fc_stopen[sid]=0;
   if (sid==0)
   {
    if (fc_stgot[0]!=fc_stsize[0]) boot_fail (BE_TRUNC);
    else
    {
     fc_win[W_BOOTPCT]=100;
     fc_win[W_BOOTSTAT]=BS_READY;
    }
   }
   break;
  default:
   break;                               /* anything else: just ACK it      */
 }
 send_simple (DEV_DBC,CMD_ACK);
}

/* The swap: read of FDFE with BOOTLOCK armed and an image ready. */
static void do_swap (void)
{
 byte *img;
 int size;
 if (!fc_bootlock || fc_win[W_BOOTSTAT]!=BS_READY || !fc_stage[0]) return;
 fc_bootlock=0;
 img=fc_stage[0]; size=fc_stsize[0];
 fc_stage[0]=NULL;
 fc_win[W_BOOTSTAT]=BS_IDLE;
 if (Verbose) printf ("FujiCart: swapping to the pushed %d-byte image\n",size);
 if (!Coleco_InstallCart (img,size)) boot_fail (BE_NOMAP);
 free (img);
}

/* ======================================================================= */
/* receive                                                                 */
/* ======================================================================= */
static const byte field_bytes[8]={0,1,2,3,4,2,4,4};
static const byte field_count[8]={0,1,2,3,4,1,2,1};
static const byte field_size[8] ={0,1,1,1,1,2,2,4};

static void got_frame (const byte *f,int n)
{
 unsigned fv[32];
 int len,i,nf=0,flen=0;
 byte d;
 byte chk;
 byte hdr[6];

 if (n<6) goto bad;
 len=f[2]|(f[3]<<8);
 if (len!=n) goto bad;
 memcpy (hdr,f,6); hdr[4]=0;
 chk=(byte)fold (fold (0,hdr,6),f+6,n-6);
 if (chk!=f[4]) goto bad;

 /* descriptors and their field values */
 i=5;
 do
 {
  int k,s;
  if (i>=n) goto bad;
  d=f[i++];
  flen+=field_bytes[d&7];
  for (k=0,s=field_size[d&7];k<field_count[d&7] && nf<32;++k) fv[nf++]=s; /* sizes for now */
 } while (d&0x80);
 if (i+flen>n) goto bad;
 {
  int k,p=i;
  for (k=0;k<nf;++k)
  {
   int s=(int)fv[k],b;
   unsigned v=0;
   for (b=0;b<s;++b) v|=(unsigned)f[p+b]<<(8*b);
   fv[k]=v; p+=s;
  }
 }

 if (f[0]==DEV_DBC)
 {
  dbc_frame (f[1],fv,nf,f+i+flen,n-i-flen);
  return;
 }
 if (fc_pending && f[0]==fc_device)
 {
  finish (fc_pendseq,E_OK,f[1],f+i+flen,n-i-flen);
  return;
 }
 return;                                /* late or stray: ignored          */
bad:
 if (fc_pending) finish (fc_pendseq,E_BADFRAME,0,NULL,0);
}

static void receive (void)
{
 byte buf[1024];
 int r,i;
 for (;;)
 {
  r=(int)recv (fc_sock,(char *)buf,sizeof(buf),0);
  if (r==0) { link_down (); return; }
  if (r<0)
  {
   if (fc_errno()!=FC_EWOULDBLOCK) link_down ();
   return;
  }
  for (i=0;i<r;++i)
  {
   byte c=buf[i];
   if (c==SL_END)
   {
    if (fc_rxlen>0 && !fc_rxover) got_frame (fc_rx,fc_rxlen);
    else if (fc_rxover && fc_pending) finish (fc_pendseq,E_TOOBIG,0,NULL,0);
    fc_rxlen=0; fc_rxesc=0; fc_rxover=0;
    if (fc_sock==FC_INVALID) return;
    continue;
   }
   if (fc_rxesc)
   {
    fc_rxesc=0;
    c=(c==SL_ESCEND)? SL_END:(c==SL_ESCESC)? SL_ESC:c;
   }
   else if (c==SL_ESC) { fc_rxesc=1; continue; }
   if (fc_rxlen<(int)sizeof(fc_rx)) fc_rx[fc_rxlen++]=c;
   else fc_rxover=1;
  }
 }
}

/* ======================================================================= */
/* the console side                                                        */
/* ======================================================================= */
static void reg_write (int r,byte v)
{
 switch (r)
 {
  case R_DEVICE:   fc_device=v; break;
  case R_COMMAND:  fc_command=v; break;
  case R_NPARAM:   fc_nparam=v; break;
  case R_DATA_RST: fc_txlen=0; fc_txover=0; break;
  case R_RXSLICE:  break;               /* vestigial: one slice only        */
  case R_SEQ:
   if (v!=0 && v!=fc_win[W_ACKSEQ] && !fc_pending) commit (v);
   break;
  case R_BOOTLOCK: fc_bootlock=(v==0xB5); break;
  case R_BOOTSEL1: case R_BOOTSEL2: break; /* firmware update: not here    */
  default: break;                       /* undefined registers: no-op      */
 }
}

void FujiCart_MapPage (void)
{
 int i;
 byte *base;
 if (!FujiCart_On) return;
 if (fc_page==0) base=CART+0x4000;      /* the window's own C000H          */
 else
 {
  int off=0x8000+(fc_page-1)*0x2000;
  base=(off+0x2000<=CartSize)? CART+off:fc_zero_page;
 }
 for (i=0;i<0x20;++i) AddrTabl[0xC0+i]=base+i*0x100;
 /* the mailbox pages show the painted window to every kind of read */
 for (i=0;i<8;++i) AddrTabl[0xF8+i]=fc_win+i*0x100;
}

unsigned char FujiCart_Read (unsigned a)
{
 byte v;
 a&=0xFFFF;
 v=fc_win[a-FUJICART_WINDOW];
 if (a<0xFD00) return v;
 switch (a>>8)
 {
  case 0xFD:
  {
   unsigned lo=a&0xFF;
   if (lo<0x80) fc_armed=(int)lo;
   else
   {
    fc_armed=-1;
    if (lo==0xFE) do_swap ();
    else if (lo<0xFE) { fc_page=(int)(lo-0x80); FujiCart_MapPage (); }
   }
   break;
  }
  case 0xFE:
   if (fc_armed>=0) { reg_write (fc_armed,(byte)(a&0xFF)); fc_armed=-1; }
   break;
  case 0xFF:
   fc_armed=-1;
   if (fc_txlen<TX_MAX) fc_tx[fc_txlen++]=(byte)(a&0xFF);
   else fc_txover=1;
   break;
 }
 return v;
}

void FujiCart_Poll (void)
{
 Uint32 now;
 if (!FujiCart_On) return;
 now=SDL_GetTicks ();
 if (fc_sock==FC_INVALID)
 {
  if ((Sint32)(now-fc_next_try)>=0)
  {
   fc_next_try=now+RECONNECT_MS;
   start_connect ();
   paint_status ();
  }
 }
 else if (fc_connecting)
 {
  check_connect ();
  if (fc_sock==FC_INVALID) fc_next_try=now+RECONNECT_MS;
  paint_status ();
 }
 else receive ();
 if (fc_pending && (Sint32)(now-fc_deadline)>=0)
  finish (fc_pendseq,E_TIMEOUT,0,NULL,0);
}

/* ======================================================================= */
/* setup                                                                   */
/* ======================================================================= */
int FujiCart_SetEndpoint (const char *spec)
{
 const char *colon;
 if (!spec || !*spec) return 0;
 if (!strcmp (spec,"off") || !strcmp (spec,"0")) { fc_disabled=1; return 1; }
 fc_disabled=0;
 colon=strrchr (spec,':');
 if (colon)
 {
  int hl=(int)(colon-spec);
  if (hl<=0 || hl>=(int)sizeof(fc_host)) return 0;
  memcpy (fc_host,spec,hl); fc_host[hl]='\0';
  fc_port=atoi (colon+1);
 }
 else if (spec[0]>='0' && spec[0]<='9') fc_port=atoi (spec);
 else
 {
  strncpy (fc_host,spec,sizeof(fc_host)-1);
  fc_host[sizeof(fc_host)-1]='\0';
 }
 return fc_port>0 && fc_port<65536;
}

int FujiCart_HasClaim (const unsigned char *image,int size)
{
 /* as Gearcoleco: a valid 55AA/AA55 header at 0 plus the claim */
 int hdr=(image[0]==0x55 && image[1]==0xAA) || (image[0]==0xAA && image[1]==0x55);
 return hdr && size>=0x8000 && !memcmp (image+FUJICART_CLAIM,"FUJI",4);
}

int FujiCart_Attach (void)
{
 int keep_ack=FujiCart_On;              /* a swap keeps the cart's state   */
 byte ack=fc_win[W_ACKSEQ];
 byte boot[3];
 if (fc_disabled || !CART || !FujiCart_HasClaim (CART,CartSize))
 {
  FujiCart_On=0;
  return 0;
 }
 memcpy (boot,fc_win+W_BOOTSTAT,3);
 /* the image's own top 2K (clear but for the claim), then the paint */
 memcpy (fc_win,CART+0x7800,0x800);
 if (keep_ack) { fc_win[W_ACKSEQ]=ack; memcpy (fc_win+W_BOOTSTAT,boot,3); }
 FujiCart_On=1;
 fc_page=0;
 fc_armed=-1;
 paint_status ();
 FujiCart_MapPage ();
 if (!keep_ack)
 {
  fc_next_try=0;
  if (Verbose)
   printf ("FujiCart: FujiNet cartridge image (%d bytes); fujinet-pc at %s:%d\n",
           CartSize,fc_host,fc_port);
 }
 return 1;
}

void FujiCart_Shutdown (void)
{
 int i;
 if (fc_sock!=FC_INVALID) { fc_close (fc_sock); fc_sock=FC_INVALID; }
 for (i=0;i<2;++i) { free (fc_stage[i]); fc_stage[i]=NULL; }
#ifdef _WIN32
 if (fc_wsa) { WSACleanup (); fc_wsa=0; }
#endif
 FujiCart_On=0;
}
