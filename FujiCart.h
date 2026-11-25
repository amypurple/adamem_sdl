/****************************************************************************/
/**                                                                        **/
/**                               FujiCart.h                               **/
/**                                                                        **/
/** FujiNet ColecoVision cartridge: the read-only "mailbox" (protocol 1)   **/
/** plus APPBANK paging, bridged to a FujiNet (fujinet-pc) over Bus-over-IP**/
/** (a raw TCP stream of SLIP-framed FujiBus packets).                     **/
/**                                                                        **/
/** Reference: FujiNet ColecoVision Programmers Handbook (fujinet-firmware **/
/** pico/coleco), first edition 2026.                                      **/
/****************************************************************************/
#ifndef FUJICART_H
#define FUJICART_H

#define FUJICART_DEFAULT_HOST "127.0.0.1"
#define FUJICART_DEFAULT_PORT 9995

/* Console addresses */
#define FUJICART_WINDOW   0xF800        /* mailbox pages: F800H-FFFFH       */
#define FUJICART_CLAIM    0x7CFC        /* "FUJI" claim, as a cart offset   */

extern int FujiCart_On;                 /* nonzero: the mailbox is live     */

/* Set the fujinet-pc endpoint ("host:port", "port" or "off"). Call before
   FujiCart_Attach. Returns 0 on a malformed argument. */
int  FujiCart_SetEndpoint (const char *spec);

/* Nonzero if a cartridge image carries the FUJI claim (a FujiNet client). */
int  FujiCart_HasClaim (const unsigned char *image,int size);

/* Take over the cartridge window for a claimed image already in CART:
   paints the mailbox and starts connecting. Returns nonzero if live. */
int  FujiCart_Attach (void);

/* Cartridge read at F800H-FFFFH (hotspots, status, reply). */
unsigned char FujiCart_Read (unsigned a);

/* Called often from the emulation thread (about every emulated ms):
   network I/O, timeouts, reconnects. */
void FujiCart_Poll (void);

/* Map the current APPBANK page at C000H-DFFFH (after a memory remap). */
void FujiCart_MapPage (void);

void FujiCart_Shutdown (void);

#endif
