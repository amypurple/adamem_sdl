# FujiNet ColecoVision cartridge in ADAMEm

ADAMEm emulates the FujiNet ColecoVision cartridge, as described in the
*FujiNet ColecoVision Programmers Handbook*. The emulator stands in for the
cartridge's RP2040, and a running `fujinet-pc` stands in for its ESP32-S3.

## Running

1. Start `fujinet-pc` with its Bus-over-IP (BoIP) listener. The handbook uses
   `127.0.0.1:9995`.
2. Run a FujiNet client image (for example `fujiblst.col` or M.U.L.E.'s
   ColecoVision build) in ColecoVision mode:

       adamem -cv fujiblst.col
       adamem -cv -fujicart 127.0.0.1:9995 fujiblst.col

The cartridge switches on automatically for any image that carries the
`FUJI` claim at cart offset `7CFCH`; other cartridges behave as before.

| Option | Meaning |
|---|---|
| `-fujicart host:port` (`-fc`) | Where fujinet-pc listens [127.0.0.1:9995] |
| `-fujicart port` | Same, on 127.0.0.1 |
| `-fujicart off` | Never enable the cartridge |

`-fujinet` is unrelated: it is the ADAM's AdamNet link, used in ADAM mode.

With `-verbose 1` the console log shows when the cartridge is found, when
the link to fujinet-pc comes up or drops, and image swaps.

## What is emulated

* **Mailbox (protocol 1).** It covers the hotspot pages at `FD00H`, `FE00H`
  and `FF00H`, the register file, the 320-byte TX stream, the 1K reply window
  at `F800H` and the status bytes at `FC00H`. `FN_ACKSEQ` is always painted
  last.
* **Defences.**
  * A REGDATA read without a REGSEL read is ignored.
  * A transaction launches only on a SEQ that is nonzero and new.
  * The swap at `FDFEH` needs BOOTLOCK set to `B5H`.
  * Cartridge state survives a console RESET.
* **APPBANK.** A read of `FD80H+n` maps image page n at `C000H`-`DFFFH`.
  Page 0 is image `4000H`; page n >= 1 is image `8000H + (n-1)*2000H`.
* **FujiBus.**
  * Each transaction is sent as one SLIP-framed packet over TCP.
  * Parameters are packed into FEP-004 field descriptors, chained with
    bit 7.
  * Checksum: an end-around-carry byte sum.
  * The verdicts in `FN_ERR`:
    * 1: no link
    * 2: timeout (5 s, or 60 s for `MOUNT_IMAGE`)
    * 3: bad frame
    * 4: too big
* **Boot push and swap.**
  * During `MOUNT_IMAGE`, frames for device `FFH` (OPEN / WRITE / CLOSE) are
    collected as the boot image, and each one is answered with an ACK.
  * `FN_BOOTSTAT` and `FN_BOOTPCT` move as the image arrives.
  * The swap installs the image. A claimed image keeps the mailbox; any
    other image runs as a plain game, with the same mapper detection as a
    cartridge file: flat, MegaCart or Activision.

## Not emulated

* The X-in-1 and SGM mappers for pushed games (flat, MegaCart and Activision
  are supported), and the Activision board's EEPROM.
* BOOTSEL (rebooting the cartridge into firmware-update mode).
