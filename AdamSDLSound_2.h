#ifndef _ADAMSDLSOUND_H
#define _ADAMSDLSOUND_H

#include "Sound.h"
#include "SDL.h"
/*
 *  AdamSDLSound_2.h
 *  adamem
 *
 *  Created by Geoffrey Oltmans on 6/10/08.
 *  Copyright 2008 Geoffrey Oltmans. All rights reserved.
 *
 *  2026: the chip state now lives privately in AdamSDLSound_2.c.
 */

void soundData(void *userdata, Uint8 *stream, int len);

#endif
