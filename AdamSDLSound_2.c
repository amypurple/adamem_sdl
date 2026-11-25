/*
 *  AdamSDLSound_2.c
 *  adamem
 *
 *  Created by Geoffrey Oltmans on 6/10/08.
 *  Copyright 2008 Geoffrey Oltmans. All rights reserved.
 *
 *  2026 sound core rewrite:
 *   - Fixed-point (16.16) chip clocks: the old integer "5 ticks per sample"
 *     (real value 5.073) made everything about 25 cents flat.
 *   - Both chips output bipolar +1/-1 square waves. A tone whose period is
 *     too short to be heard (0..5 on the SN76489, at or above Nyquist) is held
 *     at a constant +1, so writing only the volume register plays at that
 *     volume: PCM-style voice playback (e.g. Squish'em Sam) works.
 *   - Box-filtered (area-averaged) square waves to reduce aliasing.
 *   - SN76489 noise rates corrected to clock/512, /1024, /2048 (were one
 *     octave too low).
 *   - AY-3-8910 mixer fixed: output = (tone OR tone_off) AND (noise OR
 *     noise_off). With both off the channel holds +1, so AY volume-register
 *     sample playback works too.
 *   - AY envelope period register decoding fixed (16 bits, no /16), correct
 *     step rate, envelope restarts on every shape (R13) write.
 *   - AY noise: 17-bit LFSR (taps 0 and 3), 5-bit period.
 *   - AY logarithmic volume table (about 3 dB steps) instead of the SN table.
 *   - Cycle-timed register writes: each write is stamped with the Z80
 *     T-state count and queued; the audio thread replays the writes at their
 *     emulated time, on a clock that tracks the emulator (with slow rate
 *     correction). The emulator runs a whole frame in a burst then waits, so
 *     before this every write in a frame landed in the same audio buffer and
 *     sub-frame changes (PCM voices via volume writes) were lost. Timing is
 *     now accurate to one output sample (about 23 us).
 */

#include "AdamSDLSound_2.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "Z80.h"
#include "Coleco.h"

#define SAMPLE_RATE   44100
#define FIX_SHIFT     16
#define SN_CLOCK      3579545.0   /* SN76489AN input clock (Hz)           */
#define AY_CLOCK      1789772.0   /* AY-3-8910 input clock on the SGM (Hz) */
#define CHANNEL_PEAK  2550.0f     /* peak level of one channel at master 15 */

/* Timestamped register-write queue: single producer (emulator thread),
   single consumer (audio thread). Times are Z80 T-states, wrapping at 2^32;
   compare them only through signed differences. */
typedef struct {
	Uint32 t;      /* Z80_CyclesNow() at the write                     */
	Uint8  chip;   /* 0 = SN76489, 1 = AY-3-8910                       */
	Uint8  reg;
	Uint16 val;
} SoundEvent;

#define EVQ_SIZE 65536u                 /* power of two */
#define EVQ_MASK (EVQ_SIZE - 1u)

static SoundEvent   evq[EVQ_SIZE];
static SDL_atomic_t evq_head;           /* written by the audio thread only    */
static SDL_atomic_t evq_tail;           /* written by the emulator thread only */
static SDL_atomic_t emu_time;           /* all writes up to this time are queued */

/* Audio thread clock, in Z80 T-states. */
static Uint32 audio_time;
static double audio_frac;
static double lag_filtered;
static int    timing_primed;

static void ApplySoundReg (int r,int v);
static void ApplyPSGSoundReg (int r,int v);

int soundquality=3;

static int initialised;
static SDL_AudioDeviceID audio_dev;
static Uint8 channel_on[4]= {1,1,1,1};
static Uint8 sound_on=1;      /* user toggle                                */
static Uint8 sound_paused=0;  /* emulator paused (Stop/Resume)              */

/* Counter ticks per output sample, 16.16 fixed point. */
static int64_t sn_step;       /* SN76489 counters run at clock/16           */
static int64_t ay_step;       /* AY-3-8910 counters run at clock/8          */

/****************************************************************************/
/* SN76489AN                                                               */
/****************************************************************************/

typedef struct {
	int     period;   /* 10-bit tone period                                 */
	int     atten;    /* 0 = loudest .. 15 = off                            */
	int64_t count;    /* 16.16 ticks left before the next output toggle     */
	int     out;      /* 1 = high (+1), 0 = low (-1)                        */
} SNTone;

static struct {
	SNTone   tone[3];
	int      noise_ctrl;   /* bit 2 = white noise, bits 0-1 = rate          */
	int      noise_atten;
	int64_t  noise_count;
	int      noise_ff;     /* noise counter flip-flop; LFSR shifts on rise  */
	uint16_t lfsr;         /* 15-bit shift register (TI SN76489AN)          */
} sn;

/* 2 dB per attenuation step, 15 = silent. */
static float sn_vol[16];

/* One output sample of a tone channel, averaged over the sample, -1..+1. */
static float SN_ToneSample (SNTone *t)
{
	int64_t per = (int64_t)t->period << FIX_SHIFT;
	int64_t remaining = sn_step;
	int64_t acc = 0;

	if (per <= sn_step)
	{
		/* Period 0..5: inaudible (at or above Nyquist). Hold the output high so
		   the channel plays at its volume level: volume-register PCM playback. */
		t->out = 1;
		t->count = 0;
		return 1.0f;
	}
	while (t->count <= remaining)
	{
		acc += t->out ? t->count : -t->count;
		remaining -= t->count;
		t->count = per;
		t->out ^= 1;
	}
	acc += t->out ? remaining : -remaining;
	t->count -= remaining;
	return (float)acc / (float)sn_step;
}

static void SN_NoiseShift (void)
{
	int bit;
	if (sn.noise_ctrl & 4)   /* white noise: taps on bits 0 and 1 */
		bit = (sn.lfsr ^ (sn.lfsr >> 1)) & 1;
	else                     /* periodic noise: 15-step cycle */
		bit = sn.lfsr & 1;
	sn.lfsr = (uint16_t)((sn.lfsr >> 1) | (bit << 14));
}

static float SN_NoiseSample (void)
{
	int64_t per;
	int64_t remaining = sn_step;
	int64_t acc = 0;
	int mode = sn.noise_ctrl & 3;

	if (mode == 3)
	{
		per = sn.tone[2].period;
		if (per == 0) per = 1024;   /* TI chip: period 0 counts as 1024 */
	}
	else
		per = 16 << mode;           /* clock/512, /1024, /2048 */
	per <<= FIX_SHIFT;

	while (sn.noise_count <= remaining)
	{
		acc += (sn.lfsr & 1) ? sn.noise_count : -sn.noise_count;
		remaining -= sn.noise_count;
		sn.noise_count = per;
		sn.noise_ff ^= 1;
		if (sn.noise_ff)
			SN_NoiseShift ();
	}
	acc += (sn.lfsr & 1) ? remaining : -remaining;
	sn.noise_count -= remaining;
	return (float)acc / (float)sn_step;
}

/****************************************************************************/
/* AY-3-8910 (Super Game Module)                                           */
/****************************************************************************/

#ifdef SOUND_PSG
/* Envelope shapes over 32 steps (two cycles, so repeating shapes can wrap). */
static const Uint8 PSG_envelopeWaves[16][32] =
{
	{15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0},
	{15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15},
	{15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15},
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15},
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15},
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0},
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

/* AY-3-8910 DAC levels (logarithmic, about 3 dB per step), 0 = silent. */
static const float ay_vol[16] =
{
	0.0f,      0.00999f, 0.01445f, 0.02106f, 0.03070f, 0.04555f, 0.06450f, 0.10736f,
	0.12659f,  0.20499f, 0.29221f, 0.37284f, 0.49253f, 0.63532f, 0.80558f, 1.0f
};

static struct {
	Uint8    reg[16];
	int64_t  tone_count[3];
	int      tone_out[3];      /* 1 = high, 0 = low                        */
	int64_t  noise_count;
	uint32_t lfsr;             /* 17-bit shift register                    */
	int64_t  env_count;
	int      env_pos;          /* 0..31 into PSG_envelopeWaves             */
} ay;

static int AY_TonePeriod (int c)
{
	int p = ay.reg[c*2] | ((ay.reg[c*2+1] & 0x0F) << 8);
	return p ? p : 1;
}

/* Average of the tone's high/low state over one sample, 0..1. */
static float AY_ToneLevel (int c)
{
	int64_t per = (int64_t)AY_TonePeriod (c) << FIX_SHIFT;
	int64_t remaining = ay_step;
	int64_t high = 0;

	if (per <= ay_step)
	{
		/* Inaudible tone: hold high, as on the SN side. */
		ay.tone_out[c] = 1;
		ay.tone_count[c] = 0;
		return 1.0f;
	}
	while (ay.tone_count[c] <= remaining)
	{
		if (ay.tone_out[c]) high += ay.tone_count[c];
		remaining -= ay.tone_count[c];
		ay.tone_count[c] = per;
		ay.tone_out[c] ^= 1;
	}
	if (ay.tone_out[c]) high += remaining;
	ay.tone_count[c] -= remaining;
	return (float)high / (float)ay_step;
}

/* Average of the noise output over one sample, 0..1. */
static float AY_NoiseLevel (void)
{
	int np = ay.reg[6] & 0x1F;
	int64_t per, remaining = ay_step, high = 0;
	if (np == 0) np = 1;
	per = (int64_t)(np * 2) << FIX_SHIFT;  /* noise clock is half the tone clock */

	while (ay.noise_count <= remaining)
	{
		if (ay.lfsr & 1) high += ay.noise_count;
		remaining -= ay.noise_count;
		ay.noise_count = per;
		ay.lfsr = (ay.lfsr >> 1) | (((ay.lfsr ^ (ay.lfsr >> 3)) & 1) << 16);
	}
	if (ay.lfsr & 1) high += remaining;
	ay.noise_count -= remaining;
	return (float)high / (float)ay_step;
}

static void AY_EnvelopeStep (void)
{
	int shape = ay.reg[13] & 0x0F;
	int cont  = shape & 0x08;
	int hold  = shape & 0x01;
	if (ay.env_pos == 31)
	{
		if (cont && !hold)
			ay.env_pos = 0;
	}
	else
		ay.env_pos++;
}

/* Advance the envelope by one sample and return its current level 0..15. */
static int AY_EnvelopeLevel (void)
{
	int64_t ep = ay.reg[11] | (ay.reg[12] << 8);
	int level;
	if (ep == 0) ep = 1;
	/* One envelope step every 16*EP input clocks = 2*EP counter ticks. */
	ep = (ep * 2) << FIX_SHIFT;

	level = PSG_envelopeWaves[ay.reg[13] & 0x0F][ay.env_pos];
	ay.env_count -= ay_step;
	while (ay.env_count <= 0)
	{
		ay.env_count += ep;
		AY_EnvelopeStep ();
	}
	return level;
}

static float AY_Sample (void)
{
	float noise = AY_NoiseLevel ();
	int   env   = AY_EnvelopeLevel ();
	Uint8 mixer = ay.reg[7];
	float mix = 0.0f;
	int c;

	for (c = 0; c < 3; c++)
	{
		Uint8 v = ay.reg[8+c];
		float level = (v & 0x10) ? ay_vol[env] : ay_vol[v & 0x0F];
		float tone  = AY_ToneLevel (c);   /* always run so phase stays right */
		float t = (mixer & (1 << c))       ? 1.0f : tone;
		float n = (mixer & (1 << (c + 3))) ? 1.0f : noise;
		/* (tone OR tone_off) AND (noise OR noise_off), mapped to -1..+1 */
		mix += (2.0f * t * n - 1.0f) * level;
	}
	return mix;
}
#endif /* SOUND_PSG */

/****************************************************************************/
/* Audio callback                                                           */
/****************************************************************************/

void soundData(void *userdata, Uint8 *stream, int len)
{
	Sint16 *buffer = (Sint16 *)stream;
	int n = len / (int)sizeof(Sint16);
	int i, j;
	float gain;
	unsigned head, tail;
	Uint32 emu_now;
	Sint32 lag;
	double cps, cps_sec, target, ratio, step;
	int hold;
	(void)userdata;

	gain = (sound_on && !sound_paused) ? CHANNEL_PEAK * (float)mastervolume / 15.0f : 0.0f;

	/* Read the emulated time first, then the queue tail: every write stamped
	   at or before emu_now is then guaranteed to be visible. */
	emu_now = (Uint32)SDL_AtomicGet (&emu_time);
	SDL_MemoryBarrierAcquire ();
	tail = (unsigned)SDL_AtomicGet (&evq_tail);
	SDL_MemoryBarrierAcquire ();
	head = (unsigned)SDL_AtomicGet (&evq_head);

	/* Z80 T-states per output sample (Z80_IPeriod is one millisecond). */
	cps_sec = (double)Z80_IPeriod * 1000.0;
	if (cps_sec < 1000.0) cps_sec = 3579545.0;
	cps = cps_sec / SAMPLE_RATE;
	/* Stay behind the emulator by two frames plus this buffer: it runs each
	   frame in a burst, so its time jumps ahead by a frame at a time. */
	target = cps_sec * 2.0 / (IFreq > 0 ? IFreq : 60) + cps * n;

	lag = (Sint32)(emu_now - audio_time);
	if (!timing_primed || lag > target + cps_sec * 0.25 || lag < -cps_sec * 0.25)
	{
		/* Start-up, or far out of step (fast-forward, reset): jump. Writes
		   older than the new position are applied at once, in order. */
		audio_time = emu_now - (Uint32)target;
		audio_frac = 0.0;
		lag_filtered = target;
		lag = (Sint32)target;
		timing_primed = 1;
	}
	lag_filtered += ((double)lag - lag_filtered) / 16.0;

	/* Slow rate correction toward the target lag, at most +/-0.5%. */
	ratio = 1.0 + (lag_filtered - target) / (cps_sec * 2.0);
	if (ratio > 1.005) ratio = 1.005;
	if (ratio < 0.995) ratio = 0.995;
	step = cps * ratio;

	/* Too close to the emulator (start-up, after a pause): keep the chips
	   running but hold the clock for this buffer so the margin rebuilds. */
	hold = (double)lag < target * 0.5;

	for (i = 0; i < n; i++)
	{
		float mix = 0.0f;
		int   s;

		/* Apply writes that are due at this sample. */
		while (head != tail && (Sint32)(evq[head & EVQ_MASK].t - audio_time) <= 0)
		{
			SoundEvent *e = &evq[head & EVQ_MASK];
			if (e->chip) ApplyPSGSoundReg (e->reg,e->val);
			else         ApplySoundReg (e->reg,e->val);
			head = (head + 1u) & EVQ_MASK;
		}

		/* Chips always run, even when muted, so their phase stays correct. */
		for (j = 0; j < 3; j++)
		{
			float t = SN_ToneSample (&sn.tone[j]);
			if (channel_on[j])
				mix += t * sn_vol[sn.tone[j].atten];
		}
		{
			float t = SN_NoiseSample ();
			if (channel_on[3])
				mix += t * sn_vol[sn.noise_atten];
		}
#ifdef SOUND_PSG
		mix += AY_Sample ();
#endif
		s = (int)lrintf (mix * gain);
		if (s >  32767) s =  32767;
		if (s < -32768) s = -32768;
		*buffer++ = (Sint16)s;

		/* Advance the audio clock, never past the emulator's time. */
		if (!hold)
		{
			double adv = audio_frac + step;
			Uint32 whole = (Uint32)adv;
			Sint32 avail = (Sint32)(emu_now - audio_time);
			if (avail <= 0)
			{
				whole = 0;
				audio_frac = 0.0;
			}
			else if ((Sint32)whole > avail)
			{
				whole = (Uint32)avail;
				audio_frac = 0.0;
			}
			else
				audio_frac = adv - (double)whole;
			audio_time += whole;
		}
	}
	SDL_MemoryBarrierRelease ();
	SDL_AtomicSet (&evq_head,(int)head);
}

/****************************************************************************/
/* Register writes                                                          */
/****************************************************************************/

static void ApplySoundReg (int r,int v)
{
	switch (r)
	{
		case 0: case 2: case 4:
			sn.tone[r>>1].period = v & 0x3FF;
			break;
		case 1: case 3: case 5:
			sn.tone[r>>1].atten = v & 0x0F;
			break;
		case 6:
			/* Any write to the noise register resets the shift register. */
			sn.noise_ctrl = v & 0x07;
			sn.lfsr = 0x4000;
			break;
		case 7:
			sn.noise_atten = v & 0x0F;
			break;
	}
}

static void ApplyPSGSoundReg (int r,int v)
{
#ifdef SOUND_PSG
	if (r < 0 || r > 15)
		return;
	ay.reg[r] = (Uint8)v;
	if (r == 13)
	{
		/* Writing the shape restarts the envelope. */
		int64_t ep = ay.reg[11] | (ay.reg[12] << 8);
		if (ep == 0) ep = 1;
		ay.env_pos = 0;
		ay.env_count = (ep * 2) << FIX_SHIFT;
	}
#else
	(void)r; (void)v;
#endif
}

/* Emulator thread: stamp a write with the current Z80 time and queue it. */
static void PushSoundEvent (int chip,int r,int v)
{
	Uint32 now = (Uint32)Z80_CyclesNow ();
	unsigned tail, next;
	int tries = 0;

	if (!audio_dev)
	{
		if (chip) ApplyPSGSoundReg (r,v);
		else      ApplySoundReg (r,v);
		return;
	}
	tail = (unsigned)SDL_AtomicGet (&evq_tail);
	next = (tail + 1u) & EVQ_MASK;
	while (next == (unsigned)SDL_AtomicGet (&evq_head))
	{
		/* Queue full: the emulator is far ahead. Give the audio thread time;
		   if it is not draining at all, apply directly as a last resort. */
		if (++tries > 200)
		{
			SDL_LockAudioDevice (audio_dev);
			if (chip) ApplyPSGSoundReg (r,v);
			else      ApplySoundReg (r,v);
			SDL_UnlockAudioDevice (audio_dev);
			return;
		}
		SDL_Delay (1);
	}
	evq[tail].t    = now;
	evq[tail].chip = (Uint8)chip;
	evq[tail].reg  = (Uint8)r;
	evq[tail].val  = (Uint16)v;
	SDL_MemoryBarrierRelease ();
	SDL_AtomicSet (&evq_tail,(int)next);
	SDL_AtomicSet (&emu_time,(int)now);
}

/* Emulator thread, about once per emulated millisecond (from Z80_Interrupt). */
void SoundSync (void)
{
	SDL_MemoryBarrierRelease ();
	SDL_AtomicSet (&emu_time,(int)(Uint32)Z80_CyclesNow ());
}

#ifdef ADAM_SOUND_QUEUE
/* Kept for hosts that still call it; timing now comes from SoundSync. */
void AdamSoundFrameTick (void)
{
}
#endif

static void WriteSoundReg (int r,int v)
{
	PushSoundEvent (0,r,v);
}

static void PSGWriteSoundReg (int r,int v)
{
	PushSoundEvent (1,r,v);
}

/****************************************************************************/
/* Driver                                                                   */
/****************************************************************************/

static void ResetChips (void)
{
	int i;
	memset (&sn,0,sizeof(sn));
	for (i = 0; i < 3; i++)
	{
		sn.tone[i].atten = 15;
		sn.tone[i].out = 1;
	}
	sn.noise_atten = 15;
	sn.lfsr = 0x4000;
#ifdef SOUND_PSG
	memset (&ay,0,sizeof(ay));
	ay.reg[7] = 0xFF;          /* all tone and noise outputs disabled */
	ay.lfsr = 1;
	for (i = 0; i < 3; i++)
		ay.tone_out[i] = 1;
#endif
}

static int Init (void)
{
	SDL_AudioSpec desired, obtained;
	int i;

	/* Re-anchor the audio clock to the emulator on the next callback. */
	timing_primed = 0;

	if (initialised)
		return 1;

	sn_step = (int64_t)(SN_CLOCK / 16.0 / SAMPLE_RATE * (1 << FIX_SHIFT) + 0.5);
	ay_step = (int64_t)(AY_CLOCK /  8.0 / SAMPLE_RATE * (1 << FIX_SHIFT) + 0.5);
	for (i = 0; i < 15; i++)
		sn_vol[i] = (float)pow (10.0, -0.1 * i);   /* -2 dB per step */
	sn_vol[15] = 0.0f;
	ResetChips ();

	SDL_zero (desired);
	desired.freq = SAMPLE_RATE;
	desired.format = AUDIO_S16SYS;
	desired.channels = 1;
	desired.samples = 512;
	desired.callback = soundData;
	desired.userdata = NULL;

	audio_dev = SDL_OpenAudioDevice (NULL, 0, &desired, &obtained, 0);
	if (!audio_dev)
	{
		if (Verbose) printf ("Couldn't open audio: %s\n", SDL_GetError ());
		return 0;
	}
	if (Verbose) printf ("OK\n  Opened audio...OK\n");
	SDL_PauseAudioDevice (audio_dev, 0); /* start audio playing. */

	initialised = 1;
	return 1;
}

static void Reset (void)
{
}

static void Toggle (void)
{
	sound_on ^= 1;
}

static void ToggleChannel (int channel)
{
	if (channel >= 0 && channel < 4)
		channel_on[channel] ^= 1;
}

static void SetMasterVolume (int newvolume)
{
	/* The callback reads the global mastervolume directly. */
	(void)newvolume;
}

static void Stop (void)
{
	sound_paused = 1;
}

static void Resume (void)
{
	sound_paused = 0;
}

struct SoundDriver_struct SB_SoundDriver=
{
	Init,
	Reset,
	WriteSoundReg,
	Toggle,
	ToggleChannel,
	SetMasterVolume,
	Stop,
	Resume,
#ifdef SOUND_PSG
	PSGWriteSoundReg
#endif
};
