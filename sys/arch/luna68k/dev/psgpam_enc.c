/* 
 * XXX TODO:
 * - add copyright notice (moveccr?)
 * - consider how psgpam_table.c should be built
 */
/*
 * PSGPAM, PSGPCM encoders
 * Function names are PSGPAM, but also used by PSGPCM.
 *
 * PSGPAM and PSGPCM internally use the unsigned type auint_t for
 * intermediate calculations to manage non-linear conversions.
 */

#include <sys/types.h>

#if defined(_KERNEL)
#include <sys/device.h>
#include <sys/audioio.h>
#include <dev/audio/audio_if.h>
#include <luna68k/dev/psgpam_enc.h>
#else
#include <stdint.h>
#include <stdlib.h>
#include "audio/userland.h"
#include "psgpam_enc.h"
#endif

#include "psgpam_table.c"

void
psgpam_init_context(struct psgpam_codecvar *ctx, u_int sample_rate)
{

	ctx->offset = 65535;
	ctx->sample_rate = sample_rate;
	ctx->expire_initial = sample_rate / 10;
	ctx->expire = ctx->expire_initial;
}

static inline auint_t
dynamic_offset(struct psgpam_codecvar *ctx, auint_t v)
{

	/*
	 * if (the passed value cannot be handled by current offset) {
	 *   update offset to handle the passed value
	 * } else {
	 *   increment offset
	 * }
	 */ 
	if (v <= ctx->offset) {
		ctx->offset = v;
	} else {
		if (--ctx->expire < 0) {
			ctx->offset += 1; 
			ctx->expire = ctx->expire_initial;
		}
	}
	return v - ctx->offset;
}

#define BULK(table) *d++ = table[v]

#define W8(table) *d++ = table[v]

#define W16(table) do {							\
	uint16_t t = (uint16_t)table[v];				\
	*d++ = ((t & 0xf0) << 4) | (t & 0x0f);				\
} while (0)

#define W32(table) do {							\
	uint32_t t = (uint32_t)table[v];				\
	*d++ = ((t & 0xf000) << 12)					\
	     | ((t & 0x0f00) <<  8)					\
	     | ((t & 0x00f0) <<  4)					\
	     | ((t & 0x000f));						\
} while (0)

#define SPLIT3(table) do {						\
	uint16_t t = (uint16_t)table[v];				\
	*d++ = ((t & 0xf000) >> 12);					\
	*d++ = ((t & 0x0f00) >>  8);					\
	*d++ = ((t & 0x000f));						\
} while (0)

#define ENCODER_DEFINE(enc, TT, table, writer) 				\
void									\
psgpam_aint_to_##enc(audio_filter_arg_t *arg)				\
{									\
	const aint_t *s = arg->src;					\
	TT *d = arg->dst;						\
									\
	for (int i = 0; i < arg->count; i++) {				\
		auint_t v = (*s++) ^ AINT_T_MIN;			\
		v >>= (AUDIO_INTERNAL_BITS - table##_BITS);		\
		writer(table);						\
	}								\
}

#define ENCODER_D_DEFINE(enc, TT, table, writer)			\
void									\
psgpam_aint_to_##enc##_d(audio_filter_arg_t *arg)			\
{									\
	const aint_t *s = arg->src;					\
	TT *d = arg->dst;						\
									\
	for (int i = 0; i < arg->count; i++) {				\
		auint_t v = (*s++) ^ AINT_T_MIN;			\
		v >>= (AUDIO_INTERNAL_BITS - table##_BITS);		\
		v = dynamic_offset(arg->context, v);			\
		writer(table);						\
	}								\
}

ENCODER_DEFINE(pam2a, uint16_t, PAM2A_TABLE, W16)
ENCODER_DEFINE(pam2b, uint16_t, PAM2B_TABLE, W16)
ENCODER_DEFINE(pam3a, uint32_t, PAM3A_TABLE, W32)
ENCODER_DEFINE(pam3b, uint32_t, PAM3B_TABLE, W32)
ENCODER_DEFINE(pcm1, uint8_t,  PCM1_TABLE, W8)
ENCODER_DEFINE(pcm2, uint16_t, PCM2_TABLE, W16)
ENCODER_DEFINE(pcm3, uint8_t,  PCM3_TABLE, SPLIT3)

ENCODER_D_DEFINE(pam2a, uint16_t, PAM2A_TABLE, W16)
ENCODER_D_DEFINE(pam2b, uint16_t, PAM2B_TABLE, W16)
ENCODER_D_DEFINE(pam3a, uint32_t, PAM3A_TABLE, W32)
ENCODER_D_DEFINE(pam3b, uint32_t, PAM3B_TABLE, W32)
ENCODER_D_DEFINE(pcm1, uint8_t,  PCM1_TABLE, W8)
ENCODER_D_DEFINE(pcm2, uint16_t, PCM2_TABLE, W16)
ENCODER_D_DEFINE(pcm3, uint8_t,  PCM3_TABLE, SPLIT3)
