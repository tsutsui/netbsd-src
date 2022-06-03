#pragma once

#define XP_ATN_RELOAD	0x80808080
#define XP_ATN_STAT	0x40404040
#define XP_ATN_RESET	0x20202020

struct psgpam_codecvar {
	int expire;		/* min expire counter */
	int expire_initial;	/* expire initial (as 10Hz) */
	auint_t offset;		/* dynamic offset */

	u_int sample_rate;
};

void psgpam_init_context(struct psgpam_codecvar *ctx,
 u_int sample_rate);

void psgpam_aint_to_pam2a(audio_filter_arg_t *arg);
void psgpam_aint_to_pam2b(audio_filter_arg_t *arg);
void psgpam_aint_to_pam3a(audio_filter_arg_t *arg);
void psgpam_aint_to_pam3b(audio_filter_arg_t *arg);
void psgpam_aint_to_pcm1(audio_filter_arg_t *arg);
void psgpam_aint_to_pcm2(audio_filter_arg_t *arg);
void psgpam_aint_to_pcm3(audio_filter_arg_t *arg);

void psgpam_aint_to_pam2a_d(audio_filter_arg_t *arg);
void psgpam_aint_to_pam2b_d(audio_filter_arg_t *arg);
void psgpam_aint_to_pam3a_d(audio_filter_arg_t *arg);
void psgpam_aint_to_pam3b_d(audio_filter_arg_t *arg);
void psgpam_aint_to_pcm1_d(audio_filter_arg_t *arg);
void psgpam_aint_to_pcm2_d(audio_filter_arg_t *arg);
void psgpam_aint_to_pcm3_d(audio_filter_arg_t *arg);

