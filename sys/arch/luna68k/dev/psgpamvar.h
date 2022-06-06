/*	$NetBSD$	*/

/*
 * Copyright (c) 2018 Yosuke Sugahara. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#define XPENC_PCM1	0
#define XPENC_PCM2	1
#define XPENC_PCM3	2

#define XPENC_PAM2_0	3
#define XPENC_PAM2_9	4
#define XPENC_PAM2_12	5
#define XPENC_PAM2_19	6

#define XPENC_PAM3_0	7
#define XPENC_PAM3_12	8

#define XPENC_PAM4_12	9

#define XPENC_MAX	10

#define XP_VAR_BASE     0x0100
#define XP_MAGIC        (XP_VAR_BASE + 0)

#define XP_CMD_START    (XP_VAR_BASE + 8)
#define XP_TIMER        (XP_VAR_BASE + 9)
#define XP_ENC          (XP_VAR_BASE + 10)

/* word */
#define XP_BUFTOP       (XP_VAR_BASE + 12)

#define XP_STAT_READY   (XP_VAR_BASE + 14)
#define XP_STAT_ERROR   (XP_VAR_BASE + 15)
#define XP_STAT_PLAY    (XP_VAR_BASE + 16)
#define XP_STAT_RESET   (XP_VAR_BASE + 17)
/* word */
#define XP_STAT_PTR     (XP_VAR_BASE + 18)

#define XP_CPU_FREQ	6144000
