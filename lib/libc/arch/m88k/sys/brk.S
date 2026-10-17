/*	$OpenBSD: brk.S,v 1.8 2005/08/07 16:40:15 espie Exp $	*/

/*-
 * Copyright (c) 1990 The Regents of the University of California.
 * All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * the Systems Programming Group of the University of Utah Computer
 * Science Department.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include "SYS.h"

#ifdef WEAK_ALIAS
WEAK_ALIAS(brk, _brk)
#endif

	.globl	_end
	.globl	__curbrk

	.data
	.align 8
ASGLOBAL(__minbrk)
	.long	_end

ENTRY(_brk)
#ifdef PIC
/* __minbrk/curbrk belong to libc, not to the caller's GOT. */
	or	r10,r1,0
	or.u	r11,r0,hi16(.Lgot_brk#abdiff)
	bsr.n	.Lgot_brk
	 or	r11,r11,lo16(.Lgot_brk#abdiff)
.Lgot_brk:	addu	r11,r11,r1
	or	r1,r10,0
	or.u	r5,r0,hi16(__minbrk#got_rel)
	or	r5,r5,lo16(__minbrk#got_rel)
	ld	r5,r11,r5
	ld	r5,r5,0
#else
	or.u	r5,r0,hi16(__minbrk)
	ld	r5,r5,lo16(__minbrk)
#endif
	cmp	r3,r5,r2
	bb1	ls,r3,1f	/* r2 should be > __minbrk; can't go below end */
	or	r2,r5,0		/* otherwise, set r2 to __minbrk */
1:
	or	r4,r2,0
	or	r13,r0,__SYSCALLNAME(SYS_,break)
	tb0	0,r0,128
	br	CERROR
#ifdef PIC
	or.u	r5,r0,hi16(__curbrk#got_rel)
	or	r5,r5,lo16(__curbrk#got_rel)
	ld	r5,r11,r5
	st	r4,r5,0
#else
	or.u	r5,r0,hi16(__curbrk)
	st	r4,r5,lo16(__curbrk)
#endif
	or	r2,r0,0		/* clear r2/r3 to indicate success */
	jmp.n	r1
	 or	r3,r0,0
