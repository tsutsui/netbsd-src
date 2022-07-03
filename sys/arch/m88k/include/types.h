/*	$NetBSD: types.h,v 1.7 1995/07/05 17:46:11 pk Exp $ */
/*	$OpenBSD: types.h,v 1.3 2004/11/26 21:23:05 miod Exp $ */

/*
 * Copyright (c) 1992, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This software was developed by the Computer Systems Engineering group
 * at Lawrence Berkeley Laboratory under DARPA contract BG 91-66 and
 * contributed to Berkeley.
 *
 * All advertising materials mentioning features or use of this software
 * must display the following acknowledgement:
 *	This product includes software developed by the University of
 *	California, Lawrence Berkeley Laboratory.
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
 *
 *	@(#)types.h	8.1 (Berkeley) 6/11/93
 */

#ifndef	_M88K_TYPES_H_
#define	_M88K_TYPES_H_

#include <sys/cdefs.h>
#include <sys/featuretest.h>
#include <m88k/int_types.h>

#if defined(_KERNEL)
/*
 * Define register save area for kernel setjmp()/longjmp().
 * Reserve enough space for all callee-saved registers:
 *
 *  r1		(1)
 *  r14 - r31	(18)
 */
typedef struct label_t {
	int val[19];
} label_t;
#endif

/* NB: This should probably be if defined(_KERNEL) */
#if defined(_NETBSD_SOURCE)
typedef unsigned long	vaddr_t;
typedef unsigned long	paddr_t;
typedef unsigned long	vsize_t;
typedef unsigned long	psize_t;
#endif

typedef __uint32_t		register_t;

typedef __volatile unsigned int	__cpu_simple_lock_t;

/* do not change these - code below assumes r0 == __SIMPLELOCK_UNLOCKED */
#define	__SIMPLELOCK_LOCKED	1
#define	__SIMPLELOCK_UNLOCKED	0

#endif	/* _M88K_TYPES_H_ */
