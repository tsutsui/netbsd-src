/*	$NetBSD: __clone.S,v 1.2 2002/07/30 06:07:59 matt Exp $	*/

/*-
 * Copyright (c) 2001 Tsubai Masanari.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/errno.h>
#include "SYS.h"

#ifdef WEAK_ALIAS
WEAK_ALIAS(clone, __clone)
#endif

/*
 * int __clone(int (*fn)(void *), void *stack, int flags, void *arg);
 */
ENTRY(__clone)
	/*
	 * Sanity checks: func and stack may not be NULL.
	 */
	bcnd	eq0,r2,2f
	bcnd	eq0,r3,2f

	or	r6,r0,r2		/* Save fn in r6 */
	or	r2,r0,r4
	or	r13,r0,SYS___clone	/* (flags, stack) */
	tb0	0,r0,128
	br	_C_LABEL(_cerror)	/* error... */

	bcnd	ne0,r2,1f
	jsr	r1		/* We're the parent, just return. */

1:
	jsr.n	r6		/* Call the clone's entry point. */
	 or	r2,r0,r5	/* arg */

	bsr	_C_LABEL(_exit)

2:
	or	r2,r0,EINVAL
	br	_C_LABEL(_cerror)
