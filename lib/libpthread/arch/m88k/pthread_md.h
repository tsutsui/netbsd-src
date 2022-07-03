/*	$NetBSD: pthread_md.h,v 1.2 2003/01/18 10:34:17 thorpej Exp $	*/

/*-
 * Copyright (c) 2001 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Nathan J. Williams.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *        This product includes software developed by the NetBSD
 *        Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _LIB_PTHREAD_M88K_MD_H
#define _LIB_PTHREAD_M88K_MD_H

static __inline long
pthread__sp(void)
{
	long ret;

	__asm("or %0,r0,r31" : "=r" (ret));

	return ret;
}

#define pthread__uc_sp(ucp) ((ucp)->uc_mcontext.__gregs[_REG_R31])
#define pthread__uc_pc(ucp) ((ucp)->uc_mcontext.__gregs[_REG_PC])

/*
 * Usable stack space below the ucontext_t. 
 * See comment in pthread_switch.S about STACK_SWITCH.
 */
#define STACKSPACE      12      /* room for 3 integer values */

/*
 * Set initial, sane values for registers whose values aren't just
 * "don't care".
 */
#define _INITCONTEXT_U_MD(ucp)						\
	(ucp)->uc_mcontext.__gregs[_REG_PSR] = 0x0000;

/*
 * Conversions between struct reg and struct mcontext. Used by
 * libpthread_dbg.
 */

/* XXX TKM - How should lack of FP regs be handled? */
#define fpreg reg

#define PTHREAD_UCONTEXT_TO_REG(reg, uc) do {				\
	memcpy(&(reg)->r, &(uc)->uc_mcontext.__gregs,			\
		sizeof(__gregset_t));					\
	} while (/*CONSTCOND*/0)

#define PTHREAD_REG_TO_UCONTEXT(uc, reg) do {				\
	memcpy(&(uc)->uc_mcontext.__gregs, &(reg)->r,			\
		sizeof(__gregset_t));					\
	(uc)->uc_flags = ((uc)->uc_flags | _UC_CPU) & ~_UC_USER;       	\
	} while (/*CONSTCOND*/0)

#define PTHREAD_UCONTEXT_TO_FPREG(freg, uc)       			\
	memcpy(&(freg)->r, &(uc)->uc_mcontext.__gregs,			\
	    sizeof(struct fpreg))					\

#define PTHREAD_FPREG_TO_UCONTEXT(uc, freg) do {       	       		\
	memcpy(&(uc)->uc_mcontext.__gregs, &(freg)->r,			\
	    sizeof(struct fpreg));					\
	(uc)->uc_flags = ((uc)->uc_flags | _UC_FPU) & ~_UC_USER;       	\
	} while (/*CONSTCOND*/0)

#endif /* _LIB_PTHREAD_M88K_MD_H */
