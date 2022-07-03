/*	$NetBSD: mcontext.h,v 1.3 2003/10/08 22:43:01 thorpej Exp $	*/

/*-
* Copyright (c) 2001 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Klaus Klein.
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

#ifndef _M88K_MCONTEXT_H_
#define _M88K_MCONTEXT_H_

/*
 * General register state (important: 0-31 maps to `struct reg')
 */
#define _NGREG		36	/* 0-31, EPSR, FPSR, FPCR, PC */
typedef	long		__greg_t;
typedef	__greg_t	__gregset_t[_NGREG];

/* Convenience synonyms */
#define	_REG_R0		0
#define	_REG_R1		1
#define	_REG_R2		2
#define	_REG_R3		3
#define	_REG_R4		4
#define	_REG_R5		5
#define	_REG_R6		6
#define	_REG_R7		7
#define	_REG_R8		8
#define	_REG_R9		9
#define	_REG_R10	10
#define	_REG_R11	11
#define	_REG_R12	12
#define	_REG_R13	13
#define	_REG_R14	14
#define	_REG_R15	15
#define	_REG_R16	16
#define	_REG_R17	17
#define	_REG_R18	18
#define	_REG_R19	19
#define	_REG_R20	20
#define	_REG_R21	21
#define	_REG_R22	22
#define	_REG_R23	23
#define	_REG_R24	24
#define	_REG_R25	25
#define	_REG_R26	26
#define	_REG_R27	27
#define	_REG_R28	28
#define	_REG_R29	29
#define	_REG_R30	30
#define	_REG_R31	31
#define _REG_PSR	32
#define _REG_FPSR	33
#define _REG_FPCR	34
#define _REG_PC		35

/* XXX probably want to include SNIP and SFIP in mcontext.  See also pthread_md.h.  -TKM */

typedef struct {
    __gregset_t		__gregs;
} mcontext_t;

/* Machine-dependent uc_flags - None */

#define _UC_MACHINE_SP(uc)	((uc)->uc_mcontext.__gregs[_REG_R31])
#define _UC_MACHINE_PC(uc)	((uc)->uc_mcontext.__gregs[_REG_PC])
#define _UC_MACHINE_INTRV(uc)	((uc)->uc_mcontext.__gregs[_REG_R2])

#define	_UC_MACHINE_SET_PC(uc, pc)	_UC_MACHINE_PC(uc) = (pc)

#endif	/* !_M88K_MCONTEXT_H_ */
