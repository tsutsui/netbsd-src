/*	$NetBSD: frame.h,v 1.15 1997/05/03 12:49:05 mycroft Exp $	*/
/*	$OpenBSD: frame.h,v 1.3 2007/01/13 22:00:56 miod Exp $ */

/*
 * Copyright (c) 1982, 1990, 1993
 *	The Regents of the University of California.  All rights reserved.
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
 *
 * from: Utah $Hdr: frame.h 1.8 92/12/20$
 *
 *	@(#)frame.h	8.1 (Berkeley) 6/10/93
 */
/*
 * Copyright (c) 1996 Nivas Madhur
 * Mach Operating System
 * Copyright (c) 1993-1992 Carnegie Mellon University
 * All Rights Reserved.
 *
 * Permission to use, copy, modify and distribute this software and its
 * documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 *
 * CARNEGIE MELLON AND OMRON ALLOW FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON AND OMRON DISCLAIM ANY LIABILITY OF ANY KIND
 * FOR ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 *
 * Carnegie Mellon requests users of this software to return to
 *
 *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 *
 * any improvements or extensions that they make and grant Carnegie Mellon
 * the rights to redistribute these changes.
 */
/*
 * Copyright (c) 1988 University of Utah.
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
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the University of
 *	California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
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
 * from: Utah $Hdr: frame.h 1.8 92/12/20$
 *
 *	@(#)frame.h	8.1 (Berkeley) 6/10/93
 */

#ifndef	_M88K_FRAME_H_
#define	_M88K_FRAME_H_

#include <machine/reg.h>

/*
 * Our PCB is the regular PCB+Save area for kernel frame.
 * Upon entering kernel mode from user land, save the user context
 * in the saved_state area - this is passed as the exception frame.
 * On a context switch, only registers that need to be saved by the
 * C calling convention and few other regs (pc, psr etc) are saved
 * in the kernel_state part of the PCB. Typically, trap frames are
 * saved on the stack (by low level handlers or by hardware) but,
 * we just decided to do it in the PCB.
 */

struct m88100_pcb {
	register_t pcb_pc;	/* address to return */
	register_t pcb_ipl;
	register_t pcb_r14;
	register_t pcb_r15;
	register_t pcb_r16;
	register_t pcb_r17;
	register_t pcb_r18;
	register_t pcb_r19;
	register_t pcb_r20;
	register_t pcb_r21;
	register_t pcb_r22;
	register_t pcb_r23;
	register_t pcb_r24;
	register_t pcb_r25;
	register_t pcb_r26;
	register_t pcb_r27;
	register_t pcb_r28;
	register_t pcb_r29;
	register_t pcb_r30;
	register_t pcb_sp; 	/* kernel stack pointer */
	/* floating-point state */
	register_t pcb_fcr62;
	register_t pcb_fcr63;
};
#define switchframe m88100_pcb

/*
 * Motorola 88100 exception frame definitions
 *
 */
struct trapframe {
	struct reg	tf_regs;
	register_t	tf_vector;	/* exception vector number */
	register_t	tf_mask;	/* interrupt mask level */
	register_t	tf_mode;	/* interrupt mode */
	register_t	tf_scratch1;	/* reserved for use by locore */
	register_t	tf_ipfsr;	/* P BUS status */
	register_t	tf_dpfsr;	/* P BUS status */
	void		*tf_cpu;	/* cpu_info pointer */
};

#define	tf_r		tf_regs.r
#define	tf_sp		tf_regs.r[31]
#define	tf_epsr		tf_regs.epsr
#define	tf_fpsr		tf_regs.fpsr
#define	tf_fpcr		tf_regs.fpcr
#define	tf_sxip		tf_regs.sxip
#define	tf_snip		tf_regs.snip
#define	tf_sfip		tf_regs.sfip
#define	tf_exip		tf_regs.sxip
#define	tf_enip		tf_regs.snip
#define	tf_ssbr		tf_regs.ssbr
#define	tf_dmt0		tf_regs.dmt0
#define	tf_dmd0		tf_regs.dmd0
#define	tf_dma0		tf_regs.dma0
#define	tf_dmt1		tf_regs.dmt1
#define	tf_dmd1		tf_regs.dmd1
#define	tf_dma1		tf_regs.dma1
#define	tf_dmt2		tf_regs.dmt2
#define	tf_dmd2		tf_regs.dmd2
#define	tf_dma2		tf_regs.dma2
#define	tf_duap		tf_regs.ssbr
#define	tf_dsr		tf_regs.dmt0
#define	tf_dlar		tf_regs.dmd0
#define	tf_dpar		tf_regs.dma0
#define	tf_isr		tf_regs.dmt1
#define	tf_ilar		tf_regs.dmd1
#define	tf_ipar		tf_regs.dma1
#define	tf_isap		tf_regs.dmt2
#define	tf_dsap		tf_regs.dmd2
#define	tf_iuap		tf_regs.dma2
#define	tf_fpecr	tf_regs.fpecr
#define	tf_fphs1	tf_regs.fphs1
#define	tf_fpls1	tf_regs.fpls1
#define	tf_fphs2	tf_regs.fphs2
#define	tf_fpls2	tf_regs.fpls2
#define	tf_fppt		tf_regs.fppt
#define	tf_fprh		tf_regs.fprh
#define	tf_fprl		tf_regs.fprl
#define	tf_fpit		tf_regs.fpit

struct frame {
	struct trapframe  __packed F_t;
};

/*
 * proctrampframe - Frame used by proc_trampoline;
 * should be double word aligned.
 */
struct proctrampframe {
	void		(*ptf_func)(void *);
	void		*ptf_arg;
	void		(*ptf_ret)(struct lwp*);
	struct lwp	*ptf_lwp;
};

/*
 * WARNING: sigcode() in locore.s assumes the layout shown for sf_signo
 * through sf_handler so... don't screw with them!
 */
#if defined(COMPAT_16) && defined(_KERNEL)
struct sigframe_sigcontext {
	/*  r1 address of trampoline */
        /*  r2 signum for handler */
	/*  r3 code for handler */
	/*  r4 struct sigcontext for handler */
	struct sigcontext	 sf_sc;		/* actual context */
};
#endif

struct sigframe_siginfo {
	/*  r1 address of trampoline */
        /*  r2 signal number arg for handler */
	/*  r3 siginfo_t * arg for handler */
	/*  r4 ucontext_t * arg for handler */
	siginfo_t	sf_si;		/* actual saved siginfo */
	ucontext_t	sf_uc;		/* actual saved ucontext */
};

#ifdef _KERNEL
void *getframe(const struct lwp *, int, int *);
void buildcontext(struct lwp *, const void *, const void *, const void *);
#endif

#endif	/* _M88K_FRAME_H_ */
