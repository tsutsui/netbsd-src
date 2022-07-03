/*	$OpenBSD: process.S,v 1.13 2005/12/11 21:45:30 miod Exp $	*/
/*
 * Copyright (c) 1996 Nivas Madhur
 * All rights reserved.
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
 *      This product includes software developed by Nivas Madhur.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "assym.h"
#include <sys/cdefs.h>
#include <machine/asm.h>
#include <machine/psl.h>
#include <machine/intr.h>

/*
 * switch_exit(struct lwp *l, void (*exit)(struct lwp *))
 *
 * Do the final work to exit from a process. After switching to the
 * idle stack and pcb, invoke lwp_exit2() on behalf of the exiting process,
 * then continue into cpu_switch() to select another process to run.
 */
ENTRY(switch_exit)
	/*
	 * Change pcb to idle u. area, i.e., set r31 to top of stack
	 * and set curpcb to point to the cpu's idle stack.
	 * r2 contains lwp *l.
	 */
	ldcr	r10, CPU
	ld	r30, r10, CI_IDLE_PCB
	addu	r31, r30, USIZE				/* now on idle stack */
	st	r30, r10, CI_CURPCB			/* curpcb = idle_pcb */

	/* Schedule the vmspace and stack to be freed. */
	jsr.n	r3					/* exit(l) */
	 st	r0, r10, CI_CURLWP			/* ci_curlwp = NULL */

	or	r2, r0, r0
	bsr.n	_C_LABEL(cpu_switch)	/* cpu_switch(NULL, NULL) */
	 or	r3, r0, r0

/*
 * Low-level context-switch operation.  cpu_switch() is in C -- this
 * just handles the bits that can't be done in C.
 *
 * void cpu_loswitch(struct switchframe *from, struct switchframe *to)
 */
ENTRY(cpu_loswitch)
	st	r1, r2, PCB_PC		/* save return address */
	bsr.n	_ASM_LABEL(__savectx)
	 stcr	r3,  SR2		/* save 'to' in SR2 */
	/* note that we don't need to recover r1 at this point */

	/* restore from the current context */
	ldcr	r10, SR2    /* restore 'to' */

	ld	r2,  r10, PCB_FCR62
	ld	r3,  r10, PCB_FCR63
	fstcr	r2,  fcr62
	fstcr	r3,  fcr63
	ld	r15, r10, PCB_R15
	ld	r16, r10, PCB_R16
	ld	r17, r10, PCB_R17
	ld	r18, r10, PCB_R18
	ld	r19, r10, PCB_R19
	ld	r20, r10, PCB_R20
	ld	r21, r10, PCB_R21
	ld	r22, r10, PCB_R22
	ld	r23, r10, PCB_R23
	ld	r24, r10, PCB_R24
	ld	r25, r10, PCB_R25
	ld	r26, r10, PCB_R26
	ld	r27, r10, PCB_R27
	ld	r28, r10, PCB_R28
	ld	r29, r10, PCB_R29
	or	r14, r10, r0		/* preserve curpcb in a register... */
	bsr.n	_C_LABEL(setipl)
	 ld	r2,  r10, PCB_IPL	/* restore interrupt mask */
	ld	r1,  r14, PCB_PC
	ld	r30, r14, PCB_R30	/* restore frame pointer & stack */
	ld	r31, r14, PCB_SP
	jmp.n	r1
	 ld	r14, r14, PCB_R14

/*
 * void savectx(struct switchframe *);
 * Update pcb, saving current processor state.
 */
ENTRY(savectx)
	/*
	 * Save preserved general register set.
	 */
	st	r1,  r2,  PCB_PC		/* save return address */
ASLOCAL(__savectx)
	st	r14, r2,  PCB_R14
	st	r15, r2,  PCB_R15
	st	r16, r2,  PCB_R16
	st	r17, r2,  PCB_R17
	st	r18, r2,  PCB_R18
	st	r19, r2,  PCB_R19
	st	r20, r2,  PCB_R20
	st	r21, r2,  PCB_R21
	st	r22, r2,  PCB_R22
	st	r23, r2,  PCB_R23
	st	r24, r2,  PCB_R24
	st	r25, r2,  PCB_R25
	st	r26, r2,  PCB_R26
	st	r27, r2,  PCB_R27
	st	r28, r2,  PCB_R28
	st	r29, r2,  PCB_R29
	st	r30, r2,  PCB_R30	/* save frame pointer & stack pointer */
	st	r31, r2,  PCB_SP

	/*
	 * Get the current spl.
	 * We need to save r1 on the stack because we don't know if we were
	 * called as savectx or __savectx.
	 */
	subu	r31, r31, 16		/* allocate stack for r1 and args */
	st	r1,  r31, 0
	bsr.n	_C_LABEL(getipl)	/* get the current interrupt mask */
	 or	r14, r0,  r2
	st	r2,  r14, PCB_IPL	/* save interrupt mask */
	ld	r1,  r31, 0		/* recover return address */
	addu	r31, r31, 16		/* put stack pointer back */

	/*
	 * Save FP state.
	 */
	fldcr	r2,  fcr62
	fldcr	r3,  fcr63
	st	r2,  r14, PCB_FCR62
	st	r3,  r14, PCB_FCR63
	jmp.n	r1
	 ld	r14, r14, PCB_R14	/* preserve r14 */
