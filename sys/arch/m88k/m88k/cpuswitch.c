/*	$NetBSD: cpuswitch.c,v 1.5.26.1 2006/03/10 13:17:33 tron Exp $	*/

/*
 * Copyright (c) 2000 Ben Harris.
 * Copyright (C) 1995, 1996 Wolfgang Solfrank.
 * Copyright (C) 1995, 1996 TooLs GmbH.
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
 *	This product includes software developed by TooLs GmbH.
 * 4. The name of TooLs GmbH may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY TOOLS GMBH ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TOOLS GMBH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Some additional routines that happened to be in locore.S traditionally,
 * but have no need to be coded in assembly.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: cpuswitch.c,v 1.5.26.1 2006/03/10 13:17:33 tron Exp $");

#include "opt_lockdebug.h"

#include <sys/param.h>
#include <sys/proc.h>
#include <sys/sched.h>
#include <sys/systm.h>
#include <sys/user.h>
#include <sys/ras.h>

#include <uvm/uvm_extern.h>

void idle(void);

/*
 * Idle
 */
void
idle()
{
#if defined(MULTIPROCESSOR) || defined(LOCKDEBUG)
	sched_unlock_idle();
#endif
	spl0();
	while (sched_whichqs == 0)
		continue;
	splhigh();
#if defined(MULTIPROCESSOR) || defined(LOCKDEBUG)
	sched_lock_idle();
#endif
}

/*
 * Find the highest-priority runnable process and switch to it.
 */
int
cpu_switch(struct lwp *l1, struct lwp *newl)
{
	int which;
	struct prochd *q;
	struct lwp *l2;
#if 0   /* XXX not yet implemented -TKM */
	struct proc *p2;
#endif
	struct switchframe dummy; 

	/*
	 * We enter here with interrupts blocked and sched_lock held.
	 */

#ifdef SCHEDDEBUG
	printf("cpu_switch: %p ->", l1);
#endif
	curlwp = NULL;
	curpcb = NULL;
	while (sched_whichqs == 0)
		idle();
	which = ffs(sched_whichqs) - 1;
	q = &sched_qs[which];
	l2 = q->ph_link;
	remrunqueue(l2);
	curcpu()->ci_want_resched = 0;
#ifdef SCHEDDEBUG
	printf(" %p\n", l2);
#endif
#if defined(MULTIPROCESSOR) || defined(LOCKDEBUG)
	sched_unlock_idle();
#endif
	/* p->p_cpu initialized in fork1() for single-processor */
	l2->l_stat = LSONPROC;
	curlwp = l2;
	curpcb = &curlwp->l_addr->u_pcb;

	if (l2 == l1)
		return (0);
	if (l1 != NULL)
		pmap_deactivate(l1);
	pmap_activate(l2);

#if 0   /* XXX not yet implemented -TKM */
	/* Check for Restartable Atomic Sequences. */
	p2 = l2->l_proc;
	if (!LIST_EMPTY(&p2->p_raslist)) {
		struct trapframe *tf = l2->l_addr->u_pcb.pcb_tf;
		caddr_t pc;

		pc = ras_lookup(p2, (caddr_t) tf->tf_pc);
		if (pc != (caddr_t) -1)
			tf->tf_pc = (register_t) pc;
	}
#endif

	cpu_loswitch(l1 ? &l1->l_addr->u_pcb.kernel_state : &dummy, 
	    &l2->l_addr->u_pcb.kernel_state);
	/* We only get back here after the other process has run. */
	return (1);
}

/*
 * Switch to the indicated lwp.
 */
void
cpu_switchto(struct lwp *old, struct lwp *new)
{

	/*
	 * We enter here with interrupts blocked and sched_lock held.
	 */

#ifdef SCHEDDEBUG
	printf("cpu_switchto: %p -> %p", old, new);
#endif
	curcpu()->ci_want_resched = 0;
#if defined(MULTIPROCESSOR) || defined(LOCKDEBUG)
	sched_unlock_idle();
#endif
	/* p->p_cpu initialized in fork1() for single-processor */
	new->l_stat = LSONPROC;
	curlwp = new;
	curpcb = &curlwp->l_addr->u_pcb;
	pmap_deactivate(old);
	pmap_activate(new);
	cpu_loswitch(&old->l_addr->u_pcb.kernel_state, &new->l_addr->u_pcb.kernel_state);
	/* We only get back here after the other process has run. */
}
