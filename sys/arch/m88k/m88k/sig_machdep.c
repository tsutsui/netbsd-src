/*	$OpenBSD: sig_machdep.c,v 1.4 2006/01/02 19:46:12 miod Exp $	*/
/*
 * Copyright (c) 1998, 1999, 2000, 2001 Steve Murphree, Jr.
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
/*
 * Mach Operating System
 * Copyright (c) 1993-1991 Carnegie Mellon University
 * Copyright (c) 1991 OMRON Corporation
 * All Rights Reserved.
 *
 * Permission to use, copy, modify and distribute this software and its
 * documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 *
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/proc.h>
#include <sys/user.h>
#include <sys/ras.h>
#include <sys/sa.h>
#include <sys/savar.h>
#include <sys/signal.h>
#include <sys/signalvar.h>
#include <sys/ucontext.h>

#include <sys/mount.h>
#include <sys/syscallargs.h>

#include <machine/cpu.h>
#include <machine/reg.h>
#include <machine/frame.h>

#ifdef DEBUG
int sigdebug = 0;
int sigpid = 0;
#define SDB_FOLLOW	0x01
#define SDB_KSTACK	0x02
#endif

static register_t
m88k_read_fpsr(void)
{
	register_t value;

	__asm__ __volatile__ ("fldcr %0,fcr62" : "=r" (value));
	return value;
}

static register_t
m88k_read_fpcr(void)
{
	register_t value;

	__asm__ __volatile__ ("fldcr %0,fcr63" : "=r" (value));
	return value;
}

static void
m88k_write_fpsr(register_t value)
{
	__asm__ __volatile__ ("fstcr %0,fcr62" : : "r" (value));
}

static void
m88k_write_fpcr(register_t value)
{
	__asm__ __volatile__ ("fstcr %0,fcr63" : : "r" (value));
}

void *
getframe(const struct lwp *l, int sig, int *onstack)
{
	void		   *frame;
	struct proc	   *p = l->l_proc;
	struct trapframe  *tf = l->l_md.md_tf;

	/* Do we need to jump onto the signal stack? */
	*onstack =
	    (p->p_sigctx.ps_sigstk.ss_flags & (SS_DISABLE | SS_ONSTACK)) == 0 &&
	    (SIGACTION(p, sig).sa_flags & SA_ONSTACK) != 0;

	if (*onstack)
		frame = (char *)p->p_sigctx.ps_sigstk.ss_sp + p->p_sigctx.ps_sigstk.ss_size;
	else
		frame = (void *)tf->tf_sp;

	return frame;
}

void
buildcontext(struct lwp *l, const void *call, const void *ra, const void *sp)
{
	struct trapframe *tf = l->l_md.md_tf;

	tf->tf_r[_REG_R1] = (register_t)ra;

#ifdef M88100
	if (CPU_IS88100) {
		tf->tf_snip = ((register_t)call & NIP_ADDR) | NIP_V;
		tf->tf_sfip = (tf->tf_snip + 4) | FIP_V;
	}
#endif
#ifdef M88110
	if (CPU_IS88110) {
		tf->tf_exip = ((register_t)call & XIP_ADDR);
	}
#endif
	tf->tf_sp = (register_t)sp;
}

/*
 * Send an interrupt to process.
 */
void
sendsig(const ksiginfo_t *ksi, const sigset_t *mask)
{
	struct sigframe_siginfo sf;
	struct lwp *l = curlwp;
	struct proc *p = l->l_proc;
	struct sigacts *ps = p->p_sigacts;
	struct trapframe *tf = l->l_md.md_tf;
	int onstack, sig = ksi->ksi_signo;
	struct sigframe_siginfo *fp = getframe(l, sig, &onstack);
	sig_t catcher = SIGACTION(p, ksi->ksi_signo).sa_handler;

	fp--;

	/* The embedded ucontext requires 16-byte alignment. */
	if (((vaddr_t)fp & 0x0f) != 0)
		fp = (struct sigframe_siginfo *)((vaddr_t)fp & ~0x0f);

	/* Build stack frame for signal trampoline. */
	switch (ps->sa_sigdesc[sig].sd_vers) {
	case 0:		/* handled by sendsig_sigcontext */
	case 1:		/* handled by sendsig_sigcontext */
	default:	/* unknown version */
		printf("nsendsig: bad version %d\n",
		    ps->sa_sigdesc[sig].sd_vers);
		sigexit(l, SIGILL);
	case 2:
		break;
	}

#ifdef DEBUG
	if ((sigdebug & SDB_KSTACK) && p->p_pid == sigpid)
		printf("sendsig_siginfo(%d): sig %d ssp %p usp %p\n", p->p_pid,
		    sig, &onstack, fp);
#endif

	/* Build stack frame for signal trampoline. */

	sf.sf_si._info = ksi->ksi_info;
	sf.sf_uc.uc_flags = _UC_SIGMASK;
	sf.sf_uc.uc_sigmask = *mask;
	sf.sf_uc.uc_link = NULL;
	memset(&sf.sf_uc.uc_stack, 0, sizeof(sf.sf_uc.uc_stack));
	cpu_getmcontext(l, &sf.sf_uc.uc_mcontext, &sf.sf_uc.uc_flags);

	if (copyout((caddr_t)&sf, (caddr_t)fp, sizeof sf) != 0) {
		/*
		 * Process has trashed its stack; give it an illegal
		 * instruction to halt it in its tracks.
		 */
#ifdef DEBUG
		if ((sigdebug & SDB_KSTACK) && p->p_pid == sigpid)
			printf("sendsig_siginfo(%d): copyout failed on sig %d\n",
			    p->p_pid, sig);
#endif
		sigexit(l, SIGILL);
		/* NOTREACHED */
	}

#ifdef DEBUG
	if (sigdebug & SDB_FOLLOW)
		printf("sendsig_siginfo(%d): sig %d usp %p code %x\n",
		       p->p_pid, sig, fp, ksi->ksi_code);
#endif

	/*
	 * Set up registers for the signal handler invocation.
	 */
	tf->tf_r[_REG_R2] = sig;
	tf->tf_r[_REG_R3] = (register_t)&fp->sf_si;
	tf->tf_r[_REG_R4] = (register_t)&fp->sf_uc;

	buildcontext(l,catcher,ps->sa_sigdesc[sig].sd_tramp,fp);

	/* Remember that we're now on the signal stack. */
	if (onstack)
		p->p_sigctx.ps_sigstk.ss_flags |= SS_ONSTACK;

#ifdef DEBUG
	if ((sigdebug & SDB_FOLLOW) ||
	    ((sigdebug & SDB_KSTACK) && p->p_pid == sigpid))
		printf("sendsig(%d): sig %d returns\n", p->p_pid, sig);
#endif
}

void 
cpu_upcall(struct lwp *l, int type, int nevents, int ninterrupted, void *sas, void *ap, void *sp, sa_upcall_t upcall)
{
       	struct trapframe *tf = l->l_md.md_tf;

	tf->tf_r[_REG_R2] = type;
	tf->tf_r[_REG_R3] = (register_t)sas;
	tf->tf_r[_REG_R4] = nevents;
	tf->tf_r[_REG_R5] = ninterrupted;
	tf->tf_r[_REG_R6] = (register_t)ap;

	buildcontext(l, (void*)upcall, 0, sp);
}

void
cpu_getmcontext(l, mcp, flags)
	struct lwp *l;
	mcontext_t *mcp;
	unsigned int *flags;
{
	struct trapframe *tf = l->l_md.md_tf;
	__greg_t *gr = mcp->__gregs;
	int i;

	memset(mcp, 0, sizeof(*mcp));
	for (i = 0; i < 32; i++)
		gr[i] = tf->tf_r[i];
	gr[_REG_PSR] = tf->tf_epsr;
	gr[_REG_FPSR] = m88k_read_fpsr();
	gr[_REG_FPCR] = m88k_read_fpcr();

#ifdef M88100
	if (CPU_IS88100) {
		gr[_REG_PC] = tf->tf_snip & NIP_ADDR;
		gr[_REG_nPC] = tf->tf_sfip & FIP_ADDR;
	}
#endif
#ifdef M88110
	if (CPU_IS88110) {
		register_t pc = tf->tf_exip & XIP_ADDR;

		gr[_REG_PC] = pc;
		gr[_REG_nPC] = (tf->tf_exip & XIP_E) ?
		    (tf->tf_enip & XIP_ADDR) : pc + 4;
	}
#endif

	*flags |= _UC_CPU;
}

int
cpu_setmcontext(l, mcp, flags)
	struct lwp *l;
	const mcontext_t *mcp;
	unsigned int flags;
{
	struct trapframe *tf = l->l_md.md_tf;
	const __greg_t *gr = mcp->__gregs;
	register_t pc, npc, epsr, fpsr, fpcr;
#ifdef M88100
	register_t new_sxip = 0, new_snip = 0, new_sfip = 0;
#endif
#ifdef M88110
	register_t new_exip = 0, new_enip = 0;
#endif
	int i;

	if ((flags & _UC_CPU) == 0)
		return 0;

	/* Phase A: validate and prepare the return pipeline in local state. */
	pc = gr[_REG_PC];
	npc = gr[_REG_nPC];
	if (((pc | npc) & 3) != 0)
		return EINVAL;
	epsr = gr[_REG_PSR];
	if (((epsr ^ tf->tf_epsr) & PSR_USERSTATIC) != 0)
		return EINVAL;
	fpsr = gr[_REG_FPSR];
	fpcr = gr[_REG_FPCR];

#ifdef M88100
	if (CPU_IS88100) {
		new_sxip = 0;
		new_snip = (pc & NIP_ADDR) | NIP_V;
		new_sfip = (npc & FIP_ADDR) | FIP_V;
	}
#endif
#ifdef M88110
	if (CPU_IS88110) {
		new_enip = npc & XIP_ADDR;
		new_exip = (pc & XIP_ADDR) |
		    ((npc == pc + 4) ? 0 : XIP_E);
	}
#endif

	/* Phase B: the remaining operations cannot fail. */
	tf->tf_r[0] = 0;
	for (i = 1; i < 32; i++)
		tf->tf_r[i] = gr[i];
	tf->tf_epsr = epsr;
	tf->tf_fpsr = fpsr;
	tf->tf_fpcr = fpcr;
#ifdef M88100
	if (CPU_IS88100) {
		tf->tf_sxip = new_sxip;
		tf->tf_snip = new_snip;
		tf->tf_sfip = new_sfip;
	}
#endif
#ifdef M88110
	if (CPU_IS88110) {
		tf->tf_enip = new_enip;
		tf->tf_exip = new_exip;
	}
#endif
	m88k_write_fpsr(fpsr);
	m88k_write_fpcr(fpcr);

	return 0;
}

#if 0  /* XXX not needed? -TKM */
/*
 * System call to cleanup state after a signal has been taken.  Reset signal
 * mask and stack state from context left by sendsig (above).  Return to
 * previous pc and psl as specified by context left by sendsig.  Check
 * carefully to make sure that the user has not modified the psl to gain
 * improper privileges or to cause a machine fault.
 */

/* ARGSUSED */
int
sys_sigreturn(struct proc *p, void *v, register_t *retval)
{
	struct sys_sigreturn_args /* {
	   syscallarg(struct sigcontext *) sigcntxp;
	} */ *uap = v;
	struct sigcontext *scp;
	struct trapframe *tf;
	struct sigcontext ksc;

	scp = (struct sigcontext *)SCARG(uap, sigcntxp);
#ifdef DEBUG
	if (sigdebug & SDB_FOLLOW)
		printf("sigreturn: pid %d, scp %x\n", p->p_pid, scp);
#endif
	if (((vaddr_t)scp & 3) != 0 ||
	    copyin((caddr_t)scp, (caddr_t)&ksc, sizeof(struct sigcontext)))
		return (EINVAL);

	tf = p->p_md.md_tf;
	scp = &ksc;

	bcopy((const void *)&scp->sc_regs, (caddr_t)&tf->tf_regs,
	    sizeof(scp->sc_regs));

	/*
	 * Restore the user supplied information
	 */
	if (scp->sc_onstack & SS_ONSTACK)
		p->p_sigacts->ps_sigstk.ss_flags |= SS_ONSTACK;
	else
		p->p_sigacts->ps_sigstk.ss_flags &= ~SS_ONSTACK;
	p->p_sigmask = scp->sc_mask & ~sigcantmask;

	return (EJUSTRETURN);
}
#endif
