/*	$OpenBSD: vm_machdep.c,v 1.14 2007/05/27 20:59:25 miod Exp $	*/

/*
 * Copyright (c) 1998 Steve Murphree, Jr.
 * Copyright (c) 1996 Nivas Madhur
 * Copyright (c) 1993 Adam Glass
 * Copyright (c) 1988 University of Utah.
 * Copyright (c) 1982, 1986, 1990 The Regents of the University of California.
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
 *
 *	from: Utah $Hdr: vm_machdep.c 1.21 91/04/06$
 *	from: @(#)vm_machdep.c	7.10 (Berkeley) 5/7/91
 *	vm_machdep.c,v 1.3 1993/07/07 07:09:32 cgd Exp
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/sa.h>
#include <sys/savar.h>
#include <sys/signalvar.h>
#include <sys/malloc.h>
#include <sys/buf.h>
#include <sys/user.h>
#include <sys/vnode.h>
#include <sys/core.h>
#include <sys/exec.h>
#include <sys/ptrace.h>

#include <uvm/uvm_extern.h>

#include <machine/mmu.h>
#include <machine/cmmu.h>
#include <machine/cpu.h>
#include <machine/frame.h>
#include <machine/trap.h>

#include <m88k/userret.h>

/*
 * Finish a fork operation, with process l2 nearly set up.
 * Copy and update the kernel stack and pcb, making the child
 * ready to run, and marking it so that it can return differently
 * than the parent.  Returns 1 in the child process, 0 in the parent.
 * We currently double-map the user area so that the stack is at the same
 * address in each process; in the future we will probably relocate
 * the frame pointers on the stack after copying.
 */

/* XXX - why do we need both switchframe and ksigframe?  ksigframe seems redundant. -TKM */
void
cpu_lwp_fork(struct lwp *l1, struct lwp *l2, void *stack, size_t stacksize,
    void (*func)(void *), void *arg)
{
	struct proctrampframe *l2ptf;
	extern void proc_trampoline(void);

	/* Copy pcb from l1 to l2. */
	if (l1 == curlwp) {
		/* Sync the PCB before we copy it. */
		savectx(&curpcb->pcb_sf);
	}
#ifdef DIAGNOSTIC
	else if (l1 != &lwp0)
		panic("cpu_lwp_fork: curproc");
#endif

	bcopy(&l1->l_addr->u_pcb, &l2->l_addr->u_pcb, sizeof(struct pcb));
	l2->l_md.md_tf = (struct trapframe *)USER_REGS(l2);

	/*
	 * Allocate a trampoline frame on the new l2 kernel stack.
	 */
	l2ptf = (struct proctrampframe *)((char *)l2->l_addr + USPACE - 8) - 1;
	l2->l_addr->u_pcb.kernel_state.pcb_sp = (u_int)l2ptf;

	/*
	 * If specified, give the child a different stack.
	 */
	if (stack != NULL)
		USER_REGS(l2)->r[31] = (u_int)stack + stacksize;

	/*
	 * When this process resumes, r31 will be l2ptf and
	 * the process will be at the beginning of proc_trampoline().
	 * proc_trampoline will execute the function func, pop off
	 * l2ptf frame, and then call proc_do_uret.
	 */
	l2->l_addr->u_pcb.kernel_state.pcb_pc = (u_int)proc_trampoline;

	l2ptf->ptf_func	= func;
	l2ptf->ptf_arg	= arg;
	l2ptf->ptf_ret	= proc_do_uret;
	l2ptf->ptf_lwp	= l2;
}

/*
 * Start a new LWP
 */
void
startlwp(arg)
        void *arg;
{
        int err;
        ucontext_t *uc = arg;
        struct lwp *l = curlwp;

        err = cpu_setmcontext(l, &uc->uc_mcontext, uc->uc_flags);
#if DIAGNOSTIC
        if (err) {
                printf("Error %d from cpu_setmcontext.", err);
        }
#endif

        pool_put(&lwp_uc_pool, uc);

        userret(l);
}

/*
 * XXX This is a terrible name.
 */
void
upcallret(struct lwp *l)
{
        KERNEL_PROC_UNLOCK(l);

        userret(l);
}

void
cpu_setfunc(struct lwp *l, void (*func)(void *), void *arg)
{
	struct proctrampframe *ptf;
	extern void proc_trampoline(void);

	/*
	 * Allocate a trampoline frame on the kernel stack.
	 */
	ptf = (struct proctrampframe *)l->l_addr->u_pcb.kernel_state.pcb_sp - 1;
	l->l_addr->u_pcb.kernel_state.pcb_sp = (u_int)ptf;

	/*
	 * When this process resumes, r31 will be l2ptf and
	 * the process will be at the beginning of proc_trampoline().
	 * proc_trampoline will execute the function func, pop off
	 * l2ptf frame, and then call proc_do_uret.
	 */
	l->l_addr->u_pcb.kernel_state.pcb_pc = (u_int)proc_trampoline;

	ptf->ptf_func	= func;
	ptf->ptf_arg	= arg;
	ptf->ptf_ret	= proc_do_uret;
	ptf->ptf_lwp	= l;
}

void
cpu_lwp_free(struct lwp *l, int proc)
{
}

/*
 * cpu_exit is called as the last action during exit.
 * We release the address space and machine-dependent resources,
 * including the memory for the user structure and kernel stack.
 * Once finished, we call switch_exit, which switches to a temporary
 * pcb and stack and never returns.  We block memory allocation
 * until switch_exit has made things safe again.
 */
void
cpu_exit(struct lwp *l)
{
	int s;

	void switch_exit(struct lwp *, void (*)(struct lwp *));

 	SCHED_LOCK(s);			/* expected by cpu_switch */
 	switch_exit(l, lwp_exit2);
	/* NOTREACHED */
}

/*
 * Dump the machine specific header information at the start of a core dump.
 */
int
cpu_coredump(struct lwp *l, struct vnode *vp, struct ucred *cred,
    struct core *chdr)
{
	struct reg reg;
	struct coreseg cseg;
	int error;

	CORE_SETMAGIC(*chdr, COREMAGIC, MID_MACHINE, 0);
	chdr->c_hdrsize = ALIGN(sizeof(*chdr));
	chdr->c_seghdrsize = ALIGN(sizeof(cseg));
	chdr->c_cpusize = sizeof(reg);

	/* Save registers. */
	error = process_read_regs(l, &reg);
	if (error)
		return error;

	CORE_SETMAGIC(cseg, CORESEGMAGIC, MID_MACHINE, CORE_CPU);
	cseg.c_addr = 0;
	cseg.c_size = chdr->c_cpusize;

	error = vn_rdwr(UIO_WRITE, vp, (caddr_t)&cseg, chdr->c_seghdrsize,
	    (off_t)chdr->c_hdrsize, UIO_SYSSPACE, IO_NODELOCKED|IO_UNIT, cred,
	    NULL, l->l_proc);
	if (error)
		return error;

	error = vn_rdwr(UIO_WRITE, vp, (caddr_t)&reg, sizeof(reg),
	    (off_t)(chdr->c_hdrsize + chdr->c_seghdrsize), UIO_SYSSPACE,
	    IO_NODELOCKED|IO_UNIT, cred, NULL, l->l_proc);
	if (error)
		return error;

	chdr->c_nseg++;
	return 0;
}

/*
 * Map an IO request into kernel virtual address space via phys_map.
 */
void
vmapbuf(bp, len)
	struct buf *bp;
	vsize_t len;
{
	caddr_t addr;
	vaddr_t ova, kva, off;
	paddr_t pa;
	struct pmap *pmap;
	u_int pg;

#ifdef DIAGNOSTIC
	if ((bp->b_flags & B_PHYS) == 0)
		panic("vmapbuf");
#endif

	addr = (caddr_t)trunc_page((vaddr_t)(bp->b_saveaddr = bp->b_data));
	off = (vaddr_t)bp->b_saveaddr & PGOFSET;
	len = round_page(off + len);
	pmap = vm_map_pmap(&bp->b_proc->p_vmspace->vm_map);

	/*
	 * You may ask: Why phys_map? kernel_map should be OK - after all,
	 * we are mapping user va to kernel va or remapping some
	 * kernel va to another kernel va. The answer is TLB flushing
	 * when the address gets a new mapping.
	 */

	ova = kva = uvm_km_valloc_wait(phys_map, len);

	/*
	 * Flush the TLB for the range [kva, kva + off]. Strictly speaking,
	 * we should do this in vunmapbuf(), but we do it lazily here, when
	 * new pages get mapped in.
	 */

	cmmu_flush_tlb(cpu_number(), 1, kva, btoc(len));

	bp->b_data = (caddr_t)(kva + off);
	for (pg = atop(len); pg != 0; pg--) {
		if (pmap_extract(pmap, (vaddr_t)addr, &pa) == FALSE)
			panic("vmapbuf: null page frame");
		pmap_enter(vm_map_pmap(phys_map), kva, pa,
			   VM_PROT_READ | VM_PROT_WRITE,
			   VM_PROT_READ | VM_PROT_WRITE | PMAP_WIRED);
		addr += PAGE_SIZE;
		kva += PAGE_SIZE;
	}
	/* make sure snooping will be possible... */
	pmap_cache_ctrl(pmap_kernel(), ova, ova + len, CACHE_GLOBAL);
	pmap_update(pmap_kernel());
}

/*
 * Free the io map PTEs associated with this IO operation.
 * We also restore the original b_addr.
 */
void
vunmapbuf(bp, len)
	struct buf *bp;
	vsize_t len;
{
	vaddr_t addr, off;

#ifdef DIAGNOSTIC
	if ((bp->b_flags & B_PHYS) == 0)
		panic("vunmapbuf");
#endif

	addr = trunc_page((vaddr_t)bp->b_data);
	off = (vaddr_t)bp->b_data & PGOFSET;
	len = round_page(off + len);
	uvm_km_free_wakeup(phys_map, addr, len);
	bp->b_data = bp->b_saveaddr;
	bp->b_saveaddr = 0;
}
