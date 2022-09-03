/* $OpenBSD: machdep.c,v 1.184 2006/07/07 19:36:56 miod Exp $	*/
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

#include "opt_ddb.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/proc.h>
#include <sys/user.h>
#include <sys/buf.h>
#include <sys/reboot.h>
#include <sys/conf.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/mount.h>
#include <sys/msgbuf.h>
#include <sys/sa.h>
#include <sys/syscallargs.h>
#ifdef SYSVMSG
#include <sys/msg.h>
#endif
#include <sys/exec.h>
#include <sys/sysctl.h>
#include <sys/errno.h>
#include <sys/extent.h>
#include <sys/core.h>
#include <sys/kcore.h>

#include <machine/asm.h>
#include <machine/asm_macro.h>
#include <machine/bug.h>
#include <machine/bugio.h>
#include <machine/cmmu.h>
#include <machine/cpu.h>
#include <machine/kcore.h>
#include <machine/reg.h>

#include <mvme88k/dev/bootcons.h>

#include <uvm/uvm_extern.h>

#include "ksyms.h"
#if DDB
#include <machine/db_machdep.h>
#include <ddb/db_extern.h>
#include <ddb/db_interface.h>
#include <ddb/db_variables.h>
#endif /* DDB */

static int	getcpuspeed(struct mvmeprom_brdid *);
static void	identifycpu(void);
static int	cpu_dumpsize(void);
static u_long	cpu_dump_mempagecnt(void);
static int	cpu_dump(dev_type_dump((*dump)), daddr_t *blknop);

static void	dumpsys(void);

/* Called from m88k/m88k/eh_common.S: */
u_int	getipl(void);

/* Called from locore.S: */
void	secondary_main(void);
void	secondary_pre_main(void);
void	mvme_bootstrap(void);
void	mvme88k_vector_init(u_int32_t *, u_int32_t *);

/* Defined in locore.S: */
void	_doboot(void);

extern void	m187_bootstrap(void);
extern vaddr_t	m187_memsize(void);
extern void	m187_startup(void);
extern void	m188_bootstrap(void);
extern vaddr_t	m188_memsize(void);
extern void	m188_startup(void);
extern void	m197_bootstrap(void);
extern vaddr_t	m197_memsize(void);
extern void	m197_startup(void);

intrhand_t intr_handlers[NVMEINTR];

/* board dependent pointers */
void (*md_interrupt_func_ptr)(u_int, struct trapframe *);
u_int (*md_getipl)(void);
u_int (*md_setipl)(u_int);
u_int (*md_raiseipl)(u_int);
void (*md_init_clocks)(void);

int physmem;	  /* available physical memory, in pages */

struct vm_map *exec_map = NULL;
struct vm_map *mb_map = NULL;
struct vm_map *phys_map = NULL;

#ifdef MULTIPROCESSOR
__cpu_simple_lock_t cpu_mutex = __SIMPLELOCK_UNLOCKED;
#endif

/*
 * Info for CTL_HW
 */
char  machine[] = MACHINE;	 /* cpu "architecture" */
char  cpu_model[120];

#if defined(DDB) || NKSYMS > 0
extern char *ssym, *esym;
#endif

int boothowto;					/* set in locore.S */
int cputyp;					/* set in locore.S */
int brdtyp;					/* set in mvme_bootstrap() */
int cpumod;					/* set in mvme_bootstrap() */
int cpuspeed = 25;				/* safe guess */

#define ETHERPAGES 16
u_long ether_data_buff_size =  ETHERPAGES * PAGE_SIZE;
u_char	mvme_ea[6];				/* set in mvme_bootstrap() */

extern struct user *proc0paddr;

/*
 * Early console initialization: called early on from main, before vm init.
 * We want to stick to the BUG routines for now, and we'll switch to the
 * real console in cpu_startup().
 */
void
consinit(void)
{
#if defined(DDB)
	ddb_init((char*)esym - (char*)ssym, ssym, esym);
	if (boothowto & RB_KDB)
		Debugger();
#endif
}

int
getcpuspeed(struct mvmeprom_brdid *brdid)
{
	int speed = 0;
	u_int i, c;

	for (i = 0; i < 4; i++) {
		c = (u_int)brdid->speed[i];
		if (c == ' ')
			c = '0';
		else if (c > '9' || c < '0') {
			speed = 0;
			break;
		}
		speed = speed * 10 + (c - '0');
	}
	speed = speed / 100;

	switch (brdtyp) {
#ifdef MVME187
	case BRD_187:
	case BRD_8120:
		if (speed == 25 || speed == 33)
			return speed;
		speed = 25;
		break;
#endif
#ifdef MVME188
	case BRD_188:
		/*
		 * If BUG version prior to 5.x, there is no CNFG block and
		 * speed can be found in the environment.
		 * XXX We don't process ENV data yet - assume 20MHz in this
		 * case.
		 */
		if ((u_int)brdid->rev < 0x50) {
			speed = 20;
		} else {
			if (speed == 20 || speed == 25)
				return speed;
			speed = 25;
		}
		break;
#endif
#ifdef MVME197
	case BRD_197:
		if (speed == 40 || speed == 50)
			return speed;
		speed = 50;
		break;
#endif
	}

	/*
	 * If we end up here, the board information block is damaged and
	 * we can't trust it.
	 * Suppose we are running at the most common speed for our board,
	 * and hope for the best (this really only affects osiop).
	 */
	printf("WARNING: Board Configuration Data invalid, "
	    "replace NVRAM and restore values\n");
	return speed;
}

void
identifycpu(void)
{
	struct mvmeprom_brdid brdid;
	char suffix[4];
	u_int i;

	bzero(&brdid, sizeof(brdid));
	bugbrdid(&brdid);

	cpuspeed = getcpuspeed(&brdid);

	i = 0;
	if (brdid.suffix[0] >= ' ' && brdid.suffix[0] < 0x7f) {
		if (brdid.suffix[0] != '-')
			suffix[i++] = '-';
		suffix[i++] = brdid.suffix[0];
	}
	if (brdid.suffix[1] >= ' ' && brdid.suffix[1] < 0x7f)
		suffix[i++] = brdid.suffix[1];
	suffix[i++] = '\0';

	snprintf(cpu_model, sizeof cpu_model,
	    "Motorola MVME%x%s, %dMHz", brdtyp, suffix, cpuspeed);
}

/*
 * cpu_startup: allocate memory for variable-sized tables,
 * initialize CPU, and do autoconfiguration.
 */
void
cpu_startup(void)
{
	char pbuf[9];
	int i;
	vaddr_t minaddr, maxaddr;

	/*
	 * Initialize error message buffer (at end of core).
	 * avail_end was pre-decremented in mvme_bootstrap() to compensate.
	 */
	{
		for (i = 0; i < btoc(MSGBUFSIZE); i++)
			pmap_kenter_pa((paddr_t)msgbufp + i * PAGE_SIZE,
			    avail_end + i * PAGE_SIZE, VM_PROT_READ | VM_PROT_WRITE);
		pmap_update(pmap_kernel());
		initmsgbuf((caddr_t)msgbufp, round_page(MSGBUFSIZE));
	}

	/*
	 * Good {morning,afternoon,evening,night}.
	 */
	printf(version);
	identifycpu();
	format_bytes(pbuf, sizeof(pbuf), ctob(physmem));
	printf("total memory = %s\n", pbuf);

	/*
	 * Grab machine dependent memory spaces
	 */
	switch (brdtyp) {
#ifdef MVME187
	case BRD_187:
	case BRD_8120:
		m187_startup();
		break;
#endif
#ifdef MVME188
	case BRD_188:
		m188_startup();
		break;
#endif
#ifdef MVME197
	case BRD_197:
		m197_startup();
		break;
#endif
	}

	/*
	 * Allocate a submap for exec arguments.  This map effectively
	 * limits the number of processes exec'ing at any time.
	 */
	minaddr = 0;
	exec_map = uvm_km_suballoc(kernel_map, &minaddr, &maxaddr,
	    16 * NCARGS, VM_MAP_PAGEABLE, FALSE, NULL);

	/*
	 * Allocate map for physio.
	 */
	phys_map = uvm_km_suballoc(kernel_map, &minaddr, &maxaddr,
	    VM_PHYS_SIZE, 0, FALSE, NULL);

	/*
	 * Finally, allocate mbuf cluster submap.
	 */
	mb_map = uvm_km_suballoc(kernel_map, &minaddr, &maxaddr,
				 nmbclusters * mclbytes, VM_MAP_INTRSAFE,
				 FALSE, NULL);

	format_bytes(pbuf, sizeof(pbuf), ptoa(uvmexp.free));
	printf("avail memory = %s\n", pbuf);

	/*
	 * Set up interrupt handlers.
	 */
	for (i = 0; i < NVMEINTR; i++)
		SLIST_INIT(&intr_handlers[i]);

	/*
	 * Finally switch to the real console driver,
	 * and say goodbye to the BUG!
	 */
	cninit();
}

__dead void
_doboot(void)
{
	cmmu_shutdown();
	bugreturn();
	/*NOTREACHED*/
	for (;;);		/* appease gcc */
}

void
cpu_reboot(int howto, char *bootstr)
{
	/* take a snapshot before clobbering any registers */
	if (curlwp && curlwp->l_addr)
		savectx(&curpcb->pcb_sf);

	/* If system is cold, just halt. */
	if (cold) {
#if 0 /* FIXME */		/* (Unless the user explicitly asked for reboot.) */
		if ((howto & RB_USERREQ) == 0)
#endif
			howto |= RB_HALT;
		goto haltsys;
	}

	boothowto = howto;
	if ((howto & RB_NOSYNC) == 0) {
		vfs_shutdown();
#if 0 /* FIXME */
		/*
		 * If we've been adjusting the clock, the todr
		 * will be out of synch; adjust it now unless
		 * the system was sitting in ddb.
		 */
		if ((howto & RB_TIMEBAD) == 0)
			resettodr();
		else
#endif
			printf("WARNING: not updating battery clock\n");
	}

	/* Disable interrupts. */
	splhigh();

	/* If rebooting and a dump is requested, do it. */
	if (howto & RB_DUMP)
		dumpsys();

haltsys:
	/* Run any shutdown hooks. */
	doshutdownhooks();

	if (howto & RB_HALT) {
		printf("System halted. Press any key to reboot...\n\n");
		cngetc();
	}

	doboot();

	for (;;);
	/*NOTREACHED*/
}

/*
 * These variables are needed by /sbin/savecore
 */
u_int32_t dumpmag = 0x8fca0101;	/* magic number */
int	dumpsize = 0;		/* pages */
long	dumplo = 0;		/* blocks */

/*
 * cpu_dumpsize: calculate the size of machine-dependent crash dump header.
 *               Returns size in disk blocks.
 */

#define CHDRSIZE (ALIGN(sizeof(kcore_seg_t)) + ALIGN(sizeof(cpu_kcore_hdr_t)))
#define MDHDRSIZE roundup(CHDRSIZE, dbtob(1))

int
cpu_dumpsize(void)
{
	return btodb(MDHDRSIZE);
}

/*
 * cpu_dump_mempagecnt: calculate size of RAM (in pages) to be dumped.
 */
u_long
cpu_dump_mempagecnt(void)
{
	return physmem;
}

/*
 * This is called by configure to set dumplo and dumpsize.
 * Dumps always skip the first PAGE_SIZE of disk space
 * in case there might be a disk label stored there.
 * If there is extra space, put dump at the end to
 * reduce the chance that swapping trashes it.
 */
void
cpu_dumpconf(void)
{
	const struct bdevsw *bdev;
	int nblks;	/* size of dump area */

	if (dumpdev == NODEV)
		goto bad;

	bdev = bdevsw_lookup(dumpdev);
	if (bdev == NULL)
		panic("dumpconf: bad dumpdev=0x%x", dumpdev);

	if (bdev->d_psize == NULL)
		goto bad;

	nblks = (*bdev->d_psize)(dumpdev);
	if (nblks <= ctod(1))
		goto bad;

	if (cpu_dumpsize() < 0)
		goto bad;

	/*
	 * Don't dump on the first block
	 * in case the dump device includes a disk label.
	 */
	if (dumplo < ctod(1))
		dumplo = ctod(1);

	/* dumpsize is in page units, and doesn't include headers. */
	dumpsize = cpu_dump_mempagecnt();

	/* Make dump fit in available space. */
	if (dumpsize > dtoc(nblks - (dumplo + cpu_dumpsize())))
		dumpsize = dtoc(nblks - (dumplo + cpu_dumpsize()));

	/* Put dump at end of partition. */
	if (dumplo < (nblks - (ctod(dumpsize) + cpu_dumpsize())))
		dumplo = nblks - (ctod(dumpsize) + cpu_dumpsize());

	return;

 bad:
	dumpsize = 0;
}

/*
 * Called by dumpsys() to dump the machine-dependent header.
 */
int
cpu_dump(dev_type_dump((*dump)), daddr_t *blknop)
{
	u_int8_t buf[MDHDRSIZE]; 
	cpu_kcore_hdr_t *chdr;
	kcore_seg_t *kseg;
	int error;

	memset(buf, 0, sizeof buf);
	kseg = (kcore_seg_t *)buf;
	chdr = (cpu_kcore_hdr_t *)&buf[ALIGN(sizeof(kcore_seg_t))];

	/* Create the segment header. */
	CORE_SETMAGIC(*kseg, KCORE_MAGIC, MID_MACHINE, CORE_CPU);
	kseg->c_size = MDHDRSIZE - ALIGN(sizeof(kcore_seg_t));

	/*
	 * Add the machine-dependent header info.
	 */
	chdr->cputype = cputyp;
	/* mvme88k only uses a single segment. */
	chdr->ram_segs[0].start = 0;
	chdr->ram_segs[0].size = ctob(physmem);

	error = (*dump)(dumpdev, *blknop, (caddr_t)buf, sizeof(buf));
	*blknop += btodb(sizeof(buf));

	return error;
}

/*
 * Dump physical memory onto the dump device.  Called by cpu_reboot().
 */
struct pcb dumppcb;

void
dumpsys(void)
{
	const struct bdevsw *bdev;
	u_long totalbytesleft, i, n;
	paddr_t maddr;
	int psize;
	daddr_t blkno;
	int (*dump) __P((dev_t, daddr_t, caddr_t, size_t));
	int error;

	/* Save registers. */
	savectx(&dumppcb.pcb_sf);

	if (dumpdev == NODEV)
		return;
	bdev = bdevsw_lookup(dumpdev);
	if (bdev == NULL || bdev->d_psize == NULL)
		return;

	/*
	 * For dumps during autoconfiguration,
	 * if dump device has already configured...
	 */
	if (dumpsize == 0)
		cpu_dumpconf();
	if (dumplo <= 0) {
		printf("\ndump to dev %u,%u not possible\n", major(dumpdev),
		    minor(dumpdev));
		return;
	}
	printf("\ndumping to dev %u,%u offset %ld\n", major(dumpdev),
	    minor(dumpdev), dumplo);

	psize = (*bdev->d_psize)(dumpdev);
	printf("dump ");
	if (psize == -1) {
		printf("area unavailable\n");
		return;
	}

	/* XXX should purge all outstanding keystrokes. */

	dump = bdev->d_dump;
	blkno = dumplo;

	if ((error = cpu_dump(dump, &blkno)) != 0)
		goto err;

	totalbytesleft = ptoa(dumpsize);
	maddr = (paddr_t)0;

	for (i = 0; i < totalbytesleft; i += n, totalbytesleft -= n) {

		/* Print out how many MBs we have left to go. */
		if ((totalbytesleft % (1024*1024)) == 0)
			printf("%ld ", totalbytesleft / (1024 * 1024));

		/* Limit size for next transfer. */
		n = totalbytesleft - i;
		if (n > PAGE_SIZE)
			n = PAGE_SIZE;

		pmap_enter(pmap_kernel(), (vaddr_t)vmmap, maddr,
		    VM_PROT_READ, VM_PROT_READ|PMAP_WIRED);
		pmap_update(pmap_kernel());

		error = (*dump)(dumpdev, blkno, vmmap, n);
		if (error)
			goto err;
		maddr += n;
		blkno += btodb(n);
	}

 err:
	switch (error) {

	case ENXIO:
		printf("device bad\n");
		break;

	case EFAULT:
		printf("device not ready\n");
		break;

	case EINVAL:
		printf("area improper\n");
		break;

	case EIO:
		printf("i/o error\n");
		break;

	case EINTR:
		printf("aborted from console\n");
		break;

	case 0:
		printf("succeeded\n");
		break;

	default:
		printf("error %d\n", error);
		break;
	}
	printf("\n\n");
}

#ifdef MULTIPROCESSOR

/*
 * Secondary CPU early initialization routine.
 * Determine CPU number and set it, then allocate the idle pcb (and stack).
 *
 * Running on a minimal stack here, with interrupts disabled; do nothing fancy.
 */
void
secondary_pre_main(void)
{
	struct cpu_info *ci;

	set_cpu_number(cmmu_cpu_number()); /* Determine cpu number by CMMU */
	ci = curcpu();
	ci->ci_curlwp = &lwp0;

	splhigh();

	/*
	 * Setup CMMUs and translation tables (shared with the master cpu).
	 */
	pmap_bootstrap_cpu(ci->ci_cpuid);

	/*
	 * Allocate UPAGES contiguous pages for the idle PCB and stack.
	 */
	ci->ci_idle_pcb = (struct pcb *)uvm_km_zalloc(kernel_map, USPACE);
	if (ci->ci_idle_pcb == NULL) {
		printf("cpu%d: unable to allocate idle stack\n", ci->ci_cpuid);
	}
}

/*
 * Further secondary CPU initialization.
 *
 * We are now running on our idle stack, with proper page tables.
 * There is nothing to do but display some details about the CPU and its CMMUs.
 */
void
secondary_main(void)
{
	struct cpu_info *ci = curcpu();

	cpu_configuration_print(0);
	__cpu_simple_unlock(&cpu_mutex);

	microuptime(&ci->ci_schedstate.spc_runtime);
	ci->ci_curlwp = NULL;
	SET(ci->ci_flags, CIF_ALIVE);

	/*
	 * Upon return, the secondary cpu bootstrap code in locore will
	 * enter the idle loop, waiting for some food to process on this
	 * processor.
	 */
}

#endif	/* MULTIPROCESSOR */

/*
 * Search for the first available interrupt vector in the range start, end.
 * This should really only be used by VME devices.
 */
int
intr_findvec(int start, int end, int skip)
{
	int vec;

#ifdef DEBUG
	if (start < 0 || end >= NVMEINTR || start > end)
		panic("intr_findvec(%d,%d): bad parameters", start, end);
#endif

	for (vec = start; vec <= end; vec++) {
		if (vec == skip)
			continue;
		if (SLIST_EMPTY(&intr_handlers[vec]))
			return vec;
	}
#ifdef DIAGNOSTIC
	printf("intr_findvec(%d,%d,%d): no vector available\n",
	    start, end, skip);
#endif
	return -1;
}

/*
 * Try to insert ihand in the list of handlers for vector vec.
 */
int
intr_establish(int vec, struct intrhand *ihand, const char *name)
{
	struct intrhand *intr;
	intrhand_t *list;

#ifdef DEBUG
	printf("intr_establish: vec 0x%x ipl %x\n", vec, ihand->ih_ipl);
#endif

	if (vec < 0 || vec >= NVMEINTR) {
#ifdef DIAGNOSTIC
		panic("intr_establish: vec (0x%x) not between 0x00 and 0xff",
		      vec);
#endif /* DIAGNOSTIC */
		return (EINVAL);
	}

	list = &intr_handlers[vec];
	if (!SLIST_EMPTY(list)) {
		intr = SLIST_FIRST(list);
		if (intr->ih_ipl != ihand->ih_ipl) {
#ifdef DIAGNOSTIC
			panic("intr_establish: there are other handlers with "
			    "vec (0x%x) at ipl %x, but you want it at %x",
			    vec, intr->ih_ipl, ihand->ih_ipl);
#endif /* DIAGNOSTIC */
			return (EINVAL);
		}
	}

#if 0 /* XXX  - not sure about this -TKM */
	evcount_attach(&ihand->ih_count, name, (void *)&ihand->ih_ipl,
	    &evcount_intr);
#endif

	SLIST_INSERT_HEAD(list, ihand, ih_link);
	return (0);
}

void
nmihand(void *frame)
{
#ifdef DDB
	printf("Abort switch pressed\n");
	/* if (db_console)   XXX - what is NetBSD equivalent? -TKM */
	{
		/*
		 * We can't use Debugger() here, as we are coming from an
		 * exception handler, and can't assume anything about the
		 * state we are in. Invoke the post-trap ddb entry directly.
		 */
		extern void m88k_db_trap(int, struct trapframe *);
		m88k_db_trap(T_KDB_ENTRY, (struct trapframe *)frame);
	}
#endif
}

int
sys_sysarch(struct lwp *l, void *v, register_t *retval)
{
#if 0
	struct sys_sysarch_args	/* {
	   syscallarg(int) op;
	   syscallarg(char *) parm;
	} */ *uap = v;
#endif

	return (ENOSYS);
}

/*
 * machine dependent system variables.
 */
SYSCTL_SETUP(sysctl_machdep_setup, "sysctl machdep subtree setup")
{
	sysctl_createv(clog, 0, NULL, NULL,
		       CTLFLAG_PERMANENT,
		       CTLTYPE_NODE, "machdep", NULL,
		       NULL, 0, NULL, 0,
		       CTL_MACHDEP, CTL_EOL);

	sysctl_createv(clog, 0, NULL, NULL,
		       CTLFLAG_PERMANENT,
		       CTLTYPE_STRUCT, "console_device", NULL,
		       sysctl_consdev, 0, NULL, sizeof(dev_t),
		       CTL_MACHDEP, CPU_CONSDEV, CTL_EOL);
}

void
mvme88k_vector_init(u_int32_t *vbr, u_int32_t *vectors)
{
	extern void vector_init(u_int32_t *, u_int32_t *);	/* gross */

	/* Save BUG vector */
	bugvec[0] = vbr[MVMEPROM_VECTOR * 2 + 0];
	bugvec[1] = vbr[MVMEPROM_VECTOR * 2 + 1];

	vector_init(vbr, vectors);

	/* Save new BUG vector */
	sysbugvec[0] = vbr[MVMEPROM_VECTOR * 2 + 0];
	sysbugvec[1] = vbr[MVMEPROM_VECTOR * 2 + 1];
}

/*
 * Called from locore.S during boot, this is the first C code that's run.
 */
void
mvme_bootstrap(void)
{
	extern int kernelstart;

	struct mvmeprom_brdid brdid;
#ifndef MULTIPROCESSOR
	cpuid_t master_cpu;
#endif
	const vaddr_t   end_24bit = (vaddr_t)0x1000000;
	vaddr_t        start_24bit = VM_MAX_ADDRESS;

	/* Get board ID & Ethernet address from BUG */
	buginit();

	bugbrdid(&brdid);
	brdtyp = brdid.model;
	memcpy(mvme_ea, &brdid.etheraddr, sizeof mvme_ea);

	/*
	 * Set up interrupt and fp exception handlers based on the machine.
	 */
	switch (brdtyp) {
#ifdef MVME187
	case BRD_187:
	case BRD_8120:
		m187_bootstrap();
		break;
#endif
#ifdef MVME188
	case BRD_188:
		m188_bootstrap();
		break;
#endif
#ifdef MVME197
	case BRD_197:
		m197_bootstrap();
		break;
#endif
	default:
		panic("Sorry, this kernel does not support MVME%x", brdtyp);
	}

	/*
	 * Use the BUG as console for now. After autoconf, we'll switch to
	 * real hardware.
	 *
	 * Note:  Can't use printf until after mXXX_bootstrap, as ipl function
	 *        pointers are not set up, and printf will attempt to change
	 *        ipl.
	 */
	bootcons.cn_init(&bootcons);

	uvmexp.pagesize = PAGE_SIZE;
	uvm_setpagesize();
	first_addr = round_page(first_addr);

	switch (brdtyp) {
#ifdef MVME187
	case BRD_187:
	case BRD_8120:
		last_addr = m187_memsize();
		break;
#endif
#ifdef MVME188
	case BRD_188:
		last_addr = m188_memsize();
		break;
#endif
#ifdef MVME197
	case BRD_197:
		last_addr = m197_memsize();
		break;
#endif
	}
	physmem = btoc(last_addr);

	setup_board_config();
	master_cpu = cmmu_init();
	set_cpu_number(master_cpu);
	SET(curcpu()->ci_flags, CIF_ALIVE | CIF_PRIMARY);

	/*
	 * Now that set_cpu_number() set us with a valid cpu_info pointer,
	 * we need to initialize l_addr and curpcb before autoconf, for the
	 * fault handler to behave properly [except for badaddr() faults,
	 * which can be taken care of without a valid curcpu()].
	 */
	lwp0.l_addr = proc0paddr;
	curpcb = &proc0paddr->u_pcb;

	avail_start = first_addr;
	avail_end = last_addr;

#ifdef DEBUG
	printf("MVME%x boot: memory from %p to %p\n",
	    brdtyp, (void*)avail_start, (void*)avail_end);
#endif

	/*
	 * Steal MSGBUFSIZE at the top of physical memory for msgbuf
	 */
	avail_end -= round_page(MSGBUFSIZE);

	/*
	 * If there's memory beyond 16MB available, reserve any memory
	 * below 16MB for the 24-bit freelist.
	 */
	if (end_24bit < avail_end && round_page(avail_start) < end_24bit)
	{
		start_24bit = round_page(avail_start);
		avail_start = end_24bit;
#ifdef DEBUG
		printf("Reserved from %p to %p for 24-bit freelist\n",
		    (void*)start_24bit, (void*)(end_24bit-1));
#endif
	}

	pmap_bootstrap((vaddr_t)trunc_page((unsigned)&kernelstart));

	/*
	 * Tell the VM system about available physical memory.
	 *
	 * One segment & freelist is used for 24-bit memory, if available.
	 * All other memory is put into the main segment, which uses the
	 * default freelist.
	 *
	 * BUG could be set up to configure a non-contiguous scheme; also, we
	 * might want to register ECC memory separately later on...
	 */
	uvm_page_physload(atop(avail_start), atop(avail_end),
	    atop(avail_start), atop(avail_end), VM_FREELIST_DEFAULT);

	if (start_24bit < end_24bit)
	{
		uvm_page_physload(atop(start_24bit), atop(end_24bit),
		    atop(start_24bit), atop(end_24bit), VM_FREELIST_24BIT);
	}

	/* Initialize the "u-area" pages. */
	bzero((caddr_t)curpcb, USPACE);

#ifdef DEBUG
	printf("leaving mvme_bootstrap()\n");
#endif
}

#ifdef MULTIPROCESSOR
void
cpu_boot_secondary_processors(void)
{
	cpuid_t cpu;
	int rc;
	extern void secondary_start(void);

	switch (brdtyp) {
#if defined(MVME188) || defined(MVME197)
#ifdef MVME188
	case BRD_188:
#endif
#ifdef MVME197
	case BRD_197:
#endif
		for (cpu = 0; cpu < max_cpus; cpu++) {
			if (cpu != curcpu()->ci_cpuid) {
				rc = spin_cpu(cpu, (vaddr_t)secondary_start);
				if (rc != 0 && rc != FORKMPU_NO_MPU)
					printf("cpu%d: spin_cpu error %d\n",
					    cpu, rc);
			}
		}
		break;
#endif
	default:
		break;
	}
}
#endif

u_int
getipl(void)
{
	u_int curspl, psr;

	disable_interrupt(psr);
	curspl = (*md_getipl)();
	set_psr(psr);
	return curspl;
}

unsigned
setipl(unsigned level)
{
	u_int curspl, psr;

	disable_interrupt(psr);
	curspl = (*md_setipl)(level);

	/*
	 * The flush pipeline is required to make sure the above change gets
	 * through the data pipe and to the hardware; otherwise, the next
	 * bunch of instructions could execute at the wrong spl protection.
	 */
	flush_pipeline();

	set_psr(psr);
	return curspl;
}

unsigned
raiseipl(unsigned level)
{
	u_int curspl, psr;

	disable_interrupt(psr);
	curspl = (*md_raiseipl)(level);

	/*
	 * The flush pipeline is required to make sure the above change gets
	 * through the data pipe and to the hardware; otherwise, the next
	 * bunch of instructions could execute at the wrong spl protection.
	 */
	flush_pipeline();

	set_psr(psr);
	return curspl;
}
