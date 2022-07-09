/*	$OpenBSD: machdep.c,v 1.44 2007/06/06 17:15:12 deraadt Exp $	*/
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
 * any improvements or extensions that they make and grant Carnegie the
 * rights to redistribute these changes.
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
#include <sys/mount.h>
#include <sys/mbuf.h>
#include <sys/msgbuf.h>
#include <sys/sa.h>
#include <sys/syscallargs.h>
#include <sys/ksyms.h>
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
#include <machine/board.h>
#include <machine/cmmu.h>
#include <machine/cpu.h>
#include <machine/kcore.h>
#include <machine/reg.h>
#include <machine/trap.h>
#include <machine/m88100.h>

#include <luna88k/luna88k/isr.h>

#include <dev/cons.h>

#include <uvm/uvm_extern.h>

#include "ksyms.h"
#if DDB
#include <machine/db_machdep.h>
#include <ddb/db_extern.h>
#include <ddb/db_interface.h>
#include <ddb/db_output.h>		/* db_printf()		*/
#endif /* DDB */

caddr_t	allocsys(caddr_t);
void	consinit(void);
void	dumpconf(void);
void	dumpsys(void);
int	getcpuspeed(void);
u_int	getipl(void);
void	identifycpu(void);
int	cpu_dumpsize(void);
u_long	cpu_dump_mempagecnt(void);
int	cpu_dump(dev_type_dump((*dump)), daddr_t *blknop);
void	luna88k_bootstrap(void);
void	secondary_main(void);
void	secondary_pre_main(void);
void	setlevel(unsigned int);

vaddr_t size_memory(void);
void powerdown(void);
void get_fuse_rom_data(void);
void get_nvram_data(void);
char *nvram_by_symbol(char *);
void get_autoboot_device(void);			/* in disksubr.c */
int clockintr(void *);				/* in clock.c */

/*
 * *int_mask_reg[CPU]
 * Points to the hardware interrupt status register for each CPU.
 */
unsigned int *volatile int_mask_reg[] = {
	(unsigned int *)INT_ST_MASK0,
	(unsigned int *)INT_ST_MASK1,
	(unsigned int *)INT_ST_MASK2,
	(unsigned int *)INT_ST_MASK3
};

unsigned int luna88k_curspl[] = {0, 0, 0, 0};

unsigned int int_set_val[INT_LEVEL] = {
	INT_SET_LV0,
	INT_SET_LV1,
	INT_SET_LV2,
	INT_SET_LV3,
	INT_SET_LV4,
	INT_SET_LV5,
	INT_SET_LV6,
	INT_SET_LV7
};

/*
 * *clock_reg[CPU]
 */
unsigned int *volatile clock_reg[] = {
	(unsigned int *)OBIO_CLOCK0,
	(unsigned int *)OBIO_CLOCK1,
	(unsigned int *)OBIO_CLOCK2,
	(unsigned int *)OBIO_CLOCK3
};

/*
 * FUSE ROM and NVRAM data
 */
struct fuse_rom_byte {
	u_int32_t h;
	u_int32_t l;
};
#define FUSE_ROM_BYTES        (FUSE_ROM_SPACE / sizeof(struct fuse_rom_byte))
char fuse_rom_data[FUSE_ROM_BYTES];

#define NNVSYM		8
#define NVSYMLEN	16
#define NVVALLEN	16
struct nvram_t {
	char symbol[NVSYMLEN];
	char value[NVVALLEN];
} nvram[NNVSYM];

vaddr_t obiova;

int physmem;	  /* available physical memory, in pages */

struct vm_map *exec_map = NULL;
struct vm_map *mb_map = NULL;
struct vm_map *phys_map = NULL;

/*
 * Info for CTL_HW
 */
char  machine[] = MACHINE;	 /* cpu "architecture" */
char  cpu_model[120];

#if defined(DDB) || NKSYMS > 0
extern char *esym;
#endif

int machtype = LUNA_88K;	/* may be overwritten in cpu_startup() */
int cputyp = CPU_88100;		/* XXX: aoyama */
int boothowto;			/* XXX: should be set in boot loader and locore.S */
int bootdev;			/* XXX: should be set in boot loader and locore.S */
int cpuspeed = 33;		/* safe guess */
int sysconsole = 1;		/* 0 = ttya, 1 = keyboard/mouse, used in dev/sio.c */
u_int16_t dipswitch = 0;	/* set in locore.S */
int hwplanebits;		/* set in locore.S */

extern struct consdev syscons;	/* in dev/siotty.c */

extern void syscnattach(int);	/* in dev/siotty.c */
extern int omfb_cnattach(void);	/* in dev/lunafb.c */
extern void ws_cnattach(void);	/* in dev/lunaws.c */

vaddr_t first_addr;
vaddr_t last_addr;

vaddr_t avail_start, avail_end;
vaddr_t virtual_avail, virtual_end;

extern struct user *proc0paddr;

/*
 * This is to fake out the console routines, while booting.
 * We could use directly the romtty console, but we want to be able to
 * configure a kernel without romtty since we do not necessarily need a
 * full-blown console driver.
 */
cons_decl(romtty);
extern void nullcnpollc(dev_t, int);

struct consdev romttycons = {
	NULL, 
	NULL, 
	romttycngetc, 
	romttycnputc,
	nullcnpollc,
	NULL,
	NULL,
	NULL,
	makedev(7, 0),
	CN_NORMAL,
};

/*
 * Early console initialization: called early on from main, before vm init.
 */
void
consinit(void)
{
	/*
	 * Initialize the console before we print anything out.
	 */
	if (sysconsole == 0) {
                syscnattach(0);
        } else {
                omfb_cnattach();
                ws_cnattach();
        }

#if NKSYMS || defined(DDB) || defined(LKM)
	{
		extern int end;

		ksyms_init(*(int *)&end, ((int *)&end) + 1, esym);
	}
#endif
#if defined(DDB)
	if (boothowto & RB_KDB)
		Debugger();
#endif
}

/*
 * Figure out how much real memory is available.
 * Start looking from the megabyte after the end of the kernel data,
 * until we find non-memory.
 */
vaddr_t
size_memory(void)
{
	unsigned int *volatile look;
	unsigned int *max;
#if 0
	extern char *end;
#endif
#define PATTERN   0x5a5a5a5a
#define STRIDE    (4*1024) 	/* 4k at a time */
#define Roundup(value, stride) (((unsigned)(value) + (stride) - 1) & ~((stride)-1))
	/*
	 * count it up.
	 */
	max = (void *)MAXPHYSMEM;
#if 0
	for (look = (void *)Roundup(end, STRIDE); look < max;
#else
	for (look = (void *)first_addr; look < max;
#endif
	    look = (int *)((unsigned)look + STRIDE)) {
		unsigned save;

		/* if can't access, we've reached the end */
		if (badaddr((vaddr_t)look, 4)) {
#if defined(DEBUG)
			printf("%x\n", look);
#endif
			look = (int *)((int)look - STRIDE);
			break;
		}

		/*
		 * If we write a value, we expect to read the same value back.
		 * We'll do this twice, the 2nd time with the opposite bit
		 * pattern from the first, to make sure we check all bits.
		 */
		save = *look;
		if (*look = PATTERN, *look != PATTERN)
			break;
		if (*look = ~PATTERN, *look != ~PATTERN)
			break;
		*look = save;
	}

	return (trunc_page((unsigned)look));
}

int
getcpuspeed(void)
{
	switch(machtype) {
	case LUNA_88K:
		return 25;
	case LUNA_88K2:
		return 33;
	default:
		panic("getcpuspeed: can not determine CPU speed");
	}
}

void
identifycpu(void)
{
	cpuspeed = getcpuspeed();
	snprintf(cpu_model, sizeof cpu_model,
	    "OMRON LUNA-88K%s, %dMHz", 
	    machtype == LUNA_88K2 ? "2" : "", cpuspeed);
}

void
cpu_startup(void)
{
	char pbuf[9];
	caddr_t v;
	int sz, i;
	vaddr_t minaddr, maxaddr;

	/*
	 * Initialize error message buffer (at end of core).
	 * avail_end was pre-decremented in luna88k_bootstrap() to compensate.
	 */
	for (i = 0; i < btoc(MSGBUFSIZE); i++)
		pmap_kenter_pa((paddr_t)msgbufp + i * PAGE_SIZE,
		    avail_end + i * PAGE_SIZE, VM_PROT_READ | VM_PROT_WRITE);
	pmap_update(pmap_kernel());
	initmsgbuf((caddr_t)msgbufp, round_page(MSGBUFSIZE));

	/* Determine the machine type from FUSE ROM data.  */
	get_fuse_rom_data();
	if (strncmp(fuse_rom_data, "MNAME=LUNA88K+", 14) == 0) {
		machtype = LUNA_88K2;
	}

        /* Determine the 'auto-boot' device from NVRAM data */
        get_nvram_data();
        get_autoboot_device();

	/*
	 * Good {morning,afternoon,evening,night}.
	 */
	printf(version);
	identifycpu();
	format_bytes(pbuf, sizeof(pbuf), ctob(physmem));
	printf("total memory = %s\n", pbuf);

	/*
	 * Check front DIP switch setting
	 */
#ifdef DEBUG
	printf("dipsw = 0x%x\n", dipswitch);
#endif

	/* Check DIP switch 1 - 1 */
	if ((0x8000 & dipswitch) == 0) {
		boothowto |= RB_SINGLE;
	}

	/* Check DIP switch 1 - 3 */
	if ((0x2000 & dipswitch) == 0) {
		boothowto |= RB_ASKNAME;
	}

	/* Check DIP switch 1 - 4 */
	if ((0x1000 & dipswitch) == 0) {
		boothowto |= RB_USERCONF;
	}

	/*
	 * Check frame buffer depth.
	 */
	switch (hwplanebits) {
	case 0:				/* No frame buffer */
	case 1:
	case 4:
	case 8:
		break;
	default:
		printf("unexpected frame buffer depth = %d\n", hwplanebits);
		hwplanebits = 0;
		break;
	}

#if 0 /* just for test */
	/*
	 * Get boot arguments
	 */
	{
		char buf[256];
		char **p = (volatile char **)0x00001120;

		strncpy(buf, *p, 256);
		if (buf[255] != '\0')
			buf[255] = '\0';

		printf("boot arg: (0x%x) %s\n", *p, buf);
	}
#endif

	/*
	 * Find out how much space we need, allocate it,
	 * and then give everything true virtual addresses.
	 */
	sz = (int)allocsys((caddr_t)0);

	if ((v = (caddr_t)uvm_km_zalloc(kernel_map, round_page(sz))) == 0)
		panic("startup: no room for tables");
	if (allocsys(v) - v != sz)
		panic("startup: table size inconsistency");

	/*
	 * Grab the OBIO space that we hardwired in pmap_bootstrap
	 */
	obiova = OBIO_START;
	uvm_map(kernel_map, (vaddr_t *)&obiova, OBIO_SIZE,
	    NULL, UVM_UNKNOWN_OFFSET, 0,
	      UVM_MAPFLAG(UVM_PROT_NONE, UVM_PROT_NONE, UVM_INH_NONE,
	        UVM_ADV_NORMAL, 0));
	if (obiova != OBIO_START)
		panic("obiova %lx: OBIO not free", obiova);

	/*
	 * Allocate a submap for exec arguments.  This map effectively
	 * limits the number of processes exec'ing at any time.
	 */
	minaddr = vm_map_min(kernel_map);
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
	    nmbclusters * mclbytes, VM_MAP_INTRSAFE, FALSE, NULL);

	format_bytes(pbuf, sizeof(pbuf), ptoa(uvmexp.free));
	printf("avail memory = %s\n", pbuf);

	/*
	 * Initialize the autovectored interrupt list.
	 */
	isrinit();
}

/*
 * Allocate space for system data structures.  We are given
 * a starting virtual address and we return a final virtual
 * address; along the way we set each data structure pointer.
 *
 * We call allocsys() with 0 to find out how much space we want,
 * allocate that much and fill it with zeroes, and then call
 * allocsys() again with the correct base virtual address.
 */
caddr_t
allocsys(caddr_t v)
{

#define	valloc(name, type, num) \
	    v = (caddr_t)(((name) = (type *)v) + (num))

#ifdef SYSVMSG
	valloc(msgpool, char, msginfo.msgmax);
	valloc(msgmaps, struct msgmap, msginfo.msgseg);
	valloc(msghdrs, struct msg, msginfo.msgtql);
	valloc(msqids, struct msqid_ds, msginfo.msgmni);
#endif

	return v;
}

void
cpu_reboot(int howto, char *bootstr)
{
	/* take a snapshot before clobbering any registers */
	if (curlwp && curlwp->l_addr)
		savectx(&curlwp->l_addr->u_pcb.kernel_state);

	/* If system is cold, just halt. */
	if (cold) {
		howto |= RB_HALT;
		goto haltsys;
	}

	boothowto = howto;
	if ((howto & RB_NOSYNC) == 0) {
		vfs_shutdown();
		/*
		 * If we've been adjusting the clock, the todr
		 * will be out of synch; adjust it now unless
		 * the system was sitting in ddb.
		 */
		resettodr();
	}

	/* Disable interrupts. */
	splhigh();

	/* If rebooting and a dump is requested, do it. */
	if (howto & RB_DUMP)
		dumpsys();

haltsys:
	/* Run any shutdown hooks. */
	doshutdownhooks();

	/* Luna88k supports automatic powerdown */
	if ((howto & RB_POWERDOWN) == RB_POWERDOWN) {
		printf("attempting to power down...\n");
		powerdown();
		/* if failed, fall through. */
	}

	if (howto & RB_HALT) {
		printf("halted\n\n");
	} else {
		/* Reset all cpus, which causes reboot */
		*((volatile unsigned *)0x6d000010) = 0;
	}

	for (;;);  /* to keep compiler happy, and me from going crazy */
	/*NOTREACHED*/
}

u_int32_t dumpmag = 0x8fca0101;	 /* magic number for savecore */
int   dumpsize = 0;	/* also for savecore */
long  dumplo = 0;

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

	/*
	 * Don't dump on the first block
	 * in case the dump device includes a disk label.
	 */
	if (dumplo < ctod(1))
		dumplo = ctod(1);

	/* Make dump fit in available space. */
	if (dumpsize > dtoc(nblks - (dumplo + cpu_dumpsize())))
		dumpsize = dtoc(nblks - (dumplo + cpu_dumpsize()));

	/* dumpsize is in page units, and doesn't include headers. */
	dumpsize = cpu_dump_mempagecnt();

	/* Put dump at end of partition, and make it fit. */
	if (dumplo < (nblks - (ctod(dumpsize) + cpu_dumpsize())))
		dumplo = nblks - (ctod(dumpsize) + cpu_dumpsize());

	return;

 bad:
	dumpsize = 0;
}

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
	/* luna88k only uses a single segment. */
	chdr->ram_segs[0].start = 0;
	chdr->ram_segs[0].size = ctob(physmem);

	error = (*dump)(dumpdev, *blknop, (caddr_t)buf, sizeof(buf));
	*blknop += btodb(sizeof(buf));

	return error;
}

struct pcb dumppcb;

/*
 * Doadump comes here after turning off memory management and
 * getting on the dump stack, either when called above, or by
 * the auto-restart code.
 */
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

	set_cpu_number(cmmu_cpu_number());
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

	/*
	 * Upon return, the secondary cpu bootstrap code in locore will
	 * enter the idle loop, waiting for some food to process on this
	 * processor.
	 */
}

#endif	/* MULTIPROCESSOR */

/*
 *	Device interrupt handler for LUNA88K
 */

void 
luna88k_ext_int(u_int v, struct trapframe *eframe)
{
	int cpu = cpu_number();
	unsigned int cur_mask, cur_int;
	unsigned int level, old_spl;

	cur_mask = *int_mask_reg[cpu];
	old_spl = luna88k_curspl[cpu];
	eframe->tf_mask = old_spl;

	cur_int = cur_mask >> 29;

	if (cur_int == 0) {
		/*
		 * Spurious interrupts - may be caused by debug output clearing
		 * serial port interrupts.
		 */
#ifdef DEBUG
		printf("luna88k_ext_int(): Spurious interrupts?\n");
#endif
		flush_pipeline();
		goto out;
	}
 
	uvmexp.intrs++;

	/* 
	 * We want to service all interrupts marked in the IST register
	 * They are all valid because the mask would have prevented them
	 * from being generated otherwise.  We will service them in order of
	 * priority. 
	 */

	/* XXX: This is very rough. Should be considered more. (aoyama) */
	do {
		level = (cur_int > old_spl ? cur_int : old_spl);

#ifdef DEBUG
		if (level > 7 || (char)level < 0) {
			panic("int level (%x) is not between 0 and 7", level);
		}
#endif

		setipl(level);
	  
		set_psr(get_psr() & ~PSR_IND);

		switch(cur_int) {
		case CLOCK_INT_LEVEL:
			clockintr((void *)eframe);
			break;
		case 5:
		case 4:
		case 3:
			isrdispatch_autovec(cur_int);
			break;
		default:
			printf("luna88k_ext_int(): level %d interrupt.\n", cur_int);
			break;
		}
	} while ((cur_int = (*int_mask_reg[cpu]) >> 29) != 0);

out:
	/*
	 * process any remaining data access exceptions before
	 * returning to assembler
	 */
	if (eframe->tf_dmt0 & DMT_VALID)
		m88100_trap(T_DATAFLT, eframe);

	/*
	 * Disable interrupts before returning to assembler, the spl will
	 * be restored later.
	 */
	set_psr(get_psr() | PSR_IND);
}

int
cpu_exec_aout_makecmds(struct proc *p, struct exec_package *epp)
{

	return (ENOEXEC);
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

/*
 * Called from locore.S during boot,
 * this is the first C code that's run.
 */
void
luna88k_bootstrap(void)
{
	extern int kernelstart;
	extern struct consdev *cn_tab;
	extern struct cmmu_p cmmu8820x;
	extern char *end;
#ifndef MULTIPROCESSOR
	cpuid_t master_cpu;
#endif
	cpuid_t cpu;
	extern void m8820x_initialize_cpu(cpuid_t);
	extern void m8820x_set_sapr(cpuid_t, apr_t);
	extern void cpu_boot_secondary_processors(void);

	cmmu = &cmmu8820x;

	/* clear and disable all interrupts */
	*int_mask_reg[0] = 0;
	*int_mask_reg[1] = 0;
	*int_mask_reg[2] = 0;
	*int_mask_reg[3] = 0;

	/* startup fake console driver.  It will be replaced by consinit() */
	cn_tab = &romttycons;

	uvmexp.pagesize = PAGE_SIZE;
	uvm_setpagesize();

	first_addr = round_page((vaddr_t)&end);	/* XXX temp until symbols */
	last_addr = size_memory();
	physmem = btoc(last_addr);

	setup_board_config();
	master_cpu = cmmu_init();
	set_cpu_number(master_cpu);

	m88100_apply_patches();

	/*
	 * Now that set_cpu_number() set us with a valid cpu_info pointer,
	 * we need to initialize p_addr and curpcb before autoconf, for the
	 * fault handler to behave properly [except for badaddr() faults,
	 * which can be taken care of without a valid curcpu()].
	 */
	lwp0.l_addr = proc0paddr;
	curpcb = &proc0paddr->u_pcb;

	avail_start = first_addr;
	avail_end = last_addr;

	/*
	 * Steal MSGBUFSIZE at the top of physical memory for msgbuf
	 */
	avail_end -= round_page(MSGBUFSIZE);

#ifdef DEBUG
	printf("LUNA88K boot: memory from 0x%x to 0x%x\n",
	    avail_start, avail_end);
#endif
	pmap_bootstrap((vaddr_t)trunc_page((unsigned)&kernelstart));

	/*
	 * Tell the VM system about available physical memory.
	 * luna88k only has one segment.
	 */
	uvm_page_physload(atop(avail_start), atop(avail_end),
	    atop(avail_start), atop(avail_end),VM_FREELIST_DEFAULT);

	/* Initialize the "u-area" pages. */
	bzero((caddr_t)curpcb, USPACE);

	/*
	 * On the luna88k, secondary processors are not disabled while the
	 * kernel is initializing. We just initialized the CMMUs tied to the
	 * currently-running CPU; initialize the others with similar settings
	 * as well, after calling pmap_bootstrap() above.
	 */
	for (cpu = 0; cpu < max_cpus; cpu++) {
		if (cpu == master_cpu)
			continue;
		if (m88k_cpus[cpu].ci_alive == 0)
			continue;
		m8820x_initialize_cpu(cpu);
		cmmu_set_sapr(cpu, kernel_pmap->pm_apr);
	}
	/* Release the cpu_mutex */
	cpu_boot_secondary_processors();

#ifdef DEBUG
	printf("leaving luna88k_bootstrap()\n");
#endif
}

/*
 * Rom console routines: 
 * Enables printing of boot messages before consinit().
 */

#define __ROM_FUNC_TABLE	((int **)0x00001100)
#define ROMGETC()	(*(int (*)(void))__ROM_FUNC_TABLE[3])()
#define ROMPUTC(x)	(*(void (*)(int))__ROM_FUNC_TABLE[4])(x)

void
romttycnprobe(struct consdev *cp)
{
	cp->cn_dev = makedev(14, 0);
	cp->cn_pri = CN_NORMAL;
}

void
romttycninit(struct consdev *cp)
{
	/* Nothing to do */
}

int
romttycngetc(dev_t dev)
{
	int s, c;

	do {
		s = splhigh();
		c = ROMGETC();
		splx(s);
	} while (c == -1);
	return c;
}

void
romttycnputc(dev_t dev, int c)
{
	int s;

#if 0
	if ((char)c == '\n')
		ROMPUTC('\r');
#endif
	s = splhigh();
	ROMPUTC(c);
	splx(s);
}

/* taken from NetBSD/luna68k */
void
microtime(register struct timeval *tvp)
{
        int s = splclock();
        static struct timeval lasttime;

        *tvp = time;
#ifdef notdef
        tvp->tv_usec += clkread();
        while (tvp->tv_usec >= 1000000) {
                tvp->tv_sec++;
                tvp->tv_usec -= 1000000;
        }
#endif
        if (tvp->tv_sec == lasttime.tv_sec &&
            tvp->tv_usec <= lasttime.tv_usec &&
            (tvp->tv_usec = lasttime.tv_usec + 1) >= 1000000) {
                tvp->tv_sec++;
                tvp->tv_usec -= 1000000;
        }
        lasttime = *tvp;
        splx(s);
}

/* powerdown */

struct pio {
	volatile u_int8_t portA;
	volatile unsigned : 24;
	volatile u_int8_t portB;
	volatile unsigned : 24;
	volatile u_int8_t portC;
	volatile unsigned : 24;
	volatile u_int8_t cntrl;
	volatile unsigned : 24;
};

#define	PIO1_POWER	0x04

#define	PIO1_ENABLE	0x01
#define	PIO1_DISABLE	0x00

void
powerdown(void) 
{
	struct pio *p1 = (struct pio *)OBIO_PIO1_BASE;

	DELAY(100000);
	p1->cntrl = (PIO1_POWER << 1) | PIO1_DISABLE;
	*(volatile u_int8_t *)&p1->portC;
}

/* Get data from FUSE ROM */

void
get_fuse_rom_data(void)
{
	int i;
	struct fuse_rom_byte *p = (struct fuse_rom_byte *)FUSE_ROM_ADDR;

	for (i = 0; i < FUSE_ROM_BYTES; i++) {
		fuse_rom_data[i] =
		    (char)((((p->h) >> 24) & 0x000000f0) |
		           (((p->l) >> 28) & 0x0000000f));
		p++;                                                                            
	}
}

/* Get data from NVRAM */

void
get_nvram_data(void)
{
	int i, j;
	u_int8_t *page;
	char buf[NVSYMLEN], *data;

	if (machtype == LUNA_88K) {
		data = (char *)(NVRAM_ADDR + 0x80);

		for (i = 0; i < NNVSYM; i++) {
			for (j = 0; j < NVSYMLEN; j++) {
				buf[j] = *data;
				data += 4;
			}
			strlcpy(nvram[i].symbol, buf, sizeof(nvram[i].symbol));

			for (j = 0; j < NVVALLEN; j++) {
				buf[j] = *data;
				data += 4;
			}
			strlcpy(nvram[i].value, buf, sizeof(nvram[i].value));
		}
	} else if (machtype == LUNA_88K2) {
		page = (u_int8_t *)(NVRAM_ADDR_88K2 + 0x20);

		for (i = 0; i < NNVSYM; i++) {
			*page = (u_int8_t)i;

			data = (char *)NVRAM_ADDR_88K2;
			strlcpy(nvram[i].symbol, data, sizeof(nvram[i].symbol));

			data = (char *)(NVRAM_ADDR_88K2 + 0x10);
			strlcpy(nvram[i].value, data, sizeof(nvram[i].value));
		}
	}
}

char *
nvram_by_symbol(char *symbol)
{
	char *value;
	int i;

	value = NULL;

	for (i = 0; i < NNVSYM; i++) {
		if (strncmp(nvram[i].symbol, symbol, NVSYMLEN) == 0) {
			value = nvram[i].value;
			break;
		}
	}

	return value;
}

void
setlevel(unsigned int level)
{
	unsigned int set_value;
	int cpu = cpu_number();

	set_value = int_set_val[level];

#ifdef MULTIPROCESSOR
	if (cpu != master_cpu)
		set_value &= INT_SLAVE_MASK;
#endif

	*int_mask_reg[cpu] = set_value;
	luna88k_curspl[cpu] = level;
}

u_int
getipl(void)
{
	u_int curspl, psr;

	disable_interrupt(psr);
	curspl = luna88k_curspl[cpu_number()];
	set_psr(psr);
	return curspl;
}

unsigned
setipl(unsigned level)
{
	unsigned int curspl, psr;

	disable_interrupt(psr);
	curspl = luna88k_curspl[cpu_number()];
	setlevel(level);

	/*
	 * The flush pipeline is required to make sure the above write gets
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
	unsigned int curspl, psr;

	disable_interrupt(psr);
	curspl = luna88k_curspl[cpu_number()];
	if (curspl < level)
		setlevel(level);

	/*
	 * The flush pipeline is required to make sure the above write gets
	 * through the data pipe and to the hardware; otherwise, the next
	 * bunch of instructions could execute at the wrong spl protection.
	 */
	flush_pipeline();

	set_psr(psr);
	return curspl;
}
