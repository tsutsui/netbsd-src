/*	$NetBSD: procfs_machdep.c,v 1.2 2005/01/01 17:11:39 chs Exp $ */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: procfs_machdep.c,v 1.2 2005/01/01 17:11:39 chs Exp $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/mount.h>
#include <sys/vnode.h>
#include <miscfs/procfs/procfs.h>

#include <m88k/param.h>

/*
 * Linux-style /proc/cpuinfo.
 * Only used when procfs is mounted with -o linux.
 */
int
procfs_getcpuinfstr(char *buf, int *len)
{
	*len = 0;
	const char *cpu;

	switch (curcpu()->ci_cpuid) {
	case CPU_88100:
		cpu = "88100";
		break;
	case CPU_88110:
		cpu = "88110";
		break;
	default:
		cpu = "88000";
		break;
	}

	*len = snprintf(buf, sizeof(buf),
	    /* as seen in Linux 2.4.27 */
	    "CPU:\t\t%s\n",
	    /*
	     * in Linux m68k /proc/cpuinfo there are also "MMU", "FPU", "Clocking",
	     * "BogoMips" and "Calibration".
	     */
	    cpu);

	return 0;
}
