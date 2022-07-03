/*	$OpenBSD: bugio.c,v 1.17 2006/05/02 21:43:08 miod Exp $ */
/*  Copyright (c) 1998 Steve Murphree, Jr. */

#include <sys/param.h>
#include <sys/systm.h>

#include <machine/asm_macro.h>
#include <machine/bugio.h>
#include <machine/prom.h>

register_t ossr0, ossr1, ossr2, ossr3;
register_t bugsr3;

unsigned long bugvec[2], sysbugvec[2];

void	bugsyscall(unsigned int call);

void bug_vector(void);
void sysbug_vector(void);

static int
bugenviron(struct prom_environ_hdr* buf, unsigned char size, unsigned char operation);

#ifdef MULTIPROCESSOR
#include <sys/lock.h>
__cpu_simple_lock_t bug_lock = __SIMPLELOCK_UNLOCKED;
#define	BUG_LOCK()	__cpu_simple_lock(&bug_lock)
#define	BUG_UNLOCK()	__cpu_simple_unlock(&bug_lock)
#else
#define	BUG_LOCK()	do { } while (0)
#define	BUG_UNLOCK()	do { } while (0)
#endif

void
bug_vector()
{
	unsigned long *vbr;

	__asm__ __volatile__ ("ldcr %0, cr7" : "=r" (vbr));
	vbr[2 * MVMEPROM_VECTOR + 0] = bugvec[0];
	vbr[2 * MVMEPROM_VECTOR + 1] = bugvec[1];
}

void
sysbug_vector()
{
	unsigned long *vbr;

	__asm__ __volatile__ ("ldcr %0, cr7" : "=r" (vbr));
	vbr[2 * MVMEPROM_VECTOR + 0] = sysbugvec[0];
	vbr[2 * MVMEPROM_VECTOR + 1] = sysbugvec[1];
}

#define	BUGCTXT()							\
{									\
	BUG_LOCK();							\
	disable_interrupt(psr);			/* paranoia */		\
	bug_vector();							\
	__asm__ __volatile__ ("ldcr %0, cr17" : "=r" (ossr0));		\
	__asm__ __volatile__ ("ldcr %0, cr18" : "=r" (ossr1));		\
	__asm__ __volatile__ ("ldcr %0, cr19" : "=r" (ossr2));		\
	__asm__ __volatile__ ("ldcr %0, cr20" : "=r" (ossr3));		\
									\
	__asm__ __volatile__ ("stcr %0, cr20" :: "r"(bugsr3));		\
}

#define	OSCTXT()							\
{									\
	__asm__ __volatile__ ("ldcr %0, cr20" : "=r" (bugsr3));		\
									\
	__asm__ __volatile__ ("stcr %0, cr17" :: "r"(ossr0));		\
	__asm__ __volatile__ ("stcr %0, cr18" :: "r"(ossr1));		\
	__asm__ __volatile__ ("stcr %0, cr19" :: "r"(ossr2));		\
	__asm__ __volatile__ ("stcr %0, cr20" :: "r"(ossr3));		\
	sysbug_vector();						\
	set_psr(psr);							\
	BUG_UNLOCK();							\
}

void
bugpcrlf(void)
{
	u_int psr;

	BUGCTXT();
	MVMEPROM_CALL(MVMEPROM_OUTCRLF);
	OSCTXT();
}

void
buginit()
{
	__asm__ __volatile__ ("ldcr %0, cr20" : "=r" (bugsr3));
}

char
buginchr(void)
{
	u_int psr;
	int ret;

	BUGCTXT();
	MVMEPROM_CALL(MVMEPROM_INCHR);
	__asm__ __volatile__ ("or %0,r0,r2" : "=r" (ret));
	OSCTXT();
	return ((char)ret & 0xff);
}

void
bugoutchr(int c)
{
	u_int psr;

	BUGCTXT();
	__asm__ __volatile__ ("or r2,r0,%0" : : "r" (c));
	MVMEPROM_CALL(MVMEPROM_OUTCHR);
	OSCTXT();
}

/* return 1 if not empty else 0 */
int
buginstat(void)
{
	u_int psr;
	int ret;

	BUGCTXT();
	MVMEPROM_CALL(MVMEPROM_INSTAT);
	__asm__ __volatile__  ("or %0,r0,r2" : "=r" (ret));
	OSCTXT();
	return ((ret & 0x08) >> 3);
}

void
bugoutstr(char *s, char *se)
{
	u_int psr;

	BUGCTXT();
	MVMEPROM_CALL(MVMEPROM_OUTSTR);
	OSCTXT();
}

void
bugrtcrd(struct mvmeprom_time *rtc)
{
	u_int psr;

	BUGCTXT();
	MVMEPROM_CALL(MVMEPROM_RTC_RD);
	OSCTXT();
}

static int
bugenviron(struct prom_environ_hdr* buf, unsigned char size, unsigned char operation)
{
        u_int psr;
	int   ret;

	BUGCTXT();
	__asm__ __volatile__ ("or r2, r0, %0;"
			       "or r3, r0, %1;"
			       "or r4, r0, %2"
				:
				: "r" (buf), "r" (size), "r" (operation)
				: "r2", "r3", "r4"
			      );
	MVMEPROM_CALL(MVMEPROM_ENVIRON);
	__asm__ __volatile__  ("or %0,r0,r2" : "=r" (ret) : );
	OSCTXT();

#ifdef DEBUG
	printf("bugenviron: ret = %x\n", ret);
#endif
	// XXX not sure what is actually returned on success here. -TKM
	return (operation == 0 ? ret : (ret != -1 ? 0 : 1));
}

int
bugenvgetvar(struct prom_environ_hdr* pkt)
{
	unsigned char sz;
	struct prom_environ_hdr* env;

	sz = bugenviron(NULL, 0, ENVIRONCMD_GETSIZE);

#ifdef DEBUG
	printf("bugenvgetvar: env size %d\n", sz);
#endif

	env = __builtin_alloca(sz);

	if (env != NULL)
	{
		int ret;

		ret = bugenviron(env, sz, ENVIRONCMD_READ);

		if (ret == 0)
		{
			while (env->type != 0)
			{
#ifdef DEBUG
				printf("bugenvgetvar: packet type %d\n", env->type);
#endif
				if (env->type == pkt->type)
				{
					bcopy(env, pkt, sizeof *env + env->len);
					return 0;
				}

				env = (struct prom_environ_hdr*)((char*)env + sizeof(*env) + env->len);
			}
		}
	}

	return 1;
}

void
bugreturn(void)
{
	u_int psr;

	BUGCTXT();
	MVMEPROM_CALL(MVMEPROM_EXIT);
	OSCTXT();
}

void
bugbrdid(struct mvmeprom_brdid *id)
{
	u_int psr;
	struct mvmeprom_brdid *ptr;

	BUGCTXT();
	MVMEPROM_CALL(MVMEPROM_GETBRDID);
	__asm__ __volatile__ ("or %0,r0,r2" : "=r" (ptr));
	OSCTXT();

	bcopy(ptr, id, sizeof(struct mvmeprom_brdid));
}

void
bugdiskrd(struct mvmeprom_dskio *dio)
{
	u_int psr;

	BUGCTXT();
	__asm__ __volatile__ ("or r2, r0, %0" : : "r" (dio));
	MVMEPROM_CALL(MVMEPROM_DSKRD);
	OSCTXT();
}

#ifdef MULTIPROCESSOR

/*
 * Ask the BUG to start a particular cpu at our provided address.
 */
int
spin_cpu(cpuid_t cpu, vaddr_t address)
{
	u_int psr;
	int ret;

	BUGCTXT();
	__asm__ __volatile__ ("or r2, r0, %0" : : "r" (cpu));
	__asm__ __volatile__ ("or r3, r0, %0" : : "r" (address));
	MVMEPROM_CALL(MVMEPROM_FORKMPU);
	__asm__ __volatile__ ("or %0,r0,r2" : "=r" (ret));
	OSCTXT();

	return (ret);
}

#endif	/* MULTIPROCESSOR */
