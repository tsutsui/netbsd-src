/*	$OpenBSD: crt0.c,v 1.10 2013/01/05 11:20:55 miod Exp $	*/

/*   
 *   Mach Operating System
 *   Copyright (c) 1991, 1992 Carnegie Mellon University
 *   Copyright (c) 1991, 1992 Omron Corporation
 *   All Rights Reserved.
 *   
 *   Permission to use, copy, modify and distribute this software and its
 *   documentation is hereby granted, provided that both the copyright
 *   notice and this permission notice appear in all copies of the
 *   software, derivative works or modified versions, and any portions
 *   thereof, and that both notices appear in supporting documentation.
 *   
 *   CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 *   CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND FOR
 *   ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 *   
 *   Carnegie Mellon requests users of this software to return to
 *   
 *    Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *    School of Computer Science
 *    Carnegie Mellon University
 *    Pittsburgh PA 15213-3890
 *   
 *   any improvements or extensions that they make and grant Carnegie Mellon 
 *   the rights to redistribute these changes.
 */

/*
 * When a program begins, r31 points to a structure passed by the kernel.
 *
 * This structure contains argc, the argv[] NULL-terminated array, and
 * the envp[] NULL-terminated array.
 */

#include <stdlib.h>

#include "common.h"

void __start(int, char **, char **, void (*)(void), const Obj_Entry *);

/* Preserve the loader's cleanup/Obj before replacing r3/r4 with argv/envp. */
__asm__ (
"	.text\n"
"	.align 8\n"
"	.globl _start\n"
"_start:\n"
"	or	r5, r3, r0\n"
"	or	r6, r4, r0\n"
"	ld	r2, r31, 0\n"
"	addu	r3, r31, 4\n"
"	lda	r4, r3[r2]\n"
"	br.n	__start\n"
"	 addu	r4, r4, 4\n"
);

void
__start(int argc, char **argv, char **envp, void (*cleanup)(void),
    const Obj_Entry *obj)
{
	char *ap;

	environ = envp;

	if ((ap = argv[0]) != NULL) {
		if ((__progname = _strrchr(ap, '/')) == NULL)
			__progname = ap;
		else
			++__progname;
	}

#ifdef DYNAMIC
	/* NetBSD common_elf checks Obj and registers the DSO cleanup first. */
	if (&_DYNAMIC != NULL)
		_rtld_setup(cleanup, obj);
#endif

#ifdef MCRT0
	atexit(_mcleanup);
	monstartup((u_long)&_eprol, (u_long)&_etext);
#endif

	atexit(_fini);
	_init();

__asm__ ("__callmain:");	/* Defined for the benefit of debuggers */
	exit(main(argc, argv, environ));
}

#include "common.c"
