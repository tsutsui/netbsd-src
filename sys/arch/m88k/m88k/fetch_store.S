/* $NetBSD$ */
/* $OpenBSD: subr.S,v 1.6 2005/05/15 16:23:45 miod Exp $	*/
/*
 * Mach Operating System
 * Copyright (c) 1993-1992 Carnegie Mellon University
 * Copyright (c) 1991 OMRON Corporation
 * Copyright (c) 1996 Nivas Madhur
 * Copyright (c) 1998 Steve Murphree, Jr.
 * Copyright (c) 2006 Timothy McIntosh
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

/* See fetch(9) for details. */

#include "assym.h"
#include <sys/cdefs.h>
#include <machine/asm.h>

/*
 * Macro containing common prologue code of all store and fetch routines.
 */
.macro install_fault_handler handler
	/*
	 * set up fault handler: 
	 *
	 * curpcb->pcb_onfault = fsufault
	 */
	ldcr	r5,   CPU
	ld	r6,   r5,   CI_CURPCB			/* r6 <- curpcb */

	or.u	r5,  r0,  hi16(_ASM_LABEL(\handler))
	or	r5,  r5,  lo16(_ASM_LABEL(\handler))	/* r5 <- fsufault */

	st	r5,  r6,  PCB_ONFAULT
.endm

/*
 * fetch/store user fault handler
 */
ASLOCAL(fsufault)
	br.n	_ASM_LABEL(fudone)
	 sub	r2,  r0,  1	/* return error (-1) */

/*
 * Common epilogue code for store (sudone) and fetch (fudone) routines.
 */
ASLOCAL(sudone)
	or	r2,  r0,  r0	/* return success (0) */
	/* FALLTHROUGH */
ASLOCAL(fudone)
	/*
	 * clear fault handler: 
	 *
	 * curpcb->pcb_onfault = NULL
	 */
	ldcr	r5,   CPU
	ld	r6,   r5,   CI_CURPCB			/* r6 <- curpcb */

	jmp.n	r1
	 st	r0,  r6,  PCB_ONFAULT

/*
 * int fubyte(const void *base);
 */
ENTRY(fubyte)
	install_fault_handler fsufault
#ifdef ERRATA__XXX_USR
	NOP
	ld.bu.usr r2,  r0,  r2
	NOP
	NOP
	NOP
	br	_ASM_LABEL(fudone)
#else
	br.n	_ASM_LABEL(fudone)
	 ld.bu.usr r2,  r0,  r2
#endif

/*
 * int fusword(const void *base);
 */
ENTRY(fusword)
	install_fault_handler fsufault
#ifdef ERRATA__XXX_USR
	NOP
	ld.hu.usr r2,  r0,  r2
	NOP
	NOP
	NOP
	br	_ASM_LABEL(fudone)
#else
	br.n	_ASM_LABEL(fudone)
	 ld.hu.usr r2,  r0,  r2
#endif

/*
 * long fuword(const void *base);
 */
ENTRY(fuword)
	install_fault_handler fsufault
#ifdef ERRATA__XXX_USR
	NOP
	ld.usr	r2,  r0,  r2
	NOP
	NOP
	NOP
	br	_ASM_LABEL(fudone)
#else
	br.n	_ASM_LABEL(fudone)
	 ld.usr	r2,  r0,  r2
#endif

/*
 * int fuswintr(const void *base);
 *
 * XXX  Current implementation always returns error.
 *      Used only by profiling code in subr_prof.c, which will work
 *	without this function.
 */
ENTRY(fuswintr)
	jmp.n	r1
	 sub	r2,  r0,  1	/* return error (-1) */


/*
 * int subyte(void *base, int c);
 */
ENTRY(subyte)
	install_fault_handler fsufault
#ifdef ERRATA__XXX_USR
	NOP
	st.b.usr r3,  r0,  r2
	NOP
	NOP
	NOP
	br	_ASM_LABEL(sudone)
#else
	br.n	_ASM_LABEL(sudone)
	 st.b.usr r3,  r0,  r2
#endif

/*
 * int susword(void *base, short c);
 */
ENTRY(susword)
	install_fault_handler fsufault
#ifdef ERRATA__XXX_USR
	NOP
	st.h.usr r3,  r0,  r2
	NOP
	NOP
	NOP
	br	_ASM_LABEL(sudone)
#else
	br.n	_ASM_LABEL(sudone)
	 st.h.usr r3,  r0,  r2
#endif

/*
 * int suword(void *base, long c);
 */
ENTRY(suword)
	install_fault_handler fsufault
#ifdef ERRATA__XXX_USR
	NOP
	st.usr	r3,  r0,  r2
	NOP
	NOP
	NOP
	br	_ASM_LABEL(sudone)
#else
	br.n	_ASM_LABEL(sudone)
	 st.usr	r3,  r0,  r2
#endif

/*
 * int suswintr(void *base, short c);
 *
 * XXX  Current implementation always returns error.
 *      Used only by profiling code in subr_prof.c, which will work
 *	without this function.
 */
ENTRY(suswintr)
	jmp.n	r1
	 sub	r2,  r0,  1	/* return error (-1) */
