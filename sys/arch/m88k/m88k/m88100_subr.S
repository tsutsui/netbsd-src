/* $NetBSD$ */
/* $OpenBSD: subr.S,v 1.6 2005/05/15 16:23:45 miod Exp $	*/
/*
 * Mach Operating System
 * Copyright (c) 1993-1992 Carnegie Mellon University
 * Copyright (c) 1991 OMRON Corporation
 * Copyright (c) 1996 Nivas Madhur
 * Copyright (c) 1998 Steve Murphree, Jr.
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

#include "assym.h"
#include <sys/cdefs.h>
#include <machine/asm.h>

/*
 * DO_LOAD_ADDRESS
 *
 * 	unsigned int do_load_word(address, supervisor_mode)
 *		vaddr_t address;		\\ in r2
 *		boolean_t supervisor_mode;	\\ in r3
 *
 * Return the word at ADDRESS (from user space if SUPERVISOR_MODE is zero,
 * supervisor space if non-zero).
 *
 */

ENTRY(do_load_word)	/* do_load_word(address, supervisor) */
	bcnd	ne0,r3,1f
#ifdef ERRATA__XXX_USR
	NOP
	ld.usr	r2,r2,r0
	NOP
	NOP
	NOP
	jmp	r1
#else
	jmp.n	r1
	 ld.usr	r2,r2,r0
#endif
1:	jmp.n	r1
	 ld	r2,r2,r0

ENTRY(do_load_half)	/* do_load_half(address, supervisor) */
	bcnd	ne0,r3,1f
#ifdef ERRATA__XXX_USR
	NOP
	ld.h.usr	r2,r2,r0
	NOP
	NOP
	NOP
	jmp	r1
#else
	jmp.n	r1
	 ld.h.usr	r2,r2,r0
#endif
1:	jmp.n	r1
	 ld.h	r2,r2,r0

ENTRY(do_load_byte)	/* do_load_byte(address, supervisor) */
	bcnd	ne0,r3,1f
#ifdef ERRATA__XXX_USR
	NOP
	ld.b.usr	r2,r2,r0
	NOP
	NOP
	NOP
	jmp	r1
#else
	jmp.n	r1
	 ld.b.usr	r2,r2,r0
#endif
1:	jmp.n	r1
	 ld.b	r2,r2,r0

ENTRY(do_store_word)	/* do_store_word(address, data, supervisor) */
	bcnd	ne0,r4,1f
#ifdef ERRATA__XXX_USR
	NOP
	st.usr	r3,r2,r0
	NOP
	NOP
	NOP
	jmp	r1
#else
	jmp.n	r1
	 st.usr	r3,r2,r0
#endif
1:	jmp.n	r1
	 st	r3,r2,r0

ENTRY(do_store_half)	/* do_store_half(address, data, supervisor) */
	bcnd	ne0,r4,1f
#ifdef ERRATA__XXX_USR
	NOP
	st.h.usr	r3,r2,r0
	NOP
	NOP
	NOP
	jmp	r1
#else
	jmp.n	r1
	 st.h.usr	r3,r2,r0
#endif
1:	jmp.n	r1
	 st.h	r3,r2,r0

ENTRY(do_store_byte)	/* do_store_byte(address, data, supervisor) */
	bcnd	ne0,r4,1f
#ifdef ERRATA__XXX_USR
	NOP
	st.b.usr	r3,r2,r0
	NOP
	NOP
	NOP
	jmp	r1
#else
	jmp.n	r1
	 st.b.usr	r3,r2,r0
#endif
1:	jmp.n	r1
	 st.b	r3,r2,r0

ENTRY(do_xmem_word)	/* do_xmem_word(address, data, supervisor) */
	bcnd	ne0,r4,1f
#ifdef ERRATA__XXX_USR
	NOP
#endif
	xmem.usr	r3,r2,r0
#ifdef ERRATA__XXX_USR
	NOP
	NOP
	NOP
#endif
	jmp.n	r1
	 or	r2, r3, r0
1:	xmem	r3, r2, r0
	jmp.n	r1
	 or	r2, r3, r0

ENTRY(do_xmem_byte)	/* do_xmem_byte(address, data, supervisor) */
	bcnd	ne0,r4,1f
#ifdef ERRATA__XXX_USR
	NOP
#endif
	xmem.bu.usr	r3,r2,r0
#ifdef ERRATA__XXX_USR
	NOP
	NOP
	NOP
#endif
	jmp.n	r1
	 or	r2, r3, r0
1:	xmem.bu	r3, r2, r0
	jmp.n	r1
	 or	r2, r3, r0
