/*	$NetBSD: pipe.S,v 1.7 2002/07/30 06:08:00 matt Exp $	*/

#include "SYS.h"

#ifdef WEAK_ALIAS
WEAK_ALIAS(pipe, _pipe)
#endif

/*
 * pipe(fildes) int fildes[2];
 */
ENTRY(_pipe)
	or	r6,r0,r2	| save pointer
        or      r13,r0,__SYSCALLNAME(SYS_,pipe)
	tb0	0,r0,128	| assume, that r6 is kept  XXX - is this true? -TKM
	br      _C_LABEL(_cerror)
 	st	r2,r6,0		| success, store fds
	st	r3,r6,4
	jmp.n	r1
	 or	r2,r0,r0	| and return 0
