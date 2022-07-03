/*	$NetBSD: __vfork14.S,v 1.3 2002/07/30 06:07:59 matt Exp $	*/

#include "SYS.h"

/* r2 = pid. r3 = 0 in parent, 1 in child */

SYSCALL(__vfork14,__vfork14)
	bcnd	eq0,r3,parent
	or	r2,r0,0
	or	r3,r0,0
parent:
	jmp	r1		/* pid = fork() */
