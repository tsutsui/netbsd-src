/*	$NetBSD: ptrace.S,v 1.4 2002/07/30 06:08:00 matt Exp $	*/

#include "SYS.h"

	.globl	_C_LABEL(errno)

ENTRY(ptrace)
#ifdef PIC
	/* Access libc's errno through libc's GOT, not the caller's GOT. */
	or	r10,r1,0
	or.u	r11,r0,hi16(.Lgot_ptrace#abdiff)
	bsr.n	.Lgot_ptrace
	 or	r11,r11,lo16(.Lgot_ptrace#abdiff)
.Lgot_ptrace:
	addu	r11,r11,r1
	or	r1,r10,0
	or.u	r10,r0,hi16(_C_LABEL(errno)#got_rel)
	or	r10,r10,lo16(_C_LABEL(errno)#got_rel)
	ld	r10,r11,r10
	st	r0,r10,0
#else
	or.u	r10,r0,hi16(_C_LABEL(errno))
	st	r0,r10,lo16(_C_LABEL(errno))
#endif
	__DO_SYSCALL(ptrace)
	br	CERROR
	jmp	r1
