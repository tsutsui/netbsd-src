/*	$OpenBSD: SYS.h,v 1.11 2004/07/28 08:48:15 miod Exp $*/
/*-
 * Copyright (c) 1990 The Regents of the University of California.
 * All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * the Systems Programming Group of the University of Utah Computer
 * Science Department.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	from: @(#)SYS.h	5.5 (Berkeley) 5/7/91
 */

#include <sys/cdefs.h>
#include <sys/syscall.h>
#include <machine/asm.h>

#ifdef __STDC__
#define	__ENTRY(p,x)	ENTRY(p##x)
#define	__SYSCALLNAME(p,x)	p##x
#define	__ALIAS(prefix,name)	WEAK_ALIAS(name,prefix##name)
#else
#define	__ENTRY(p,x)	ENTRY(p/**/x)
#define	__SYSCALLNAME(p,x)	p/**/x
#define	__ALIAS(prefix,name)	WEAK_ALIAS(name,prefix/**/name)
#endif

/*
 * XXX TKM - added the following comments - someone please verify.
 *
 * The system call code is passed in r13.
 *
 * The first 8 argument words are in registers r2-r9; the rest
 * are on the stack.  The first 3 stack-based argument words are
 * moved into r10-r12 for syscalls.  Syscalls with > 11 argument
 * words are not needed/allowed/supported on m88k.
 *
 * NetBSD/88k uses the SysV ABI, which reserves stack space for
 * the first 8 arguments even though they are passed in registers.
 * Note that OpenBSD does not reserve this stack space.
 */
#define	__DO_SYSCALL(x)							\
	ld r10,r31,0x20;						\
	ld r11,r31,0x24;						\
	ld r12,r31,0x28;						\
	or r13,r0,__SYSCALLNAME(SYS_,x);				\
	tb0 0, r0, 128

/*
 * The kernel side syscall implementation in sys/arch/m88k/m88k/trap.c
 * adjusts return address on normal return to skip the following branch
 * instruction to cerror, so no need to prepare special NOERROR() macro.
 */
#define	__SYSCALL__NOERROR(p,x,y)					\
	__ENTRY(p,x);							\
	__ALIAS(p,x);							\
	__DO_SYSCALL(y);						\
	br _C_LABEL(_cerror)

#define	__SYSCALL(p,x,y)						\
	__SYSCALL__NOERROR(p,x,y);

/*
 * System calls entry points are really named _thread_sys_{syscall},
 * and weakly aliased to the name {syscall}. This allows the thread
 * library to replace system calls at link time.
 */
#define SYSCALL(x,y)		__SYSCALL(_thread_sys_,x,y)
#define SYSCALL_NOERROR(x,y)	__SYSCALL__NOERROR(_thread_sys_,x,y)

#define	SYSENTRY(x)		__ENTRY(_thread_sys_,x);		\
				__ALIAS(_thread_sys_,x)

/*
 * Do a renamed or pseudo syscall (e.g., _exit()), where the entrypoint
 * and syscall name are not the same.
 */
#define	PSEUDO(x,y)		SYSCALL(x,y);				\
				jmp r1
#define	PSEUDO_NOERROR(x,y)	SYSCALL_NOERROR(x,y);			\
				jmp r1

/*
 * Do a normal syscall.
 */
#define	RSYSCALL(x)		PSEUDO(x,x)
#define	RSYSCALL_NOERROR(x)	PSEUDO_NOERROR(x,x)

/*
 * Do a syscall that has an internal name and a weak external alias.
 */
#ifdef WEAK_ALIAS
#define	WSYSCALL(weak,strong)						\
	WEAK_ALIAS(weak,strong);					\
	PSEUDO(strong,weak)
#else
#define	WSYSCALL(weak,strong)						\
	PSEUDO(weak,weak)
#endif

#define	ASMSTR		.asciz

	.globl	_C_LABEL(_cerror)
