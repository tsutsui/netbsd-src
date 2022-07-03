/*	$NetBSD: exec_mvme.c,v 1.12 2003/08/07 16:28:45 agc Exp $ */

/*-
 * Copyright (c) 1982, 1986, 1990, 1993
 *	The Regents of the University of California.  All rights reserved.
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
 * 	@(#)boot.c	8.1 (Berkeley) 6/10/93
 */

#include <sys/param.h>
#include <sys/reboot.h>
#include <machine/prom.h>

#include "loadfile.h"

#include "stand.h"
#include "libsa.h"

/* These must agree with what locore.s expects */
#define	KERN_LOADADDR	0x0	/* Offset added to kernel symbols */
#if 0
typedef void (*kentry_t)(int, u_int, u_int, u_int, int, u_long);
#else
typedef void (*kentry_t)(int, u_int, u_int, u_long, u_long, u_long);
#endif


/*ARGSUSED*/
void
exec_mvme(file, flag, part)
	char	*file;
	int	flag;
	int	part;
{
	kentry_t	entry;
	u_long		marks[MARK_MAX];
	int		fd;
	int		lflags;
	int		bootdev;

#ifdef DEBUG
	printf("file: %s, flag: %d, part: %d\n", file, flag, part);
#endif

	lflags = LOAD_KERNEL;
	if ((flag & RB_NOSYM) != 0 )
		lflags &= ~LOAD_SYM;

	marks[MARK_START] = KERN_LOADADDR;
	if ((fd = loadfile(file, marks, lflags)) == -1)
		return;
	close(fd);

	marks[MARK_END] = (((u_long) marks[MARK_END] + sizeof(int) - 1)) &
	    (-sizeof(int));

	printf("Start @ 0x%lx [%ld=0x%lx-0x%lx]...\n",
	    marks[MARK_ENTRY], marks[MARK_NSYM],
	    marks[MARK_SYM], marks[MARK_END]);

	entry = (kentry_t) marks[MARK_ENTRY];

#if 0 // XXX - Make mvme88k entry point same as 68k
	(*entry)(flag, bugargs.ctrl_addr, bugargs.ctrl_lun, bugargs.dev_lun,
	    part, marks[MARK_END]);
#else
	printf("Controller Address 0x%x\n", bugargs.ctrl_addr);
	if (flag & RB_HALT)
		_rtt();

	bootdev = (bugargs.ctrl_lun << 8) | (bugargs.dev_lun & 0xFF);

	(*entry)(flag, bugargs.ctrl_addr, bootdev, bugargs.cputyp, marks[MARK_SYM], marks[MARK_END]);
#endif

	printf("exec: kernel returned!\n");
	return;
}
