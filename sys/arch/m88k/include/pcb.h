/*	$OpenBSD: pcb.h,v 1.2 2005/12/03 14:30:05 miod Exp $ */
/*
 * Copyright (c) 1996 Nivas Madhur
 * Mach Operating System
 * Copyright (c) 1993-1992 Carnegie Mellon University
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
 * any improvements or extensions that they make and grant Carnegie Mellon
 * the rights to redistribute these changes.
 */
/*
 * Motorola 88100 pcb definitions
 *
 */
/*
 */
#ifndef _M88K_PCB_H_
#define _M88K_PCB_H_

#include <machine/frame.h>

struct pcb
{
	struct switchframe	kernel_state;
	struct trapframe	user_state;
	int			pcb_onfault;
};
#define pcb_sf	kernel_state
#define pcb_tf  user_state

/*
 *	Location of saved user registers for the proc.
 */
#define	USER_REGS(l) \
	(((struct reg *)(&((l)->l_addr->u_pcb.user_state))))
/*
 * The pcb is augmented with machine-dependent additional data for
 * core dumps.  Note that the trapframe here is a copy of the one
 * from the top of the kernel stack (included here so that the kernel
 * stack itself need not be dumped).
 */
struct md_coredump {
	struct	trapframe md_tf;
};

#endif /* _M88K_PCB_H_ */
