/*	$OpenBSD: db_machdep.h,v 1.10 2006/05/08 14:03:34 miod Exp $ */
/*
 * Mach Operating System
 * Copyright (c) 1993-1991 Carnegie Mellon University
 * Copyright (c) 1991 OMRON Corporation
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

#ifndef  _M88K_DB_MACHDEP_H_
#define  _M88K_DB_MACHDEP_H_

#include "opt_ddb.h"

#include <machine/pcb.h>
#include <machine/trap.h>
#include <sys/param.h>
#include <uvm/uvm_extern.h>

typedef	vaddr_t		db_addr_t;
typedef	long		db_expr_t;
typedef	struct reg	db_regs_t;
extern db_regs_t	ddb_regs;	/* register state */
#define	DDB_REGS	(&ddb_regs)

/* trap numbers used by ddb */
#define	DDB_ENTRY_BKPT_NO	130
#define	DDB_ENTRY_TRACE_NO	131
#define DDB_ENTRY_TRAP_NO	132

/*
 * Extract the real pc from the exception pipeline.
 */
#define	PC_REGS(regs)							\
	(CPU_IS88110 ? ((regs)->exip & XIP_ADDR) :			\
	 ((regs)->sxip & XIP_V ? (regs)->sxip & XIP_ADDR :		\
	  ((regs)->snip & NIP_V ? (regs)->snip & NIP_ADDR :		\
				   (regs)->sfip & FIP_ADDR)))

#define PC_ADVANCE(regs)						\
	do {								\
		(regs)->sxip += BKPT_SIZE;				\
		(regs)->snip += BKPT_SIZE + 4;				\
	} while (0)

#ifdef DDB

#define	BKPT_ADDR(addr)	(addr)		/* breakpoint address */
#define BKPT_SIZE	(4)	/* number of bytes in bkpt inst. */
#define BKPT_INST	(0xf000d000 | DDB_ENTRY_BKPT_NO) /* tb0, 0,r0, 130 */
#define BKPT_SET(inst)	(BKPT_INST)

/* Entry trap for the debugger - used for inline assembly breaks*/
#define DDB_ENTRY_ASM		__asm__ __volatile__ ( "tb0 0, r0, %0" : : "i" (DDB_ENTRY_TRAP_NO) )

int	inst_load(u_int);
int	inst_store(u_int);
int	ddb_break_trap(int, db_regs_t *);
int	ddb_entry_trap(int, db_regs_t *);
void	m88k_print_instruction(int, u_int, u_int32_t);	/* db_disasm.c */

/*
 * inst_call(ins) - is the instruction a function call.
 * Could be either bsr or jsr.
 */
#define	inst_call(I) \
	(((I) & 0xf8000000) == 0xc8000000 /* bsr */ || \
	 ((I) & 0xfffffbe0) == 0xf400c800 /* jsr */)
/*
 * inst_return(ins) - is the instruction a function call return.
 * Not mutually exclusive with inst_branch. Should be a jmp r1.
 */
#define	inst_return(I)	(((I) & 0xfffffbff) == 0xf400c001)

/*
 * inst_trap_return(ins) - is the instruction a return from trap.
 * Should be a rte.
 */
#define	inst_trap_return(I)	((I) == 0xf400c000)

/* breakpoint/watchpoint foo */
#define IS_BREAKPOINT_TRAP(type,code) ((type)==T_KDB_BREAK)
#define IS_WATCHPOINT_TRAP(type,code) 0

/* machine specific commands have been added to ddb */
#define DB_MACHINE_COMMANDS

/*
 * We use Elf32 symbols in DDB.
 */
#define	DB_ELF_SYMBOLS
#define	DB_ELFSIZE	32

/* XXX - TKM
#define	DB_AOUT_SYMBOLS
*/

#endif	/* DDB */

#endif	/* _M88K_DB_MACHDEP_H_ */
