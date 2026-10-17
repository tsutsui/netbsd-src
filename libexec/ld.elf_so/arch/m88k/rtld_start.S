/*	$NetBSD$	*/
/*	$OpenBSD: ldasm.S,v 1.5 2013/04/05 12:58:03 kurt Exp $	*/
/*
 * Copyright (c) 2013 Miodrag Vallat.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
/*
 * Copyright (c) 2006 Dale Rahn
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <machine/asm.h>
/* sys/exec_elf.h contains C types and cannot be included by assembly. */
#define	AT_BASE	7	/* Interpreter base addr, from sys/exec_elf.h. */

/*
 * ld.elf_so entry point, adapted from OpenBSD _dl_start.
 * r31 points to argc, followed by argv, envp and kernel AuxInfo pairs.
 * NetBSD setregs enters at the first instruction; OpenBSD's two NOPs
 * padded an entry where setregs could skip up to two instructions.
 * NetBSD MI parses the original stack and returns cleanup and Obj in
 * the two words before argc, rather than using an AUX pointers array.
 */
#define	RTLD_RETURN_SIZE	(2 * 4)
#define	RTLD_FRAME_SIZE		16
#define	CLEANUP_OFFSET		(RTLD_FRAME_SIZE - RTLD_RETURN_SIZE)
#define	OBJ_OFFSET		(CLEANUP_OFFSET + 4)
ASENTRY(.rtld_start)
	or	r30, r31, 0
	/* Reserve the MI return slots and pad to 16-byte stack alignment. */
	subu	r31, r31, RTLD_FRAME_SIZE

	/* Scan argument and environment vectors to find auxiliary data. */
	ld	r2, r30, 0
	addu	r3, r30, 8
	lda	r3, r3[r2]
1:	ld	r4, r3, 0
	addu	r3, r3, 4
	bcnd	ne0, r4, 1b
	/* Kernel exec supplies AT_BASE, including a valid zero value. */
2:	ld	r4, r3, 0
	ld	r14, r3, 4
	addu	r3, r3, 8
	cmp	r5, r4, AT_BASE
	bb0	eq, r5, 2b

	/* Bootstrap the loader using AT_BASE and the VMA-zero link contract. */
	bsr.n	_rtld_bootstrap
	 or	r2, r14, 0		/* loader bias */
	subu	r2, r30, RTLD_RETURN_SIZE	/* MI return slots */
	bsr.n	_rtld
	 or	r3, r14, 0		/* loader bias */
	or	r11, r2, 0
	ld	r3, r31, CLEANUP_OFFSET
	ld	r4, r31, OBJ_OFFSET
	/* CSU receives OSP, cleanup and Obj; it prepares the C arguments. */
	or	r2, r30, 0
	jmp.n	r11
	 or	r31, r30, 0

/*
 * PLT resolver entry point.
 * Invoked with the following stack frame:
 *	r31(0x00)	zero
 *	r31(0x04)	ELF object
 *	r31(0x08)	saved r11
 *	r31(0x0c)	saved r1
 * All registers but r1 and r11 must be preserved. The resolver must return
 * to the resolved address with r1 restored.
 *
 * Retained OpenBSD _dl_bind_start frame, adapted to NetBSD Obj_Entry/binder.
 * MI forces eager binding on m88k, so this is not an active runtime path.
 * Enabling lazy binding requires the follow-up runtime review in SL-D007.
 */
#define	OBJECT_OFFSET	(4 * 1)
#define	R11_OFFSET	(4 * 2)
#define	R1_OFFSET	(4 * 3)
#define	PLT_FRAME_SIZE	(4 * 4)
#define	REG_SIZE	(4 * 12)
ENTRY(_rtld_bind_start)
	/*
	 * Preserve caller-saved registers.
	 */
	subu	r31, r31, REG_SIZE
	st.d	r2,  r31, 4 * 0
	st.d	r4,  r31, 4 * 2
	st.d	r6,  r31, 4 * 4
	st.d	r8,  r31, 4 * 6
	st	r10, r31, 4 * 8
	st.d	r12, r31, 4 * 10

	/*
	 * Invoke resolver entry point.
	 */
	ld	r2,  r31, REG_SIZE + OBJECT_OFFSET	/* Obj */
	bsr.n	_rtld_bind
	 ld	r3,  r31, REG_SIZE + R11_OFFSET	/* relocation byte offset */

	/*
	 * Preserve the resolved function address.
	 */
	or	r11, r2,  r0

	/*
	 * Restore caller-saved registers.
	 */
	ld.d	r12, r31, 4 * 10
	ld	r10, r31, 4 * 8
	ld.d	r8,  r31, 4 * 6
	ld.d	r6,  r31, 4 * 4
	ld.d	r4,  r31, 4 * 2
	ld.d	r2,  r31, 4 * 0
	ld	r1,  r31, REG_SIZE + R1_OFFSET

	jmp.n	r11
	 addu	r31, r31, REG_SIZE + PLT_FRAME_SIZE

/*
 * Adapted from OpenBSD _dl_cacheflush. Synchronize the address in r2 and
 * byte length in r3 with data writeback followed by I-cache invalidation.
 * Trap success skips the _cerror branch and returns through r1.
 */
ENTRY(_rtld_sync_icache)
	tb0	0, r0, 451
	br	_C_LABEL(_cerror)	/* never hit */
	jmp	r1
