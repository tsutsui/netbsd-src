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
