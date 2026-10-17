/*	$NetBSD$	*/
/*	$OpenBSD: loader.c,v 1.132 2013/04/05 12:58:03 kurt Exp $ */

/*
 * Copyright (c) 1998 Per Fogelstrom, Opsycon AB
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

/*
 * Also taken from
 * $OpenBSD: archdep.h,v 1.1 2013/01/20 23:01:44 miod Exp $
 */

#include <sys/types.h>
#include <sys/exec_elf.h>

#include <stddef.h>

extern Elf32_Dyn _DYNAMIC[];
void _rtld_bootstrap(Elf32_Addr);

/*
 * Adapted from src/libexec/ld.so/m88k/archdep.h::RELOC_RELA.
 * The linked loader uses only symbol-zero BBASED_32 RELA entries. Like
 * the existing NetBSD self relocators, this inline operation relies on
 * that build contract rather than checking types or reporting errors.
 */
static inline void
reloc_self(const Elf32_Rela *rela, Elf32_Addr *where, Elf32_Addr relocbase)
{

	*where = relocbase + rela->r_addend;
}

/*
 * Adapted from src/libexec/ld.so/loader.c::_dl_boot_bind. Entry assembly
 * supplies AT_BASE instead of an auxiliary-data array. The loader is
 * linked at VMA zero, so AT_BASE is its load bias. Before relocation,
 * the _DYNAMIC GOT slot contains its link VMA.
 *
 * Scan the DYNAMIC section for the loader. Cache the data for easier
 * access, using only the RELA pointer and byte size needed here.
 * No symbol lookup, libc calls or relocated global data may be used.
 */
void
_rtld_bootstrap(Elf32_Addr relocbase)
{
	Elf32_Dyn *dynp;
	const Elf32_Rela *rela, *entry;
	Elf32_Addr size_bytes, offset_bytes;

	rela = NULL;
	size_bytes = 0;
	dynp = (Elf32_Dyn *)(relocbase + (Elf32_Addr)_DYNAMIC);
	for (; dynp->d_tag != DT_NULL; dynp++) {
		switch (dynp->d_tag) {
		case DT_RELA:
			rela = (const Elf32_Rela *)
			    (relocbase + dynp->d_un.d_ptr);
			break;
		case DT_RELASZ:
			size_bytes = dynp->d_un.d_val;
			break;
		}
	}
	if (size_bytes == 0)
		return;

	/* Do the bootstrap relocation using the source's byte-count loop. */
	for (offset_bytes = 0; offset_bytes < size_bytes;
	    offset_bytes += sizeof(*rela)) {
		entry = (const Elf32_Rela *)((const char *)rela + offset_bytes);
		reloc_self(entry, (Elf32_Addr *)(relocbase + entry->r_offset),
		    relocbase);
	}
}
