/*	$NetBSD$	*/
/*	$OpenBSD: rtld_machine.c,v 1.5 2013/04/20 18:40:42 miod Exp $	*/

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
 * Copyright (c) 1999 Dale Rahn
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

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include "rtld.h"
#include "debug.h"

void _rtld_bind_start(void);
void _rtld_sync_icache(Elf_Addr, Elf_Addr);
caddr_t _rtld_bind(const Obj_Entry *, Elf_Word);
int _rtld_relocate_plt_objects(const Obj_Entry *);

/*
 * Adapt the local-symbol handling of OpenBSD _dl_md_reloc to the NetBSD
 * definition/Obj return interface. Other symbols use MI search order.
 */
static const Elf_Sym *
find_symdef_with_local(const Obj_Entry *obj, unsigned long symnum,
    const Obj_Entry **defobj, bool in_plt)
{
	const Elf_Sym *sym = obj->symtab + symnum;

	if (ELF_ST_BIND(sym->st_info) == STB_LOCAL &&
	    sym->st_shndx != SHN_UNDEF) {
		*defobj = obj;
		return sym;
	}
	return _rtld_find_symdef(symnum, obj, defobj, in_plt);
}

/*
 * OpenBSD _dl_md_reloc's non-PLT RELA loop, using NetBSD Obj tables and
 * lookup/error interfaces. MI handles text protection and deferred COPY.
 */
int
_rtld_relocate_nonplt_objects(const Obj_Entry *obj)
{
	const Elf_Rela *rela;
	const Elf_Sym *def;
	const Obj_Entry *defobj;
	Elf_Addr *where;
	unsigned long symnum;

	if (obj->rel != NULL) {
		_rtld_error("%s: m88k requires RELA relocations", obj->path);
		return -1;
	}
	for (rela = obj->rela; rela < obj->relalim; rela++) {
		symnum = ELF_R_SYM(rela->r_info);
		where = (Elf_Addr *)(obj->relocbase + rela->r_offset);
		switch (ELF_R_TYPE(rela->r_info)) {
		case R_TYPE(NONE):
			break;
		case R_TYPE(BBASED_32):
			if (symnum != 0) {
				_rtld_error("%s: nonzero BBASED_32 symbol",
				    obj->path);
				return -1;
			}
			*where = (Elf_Addr)obj->relocbase + rela->r_addend;
			break;
		case R_TYPE(32):
			if (symnum == 0)
				*where = rela->r_addend;
			else {
				def = find_symdef_with_local(obj, symnum,
				    &defobj, false);
				if (def == NULL)
					return -1;
				*where = (Elf_Addr)defobj->relocbase +
				    def->st_value + rela->r_addend;
			}
			break;
		case R_TYPE(COPY):
			if (obj->isdynamic) {
				_rtld_error("%s: COPY relocation in "
				    "shared library",
				    obj->path);
				return -1;
			}
			/* NetBSD MI processes main COPY after all objects. */
			break;
		default:
			_rtld_error("%s: unsupported m88k relocation %lu",
			    obj->path,
			    (unsigned long)ELF_R_TYPE(rela->r_info));
			return -1;
		}
	}
	return 0;
}

/*
 * GOTP_ENT relocations are special in that they define both a .got and a
 * .plt relocation. Adapted from OpenBSD _dl_md_reloc_gotp_ent with the
 * same store order, using typed GOT and PLT pointers.
 */
static void
reloc_gotp(Elf_Addr *got, uint16_t *plt, Elf_Addr value)
{

	/* .got update */
	*got = value;
	/* .plt update */
	plt[1] = (Elf_Addr)got >> 16;
	plt[3] = (Elf_Addr)got & 0xffff;
}

/*
 * Adapt OpenBSD _dl_md_reloc_got's lazy setup and _dl_md_reloc's eager
 * JMPREL loop to NetBSD's two MI hooks. Keep range/protection state here;
 * MI's setup hook supplies GOT metadata after both relocation hooks.
 * GOT stays RW under NetBSD's policy, without OpenBSD's RO GOT lock.
 */
static int
relocate_plt(const Obj_Entry *obj, bool bind_now)
{
	const Elf_Rela *rela;
	const Elf_Sym *def;
	const Obj_Entry *defobj;
	const Elf_Dyn *dynp;
	Elf_Addr start, end, page, size, got, plt, value, mapbase, mapend;
	Elf_Addr textend;
	unsigned int plttags;
	int result;

	start = end = 0;
	plttags = 0;
	result = 0;

	if (obj->pltrel != NULL) {
		_rtld_error("%s: m88k PLT requires RELA", obj->path);
		return -1;
	}
	if (obj->pltrela == obj->pltrelalim)
		return 0;
	/*
	 * Post-5.3 binaries use dynamic tags to provide the .plt boundaries.
	 * Read the paired tags here, since only this MD consumer needs them.
	 * The adopted producer makes the old symbol-search fallback needless.
	 */
	for (dynp = obj->dynamic; dynp->d_tag != DT_NULL; dynp++) {
		switch (dynp->d_tag) {
		case DT_88K_PLTSTART:
			start = (Elf_Addr)obj->relocbase + dynp->d_un.d_ptr;
			plttags |= 1;
			break;
		case DT_88K_PLTEND:
			end = (Elf_Addr)obj->relocbase + dynp->d_un.d_ptr;
			plttags |= 2;
			break;
		}
	}
	mapbase = (Elf_Addr)obj->mapbase;
	mapend = mapbase + obj->mapsize;
	textend = mapbase + obj->textsize;
	if (mapend < mapbase || textend < mapbase ||
	    obj->mapsize < sizeof(Elf_Addr) ||
	    plttags != 3 || end <= start || end - start < 12 ||
	    start < mapbase || end > textend) {
		_rtld_error("%s: invalid m88k PLT boundary tags", obj->path);
		return -1;
	}
	/* Validate this ABI's paired text/data writes before unprotecting. */
	for (rela = obj->pltrela; rela < obj->pltrelalim; rela++) {
		got = (Elf_Addr)obj->relocbase + rela->r_offset;
		plt = (Elf_Addr)obj->relocbase + rela->r_addend;
		if (ELF_R_TYPE(rela->r_info) != R_TYPE(GOTP_ENT) ||
		    got < mapbase || got > mapend - sizeof(Elf_Addr) ||
		    plt < start || plt > end - 12) {
			_rtld_error("%s: invalid m88k GOTP_ENT relocation",
			    obj->path);
			return -1;
		}
	}
	/* The unbiased lazy continuation already has its final addresses. */
	if (!bind_now && (Elf_Addr)obj->relocbase == 0)
		return 0;
	page = round_down(start);
	size = round_up(end) - page;
	/* GOT relocation will require PLT to be writeable. */
	if (mprotect((void *)page, size,
	    PROT_READ | PROT_WRITE | PROT_EXEC) == -1) {
		_rtld_error("%s: cannot write-enable m88k PLT", obj->path);
		return -1;
	}
	for (rela = obj->pltrela; rela < obj->pltrelalim; rela++) {
		got = (Elf_Addr)obj->relocbase + rela->r_offset;
		plt = (Elf_Addr)obj->relocbase + rela->r_addend;
		if (bind_now) {
			def = find_symdef_with_local(obj,
			    ELF_R_SYM(rela->r_info), &defobj, true);
			if (def == NULL) {
				result = -1;
				break;
			}
			value = (Elf_Addr)defobj->relocbase + def->st_value;
		} else
			value = *(Elf_Addr *)got + (Elf_Addr)obj->relocbase;
		reloc_gotp((Elf_Addr *)got, (uint16_t *)plt, value);
	}
	/*
	 * Force a cache sync on the whole PLT here, otherwise I$ might have
	 * stale information. Also sync after a partial lookup failure.
	 */
	_rtld_sync_icache(page, size);
	if (mprotect((void *)page, size, PROT_READ | PROT_EXEC) == -1) {
		_rtld_error("%s: cannot write-protect m88k PLT",
		    obj->path);
		return -1;
	}
	return result;
}

int
_rtld_relocate_plt_lazy(const Obj_Entry *obj)
{

	/* Relocate the OpenBSD lazy continuation before MI forces eager. */
	return relocate_plt(obj, false);
}

int
_rtld_relocate_plt_objects(const Obj_Entry *obj)
{

	return relocate_plt(obj, true);
}

/* OpenBSD _dl_md_reloc_got metadata, after the NetBSD MI hooks succeed. */
void
_rtld_setup_pltgot(const Obj_Entry *obj)
{
	obj->pltgot[1] = (Elf_Addr)obj;
	obj->pltgot[2] = (Elf_Addr)_rtld_bind_start;
}

/*
 * Adapted from OpenBSD _dl_bind. reloff is a byte offset into JMPREL.
 * Retained for upstream correspondence. MI currently forces eager on m88k.
 * GOT remains writable, following NetBSD; no OpenBSD GOT re-protection lock.
 */
caddr_t
_rtld_bind(const Obj_Entry *obj, Elf_Word reloff)
{
	const Elf_Rela *rela;
	const Elf_Sym *def;
	const Obj_Entry *defobj;
	Elf_Addr value;

	rela = (const Elf_Rela *)((const char *)obj->pltrela + reloff);
	def = find_symdef_with_local(obj, ELF_R_SYM(rela->r_info),
	    &defobj, true);
	if (def == NULL)
		_rtld_die();
	value = (Elf_Addr)defobj->relocbase + def->st_value;
	*(Elf_Addr *)(obj->relocbase + rela->r_offset) = value;
	return (caddr_t)value;
}
