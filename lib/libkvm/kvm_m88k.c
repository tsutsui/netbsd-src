/*	$NetBSD$ */
/*	$OpenBSD: kvm_m88k.c,v 1.3 2004/09/15 19:31:31 miod Exp $	*/
/*	NetBSD: kvm_alpha.c,v 1.2 1995/09/29 03:57:48 cgd Exp */

/*
 * Copyright (c) 1994, 1995 Carnegie-Mellon University.
 * All rights reserved.
 *
 * Author: Chris G. Demetriou
 *
 * Permission to use, copy, modify and distribute this software and
 * its documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 *
 * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND
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

#include <sys/param.h>
#include <sys/user.h>
#include <sys/proc.h>
#include <sys/stat.h>
#include <sys/kcore.h>
#include <unistd.h>
#include <stdlib.h>
#include <nlist.h>
#include <kvm.h>

#include <uvm/uvm_extern.h>
#include <machine/vmparam.h>
#include <machine/kcore.h>
#include <machine/mmu.h>
#include <machine/m8820x.h>

#include <limits.h>
#include <db.h>

#include "kvm_private.h"

void
_kvm_freevtop(kvm_t *kd)
{

	if (kd->vmst != NULL) {
		free(kd->vmst);
		kd->vmst = NULL;
	}
}

int
_kvm_initvtop(kvm_t *kd)
{
	cpu_kcore_hdr_t *h = kd->cpu_data;
	phys_ram_seg_t *ram;
	u_long root;
	int i;

	/* Check the version and size of cpu kcore. */
	if (kd->cpu_dsize < 2 * sizeof(uint32_t)) {
		_kvm_err(kd, 0, "short m88k CPU header");
		return -1;
	}
	if (h->version == 0) {
		_kvm_err(kd, 0, "m88k version 0 dump has no supervisor APR");
		return -1;
	}
	if (h->version != M88K_KCORE_VERSION) {
		_kvm_err(kd, 0, "unsupported m88k kcore version %u",
		    h->version);
		return -1;
	}
	if (kd->cpu_dsize < sizeof(*h)) {
		_kvm_err(kd, 0, "short m88k CPU header");
		return -1;
	}

	/* Check memry segment ranges. */
	for (i = 0; i < NPHYS_RAM_SEGS; i++) {
		ram = &h->ram_segs[i];
		if (ram->size == 0)
			continue;
		if (ram->start > 0xffffffffULL ||
		    ram->size > 0x100000000ULL - ram->start ||
		    ((ram->start | ram->size) & PAGE_MASK) != 0) {
			_kvm_err(kd, 0, "invalid m88k dump RAM segment");
			return -1;
		}
	}

	switch (h->cputype) {
	case CPU_88100:
		break;
	case CPU_88110:	/* XXX not implemented yet */
	default:
		_kvm_err(kd, 0, "unsupported m88k CPU type 0x%x", h->cputype);
		return -1;
	}

	if ((h->sapr & APR_V) == 0) {
		_kvm_err(kd, 0,
		    "unsupported supervisor APR with translation disabled");
		return -1;
	}

	root = h->sapr & PG_FRAME;
	if (_kvm_pa2off(kd, root) == (off_t)-1 ||
	    _kvm_pa2off(kd, root + SDT_SIZE - 1) == (off_t)-1)
		return -1;

	for (i = 0; i < BATC_MAX; i++) {
		uint32_t bat = h->dbatc[i];
		int j;

		if ((bat & (BATC_V | BATC_SO)) != (BATC_V | BATC_SO))
			continue;
		if ((bat & ~BATC_BLKMASK) >= (uint32_t)BATC8_VA) {
			_kvm_err(kd, 0, "unsupported hardwired-region DBATC");
			return -1;
		}
		for (j = 0; j < i; j++) {
			uint32_t prev = h->dbatc[j];
			if ((prev & (BATC_V | BATC_SO)) == (BATC_V | BATC_SO) &&
			    (prev & ~BATC_BLKMASK) == (bat & ~BATC_BLKMASK)) {
				_kvm_err(kd, 0, "duplicate supervisor DBATC");
				return -1;
			}
		}
	}

	return 0;
}

/* Read SDT/PDT entry at specified pa */
static int
read_table_entry(kvm_t *kd, u_long pa, uint32_t *entry)
{
	off_t off;

	off = _kvm_pa2off(kd, pa);
	if (off == (off_t)-1 ||
	    _kvm_pa2off(kd, pa + sizeof(*entry) - 1) == (off_t)-1)
		return -1;
	if (pread(kd->pmfd, entry, sizeof(*entry), off) != sizeof(*entry)) {
		_kvm_err(kd, 0, "cannot read m88k translation entry at 0x%lx",
		    pa);
		return -1;
	}
	return 0;
}

static int
_kvm_kvatop_88100(kvm_t *kd, u_long va, u_long *pa)
{
	cpu_kcore_hdr_t *h = kd->cpu_data;
	uint32_t ste, pte;
	u_long addr, frame, offset;
	int i;

	if (va >= (uint32_t)BATC8_VA) {
		_kvm_err(kd, 0, "unsupported hardwired device address 0x%lx",
		    va);
		return 0;
	}

	/*
	 * Check DBATC translations first.
	 *
	 * Note there is no proper way to handle IBATC translations
	 * if IBATC has different mappings from DBATC or SDT/PDT.
	 */
	for (i = 0; i < BATC_MAX; i++) {
		uint32_t bat = h->dbatc[i];
		if ((bat & (BATC_V | BATC_SO)) == (BATC_V | BATC_SO) &&
		    (bat & ~BATC_BLKMASK) == (va & ~BATC_BLKMASK)) {
			addr = (((bat >> BATC_PSHIFT) & 0x1fffU) <<
			    BATC_BLKSHIFT) | (va & BATC_BLKMASK);
			frame = addr & PG_FRAME;
			offset = addr & PAGE_MASK;
			goto check_frame;
		}
	}

	/*
	 * Translate PA to VA via SDT/PDT mappings.
	 */
	addr = (h->sapr & PG_FRAME) + SDTIDX(va) * sizeof(ste);
	if (read_table_entry(kd, addr, &ste) < 0)
		return 0;
	if ((ste & SG_V) == 0) {
		_kvm_err(kd, 0, "invalid SDT entry for 0x%lx", va);
		return 0;
	}
	addr = (ste & PG_FRAME) + PDTIDX(va) * sizeof(pte);

	if (read_table_entry(kd, addr, &pte) < 0)
		return 0;
	if ((pte & IND_MASKED) != 0) {
		_kvm_err(kd, 0, "unsupported indirect PTE for 0x%lx", va);
		return 0;
	}
	if ((pte & PG_V) == 0) {
		_kvm_err(kd, 0, "invalid PTE for 0x%lx", va);
		return 0;
	}
	frame = pte & PG_FRAME;
	offset = va & PAGE_MASK;

 check_frame:
	if (_kvm_pa2off(kd, frame) == (off_t)-1 ||
	    _kvm_pa2off(kd, frame + PAGE_MASK) == (off_t)-1)
		return 0;
	*pa = frame + offset;
	return PAGE_SIZE - offset;
}

int
_kvm_kvatop(kvm_t *kd, u_long va, u_long *pa)
{
	cpu_kcore_hdr_t *h = kd->cpu_data;

	switch (h->cputype) {
	case CPU_88100:
		return _kvm_kvatop_88100(kd, va, pa);
	case CPU_88110:	/* XXX not implemented yet */
	default:
		_kvm_err(kd, 0, "unsupported m88k CPU type 0x%x", h->cputype);
		return 0;
	}
}

off_t
_kvm_pa2off(kvm_t *kd, u_long pa)
{
	cpu_kcore_hdr_t *h = kd->cpu_data;
	phys_ram_seg_t *ram;
	off_t off = kd->dump_off;
	int i;

	for (i = 0; i < NPHYS_RAM_SEGS; i++) {
		ram = &h->ram_segs[i];
		if (pa >= ram->start && pa - ram->start < ram->size)
			return off + (pa - ram->start);
		off += ram->size;
	}
	_kvm_err(kd, 0, "physical address 0x%lx outside dump RAM", pa);
	return (off_t)-1;
}

/*
 * Machine-dependent initialization for ALL open kvm descriptors,
 * not just those for a kernel crash dump.  Some architectures
 * have to deal with these NOT being constants!  (i.e. m68k)
 */
int
_kvm_mdopen(kd)
        kvm_t   *kd;
{
        kd->usrstack = USRSTACK;
        kd->min_uva = VM_MIN_ADDRESS;
        kd->max_uva = VM_MAXUSER_ADDRESS;

        return 0;
}
