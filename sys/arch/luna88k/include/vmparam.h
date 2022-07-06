/* $OpenBSD: vmparam.h,v 1.3 2004/04/26 14:31:08 miod Exp $ */
/* public domain */

#ifndef _MACHINE_VMPARAM_H_
#define _MACHINE_VMPARAM_H_

#include <m88k/vmparam.h>

/*
 * Constants which control the way the VM system deals with memory segments.
 * Only one physical contigous memory segment.
 */
#define VM_PHYSSEG_MAX          1
#define VM_PHYSSEG_STRAT        VM_PSTRAT_BSEARCH
#define VM_PHYSSEG_NOADD

#define VM_NFREELIST            1
#define VM_FREELIST_DEFAULT     0

#endif /* _MACHINE_VMPARAM_H_ */
