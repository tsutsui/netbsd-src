/* $OpenBSD: vmparam.h,v 1.27 2004/04/26 14:31:11 miod Exp $ */
/* public domain */
#ifndef _MVME88K_VMPARAM_H_
#define _MVME88K_VMPARAM_H_

#include <m88k/vmparam.h>

/*
 * Constants which control the way the VM system deals with memory segments.
 * The mvme88k port has two physical memory segments & associated freelists:
 *   - onboard RAM above 16MB (24-bit) boundary: VM_FREELIST_DEFAULT
 *   - onboard RAM below 16MB (24-bit) boundary: VM_FREELIST_24BIT
 */
#define	VM_PHYSSEG_MAX		2
#define	VM_PHYSSEG_STRAT	VM_PSTRAT_RANDOM
#define	VM_PHYSSEG_NOADD

#define VM_NFREELIST		2

#define VM_FREELIST_DEFAULT	0
#define VM_FREELIST_24BIT	1

#endif /* _MVME88K_VMPARAM_H_ */
