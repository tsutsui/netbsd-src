/* $NetBSD: aout_machdep.h,v 1.3 2002/12/10 17:14:02 thorpej Exp $ */

#ifndef _M88K_EXEC_H_
#define	_M88K_EXEC_H_

/* Size of a page in an object file. */
#define AOUT_LDPGSZ        4096

struct relocation_info_m88k {
        unsigned int r_address;         /* offset in text or data segment */
        unsigned int r_symbolnum : 24,  /* ordinal number of add symbol */
                        r_extern :  1,  /* 1 if need to add symbol to value */
                        r_baserel : 1,
                        r_pcrel : 1,
                        r_jmptable : 1,
                        r_type : 4;

        int r_addend;
};
#define relocation_info relocation_info_m88k

#endif /* !_M88K_EXEC_H_ */
