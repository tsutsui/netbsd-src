#	$OpenBSD: Makefile,v 1.6 2005/05/06 11:16:48 pedro Exp $

# from: @(#)Makefile	8.1 (Berkeley) 6/11/93
#
# Makefile for mvme88k links, tags file

TMVME88K=../mvme88k/tags
SMVME88K=../arch/mvme88k/dev/*.[ch] ../arch/mvme88k/include/*.[ch] \
	 ../arch/mvme88k/mvme88k/*.[ch]
AMVME88K=../arch/mvme88k/mvme88k/*.[sS]

# Directories in which to place tags links
DMVME88K=	conf dev include mvme88k

.include "../../kern/Make.tags.inc"

tags:
	-ctags -wdtf ${TMVME88K} ${COMM} ${SMVME88K}
	egrep "^ENTRY(.*)|^ALTENTRY(.*)" ${AMVME88K} | \
	    sed "s;\([^:]*\):\([^(]*\)(\([^, )]*\)\(.*\);\3	\1	/^\2(\3\4$$/;" \
		>> ${TMVME88K}
	sort -o ${TMVME88K} ${TMVME88K}

links::
	-for i in ${DMVME88K}; do \
	    (cd $$i && { rm -f tags; ln -s ${TMVME88K} tags; }) done

SUBDIR=	compile include stand 

.include <bsd.subdir.mk>
