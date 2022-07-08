#	$NetBSD$

# Makefile for luna88k tags file

# Find where m88k source files are for inclusion in tags
.include <../m88k/Makefile.inc>

TLUNA88K=	../luna88k/tags
SLUNA88K=	../luna88k/luna88k/*.[ch] ../luna88k/include/*.h \
		../luna88k/dev/*.[ch]
ALUNA88K=	../luna88k/luna88k/*.[sS]

# Directories in which to place tags links
DLUNA88K=	dev include

.include "../../kern/Make.tags.inc"

tags:
	-ctags -wdtf ${TLUNA88K} ${SLUNA88K} ${COMM}
	egrep "^ENTRY(.*)|^ALTENTRY(.*)" ${ALUNA88K} | \
	    sed "s;\([^:]*\):\([^(]*\)(\([^, )]*\)\(.*\);\3 \1 /^\2(\3\4$$/;" \
	    >> ${TLUNA88K}
	sort -o ${TLUNA88K} ${TLUNA88K}

links:
	-for i in ${DLUNA88K}; do \
	    cd $$i && rm -f tags; ln -s ../tags tags; done

SUBDIR=	compile include stand

.include <bsd.subdir.mk>
