#	$OpenBSD: Makefile.inc,v 1.10 2006/05/16 22:52:07 miod Exp $

MDEC_DIR?=/usr/mdec

.if defined(PROG) && (${PROG} == "wrtvid" || ${PROG} == "prtvid" || ${PROG} == "installboot")

.if ${MACHINE} == "mvme88k"
BINDIR=${MDEC_DIR}
DPADD=${LIBUTIL}
LDADD=-lutil
# Need this to work in the miniroot
LDSTATIC=-static
.else
NOPROG=
.endif

.else

# Load addresses for first and second stage bootstraps
STAGE1_RELOC=0x00080000
STAGE2_RELOC=0x00040000

S=${.CURDIR}/../../../..

COPTS?=		-O2 -Wall -Werror -Wno-main -Wno-uninitialized -Wmissing-prototypes -Wstrict-prototypes \
		-ffreestanding
CFLAGS+= -D_STANDALONE -DLIBSA_USE_MEMSET -DLIBSA_USE_MEMCPY -nostdinc -I${S} -I.

CLEANFILES+= machine m88k

.ifdef SRCS
${SRCS}: machine m88k
.endif

machine :
	-rm -f ${.TARGET}
	ln -s $S/arch/mvme88k/include machine

m88k :
	-rm -f ${.TARGET}
	ln -s $S/arch/m88k/include m88k

.if exists(${.CURDIR}/version)

SRCS+= vers.c
CLEANFILES+= vers.c

vers.c: version
	${HOST_SH} ${S}/conf/newvers_stand.sh ${.CURDIR}/version ${MACHINE}
.endif

.endif
