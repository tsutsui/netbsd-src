#	from: @(#)Makefile	8.1 (Berkeley) 6/10/93
#	$OpenBSD: Makefile,v 1.13 2006/05/16 22:52:24 miod Exp $

S=	${.CURDIR}/../../../..
DEFS=
INCPATH=-I${.CURDIR} -I${.CURDIR}/../libsa -I${.CURDIR}/../libbug \
	-I${.CURDIR}/../../include -I${S} -I${S}/lib/libsa
CFLAGS+=${INCPATH} ${DEFS} ${COPTS}
LDFLAGS+= -nostdlib -N -Ttext ${STAGE2_RELOC}
CLEANFILES+=bootsd

.include "${S}/arch/mvme88k/stand/bugcrt/Makefile.inc"
.include "${S}/arch/mvme88k/stand/libbug/Makefile.inc"
.include "${S}/arch/mvme88k/stand/libsa/Makefile.inc"
.include "${S}/arch/mvme88k/stand/libz/Makefile.inc"

SRCS=   boot.c conf.c version.c

LIBS=	${LIBSA} ${LIBBUG} ${LIBZ} ${DESTDIR}/usr/lib/libc.a ${DESTDIR}/usr/lib/libgcc.a

OBJS= ${SRCS:N*.h:R:S/$/.o/g}

BOOTS=	bootsd
ALL=	${BOOTS}

all: ${ALL}

bootsd: ${OBJS} ${STAGE2} ${LIBS}
	${LD} ${LDFLAGS} -o $@ \
		${STAGE2} ${OBJS} ${LIBS}
	@${SIZE} $@

install:
	${INSTALL_FILE} ${STRIPFLAG} -o ${BINOWN} -g ${BINGRP} -m ${BINMODE} \
	  ${.OBJDIR}/${BOOTS} ${DESTDIR}${MDEC_DIR}

.include <bsd.prog.mk>
