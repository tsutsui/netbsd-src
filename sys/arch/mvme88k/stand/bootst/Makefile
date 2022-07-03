#	from: @(#)Makefile	8.1 (Berkeley) 6/10/93
#	$OpenBSD: Makefile,v 1.12 2006/05/16 22:52:26 miod Exp $

S=	${.CURDIR}/../../../..
DEFS=
INCPATH=-I${.CURDIR} -I${.CURDIR}/../libsa -I${.CURDIR}/../libbug \
	-I${.CURDIR}/../../include -I${S} -I${S}/lib/libsa
CFLAGS+=${INCPATH} ${DEFS} ${COPTS}
LDFLAGS+= -nostdlib -N -Ttext ${STAGE2_RELOC}
LDFLAGS+= --allow-multiple-definition # XXX
CLEANFILES+=stboot bootst bootst.bug

.include "${S}/arch/mvme88k/stand/wrtvid/Makefile.inc"
.include "${S}/arch/mvme88k/stand/bugcrt/Makefile.inc"
.include "${S}/arch/mvme88k/stand/libbug/Makefile.inc"
.include "${S}/arch/mvme88k/stand/libsa/Makefile.inc"
.include "${S}/arch/mvme88k/stand/libz/Makefile.inc"

SRCS=   boot.c conf.c dev_tape.c rawfs.c version.c

LIBS=	${LIBSA} ${LIBBUG} ${LIBZ} ${DESTDIR}/usr/lib/libc.a ${DESTDIR}/usr/lib/libgcc.a

OBJS= ${SRCS:N*.h:R:S/$/.o/g}

BOOTS=	bootst stboot
ALL=	${BOOTS}

all: ${ALL}

bootst.bug:	${OBJS} ${SINGLE} ${LIBS}
	${LD} ${LDFLAGS} ${SINGLE} -o $@ \
	    ${OBJS} ${LIBS}
	@${SIZE} $@

bootst stboot: bootst.bug ${WRTVID}
	${WRTVID} bootst.bug

proginstall::
	${INSTALL_FILE} ${STRIPFLAG} -o ${BINOWN} -g ${BINGRP} -m ${BINMODE} \
		bootst ${DESTDIR}${MDEC_DIR}
	${INSTALL_FILE} ${STRIPFLAG} -o ${BINOWN} -g ${BINGRP} -m ${BINMODE} \
		stboot ${DESTDIR}${MDEC_DIR}

.include <bsd.prog.mk>
