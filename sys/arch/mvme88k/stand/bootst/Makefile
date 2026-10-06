#	from: @(#)Makefile	8.1 (Berkeley) 6/10/93
#	$OpenBSD: Makefile,v 1.12 2006/05/16 22:52:26 miod Exp $

CLEANFILES+=	stboot bootst

PROG=	bootst.bug
SRCS=	boot.c conf.c dev_tape.c rawfs.c version.c
LIBS=	${LIBSA} ${LIBBUG} ${LIBZ} ${DESTDIR}/usr/lib/libc.a \
	${DESTDIR}/usr/lib/libgcc.a
SRTOBJ=	${SINGLE}
DPADD=	${LIBS} ${WRTVID}
LDFLAGS+= -nostdlib --allow-multiple-definition

.include "../Makefile.booters"

bootst stboot: ${PROG} ${WRTVID}
	${WRTVID} ${PROG}

proginstall::
	${INSTALL_FILE} ${STRIPFLAG} -o ${BINOWN} -g ${BINGRP} -m ${BINMODE} \
		bootst ${DESTDIR}${MDEC_DIR}
	${INSTALL_FILE} ${STRIPFLAG} -o ${BINOWN} -g ${BINGRP} -m ${BINMODE} \
		stboot ${DESTDIR}${MDEC_DIR}

all realall: bootst

.include <bsd.prog.mk>
