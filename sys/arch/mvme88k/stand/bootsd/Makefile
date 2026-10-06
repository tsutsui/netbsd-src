#	from: @(#)Makefile	8.1 (Berkeley) 6/10/93
#	$OpenBSD: Makefile,v 1.13 2006/05/16 22:52:24 miod Exp $

SRCS=	boot.c conf.c version.c
PROG=	bootsd
LIBS=	${LIBSA} ${LIBBUG} ${LIBZ} ${DESTDIR}/usr/lib/libc.a \
	${DESTDIR}/usr/lib/libgcc.a
DPADD=	${LIBS}
SRTOBJ=	${STAGE2}
LDFLAGS+= -nostdlib

.include "../Makefile.booters"

.include <bsd.prog.mk>
