#	from: @(#)Makefile	8.1 (Berkeley) 6/10/93
#	$OpenBSD: Makefile,v 1.11 2006/05/16 22:51:30 miod Exp $

DEFS=	-DSTAGE2_RELOC=${STAGE2_RELOC}
RELOC=	${STAGE1_RELOC}
LDFLAGS+= -S

SRCS=	bootxx.c conf.c version.c
PROG=	bootxx
LIBS=	${LIBSA} ${LIBBUG} ${LIBZ}
DPADD=	${LIBS}
SRTOBJ=	${STAGE1}

.include "../Makefile.booters"

.include <bsd.prog.mk>
