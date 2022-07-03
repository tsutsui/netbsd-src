#	$OpenBSD: Makefile,v 1.7 2004/01/26 19:48:33 miod Exp $

LIB=bug

NOPIC= # defined
NOLINT= # defined
NOPROFILE= # defined

S=${.CURDIR}/../../../..
DIR_SA=$S/lib/libsa

CFLAGS+=-I${.CURDIR}/../../include -I${DIR_SA} -I${.CURDIR}

SRCS=	delay.c diskrd.c diskwr.c getbrdid.c inchr.c instat.c \
	netfopen.c netfread.c outln.c outstr.c putchar.c return.c rtc_rd.c

# only needed during build
libinstall::

all realall: lib${LIB}.a

.include <bsd.lib.mk>
