#	$NetBSD: Makefile,v 1.5 2019/01/27 05:16:38 dholland Exp $

HOSTPROGNAME=	${_TOOL_PREFIX}m88k-elf2aout
HOST_SRCDIR=	usr.bin/elf2aout
HOST_CPPFLAGS=	-I${.CURDIR}
CPPFLAGS+= -DTARGET_BYTE_ORDER=BIG_ENDIAN

.include "${.CURDIR}/../Makefile.host"
