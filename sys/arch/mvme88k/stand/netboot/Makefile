#	$OpenBSD: Makefile,v 1.14 2006/05/16 22:51:30 miod Exp $

SIZE?=	size
STRIP?= strip

S=	${.CURDIR}/../../../..
DEFS=
#DEFS+=-DNETIF_DEBUG
INCPATH=-I${.CURDIR} -I${.CURDIR}/../libsa -I${.CURDIR}/../libbug \
	-I${S} -I${S}/lib/libsa
CFLAGS+=${DEFS} ${INCPATH} ${COPTS}
CLEANFILES+=netboot netboot.bin

.include "${S}/arch/mvme88k/stand/bugcrt/Makefile.inc"
.include "${S}/arch/mvme88k/stand/libbug/Makefile.inc"
.include "${S}/arch/mvme88k/stand/libsa/Makefile.inc"
.include "${S}/arch/mvme88k/stand/libz/Makefile.inc"

SRCS=   boot.c conf.c devopen.c
SRCS+=	if_ie.c
#SRCS+=  if_le.c
OBJS=  ${SRCS:S/.c/.o/g}
LIBS= ${LIBSA} ${LIBBUG} ${LIBZ} ${DESTDIR}/usr/lib/libc.a ${DESTDIR}/usr/lib/libgcc.a
LDFLAGS+= -nostdlib -N -Ttext ${STAGE2_RELOC}

.include <bsd.prog.mk>

all:	netboot.bin

netboot: ${OBJS} ${SINGLE} ${LIBS}
	${LD} ${LDFLAGS} -o $@ \
		${SINGLE} ${OBJS} ${LIBS}
	@${SIZE} $@

netboot.bin: netboot
	${STRIP} ${.OBJDIR}/netboot
	${OBJCOPY} -R.comment -R.note ${.OBJDIR}/netboot -O binary ${.TARGET}

install:
	${INSTALL_FILE} ${STRIPFLAG} -o ${BINOWN} -g ${BINGRP} -m ${BINMODE} \
	  ${.OBJDIR}/netboot.bin ${DESTDIR}${MDEC_DIR}/netboot
