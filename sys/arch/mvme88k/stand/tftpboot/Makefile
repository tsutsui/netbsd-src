#	$OpenBSD: Makefile,v 1.3 2006/05/16 22:51:30 miod Exp $

SIZE?=	size
STRIP?= strip

S=	${.CURDIR}/../../../..
DEFS=
#DEFS+=-DNETIF_DEBUG
INCPATH=-I${.CURDIR} -I${.CURDIR}/../libsa -I${.CURDIR}/../libbug \
	-I${S} -I${S}/lib/libsa
CFLAGS+=${DEFS} ${INCPATH} ${COPTS}
CLEANFILES+=tftpboot tftpboot.bin tftpboot.sym

.include "${S}/arch/mvme88k/stand/bugcrt/Makefile.inc"
.include "${S}/arch/mvme88k/stand/libbug/Makefile.inc"
.include "${S}/arch/mvme88k/stand/libsa/Makefile.inc"
.include "${S}/arch/mvme88k/stand/libz/Makefile.inc"

SRCS=   boot.c conf.c tftpfs.c netdev.c
OBJS=  ${SRCS:S/.c/.o/g}
LIBS= ${LIBSA} ${LIBBUG} ${LIBZ}
LDFLAGS+= -nostdlib -N -Ttext ${STAGE2_RELOC}
LDFLAGS+= --allow-multiple-definition # XXX

.include <bsd.prog.mk>

all:	tftpboot.bin

tftpboot: ${OBJS} ${SINGLE} ${LIBS}
	${LD} ${LDFLAGS} -o $@ \
		${SINGLE} ${OBJS} ${LIBS}
	@${SIZE} $@

tftpboot.bin: tftpboot
#	${STRIP} ${.OBJDIR}/tftpboot
	${OBJCOPY} -R.comment -R.note ${.OBJDIR}/tftpboot -O binary ${.TARGET}

install: tftpboot.bin
	${INSTALL_FILE} ${STRIPFLAG} -o ${BINOWN} -g ${BINGRP} -m ${BINMODE} \
	  ${.OBJDIR}/tftpboot.bin ${DESTDIR}${MDEC_DIR}/tftpboot
