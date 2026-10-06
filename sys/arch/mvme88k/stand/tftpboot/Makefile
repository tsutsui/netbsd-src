#	$OpenBSD: Makefile,v 1.3 2006/05/16 22:51:30 miod Exp $

#DEFS+=	-DNETIF_DEBUG

SRCS=	boot.c conf.c tftpfs.c netdev.c
PROG=	tftpboot
LIBS=	${LIBSA} ${LIBBUG} ${LIBZ}
DPADD=	${LIBS}
SRTOBJ=	${SINGLE}
LDFLAGS+= -nostdlib --allow-multiple-definition
CLEANFILES+= tftpboot.bin tftpboot.sym

.include "../Makefile.booters"

tftpboot.bin: ${PROG}
#	${STRIP} ${.OBJDIR}/${PROG}
	${OBJCOPY} -R.comment -R.note ${.OBJDIR}/${PROG} -O binary ${.TARGET}

all realall: tftpboot.bin

proginstall:: tftpboot.bin
	${INSTALL_FILE} ${STRIPFLAG} -o ${BINOWN} -g ${BINGRP} -m ${BINMODE} \
		${.OBJDIR}/tftpboot.bin ${DESTDIR}${MDEC_DIR}/tftpboot

.include <bsd.prog.mk>
