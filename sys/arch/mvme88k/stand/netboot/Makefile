#	$OpenBSD: Makefile,v 1.14 2006/05/16 22:51:30 miod Exp $

#DEFS+=	-DNETIF_DEBUG

SRCS=	boot.c conf.c devopen.c if_ie.c
#SRCS+=	if_le.c
PROG=	netboot
LIBS=	${LIBSA} ${LIBBUG} ${LIBZ} ${DESTDIR}/usr/lib/libc.a \
	${DESTDIR}/usr/lib/libgcc.a
DPADD=	${LIBS}
SRTOBJ=	${SINGLE}
LDFLAGS+= -nostdlib
CLEANFILES+= netboot.bin

.include "../Makefile.booters"

netboot.bin: ${PROG}
	${STRIP} ${.OBJDIR}/${PROG}
	${OBJCOPY} -R.comment -R.note ${.OBJDIR}/${PROG} -O binary ${.TARGET}

all realall: netboot.bin

proginstall:: netboot.bin
	${INSTALL_FILE} ${STRIPFLAG} -o ${BINOWN} -g ${BINGRP} -m ${BINMODE} \
		${.OBJDIR}/netboot.bin ${DESTDIR}${MDEC_DIR}/netboot

.include <bsd.prog.mk>
