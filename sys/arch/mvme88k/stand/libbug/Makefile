#	$OpenBSD: Makefile,v 1.7 2004/01/26 19:48:33 miod Exp $

LIB=	bug

SRCS=	delay.c diskrd.c diskwr.c getbrdid.c inchr.c instat.c \
	netfopen.c netfread.c outln.c outstr.c putchar.c return.c rtc_rd.c

STAGE1_CFLAGS=	-DSTACK_ASM_OP="\".long ${STAGE1_RELOC}\"" -DSTAGE1
STAGE2_CFLAGS=	-DSTACK_ASM_OP="\".long ${STAGE2_RELOC}\""
SINGLE_CFLAGS=	-DSTACK_ASM_OP="\".long ${STAGE2_RELOC}\"" -DSTAGE1
BUGCRT_OBJS=	stage1crt.o stage2crt.o singlecrt.o
CLEANFILES+=	${BUGCRT_OBJS} ${BUGCRT_OBJS:=.tmp}

.include "../Makefile.booters"

# only needed during build
libinstall::

${BUGCRT_OBJS}: crt.c machine m88k

stage1crt.o: crt.c
	${CC} ${CFLAGS} ${STAGE1_CFLAGS} -c ${.CURDIR}/crt.c -o ${.TARGET}.tmp
	${LD} -x -r ${.TARGET}.tmp -o ${.TARGET}
	rm -f ${.TARGET}.tmp

stage2crt.o: crt.c
	${CC} ${CFLAGS} ${STAGE2_CFLAGS} -c ${.CURDIR}/crt.c -o ${.TARGET}.tmp
	${LD} -x -r ${.TARGET}.tmp -o ${.TARGET}
	rm -f ${.TARGET}.tmp

singlecrt.o: crt.c
	${CC} ${CFLAGS} ${SINGLE_CFLAGS} -c ${.CURDIR}/crt.c -o ${.TARGET}.tmp
	${LD} -x -r ${.TARGET}.tmp -o ${.TARGET}
	rm -f ${.TARGET}.tmp

all realall: lib${LIB}.a ${BUGCRT_OBJS}

.include <bsd.own.mk>
.undef DESTDIR
.include <bsd.lib.mk>
