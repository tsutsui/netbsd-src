
#### xplx

TOOL_AWK?=	awk
TOOL_HEXDUMP?=	hexdump
HEXDUMP_OPT=	-v -e '"" 1/1 "0x%02x,\n"'

ZASM?=	zasm	# see "used language" comments in xplx.asm

.PHONY: all
all: xplx.inc xplxdefs.h # xplxfirm.o

xplx.rom xplx.lst: xplx.asm
	${ZASM} -uwy xplx.asm || rm -f xplx.rom xplx.lst

xplx.inc: xplx.rom
	${TOOL_HEXDUMP} ${HEXDUMP_OPT} xplx.rom > ${.TARGET}.tmp
	mv ${.TARGET}.tmp ${.TARGET}

xplxfirm.o: ../xplxfirm.c xplx.inc
	${CC} ${CFLAGS} -c ../xplxfirm.c -o ${.TARGET}

xplxdefs.h: mkdefs.awk xplx.lst
	${TOOL_AWK} -f mkdefs.awk < xplx.lst > ${.TARGET}.tmp
	mv ${.TARGET}.tmp ${.TARGET}

.PHONY:	clean
clean:
	rm -f xplxfirm.o xplx.rom xplx.lst xplx.inc
	rm -f *.tmp
