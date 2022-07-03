#	$OpenBSD: Makefile.inc,v 1.8 2004/04/01 20:39:59 miod Exp $

SRCS+=	__main.c __assert.c bswap16.c bswap32.c bswap64.c \
	imax.c imin.c lmax.c lmin.c max.c min.c ulmax.c ulmin.c \
	memchr.c memcmp.c memset.c bcmp.c random.c \
	strcmp.c strlcat.c strcpy.c strlcpy.c strlen.c strncmp.c \
	strncpy.c scanc.c skpc.c strncasecmp.c ffs.c

SRCS+=  bcopy.S bzero.S memcpy.S ovbcopy.S setjmp.S

SRCS+=	moveDI41x.s moveDI43x.s moveDI45x.s moveDI47x.s moveHI15x.s moveQI16x.s \
	moveSI46x.s moveSI64n.s moveDI42x.s moveDI44x.s moveDI46x.s moveDI96x.s \
	moveHI48x.s moveSI45x.s moveSI47x.s moveSI96x.s

.SUFFIXES: .asm .s

.asm.s:
	grep -v '^;' $< | sed -e 's/#//g' > $@

$(SRCS:.s=.asm):
	@echo "Generating m88k move sources..."
	${NETBSDSRCDIR}/gnu/dist/gcc/gcc/config/m88k/m88k-move.sh -abi -no-tdesc
