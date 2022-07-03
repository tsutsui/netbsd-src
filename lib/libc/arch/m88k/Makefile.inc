#	$OpenBSD: Makefile.inc,v 1.2 2000/03/01 17:31:20 todd Exp $

KMINCLUDES=	arch/m88k/DEFS.h arch/m88k/SYS.h
#KMSRCS=	bcmp.S bzero.S ffs.S strcat.c strcmp.S strcpy.S strlen.S strncmp.S \
#	strncpy.S htonl.S htons.S ntohl.S ntohs.S
KMSRCS=	htonl.S htons.S ntohl.S ntohs.S

SRCS+=  __sigaction14_sigtramp.c __sigtramp1.S __sigtramp2.S

SRCS+=	moveDI41x.s moveDI43x.s moveDI45x.s moveDI47x.s moveHI15x.s moveQI16x.s \
	moveSI46x.s moveSI64n.s moveDI42x.s moveDI44x.s moveDI46x.s moveDI96x.s \
	moveHI48x.s moveSI45x.s moveSI47x.s moveSI96x.s

.SUFFIXES: .asm .s

.asm.s:
	grep -v '^;' $< | sed -e 's/#//g' > $@

$(SRCS:.s=.asm):
	@echo "Generating m88k move sources..."
	${NETBSDSRCDIR}/gnu/dist/gcc/gcc/config/m88k/m88k-move.sh -abi -no-tdesc
