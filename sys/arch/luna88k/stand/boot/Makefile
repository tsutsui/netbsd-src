#	$NetBSD$
#	$OpenBSD: Makefile,v 1.12 2019/12/10 11:28:13 aoyama Exp $
#	@(#)Makefile	8.2 (Berkeley) 8/15/93

NOMAN= # defined
NOPIE= # defined

.include <bsd.own.mk>

S= ${.CURDIR}/../../../..
LIBSADIR+=	${S}/lib/libsa

CPPFLAGS+=	-nostdinc -D_STANDALONE
CPPFLAGS+=	-I${.CURDIR} -I${.OBJDIR} -I${S} -I${S}/arch

CPPFLAGS+=	-DSUPPORT_DISK
CPPFLAGS+=	-DSUPPORT_ETHERNET
CPPFLAGS+=	-DSUPPORT_DHCP -DSUPPORT_BOOTP
#CPPFLAGS+=	-DBOOTP_DEBUG -DNETIF_DEBUG -DETHER_DEBUG -DNFS_DEBUG
#CPPFLAGS+=	-DRPC_DEBUG -DRARP_DEBUG -DNET_DEBUG -DDEBUG -DPARANOID

CFLAGS=		-Os
CFLAGS+=	-ffreestanding
CFLAGS+=	-Wall -Werror
CFLAGS+=	-Wstrict-prototypes -Wmissing-prototypes -Wpointer-arith

OBJCOPY?=	objcopy
TEXTADDR=	0x700000	# 7MB
LDSCRIPT=	${.CURDIR}/boot.ldscript
LINKFORMAT=	-static -N -Ttext ${TEXTADDR} -T ${LDSCRIPT} \
		--warn-common -nopie #-znorelro

SRCS=	locore.S
SRCS+=	init_main.c
SRCS+=	fault.c setjmp.S
SRCS+=	bcd.c
SRCS+=	devopen.c
SRCS+=	conf.c
SRCS+=	getline.c parse.c
SRCS+=	boot.c
SRCS+=	cons.c prf.c awaitkey.c
SRCS+=	sio.c
SRCS+=	bmc.c bmd.c font.c kbd.c
SRCS+=	sc.c sd.c
SRCS+=	ufs_disksubr.c
SRCS+=	strtol.c

# netboot support
SRCS+=	if_le.c lance.c getsecs.c
.PATH: ${LIBSADIR}
SRCS+=	dev_net.c

PROG=   boot

### find out what to use for libkern
KERN_AS=	library
.include "${S}/lib/libkern/Makefile.inc"

### find out what to use for libz
Z_AS=		library
.include "${S}/lib/libz/Makefile.inc"

### find out what to use for libsa
SA_AS=		library
SAMISCMAKEFLAGS+=SA_USE_LOADFILE=yes SA_USE_CREAD=yes SA_ENABLE_LS_OP=yes
.include "${S}/lib/libsa/Makefile.inc"

LIBS=	${SALIB} ${ZLIB} ${KERNLIB}

${PROG}: ${LDSCRIPT} ${OBJS} ${LIBS}
	${_MKTAGET_LINK}
	${LD} ${LINKFORMAT} -x -o ${PROG}.elf ${OBJS} ${LIBS}
	${OBJCOPY} -S -O a.out-m88k-netbsd --impure ${PROG}.elf ${PROG}

CLEANFILES+=	${PROG}.elf
CLEANFILES+=	machine m88k

.BEGIN: machine m88k
.NOPATH: machine m88k
realdepend realall: machine m88k

machine::
	-rm -f $@
	ln -s ${S}/arch/${MACHINE}/include $@

m88k::
	-rm -f $@
	ln -s ${S}/arch/m88k/include $@

cleandir distclean: .WAIT cleanlibdir

cleanlibdir:
	-rm -rf lib

# if there is a 'version' file, add rule for vers.c and add it to SRCS
# and CLEANFILES
.if exists(version)
.PHONY: vers.c
vers.c: version
	${_MKTARGET_CREATE}
	${HOST_SH} ${S}/conf/newvers_stand.sh ${.CURDIR}/version ${MACHINE}

SRCS+=  vers.c
CLEANFILES+= vers.c
.endif

.include <bsd.prog.mk>
