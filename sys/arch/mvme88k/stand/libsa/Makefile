#	$OpenBSD: Makefile,v 1.7 2006/05/16 22:51:30 miod Exp $

LIB=sa

NOPIC= # defined
NOLINT= # defined
NOPROFILE= # defined

# Logically src/sys
S=${.CURDIR}/../../../..
DIR_SA=$S/lib/libsa
DIR_KERN=$S/lib/libkern

SRC_net= arp.c \
	 bootp.c  bootparam.c \
	 ether.c \
	 in_cksum.c \
	 net.c netif.c nfs.c \
	 rarp.c rpc.c

SRC_sa=	alloc.c \
	close.c closeall.c cread.c \
	dev.c dev_net.c dkcksum.c \
	ether_sprintf.c exit.c \
	files.c fstat.c \
	getfile.c gets.c globals.c \
	intoa.c \
	loadfile.c loadfile_elf32.c lseek.c \
	memmove.c memcmp.c memcpy.c memset.c memcmp.c \
	nullfs.c \
	open.c \
	panic.c printf.c \
	read.c \
	snprintf.c sprintf.c strerror.c strncpy.c strcmp.c strlen.c \
	strlcpy.c strlcat.c snprintf.c subr_prf.c \
	twiddle.c \
	udp.c ufs.c

SRC_kern= ashrdi3.c \
	bzero.c \
	inet_addr.c

SRC_mvme= exec_mvme.c

SRC_here= bugdev.c clock.c parse_args.c

SRCS= ${SRC_net} ${SRC_sa} ${SRC_kern} ${SRC_mvme} ${SRC_here}

# DBG= -DDEBUG -DNETIF_DEBUG -DNFS_DEBUG -DRPC_DEBUG \
# 	-DNET_DEBUG -DRARP_DEBUG -DETHER_DEBUG

#DEFS= -DCOMPAT_UFS
DEFS=	-D__INTERNAL_LIBSA_CREAD
DEFS+=	-DLIBSA_NO_FS_CLOSE

DEFS+=	-DSUPPORT_BOOTP -DSUPPORT_DHCP
DEFS+=	-DSUPPORT_RARP -DSUPPORT_BOOTPARAM
DEFS+=	-DSUPPORT_NFS -DNFS_NOSYMLINK

#DEFS+=-DNETIF_DEBUG
INCL= -I${.CURDIR} -I${.CURDIR}/../libbug -I${S}/lib/libsa -I${S}
CFLAGS+= ${XCFLAGS} ${COPTS} ${DEFS} ${DBG} ${INCL}

.PATH:  ${DIR_SA} ${DIR_KERN}

all: libsa.a

install:

.include <bsd.lib.mk>
