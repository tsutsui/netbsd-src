#	$OpenBSD: Makefile,v 1.7 2006/05/16 22:51:30 miod Exp $

LIB=sa

# Logically src/sys
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

CPPFLAGS+=	-D__INTERNAL_LIBSA_CREAD
CPPFLAGS+=	-DLIBSA_NO_FS_CLOSE
CPPFLAGS+=	-DSUPPORT_BOOTP -DSUPPORT_DHCP
CPPFLAGS+=	-DSUPPORT_RARP -DSUPPORT_BOOTPARAM
CPPFLAGS+=	-DSUPPORT_NFS -DNFS_NOSYMLINK
CPPFLAGS+=	${DBG}

.include "../Makefile.booters"

.PATH:  ${DIR_SA} ${DIR_KERN}

# only needed during build
libinstall::

all realall: lib${LIB}.a

.include <bsd.own.mk>
.undef DESTDIR
.include <bsd.lib.mk>
