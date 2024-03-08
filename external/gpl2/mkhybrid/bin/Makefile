#	$OpenBSD: Makefile,v 1.7 2015/09/09 20:02:31 miod Exp $

# NOTE: The mkhybrid distrib also makes an "mkisofs" which is the same binary
# linked. When we know mkhybrid makes distrib CDs ok we should modify this
# to make mkisofs as a link to this, rather than the (earlier) version in
# the tree.

DIST=	${NETBSDSRCDIR}/external/gpl2/mkhybrid/dist

PROG=	mkhybrid
MAN=	mkhybrid.8
BINDIR=	/usr/sbin
CLEANFILES+= mkhybrid.8
.PATH:	${DIST} ${DIST}/libhfs_iso ${DIST}/libfile

SRCS=	data.c block.c low.c lfile.c btree.c node.c record.c lvolume.c \
	hfs.c file.c apprentice.c softmagic.c mkisofs.c tree.c write.c \
	hash.c rock.c multi.c joliet.c match.c name.c eltorito.c \
	apple.c volume.c desktop.c mac_label.c		
SRCS+=	getopt.c getopt1.c

CPPFLAGS+=-DSYSTEM_ID_DEFAULT=\"NetBSD\" -DAPPLE_HYB -DVANILLA_AUTOCONF \
	-I${DIST} -I${DIST}/include \
	-I${DIST}/libhfs_iso \
	-I${DIST}/libfile

mkhybrid.8: mkhybrid.8tbl
	cp ${.ALLSRC} ${.TARGET}

.include <bsd.prog.mk>
