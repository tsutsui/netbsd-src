#	$NetBSD: Makefile,v 1.1 2014/02/24 07:23:38 skrll Exp $

SUBDIR=		ramdisk .WAIT kernel cdroms sysnbsd
SUBDIR+=	.WAIT
SUBDIR+=	liveimage
TARGETS+=	release

iso_image:
	${MAKEDIRTARGET} cdroms iso_image

live_image:
	${MAKEDIRTARGET} liveimage live_image

.include <bsd.subdir.mk>
