#	$NetBSD: Makefile,v 1.2 2025/09/23 20:12:07 rillig Exp $

SUBDIR=		ramdisk .WAIT instkernel .WAIT cdroms
SUBDIR+=	liveimage
TARGETS+=	release

iso_image: .MAKE
	${MAKEDIRTARGET} cdroms iso_image

live_image: .MAKE
	${MAKEDIRTARGET} liveimage live_image

.include <bsd.subdir.mk>
