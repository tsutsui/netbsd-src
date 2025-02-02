#	$NetBSD$

LIVEIMGBASE=	NetBSD-${DISTRIBVER}-hppa-live	# gives ${IMGBASE}.img

EMUIMAGEMB?=	2048		# firmware has 2GB limitation to load a kernel
SWAPMB?=	64
LIVEIMAGEMB=	${EMUIMAGEMB}
PRIMARY_BOOT=	xxboot
SECONDARY_BOOT=	boot

.include "${.CURDIR}/../../../common/bootimage/Makefile.liveimage"
