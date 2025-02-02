#	$NetBSD$

LIVEIMGBASE=	NetBSD-${DISTRIBVER}-macppc-live	# gives ${IMGBASE}.img

EMUIMAGEMB?=	2048
SWAPMB?=	64
LIVEIMAGEMB=	${EMUIMAGEMB}
PRIMARY_BOOT=	bootxx
SECONDARY_BOOT=	ofwboot
SECONDARY_BOOT_ARG=	/${SECONDARY_BOOT}

.include "${.CURDIR}/../../../common/bootimage/Makefile.liveimage"
