#	$NetBSD: Makefile,v 1.3 2019/09/23 13:42:32 christos Exp $
CDBASE=		hp300cd			# gives ${CDBASE}.iso
CDRELEASE=	true			# include $RELEASEDIR/$RELEASEMACHINEDIR
CDRELEASE_NODEBUG=	true

CDKERNELS=	netbsd-RAMDISK.gz netbsd
CDINSTKERNEL=	../../instkernel

SYS_UBOOT=	SYS_UBOOT
BOOT=		${DESTDIR}/usr/mdec/rbootd/${SYS_UBOOT}

# make the CD bootable
prepare_md_post:
	${INSTALL} ${COPY} -m 0644 ${BOOT} cdrom/

#image_md_post:
#	${TOOL_INSTALLBOOT} -m ${MACHINE} -o append ${CDIMAGE} ${BOOTXX}

.include "${.CURDIR}/../../../common/Makefile.bootcd"
