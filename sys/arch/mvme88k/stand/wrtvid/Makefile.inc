#	$NetBSD: Makefile.booters,v 1.12 2003/10/26 07:25:36 lukem Exp $

.include <bsd.own.mk>

WRTVID_BOOT_DIR=  ${.CURDIR}/../wrtvid
WRTVID_DIR!=    cd ${WRTVID_BOOT_DIR} && ${PRINTOBJDIR}
WRTVID=${WRTVID_DIR}/wrtvid

.PHONY: ${WRTVID}
${WRTVID}:
	@echo making sure the wrtvid is up to date...
	@(cd ${WRTVID_BOOT_DIR}; ${MAKE} dependall)
