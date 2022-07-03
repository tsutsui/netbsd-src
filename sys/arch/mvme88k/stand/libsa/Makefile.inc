#	$OpenBSD: Makefile.inc,v 1.3 2006/05/16 22:51:30 miod Exp $

.include <bsd.own.mk>

LIB_SA_DIR=${S}/arch/mvme88k/stand/libsa

LIBSA_DIR!= cd ${LIB_SA_DIR} && ${PRINTOBJDIR}

LIBSA=${LIBSA_DIR}/libsa.a

$(LIBSA):	.NOTMAIN __always_make_libsa
	@echo making sure the libsa is up to date...
	@(cd ${LIB_SA_DIR}; ${MAKE} "XCFLAGS=${CFLAGS}")

__always_make_libsa:	.NOTMAIN
