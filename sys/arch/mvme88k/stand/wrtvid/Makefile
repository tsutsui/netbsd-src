#	$NetBSD: Makefile,v 1.11 2002/01/23 21:06:28 scw Exp $

S=${.CURDIR}/../../../../

HOSTPROG=	wrtvid
NOMAN=		# defined

.include <bsd.own.mk>

.if ${USETOOLS} == "yes"
COMPATOBJDIR!=  cd ${NETBSDSRCDIR}/tools/compat && ${PRINTOBJDIR}
HOST_CPPFLAGS+=	-I${COMPATOBJDIR}/include
HOST_LDFLAGS+=	-L${COMPATOBJDIR} -lnbcompat
.endif

# only needed during build
proginstall::

.include <bsd.hostprog.mk>
