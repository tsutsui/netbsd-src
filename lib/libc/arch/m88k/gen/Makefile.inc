#	$OpenBSD: Makefile.inc,v 1.5 2005/11/29 21:38:09 deraadt Exp $
#	$NetBSD: Makefile.inc,v 1.3 1995/04/10 21:09:06 jtc Exp $

SRCS+=	fabs.S

# Common ieee754 constants and functions
SRCS+=	infinityf_ieee754.c infinity_ieee754.c infinityl_dbl_ieee754.c
SRCS+=	fpclassifyf_ieee754.c fpclassifyd_ieee754.c
SRCS+=	frexp_ieee754.c
SRCS+=	isinff_ieee754.c isinfd_ieee754.c
SRCS+=	isnanf_ieee754.c isnand_ieee754.c
SRCS+=	isfinitef_ieee754.c isfinited_ieee754.c
SRCS+=	ldexp_ieee754.c modf_ieee754.c
SRCS+=	signbitf_ieee754.c signbitd_ieee754.c

SRCS+=	nanf.c

SRCS+=	flt_rounds.c fpgetmask.c fpgetround.c fpgetsticky.c fpsetmask.c \
	fpsetround.c fpsetsticky.c

SRCS+=	_setjmp.S setjmp.S sigsetjmp.S
SRCS+=	bswap16.c bswap32.c bswap64.c
SRCS+=	makecontext.c #XXX TODO TKM: resumecontext.c swapcontext.S _lwp.c

SRCS.m88k.gen=	Lint_bswap16.c Lint_bswap32.c #XXX Lint_swapcontext.c
LSRCS+=		${SRCS.m88k.gen}
DPSRCS+=	${SRCS.m88k.gen}
CLEANFILES+=	${SRCS.m88k.gen}
