#	$OpenBSD: Makefile,v 1.1 2004/04/24 13:19:15 miod Exp $

NOMAN=		# defined

PROG=		trap
CLEANFILES+=	usertrap.out

regress:
	sh ${.CURDIR}/testsuite.sh >usertrap.out 2>&1
	@if diff -u usertrap.out ${.CURDIR}/answer; then \
		echo SUCCESS; \
	else \
		echo FAIL; \
		exit 1; \
	fi

.include <bsd.prog.mk>
