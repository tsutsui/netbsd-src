#	$OpenBSD: Makefile,v 1.7 2004/01/28 08:46:29 miod Exp $

SUBDIR= libbug libsa libz .WAIT tftpboot bootxx bootsd bootst netboot \
	installboot
# not really written yet...
# SUBDIR+=	sboot

.ifmake !(install)
SUBDIR+= wrtvid prtvid
.endif

.include <bsd.subdir.mk>
