# $NetBSD$

SRCS+=  rtld_start.S selfreloc.c
CPPFLAGS+= -fPIC -DELFSIZE=32
LDFLAGS+= -e .rtld_start
