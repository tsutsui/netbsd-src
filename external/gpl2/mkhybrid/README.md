# tools'fied mkhybrid to build HFS/ISO9660 hybrid image for mac68k and macppc

(See src/distrib/cdrom/README how to fetch set binaries and build iso images)

## What's this?

This external/gpl2/mkhybrid provides tools'fied mkhybrid(8) to build
HFS/ISO9660 hybrid CD images for mac68k and macppc install media,
based on OpenBSD's mkhybrid 1.12b5.1 around 2018/09/08:
 http://cvsweb.openbsd.org/cgi-bin/cvsweb/src/gnu/usr.sbin/mkhybrid/src/

## Changes from OpenBSD's one

- pull sources in OpenBSD's src/gnu/usr.sbin/mkhybrid/src into
  NetBSD's src/external/gpl2/mkhybrid/dist
- pull 2 clause BSD licensed libfile sources from upstream cdrtools-3.01
- pull Makefile in OpenBSD's src/gnu/usr.sbin/mkhybrid/mkhybrid
  into NetBSD's src/external/gpl2/mkhybrid/bin
- src/external/gpl2/mkhybrid/bin is prepared to build tools version
  in src/tools/mkhybrid using src/tools/Makefile.host
- tweak configure.in to pull some header files on NetBSD
- pull -hide-rr-moved option from upstream mkisofs-1.13
- pull -graft-points option from upstream mkisofs-1.13 and cdrtools-2.01
- pull malloc related fixes in tree.c from upstream cdrtools-2.01

## Current status

- builds on NetBSD, ubuntu, and Cygwin hosts are tested

See github commit logs and diffs for more details.
 https://github.com/tsutsui/netbsd-src/commits/tsutsui-tools-mkhybrid/

## TODO

- add support to specify permissions via mtree-specfiles
  as native makefs(8) for non-root build
