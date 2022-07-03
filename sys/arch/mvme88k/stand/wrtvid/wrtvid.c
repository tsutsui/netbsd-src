/*	$OpenBSD: wrtvid.c,v 1.6 2006/05/18 06:11:16 miod Exp $ */

/*
 * Copyright (c) 1995 Dale Rahn <drahn@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <machine/bswap.h>

#include "../../include/disklabel.h"

void	copy_exe(int, int);
void	swabcfg(struct cpu_disklabel *);
void	swabvid(struct cpu_disklabel *);

int
main(argc, argv)
	int argc;
	char **argv;
{
	struct cpu_disklabel *pcpul;
	struct stat stat;
	int exe_file;
	int tape_vid;
	int tape_exe;
	unsigned int exe_addr;
	char *filename;
	char fileext[256];

	if (argc == 1)
		filename = "a.out";
	else
		filename = argv[1];

	exe_file = open(filename, O_RDONLY,0444);
	if (exe_file == -1) {
		perror(filename);
		exit(2);
	}
	snprintf(fileext, sizeof fileext, "%c%cboot", filename[4], filename[5]);
	tape_vid = open(fileext, O_WRONLY|O_CREAT|O_TRUNC, 0644);
	snprintf(fileext, sizeof fileext, "boot%c%c", filename[4], filename[5]);
	tape_exe = open(fileext, O_WRONLY|O_CREAT|O_TRUNC,0644);

	pcpul = (struct cpu_disklabel *)malloc(sizeof(struct cpu_disklabel));
	bzero(pcpul, sizeof(struct cpu_disklabel));

	pcpul->version = VID_VERSION_MVME88K;
	memcpy(pcpul->vid_id, VID_ID, sizeof pcpul->vid_id);

	fstat(exe_file, &stat);
	/* size in 256 byte blocks round up after a.out header removed */

	if (filename[5] == 't' ) {
		pcpul->vid_oss = 1;
	}else {
		pcpul->vid_oss = 2;
	}
	pcpul->vid_osl = (((stat.st_size -0x20) +511) / 512) *2;

	lseek(exe_file, 0x14, SEEK_SET);
	read(exe_file, &exe_addr, 4);

	/* check this, it may not work in both endian. */
	/* No, it doesn't.  Use a big endian machine for now. SPM */
	
	{
		union {
			struct s {
				unsigned short s1;
				unsigned short s2;
			} s;
			unsigned long l;
		} a;
		a.l = exe_addr;
		pcpul->vid_osa_u = a.s.s1;
		pcpul->vid_osa_l = a.s.s2;

	}
	pcpul->vid_cas = VID_CAS;
	pcpul->vid_cal = VID_CAL;
	memcpy(pcpul->vid_mot, VID_MOT, sizeof(pcpul->vid_mot));

	if (BYTE_ORDER != BIG_ENDIAN)
		swabvid(pcpul);

	pcpul->cfg_rec = CFG_REC;
	pcpul->cfg_psm = CFG_PSM;

	if (BYTE_ORDER != BIG_ENDIAN)
		swabcfg(pcpul);

	write(tape_vid, pcpul, sizeof(struct cpu_disklabel));

	free(pcpul);

	copy_exe(exe_file, tape_exe);
	close(exe_file);
	close(tape_vid);
	close(tape_exe);
	return (0);
}

#define BUF_SIZ 512
void
copy_exe(exe_file, tape_exe)
	int exe_file, tape_exe;
{
	char *buf;
	int cnt = 0;

	buf = (char *)malloc(BUF_SIZ);

	lseek (exe_file, 0x20, SEEK_SET);
	while (BUF_SIZ == (cnt = read(exe_file, buf, BUF_SIZ))) {
		write(tape_exe, buf, cnt);
	}
	bzero(&buf[cnt], BUF_SIZ-cnt);
	write(tape_exe, buf, BUF_SIZ);
}

void
swabvid(pcpul)
	struct cpu_disklabel *pcpul;
{
	bswap32(pcpul->vid_oss);
	bswap16(pcpul->vid_osl);
#if 0
	bswap16(pcpul->vid_osa_u);
	bswap16(pcpul->vid_osa_l);
#endif
	bswap32(pcpul->vid_cas);
}

void
swabcfg(pcpul)
	struct cpu_disklabel *pcpul;
{
	bswap16(pcpul->cfg_atm);
	bswap16(pcpul->cfg_prm);
	bswap16(pcpul->cfg_atm);
	bswap16(pcpul->cfg_rec);
	bswap16(pcpul->cfg_trk);
	bswap16(pcpul->cfg_psm);
	bswap16(pcpul->cfg_shd);
	bswap16(pcpul->cfg_pcom);
	bswap16(pcpul->cfg_rwcc);
	bswap16(pcpul->cfg_ecc);
	bswap16(pcpul->cfg_eatm);
	bswap16(pcpul->cfg_eprm);
	bswap16(pcpul->cfg_eatw);
	bswap16(pcpul->cfg_rsvc1);
	bswap16(pcpul->cfg_rsvc2);
}
