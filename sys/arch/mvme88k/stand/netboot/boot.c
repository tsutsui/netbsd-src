/*	$OpenBSD: boot.c,v 1.7 2006/05/16 22:51:30 miod Exp $ */

/*
 * Copyright (c) 1982, 1986, 1990, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * 	@(#)boot.c	8.1 (Berkeley) 6/10/93
 */

#include <sys/param.h>
#include <sys/reboot.h>
#include <machine/prom.h>

#include "stand.h"
#include "libsa.h"
#include "config.h"

int main(void);

extern	const char bootprog_name[], bootprog_rev[], bootprog_date[],
	bootprog_maker[];

char	line[80];

int
main(void)
{
	char *cp, *file;
	int ask = 0, howto, part;

	printf("\n>> %s MVME%x netboot [%s]\n",
		bootprog_name, bugargs.cputyp, bootprog_rev);
	printf(">> (%s, %s)\n", bootprog_maker, bootprog_date);

	/* cycle in the correct args */
	bugargs.arg_start = bugargs.nbarg_start;
	bugargs.arg_end   = bugargs.nbarg_end;
	*bugargs.arg_end = 0; /* ensure */

	parse_args(&file, &howto, &part);

	for (;;) {
		if (ask) {
			printf("boot: ");
			gets(line);
			if (line[0]) {
				bugargs.arg_start = line;
				cp = line;
				while (cp < (line + sizeof(line) - 1) && *cp)
					cp++;
				bugargs.arg_end = cp;
				parse_args(&file, &howto, &part);
			}
		}
		exec_mvme(file, howto, part);
		printf("boot: %s: %s\n", file, strerror(errno));
		ask = 1;
	}
	_rtt();
	return (0);
}

/*
 * machdep_common_ether: get ethernet address
 */
void
machdep_common_ether(ether)
	u_char *ether;
{
	u_char *ea;

	ea = (u_char *) ETHER_ADDR_16X;

	if (ea[0] + ea[1] + ea[2] + ea[3] + ea[4] + ea[5] == 0)
		panic("ERROR: ethernet address not set!");
	ether[0] = ea[0];
	ether[1] = ea[1];
	ether[2] = ea[2];
	ether[3] = ea[3];
	ether[4] = ea[4];
	ether[5] = ea[5];
}
