/*	$OpenBSD: putchar.c,v 1.4 2006/05/16 22:51:30 miod Exp $ */

/*
 * putchar: easier to do this with outstr than to add more macros to
 * handle byte passing on the stack
 */

#include <sys/types.h>
#include <machine/prom.h>

#include "stand.h"
#include "libbug.h"

void
putchar(c)
	int c;
{
	if (c == '\n')
		putchar('\r');
	MVMEPROM_ARG1((char)c);
	MVMEPROM_CALL(MVMEPROM_OUTCHR);
}
