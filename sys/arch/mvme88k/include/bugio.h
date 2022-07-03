/*	$OpenBSD: bugio.h,v 1.16 2006/05/02 21:43:08 miod Exp $ */

#ifndef __MACHINE_BUGIO_H__
#define __MACHINE_BUGIO_H__

#include <machine/prom.h>

void	buginit(void);
int	buginstat(void);
char	buginchr(void);
void	bugpcrlf(void);
void	bugoutchr(int);
void	bugrtcrd(struct mvmeprom_time *);
void	bugoutstr(char *, char *);
void	bugreturn(void);
void	bugbrdid(struct mvmeprom_brdid *);
void	bugdiskrd(struct mvmeprom_dskio *);
int	spin_cpu(cpuid_t, vaddr_t);

int bugenvgetvar(struct prom_environ_hdr* pkt);

#endif /* __MACHINE_BUGIO_H__ */
