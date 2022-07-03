/*	$OpenBSD: libbug.h,v 1.5 2006/05/17 06:21:33 miod Exp $ */

/*
 * prototypes and such.   note that get/put char are in stand.h
 */


void	mvmeprom_delay __P((int));
int	mvmeprom_diskrd __P((struct mvmeprom_dskio *));
int	mvmeprom_diskwr __P((struct mvmeprom_dskio *));
struct	mvmeprom_brdid *mvmeprom_getbrdid __P((void));
int	peekchar __P((void));
int	mvmeprom_netfopen __P((struct mvmeprom_netfopen *));
int	mvmeprom_netfread __P((struct mvmeprom_netfread *));
void	mvmeprom_outln __P((char *, char *));
void	mvmeprom_outstr __P((char *, char *));
void	mvmeprom_rtc_rd __P((struct mvmeprom_time *));

/*
 * bugcrt stuff 
 */

extern struct mvmeprom_args bugargs;

typedef void mvmeprom_client_entry_point __P((u_int, u_int, u_int, u_int, u_int, u_int, char *, char *));

void	bugexec  __P((mvmeprom_client_entry_point *));
__dead void _rtt(void);
