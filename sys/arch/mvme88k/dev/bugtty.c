/*	$OpenBSD: bugtty.c,v 1.22 2004/05/16 22:23:06 miod Exp $ */

/* Copyright (c) 1998 Steve Murphree, Jr.
 * Copyright (c) 1995 Dale Rahn.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/ioctl.h>
#include <sys/callout.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <sys/tty.h>
#include <sys/proc.h>
#include <sys/uio.h>
#include <sys/queue.h>
#include <dev/cons.h>

#include <machine/autoconf.h>
#include <machine/bugio.h>
#include <machine/cpu.h>

#include "bugtty.h"

#define BUGTTYUNIT(dev) ((dev) & (0x7f))
#define DIALOUT(dev)	((dev) & 0x80)

#define SWFLAGS(dev)	(bugttyswflags | (DIALOUT(dev) ? TIOCFLAG_SOFTCAR : 0))

#define BUGBUF 80	/* Output buffer size (characters) */

/* prototypes */
static int bugttymatch(struct device *, struct cfdata *, void *);
static void bugttyattach(struct device *, struct device *, void *);
static int bugttymctl(dev_t, int, int);
static int bugttyparam(struct tty *, struct termios *);
static void bugttyoutput(struct tty *);
static void bugtty_chkinput(void * arg);

static dev_type_open(bugttyopen);
static dev_type_close(bugttyclose);
static dev_type_read(bugttyread);
static dev_type_write(bugttywrite);
static dev_type_ioctl(bugttyioctl);
static dev_type_stop(bugttystop);
static dev_type_tty(bugttytty);
cons_decl(bugtty);

CFATTACH_DECL(bugtty,
	      sizeof (struct device),
	      bugttymatch,
	      bugttyattach,
	      NULL,
	      NULL);

const struct cdevsw bugtty_cdevsw = {
	      bugttyopen,
	      bugttyclose,
	      bugttyread,
	      bugttywrite,
	      bugttyioctl,
	      bugttystop,
	      bugttytty,
	      nopoll,
	      nommap,
	      nokqfilter,
	      D_TTY
};

static struct tty * bugtty_tty[NBUGTTY];
static char	      bug_obuffer[BUGBUF+1];
static struct callout bug_rx_callout;
static int	      bugttydefaultrate = TTYDEF_SPEED;
static int	      bugttyswflags;

int
bugttymatch(struct device *parent, struct cfdata *self, void *aux)
{
	struct confargs *ca = aux;

	/*
	 * Do not attach if a suitable console driver has been attached.
	 */
	if (cn_tab != NULL && cn_tab->cn_probe != bugttycnprobe)
		return (0);

	ca->ca_ipl = IPL_TTY;
	return (1);
}

void
bugttyattach(struct device *parent, struct device *self, void *aux)
{
	callout_init(&bug_rx_callout);

	printf(": fallback console\n");
}

int
bugttymctl(dev_t dev, int bits, int how)
{
	int s;

	/*printf("mctl: dev %x, bits %x, how %x,",dev, bits, how);*/

	/* settings are currently ignored */
	s = spltty();
	switch (how) {
	case DMSET:
		break;
	case DMBIC:
		break;
	case DMBIS:
		break;
	case DMGET:
		break;
	}
	splx(s);

	bits = 0;
	/* proper defaults? */
	bits |= TIOCM_DTR;
	bits |= TIOCM_RTS;
	bits |= TIOCM_CTS;
	bits |= TIOCM_CD;
	/* bits |= TIOCM_RI; */
	bits |= TIOCM_DSR;

	/* printf("retbits %x\n", bits); */
	return (bits);
}

int
bugttyparam(tp, tm)
	struct tty *tp;
	struct termios *tm;
{
	return (0);
}

void
bugttyoutput(tp)
	struct tty *tp;
{
	int cc, s, cnt;

	/* only supports one unit */

	if ((tp->t_state & TS_ISOPEN) == 0)
		return;

	s = spltty();
	cc = tp->t_outq.c_cc;
	while (cc > 0) {
		cnt = min(BUGBUF, cc);
		cnt = q_to_b(&tp->t_outq, bug_obuffer, cnt);
		bugoutstr(bug_obuffer, &bug_obuffer[cnt]);
		cc -= cnt;
	}
	splx(s);
}

int
bugttyopen(dev, flag, mode, p)
	dev_t dev;
	int flag, mode;
	struct proc *p;
{
	int s, unit = BUGTTYUNIT(dev);
	struct tty *tp;

	s = spltty();
	if (bugtty_tty[unit]) {
		tp = bugtty_tty[unit];
	} else {
		tp = bugtty_tty[unit] = ttymalloc();
	}
	tp->t_oproc = bugttyoutput;
	tp->t_param = NULL;
	tp->t_dev = dev;

	if ((tp->t_state & TS_ISOPEN) == 0) {
/* XXX -TKM		tp->t_state |= TS_WOPEN; */
		ttychars(tp);
		if (tp->t_ispeed == 0) {
			/*
			 * only when cleared do we reset to defaults.
			 */
			tp->t_iflag = TTYDEF_IFLAG;
			tp->t_oflag = TTYDEF_OFLAG;
			tp->t_cflag = TTYDEF_CFLAG;
			tp->t_lflag = TTYDEF_LFLAG;
			tp->t_ispeed = tp->t_ospeed = bugttydefaultrate;
		}
		/* bugtty does not have carrier */
		tp->t_cflag |= CLOCAL;
		/*
		 * do these all the time
		 */
		if (bugttyswflags & TIOCFLAG_CLOCAL)
			tp->t_cflag |= CLOCAL;
		if (bugttyswflags & TIOCFLAG_CRTSCTS)
			tp->t_cflag |= CRTSCTS;
		if (bugttyswflags & TIOCFLAG_MDMBUF)
			tp->t_cflag |= MDMBUF;
		bugttyparam(tp, &tp->t_termios);
		ttsetwater(tp);

		(void)bugttymctl(dev, TIOCM_DTR | TIOCM_RTS, DMSET);
		/*
		if ((SWFLAGS(dev) & TIOCFLAG_SOFTCAR) ||
		    (bugttymctl(dev, 0, DMGET) & TIOCM_CD))
			tp->t_state |= TS_CARR_ON;
		else
			tp->t_state &= ~TS_CARR_ON;
		*/
		tp->t_state |= TS_CARR_ON;

		/* XXX TKM - I believe this driver only supports one unit */
		bugtty_chkinput(tp);
	} else if (tp->t_state & TS_XCLUDE && p->p_ucred->cr_uid != 0) {
		splx(s);
		return (EBUSY);
	}

	/*
	 * if NONBLOCK requested, ignore carrier
	 */
/*
	if (flag & O_NONBLOCK)
		goto done;
*/

	splx(s);
	/*
	 * Reset the tty pointer, as there could have been a dialout
	 * use of the tty with a dialin open waiting.
	 */
	tp->t_dev = dev;
	return (tp->t_linesw->l_open(dev, tp));
}

int
bugttyclose(dev, flag, mode, p)
	dev_t dev;
	int flag, mode;
	struct proc *p;
{
	int unit = BUGTTYUNIT(dev);
	struct tty *tp = bugtty_tty[unit];

	tp->t_linesw->l_close(tp, flag);

	ttyclose(tp);
#if 0
	bugtty_tty[unit] = NULL;
#endif
	return (0);
}

int
bugttyread(dev, uio, flag)
	dev_t dev;
	struct uio *uio;
	int flag;
{
	struct tty *tp;

	if ((tp = bugtty_tty[BUGTTYUNIT(dev)]) == NULL)
		return (ENXIO);
	return (tp->t_linesw->l_read(tp, uio, flag));
}

/* only to be called at splclk() */
void
bugtty_chkinput(void * arg)
{
	struct tty * tp = arg;

	if (tp == NULL)
		return;

	while (buginstat() != 0) {
		u_char c = buginchr() & 0xff;
		tp->t_linesw->l_rint(c, tp);
	}

	callout_reset(&bug_rx_callout, 1, bugtty_chkinput, tp);
}

int
bugttywrite(dev, uio, flag)
	dev_t dev;
	struct uio *uio;
	int flag;
{
#if 0
	/* bypass tty output routines. */
	int i, cnt, s;
	int oldoff;

	s = spltty();
	oldoff = uio->uio_offset;
	do  {
		uiomove(bug_obuffer, BUGBUF, uio);
		bugoutstr(bug_obuffer, &bug_obuffer[uio->uio_offset - oldoff]);
		oldoff = uio->uio_offset;
	} while (uio->uio_resid != 0);
	splx(s);

	return (0);
#else
	struct tty *tp;
	if((tp = bugtty_tty[BUGTTYUNIT(dev)]) == NULL)
		return (ENXIO);
	return (tp->t_linesw->l_write(tp, uio, flag));
#endif
}

int
bugttyioctl(dev, cmd, data, flag, p)
	dev_t dev;
	u_long cmd;
	caddr_t data;
	int flag;
	struct proc *p;
{
	int unit = BUGTTYUNIT(dev);
	struct tty *tp = bugtty_tty[unit];
	int error;

	if (!tp)
		return (ENXIO);

	error = tp->t_linesw->l_ioctl(tp, cmd, data, flag, p);
	if (error >= 0)
		return (error);

	error = ttioctl(tp, cmd, data, flag, p);
	if (error >= 0)
		return (error);

	switch (cmd) {
	case TIOCSBRK:
		/* */
		break;

	case TIOCCBRK:
		/* */
		break;

	case TIOCSDTR:
		(void) bugttymctl(dev, TIOCM_DTR | TIOCM_RTS, DMBIS);
		break;

	case TIOCCDTR:
		(void) bugttymctl(dev, TIOCM_DTR | TIOCM_RTS, DMBIC);
		break;

	case TIOCMSET:
		(void) bugttymctl(dev, *(int *) data, DMSET);
		break;

	case TIOCMBIS:
		(void) bugttymctl(dev, *(int *) data, DMBIS);
		break;

	case TIOCMBIC:
		(void) bugttymctl(dev, *(int *) data, DMBIC);
		break;

	case TIOCMGET:
		*(int *)data = bugttymctl(dev, 0, DMGET);
		break;
	case TIOCGFLAGS:
		*(int *)data = SWFLAGS(dev);
		break;
	case TIOCSFLAGS:
		error = suser(p->p_ucred, 0);
		if (error != 0)
			return (EPERM);

		bugttyswflags = *(int *)data;
		bugttyswflags &= /* only allow valid flags */
		    (TIOCFLAG_SOFTCAR | TIOCFLAG_CLOCAL | TIOCFLAG_CRTSCTS);
		break;
	default:
		return (ENOTTY);
	}

	return (0);
}

void
bugttystop(tp, flag)
	struct tty *tp;
	int flag;
{
	int s;

	s = spltty();
	if (tp->t_state & TS_BUSY) {
		if ((tp->t_state & TS_TTSTOP) == 0)
			tp->t_state |= TS_FLUSH;
	}
	splx(s);
}

struct tty *
bugttytty(dev_t dev)
{
	int unit;
	unit = BUGTTYUNIT(dev);
	if (unit >= NBUGTTY) {
		return (NULL);
	}
	return bugtty_tty[unit];
}

/*
 * bugtty is the last possible choice for a console device.
 */
void
bugttycnprobe(cp)
	struct consdev *cp;
{
	int maj;

	/*
	 * Locate the major number
	 */
	maj = cdevsw_lookup_major(&bugtty_cdevsw);

	cp->cn_dev = makedev(maj, 0);
	cp->cn_pri = CN_NORMAL;
}

void
bugttycninit(cp)
	struct consdev *cp;
{
	cn_tab = cp;
	printf("bugtty: console installed.\n");
}

int
bugttycngetc(dev)
	dev_t dev;
{
	return (buginchr());
}

void
bugttycnputc(dev, c)
	dev_t dev;
	char c;
{
	if (c == '\n')
		bugpcrlf();
	else
		bugoutchr(c);
}
