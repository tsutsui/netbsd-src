/*	$OpenBSD: lunaws.c,v 1.5 2007/04/10 22:37:17 miod Exp $	*/
/* $NetBSD: lunaws.c,v 1.6 2002/03/17 19:40:42 atatat Exp $ */

/*-
 * Copyright (c) 2000 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Tohru Nishimura.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "opt_wsdisplay_compat.h"
#include "wsmouse.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/device.h>

#include <dev/wscons/wsconsio.h>
#include <dev/wscons/wskbdvar.h>
#include <dev/wscons/wsksymdef.h>
#include <dev/wscons/wsksymvar.h>
#include <dev/wscons/wsmousevar.h>

#include <luna88k/dev/omkbdmap.h>
#include <luna88k/dev/sioreg.h>
#include <luna88k/dev/siovar.h>

#include <machine/board.h>

#include "ioconf.h"

static const u_int8_t ch1_regs[6] = {
	WR0_RSTINT,				/* Reset E/S Interrupt */
	WR1_RXALLS,				/* Rx per char, No Tx */
	0,					/* */
	WR3_RX8BIT | WR3_RXENBL,		/* Rx */
	WR4_BAUD96 | WR4_STOP1 | WR4_NPARITY,	/* Tx/Rx */
	WR5_TX8BIT | WR5_TXENBL,		/* Tx */
};

struct ws_softc {
	struct device	sc_dv;
	struct sioreg	*sc_ctl;
	u_int8_t	sc_wr[6];
	struct device	*sc_wskbddev;
#if NWSMOUSE > 0
	struct device	*sc_wsmousedev;
	int		sc_msreport;
	int		buttons, dx, dy;
#endif

#ifdef WSDISPLAY_COMPAT_RAWKBD
	int		sc_rawkbd;
#endif
};

static void omkbd_input(void *, int);
static void omkbd_decode(void *, int, u_int *, int *);
static int  omkbd_enable(void *, int);
static void omkbd_set_leds(void *, int);
static int  omkbd_ioctl(void *, u_long, caddr_t, int, struct proc *);

static const struct wskbd_mapdata omkbd_keymapdata = {
	.keydesc = omkbd_keydesctab,
	.layout =
#ifdef	OMKBD_LAYOUT
	    OMKBD_LAYOUT,
#else
	    KB_JP,
#endif
};

static const struct wskbd_accessops omkbd_accessops = {
	.enable   = omkbd_enable,
	.set_leds = omkbd_set_leds,
	.ioctl    = omkbd_ioctl,
};

void	ws_cnattach(void);
static void ws_cngetc(void *, u_int *, int *);
static void ws_cnpollc(void *, int);
static const struct wskbd_consops ws_consops = {
	.getc  = ws_cngetc,
	.pollc = ws_cnpollc,
	.bell  = NULL
};

#if NWSMOUSE > 0
static int  omms_enable(void *);
static int  omms_ioctl(void *, u_long, caddr_t, int, struct proc *);
static void omms_disable(void *);

static const struct wsmouse_accessops omms_accessops = {
	.enable  = omms_enable,
	.ioctl   = omms_ioctl,
	.disable = omms_disable,
};
#endif

static void wsintr(void *);

static int  wsmatch(struct device *, struct cfdata *, void *);
static void wsattach(struct device *, struct device *, void *);
static int  ws_submatch_kbd(struct device *, struct cfdata *, void *);
#if NWSMOUSE > 0
static int  ws_submatch_mouse(struct device *, struct cfdata *, void *);
#endif

CFATTACH_DECL(ws, sizeof(struct ws_softc),
    wsmatch, wsattach, NULL, NULL);

static int
wsmatch(struct device *parent, struct cfdata *match, void *aux)
{
	struct sio_attach_args *args = aux;

	if (args->channel != 1)
		return 0;
	return 1;
}

static void
wsattach(struct device *parent, struct device *self, void *aux)
{
	struct ws_softc *sc = (struct ws_softc *)self;
	struct sio_softc *siosc = (struct sio_softc *)parent;
	struct sio_attach_args *args = aux;
	int channel = args->channel;
	struct wskbddev_attach_args a;

	sc->sc_ctl = &siosc->sc_ctl[channel];
	memcpy(sc->sc_wr, ch1_regs, sizeof(ch1_regs));
	siosc->sc_intrhand[channel].ih_func = wsintr;
	siosc->sc_intrhand[channel].ih_arg = sc;
	
	setsioreg(sc->sc_ctl, WR0, sc->sc_wr[WR0]);
	setsioreg(sc->sc_ctl, WR4, sc->sc_wr[WR4]);
	setsioreg(sc->sc_ctl, WR3, sc->sc_wr[WR3]);
	setsioreg(sc->sc_ctl, WR5, sc->sc_wr[WR5]);
	setsioreg(sc->sc_ctl, WR0, sc->sc_wr[WR0]);

	sioputc(sc->sc_ctl, 0x20); /* keep quiet mouse */

	/* enable interrupt */
	setsioreg(sc->sc_ctl, WR1, sc->sc_wr[WR1]);

	aprint_normal("\n");

	a.console = (args->hwflags == 1);
	a.keymap = &omkbd_keymapdata;
	a.accessops = &omkbd_accessops;
	a.accesscookie = (void *)sc;
	sc->sc_wskbddev = config_found_sm(self, &a, wskbddevprint,
					ws_submatch_kbd);

#if NWSMOUSE > 0
	{
	struct wsmousedev_attach_args b;
	b.accessops = &omms_accessops;
	b.accesscookie = (void *)sc;	
	sc->sc_wsmousedev = config_found_sm(self, &b, wsmousedevprint,
					ws_submatch_mouse);
	sc->sc_msreport = 0;
	}
#endif
}

static int
ws_submatch_kbd(struct device *parent, struct cfdata *match, void *aux)
{
	struct cfdata *cf = match;

	if (strcmp(cf->cf_name, "wskbd"))
		return (0);
	return (config_match(parent, cf, aux));
}

#if NWSMOUSE > 0

static int
ws_submatch_mouse(struct device *parent, struct cfdata *match, void *aux)
{
	struct cfdata *cf = match;

	if (strcmp(cf->cf_name, "wsmouse"))
		return (0);
	return (config_match(parent, cf, aux));
}

#endif

static void
wsintr(void *arg)
{
	struct ws_softc *sc = arg;
	struct sioreg *sio = sc->sc_ctl;
	uint8_t code;
	uint16_t rr;

	rr = getsiocsr(sio);
	if (rr & RR_RXRDY) {
		do {
			code = sio->sio_data;
			if (rr & (RR_FRAMING | RR_OVERRUN | RR_PARITY)) {
				sio->sio_cmd = WR0_ERRRST;
				continue;
			}
#if NWSMOUSE > 0
			/*
			 * if (code >= 0x80 && code <= 0x87), then
			 * it's the first byte of 3 byte long mouse report
			 * 	code[0] & 07 -> LMR button condition
			 *	code[1], [2] -> x,y delta
			 * otherwise, key press or release event.
			 */
			if (sc->sc_msreport == 0) {
				if (code < 0x80 || code > 0x87) {
					omkbd_input(sc, code);
					continue;
				}
				code = (code & 07) ^ 07;
				/* LMR->RML: wsevent counts 0 for leftmost */
				sc->buttons = (code & 02);
				if (code & 01)
					sc->buttons |= 04;
				if (code & 04)
					sc->buttons |= 01;
				sc->sc_msreport = 1;
			}
			else if (sc->sc_msreport == 1) {
				sc->dx = (signed char)code;
				sc->sc_msreport = 2;
			}
			else if (sc->sc_msreport == 2) {
				sc->dy = (signed char)code;
				if (sc->sc_wsmousedev != NULL)
					wsmouse_input(sc->sc_wsmousedev,
					    sc->buttons, sc->dx, sc->dy, 0, 0);
				sc->sc_msreport = 0;
			}
#else
			omkbd_input(sc, code);
#endif
		} while ((rr = getsiocsr(sio)) & RR_RXRDY);
	}
	if (rr && RR_TXRDY)
		sio->sio_cmd = WR0_RSTPEND;
	/* not capable of transmit, yet */
}

static void
omkbd_input(void *v, int data)
{
	struct ws_softc *sc = v;
	u_int type;
	int key;

	omkbd_decode(v, data, &type, &key);

#ifdef WSDISPLAY_COMPAT_RAWKBD
	if (sc->sc_rawkbd) {
		u_char cbuf[2];
		int c, j = 0;

		c = omkbd_raw[key];
		if (c != 0x00) {
			/* fake extended scancode if necessary */
			if (c & 0x80)
				cbuf[j++] = 0xe0;
			cbuf[j] = c & 0x7f;
			if (type == WSCONS_EVENT_KEY_UP)
				cbuf[j] |= 0x80;
			j++;

			wskbd_rawinput(sc->sc_wskbddev, cbuf, j);
		}
	} else
#endif
	{
		if (sc->sc_wskbddev != NULL)
			wskbd_input(sc->sc_wskbddev, type, key);	
	}
}

static void
omkbd_decode(void *v, int datain, u_int *type, int *dataout)
{
	*type = (datain & 0x80) ? WSCONS_EVENT_KEY_UP : WSCONS_EVENT_KEY_DOWN;
	*dataout = datain & 0x7f;
}

static void
ws_cngetc(void *cookie, u_int *type, int *data)
{
	struct ws_softc *sc = cookie;	/* currently unused */
	struct sioreg *sio, *sio_base;
	int code;

	sio_base = (struct sioreg *)OBIO_SIO;
	sio = &sio_base[1];	/* channel B */

	code = siogetc(sio);
	omkbd_decode(sc, code, type, data);
}

static void
ws_cnpollc(void *cookie, int on)
{
}

/* EXPORT */ void
ws_cnattach(void)
{
	static int voidfill;

	/* XXX need CH.B initialization XXX */

	wskbd_cnattach(&ws_consops, &voidfill, &omkbd_keymapdata);
}

static int
omkbd_enable(void *v, int on)
{
	return 0;
}

static void
omkbd_set_leds(void *v, int leds)
{
}

static int
omkbd_ioctl(void *v, u_long cmd, caddr_t data, int flag, struct proc *p)
{
#ifdef WSDISPLAY_COMPAT_RAWKBD
	struct ws_softc *sc = v;
#endif

	switch (cmd) {
	case WSKBDIO_GTYPE:
		*(int *)data = WSKBD_TYPE_LUNA;
		return 0;
	case WSKBDIO_SETLEDS:
	case WSKBDIO_GETLEDS:
	case WSKBDIO_COMPLEXBELL:	/* XXX capable of complex bell */
		return 0;
#ifdef WSDISPLAY_COMPAT_RAWKBD
	case WSKBDIO_SETMODE:
		sc->sc_rawkbd = *(int *)data == WSKBD_RAW;
		return 0;
	case WSKBDIO_GETMODE:
		*(int *)data = sc->sc_rawkbd;
		return 0;
#endif
	}
	return EPASSTHROUGH;
}

#if NWSMOUSE > 0

static int
omms_enable(void *cookie)
{
	struct ws_softc *sc = cookie;	/* currently unused */
	struct sioreg *sio, *sio_base;

	sio_base = (struct sioreg *)OBIO_SIO;
	sio = &sio_base[1];	/* channel B */

	sioputc(sio, 0x60);
	sc->sc_msreport = 0;
	return 0;
}

/*ARGUSED*/
static int
omms_ioctl(void *v, u_long cmd, caddr_t data, int flag, struct proc *p)
{
#if 0
	struct ws_softc *sc = v;
#endif

	switch (cmd) {
	case WSMOUSEIO_GTYPE:
		*(u_int *)data = 0x19991005; /* XXX */;
		return 0;
	}

	return -1;
}

static void
omms_disable(void *cookie)
{
	struct ws_softc *sc = cookie;	/* currently unused */
	struct sioreg *sio, *sio_base;

	sio_base = (struct sioreg *)OBIO_SIO;
	sio = &sio_base[1];	/* channel B */

	sioputc(sio, 0x20); /* quiet mouse */
	sc->sc_msreport = 0;
}
#endif
