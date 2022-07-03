/*	$NetBSD: timekeeper.c,v 1.8 2003/11/01 22:48:01 tsutsui Exp $	*/

/*-
 * Copyright (c) 2001 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Steve C. Woodford.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the NetBSD
 *	Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

/*
 * Attachment interface for the Time-Keeper RAMs on mvme boards.
 *
 * XXXSCW: Needs to be extended to allow read/write to NVRAM by
 * userland application.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: timekeeper.c,v 1.8 2003/11/01 22:48:01 tsutsui Exp $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/conf.h>

#include <machine/autoconf.h>
#include <machine/cpu.h>
#include <machine/bus.h>

#include <dev/mvme/clockvar.h>
#include <dev/mvme/pcctwovar.h>

#include <dev/ic/mk48txxreg.h>
#include <dev/ic/mk48txxvar.h>

int timekeeper_match(struct device *, struct cfdata *, void *);
void timekeeper_attach(struct device *, struct device *, void *);
#ifdef MVME188
u_int8_t m188_nvrd(struct mk48txx_softc *sc, int off);
void m188_nvwr(struct mk48txx_softc *sc, int off, u_int8_t v);
#endif

CFATTACH_DECL(timekeeper, sizeof(struct mk48txx_softc),
    timekeeper_match, timekeeper_attach, NULL, NULL);

extern struct cfdriver timekeeper_cd;

int
timekeeper_match(parent, cf, aux)
	struct device *parent;
	struct cfdata *cf;
	void *aux;
{
	struct pcctwo_attach_args *pa = aux;
	bus_space_handle_t ioh;
	int rc;

	if (bus_space_map(pa->pa_bust, pa->pa_offset, PAGE_SIZE, 0, &ioh) != 0)
		return (0);
	rc = bus_space_peek_1(pa->pa_bust, ioh, 0, NULL) == 0;
	bus_space_unmap(pa->pa_bust, ioh, PAGE_SIZE);
	return (rc);
}

void
timekeeper_attach(parent, self, aux)
	struct device *parent;
	struct device *self;
	void *aux;
{
	struct mk48txx_softc *sc = (void *)self;
	struct pcctwo_attach_args *pa = aux;
	bus_size_t size;

	sc->sc_bst = pa->pa_bust;

	switch (brdtyp) {
#ifdef MVME188
	case BRD_188:
		size = MK48T02_CLKSZ * 4;
		sc->sc_model = "mk48t02";
		sc->sc_nvrd = m188_nvrd;
		sc->sc_nvwr = m188_nvwr;
		break;
#endif
	default:
		size = MK48T08_CLKSZ;
		sc->sc_model = "mk48t08";
		break;
	}

	bus_space_map(sc->sc_bst, pa->pa_offset, size, BUS_SPACE_MAP_LINEAR, &sc->sc_bsh);

	sc->sc_year0 = YEAR0;
	mk48txx_attach(sc);

	printf(" Time-Keeper RAM\n");
	printf("%s: %ld bytes NVRAM plus Realtime Clock\n",
	    sc->sc_dev.dv_xname, sc->sc_nvramsz);

	todr_attach(&sc->sc_handle);
}

#ifdef MVME188
u_int8_t
m188_nvrd(struct mk48txx_softc *sc, int off)
{
	return bus_space_read_4(sc->sc_bst, sc->sc_bsh, off << 2) & 0xff;
}

void
m188_nvwr(struct mk48txx_softc *sc, int off, u_int8_t v)
{
	bus_space_write_4(sc->sc_bst, sc->sc_bsh, off << 2, v);
}
#endif
