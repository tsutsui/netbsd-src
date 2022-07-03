/*	$NetBSD: pcctwo_68k.c,v 1.4 2003/07/15 02:43:47 lukem Exp $	*/

/*-
 * Copyright (c) 1999, 2002 The NetBSD Foundation, Inc.
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
 *	      This product includes software developed by the NetBSD
 *	      Foundation, Inc. and its contributors.
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
 * PCCchip2 and MCchip Mvme68k Front End Driver
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: pcctwo_68k.c,v 1.4 2003/07/15 02:43:47 lukem Exp $");

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/device.h>

#include <machine/autoconf.h>
#include <machine/cpu.h>

#include <dev/mvme/pcctworeg.h>
#include <dev/mvme/pcctwovar.h>

#include "bussw.h"

/*
 * Autoconfiguration stuff.
 */
void pcctwoattach(struct device *, struct device *, void *);
int pcctwomatch(struct device *, struct cfdata *, void *);
int	pcctwo_scan(struct device *parent, struct cfdata *child, void *args);

CFATTACH_DECL(pcctwo, sizeof(struct pcctwo_softc),
    pcctwomatch, pcctwoattach, NULL, NULL);

extern struct cfdriver pcctwo_cd;


static int pcctwo_vec2icsr_1x7[PCCTWOV_MAX] = {
	VEC2ICSR(PCC2REG_PRT_BUSY_ICSR,  PCCTWO_ICR_EDGE | PCCTWO_ICR_ICLR),
	VEC2ICSR(PCC2REG_PRT_PE_ICSR,    PCCTWO_ICR_EDGE | PCCTWO_ICR_ICLR),
	VEC2ICSR(PCC2REG_PRT_SEL_ICSR,   PCCTWO_ICR_EDGE | PCCTWO_ICR_ICLR),
	VEC2ICSR(PCC2REG_PRT_FAULT_ICSR, PCCTWO_ICR_EDGE | PCCTWO_ICR_ICLR),
	VEC2ICSR(PCC2REG_PRT_ACK_ICSR,   PCCTWO_ICR_EDGE | PCCTWO_ICR_ICLR),
	VEC2ICSR(PCC2REG_SCSI_ICSR,      0),
	VEC2ICSR(PCC2REG_ETH_BERR_ICSR,  PCCTWO_ICR_SC_WR(0) | PCCTWO_ICR_ICLR),
	VEC2ICSR(PCC2REG_ETH_ICSR,       PCCTWO_ICR_EDGE | PCCTWO_ICR_ICLR),
	VEC2ICSR(PCC2REG_TIMER2_ICSR,    PCCTWO_ICR_ICLR),
	VEC2ICSR(PCC2REG_TIMER1_ICSR,    PCCTWO_ICR_ICLR),
	VEC2ICSR(PCC2REG_GPIO_ICSR,      PCCTWO_ICR_EDGE | PCCTWO_ICR_ICLR),
	-1,
	VEC2ICSR(PCC2REG_SCC_RX_ICSR,    0),
	VEC2ICSR(PCC2REG_SCC_MODEM_ICSR, 0),
	VEC2ICSR(PCC2REG_SCC_TX_ICSR,    0),
	VEC2ICSR(PCC2REG_SCC_RX_ICSR,    0)
};

static void pcctwoisrlink(void *, int (*)(void *), void *,
		int, int, struct evcnt *);
static void pcctwoisrunlink(void *, int);
static struct evcnt *pcctwoisrevcnt(void *, int);


/* ARGSUSED */
int
pcctwomatch(parent, cf, args)
	struct device *parent;
	struct cfdata *cf;
	void *args;
{
	struct confargs *ca = args;
	bus_space_handle_t ioh;
	int rc;
	u_int8_t chipid;

	/* There can be only one. */
	if (sys_pcctwo)
		return (0);

	if (ca->ca_paddr == -1)
		return (0);

	/* Bomb if wrong CPU */
	switch (brdtyp) {
	case MVME_197:
		if (strcmp("bussw", parent->dv_cfdata->cf_atname) != 0)
			return (0);
	case MVME_187:
	case MVME_8120:
		break;
	default:
		return (0);
	}

	/*
	 * Map the registers
	 */
	if (bus_space_map(ca->ca_iot, PCCTWO_REG_OFF + ca->ca_paddr,
	    PCC2REG_SIZE, 0, &ioh) != 0)
		return (0);

	/*
	 * Grab the Chip's ID
	 */
	rc = bus_space_peek_1(ca->ca_iot, ioh, PCC2REG_CHIP_ID, &chipid);
	bus_space_unmap(ca->ca_iot, ioh, PCC2REG_SIZE);

	if (rc == 0 && chipid != PCCTWO_CHIP_ID_PCC2) {
#ifdef DEBUG
		printf("==> pcctwo: wrong chip id %x.\n", chipid);
#endif
		return (0);
	}

	return (1);
}

/* ARGSUSED */
void
pcctwoattach(parent, self, args)
	struct device *parent;
	struct device *self;
	void *args;
{
	struct confargs *ca = args;
	struct pcctwo_softc *sc = sys_pcctwo = (struct pcctwo_softc *) self;

	/* Get a handle to the PCCChip2's registers */
	sc->sc_bust = ca->ca_iot;
	sc->sc_dmat = ca->ca_dmat;
	if (bus_space_map(sc->sc_bust, PCCTWO_REG_OFF + ca->ca_paddr,
	    PCC2REG_SIZE, 0, &sc->sc_bush) != 0) {
		printf(": can't map registers!\n");
		return;
	}

#if NBUSSW > 0
	if (ca->ca_bustype == BUS_BUSSWITCH) {
                /* Make sure the bus is mc68040 compatible */
		pcc2_reg_write(sc, PCC2REG_GENERAL_CONTROL,
		    pcc2_reg_read(sc, PCC2REG_GENERAL_CONTROL) | PCCTWO_GEN_CTRL_C040);
	}
#endif

	sc->sc_vec2icsr  = pcctwo_vec2icsr_1x7;
	sc->sc_vecbase   = PCCTWO_VECBASE;
	sc->sc_isrcookie = sc;
	sc->sc_isrlink   = pcctwoisrlink;
	sc->sc_isrevcnt  = pcctwoisrevcnt;
	sc->sc_isrunlink = pcctwoisrunlink;

	evcnt_attach_dynamic(&sc->sc_evcnt, EVCNT_TYPE_INTR, NULL,
	    sc->sc_dev.dv_xname, "intr");

	/* Finish initialisation in common code */
	pcctwo_init(sc, NULL, ca->ca_offset);

	config_search(pcctwo_scan, self, args);
}

int
pcctwo_scan(parent, cf, args)
	struct device *parent;
	struct cfdata *cf;
	void *args;
{
	struct pcctwo_softc *sc = (struct pcctwo_softc *) parent;
	struct confargs *ca = args;
	struct pcctwo_attach_args npa;

	if (cf->pcctwocf_offset == -1)
		return (0);

	/*
	 * Note that IPL is filled in by match function.
	 */
	npa.pa_name = cf->cf_name;
	npa.pa_ipl = -1;
	npa.pa_dmat = sc->sc_dmat;
	npa.pa_bust = sc->sc_bust;
	npa._pa_base = ca->ca_paddr;
	npa.pa_offset = npa._pa_base + cf->pcctwocf_offset;

	/* Attach the device if configured. */
	if (config_match(parent, cf, &npa) == 0)
		return (0);

	config_attach(parent, cf, &npa, pcctwoprint);
	return (1);
}

/* ARGSUSED */
static void
pcctwoisrlink(cookie, fn, arg, ipl, vec, evcnt)
	void *cookie;
	int (*fn)(void *);
	void *arg;
	int ipl, vec;
	struct evcnt *evcnt;
{
	static struct intrhand pcctwo_handlers[PCCTWOV_MAX];

	struct intrhand * ihand = &pcctwo_handlers[vec - PCCTWO_VECBASE];

	/* One handler per vector, please! */
	KASSERT(ihand->ih_fn == NULL);

	ihand->ih_fn = fn;
	ihand->ih_arg = arg;
	ihand->ih_ipl = ipl;
	/* When 'arg' != NULL, pass 'arg' to handler instead of eframe */
	ihand->ih_wantframe = (arg == NULL);

	intr_establish(vec, ihand, evcnt != NULL ? evcnt->ev_name : "pcctwo");
}

/* ARGSUSED */
static void
pcctwoisrunlink(cookie, vec)
	void *cookie;
	int vec;
{
	printf("pcctwoisrunlink: unimplemented\n"); /* XXX TKM - implement */
}

/* ARGSUSED */
static struct evcnt *
pcctwoisrevcnt(cookie, ipl)
	void *cookie;
	int ipl;
{
	struct pcctwo_softc *sc = (struct pcctwo_softc *) cookie;

	return &sc->sc_evcnt;
}
