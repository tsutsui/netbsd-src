/*	$NetBSD: clmpcc_pcctwo.c,v 1.9 2005/02/04 02:10:43 perry Exp $	*/

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
 *        This product includes software developed by the NetBSD
 *        Foundation, Inc. and its contributors.
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
 * Cirrus Logic CD2401 4-channel serial chip. PCCchip2 Front-end.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: clmpcc_pcctwo.c,v 1.9 2005/02/04 02:10:43 perry Exp $");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/device.h>
#ifndef __HAVE_GENERIC_SOFT_INTERRUPTS
#include <sys/callout.h>
#endif
#include <sys/conf.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/tty.h>
#include <sys/time.h>
#include <sys/kernel.h>
#include <sys/syslog.h>

#include <dev/cons.h>

#include <machine/cpu.h>
#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/ic/clmpccvar.h>

#include <dev/mvme/pcctwovar.h>
#include <dev/mvme/pcctworeg.h>

#if defined(mvme68k)
#include <mvme68k/dev/mainbus.h>
#elif defined(mvme88k)
#include <mvme88k/dev/mainbus.h>
#endif

#include "opt_clmpcc_console.h"

#ifndef CLMPCC_CONSOLE_CHAN
  #define CLMPCC_CONSOLE_CHAN 0
#endif

#ifndef CLMPCC_CONSOLE_RATE
  #define CLMPCC_CONSOLE_RATE 9600
#endif

#define CLMPCC_REG_SIZE 0x100

/* Definition of the driver for autoconfig. */
int clmpcc_pcctwo_match(struct device *, struct cfdata *, void *);
void clmpcc_pcctwo_attach(struct device *, struct device *, void *);
#ifndef __HAVE_GENERIC_SOFT_INTERRUPTS
void clmpcc_pcctwo_softhook(struct clmpcc_softc *);
#endif
void clmpcc_pcctwo_iackhook(struct clmpcc_softc *, int);

struct clmpcc_pcctwo_softc
{
	struct clmpcc_softc	sc_super;
#define sc_dev sc_super.sc_dev
#define sc_iot sc_super.sc_iot
#define sc_ioh sc_super.sc_ioh
#define sc_clk sc_super.sc_clk
#define sc_byteswap sc_super.sc_byteswap
#define sc_swaprtsdtr sc_super.sc_swaprtsdtr
#define sc_iackhook sc_super.sc_iackhook
#define sc_softhook sc_super.sc_softhook
#define sc_vector_base sc_super.sc_vector_base
#define sc_rpilr sc_super.sc_rpilr
#define sc_tpilr sc_super.sc_tpilr
#define sc_mpilr sc_super.sc_mpilr
#define sc_evcnt sc_super.sc_evcnt

#ifndef __HAVE_GENERIC_SOFT_INTERRUPTS
	struct callout		sc_callout;
#endif
};

CFATTACH_DECL(clmpcc_pcctwo, sizeof(struct clmpcc_pcctwo_softc),
    clmpcc_pcctwo_match, clmpcc_pcctwo_attach, NULL, NULL);

extern struct cfdriver clmpcc_cd;

extern const struct cdevsw clmpcc_cdevsw;

/*
 * For clmpcccn*()
 */
cons_decl(clmpcc);

static struct clmpcc_pcctwo_softc cons_sc;

/* Used by clmpcc_pcctwo_iackhook() prior to clmpcc_attach() being called */
static bus_space_handle_t cons_pcctwo_bus_handle;


/*
 * Is the CD2401 chip present?
 */
int
clmpcc_pcctwo_match(parent, cf, aux)
	struct device *parent;
	struct cfdata *cf;
	void *aux;
{
	struct pcctwo_attach_args *pa;

	pa = aux;

	if (strcmp(pa->pa_name, clmpcc_cd.cd_name))
		return (0);

	pa->pa_ipl = cf->pcctwocf_ipl;

	return (1);
}

/*
 * Attach a found CD2401.
 */
void
clmpcc_pcctwo_attach(parent, self, aux)
	struct device *parent;
	struct device *self;
	void *aux;
{
	struct clmpcc_pcctwo_softc *sc;
	struct pcctwo_attach_args *pa;
	int level = pa->pa_ipl;

	sc = (struct clmpcc_pcctwo_softc *)self;
	pa = aux;
	level = pa->pa_ipl;
	sc->sc_iot = pa->pa_bust;
	bus_space_map(pa->pa_bust, pa->pa_offset, CLMPCC_REG_SIZE, 0, &sc->sc_ioh);

	sc->sc_clk = 20000000;
	sc->sc_byteswap = CLMPCC_BYTESWAP_LOW;
	sc->sc_swaprtsdtr = 1;
	sc->sc_iackhook = clmpcc_pcctwo_iackhook;
	sc->sc_vector_base = PCCTWO_SCC_VECBASE;
	sc->sc_rpilr = 0x03;
	sc->sc_tpilr = 0x02;
	sc->sc_mpilr = 0x01;
	sc->sc_evcnt = pcctwointr_evcnt(level);

#ifndef __HAVE_GENERIC_SOFT_INTERRUPTS
	callout_init(&sc->sc_callout);
	sc->sc_softhook = clmpcc_pcctwo_softhook;
#endif
	/* Do common parts of CD2401 configuration. */
	clmpcc_attach(&sc->sc_super);

	/* Hook the interrupts */
	pcctwointr_establish(PCCTWOV_SCC_RX, clmpcc_rxintr, level, sc, NULL);
	pcctwointr_establish(PCCTWOV_SCC_RX_EXCEP, clmpcc_rxintr, level, sc,
	    NULL);
	pcctwointr_establish(PCCTWOV_SCC_TX, clmpcc_txintr, level, sc, NULL);
	pcctwointr_establish(PCCTWOV_SCC_MODEM, clmpcc_mdintr, level, sc, NULL);
}

#ifndef __HAVE_GENERIC_SOFT_INTERRUPTS
void clmpcc_pcctwo_softhook(ssc)
	struct clmpcc_softc *ssc;
{
	struct clmpcc_pcctwo_softc *sc = (struct clmpcc_pcctwo_softc *)ssc;

	callout_reset(&sc->sc_callout, 1, clmpcc_softintr, sc);
}
#endif

void
clmpcc_pcctwo_iackhook(ssc, which)
	struct clmpcc_softc *ssc;
	int which;
{
	struct clmpcc_pcctwo_softc *sc = (struct clmpcc_pcctwo_softc *)ssc;
	bus_size_t offset;
	volatile u_char foo;

	switch (which) {
	case CLMPCC_IACK_MODEM:
		offset = PCC2REG_SCC_MODEM_PIACK;
		break;

	case CLMPCC_IACK_RX:
		offset = PCC2REG_SCC_RX_PIACK;
		break;

	case CLMPCC_IACK_TX:
		offset = PCC2REG_SCC_TX_PIACK;
		break;
	default:
#ifdef DEBUG
		printf("%s: Invalid IACK number '%d'\n",
		    sc->sc_dev.dv_xname, which);
#endif
		panic("clmpcc_pcctwo_iackhook %d", which);
	}

	if (sc == &cons_sc)
	{
		/*
		 * We need to fake the tag and handle since 'sys_pcctwo' will
		 * be NULL during early system startup...
		 */
		foo = bus_space_read_1(sc->sc_iot, cons_pcctwo_bus_handle, offset);
	}
	else
	{
		foo = pcc2_reg_read(sys_pcctwo, offset);
	}
}


/****************************************************************
 * Console support functions (MVME PCCchip2 specific!)
 ****************************************************************/

/*
 * Check for CD2401 console.
 */
void
clmpcccnprobe(cp)
	struct consdev *cp;
{
	int maj;

#if defined(mvme68k)
	if (machineid != MVME_167 && machineid != MVME_177)
#elif defined(mvme88k)
	if (machineid != MVME_187 && machineid != MVME_197)
#endif
	{
		cp->cn_pri = CN_DEAD;
		return;
	}

	/*
	 * Locate the major number
	 */
	maj = cdevsw_lookup_major(&clmpcc_cdevsw);

	/* Initialize required fields. */
	cp->cn_dev = makedev(maj, CLMPCC_CONSOLE_CHAN);
	cp->cn_pri = CN_NORMAL;
}

void
clmpcccninit(cp)
	struct consdev *cp;
{
	cons_sc.sc_iot = &_mainbus_space_tag;

#if defined(mvme88k)
	bus_space_map(cons_sc.sc_iot,
	    (vaddr_t)PCCTWO_PADDR(PCCTWO_SCC_OFF),
	    CLMPCC_REG_SIZE, 0, &cons_sc.sc_ioh);

	bus_space_map(cons_sc.sc_iot,
	    (vaddr_t)PCCTWO_PADDR(PCCTWO_REG_OFF),
	    PCC2REG_SIZE, 0, &cons_pcctwo_bus_handle);
#elif defined(mvme68k)
	bus_space_map(cons_sc.sc_iot,
	    intiobase_phys + MAINBUS_PCCTWO_OFFSET + PCCTWO_SCC_OFF,
	    CLMPCC_REG_SIZE, 0, &cons_sc.sc_ioh);

	cons_pcctwo_bus_handle = (bus_space_handle_t)
	    & (intiobase[MAINBUS_PCCTWO_OFFSET + PCCTWO_REG_OFF]);
#else
#error "Need PCC2 & SCC mappings for console I/O"
#endif
	cons_sc.sc_clk = 20000000;
	cons_sc.sc_byteswap = CLMPCC_BYTESWAP_LOW;
	cons_sc.sc_swaprtsdtr = 1;
	cons_sc.sc_iackhook = clmpcc_pcctwo_iackhook;
	cons_sc.sc_vector_base = PCCTWO_SCC_VECBASE;
	cons_sc.sc_rpilr = 0x03;
	cons_sc.sc_tpilr = 0x02;
	cons_sc.sc_mpilr = 0x01;

	clmpcc_cnattach(&cons_sc.sc_super, CLMPCC_CONSOLE_CHAN, CLMPCC_CONSOLE_RATE);
}
