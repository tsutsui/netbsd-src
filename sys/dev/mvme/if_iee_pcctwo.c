/* $NetBSD: if_iee_pcctwo.c,v 1.1 2004/03/12 11:41:39 jkunz Exp $ */

/*
 * Copyright (c) 2006 Timothy McIntosh
 * Copyright (c) 2003 Jochen Kunz.
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
 * 3. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL JOCHEN KUNZ
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * MVME PCC2 MD frontend for the iee(4) Intel i82596 Ethernet driver.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: if_iee_pcctwo.c,v 1.1 2006/10/15 11:41:39 tmcintos Exp $");

/* autoconfig and device stuff */
#include <sys/param.h>
#include <sys/device.h>
#include <sys/conf.h>
#include <machine/autoconf.h>
#include <hp700/dev/cpudevs.h>
#include <hp700/gsc/gscbusvar.h>
#include "locators.h"
#include "ioconf.h"

/* bus_space / bus_dma etc. */
#include <machine/bus.h>
#include <machine/intr.h>

/* general system data and functions */
#include <sys/systm.h>
#include <sys/ioctl.h>
#include <sys/ioccom.h>
#include <sys/types.h>

/* tsleep / sleep / wakeup */
#include <sys/proc.h>
/* hz for above */
#include <sys/kernel.h>

/* network stuff */
#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_ether.h>
#include <sys/socket.h>
#include <sys/mbuf.h>

#include "bpfilter.h"
#if NBPFILTER > 0 
#include <net/bpf.h>
#endif 

#include <dev/ic/i82596reg.h>
#include <dev/ic/i82596var.h>

#include <dev/mvme/if_iereg.h>
#include <dev/mvme/pcctwovar.h>
#include <dev/mvme/pcctworeg.h>

#define IEE_ISCP_BUSSY 0x1

#define CACHE_BYTE_ALIGNMENT	16


/* autoconfig stuff */
static int iee_pcctwo_match(struct device *, struct cfdata *, void *);
static void iee_pcctwo_attach(struct device *, struct device *, void *);
static int iee_pcctwo_detach(struct device*, int);


struct iee_pcctwo_softc {
	struct iee_softc iee_sc;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;
	void *sc_ih;
};



CFATTACH_DECL(
	iee_pcctwo,
	sizeof(struct iee_pcctwo_softc),
	iee_pcctwo_match,
	iee_pcctwo_attach,
	iee_pcctwo_detach,
    	NULL
);



int iee_pcctwo_cmd(struct iee_softc *, u_int32_t);
int iee_pcctwo_reset(struct iee_softc *);
static int iee_failintr(void *v);



int
iee_pcctwo_cmd(struct iee_softc *sc, u_int32_t cmd)
{
	struct iee_pcctwo_softc *sc_pcctwo = (struct iee_pcctwo_softc *) sc;
	int n;

	SC_SCB->scb_cmd = cmd;
	bus_dmamap_sync(sc->sc_dmat, sc->sc_shmem_map, IEE_SCB_OFF, IEE_SCB_SZ,
	    BUS_DMASYNC_PREWRITE);
	/* Issue a Channel Attention to force the chip to read the cmd. */
	bus_space_write_4(sc_pcctwo->sc_iot, sc_pcctwo->sc_ioh, IE_MPUREG_CA, 0);
	/* Wait for the cmd to finish */
	for (n = 0 ; n < 100000; n++) {
		DELAY(1);
		bus_dmamap_sync(sc->sc_dmat, sc->sc_shmem_map, IEE_SCB_OFF, 
		    IEE_SCB_SZ, BUS_DMASYNC_PREREAD);
		if (SC_SCB->scb_cmd == 0)
			break;
	}
	bus_dmamap_sync(sc->sc_dmat, sc->sc_shmem_map, IEE_SCB_OFF, IEE_SCB_SZ,
	    BUS_DMASYNC_PREREAD);
	if (cmd == IEE_SCB_ACK)
	{
		/* Clear interrupt */
		pcc2_reg_write(sys_pcctwo, PCC2REG_ETH_ICSR,
		    PCCTWO_ICR_ICLR | pcc2_reg_read(sys_pcctwo, PCC2REG_ETH_ICSR));
	}
	if (n < 100000)
		return(0);
	printf("%s: iee_pcctwo_cmd: timeout n=%d\n", sc->sc_dev.dv_xname, n);
	return(-1);
}



int
iee_pcctwo_reset(struct iee_softc *sc)
{
	struct iee_pcctwo_softc *sc_pcctwo = (struct iee_pcctwo_softc *) sc;
	int n;
	u_int32_t cmd;

	/* Make sure the bussy byte is set and the cache is flushed. */
	SC_ISCP->iscp_bussy = IEE_ISCP_BUSSY;
	bus_dmamap_sync(sc->sc_dmat, sc->sc_shmem_map, IEE_SCP_OFF, IEE_SCP_SZ 
	    + IEE_ISCP_SZ + IEE_SCB_SZ, BUS_DMASYNC_PREWRITE);
	/* Setup the PORT Command with pointer to SCP. */
	cmd = IEE_PORT_SCP | IEE_PHYS_SHMEM(IEE_SCP_OFF);
	/* Initiate a Hardware reset. */
	bus_space_write_2(sc_pcctwo->sc_iot, sc_pcctwo->sc_ioh, IE_MPUREG_UPPER, IEE_PORT_RESET);
	bus_space_write_2(sc_pcctwo->sc_iot, sc_pcctwo->sc_ioh, IE_MPUREG_LOWER, 0);
	DELAY(1000);
	/* Write it to the chip, it wants this in two 16 bit parts. */
	bus_space_write_2(sc_pcctwo->sc_iot, sc_pcctwo->sc_ioh, IE_MPUREG_UPPER,
	    (cmd & 0xffff));
	DELAY(1000);
	bus_space_write_2(sc_pcctwo->sc_iot, sc_pcctwo->sc_ioh, IE_MPUREG_LOWER,
	    (cmd >> 16) & 0xffff);
	DELAY(1000);
	/* Issue a Channel Attention to read SCP */
	bus_space_write_4(sc_pcctwo->sc_iot, sc_pcctwo->sc_ioh, IE_MPUREG_CA, 0);
	/* Wait for the chip to initialize and read SCP and ISCP. */
	for (n = 0 ; n < 1000; n++) {
		bus_dmamap_sync(sc->sc_dmat, sc->sc_shmem_map, IEE_ISCP_OFF, 
		    IEE_ISCP_SZ, BUS_DMASYNC_PREREAD);
		if (SC_ISCP->iscp_bussy != IEE_ISCP_BUSSY)
			break;
		DELAY(100);
	}
	bus_dmamap_sync(sc->sc_dmat, sc->sc_shmem_map, IEE_ISCP_OFF, 
	    IEE_ISCP_SZ, BUS_DMASYNC_PREREAD);
	if (n < 1000) {
		/* ACK interrupts we may have caused */
		(sc->sc_iee_cmd)(sc, IEE_SCB_ACK);
		return(0);
	}
	printf("%s: iee_pcctwo_reset timeout bussy=0x%x\n", sc->sc_dev.dv_xname, 
	    SC_ISCP->iscp_bussy);
	return(-1);
}




static int
iee_failintr(void *v)
{
	struct iee_softc *sc = v;

	/* safe: clear regular irq */
	pcc2_reg_write(sys_pcctwo, PCC2REG_ETH_ICSR,
	    PCCTWO_ICR_ICLR | pcc2_reg_read(sys_pcctwo, PCC2REG_ETH_ICSR));

	/* clear bus error irq */
	pcc2_reg_write(sys_pcctwo, PCC2REG_ETH_BERR_ICSR,
	    PCCTWO_ICR_ICLR | pcc2_reg_read(sys_pcctwo, PCC2REG_ETH_BERR_ICSR));

	/* reset error in status register */
	pcc2_reg_write(sys_pcctwo, PCC2REG_ETH_ERR_STATUS,
	    PCCTWO_ERR_SR_SCLR | pcc2_reg_read(sys_pcctwo, PCC2REG_ETH_ERR_STATUS));

	iee_pcctwo_reset(sc);

printf("iee_failintr\n");

	return (1);
}



static int
iee_pcctwo_match(struct device *parent, struct cfdata *match, void *aux)
{
	struct pcctwo_attach_args *pa = aux;

	if (strcmp(pa->pa_name, iee_cd.cd_name))
		return (0);

	pa->pa_ipl = match->pcctwocf_ipl;

	return (1);
}



static void
iee_pcctwo_attach(struct device *parent, struct device *self, void *aux)
{
	struct iee_pcctwo_softc *sc_pcctwo = (struct iee_pcctwo_softc *) self;
	struct iee_softc *sc = (struct iee_softc *) self;
	struct pcctwo_attach_args *pa = aux;
	int media[2];
	int rsegs;

	sc->sc_type = I82596_CA;	/* LASI based */
	sc->sc_flags = IEE_NEED_SWAP;
	sc->sc_cl_align = CACHE_BYTE_ALIGNMENT;

	sc_pcctwo->sc_iot = pa->pa_bust;
	if (bus_space_map(sc_pcctwo->sc_iot, pa->pa_offset, IE_MPUREG_SIZE, 0, 
	    &sc_pcctwo->sc_ioh)) {
		aprint_normal(": iee_pcctwo_attach: can't map I/O space\n");
		return;
	}

	sc->sc_dmat = pa->pa_dmat;
	if (bus_dmamem_alloc(sc->sc_dmat, IEE_SHMEM_MAX, PAGE_SIZE, 0,
	    &sc->sc_dma_segs, 1, &rsegs,
	    BUS_DMA_NOWAIT | BUS_DMA_ONBOARD_RAM ) != 0) {
		aprint_normal(": iee_pcctwo_attach: can't allocate %d bytes of "
		    "DMA memory\n", (int)IEE_SHMEM_MAX);
		return;
	}
	if (bus_dmamem_map(sc->sc_dmat, &sc->sc_dma_segs, rsegs, IEE_SHMEM_MAX, 
	    &sc->sc_shmem_addr, BUS_DMA_NOWAIT ) != 0) {
		aprint_normal(": iee_pcctwo_attach: can't map DMA memory\n");
		bus_dmamem_free(sc->sc_dmat, &sc->sc_dma_segs, rsegs);
		return;
	}
	if (bus_dmamap_create(sc->sc_dmat, IEE_SHMEM_MAX, rsegs, 
	    IEE_SHMEM_MAX, 0, BUS_DMA_NOWAIT | BUS_DMA_ALLOCNOW,
	    &sc->sc_shmem_map) != 0) {
		aprint_normal(": iee_pcctwo_attach: can't create DMA map\n");
		bus_dmamem_unmap(sc->sc_dmat, sc->sc_shmem_addr, IEE_SHMEM_MAX);
		bus_dmamem_free(sc->sc_dmat, &sc->sc_dma_segs, rsegs);
		return;
	}
#if 0
	if (bus_dmamap_load(sc->sc_dmat, sc->sc_shmem_map, sc->sc_shmem_addr,
	    IEE_SHMEM_MAX, NULL, BUS_DMA_NOWAIT) != 0) {
#else
	if (bus_dmamap_load_raw(sc->sc_dmat, sc->sc_shmem_map, &sc->sc_dma_segs, rsegs,
	    IEE_SHMEM_MAX, BUS_DMA_NOWAIT) != 0) {
#endif
		aprint_normal(": iee_pcctwo_attach: can't load DMA map\n");
		bus_dmamap_destroy(sc->sc_dmat, sc->sc_shmem_map);
		bus_dmamem_unmap(sc->sc_dmat, sc->sc_shmem_addr, IEE_SHMEM_MAX);
		bus_dmamem_free(sc->sc_dmat, &sc->sc_dma_segs, rsegs);
		return;
	}
	memset(sc->sc_shmem_addr, 0, IEE_SHMEM_MAX);

	/* Setup SYSBUS byte. */
	SC_SCP->scp_sysbus = IEE_SYSBUS_BE | IEE_SYSBUS_LOCK |
	    IEE_SYSBUS_LIEAR | IEE_SYSBUS_STD;

	pcctwointr_establish(PCCTWOV_LANC_IRQ, iee_intr, pa->pa_ipl, sc, NULL);
	pcctwointr_establish(PCCTWOV_LANC_ERR, iee_failintr, pa->pa_ipl, sc, NULL);

	sc->sc_iee_reset = iee_pcctwo_reset;
	sc->sc_iee_cmd = iee_pcctwo_cmd;
	sc->sc_mediachange = NULL;
	sc->sc_mediastatus = NULL;

	media[0] = IFM_ETHER | IFM_MANUAL;
	media[1] = IFM_ETHER | IFM_AUTO;
	iee_attach(sc, mvme_ea, media, 2, IFM_ETHER | IFM_AUTO);
}



int
iee_pcctwo_detach(struct device* self, int flags)
{
	struct iee_pcctwo_softc *sc_pcctwo = (struct iee_pcctwo_softc *) self;
	struct iee_softc *sc = (struct iee_softc *) self;

	iee_detach(sc, flags);
	bus_space_unmap(sc_pcctwo->sc_iot, sc_pcctwo->sc_ioh, IE_MPUREG_SIZE);
	bus_dmamap_unload(sc->sc_dmat, sc->sc_shmem_map);
	bus_dmamap_destroy(sc->sc_dmat, sc->sc_shmem_map);
	bus_dmamem_unmap(sc->sc_dmat, sc->sc_shmem_addr, IEE_SHMEM_MAX);
	bus_dmamem_free(sc->sc_dmat, &sc->sc_dma_segs, 1);
	pcctwointr_disestablish(PCCTWOV_LANC_IRQ);
	pcctwointr_disestablish(PCCTWOV_LANC_ERR);
	return(0);
}
