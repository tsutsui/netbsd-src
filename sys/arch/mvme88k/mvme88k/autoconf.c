/*	$OpenBSD: autoconf.c,v 1.38 2007/06/01 19:25:10 deraadt Exp $	*/
/*
 * Copyright (c) 1998 Steve Murphree, Jr.
 * Copyright (c) 1996 Nivas Madhur
 * Copyright (c) 1994 Christian E. Hopps
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
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Christian E. Hopps.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission
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
 *
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/buf.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/reboot.h>
#include <sys/device.h>

#include <machine/asm_macro.h>   /* enable/disable interrupts */
#include <machine/autoconf.h>
#include <machine/bugio.h>

#include <dev/cons.h>

#include <dev/mvme/pcctworeg.h>
#include <dev/mvme/pcctwovar.h>

#include <dev/scsipi/scsi_all.h>
#include <dev/scsipi/scsipi_all.h>
#include <dev/scsipi/scsiconf.h>

/*
 * The following several variables are related to
 * the configuration process, and are used in initializing
 * the machine.
 */

void	*bootaddr;	/* PA of boot device */
int	bootctrllun;	/* ctrl_lun of boot device */
int	bootdevlun;	/* dev_lun of boot device */

/* Prototypes for private functions */
static int	get_target(int *, int *, int *);

/*
 * configure all devices on the system.
 *   called at boot time _after_ autoconfiguration.
 */
void
cpu_configure()
{
	if (config_rootfound("mainbus", "mainbus") == 0)
		panic("no mainbus found");

	/*
	 * Turn external interrupts on.
	 *
	 * XXX We have a race here. If we enable interrupts after setroot(),
	 * the kernel dies.
	 */
	set_psr(get_psr() & ~PSR_IND);
	spl0();
}

void
cpu_rootconf()
{
	printf("boot device: %s",
		booted_device ? booted_device->dv_xname : "<unknown>");

	/* XXX - booted_partition currently not initialized -TKM */
	if (booted_partition > 0)
		printf(" (partition %d)\n", booted_partition);
	else
		printf("\n");

	setroot(booted_device, booted_partition);
}

/*
 * Try to determine the boot device.
 */
void
device_register(struct device *dev, void *aux)
{
	static int found;

	if (found)
		return;

	if (booted_partition == -1) /* ignore flag from controller driver? */
		return;

	/*
	 * ethernet: ie,le
	 */
	if (strncmp("ie", dev->dv_xname, 2) == 0 ||
	    strncmp("le", dev->dv_xname, 2) == 0) {
		struct pcctwo_attach_args *pa = aux;

		if (PCCTWO_PADDR(pa->pa_offset - pa->_pa_base) == bootaddr) {
			booted_device = dev;
			found = 1;
			return;
		}
	}

	/*
	 * scsi: sd,cd
	 */
	else if (strncmp("cd", dev->dv_xname, 2) == 0 ||
	    strncmp("sd", dev->dv_xname, 2) == 0 ||
	    strncmp("st", dev->dv_xname, 2) == 0) {
		struct scsipibus_attach_args *sa = aux;
		int target, bus, lun;

		if (get_target(&target, &bus, &lun) != 0)
			return;

#if 0 /* XXX - Not needed until VMEbus SCSI (vs) driver is ported -TKM */ 
		/* make sure we are on the expected scsibus */
		if (bootbus != bus)
			return;
#endif

		if (sa->sa_periph->periph_target == target &&
		    sa->sa_periph->periph_lun == lun) {
			booted_device = dev;
			found = 1;
			return;
		}
	}
}

/*
 * Returns the ID of the SCSI disk based on Motorola's CLUN/DLUN stuff
 * This handles SBC SCSI and MVME32[78].
 */
int
get_target(int *target, int *bus, int *lun)
{
	switch (bootctrllun) {
	/* built-in controller */
	case 0x00:
	/* MVME327 */
	case 0x02:
	case 0x03:
		*bus = 0;
		*target = (bootdevlun & 0x70) >> 4;
		*lun = (bootdevlun & 0x07);
		return (0);
	/* MVME328 */
	case 0x06:
	case 0x07:
	case 0x16:
	case 0x17:
	case 0x18:
	case 0x19:
		*bus = (bootdevlun & 0x40) >> 6;
		*target = (bootdevlun & 0x38) >> 3;
		*lun = (bootdevlun & 0x07);
		return (0);
	default:
		return (ENODEV);
	}
}
