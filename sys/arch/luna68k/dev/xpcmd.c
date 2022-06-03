/* 
 * XXX TODO:
 * - add copyright notice (moveccr?)
 * - KNF (make READY/CMD/RESULT static?)
 */

#include <sys/types.h>

#if defined(_KERNEL)

#include <sys/stdint.h>
#include <machine/param.h>

#include <arch/luna68k/dev/xpbusvar.h>
#include <arch/luna68k/dev/xpcmd.h>
#include <arch/luna68k/dev/xplx/xplxdefs.h>
#else

#include <stdint.h>
#include "xpbusvar.h"
#include "xpcmd.h"
#include "xplx/xplxdefs.h"

#endif

const int READY = 0;
const int CMD = 1;
const int RESULT = 2;

static
int
xp_waitfor_ready(int xplx_devid, int timeout)
{
	uint16_t addr = XPLX_VAR_BASE + xplx_devid * 16;
	do {
		int r;
		r = xp_readmem8(addr + READY);
		if (r) return 1;
		delay(100);
	} while (timeout--);
	return 0;
}

int
xp_cmd_nowait(int xplx_devid, uint8_t cmd)
{
	uint8_t r;

	uint16_t addr = XPLX_VAR_BASE + xplx_devid * 16;
	r = xp_waitfor_ready(xplx_devid, 1000);
	if (r == 0) return 0;
	xp_writemem8(addr + RESULT, 0);
	xp_writemem8(addr + CMD, cmd);
	return XPLX_R_OK;
}

int
xp_cmd(int xplx_devid, uint8_t cmd)
{
	uint8_t rv;
	uint8_t r;

	uint16_t addr = XPLX_VAR_BASE + xplx_devid * 16;
	r = xp_waitfor_ready(xplx_devid, 1000);
	if (r == 0) return 0;
	xp_writemem8(addr + RESULT, 0);
	xp_writemem8(addr + CMD, cmd);
	while ((rv = xp_readmem8(addr + RESULT)) == 0);
	return rv;
}

