/*	$NetBSD$	*/

/*-
 * Copyright (c) 2026 Izumi Tsutsui.  All rights reserved.
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

#ifndef _NEWS68K_FBBMIO_H_
#define _NEWS68K_FBBMIO_H_

#include <sys/types.h>
#include <sys/ioccom.h>

/* Private and experimental NWB-225 stored-rectangle upload interface. */
struct fbbmio_hwrite {
	const void *fh_data;
	uint32_t fh_stride;
	uint16_t fh_x;
	uint16_t fh_y;
	uint16_t fh_width;
	uint16_t fh_height;
};

/* Command 225 is mnemonic for the NWB-225; this ioctl is fbbm-private. */
#define FBBMIO_HWRITE	_IOW('F', 225, struct fbbmio_hwrite)

#endif /* _NEWS68K_FBBMIO_H_ */
