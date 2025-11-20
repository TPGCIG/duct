/*
 * Copyright 2025, The University of Queensland
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#if !defined(_DEV_DUCTVAR_H)
#define	_DEV_DUCTVAR_H

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/ioccom.h>

struct duct_info_arg {
	uint32_t	duct_major;
	uint32_t	duct_minor;
	uint32_t	duct_hwaddr;
};

struct duct_mcast_arg {
	uint32_t	duct_mask;
	uint32_t	duct_addr;
};

#define	DUCTIOC_GET_INFO	_IOR('z', 0, struct duct_info_arg)
#define	DUCTIOC_ADD_MCAST	_IOW('z', 1, struct duct_mcast_arg)
#define	DUCTIOC_RM_MCAST	_IOW('z', 2, struct duct_mcast_arg)

struct duct_packet_hdr {
	uint32_t	dpkt_source;
	uint32_t	dpkt_destination;
	uint32_t	dpkt_length;
	uint32_t	dpkt_reserved;
};

#endif /* !_DEV_DUCTVAR_H */
