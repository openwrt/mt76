/*
 * Copyright (C) 2016 Felix Fietkau <nbd@openwrt.org>
 * Copyright (C) 2004 - 2009 Ivo van Doorn <IvDoorn@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __MT76_UTIL_H
#define __MT76_UTIL_H

#include <linux/skbuff.h>
#include <linux/bitops.h>

#ifndef GENMASK
#define GENMASK(h, l)       (((U32_C(1) << ((h) - (l) + 1)) - 1) << (l))
#endif


/*
 * Power of two check, this will check
 * if the mask that has been given contains and contiguous set of bits.
 * Note that we cannot use the is_power_of_2() function since this
 * check must be done at compile-time.
 */
#define is_power_of_two(x)	( !((x) & ((x)-1)) )
#define low_bit_mask(x)		( ((x)-1) & ~(x) )
#define is_valid_mask(x)	is_power_of_two(1LU + (x) + low_bit_mask(x))

/*
 * Macros to find first set bit in a variable.
 * These macros behave the same as the __ffs() functions but
 * the most important difference that this is done during
 * compile-time rather then run-time.
 */
#define compile_ffs2(__x) \
	__builtin_choose_expr(((__x) & 0x1), 0, 1)

#define compile_ffs4(__x) \
	__builtin_choose_expr(((__x) & 0x3), \
			      (compile_ffs2((__x))), \
			      (compile_ffs2((__x) >> 2) + 2))

#define compile_ffs8(__x) \
	__builtin_choose_expr(((__x) & 0xf), \
			      (compile_ffs4((__x))), \
			      (compile_ffs4((__x) >> 4) + 4))

#define compile_ffs16(__x) \
	__builtin_choose_expr(((__x) & 0xff), \
			      (compile_ffs8((__x))), \
			      (compile_ffs8((__x) >> 8) + 8))

#define compile_ffs32(__x) \
	__builtin_choose_expr(((__x) & 0xffff), \
			      (compile_ffs16((__x))), \
			      (compile_ffs16((__x) >> 16) + 16))


/*
 * This macro will check the requirements for the FIELD{8,16,32} macros
 * The mask should be a constant non-zero contiguous set of bits which
 * does not exceed the given typelimit.
 */
#define FIELD_CHECK(__mask) \
	BUILD_BUG_ON(!(__mask) || !is_valid_mask(__mask))

#define MT76_SET(_mask, _val) \
	({ FIELD_CHECK(_mask); (((u32) (_val)) << compile_ffs32(_mask)) & _mask; })

#define MT76_GET(_mask, _val) \
	({ FIELD_CHECK(_mask); (u32) (((_val) & _mask) >> compile_ffs32(_mask)); })

#define MT76_INCR(_var, _size) \
	_var = (((_var) + 1) % _size)

int mt76_insert_hdr_pad(struct sk_buff *skb);
void mt76_remove_hdr_pad(struct sk_buff *skb);
int mt76_wcid_alloc(unsigned long *mask, int size);

static inline void
mt76_wcid_free(unsigned long *mask, int idx)
{
	mask[idx / BITS_PER_LONG] &= ~BIT(idx % BITS_PER_LONG);
}

static inline void
mt76_skb_set_moredata(struct sk_buff *skb, bool enable)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;

	if (enable)
		hdr->frame_control |= cpu_to_le16(IEEE80211_FCTL_MOREDATA);
	else
		hdr->frame_control &= ~cpu_to_le16(IEEE80211_FCTL_MOREDATA);
}

#endif
