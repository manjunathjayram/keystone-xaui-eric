/*
 * TI Common Platform Time Sync
 *
 * Copyright (C) 2012 Richard Cochran <richardcochran@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include <linux/err.h>
#include <linux/if.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/net_tstamp.h>
#include <linux/ptp_classify.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include "cpts.h"

#ifdef CONFIG_TI_CPTS

static struct sock_filter ptp_default_filter[] = {
	PTP_FILTER
};

#define cpts_read32(c, r)	__raw_readl(&c->reg->r)
#define cpts_write32(c, v, r)	__raw_writel(v, &c->reg->r)

#define CPTS_COUNTER_BITS	32
#define TS_COMP_LAST_REG(cpts)	(cpts->ts_comp_last & cpts->cc.mask)

static int event_expired(struct cpts_event *event)
{
	return time_after(jiffies, event->tmo);
}

static int event_type(struct cpts_event *event)
{
	return (event->high >> EVENT_TYPE_SHIFT) & EVENT_TYPE_MASK;
}

static int cpts_fifo_pop(struct cpts *cpts, u32 *high, u32 *low)
{
	u32 r = cpts_read32(cpts, intstat_raw);

	if (r & TS_PEND_RAW) {
		*high = cpts_read32(cpts, event_high);
		*low  = cpts_read32(cpts, event_low);
		cpts_write32(cpts, EVENT_POP, event_pop);
		return 0;
	}
	return -1;
}

static int cpts_event_list_clean_up(struct cpts *cpts)
{
	struct list_head *this, *next;
	struct cpts_event *event;
	int removed = 0;

	list_for_each_safe(this, next, &cpts->events) {
		event = list_entry(this, struct cpts_event, list);
		if (event_expired(event)) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			++removed;
		}
	}
	return removed;
}

/*
 * Returns zero if matching event type was found.
 */
static int cpts_fifo_read(struct cpts *cpts, int match)
{
	int i, type = -1, removed;
	u32 hi, lo;
	struct cpts_event *event;

	for (i = 0; i < CPTS_FIFO_DEPTH; i++) {
		if (cpts_fifo_pop(cpts, &hi, &lo))
			break;
		if (list_empty(&cpts->pool)) {
			removed = cpts_event_list_clean_up(cpts);
			if (!removed) {
				pr_err("cpts: event pool is empty\n");
				return -1;
			}
			pr_debug("cpts: list cleaned up %d\n", removed);
		}
		event = list_first_entry(&cpts->pool, struct cpts_event, list);
		event->tmo = jiffies + 2;
		event->high = hi;
		event->low = lo;
		type = event_type(event);
		switch (type) {
		case CPTS_EV_PUSH:
		case CPTS_EV_RX:
		case CPTS_EV_TX:
		case CPTS_EV_COMP:
			list_del_init(&event->list);
			list_add_tail(&event->list, &cpts->events);
			break;
		case CPTS_EV_ROLL:
		case CPTS_EV_HALF:
		case CPTS_EV_HW:
			break;
		default:
			pr_err("cpts: unkown event type\n");
			break;
		}
		if (type == match)
			break;
	}
	return type == match ? 0 : -1;
}

static cycle_t cpts_systim_read(const struct cyclecounter *cc)
{
	u64 val = 0;
	struct cpts_event *event;
	struct list_head *this, *next;
	struct cpts *cpts = container_of(cc, struct cpts, cc);

	cpts_write32(cpts, TS_PUSH, ts_push);
	if (cpts_fifo_read(cpts, CPTS_EV_PUSH))
		pr_err("cpts: unable to obtain a time stamp\n");

	list_for_each_safe(this, next, &cpts->events) {
		event = list_entry(this, struct cpts_event, list);
		if (event_type(event) == CPTS_EV_PUSH) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			val = event->low;
			break;
		}
	}

	return val;
}

static u64 cpts_cc_ns2cyc(struct cpts *cpts, u64 nsec)
{
	u64 max_ns, max_cyc, cyc = 0;
	u32 sh = cpts->cc.shift;

	max_ns = ((1ULL << (64 - sh)) - 1);
	max_cyc = div_u64(max_ns << cpts->cc.shift, cpts->cc.mult);

	while (nsec >= max_ns) {
		nsec -= max_ns;
		cyc += max_cyc;
	}

	if (nsec)
		cyc += div_u64(nsec << cpts->cc.shift, cpts->cc.mult);

	return cyc;
}

static inline void cpts_ts_comp_add_ns(struct cpts *cpts, s64 add_ns)
{
	u64 temp, cyc;
	int neg_adj = 0;

	if (add_ns < 0) {
		neg_adj = 1;
		add_ns = -add_ns;
	}

	if (add_ns == NSEC_PER_SEC) {
		/* avoid calculation */
		cpts->ts_comp_last += cpts->pps_one_sec;
		return;
	}

	temp = add_ns;
	cyc = cpts_cc_ns2cyc(cpts, temp);
	cpts->ts_comp_last += (neg_adj ? -cyc : cyc);
}

static inline void cpts_disable_ts_comp(struct cpts *cpts)
{
	cpts_write32(cpts, 0, ts_comp_length);
}

static inline void cpts_enable_ts_comp(struct cpts *cpts)
{
	cpts_write32(cpts, TS_COMP_LAST_REG(cpts), ts_comp_val);
	cpts_write32(cpts, cpts->ts_comp_length, ts_comp_length);
}

static void cpts_pps_adjfreq(struct cpts *cpts, u32 old_mult)
{
	struct timecounter *tc = &cpts->tc;
	s64 ns_to_pulse;

	ns_to_pulse = clocksource_cyc2ns(cpts->ts_comp_last - tc->cycle_last,
				old_mult, cpts->cc.shift);

	cpts->ts_comp_last = tc->cycle_last;
	/* add the ns to ts_comp to align on the sec boundary in new freq */
	cpts_ts_comp_add_ns(cpts, ns_to_pulse);
	/* enable ts_comp pulse with new val */
	cpts_enable_ts_comp(cpts);
	/* update one sec equivalent after freq adjust */
	cpts->pps_one_sec = cpts_cc_ns2cyc(cpts, NSEC_PER_SEC);
}

/* If delta < 0, eg. -3.25sec, ns_before_sec after adj is 0.25 sec
 * If delta > 0, eg. +3.25sec, ns_before_sec after adj is 0.75 sec
 * Assumes the ts_comp is stopped when this function is called
 */
static int cpts_pps_adjtime(struct cpts *cpts, s64 delta)
{
	s64 sec;
	int neg_adj = 0;
	s32 ns_before_sec;

	if (delta < 0) {
		neg_adj = 1;
		delta = -delta;
	}

	sec = div_s64_rem(delta, NSEC_PER_SEC, &ns_before_sec);

	if (ns_before_sec == 0 || !neg_adj)
		ns_before_sec = NSEC_PER_SEC - ns_before_sec;

	/* add the ns to ts_comp to align on the sec boundary */
	cpts_ts_comp_add_ns(cpts, ns_before_sec);
	/* enable ts_comp pulse with new val */
	cpts_enable_ts_comp(cpts);
	return 0;
}

/* Assumes the ts_comp is stopped when this function is called */
static inline void cpts_pps_settime(struct cpts *cpts,
				const struct timespec *ts)
{
	cpts->ts_comp_last = cpts->tc.cycle_last;
	/* align to next sec boundary and add one sec */
	cpts_ts_comp_add_ns(cpts, NSEC_PER_SEC - ts->tv_nsec);

	cpts_enable_ts_comp(cpts);
}

static int cpts_ts_comp_add_reload(struct cpts *cpts, s64 add_ns)
{
	struct list_head *this, *next;
	struct ptp_clock_event pevent;
	struct cpts_event *event;
	unsigned long flags;
	u64 ns;

	spin_lock_irqsave(&cpts->lock, flags);
	list_for_each_safe(this, next, &cpts->events) {
		event = list_entry(this, struct cpts_event, list);
		if (event_type(event) == CPTS_EV_COMP) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			if (TS_COMP_LAST_REG(cpts) != event->low) {
				pr_err("cpts ts_comp mismatch: %llx %08x\n",
					cpts->ts_comp_last, event->low);
				break;
			} else
				pr_debug("cpts comp ev tstamp: %u\n",
					event->low);

			/* report the event */
			ns = timecounter_cyc2time(&cpts->tc, event->low);
			pevent.type = PTP_CLOCK_PPSUSR;
			pevent.pps_times.ts_real = ns_to_timespec(ns);
			ptp_clock_event(cpts->clock, &pevent);

			/* reload: add ns to ts_comp */
			cpts_ts_comp_add_ns(cpts, add_ns);
			/* enable ts_comp pulse with new val */
			cpts_disable_ts_comp(cpts);
			cpts_enable_ts_comp(cpts);
			break;
		}
	}
	spin_unlock_irqrestore(&cpts->lock, flags);
	return 0;
}

/* PTP clock operations */

static int cpts_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	u64 adj;
	u32 diff, mult, old_mult;
	int neg_adj = 0;
	unsigned long flags;
	struct cpts *cpts = container_of(ptp, struct cpts, info);

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}
	mult = cpts->cc_mult;
	adj = mult;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);
	old_mult = cpts->cc.mult;

	spin_lock_irqsave(&cpts->lock, flags);

	if (cpts->pps_enable)
		cpts_disable_ts_comp(cpts);

	timecounter_read(&cpts->tc);

	cpts->cc.mult = neg_adj ? mult - diff : mult + diff;

	if (cpts->pps_enable)
		cpts_pps_adjfreq(cpts, old_mult);

	spin_unlock_irqrestore(&cpts->lock, flags);

	return 0;
}

static int cpts_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	s64 now;
	unsigned long flags;
	struct cpts *cpts = container_of(ptp, struct cpts, info);

	spin_lock_irqsave(&cpts->lock, flags);
	if (cpts->pps_enable)
		cpts_disable_ts_comp(cpts);
	now = timecounter_read(&cpts->tc);
	now += delta;
	timecounter_init(&cpts->tc, &cpts->cc, now);
	if (cpts->pps_enable)
		cpts_pps_adjtime(cpts, delta);
	spin_unlock_irqrestore(&cpts->lock, flags);

	return 0;
}

static int cpts_ptp_gettime(struct ptp_clock_info *ptp, struct timespec *ts)
{
	u64 ns;
	u32 remainder;
	unsigned long flags;
	struct cpts *cpts = container_of(ptp, struct cpts, info);

	spin_lock_irqsave(&cpts->lock, flags);
	ns = timecounter_read(&cpts->tc);
	spin_unlock_irqrestore(&cpts->lock, flags);

	ts->tv_sec = div_u64_rem(ns, 1000000000, &remainder);
	ts->tv_nsec = remainder;

	return 0;
}

static int cpts_ptp_settime(struct ptp_clock_info *ptp,
			    const struct timespec *ts)
{
	u64 ns;
	unsigned long flags;
	struct cpts *cpts = container_of(ptp, struct cpts, info);

	ns = ts->tv_sec * 1000000000ULL;
	ns += ts->tv_nsec;

	spin_lock_irqsave(&cpts->lock, flags);
	if (cpts->pps_enable)
		cpts_disable_ts_comp(cpts);
	timecounter_init(&cpts->tc, &cpts->cc, ns);
	if (cpts->pps_enable)
		cpts_pps_settime(cpts, ts);
	spin_unlock_irqrestore(&cpts->lock, flags);

	return 0;
}

/* PPS */

static int cpts_pps_reload(struct cpts *cpts)
{
	return cpts_ts_comp_add_reload(cpts, NSEC_PER_SEC);
}

static int cpts_pps_enable(struct cpts *cpts, int on)
{
	struct timespec ts;

	if (cpts->pps_enable == on)
		return 0;

	cpts->pps_enable = on;

	if (!on) {
		cpts_disable_ts_comp(cpts);
		return 0;
	}

	/* get current counter value */
	cpts_write32(cpts, CPTS_EN, control);
	cpts_write32(cpts, TS_PEND_EN, int_enable);
	cpts_ptp_gettime(&cpts->info, &ts);
	cpts->ts_comp_last = cpts->tc.cycle_last;
	/* align to next sec boundary and add one sec */
	cpts_ts_comp_add_ns(cpts, 2 * NSEC_PER_SEC - ts.tv_nsec);
	/* enable ts_comp pulse */
	cpts_disable_ts_comp(cpts);
	cpts_enable_ts_comp(cpts);
	return 0;
}

static int cpts_pps_init(struct cpts *cpts)
{
	cpts->pps_one_sec = cpts_cc_ns2cyc(cpts, NSEC_PER_SEC);
	return 0;
}

static int cpts_ptp_enable(struct ptp_clock_info *ptp,
			   struct ptp_clock_request *rq, int on)
{
	struct cpts *cpts = container_of(ptp, struct cpts, info);

	switch (rq->type) {
	case PTP_CLK_REQ_PPS:
		return cpts_pps_enable(cpts, on);
	default:
		break;
	}
	return -EOPNOTSUPP;
}

static struct ptp_clock_info cpts_info = {
	.owner		= THIS_MODULE,
	.name		= "CTPS timer",
	.max_adj	= 1000000,
	.n_ext_ts	= 0,
	.pps		= 1,
	.adjfreq	= cpts_ptp_adjfreq,
	.adjtime	= cpts_ptp_adjtime,
	.gettime	= cpts_ptp_gettime,
	.settime	= cpts_ptp_settime,
	.enable		= cpts_ptp_enable,
};

static void cpts_overflow_check(struct work_struct *work)
{
	struct timespec ts;
	struct cpts *cpts = container_of(work, struct cpts, overflow_work.work);

	cpts_write32(cpts, CPTS_EN, control);
	cpts_write32(cpts, TS_PEND_EN, int_enable);
	cpts_ptp_gettime(&cpts->info, &ts);
	pr_debug("cpts overflow check at %ld.%09lu\n", ts.tv_sec, ts.tv_nsec);
	if (cpts->pps_enable)
		cpts_pps_reload(cpts);
	schedule_delayed_work(&cpts->overflow_work, CPTS_OVERFLOW_PERIOD);
}

#define CPTS_REF_CLOCK_NAME "cpsw_cpts_rft_clk"

static void cpts_clk_init(struct cpts *cpts)
{
	unsigned long rate;
	u64 max_sec;

	cpts->refclk = clk_get(cpts->dev, CPTS_REF_CLOCK_NAME);
	if (IS_ERR(cpts->refclk)) {
		pr_info("No %s defined.  Assumes external ref clock.\n",
			CPTS_REF_CLOCK_NAME);
		cpts->refclk = NULL;
		return;
	} else
		cpts->rftclk_freq = clk_get_rate(cpts->refclk);

	if (!cpts->cc.mult && !cpts->cc.shift) {
		/*
		   calculate the multiplier/shift to
		   convert CPTS counter ticks to ns.
		*/
		rate = cpts->rftclk_freq;
		max_sec = ((1ULL << CPTS_COUNTER_BITS) - 1) + (rate - 1);
		do_div(max_sec, rate);

		clocks_calc_mult_shift(&cpts->cc.mult, &cpts->cc.shift, rate,
					NSEC_PER_SEC, max_sec);

		pr_info("cpts rftclk rate(%lu HZ),mult(%u),shift(%u)\n",
				rate, cpts->cc.mult, cpts->cc.shift);
	}

	if (cpts->refclk)
		clk_prepare_enable(cpts->refclk);

	cpts_write32(cpts, cpts->rftclk_sel & 0x3, rfclk_sel);
}

static void cpts_clk_release(struct cpts *cpts)
{
	if (!cpts->refclk)
		return;

	clk_disable(cpts->refclk);
	clk_put(cpts->refclk);
}

static int cpts_match(struct sk_buff *skb, unsigned int ptp_class,
		      u16 ts_seqid, u8 ts_msgtype)
{
	u16 *seqid;
	unsigned int offset;
	u8 *msgtype, *data = skb->data;

	switch (ptp_class) {
	case PTP_CLASS_V1_IPV4:
	case PTP_CLASS_V2_IPV4:
		offset = ETH_HLEN + IPV4_HLEN(data) + UDP_HLEN;
		break;
	case PTP_CLASS_V1_IPV6:
	case PTP_CLASS_V2_IPV6:
		offset = OFF_PTP6;
		break;
	case PTP_CLASS_V2_L2:
		offset = ETH_HLEN;
		break;
	case PTP_CLASS_V2_VLAN:
		offset = ETH_HLEN + VLAN_HLEN;
		break;
	case PTP_CLASS_V1_VLAN_IPV4:
	case PTP_CLASS_V2_VLAN_IPV4:
		offset = ETH_HLEN + VLAN_HLEN + IPV4_HLEN(data) + UDP_HLEN;
		break;
	case PTP_CLASS_V1_VLAN_IPV6:
	case PTP_CLASS_V2_VLAN_IPV6:
		offset = OFF_PTP6 + VLAN_HLEN;
		break;
	default:
		return 0;
	}

	if (skb->len + ETH_HLEN < offset + OFF_PTP_SEQUENCE_ID + sizeof(*seqid))
		return 0;

	if (unlikely(ptp_class & PTP_CLASS_V1))
		msgtype = data + offset + OFF_PTP_CONTROL;
	else
		msgtype = data + offset;

	seqid = (u16 *)(data + offset + OFF_PTP_SEQUENCE_ID);

	return (ts_msgtype == (*msgtype & 0xf) && ts_seqid == ntohs(*seqid));
}

static u64 cpts_find_ts(struct cpts *cpts, struct sk_buff *skb, int ev_type)
{
	u64 ns = 0;
	struct cpts_event *event;
	struct list_head *this, *next;
	unsigned int class = sk_run_filter(skb, cpts->filter);
	unsigned long flags;
	u16 seqid;
	u8 mtype;

	if (class == PTP_CLASS_NONE)
		return 0;

	spin_lock_irqsave(&cpts->lock, flags);
	cpts_fifo_read(cpts, CPTS_EV_PUSH);
	list_for_each_safe(this, next, &cpts->events) {
		event = list_entry(this, struct cpts_event, list);
		if (event_expired(event)) {
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			continue;
		}
		mtype = (event->high >> MESSAGE_TYPE_SHIFT) & MESSAGE_TYPE_MASK;
		seqid = (event->high >> SEQUENCE_ID_SHIFT) & SEQUENCE_ID_MASK;
		if (ev_type == event_type(event) &&
		    cpts_match(skb, class, seqid, mtype)) {
			ns = timecounter_cyc2time(&cpts->tc, event->low);
			list_del_init(&event->list);
			list_add(&event->list, &cpts->pool);
			break;
		}
	}
	spin_unlock_irqrestore(&cpts->lock, flags);

	return ns;
}

int cpts_rx_timestamp(struct cpts *cpts, struct sk_buff *skb)
{
	u64 ns;
	s64 temp;
	struct skb_shared_hwtstamps *ssh;

	if (!cpts->rx_enable)
		return -EPERM;
	ns = cpts_find_ts(cpts, skb, CPTS_EV_RX);
	if (!ns)
		return -ENOENT;
	ssh = skb_hwtstamps(skb);
	memset(ssh, 0, sizeof(*ssh));
	ssh->hwtstamp = ns_to_ktime(ns);
	temp = ktime_to_ns(ktime_get_monotonic_offset());
	ns = (u64)((s64)ns - ktime_to_ns(ktime_get_monotonic_offset()));
	ssh->syststamp = ns_to_ktime(ns);
	return 0;
}

int cpts_tx_timestamp(struct cpts *cpts, struct sk_buff *skb)
{
	u64 ns;
	s64 temp;
	struct skb_shared_hwtstamps ssh;

	if (!(skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS))
		return -EPERM;
	ns = cpts_find_ts(cpts, skb, CPTS_EV_TX);
	if (!ns)
		return -ENOENT;
	memset(&ssh, 0, sizeof(ssh));
	ssh.hwtstamp = ns_to_ktime(ns);

	temp = ktime_to_ns(ktime_get_monotonic_offset());
	ns = (u64)((s64)ns - ktime_to_ns(ktime_get_monotonic_offset()));
	ssh.syststamp = ns_to_ktime(ns);
	skb_tstamp_tx(skb, &ssh);
	return 0;
}

#endif /*CONFIG_TI_CPTS*/

/*
    If both mult and shift are passed in as 0, they will be
    calculated based on the cpts rfclk frequency
*/
int cpts_register(struct device *dev, struct cpts *cpts,
		  u32 mult, u32 shift)
{
#ifdef CONFIG_TI_CPTS
	int err, i;
	unsigned long flags;

	if (cpts->filter == NULL) {
		cpts->filter = ptp_default_filter;
		cpts->filter_size = ARRAY_SIZE(ptp_default_filter);
	}

	if (ptp_filter_init(cpts->filter, cpts->filter_size)) {
		pr_err("cpts: bad ptp filter\n");
		return -EINVAL;
	}
	cpts->info = cpts_info;
	cpts->clock = ptp_clock_register(&cpts->info, dev);
	if (IS_ERR(cpts->clock)) {
		err = PTR_ERR(cpts->clock);
		cpts->clock = NULL;
		return err;
	}
	spin_lock_init(&cpts->lock);

	cpts->dev = dev;
	cpts->cc.read = cpts_systim_read;
	cpts->cc.mask = CLOCKSOURCE_MASK(32);
	cpts->cc_mult = mult;
	cpts->cc.mult = mult;
	cpts->cc.shift = shift;

	INIT_LIST_HEAD(&cpts->events);
	INIT_LIST_HEAD(&cpts->pool);
	for (i = 0; i < CPTS_MAX_EVENTS; i++)
		list_add(&cpts->pool_data[i].list, &cpts->pool);

	cpts_clk_init(cpts);
	/* mult may be updated during clk init */
	cpts->cc_mult = cpts->cc.mult;
	cpts_write32(cpts, CPTS_EN, control);
	cpts_write32(cpts, TS_PEND_EN, int_enable);

	if (cpts->info.pps)
		cpts_pps_init(cpts);

	spin_lock_irqsave(&cpts->lock, flags);
	timecounter_init(&cpts->tc, &cpts->cc, ktime_to_ns(ktime_get_real()));
	spin_unlock_irqrestore(&cpts->lock, flags);

	INIT_DELAYED_WORK(&cpts->overflow_work, cpts_overflow_check);
	schedule_delayed_work(&cpts->overflow_work, CPTS_OVERFLOW_PERIOD);

	cpts->phc_index = ptp_clock_index(cpts->clock);
#endif
	return 0;
}

void cpts_unregister(struct cpts *cpts)
{
#ifdef CONFIG_TI_CPTS
	if (cpts->clock) {
		ptp_clock_unregister(cpts->clock);
		cancel_delayed_work_sync(&cpts->overflow_work);
	}
	if (cpts->refclk)
		cpts_clk_release(cpts);
#endif
}
