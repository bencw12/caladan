/*
 * tcp_out.c - the egress datapath for TCP
 */

#include <string.h>

#include <base/stddef.h>
#include <net/ip.h>
#include <net/tcp.h>
#include <net/chksum.h>

#include "tcp.h"
#include "defs.h"

static void tcp_tx_release_mbuf(struct mbuf *m)
{
	if (atomic_dec_and_test(&m->ref))
		net_tx_release_mbuf(m);
}

static struct tcp_hdr *
tcp_push_tcphdr(struct mbuf *m, tcpconn_t *c, uint8_t flags, uint16_t l4len)
{
	struct tcp_hdr *tcphdr;
	tcp_seq ack = c->tx_last_ack = load_acquire(&c->pcb.rcv_nxt);
	uint16_t win = c->tx_last_win = load_acquire(&c->pcb.rcv_wnd);

	/* write the tcp header */
	tcphdr = mbuf_push_hdr(m, *tcphdr);
	mbuf_mark_transport_offset(m);
	tcphdr->sport = hton16(c->e.laddr.port);
	tcphdr->dport = hton16(c->e.raddr.port);
	tcphdr->ack = hton32(ack);
	tcphdr->off = 5;
	tcphdr->flags = flags;
	tcphdr->win = hton16(win);
	tcphdr->seq = hton32(m->seg_seq);
	tcphdr->sum = ipv4_phdr_cksum(IPPROTO_TCP,
				      c->e.laddr.ip, c->e.raddr.ip,
				      sizeof(struct tcp_hdr) + l4len);
	return tcphdr;
}

/**
 * tcp_tx_raw_rst - send a RST without an established connection
 * @laddr: the local address
 * @raddr: the remote address
 * @seq: the segement's sequence number
 *
 * Returns 0 if successful, otherwise fail.
 */
int tcp_tx_raw_rst(struct netaddr laddr, struct netaddr raddr, tcp_seq seq)
{
	struct tcp_hdr *tcphdr;
	struct mbuf *m;
	int ret;

	m = net_tx_alloc_mbuf();
	if (unlikely((!m)))
		return -ENOMEM;

	m->txflags = OLFLAG_TCP_CHKSUM;

	/* write the tcp header */
	tcphdr = mbuf_push_hdr(m, *tcphdr);
	tcphdr->sport = hton16(laddr.port);
	tcphdr->dport = hton16(raddr.port);
	tcphdr->seq = hton32(seq);
	tcphdr->ack = hton32(0);
	tcphdr->off = 5;
	tcphdr->flags = TCP_RST;
	tcphdr->win = hton16(0);
	tcphdr->sum = ipv4_phdr_cksum(IPPROTO_TCP, laddr.ip, raddr.ip,
				      sizeof(struct tcp_hdr));

	/* transmit packet */
	ret = net_tx_ip(m, IPPROTO_TCP, raddr.ip);
	if (unlikely(ret))
		mbuf_free(m);
	return ret;
}

/**
 * tcp_tx_raw_rst_ack - send a RST/ACK without an established connection
 * @laddr: the local address
 * @raddr: the remote address
 * @seq: the segment's sequence number
 * @ack: the segment's acknowledgement number
 *
 * Returns 0 if successful, otherwise fail.
 */
int tcp_tx_raw_rst_ack(struct netaddr laddr, struct netaddr raddr,
		       tcp_seq seq, tcp_seq ack)
{
	struct tcp_hdr *tcphdr;
	struct mbuf *m;
	int ret;

	m = net_tx_alloc_mbuf();
	if (unlikely((!m)))
		return -ENOMEM;

	m->txflags = OLFLAG_TCP_CHKSUM;

	/* write the tcp header */
	tcphdr = mbuf_push_hdr(m, *tcphdr);
	tcphdr->sport = hton16(laddr.port);
	tcphdr->dport = hton16(raddr.port);
	tcphdr->seq = hton32(seq);
	tcphdr->ack = hton32(ack);
	tcphdr->off = 5;
	tcphdr->flags = TCP_RST | TCP_ACK;
	tcphdr->win = hton16(0);
	tcphdr->sum = ipv4_phdr_cksum(IPPROTO_TCP, laddr.ip, raddr.ip,
				      sizeof(struct tcp_hdr));

	/* transmit packet */
	ret = net_tx_ip(m, IPPROTO_TCP, raddr.ip);
	if (unlikely(ret))
		mbuf_free(m);
	return ret;
}

/**
 * tcp_tx_ack - send an acknowledgement and window update packet
 * @c: the connection to send the ACK
 *
 * Returns 0 if succesful, otherwise fail.
 */
int tcp_tx_ack(tcpconn_t *c)
{
	struct mbuf *m;
	int ret;

	m = net_tx_alloc_mbuf();
	if (unlikely(!m))
		return -ENOMEM;

	m->txflags = OLFLAG_TCP_CHKSUM;
	m->seg_seq = load_acquire(&c->pcb.snd_nxt);
	tcp_push_tcphdr(m, c, TCP_ACK, 0);

	/* transmit packet */
	tcp_debug_egress_pkt(c, m);
	ret = net_tx_ip(m, IPPROTO_TCP, c->e.raddr.ip);
	if (unlikely(ret))
		mbuf_free(m);
	return ret;
}

/**
 * tcp_tx_ctl - sends a control message without data
 * @c: the TCP connection
 * @flags: the control flags (e.g. TCP_SYN, TCP_FIN, etc.)
 *
 * WARNING: The caller must have write exclusive access to the socket or hold
 * @c->lock while write exclusion isn't taken.
 *
 * Returns 0 if successful, -ENOMEM if out memory.
 */
int tcp_tx_ctl(tcpconn_t *c, uint8_t flags)
{
	struct mbuf *m;
	int ret;

	BUG_ON(!c->tx_exclusive && !spin_lock_held(&c->lock));

	m = net_tx_alloc_mbuf();
	if (unlikely(!m))
		return -ENOMEM;

	m->txflags = OLFLAG_TCP_CHKSUM;
	m->seg_seq = c->pcb.snd_nxt;
	m->seg_end = c->pcb.snd_nxt + 1;
	m->flags = flags;
	tcp_push_tcphdr(m, c, flags, 0);
	store_release(&c->pcb.snd_nxt, c->pcb.snd_nxt + 1);
	list_add_tail(&c->txq, &m->link);
	m->timestamp = microtime();
	atomic_write(&m->ref, 2);
	m->release = tcp_tx_release_mbuf;
	tcp_debug_egress_pkt(c, m);
	ret = net_tx_ip(m, IPPROTO_TCP, c->e.raddr.ip);
	if (unlikely(ret)) {
		/* pretend the packet was sent */
		mbuf_push(m, sizeof(struct eth_hdr) + sizeof(struct ip_hdr));
		atomic_write(&m->ref, 1);
	}
	return ret;
}

/**
 * tcp_tx_send - transmit a buffer on a TCP connection
 * @c: the TCP connection
 * @buf: the buffer to transmit
 * @len: the length of the buffer to transmit
 * @push: indicates the data is ready for consumption by the receiver
 *
 * If @push is false, the implementation may buffer some or all of the data for
 * future transmission.
 *
 * WARNING: The caller is responsible for respecting the TCP window size limit.
 * WARNING: The caller must have write exclusive access to the socket or hold
 * @c->lock while write exclusion isn't taken.
 *
 * Returns the number of bytes transmitted, or < 0 if there was an error.
 */
ssize_t tcp_tx_send(tcpconn_t *c, const void *buf, size_t len, bool push)
{
	struct mbuf *m;
	const char *pos = buf;
	const char *end = pos + len;
	ssize_t ret = 0;
	size_t seglen;

	assert(c->pcb.state >= TCP_STATE_ESTABLISHED);
	assert((c->tx_exclusive == true) || spin_lock_held(&c->lock));

	pos = buf;
	end = pos + len;

	/* the main TCP segmenter loop */
	while (pos < end) {
		/* allocate a buffer and copy payload data */
		if (c->tx_pending) {
			m = c->tx_pending;
			c->tx_pending = NULL;
			seglen = min(end - pos, TCP_MSS - mbuf_length(m));
			m->seg_end += seglen;
		} else {
			m = net_tx_alloc_mbuf();
			if (unlikely(!m)) {
				ret = -ENOBUFS;
				break;
			}
			seglen = min(end - pos, TCP_MSS);
			m->seg_seq = c->pcb.snd_nxt;
			m->seg_end = c->pcb.snd_nxt + seglen;
			m->flags = TCP_ACK;
			atomic_write(&m->ref, 2);
			m->release = tcp_tx_release_mbuf;
		}

		memcpy(mbuf_put(m, seglen), pos, seglen);
		store_release(&c->pcb.snd_nxt, c->pcb.snd_nxt + seglen);
		pos += seglen;

		/* if not pushing, keep the last buffer for later */
		if (!push && pos == end && mbuf_length(m) -
		    sizeof(struct tcp_hdr) < TCP_MSS) {
			c->tx_pending = m;
			break;
		}

		/* initialize TCP header */
		if (push && pos == end)
			m->flags |= TCP_PUSH;
		tcp_push_tcphdr(m, c, m->flags, m->seg_end - m->seg_seq);

		/* transmit the packet */
		list_add_tail(&c->txq, &m->link);
		tcp_debug_egress_pkt(c, m);
		m->timestamp = microtime();
		m->txflags = OLFLAG_TCP_CHKSUM;
		ret = net_tx_ip(m, IPPROTO_TCP, c->e.raddr.ip);
		if (unlikely(ret)) {
			/* pretend the packet was sent */
			mbuf_push(m, sizeof(struct eth_hdr) +
				     sizeof(struct ip_hdr));
			atomic_write(&m->ref, 1);
		}
	}

	/* if we sent anything return the length we sent instead of an error */
	if (pos - (const char *)buf > 0)
		ret = pos - (const char *)buf;
	return ret;
}

/**
 * tcp_tx_retransmit - resend any pending egress packets that timed out
 * @c: the TCP connection in which to send retransmissions
 */
void tcp_tx_retransmit(tcpconn_t *c)
{
	struct mbuf *m;
	uint64_t now = microtime();
	uint16_t l4len;
	int ret;

	assert_spin_lock_held(&c->lock);

	list_for_each(&c->txq, m, link) {
		/* check if still transmitting (probably a dishonest ACK) */
		if (unlikely(atomic_read(&m->ref) != 1))
			break;
		/* check if the timeout expired */
		if (now - m->timestamp < TCP_RETRANSMIT_TIMEOUT)
			break;

		/* strip headers and reset ref count */
		mbuf_pull(m, sizeof(struct eth_hdr) + sizeof(struct ip_hdr) +
			     sizeof(struct tcp_hdr));
		atomic_write(&m->ref, 2);

		/* handle a partially acknowledged packet */
		assert(wraps_gt(m->seg_end, c->pcb.snd_una));
		if (wraps_lt(m->seg_seq, c->pcb.snd_una)) {
			mbuf_pull(m, c->pcb.snd_una - m->seg_seq);
			m->seg_seq = c->pcb.snd_una;
		}

		/* push the TCP header back on (now with fresher ack) */
		l4len = m->seg_end - m->seg_seq;
		if (m->flags & (TCP_SYN | TCP_FIN))
			l4len--;
		tcp_push_tcphdr(m, c, m->flags, l4len);

		/* transmit the packet */
		tcp_debug_egress_pkt(c, m);
		m->timestamp = now;
		ret = net_tx_ip(m, IPPROTO_TCP, c->e.raddr.ip);
		if (unlikely(ret)) {
			/* pretend the packet was sent */
			mbuf_push(m, sizeof(struct eth_hdr) +
				     sizeof(struct ip_hdr));
			atomic_write(&m->ref, 1);
		}
	}
}