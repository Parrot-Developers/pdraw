/**
 * Parrot Drones Audio and Video Vector
 * libmux stub for RTSP-over-mux test
 *
 * Copyright (c) 2018 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define ULOG_TAG stub_mux
#include "stub_mux.h"

#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ulog.h>
#include <unistd.h>

#ifdef _WIN32
#	include <winsock2.h>
#	include <ws2tcpip.h>
#	define close(fd) closesocket(fd)
#else /* !_WIN32 */
#	include <arpa/inet.h>
#	include <netinet/in.h>
#	include <sys/socket.h>
#endif /* !_WIN32 */

#define UNUSED(x) (void)(x)

ULOG_DECLARE_TAG(ULOG_TAG);

/* =========================================================================
 * Internal structures
 * ========================================================================= */

struct mux_ctx {
	atomic_int refcount;
	struct pomp_loop *loop;
	pthread_t loop_thread; /* valid only when we own the loop */
	bool owns_loop;
};


struct mux_ip_proxy {
	int fd;
	uint16_t peerport;
	uint16_t redirect_port; /* loopback socket's local port */
	uint16_t remote_port; /* RTSP server media/ctrl port */
	bool has_remote;
	struct mux_ip_proxy_cbs cbs;
	struct mux_ctx *ctx;
};


/* =========================================================================
 * Helpers
 * ========================================================================= */

static uint16_t bind_ephemeral_udp(int *out_fd)
{
#ifdef _WIN32
	SOCKET wfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (wfd == INVALID_SOCKET) {
		ULOGE("socket failed (WSAError=%d)", (int)WSAGetLastError());
		return 0;
	}
	int fd = (int)wfd;
#else
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0) {
		ULOGE("socket: %s", strerror(errno));
		return 0;
	}
#endif

	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_addr.s_addr = htonl(INADDR_LOOPBACK),
		.sin_port = 0,
	};
	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		ULOGE("bind: %s", strerror(errno));
		close(fd);
		return 0;
	}

	socklen_t len = sizeof(addr);
	if (getsockname(fd, (struct sockaddr *)&addr, &len) < 0) {
		ULOGE("getsockname: %s", strerror(errno));
		close(fd);
		return 0;
	}

	*out_fd = fd;
	return ntohs(addr.sin_port);
}


static void send_to_port(int fd, uint16_t port)
{
	struct sockaddr_in dst = {
		.sin_family = AF_INET,
		.sin_addr.s_addr = htonl(INADDR_LOOPBACK),
		.sin_port = htons(port),
	};
	/* 1-byte wakeup - processDataPkt / processCtrlPkt both handle
	 * spurious short packets gracefully (processDataPkt is a no-op;
	 * processCtrlPkt now has an mSender==NULL guard). */
	char byte = 0;
	sendto(fd, &byte, 1, 0, (struct sockaddr *)&dst, sizeof(dst));
}


/* =========================================================================
 * Proxy fd callback - called from the pomp loop when data arrives
 * ========================================================================= */

static void proxy_fd_cb(int fd, uint32_t events, void *userdata)
{
	const struct mux_ip_proxy *proxy = userdata;
	char buf[2048];
	struct sockaddr_in sender = {};
	socklen_t sender_len = sizeof(sender);

	if (!(events & POMP_FD_EVENT_IN))
		return;

	ssize_t n = recvfrom(fd,
			     buf,
			     sizeof(buf),
			     0,
			     (struct sockaddr *)&sender,
			     &sender_len);
	if (n < 0)
		return;

	uint16_t sender_port = ntohs(sender.sin_port);

	if (!proxy->has_remote)
		return;

	if (sender_port == proxy->redirect_port) {
		/* Packet from our loopback socket -> forward to server */
		struct sockaddr_in dst = {
			.sin_family = AF_INET,
			.sin_addr.s_addr = htonl(INADDR_LOOPBACK),
			.sin_port = htons(proxy->remote_port),
		};
		sendto(fd,
		       buf,
#ifdef _WIN32
		       (int)n,
#else
		       (size_t)n,
#endif
		       0,
		       (struct sockaddr *)&dst,
		       sizeof(dst));
	} else {
		/* Packet from server (RTCP RR etc.) -> redirect to our socket
		 */
		send_to_port(fd, proxy->redirect_port);
	}
}


/* =========================================================================
 * Loop thread (used only when mux_new() is called with loop == NULL)
 * ========================================================================= */

static void *loop_thread_func(void *arg)
{
	struct pomp_loop *loop = arg;
	int res;
	/* pomp has no pomp_loop_run(); drive the loop manually */
	do {
		res = pomp_loop_wait_and_process(loop, -1);
	} while (res == 0 || res == -EINTR);
	return NULL;
}


/* =========================================================================
 * Public stub API - replaces libmux symbols
 * ========================================================================= */

struct mux_ctx *stub_mux_new(void)
{
	struct mux_ctx *ctx = calloc(1, sizeof(*ctx));
	if (!ctx)
		return NULL;
	atomic_store(&ctx->refcount, 1);
	ctx->owns_loop = false;
	return ctx;
}


/* Intercepts mux_new() so pdraw can call mux_new() if it wants;
 * for this test the caller uses stub_mux_new() directly. */
struct mux_ctx *mux_new(int fd,
			struct pomp_loop *loop,
			const struct mux_ops *ops,
			uint32_t flags)
{
	struct mux_ctx *ctx = stub_mux_new();
	if (!ctx)
		return NULL;

	if (loop) {
		ctx->loop = loop;
		ctx->owns_loop = false;
	} else {
		ctx->loop = pomp_loop_new();
		if (!ctx->loop) {
			free(ctx);
			return NULL;
		}
		ctx->owns_loop = true;
		pthread_create(
			&ctx->loop_thread, NULL, loop_thread_func, ctx->loop);
	}
	return ctx;
}


void mux_ref(struct mux_ctx *ctx)
{
	if (ctx)
		atomic_fetch_add(&ctx->refcount, 1);
}


void mux_unref(struct mux_ctx *ctx)
{
	if (!ctx)
		return;
	if (atomic_fetch_sub(&ctx->refcount, 1) != 1)
		return;

	if (ctx->owns_loop && ctx->loop) {
		pomp_loop_wakeup(ctx->loop);
		pthread_join(ctx->loop_thread, NULL);
		pomp_loop_destroy(ctx->loop);
	}
	free(ctx);
}


struct pomp_loop *mux_get_loop(struct mux_ctx *ctx)
{
	return ctx ? ctx->loop : NULL;
}


int mux_ip_proxy_new(struct mux_ctx *ctx,
		     struct mux_ip_proxy_info *info,
		     struct mux_ip_proxy_cbs *cbs,
		     int timeout,
		     struct mux_ip_proxy **ret_obj)
{
	if (!ctx || !info || !cbs || !ret_obj)
		return -EINVAL;

	struct mux_ip_proxy *proxy = calloc(1, sizeof(*proxy));
	if (!proxy)
		return -ENOMEM;

	proxy->cbs = *cbs;
	proxy->ctx = ctx;
	proxy->redirect_port = info->udp_redirect_port;

	UNUSED(timeout);

	proxy->peerport = bind_ephemeral_udp(&proxy->fd);
	if (!proxy->peerport) {
		free(proxy);
		return -EIO;
	}

	/* Register fd on the pomp loop for packet forwarding */
	if (ctx->loop) {
		int res = pomp_loop_add(ctx->loop,
					proxy->fd,
					POMP_FD_EVENT_IN,
					proxy_fd_cb,
					proxy);
		if (res < 0)
			ULOGW("pomp_loop_add: %s", strerror(-res));
	}

	/* Prime the loopback socket: send a 1-byte datagram from peerport to
	 * redirect_port so that tskt_socket_write_pkt() knows its destination
	 * (it replies to the last recvfrom sender). */
	if (proxy->redirect_port)
		send_to_port(proxy->fd, proxy->redirect_port);

	ULOGI("proxy created peerport=%u redirect_port=%u",
	      proxy->peerport,
	      proxy->redirect_port);

	*ret_obj = proxy;

	/* Call open synchronously - safe because mux_ip_proxy_new() is always
	 * called from the pdraw pomp loop thread (via prepareSetup()), and
	 * proxyOpenCb() only calls pomp::Loop::idleAdd(). */
	if (cbs->open)
		cbs->open(proxy, proxy->peerport, cbs->userdata);

	return 0;
}


int mux_ip_proxy_destroy(struct mux_ip_proxy *proxy)
{
	if (!proxy)
		return 0;

	if (proxy->ctx && proxy->ctx->loop && proxy->fd >= 0)
		pomp_loop_remove(proxy->ctx->loop, proxy->fd);

	if (proxy->fd >= 0)
		close(proxy->fd);

	free(proxy);
	return 0;
}


uint16_t mux_ip_proxy_get_peerport(struct mux_ip_proxy *proxy)
{
	return proxy ? proxy->peerport : 0;
}


uint16_t mux_ip_proxy_get_remote_port(struct mux_ip_proxy *proxy)
{
	return proxy ? proxy->remote_port : 0;
}


/* "skycontroller" is mapped to 127.0.0.1 */
int mux_ip_proxy_set_udp_remote(struct mux_ip_proxy *proxy,
				const char *remote_host,
				uint16_t remote_port,
				int timeout)
{
	if (!proxy)
		return -EINVAL;

	UNUSED(remote_host);
	UNUSED(timeout);

	proxy->remote_port = remote_port;
	proxy->has_remote = true;

	ULOGI("proxy peerport=%u -> remote 127.0.0.1:%u",
	      proxy->peerport,
	      remote_port);

	if (proxy->cbs.remote_update)
		proxy->cbs.remote_update(proxy, proxy->cbs.userdata);

	return 0;
}


/* Remaining stubs (unused by RtspStreamMuxerMux but may be needed by
 * the linker if the real libmux is also in the link command). */

int mux_stop(struct mux_ctx *ctx)
{
	UNUSED(ctx);
	return 0;
}
int mux_run(struct mux_ctx *ctx)
{
	UNUSED(ctx);
	return 0;
}
int mux_reset(struct mux_ctx *ctx)
{
	UNUSED(ctx);
	return 0;
}
int mux_encode(struct mux_ctx *ctx, uint32_t chanid, struct pomp_buffer *buf)
{
	UNUSED(ctx);
	UNUSED(chanid);
	UNUSED(buf);
	return 0;
}
int mux_decode(struct mux_ctx *ctx, struct pomp_buffer *buf)
{
	UNUSED(ctx);
	UNUSED(buf);
	return 0;
}
int mux_resolve(struct mux_ctx *ctx, const char *hostname, uint32_t addr)
{
	UNUSED(ctx);
	UNUSED(hostname);
	UNUSED(addr);
	return 0;
}
int mux_channel_open(struct mux_ctx *ctx,
		     uint32_t chanid,
		     mux_channel_cb_t cb,
		     void *userdata)
{
	UNUSED(ctx);
	UNUSED(chanid);
	UNUSED(cb);
	UNUSED(userdata);
	return 0;
}
int mux_channel_close(struct mux_ctx *ctx, uint32_t chanid)
{
	UNUSED(ctx);
	UNUSED(chanid);
	return 0;
}
int mux_channel_alloc_queue(struct mux_ctx *ctx,
			    uint32_t chanid,
			    uint32_t depth,
			    struct mux_queue **queue)
{
	UNUSED(ctx);
	UNUSED(chanid);
	UNUSED(depth);
	UNUSED(queue);
	return 0;
}
uint32_t mux_get_remote_version(struct mux_ctx *ctx)
{
	UNUSED(ctx);
	return 0;
}
int mux_ip_proxy_get_local_info(struct mux_ip_proxy *proxy,
				struct mux_ip_proxy_protocol *protocol,
				uint16_t *localport)
{
	UNUSED(proxy);
	UNUSED(protocol);
	UNUSED(localport);
	return 0;
}
const char *mux_ip_proxy_get_remote_host(struct mux_ip_proxy *proxy)
{
	return "skycontroller";
}
uint32_t mux_ip_proxy_get_remote_addr(struct mux_ip_proxy *proxy)
{
	return htonl(INADDR_LOOPBACK);
}
int mux_ip_proxy_set_udp_redirect_port(struct mux_ip_proxy *proxy,
				       uint16_t redirect_port)
{
	UNUSED(proxy);
	UNUSED(redirect_port);
	return 0;
}
int mux_queue_get_buf(struct mux_queue *queue, struct pomp_buffer **buf)
{
	UNUSED(queue);
	UNUSED(buf);
	return -ENOSYS;
}
int mux_queue_try_get_buf(struct mux_queue *queue, struct pomp_buffer **buf)
{
	UNUSED(queue);
	UNUSED(buf);
	return -ENOSYS;
}
int mux_queue_timed_get_buf(struct mux_queue *queue,
			    struct pomp_buffer **buf,
			    struct timespec *timeout)
{
	UNUSED(queue);
	UNUSED(buf);
	UNUSED(timeout);
	return -ENOSYS;
}
