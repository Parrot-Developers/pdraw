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

#pragma once

/**
 * This stub replaces libmux for the pdraw-muxmuxer-mux-test executable.
 *
 * The stub intercepts the nine libmux symbols used by RtspStreamMuxerMux and
 * replaces them with local-loopback implementations:
 *
 *   mux_new / mux_ref / mux_unref / mux_get_loop
 *   mux_ip_proxy_new / mux_ip_proxy_destroy
 *   mux_ip_proxy_get_peerport / mux_ip_proxy_set_udp_remote
 *   mux_ip_proxy_get_remote_port
 *
 * Each mux_ip_proxy allocates a real UDP socket on 127.0.0.1 (peerport).
 * It sends a 1-byte "wake-up" datagram to udp_redirect_port so that the
 * corresponding loopback tskt_socket learns peerport as its remote address
 * (required by tskt_socket_write_pkt which replies to the last recvfrom
 * sender).  After mux_ip_proxy_set_udp_remote() is called the proxy
 * also forwards every subsequent datagram it receives from redirect_port
 * to 127.0.0.1:remote_port (the RTSP server's media/control ports).
 *
 * Packet forwarding requires the proxy socket to be monitored by a pomp
 * loop.  The stub registers it on the loop that was passed to mux_new().
 * When mux_new() is called with loop == NULL the stub creates its own
 * pomp loop (and a dedicated thread to run it).
 *
 * The open callback is invoked synchronously from within mux_ip_proxy_new();
 * this is valid because mux_ip_proxy_new() is always called from the pdraw
 * pomp loop thread (via RtspStreamMuxerMux::VideoMediaMux::prepareSetup()),
 * so proxyOpenCb() just calls pomp::Loop::idleAdd() which is safe.
 */

#include <libpomp.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Minimal libmux type definitions - avoids depending on <libmux.h> which is
 * absent on Windows where libmux is an optional dependency. */

struct mux_ctx;
struct mux_ip_proxy;
struct mux_queue;
struct mux_ops;

enum mux_ip_proxy_transport {
	MUX_IP_PROXY_TRANSPORT_TCP,
	MUX_IP_PROXY_TRANSPORT_UDP,
};

enum mux_ip_proxy_application {
	MUX_IP_PROXY_APPLICATION_NONE,
	MUX_IP_PROXY_APPLICATION_FTP,
};

struct mux_ip_proxy_protocol {
	enum mux_ip_proxy_transport transport;
	enum mux_ip_proxy_application application;
};

struct mux_ip_proxy_info {
	struct mux_ip_proxy_protocol protocol;
	const char *remote_host;
	uint16_t remote_port;
	uint16_t udp_redirect_port;
};

struct mux_ip_proxy_cbs {
	void (*open)(struct mux_ip_proxy *self,
		     uint16_t localport,
		     void *userdata);
	void (*close)(struct mux_ip_proxy *self, void *userdata);
	void (*remote_update)(struct mux_ip_proxy *self, void *userdata);
	void (*resolution_failed)(struct mux_ip_proxy *self,
				  int err,
				  void *userdata);
	void *userdata;
};

enum mux_channel_event {
	MUX_CHANNEL_RESET,
	MUX_CHANNEL_DATA,
};

typedef void (*mux_channel_cb_t)(struct mux_ctx *ctx,
				 uint32_t chanid,
				 enum mux_channel_event event,
				 struct pomp_buffer *buf,
				 void *userdata);

/* Prototypes for the mux_* symbols provided by this stub */
struct mux_ctx *mux_new(int fd,
			struct pomp_loop *loop,
			const struct mux_ops *ops,
			uint32_t flags);
void mux_ref(struct mux_ctx *ctx);
void mux_unref(struct mux_ctx *ctx);
struct pomp_loop *mux_get_loop(struct mux_ctx *ctx);
uint32_t mux_get_remote_version(struct mux_ctx *ctx);
int mux_stop(struct mux_ctx *ctx);
int mux_run(struct mux_ctx *ctx);
int mux_reset(struct mux_ctx *ctx);
int mux_encode(struct mux_ctx *ctx, uint32_t chanid, struct pomp_buffer *buf);
int mux_decode(struct mux_ctx *ctx, struct pomp_buffer *buf);
int mux_resolve(struct mux_ctx *ctx, const char *hostname, uint32_t addr);
int mux_channel_open(struct mux_ctx *ctx,
		     uint32_t chanid,
		     mux_channel_cb_t cb,
		     void *userdata);
int mux_channel_close(struct mux_ctx *ctx, uint32_t chanid);
int mux_channel_alloc_queue(struct mux_ctx *ctx,
			    uint32_t chanid,
			    uint32_t depth,
			    struct mux_queue **queue);
int mux_ip_proxy_new(struct mux_ctx *ctx,
		     struct mux_ip_proxy_info *info,
		     struct mux_ip_proxy_cbs *cbs,
		     int timeout,
		     struct mux_ip_proxy **ret_obj);
int mux_ip_proxy_destroy(struct mux_ip_proxy *proxy);
int mux_ip_proxy_get_local_info(struct mux_ip_proxy *proxy,
				struct mux_ip_proxy_protocol *protocol,
				uint16_t *localport);
uint16_t mux_ip_proxy_get_peerport(struct mux_ip_proxy *proxy);
uint16_t mux_ip_proxy_get_remote_port(struct mux_ip_proxy *proxy);
const char *mux_ip_proxy_get_remote_host(struct mux_ip_proxy *proxy);
uint32_t mux_ip_proxy_get_remote_addr(struct mux_ip_proxy *proxy);
int mux_ip_proxy_set_udp_remote(struct mux_ip_proxy *proxy,
				const char *remote_host,
				uint16_t remote_port,
				int timeout);
int mux_ip_proxy_set_udp_redirect_port(struct mux_ip_proxy *proxy,
				       uint16_t redirect_port);
int mux_queue_get_buf(struct mux_queue *queue, struct pomp_buffer **buf);
int mux_queue_try_get_buf(struct mux_queue *queue, struct pomp_buffer **buf);
int mux_queue_timed_get_buf(struct mux_queue *queue,
			    struct pomp_buffer **buf,
			    struct timespec *timeout);

struct mux_ctx *stub_mux_new(void);

#ifdef __cplusplus
}
#endif
