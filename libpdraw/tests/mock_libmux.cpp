/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Mock libmux implementation
 *
 * Copyright (c) 2026 Parrot Drones SAS
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

#define ULOG_TAG pdraw_mock_libmux
#include "mock_libmux.hpp"
#include <errno.h>
#include <memory>
#include <ulog.h>

ULOG_DECLARE_TAG(ULOG_TAG);

struct libmux_mock g_libmux_mock;

void libmux_mock_reset()
{
	g_libmux_mock.ctx = nullptr;
	g_libmux_mock.auto_trigger_proxy_open = true;
	g_libmux_mock.proxy_new_result = 0;
}

struct mux_ctx *libmux_mock_new()
{
	auto ctx = std::make_unique<struct mux_ctx>();
	g_libmux_mock.ctx = ctx.get();
	return ctx.release();
}

void libmux_mock_feed_channel(uint32_t chanid, struct pomp_buffer *buf)
{
	if (g_libmux_mock.ctx) {
		auto it = g_libmux_mock.ctx->channels.find(chanid);
		if (it != g_libmux_mock.ctx->channels.end() && it->second.cb) {
			it->second.cb(g_libmux_mock.ctx,
				      chanid,
				      MUX_CHANNEL_DATA,
				      buf,
				      it->second.userdata);
		}
	}
}

/* Implementations of libmux C API */

extern "C" {

void mux_ref(struct mux_ctx *ctx)
{
	if (ctx)
		ctx->refcount++;
}

void mux_unref(struct mux_ctx *ctx)
{
	if (ctx) {
		ctx->refcount--;
		if (ctx->refcount == 0) {
			if (g_libmux_mock.ctx == ctx) {
				g_libmux_mock.ctx = nullptr;
			}
			std::unique_ptr<struct mux_ctx> owner(ctx);
		}
	}
}

int mux_channel_open(struct mux_ctx *ctx,
		     uint32_t chanid,
		     mux_channel_cb_t cb,
		     void *userdata)
{
	if (!ctx)
		return -EINVAL;

	ctx->channels[chanid] = {cb, userdata};
	return 0;
}

int mux_channel_close(struct mux_ctx *ctx, uint32_t chanid)
{
	if (!ctx)
		return -EINVAL;

	ctx->channels.erase(chanid);
	return 0;
}

int mux_ip_proxy_new(struct mux_ctx *ctx,
		     struct mux_ip_proxy_info *info,
		     struct mux_ip_proxy_cbs *cbs,
		     int timeout,
		     struct mux_ip_proxy **ret_obj)
{
	if (g_libmux_mock.proxy_new_result != 0)
		return g_libmux_mock.proxy_new_result;

	if (!ctx || !info || !ret_obj)
		return -EINVAL;

	auto proxy = std::make_unique<struct mux_ip_proxy>();
	proxy->mux = ctx;
	if (cbs)
		proxy->cbs = *cbs;

	proxy->remote_host = info->remote_host ? info->remote_host : "";
	proxy->remote_port = info->remote_port;
	proxy->redirect_port = info->udp_redirect_port;

	*ret_obj = proxy.get();

	if (g_libmux_mock.auto_trigger_proxy_open && cbs && cbs->open)
		cbs->open(proxy.get(), info->udp_redirect_port, cbs->userdata);

	proxy.release();
	return 0;
}

int mux_ip_proxy_destroy(struct mux_ip_proxy *ip_proxy)
{
	if (ip_proxy) {
		if (ip_proxy->cbs.close)
			ip_proxy->cbs.close(ip_proxy, ip_proxy->cbs.userdata);

		std::unique_ptr<struct mux_ip_proxy> owner(ip_proxy);
	}
	return 0;
}

uint16_t mux_ip_proxy_get_peerport(struct mux_ip_proxy *self)
{
	return self ? self->peer_port : 0;
}

uint16_t mux_ip_proxy_get_remote_port(struct mux_ip_proxy *self)
{
	return self ? self->remote_port : 0;
}

int mux_ip_proxy_set_udp_remote(struct mux_ip_proxy *self,
				const char *remote_host,
				uint16_t remote_port,
				int timeout)
{
	if (!self)
		return -EINVAL;

	self->remote_host = remote_host ? remote_host : "";
	self->remote_port = remote_port;
	if (self->cbs.remote_update)
		self->cbs.remote_update(self, self->cbs.userdata);

	return 0;
}

} /* extern "C" */
