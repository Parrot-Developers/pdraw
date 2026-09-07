/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Mock libmux header
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

#pragma once

#include <libmux-arsdk.h>
#include <libmux.h>
#include <map>
#include <string>

#ifdef __cplusplus
extern "C" {
#endif

struct ChannelInfo {
	mux_channel_cb_t cb = nullptr;
	void *userdata = nullptr;
};

struct mux_ctx {
	int refcount = 1;
	std::map<uint32_t, ChannelInfo> channels;
};

struct mux_ip_proxy {
	struct mux_ctx *mux = nullptr;
	struct mux_ip_proxy_cbs cbs = {};
	std::string remote_host;
	uint16_t remote_port = 0;
	uint16_t peer_port = 0;
	uint16_t redirect_port = 0;
};

struct libmux_mock {
	struct mux_ctx *ctx = nullptr;
	bool auto_trigger_proxy_open = true;
	int proxy_new_result = 0;
};

extern struct libmux_mock g_libmux_mock;

void libmux_mock_reset();

struct mux_ctx *libmux_mock_new();

void libmux_mock_feed_channel(uint32_t chanid, struct pomp_buffer *buf);

#ifdef __cplusplus
}
#endif
