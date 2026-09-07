/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Mock librtmp header
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

#include <rtmp.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

struct rtmp_client;

struct librtmp_mock_control {
	bool connect_should_fail;
	int connect_fail_errno;
	bool trigger_connection_success_async;
	bool trigger_disconnection_async;
	enum rtmp_client_disconnection_reason disconnection_reason;
	bool send_should_fail_eagain;
	int send_metadata_call_count;
	int send_video_avcc_call_count;
	int send_audio_specific_config_call_count;
	int send_video_frame_call_count;
	int send_audio_data_call_count;
	int flush_call_count;
	int socket_cb_call_count;
	size_t last_txbuf_size;
	void *last_userdata;
	struct rtmp_client *last_client;
};

extern struct librtmp_mock_control g_librtmp_mock;

void librtmp_mock_reset(void);

void librtmp_mock_trigger_disconnection(
	enum rtmp_client_disconnection_reason reason);

void librtmp_mock_trigger_peer_bw(uint32_t bandwidth);

#ifdef __cplusplus
}
#endif
