/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — shared stub definitions for API tests
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

#define ULOG_TAG pdraw_test_api_common
#include "test_api_common.hpp"
#include "test_fixtures.hpp"

ULOG_DECLARE_TAG(ULOG_TAG);

namespace PdrawTest {


/* ── No-op stub callbacks for mandatory fields ──────────────────────────── */

static void
stub_flushed_coded_video_source(struct pdraw * /*p*/,
				struct pdraw_coded_video_source * /*s*/,
				void * /*u*/)
{
}


static void
stub_drained_coded_video_source(struct pdraw * /*p*/,
				struct pdraw_coded_video_source * /*s*/,
				void * /*u*/)
{
}


static void stub_flushed_raw_video_source(struct pdraw * /*p*/,
					  struct pdraw_raw_video_source * /*s*/,
					  void * /*u*/)
{
}


static void stub_drained_raw_video_source(struct pdraw * /*p*/,
					  struct pdraw_raw_video_source * /*s*/,
					  void * /*u*/)
{
}


static void stub_flush_coded_video_sink(struct pdraw * /*p*/,
					struct pdraw_coded_video_sink * /*sk*/,
					void * /*u*/)
{
}


static void stub_drain_coded_video_sink(struct pdraw * /*p*/,
					struct pdraw_coded_video_sink * /*sk*/,
					void * /*u*/)
{
}


static void stub_flush_raw_video_sink(struct pdraw * /*p*/,
				      struct pdraw_raw_video_sink * /*sk*/,
				      void * /*u*/)
{
}


static void stub_drain_raw_video_sink(struct pdraw * /*p*/,
				      struct pdraw_raw_video_sink * /*sk*/,
				      void * /*u*/)
{
}


static void stub_flushed_audio_source(struct pdraw * /*p*/,
				      struct pdraw_audio_source * /*s*/,
				      void * /*u*/)
{
}


static void stub_drained_audio_source(struct pdraw * /*p*/,
				      struct pdraw_audio_source * /*s*/,
				      void * /*u*/)
{
}


static void stub_flush_audio_sink(struct pdraw * /*p*/,
				  struct pdraw_audio_sink * /*sk*/,
				  void * /*u*/)
{
}


static void stub_drain_audio_sink(struct pdraw * /*p*/,
				  struct pdraw_audio_sink * /*sk*/,
				  void * /*u*/)
{
}


/* ── C stub definitions (all fields NULL) ───────────────────────────────── */

const struct pdraw_cbs g_stub_pdraw_cbs = {};
const struct pdraw_demuxer_cbs g_stub_demuxer_cbs = {};
const struct pdraw_muxer_cbs g_stub_muxer_cbs = {};
const struct pdraw_video_renderer_cbs g_stub_video_renderer_cbs = {};
const struct pdraw_audio_renderer_cbs g_stub_audio_renderer_cbs = {};
const struct pdraw_vipc_source_cbs g_stub_vipc_source_cbs = {};
const struct pdraw_coded_video_source_cbs g_stub_coded_video_source_cbs = {};
const struct pdraw_raw_video_source_cbs g_stub_raw_video_source_cbs = {};
const struct pdraw_coded_video_sink_cbs g_stub_coded_video_sink_cbs = {};
const struct pdraw_raw_video_sink_cbs g_stub_raw_video_sink_cbs = {};
const struct pdraw_alsa_source_cbs g_stub_alsa_source_cbs = {};
const struct pdraw_audio_source_cbs g_stub_audio_source_cbs = {};
const struct pdraw_audio_sink_cbs g_stub_audio_sink_cbs = {};
const struct pdraw_video_encoder_cbs g_stub_video_encoder_cbs = {};
const struct pdraw_video_scaler_cbs g_stub_video_scaler_cbs = {};
const struct pdraw_audio_encoder_cbs g_stub_audio_encoder_cbs = {};


/* ── C valid stubs — mandatory callbacks populated ──────────────────────── */

const struct pdraw_coded_video_source_cbs g_valid_coded_video_source_cbs = {
	.flushed = stub_flushed_coded_video_source,
	.drained = stub_drained_coded_video_source,
};

const struct pdraw_raw_video_source_cbs g_valid_raw_video_source_cbs = {
	.flushed = stub_flushed_raw_video_source,
	.drained = stub_drained_raw_video_source,
};

const struct pdraw_coded_video_sink_cbs g_valid_coded_video_sink_cbs = {
	.media_added = nullptr,
	.media_removed = nullptr,
	.flush = stub_flush_coded_video_sink,
	.drain = stub_drain_coded_video_sink,
	.session_metadata_update = nullptr,
};

const struct pdraw_raw_video_sink_cbs g_valid_raw_video_sink_cbs = {
	.media_added = nullptr,
	.media_removed = nullptr,
	.flush = stub_flush_raw_video_sink,
	.drain = stub_drain_raw_video_sink,
	.session_metadata_update = nullptr,
};

const struct pdraw_audio_source_cbs g_valid_audio_source_cbs = {
	.flushed = stub_flushed_audio_source,
	.drained = stub_drained_audio_source,
};

const struct pdraw_audio_sink_cbs g_valid_audio_sink_cbs = {
	.media_added = nullptr,
	.media_removed = nullptr,
	.flush = stub_flush_audio_sink,
	.drain = stub_drain_audio_sink,
};


/* ── Shared C++ listener instances ─────────────────────────────────────── */

StubPdrawListener g_stub_pdraw_listener;
StubDemuxerListener g_stub_demuxer_listener;
StubMuxerListener g_stub_muxer_listener;
StubVideoRendererListener g_stub_video_renderer_listener;
StubAudioRendererListener g_stub_audio_renderer_listener;
StubVipcSourceListener g_stub_vipc_source_listener;
StubCodedVideoSourceListener g_stub_coded_video_source_listener;
StubRawVideoSourceListener g_stub_raw_video_source_listener;
StubCodedVideoSinkListener g_stub_coded_video_sink_listener;
StubRawVideoSinkListener g_stub_raw_video_sink_listener;
StubAlsaSourceListener g_stub_alsa_source_listener;
StubAudioSourceListener g_stub_audio_source_listener;
StubAudioSinkListener g_stub_audio_sink_listener;
StubVideoEncoderListener g_stub_video_encoder_listener;
StubAudioEncoderListener g_stub_audio_encoder_listener;
StubVideoScalerListener g_stub_video_scaler_listener;


} /* namespace PdrawTest */
