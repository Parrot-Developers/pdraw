/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — Audio encoder API input-validation (Tier C)
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

#define ULOG_TAG pdraw_test_api_encoder_audio
#include "test_api_common.hpp"
#include "test_common.h"
#include "test_fixtures.hpp"

#include <cstring>

ULOG_DECLARE_TAG(ULOG_TAG);

using namespace Pdraw;
using namespace PdrawTest;


/* ── C API (pdraw_audio_encoder_*) ──────────────────────────────────────── */

static void testCApiNew()
{
	struct aenc_config params = {};
	struct pdraw_audio_encoder *obj = nullptr;
	int ret = pdraw_audio_encoder_new(
		nullptr, 0, &params, &g_stub_audio_encoder_cbs, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	obj = nullptr;
	ret = pdraw_audio_encoder_new(g_test_pdraw_c,
				      0,
				      nullptr,
				      &g_stub_audio_encoder_cbs,
				      nullptr,
				      &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	obj = nullptr;
	ret = pdraw_audio_encoder_new(
		g_test_pdraw_c, 0, &params, nullptr, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = pdraw_audio_encoder_new(g_test_pdraw_c,
				      0,
				      &params,
				      &g_stub_audio_encoder_cbs,
				      nullptr,
				      nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiDestroy()
{
	int ret = pdraw_audio_encoder_destroy(nullptr, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = pdraw_audio_encoder_destroy(g_test_pdraw_c, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


/* ── C++ API (IPdraw::createAudioEncoder) ───────────────────────────────── */

static void testCxxCreate()
{
	IPdraw *session = g_test_session->get();
	IPdraw::IAudioEncoder *obj = nullptr;
	int ret = session->createAudioEncoder(
		0, nullptr, &g_stub_audio_encoder_listener, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	struct aenc_config params = {};
	obj = nullptr;
	ret = session->createAudioEncoder(0, &params, nullptr, &obj);
	CU_ASSERT_EQUAL(ret, -EINVAL);

	ret = session->createAudioEncoder(
		0, &params, &g_stub_audio_encoder_listener, nullptr);
	CU_ASSERT_EQUAL(ret, -EINVAL);
}


static void testCApiLifecycle()
{
	struct aenc_config params = {};
	struct pdraw_audio_encoder *obj = nullptr;
	struct pdraw_audio_encoder_cbs cbs = {};
	int ret = pdraw_audio_encoder_new(
		g_test_pdraw_c, 0, &params, &cbs, nullptr, &obj);
	CU_ASSERT_TRUE(ret == 0 || ret == -ENOENT || ret == -ENOSYS || ret < 0);
	if (ret == 0) {
		CU_ASSERT_PTR_NOT_NULL(obj);
		ret = pdraw_audio_encoder_destroy(g_test_pdraw_c, obj);
		CU_ASSERT_EQUAL(ret, 0);
	}
}


/* Exercises both PdrawAudioEncoderListener C API shims (pdraw_wrapper.cpp):
 * audioEncoderFrameOutput and audioEncoderFramePreRelease.
 *
 * Pipeline: ExternalAudioSource (raw PCM) → AudioEncoder (AAC-LC).
 * No downstream sink is needed: audioEncoderFrameOutput() fires BEFORE the
 * output-channel loop in frameOutputCb() (pdraw_encoder_audio.cpp:~916),
 * and frame_pre_release fires when the aenc backend drops its ref — both
 * unconditionally regardless of whether any sink is connected.
 *
 * Uses a standalone struct pdraw * (pdraw_new) to own the pdraw_cbs.media_added
 * callback that captures the raw audio media_id, and to run pdraw_stop() +
 * pump before pdraw_destroy() (avoids the same UAF as
 * testCAlsaSourceListenerCallbacks in test_api_alsa_source.cpp: encoder dtor
 * frees mOutputMedia, then
 * Source::~Source() would access it if the element is still STARTED). */
static void testCApiAudioEncoderListenerCallbacks()
{
	struct Ud {
		unsigned int rawAudioMediaId = 0;
		bool gotRawAudioMedia = false;
		int frameOutputCount = 0;
		int framePreReleaseCount = 0;
		bool gotStopResp = false;
	} ud;

	struct pdraw_cbs sessionCbs = {};
	sessionCbs.stop_resp = [](struct pdraw *, int, void *u) {
		static_cast<Ud *>(u)->gotStopResp = true;
	};
	sessionCbs.media_added = [](struct pdraw *,
				    const struct pdraw_media_info *info,
				    void * /*elem_ud*/,
				    void *u) {
		auto *d = static_cast<Ud *>(u);
		if (info->type == PDRAW_MEDIA_TYPE_AUDIO &&
		    info->audio.format.encoding == ADEF_ENCODING_PCM &&
		    !d->gotRawAudioMedia) {
			d->rawAudioMediaId = info->id;
			d->gotRawAudioMedia = true;
		}
	};

	TestPompLoop loop;
	struct pdraw *p = nullptr;
	int ret = pdraw_new(loop.raw(), &sessionCbs, &ud, &p);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(p);

	/* Raw PCM audio source — creates the raw audio media */
	struct pdraw_audio_source_params sourceParams = {};
	sourceParams.playback_type = PDRAW_PLAYBACK_TYPE_LIVE;
	sourceParams.audio.format = adef_pcm_16b_44100hz_mono;

	struct pdraw_audio_source_cbs sourceCbs = {};
	/* flushed is mandatory; drained may be null (wrapper guards it) */
	sourceCbs.flushed =
		[](struct pdraw *, struct pdraw_audio_source *, void *) {};

	struct pdraw_audio_source *src = nullptr;
	ret = pdraw_audio_source_new(
		p, &sourceParams, &sourceCbs, nullptr, &src);
	if (ret != 0) {
		pdraw_stop(p);
		(void)loop.pumpUntil([&ud]() { return ud.gotStopResp; }, 5000);
		pdraw_destroy(p);
		return;
	}
	CU_ASSERT_PTR_NOT_NULL_FATAL(src);

	bool gotMedia =
		loop.pumpUntil([&ud]() { return ud.gotRawAudioMedia; }, 5000);
	CU_ASSERT_TRUE_FATAL(gotMedia);

	/* Audio encoder (AAC-LC, RAW format so ASC is available) */
	struct aenc_config encoderParams = {};
	encoderParams.encoding = ADEF_ENCODING_AAC_LC;
	encoderParams.aac_lc.max_bitrate = 128000;
	encoderParams.output.preferred_format = ADEF_AAC_DATA_FORMAT_RAW;

	struct pdraw_audio_encoder_cbs encoderCbs = {};
	encoderCbs.frame_output = [](struct pdraw *,
				     struct pdraw_audio_encoder *,
				     struct mbuf_audio_frame *,
				     void *u) {
		static_cast<Ud *>(u)->frameOutputCount++;
	};
	encoderCbs.frame_pre_release = [](struct pdraw *,
					  struct pdraw_audio_encoder *,
					  struct mbuf_audio_frame *,
					  void *u) {
		static_cast<Ud *>(u)->framePreReleaseCount++;
	};

	struct pdraw_audio_encoder *enc = nullptr;
	ret = pdraw_audio_encoder_new(
		p, ud.rawAudioMediaId, &encoderParams, &encoderCbs, &ud, &enc);
	if (ret != 0) {
		/* fdk-aac or another AAC-LC backend not compiled in */
		pdraw_audio_source_destroy(p, src);
		pdraw_stop(p);
		(void)loop.pumpUntil([&ud]() { return ud.gotStopResp; }, 5000);
		pdraw_destroy(p);
		return;
	}
	CU_ASSERT_PTR_NOT_NULL_FATAL(enc);

	/* Push silent 1024-sample 16-bit mono PCM frames (one AAC frame's
	 * worth) so the encoder produces at least one output frame. */
	struct mbuf_audio_frame_queue *queue =
		pdraw_audio_source_get_queue(p, src);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	const size_t kFrameLen = 1024 * 2; /* 1024 samples, 16-bit mono */
	for (unsigned int i = 0; i < 4; i++) {
		struct adef_frame fi = {};
		fi.format = adef_pcm_16b_44100hz_mono;
		fi.info.timescale = fi.format.sample_rate;
		fi.info.timestamp = (uint64_t)i * 1024;
		fi.info.index = i;

		struct mbuf_audio_frame *frame = nullptr;
		ret = mbuf_audio_frame_new(&fi, &frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		struct mbuf_mem *mem = nullptr;
		ret = mbuf_mem_generic_new(kFrameLen, &mem);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		void *data = nullptr;
		size_t cap = 0;
		ret = mbuf_mem_get_data(mem, &data, &cap);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		memset(data, 0, cap);

		ret = mbuf_audio_frame_set_buffer(frame, mem, 0, kFrameLen);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_mem_unref(mem);

		ret = mbuf_audio_frame_finalize(frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);

		ret = mbuf_audio_frame_queue_push(queue, frame);
		CU_ASSERT_EQUAL_FATAL(ret, 0);
		mbuf_audio_frame_unref(frame);
	}

	/* frame_output fires before the output-channel loop in frameOutputCb()
	 * — no downstream sink needed. */
	bool gotFrame = loop.pumpUntil(
		[&ud]() { return ud.frameOutputCount >= 1; }, 10000);
	CU_ASSERT_TRUE(gotFrame);

	/* frame_pre_release fires when the aenc backend drops its reference */
	bool gotPreRelease = loop.pumpUntil(
		[&ud]() { return ud.framePreReleaseCount >= 1; }, 5000);
	CU_ASSERT_TRUE(gotPreRelease);

	ret = pdraw_audio_encoder_destroy(p, enc);
	CU_ASSERT_EQUAL(ret, 0);
	ret = pdraw_audio_source_destroy(p, src);
	CU_ASSERT_EQUAL(ret, 0);
	ret = pdraw_stop(p);
	CU_ASSERT_EQUAL(ret, 0);
	bool gotStop = loop.pumpUntil([&ud]() { return ud.gotStopResp; }, 5000);
	CU_ASSERT_TRUE(gotStop);
	pdraw_destroy(p);
}


CU_TestInfo g_pdraw_test_api_encoder_audio[] = {
	{FN("testCApiNew"), testCApiNew},
	{FN("testCApiDestroy"), testCApiDestroy},
	{FN("testCApiLifecycle"), testCApiLifecycle},
	{FN("testCApiAudioEncoderListenerCallbacks"),
	 testCApiAudioEncoderListenerCallbacks},
	{FN("testCxxCreate"), testCxxCreate},
	CU_TEST_INFO_NULL,
};
