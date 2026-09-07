/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — shared helpers for decode-mode pipeline tests
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

/* Shared boilerplate for Tier B pipeline tests.
 *
 * Decode-mode demuxer helpers (test_pipeline_decoder_video,
 * test_pipeline_decoder_audio, test_pipeline_scaler,
 * test_pipeline_encoder_video, test_pipeline_encoder_audio):
 * PipelineDemuxerListener, MediaTrackingListenerBase, closeAndDestroyDemuxer,
 * stopSessionAndWait, openDecodingDemuxerAndPlay.
 *
 * Raw C-API session helpers (test_pipeline_sourcesink_{coded,raw,audio}):
 * SessionCbState, make_pdraw.
 *
 * NOTE: Include this header only AFTER defining ULOG_TAG, just like
 * test_fixtures.hpp (which this header pulls in). */

#pragma once

#include "test_common.h"
#include "test_fixtures.hpp"

#include "pdraw_demuxer.hpp"

#include <pdraw/pdraw.h>

#include <memory>
#include <vector>

namespace PdrawTest {


/* Demuxer lifecycle listener for decode-mode pipeline tests.
 * Superset: seek-response tracking (mGotSeekResponse / mSeekStatus) is only
 * exercised by test_pipeline_decoder_video; the other files that include this
 * header
 * simply leave those fields at their default zero/false values. */
class PipelineDemuxerListener : public Pdraw::IPdraw::IDemuxer::Listener {
public:
	void demuxerOpenResponse(Pdraw::IPdraw * /*p*/,
				 Pdraw::IPdraw::IDemuxer * /*d*/,
				 int status) override
	{
		mOpenStatus = status;
		mGotOpenResponse = true;
	}

	void demuxerCloseResponse(Pdraw::IPdraw * /*p*/,
				  Pdraw::IPdraw::IDemuxer * /*d*/,
				  int status) override
	{
		mCloseStatus = status;
		mGotCloseResponse = true;
	}

	void
	onDemuxerUnrecoverableError(Pdraw::IPdraw * /*p*/,
				    Pdraw::IPdraw::IDemuxer * /*d*/) override
	{
		mGotUnrecoverableError = true;
	}

	int demuxerSelectMedia(Pdraw::IPdraw * /*p*/,
			       Pdraw::IPdraw::IDemuxer * /*d*/,
			       const struct pdraw_demuxer_media * /*m*/,
			       size_t /*c*/,
			       uint32_t /*sel*/) override
	{
		/* -ENOSYS: not implemented, choose the default medias. */
		return -ENOSYS;
	}

	void demuxerReadyToPlay(Pdraw::IPdraw * /*p*/,
				Pdraw::IPdraw::IDemuxer * /*d*/,
				bool ready) override
	{
		mReady = ready;
		mGotReadyToPlay = true;
	}

	void onDemuxerEndOfRange(Pdraw::IPdraw * /*p*/,
				 Pdraw::IPdraw::IDemuxer * /*d*/,
				 uint64_t /*ts*/) override
	{
	}

	void demuxerPlayResponse(Pdraw::IPdraw * /*p*/,
				 Pdraw::IPdraw::IDemuxer * /*d*/,
				 int status,
				 uint64_t /*ts*/,
				 float /*sp*/) override
	{
		mPlayStatus = status;
		mGotPlayResponse = true;
	}

	void demuxerPauseResponse(Pdraw::IPdraw * /*p*/,
				  Pdraw::IPdraw::IDemuxer * /*d*/,
				  int /*s*/,
				  uint64_t /*ts*/) override
	{
	}

	void demuxerSeekResponse(Pdraw::IPdraw * /*p*/,
				 Pdraw::IPdraw::IDemuxer * /*d*/,
				 int status,
				 uint64_t /*ts*/,
				 float /*sp*/) override
	{
		mSeekStatus = status;
		mGotSeekResponse = true;
	}

	bool mGotOpenResponse = false;
	int mOpenStatus = 0;
	bool mGotReadyToPlay = false;
	bool mReady = false;
	bool mGotPlayResponse = false;
	int mPlayStatus = 0;
	bool mGotCloseResponse = false;
	int mCloseStatus = 0;
	bool mGotSeekResponse = false;
	int mSeekStatus = 0;
	bool mGotUnrecoverableError = false;
};


/* Close a decode-mode demuxer, pump the close response, then destroy it.
 * Takes ownership of obj via unique_ptr. timeoutMs applies to each of the
 * two pumpUntil() waits below; bump it for callers whose pipeline involves
 * heavier real codecs (see testCxxVideoEncoderProducesRealJpegOutput, which
 * already uses 15000ms for its other waits on the same pipeline). */
inline void closeAndDestroyDemuxer(Pdraw::IPdraw::IDemuxer *obj,
				   TestPompLoop *loop,
				   PipelineDemuxerListener *listener,
				   int timeoutMs = 5000)
{
	auto objOwner = std::unique_ptr<Pdraw::IPdraw::IDemuxer>(obj);
	int ret = obj->close();
	CU_ASSERT_EQUAL(ret, 0);
	bool gotClose = loop->pumpUntil(
		[listener]() { return listener->mGotCloseResponse; },
		timeoutMs);
	CU_ASSERT_TRUE(gotClose);
	CU_ASSERT_EQUAL(listener->mCloseStatus, 0);
	CU_ASSERT_FALSE(listener->mGotUnrecoverableError);

	/* demuxerCloseResponse() is dispatched via an idle callback queued
	 * BEFORE the Demuxer element's STOPPED state-change idle, which
	 * itself queues Session::asyncElementDelete() as a THIRD, later idle
	 * (same ordering as Muxer::completeStop(), pdraw_muxer.cpp). gotClose
	 * above only proves the first of these three ran -- keep pumping past
	 * it so the element is actually deleted (running
	 * DemuxerWrapper::clearElement()) while objOwner is still alive. */
	bool gotClearElement = loop->pumpUntil(
		[obj]() {
			return static_cast<Pdraw::DemuxerWrapper *>(obj)
				       ->getDemuxer() == nullptr;
		},
		timeoutMs);
	CU_ASSERT_TRUE(gotClearElement);
}


/* Stop a pdraw session and wait for its stopResponse callback.
 * Templated on the listener type: any listener with mGotStopResponse and
 * mStopStatus members works (MediaTrackingListener, SourceMediaListener…). */
template <typename ListenerT>
void stopSessionAndWait(TestPompLoop *loop,
			Pdraw::IPdraw *session,
			ListenerT *listener)
{
	int ret = session->stop();
	CU_ASSERT_EQUAL(ret, 0);
	/* Generous timeout: tearing down a real decoder pipeline takes
	 * noticeably longer than the 5s pumpUntil() default. */
	bool gotStop = loop->pumpUntil(
		[listener]() { return listener->mGotStopResponse; }, 20000);
	CU_ASSERT_TRUE(gotStop);
	CU_ASSERT_EQUAL(listener->mStopStatus, 0);
}


/* Open a decode-mode demuxer on the given path, wait for ready-to-play,
 * then call play() and wait for play response.
 *
 * path is expected to have been resolved (and validated readable) by the
 * caller via PDRAW_GET_ASSET_PATH before calling this function.
 *
 * Returns the created demuxer; the caller owns it and is responsible for
 * closing and destroying it (e.g. via closeAndDestroyDemuxer()). */
[[nodiscard]] inline Pdraw::IPdraw::IDemuxer *openDecodingDemuxerAndPlay(
	Pdraw::IPdraw *session,
	TestPompLoop *loop,
	PipelineDemuxerListener *listener,
	const char *path,
	enum pdraw_playback_mode mode = PDRAW_PLAYBACK_MODE_OFFLINE)
{
	struct pdraw_demuxer_params params = {};
	params.autodecoding_mode = PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL;
	params.playback_mode = mode;

	Pdraw::IPdraw::IDemuxer *obj = nullptr;
	int ret = session->createDemuxer(path, &params, listener, &obj);
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(obj);

	bool gotReady = loop->pumpUntil(
		[listener]() { return listener->mGotReadyToPlay; });
	CU_ASSERT_TRUE_FATAL(gotReady);
	CU_ASSERT_TRUE_FATAL(listener->mReady);

	ret = obj->play();
	CU_ASSERT_EQUAL_FATAL(ret, 0);
	bool gotPlay = loop->pumpUntil(
		[listener]() { return listener->mGotPlayResponse; });
	CU_ASSERT_TRUE_FATAL(gotPlay);
	CU_ASSERT_EQUAL_FATAL(listener->mPlayStatus, 0);

	return obj;
}


/* Base class shared by the per-TU MediaTrackingListener subclasses in
 * test_pipeline_decoder_video, test_pipeline_decoder_audio,
 * test_pipeline_scaler, test_pipeline_encoder_video, and
 * test_pipeline_encoder_audio. Factors out the common recording body
 * (Added struct, onMediaAdded, data members) so each subclass only adds
 * the finder methods it actually needs. */
class MediaTrackingListenerBase : public Pdraw::IPdraw::Listener {
public:
	struct Added {
		unsigned int id;
		enum pdraw_media_type type;
		enum vdef_frame_type videoFormat;
		struct vdef_format_info videoInfo;
		struct adef_format audioFormat;
	};

	void stopResponse(Pdraw::IPdraw * /*p*/, int status) override
	{
		mStopStatus = status;
		mGotStopResponse = true;
	}

	void onMediaAdded(Pdraw::IPdraw * /*p*/,
			  const struct pdraw_media_info *info,
			  void * /*elementUserData*/) override
	{
		Added a = {};
		a.id = info->id;
		a.type = info->type;
		if (info->type == PDRAW_MEDIA_TYPE_VIDEO) {
			a.videoFormat = info->video.format;
			if (a.videoFormat == VDEF_FRAME_TYPE_RAW)
				a.videoInfo = info->video.raw.info;
		} else if (info->type == PDRAW_MEDIA_TYPE_AUDIO) {
			a.audioFormat = info->audio.format;
		}
		mAdded.push_back(a);
	}

	void onMediaRemoved(Pdraw::IPdraw * /*p*/,
			    const struct pdraw_media_info * /*i*/,
			    void * /*u*/) override
	{
	}

	void onSocketCreated(Pdraw::IPdraw * /*p*/, int /*fd*/) override {}

	std::vector<Added> mAdded;
	bool mGotStopResponse = false;
	int mStopStatus = 0;
};


/* Minimal raw C-API (pdraw_new()) session state/callbacks, shared verbatim
 * by test_pipeline_sourcesink_{coded,raw,audio}.cpp: these only track
 * session-level stop/media_added/media_removed counts, everything specific
 * to a media type (source/sink listeners, frame parsing, ...) stays in each
 * TU. */
struct SessionCbState {
	int stopRespCount = 0;
	int mediaAddedCount = 0;
	int mediaRemovedCount = 0;
	unsigned int lastMediaId = 0;
};

inline void session_stop_resp_cb(struct pdraw * /*p*/, int /*status*/, void *ud)
{
	static_cast<SessionCbState *>(ud)->stopRespCount++;
}

inline void session_media_added_cb(struct pdraw * /*p*/,
				   const struct pdraw_media_info *info,
				   void * /*elem*/,
				   void *ud)
{
	auto *s = static_cast<SessionCbState *>(ud);
	s->mediaAddedCount++;
	s->lastMediaId = info->id;
}

inline void session_media_removed_cb(struct pdraw * /*p*/,
				     const struct pdraw_media_info * /*info*/,
				     void * /*elem*/,
				     void *ud)
{
	static_cast<SessionCbState *>(ud)->mediaRemovedCount++;
}

inline bool
make_pdraw(TestPompLoop &loop, SessionCbState &state, struct pdraw **out)
{
	struct pdraw_cbs cbs = {};
	cbs.stop_resp = session_stop_resp_cb;
	cbs.media_added = session_media_added_cb;
	cbs.media_removed = session_media_removed_cb;
	int ret = pdraw_new(loop.raw(), &cbs, &state, out);
	return ret == 0 && *out != nullptr;
}


} /* namespace PdrawTest */
