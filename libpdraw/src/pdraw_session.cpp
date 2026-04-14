/**
 * Parrot Drones Audio and Video Vector library
 * Session
 *
 * Copyright (c) 2018 Parrot Drones SAS
 * Copyright (c) 2016 Aurelien Barre
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

#define ULOG_TAG pdraw_session
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_decoder_audio.hpp"
#include "pdraw_decoder_video.hpp"
#include "pdraw_demuxer_record.hpp"
#include "pdraw_demuxer_stream_mux.hpp"
#include "pdraw_demuxer_stream_net.hpp"
#include "pdraw_encoder_video.hpp"
#include "pdraw_muxer_record.hpp"
#include "pdraw_muxer_stream_rtmp.hpp"
#include "pdraw_scaler_video.hpp"
#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"

#include <math.h>
#include <string.h>

#include <algorithm>
#include <string>
#include <vector>

namespace Pdraw {


int createPdraw(struct pomp_loop *loop,
		IPdraw::Listener *listener,
		IPdraw **retObj)
{
	ULOG_ERRNO_RETURN_ERR_IF(loop == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	try {
		auto pdraw = make_unique<Session>(loop, listener);
		*retObj = pdraw.release();
		return 0;
	} catch (const std::bad_alloc &) {
		ULOGE("failed to create pdraw instance");
		return -ENOMEM;
	}
}


const char *
pdrawDemuxerAutodecodingModeStr(enum pdraw_demuxer_autodecoding_mode val)
{
	return pdraw_demuxerAutodecodingModeStr(val);
}


enum pdraw_demuxer_autodecoding_mode
pdrawDemuxerAutodecodingModeFromStr(const char *val)
{
	return pdraw_demuxerAutodecodingModeFromStr(val);
}


const char *pdrawPlaybackTypeStr(enum pdraw_playback_type val)
{
	return pdraw_playbackTypeStr(val);
}


enum pdraw_playback_type pdrawPlaybackTypeFromStr(const char *val)
{
	return pdraw_playbackTypeFromStr(val);
}


const char *pdrawMediaTypeStr(enum pdraw_media_type val)
{
	return pdraw_mediaTypeStr(val);
}


enum pdraw_media_type pdrawMediaTypeFromStr(const char *val)
{
	return pdraw_mediaTypeFromStr(val);
}


const char *pdrawHistogramChannelStr(enum pdraw_histogram_channel val)
{
	return pdraw_histogramChannelStr(val);
}


enum pdraw_histogram_channel pdrawHistogramChannelFromStr(const char *val)
{
	return pdraw_histogramChannelFromStr(val);
}


const char *pdrawVideoRendererSchedulingModeStr(
	enum pdraw_video_renderer_scheduling_mode val)
{
	return pdraw_videoRendererSchedulingModeStr(val);
}


enum pdraw_video_renderer_scheduling_mode
pdrawVideoRendererSchedulingModeFromStr(const char *val)
{
	return pdraw_videoRendererSchedulingModeFromStr(val);
}


const char *
pdrawVideoRendererFillModeStr(enum pdraw_video_renderer_fill_mode val)
{
	return pdraw_videoRendererFillModeStr(val);
}


enum pdraw_video_renderer_fill_mode
pdrawVideoRendererFillModeFromStr(const char *val)
{
	return pdraw_videoRendererFillModeFromStr(val);
}


const char *pdrawVideoRendererTransitionFlagStr(
	enum pdraw_video_renderer_transition_flag val)
{
	return pdraw_videoRendererTransitionFlagStr(val);
}


enum pdraw_video_renderer_transition_flag
pdrawVideoRendererTransitionFlagFromStr(const char *val)
{
	return pdraw_videoRendererTransitionFlagFromStr(val);
}


const char *pdrawVipcSourceEosReasonStr(enum pdraw_vipc_source_eos_reason val)
{
	return pdraw_vipcSourceEosReasonStr(val);
}


enum pdraw_vipc_source_eos_reason
pdrawVipcSourceEosReasonFromStr(const char *val)
{
	return pdraw_vipcSourceEosReasonFromStr(val);
}


int pdrawVideoFrameToJsonStr(const struct pdraw_video_frame *frame,
			     struct vmeta_frame *metadata,
			     char *str,
			     unsigned int len)
{
	return pdraw_frameMetadataToJsonStr(frame, metadata, str, len);
}


int pdrawVideoFrameToJson(const struct pdraw_video_frame *frame,
			  struct vmeta_frame *metadata,
			  struct json_object *jobj)
{
	return pdraw_frameMetadataToJson(frame, metadata, jobj);
}


struct pdraw_media_info *pdrawMediaInfoDup(const struct pdraw_media_info *src)
{
	return pdraw_mediaInfoDup(src);
}


void pdrawMediaInfoFree(struct pdraw_media_info *media_info)
{
	return pdraw_mediaInfoFree(media_info);
}


struct pdraw_vipc_source_params *
pdrawVipcSourceParamsDup(const struct pdraw_vipc_source_params *src)
{
	return pdraw_vipcSourceParamsDup(src);
}


void pdrawVipcSourceParamsFree(struct pdraw_vipc_source_params *params)
{
	return pdraw_vipcSourceParamsFree(params);
}


struct pdraw_muxer_params *
pdrawMuxerParamsDup(const struct pdraw_muxer_params *src)
{
	return pdraw_muxerParamsDup(src);
}


void pdrawMuxerParamsFree(struct pdraw_muxer_params *params)
{
	return pdraw_muxerParamsFree(params);
}


struct pdraw_muxer_media_params *
pdrawMuxerMediaParamsDup(const struct pdraw_muxer_media_params *src)
{
	return pdraw_muxerMediaParamsDup(src);
}


void pdrawMuxerMediaParamsFree(struct pdraw_muxer_media_params *params)
{
	return pdraw_muxerMediaParamsFree(params);
}


void pdrawDemuxerMediaListFree(struct pdraw_demuxer_media *mediaList,
			       size_t mediaCount)
{
	return pdraw_demuxerMediaListFree(mediaList, mediaCount);
}


int pdrawAlsaSourceGetCapabilities(const std::string &address,
				   struct pdraw_alsa_source_caps *caps)
{
#ifdef PDRAW_USE_ALSA
	return Pdraw::AlsaSource::getCapabilities(address, caps);
#else
	PDRAW_UNUSED(address);
	PDRAW_UNUSED(caps);

	return -ENOSYS;
#endif
}


Session::Session(struct pomp_loop *loop, IPdraw::Listener *listener) :
		mFactory(this), mListener(listener), mLoop(loop)
{
	mLoopThread = pthread_self();

	setState(State::READY);
}


Session::~Session()
{
	if (mState != State::STOPPED)
		ULOGW("destroying while instance is still running");

	{
		std::unique_lock<std::recursive_mutex> lock(mMutex);
		mElements.clear();
	}

	/* Remove any leftover idle callbacks */
	if (mLoop != nullptr) {
		int err = pomp_loop_idle_remove_by_cookie(mLoop, this);
		if (err > 0)
			ULOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);
	}

	{
		std::unique_lock<std::mutex> lock(mAsyncMutex);
		while (!mMediaAddedInfoArgs.empty()) {
			struct pdraw_media_info info =
				mMediaAddedInfoArgs.front();
			mMediaAddedInfoArgs.pop();
			Media::cleanupMediaInfo(&info);
		}
		while (!mMediaRemovedInfoArgs.empty()) {
			struct pdraw_media_info info =
				mMediaRemovedInfoArgs.front();
			mMediaRemovedInfoArgs.pop();
			Media::cleanupMediaInfo(&info);
		}
	}
}


/*
 * API methods
 */

int Session::stop()
{
	int ret;
	bool stopped = true;
	std::vector<Element *>::iterator e;

	if (mState == State::STOPPING) {
		/* Return without calling the stopResponse() function */
		ULOGI("%s: already in %s state, nothing to do",
		      __func__,
		      stateStr(mState));
		return 0;
	}

	if (mState == State::STOPPED) {
		/* Call the stopResponse() function with OK status */
		ULOGI("%s: state is %s, nothing to do",
		      __func__,
		      stateStr(mState));
		ret = 0;
		goto already_stopped;
	}

	if (mState != State::READY) {
		ULOGE("%s: invalid state (%s)", __func__, stateStr(mState));
		return -EPROTO;
	}

	setState(State::STOPPING);

	{
		std::unique_lock<std::recursive_mutex> lock(mMutex);
		for (auto &elem : mElements) {
			if (elem->getState() != Element::State::STOPPED) {
				stopped = false;
				int err = elem->stop();
				if (err < 0)
					ULOG_ERRNO("element->stop", -err);
			}
		}
	}

	if (stopped) {
		/* Call the stopResponse() function with OK status */
		ULOGI("%s: all elements are stopped, closing", __func__);
		setState(State::STOPPED);
		ret = 0;
		goto already_stopped;
	}

	/* Waiting for the asynchronous stop; stopResponse()
	 * will be called when it's done */
	return 0;

already_stopped:
	if (mListener != nullptr && ret == 0)
		stopResp(ret);
	return ret;
}


/* Called on the rendering thread */
int Session::createVideoRenderer(
	unsigned int mediaId,
	const struct pdraw_rect *renderPos,
	const struct pdraw_video_renderer_params *params,
	IPdraw::IVideoRenderer::Listener *listener,
	IPdraw::IVideoRenderer **retObj)
{
	int res;
	std::unique_ptr<VideoRendererWrapper> renderer;

	ULOG_ERRNO_RETURN_ERR_IF(renderPos == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	mMutex.lock();
	if (mState == State::STOPPING || mState == State::STOPPED) {
		ULOGE("renderer creation refused in %s state",
		      stateStr(mState));
		mMutex.unlock();
		return -EPROTO;
	}

	try {
		renderer = make_unique<VideoRendererWrapper>(
			this, mediaId, renderPos, params, listener);
	} catch (const std::bad_alloc &) {
		mMutex.unlock();
		ULOGE("%s: failed to create the video renderer wrapper",
		      __func__);
		return -ENOMEM;
	}

	if (renderer->getElement() == nullptr) {
		mMutex.unlock();
		ULOGE("%s: failed to create the video renderer", __func__);
		return -EPROTO;
	}

	mElements.push_back(std::unique_ptr<Element>(renderer->getElement()));
	mMutex.unlock();

	res = renderer->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("renderer->start", -res);
		return res;
	}

	*retObj = renderer.release();

	return 0;
}


int Session::createDemuxer(const std::string &url,
			   const struct pdraw_demuxer_params *params,
			   IPdraw::IDemuxer::Listener *listener,
			   IPdraw::IDemuxer **retObj)
{
	return createDemuxer(url, nullptr, params, listener, retObj);
}


int Session::createDemuxer(const std::string &localAddr,
			   uint16_t localStreamPort,
			   uint16_t localControlPort,
			   const std::string &remoteAddr,
			   uint16_t remoteStreamPort,
			   uint16_t remoteControlPort,
			   const struct pdraw_demuxer_params *params,
			   IPdraw::IDemuxer::Listener *listener,
			   IPdraw::IDemuxer **retObj)
{
	int res;
	std::unique_ptr<DemuxerWrapper> demuxer;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	mMutex.lock();
	if (mState == State::STOPPING || mState == State::STOPPED) {
		ULOGE("demuxer creation refused in %s state", stateStr(mState));
		mMutex.unlock();
		return -EPROTO;
	}

	try {
		demuxer = make_unique<DemuxerWrapper>(this,
						      localAddr,
						      localStreamPort,
						      localControlPort,
						      remoteAddr,
						      remoteStreamPort,
						      remoteControlPort,
						      params,
						      listener);
	} catch (const std::bad_alloc &) {
		mMutex.unlock();
		ULOGE("%s: failed to create the demuxer wrapper", __func__);
		return -ENOMEM;
	}

	if (demuxer->getElement() == nullptr) {
		mMutex.unlock();
		ULOGE("%s: failed to create the demuxer", __func__);
		return -EPROTO;
	}

	mElements.push_back(std::unique_ptr<Element>(demuxer->getElement()));
	mMutex.unlock();

	res = demuxer->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("demuxer->start", -res);
		return res;
	}

	*retObj = demuxer.release();

	/* Waiting for the asynchronous open; openResponse()
	 * will be called when it's done */
	return 0;
}


int Session::createDemuxer(const std::string &url,
			   struct mux_ctx *mux,
			   const struct pdraw_demuxer_params *params,
			   IPdraw::IDemuxer::Listener *listener,
			   IPdraw::IDemuxer **retObj)
{
	int res;
	std::unique_ptr<DemuxerWrapper> demuxer;

	ULOG_ERRNO_RETURN_ERR_IF(url.length() == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	mMutex.lock();
	if (mState == State::STOPPING || mState == State::STOPPED) {
		ULOGE("demuxer creation refused in %s state", stateStr(mState));
		mMutex.unlock();
		return -EPROTO;
	}

	try {
		demuxer = make_unique<DemuxerWrapper>(
			this, url, mux, params, listener);
	} catch (const std::bad_alloc &) {
		mMutex.unlock();
		ULOGE("%s: failed to create the demuxer wrapper", __func__);
		return -ENOMEM;
	}

	if (demuxer->getElement() == nullptr) {
		mMutex.unlock();
		ULOGE("%s: failed to create the demuxer", __func__);
		return -EPROTO;
	}

	mElements.push_back(std::unique_ptr<Element>(demuxer->getElement()));
	mMutex.unlock();

	res = demuxer->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("demuxer->start", -res);
		return res;
	}

	*retObj = demuxer.release();

	/* Waiting for the asynchronous open; openResponse()
	 * will be called when it's done */
	return 0;
}


int Session::createMuxer(const std::string &url,
			 const struct pdraw_muxer_params *params,
			 IPdraw::IMuxer::Listener *listener,
			 IPdraw::IMuxer **retObj)
{
	int res;
	std::unique_ptr<MuxerWrapper> muxer;

	ULOG_ERRNO_RETURN_ERR_IF(url.length() == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	mMutex.lock();
	if (mState == State::STOPPING || mState == State::STOPPED) {
		ULOGE("muxer creation refused in %s state", stateStr(mState));
		mMutex.unlock();
		return -EPROTO;
	}

	try {
		muxer = make_unique<MuxerWrapper>(this, url, params, listener);
	} catch (const std::bad_alloc &) {
		mMutex.unlock();
		ULOGE("%s: failed to create the muxer wrapper", __func__);
		return -ENOMEM;
	}

	if (muxer->getElement() == nullptr) {
		mMutex.unlock();
		ULOGE("%s: failed to create the muxer", __func__);
		return -EPROTO;
	}

	mElements.push_back(std::unique_ptr<Element>(muxer->getElement()));
	mMutex.unlock();

	res = muxer->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("muxer->start", -res);
		return res;
	}

	*retObj = muxer.release();

	return 0;
}


int Session::internalCreateCodedVideoSink(
	Source *source,
	CodedVideoMedia *media,
	const struct pdraw_video_sink_params *params,
	IPdraw::ICodedVideoSink::Listener *listener,
	IPdraw::ICodedVideoSink **retObj)
{
	/* Note: mMutex is held while this function is called */
	int res;
	std::unique_ptr<CodedVideoSinkWrapper> sink;
	Channel *channel = nullptr;

	try {
		sink = make_unique<CodedVideoSinkWrapper>(
			this, 0, params, listener);
	} catch (const std::bad_alloc &) {
		ULOGE("%s: failed to create the sink wrapper", __func__);
		return -ENOMEM;
	}

	if (sink->getElement() == nullptr) {
		ULOGE("%s: failed to create the sink", __func__);
		return -EPROTO;
	}

	mElements.push_back(std::unique_ptr<Element>(sink->getElement()));

	res = sink->getSink()->addInputMedia(media);
	if (res < 0) {
		ULOG_ERRNO("codedVideoSink->addInputMedia", -res);
		goto error;
	}

	res = sink->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("codedVideoSink->start", -res);
		goto error;
	}

	channel = sink->getSink()->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get coded video sink input channel");
		res = -EPROTO;
		goto error;
	}

	res = source->addOutputChannel(media, channel);
	if (res < 0) {
		ULOG_ERRNO("source->addOutputChannel", -res);
		goto error;
	}

	/* Force a resync after linking the elements; this allows a coded
	 * video sink to start on an IDR frame for example */
	res = sink->getCodedVideoSink()->resync();
	if (res < 0) {
		ULOG_ERRNO("codedVideoSink->resync", -res);
		goto error;
	}

	*retObj = sink.release();

	return 0;

error:
	if ((sink != nullptr) && (channel != nullptr)) {
		/* removeOutputChannel must be called without mMutex
		 * being held, so release it here */
		mMutex.unlock();
		source->removeOutputChannel(media, channel);
		mMutex.lock();
	}
	return res;
}


int Session::internalCreateAudioSink(Source *source,
				     AudioMedia *media,
				     IPdraw::IAudioSink::Listener *listener,
				     IPdraw::IAudioSink **retObj)
{
	/* Note: mMutex is held while this function is called */
	int res;
	std::unique_ptr<AudioSinkWrapper> sink;
	Channel *channel = nullptr;

	try {
		sink = make_unique<AudioSinkWrapper>(this, 0, listener);
	} catch (const std::bad_alloc &) {
		ULOGE("%s: failed to create the sink wrapper", __func__);
		return -ENOMEM;
	}

	if (sink->getElement() == nullptr) {
		ULOGE("%s: failed to create the sink", __func__);
		return -EPROTO;
	}

	mElements.push_back(std::unique_ptr<Element>(sink->getElement()));

	res = sink->getSink()->addInputMedia(media);
	if (res < 0) {
		ULOG_ERRNO("Sink::addInputMedia", -res);
		goto error;
	}

	res = sink->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("Element::start", -res);
		goto error;
	}

	channel = sink->getSink()->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get audio sink input channel");
		res = -EPROTO;
		goto error;
	}

	res = source->addOutputChannel(media, channel);
	if (res < 0) {
		ULOG_ERRNO("Source::addOutputChannel", -res);
		goto error;
	}

	*retObj = sink.release();

	return 0;

error:
	if ((sink != nullptr) && (channel != nullptr)) {
		/* removeOutputChannel must be called without mMutex
		 * being held, so release it here */
		mMutex.unlock();
		source->removeOutputChannel(media, channel);
		mMutex.lock();
	}
	return res;
}


int Session::createVipcSource(const struct pdraw_vipc_source_params *params,
			      IPdraw::IVipcSource::Listener *listener,
			      IPdraw::IVipcSource **retObj)
{
#ifdef BUILD_LIBVIDEO_IPC
	int res;
	std::unique_ptr<VipcSourceWrapper> source;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->address == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	mMutex.lock();

	try {
		source = make_unique<VipcSourceWrapper>(this, params, listener);
	} catch (const std::bad_alloc &) {
		mMutex.unlock();
		ULOGE("%s: failed to create the video IPC source wrapper",
		      __func__);
		return -ENOMEM;
	}

	if (source->getElement() == nullptr) {
		mMutex.unlock();
		ULOGE("%s: failed to create the video IPC source", __func__);
		return -EPROTO;
	}

	mElements.push_back(std::unique_ptr<Element>(source->getElement()));
	mMutex.unlock();

	res = source->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("vipcSource->start", -res);
		return res;
	}

	*retObj = source.release();

	return 0;
#else
	return -ENOSYS;
#endif
}


int Session::createCodedVideoSource(
	const struct pdraw_video_source_params *params,
	IPdraw::ICodedVideoSource::Listener *listener,
	IPdraw::ICodedVideoSource **retObj)
{
	int res;
	std::unique_ptr<CodedVideoSourceWrapper> source;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->video.format != VDEF_FRAME_TYPE_CODED,
				 EINVAL);

	mMutex.lock();

	try {
		source = make_unique<CodedVideoSourceWrapper>(
			this, params, listener);
	} catch (const std::bad_alloc &) {
		mMutex.unlock();
		ULOGE("%s: failed to create the source wrapper", __func__);
		return -ENOMEM;
	}

	if (source->getElement() == nullptr) {
		mMutex.unlock();
		ULOGE("%s: failed to create the source", __func__);
		return -EPROTO;
	}

	mElements.push_back(std::unique_ptr<Element>(source->getElement()));
	mMutex.unlock();

	res = source->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("codedVideoSource->start", -res);
		return res;
	}

	*retObj = source.release();

	return 0;
}


int Session::createRawVideoSource(
	const struct pdraw_video_source_params *params,
	IPdraw::IRawVideoSource::Listener *listener,
	IPdraw::IRawVideoSource **retObj)
{
	int res;
	std::unique_ptr<RawVideoSourceWrapper> source;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->video.format != VDEF_FRAME_TYPE_RAW,
				 EINVAL);

	mMutex.lock();

	try {
		source = make_unique<RawVideoSourceWrapper>(
			this, params, listener);
	} catch (const std::bad_alloc &) {
		mMutex.unlock();
		ULOGE("%s: failed to create the source wrapper", __func__);
		return -ENOMEM;
	}

	if (source->getElement() == nullptr) {
		mMutex.unlock();
		ULOGE("%s: failed to create the source", __func__);
		return -EPROTO;
	}

	mElements.push_back(std::unique_ptr<Element>(source->getElement()));
	mMutex.unlock();

	res = source->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("rawVideoSource->start", -res);
		return res;
	}

	*retObj = source.release();

	return 0;
}


int Session::createCodedVideoSink(unsigned int mediaId,
				  const struct pdraw_video_sink_params *params,
				  IPdraw::ICodedVideoSink::Listener *listener,
				  IPdraw::ICodedVideoSink **retObj)
{
	int res;
	std::unique_ptr<CodedVideoSinkWrapper> sink;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	mMutex.lock();
	if (mState == State::STOPPING || mState == State::STOPPED) {
		ULOGE("sink creation refused in %s state", stateStr(mState));
		mMutex.unlock();
		return -EPROTO;
	}

	try {
		sink = make_unique<CodedVideoSinkWrapper>(
			this, mediaId, params, listener);
	} catch (const std::bad_alloc &) {
		mMutex.unlock();
		ULOGE("%s: failed to create the sink wrapper", __func__);
		return -ENOMEM;
	}

	if (sink->getElement() == nullptr) {
		mMutex.unlock();
		ULOGE("%s: failed to create the sink", __func__);
		return -EPROTO;
	}

	mElements.push_back(std::unique_ptr<Element>(sink->getElement()));
	mMutex.unlock();

	res = sink->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("codedVideoSink->start", -res);
		return res;
	}

	*retObj = sink.release();

	return 0;
}


int Session::createRawVideoSink(unsigned int mediaId,
				const struct pdraw_video_sink_params *params,
				IPdraw::IRawVideoSink::Listener *listener,
				IPdraw::IRawVideoSink **retObj)
{
	int res;
	std::unique_ptr<RawVideoSinkWrapper> sink;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	mMutex.lock();
	if (mState == State::STOPPING || mState == State::STOPPED) {
		ULOGE("sink creation refused in %s state", stateStr(mState));
		mMutex.unlock();
		return -EPROTO;
	}

	try {
		sink = make_unique<RawVideoSinkWrapper>(
			this, mediaId, params, listener);
	} catch (const std::bad_alloc &) {
		mMutex.unlock();
		ULOGE("%s: failed to create the sink wrapper", __func__);
		return -ENOMEM;
	}

	if (sink->getElement() == nullptr) {
		mMutex.unlock();
		ULOGE("%s: failed to create the sink", __func__);
		return -EPROTO;
	}

	mElements.push_back(std::unique_ptr<Element>(sink->getElement()));
	mMutex.unlock();

	res = sink->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("rawVideoSink->start", -res);
		return res;
	}

	*retObj = sink.release();

	return 0;
}


int Session::createAlsaSource(const struct pdraw_alsa_source_params *params,
			      IPdraw::IAlsaSource::Listener *listener,
			      IPdraw::IAlsaSource **retObj)
{
#ifdef PDRAW_USE_ALSA
	int res;
	std::unique_ptr<AlsaSourceWrapper> source;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->address == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	mMutex.lock();

	try {
		source = make_unique<AlsaSourceWrapper>(this, params, listener);
	} catch (const std::bad_alloc &) {
		mMutex.unlock();
		ULOGE("%s: failed to create the ALSA source wrapper", __func__);
		return -ENOMEM;
	}

	if (source->getElement() == nullptr) {
		mMutex.unlock();
		ULOGE("%s: failed to create the ALSA source", __func__);
		return -EPROTO;
	}

	mElements.push_back(std::unique_ptr<Element>(source->getElement()));
	mMutex.unlock();

	res = source->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("alsaSource->start", -res);
		return res;
	}

	*retObj = source.release();

	return 0;
#else
	return -ENOSYS;
#endif
}


int Session::createAudioSource(const struct pdraw_audio_source_params *params,
			       IPdraw::IAudioSource::Listener *listener,
			       IPdraw::IAudioSource **retObj)
{
	int res;
	std::unique_ptr<AudioSourceWrapper> source;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	mMutex.lock();

	try {
		source =
			make_unique<AudioSourceWrapper>(this, params, listener);
	} catch (const std::bad_alloc &) {
		mMutex.unlock();
		ULOGE("%s: failed to create the source wrapper", __func__);
		return -ENOMEM;
	}

	if (source->getElement() == nullptr) {
		mMutex.unlock();
		ULOGE("%s: failed to create the source", __func__);
		return -EPROTO;
	}

	mElements.push_back(std::unique_ptr<Element>(source->getElement()));
	mMutex.unlock();

	res = source->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("Element::start", -res);
		return res;
	}

	*retObj = source.release();

	return 0;
}


int Session::createAudioSink(unsigned int mediaId,
			     IPdraw::IAudioSink::Listener *listener,
			     IPdraw::IAudioSink **retObj)
{
	int res;
	std::unique_ptr<AudioSinkWrapper> sink;

	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	mMutex.lock();
	if (mState == State::STOPPING || mState == State::STOPPED) {
		ULOGE("sink creation refused in %s state", stateStr(mState));
		mMutex.unlock();
		return -EPROTO;
	}

	try {
		sink = make_unique<AudioSinkWrapper>(this, mediaId, listener);
	} catch (const std::bad_alloc &) {
		mMutex.unlock();
		ULOGE("%s: failed to create the sink wrapper", __func__);
		return -ENOMEM;
	}

	if (sink->getElement() == nullptr) {
		mMutex.unlock();
		ULOGE("%s: failed to create the sink", __func__);
		return -EPROTO;
	}

	mElements.push_back(std::unique_ptr<Element>(sink->getElement()));
	mMutex.unlock();

	res = sink->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("audioSink->start", -res);
		return res;
	}

	*retObj = sink.release();

	return 0;
}


int Session::createAudioRenderer(
	unsigned int mediaId,
	const struct pdraw_audio_renderer_params *params,
	IPdraw::IAudioRenderer::Listener *listener,
	IPdraw::IAudioRenderer **retObj)
{
	int res;
	std::unique_ptr<AudioRendererWrapper> renderer;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->address == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	mMutex.lock();
	if (mState == State::STOPPING || mState == State::STOPPED) {
		ULOGE("renderer creation refused in %s state",
		      stateStr(mState));
		mMutex.unlock();
		return -EPROTO;
	}

	try {
		renderer = make_unique<AudioRendererWrapper>(
			this, mediaId, params, listener);
	} catch (const std::bad_alloc &) {
		mMutex.unlock();
		ULOGE("%s: failed to create the audio renderer", __func__);
		return -ENOMEM;
	}

	if (renderer->getElement() == nullptr) {
		mMutex.unlock();
		ULOGE("%s: failed to create the audio renderer wrapper",
		      __func__);
		return -EPROTO;
	}

	mElements.push_back(std::unique_ptr<Element>(renderer->getElement()));
	mMutex.unlock();

	res = renderer->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("AudioRenderer::start", -res);
		return res;
	}

	*retObj = renderer.release();

	return 0;
}


int Session::createVideoEncoder(unsigned int mediaId,
				const struct venc_config *params,
				IPdraw::IVideoEncoder::Listener *listener,
				IPdraw::IVideoEncoder **retObj)
{
	int ret = 0;
	bool found = false;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	mMutex.lock();

	auto e = mElements.begin();
	while (e != mElements.end()) {
		Media *media;
		RawVideoMedia *rawMedia = nullptr;
		auto *source = dynamic_cast<Source *>(e->get());
		if (source == nullptr) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			media = source->getOutputMedia(i);
			if ((media != nullptr) && (media->id == mediaId)) {
				rawMedia = dynamic_cast<RawVideoMedia *>(media);
				found = true;
				break;
			}
		}
		if (found && rawMedia != nullptr) {
			std::unique_ptr<VideoEncoderWrapper> wrapper;
			try {
				wrapper = make_unique<VideoEncoderWrapper>(
					this, params, listener);
			} catch (const std::bad_alloc &) {
				ULOGE("%s: failed to create the "
				      "video encoder wrapper",
				      __func__);
				ret = -ENOMEM;
				goto exit;
			}
			if (wrapper->getElement() == nullptr) {
				ULOGE("%s: failed to create the video encoder",
				      __func__);
				ret = -EPROTO;
				goto exit;
			}
			ret = mFactory.addVideoEncoderForMedia(
				source,
				rawMedia,
				params,
				listener,
				wrapper->getVideoEncoder());
			if (ret < 0) {
				ULOG_ERRNO(
					"PipelineFactory"
					"::addVideoEncoderForMedia",
					-ret);
				goto exit;
			}
			*retObj = wrapper.release();
			goto exit;
		}
		e++;
	}
	ret = -ENOENT;
exit:
	mMutex.unlock();
	return ret;
}


int Session::createVideoScaler(unsigned int mediaId,
			       const struct vscale_config *params,
			       IPdraw::IVideoScaler::Listener *listener,
			       IPdraw::IVideoScaler **retObj)
{
	int ret = 0;
	bool found = false;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	mMutex.lock();

	auto e = mElements.begin();
	while (e != mElements.end()) {
		Media *media;
		RawVideoMedia *rawMedia = nullptr;
		auto *source = dynamic_cast<Source *>(e->get());
		if (source == nullptr) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			media = source->getOutputMedia(i);
			if ((media != nullptr) && (media->id == mediaId)) {
				rawMedia = dynamic_cast<RawVideoMedia *>(media);
				found = true;
				break;
			}
		}
		if (found && rawMedia != nullptr) {
			std::unique_ptr<VideoScalerWrapper> wrapper;
			try {
				wrapper = make_unique<VideoScalerWrapper>(
					this, params, listener);
			} catch (const std::bad_alloc &) {
				ULOGE("%s: failed to create the "
				      "video scaler wrapper",
				      __func__);
				ret = -ENOMEM;
				goto exit;
			}
			if (wrapper->getElement() == nullptr) {
				ULOGE("%s: failed to create the video scaler",
				      __func__);
				ret = -EPROTO;
				goto exit;
			}
			ret = mFactory.addVideoScalerForMedia(
				source,
				rawMedia,
				params,
				listener,
				wrapper->getVideoScaler());
			if (ret < 0) {
				ULOG_ERRNO(
					"PipelineFactory"
					"::addVideoScalerForMedia",
					-ret);
				goto exit;
			}
			*retObj = wrapper.release();
			goto exit;
		}
		e++;
	}
	ret = -ENOENT;
exit:
	mMutex.unlock();
	return ret;
}


int Session::createAudioEncoder(unsigned int mediaId,
				const struct aenc_config *params,
				IPdraw::IAudioEncoder::Listener *listener,
				IPdraw::IAudioEncoder **retObj)
{
	int ret = 0;
	bool found = false;

	if (params == nullptr)
		return -EINVAL;
	if (listener == nullptr)
		return -EINVAL;
	if (retObj == nullptr)
		return -EINVAL;

	mMutex.lock();

	auto e = mElements.begin();
	while (e != mElements.end()) {
		Media *media;
		AudioMedia *audioMedia = nullptr;
		auto *source = dynamic_cast<Source *>(e->get());
		if (source == nullptr) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			media = source->getOutputMedia(i);
			if ((media != nullptr) && (media->id == mediaId)) {
				audioMedia = dynamic_cast<AudioMedia *>(media);
				found = true;
				break;
			}
		}
		if (found && audioMedia != nullptr) {
			std::unique_ptr<AudioEncoderWrapper> wrapper;
			try {
				wrapper = make_unique<AudioEncoderWrapper>(
					this, params, listener);
			} catch (const std::bad_alloc &) {
				ULOGE("%s: failed to create the "
				      "video encoder wrapper",
				      __func__);
				ret = -ENOMEM;
				goto exit;
			}
			if (wrapper->getElement() == nullptr) {
				ULOGE("%s: failed to create the audio encoder",
				      __func__);
				ret = -EPROTO;
				goto exit;
			}
			ret = mFactory.addAudioEncoderForMedia(
				source,
				audioMedia,
				params,
				listener,
				wrapper->getAudioEncoder());
			if (ret < 0) {
				ULOG_ERRNO(
					"PipelineFactory"
					"::addAudioEncoderForMedia",
					-ret);
				goto exit;
			}
			*retObj = wrapper.release();
			goto exit;
		}
		e++;
	}
	ret = -ENOENT;
exit:
	mMutex.unlock();
	return ret;
}


void Session::getFriendlyNameSetting(std::string *friendlyName)
{
	mSettings.getFriendlyName(friendlyName);
}


void Session::setFriendlyNameSetting(const std::string &friendlyName)
{
	mSettings.setFriendlyName(friendlyName);
}


void Session::getSerialNumberSetting(std::string *serialNumber)
{
	mSettings.getSerialNumber(serialNumber);
}


void Session::setSerialNumberSetting(const std::string &serialNumber)
{
	mSettings.setSerialNumber(serialNumber);
}


void Session::getSoftwareVersionSetting(std::string *softwareVersion)
{
	mSettings.getSoftwareVersion(softwareVersion);
}


void Session::setSoftwareVersionSetting(const std::string &softwareVersion)
{
	mSettings.setSoftwareVersion(softwareVersion);
}


int Session::dumpPipeline(const std::string &fileName)
{
	return mFactory.dumpPipeline(fileName);
}


/*
 * Internal methods
 */

void Session::asyncElementStateChange(Element *element, Element::State state)
{
	std::unique_lock<std::mutex> lock(mAsyncMutex);
	mElementStateChangeElementArgs.push(element);
	mElementStateChangeStateArgs.push(state);
	int err = pomp_loop_idle_add_with_cookie(
		mLoop, idleElementStateChange, this, this);
	if (err > 0)
		ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


int Session::addMediaToVideoRenderer(unsigned int mediaId,
				     Pdraw::VideoRenderer *renderer)
{
	return mFactory.addMediaToVideoRenderer(mediaId, renderer);
}


int Session::addAllMediaToVideoRenderer(Pdraw::VideoRenderer *renderer)
{
	return mFactory.addAllMediaToVideoRenderer(renderer);
}


int Session::addMediaToAudioRenderer(unsigned int mediaId,
				     Pdraw::AudioRenderer *renderer)
{
	return mFactory.addMediaToAudioRenderer(mediaId, renderer);
}


int Session::addMediaToCodedVideoSink(unsigned int mediaId, Pdraw::Sink *sink)
{
	return mFactory.addMediaToCodedVideoSink(mediaId, sink);
}


int Session::addMediaToRawVideoSink(unsigned int mediaId, Pdraw::Sink *sink)
{
	return mFactory.addMediaToRawVideoSink(mediaId, sink);
}


int Session::addMediaToAudioSink(unsigned int mediaId, Pdraw::Sink *sink)
{
	return mFactory.addMediaToAudioSink(mediaId, sink);
}


int Session::addMediaToMuxer(unsigned int mediaId,
			     Pdraw::Muxer *muxer,
			     const struct pdraw_muxer_media_params *params)
{
	return mFactory.addMediaToMuxer(mediaId, muxer, params);
}


void Session::asyncElementDelete(Element *element)
{
	std::unique_lock<std::mutex> lock(mAsyncMutex);
	mElementDeleteElementArgs.push(element);
	int err = pomp_loop_idle_add_with_cookie(
		mLoop, idleElementDelete, this, this);
	if (err > 0)
		ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


void Session::setState(State state)
{
	std::unique_lock<std::recursive_mutex> lock(mMutex);
	if (state == mState) {
		mMutex.unlock();
		return;
	}

	mState = state;
	ULOGI("state change to %s", stateStr(state));
}


void Session::socketCreated(int fd)
{
	if (mListener != nullptr)
		mListener->onSocketCreated(this, fd);
}


/**
 * Calls from idle functions
 */

void Session::idleElementStateChange(void *userdata)
{
	auto *self = static_cast<Session *>(userdata);
	Element *element = nullptr;
	Element::State state;
	{
		std::unique_lock<std::mutex> lock(self->mAsyncMutex);
		element = self->mElementStateChangeElementArgs.front();
		state = self->mElementStateChangeStateArgs.front();
		self->mElementStateChangeElementArgs.pop();
		self->mElementStateChangeStateArgs.pop();
	}
	ULOG_ERRNO_RETURN_IF(element == nullptr, EINVAL);
	self->onElementStateChanged(element, state);
}


int Session::deleteElement(Element *element)
{
	int ret = 0;
	bool found = false;
	ULOG_ERRNO_RETURN_ERR_IF(element == nullptr, EINVAL);

	{
		std::unique_lock<std::recursive_mutex> lock(mMutex);
		for (auto it = mElements.begin(); it != mElements.end(); ++it) {
			if (it->get() == element) {
				found = true;
				mElements.erase(it);
				break;
			}
		}
	}

	if (!found) {
		ret = -ENOENT;
		ULOGW("%s: element not found in the list", __func__);
	}

	return ret;
}


void Session::idleElementDelete(void *userdata)
{
	int err;
	auto *self = static_cast<Session *>(userdata);
	Element *element = nullptr;
	{
		std::unique_lock<std::mutex> lock(self->mAsyncMutex);
		element = self->mElementDeleteElementArgs.front();
		self->mElementDeleteElementArgs.pop();
	}

	err = self->deleteElement(element);
	if (err < 0)
		ULOG_ERRNO("deleteElement", -err);
}


void Session::callStopResponse(void *userdata)
{
	auto *self = static_cast<Session *>(userdata);
	int status;
	{
		std::unique_lock<std::mutex> lock(self->mAsyncMutex);
		status = self->mStopRespStatusArgs.front();
		self->mStopRespStatusArgs.pop();
	}
	if (self->mListener == nullptr)
		return;
	self->mListener->stopResponse(self, status);
}


void Session::callOnMediaAdded(void *userdata)
{
	auto *self = static_cast<Session *>(userdata);
	struct pdraw_media_info info;
	void *elementUserData = nullptr;
	{
		std::unique_lock<std::mutex> lock(self->mAsyncMutex);
		info = self->mMediaAddedInfoArgs.front();
		self->mMediaAddedInfoArgs.pop();
		elementUserData = self->mMediaAddedElementUserDataArgs.front();
		self->mMediaAddedElementUserDataArgs.pop();
	}
	self->mListener->onMediaAdded(self, &info, elementUserData);
	Media::cleanupMediaInfo(&info);
}


void Session::callOnMediaRemoved(void *userdata)
{
	auto *self = static_cast<Session *>(userdata);
	struct pdraw_media_info info;
	void *elementUserData = nullptr;
	{
		std::unique_lock<std::mutex> lock(self->mAsyncMutex);
		info = self->mMediaRemovedInfoArgs.front();
		self->mMediaRemovedInfoArgs.pop();
		elementUserData =
			self->mMediaRemovedElementUserDataArgs.front();
		self->mMediaRemovedElementUserDataArgs.pop();
	}
	self->mListener->onMediaRemoved(self, &info, elementUserData);
	Media::cleanupMediaInfo(&info);
}


/* Must be called on the loop thread */
void Session::onElementStateChanged(Element *element, Element::State state)
{
	mFactory.onElementStateChanged(element, state);

	if (state == Element::State::STOPPED) {
		bool stopped = true;
		State curState;

		{
			std::unique_lock<std::recursive_mutex> lock(mMutex);
			curState = mState;
			auto e = mElements.begin();
			while (e != mElements.end()) {
				if ((e->get())->getState() !=
				    Element::State::STOPPED) {
					stopped = false;
					break;
				}
				e++;
			}
		}

		asyncElementDelete(element);

		if (stopped && curState == State::STOPPING) {
			setState(State::STOPPED);

			if (mListener != nullptr)
				stopResp(0);
		}
	}
}


/* Must be called on the loop thread */
void Session::onOutputMediaAdded(Source *source,
				 Media *media,
				 void *elementUserData)
{
	ULOGD("onOutputMediaAdded(raw) name=%s", media->getName().c_str());

	mFactory.onOutputMediaAdded(source, media);

	if (mListener != nullptr) {
		struct pdraw_media_info info;
		media->fillMediaInfo(&info);
		if (pthread_self() == mLoopThread) {
			mListener->onMediaAdded(this, &info, elementUserData);
			Media::cleanupMediaInfo(&info);
		} else {
			std::unique_lock<std::mutex> lock(mAsyncMutex);
			mMediaAddedInfoArgs.push(info);
			mMediaAddedElementUserDataArgs.push(elementUserData);
			int err = pomp_loop_idle_add_with_cookie(
				mLoop, callOnMediaAdded, this, this);
			if (err > 0) {
				ULOG_ERRNO("pomp_loop_idle_add_with_cookie",
					   -err);
			}
		}
	}
}


/* Must be called on the loop thread */
void Session::onOutputMediaRemoved(Source *source,
				   Media *media,
				   void *elementUserData)
{
	ULOGD("onOutputMediaRemoved name=%s", media->getName().c_str());

	mFactory.onOutputMediaRemoved(source, media);

	if (mListener != nullptr) {
		struct pdraw_media_info info;
		media->fillMediaInfo(&info);
		std::unique_lock<std::mutex> lock(mAsyncMutex);
		mMediaRemovedInfoArgs.push(info);
		mMediaRemovedElementUserDataArgs.push(elementUserData);
		int err = pomp_loop_idle_add_with_cookie(
			mLoop, callOnMediaRemoved, this, this);
		if (err > 0)
			ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	}
}


void Session::stopResp(int status)
{
	std::unique_lock<std::mutex> lock(mAsyncMutex);
	mStopRespStatusArgs.push(status);
	int err = pomp_loop_idle_add_with_cookie(
		mLoop, callStopResponse, this, this);
	if (err > 0)
		ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


const char *Session::stateStr(State val)
{
	switch (val) {
	case State::STOPPED:
		return "STOPPED";
	case State::READY:
		return "READY";
	case State::STOPPING:
		return "STOPPING";
	default:
		return nullptr;
	}
}


Session::PipelineFactory::PipelineFactory(Session *session) : mSession(session)
{
	return;
}


Session::PipelineFactory::~PipelineFactory()
{
	return;
}


void Session::PipelineFactory::onElementStateChanged(Element *element,
						     Element::State state)
{
	if (state == Element::State::STARTED) {
		auto *rv = dynamic_cast<Pdraw::VideoRenderer *>(element);
		auto *ra = dynamic_cast<Pdraw::AudioRenderer *>(element);
		auto *cvs =
			dynamic_cast<Pdraw::ExternalCodedVideoSink *>(element);
		auto *rvs =
			dynamic_cast<Pdraw::ExternalRawVideoSink *>(element);
		auto *as = dynamic_cast<Pdraw::ExternalAudioSink *>(element);
		if (rv != nullptr) {
			int ret = addAllMediaToVideoRenderer(rv);
			if (ret < 0)
				ULOG_ERRNO("addAllMediaToVideoRenderer", -ret);
		} else if (ra != nullptr) {
			int ret = addAllMediaToAudioRenderer(ra);
			if (ret < 0)
				ULOG_ERRNO("addAllMediaToAudioRenderer", -ret);
		} else if (cvs != nullptr) {
			int ret = addAllMediaToCodedVideoSink(cvs);
			if (ret < 0)
				ULOG_ERRNO("addAllMediaToCodedVideoSink", -ret);
		} else if (rvs != nullptr) {
			int ret = addAllMediaToRawVideoSink(rvs);
			if (ret < 0)
				ULOG_ERRNO("addAllMediaToRawVideoSink", -ret);
		} else if (as != nullptr) {
			int ret = addAllMediaToAudioSink(as);
			if (ret < 0)
				ULOG_ERRNO("addAllMediaToAudioSink", -ret);
		}
	}
}


void Session::PipelineFactory::onOutputMediaAdded(Source *source, Media *media)
{
	const auto *demuxer = dynamic_cast<Pdraw::Demuxer *>(source);
	const auto *vDecoder = dynamic_cast<VideoDecoder *>(source);
	const auto *aDecoder = dynamic_cast<AudioDecoder *>(source);
	auto *codedMedia = dynamic_cast<CodedVideoMedia *>(media);
	auto *rawMedia = dynamic_cast<RawVideoMedia *>(media);
	auto *audioMedia = dynamic_cast<AudioMedia *>(media);
	if ((demuxer != nullptr) && (codedMedia != nullptr)) {
		if (demuxer->getParams()->autodecoding_mode ==
		    PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL) {
			int ret = addVideoDecoderForMedia(source, codedMedia);
			if (ret < 0)
				ULOG_ERRNO("addVideoDecoderForMedia", -ret);
		}
		int ret = addMediaToAllToCodedVideoSinks(source, codedMedia);
		if (ret < 0)
			ULOG_ERRNO("addMediaToAllToCodedVideoSinks", -ret);
	} else if ((demuxer != nullptr) && (audioMedia != nullptr)) {
		if (demuxer->getParams()->autodecoding_mode ==
		    PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL) {
			int ret = addAudioDecoderForMedia(source, audioMedia);
			if (ret < 0)
				ULOG_ERRNO("addAudioDecoderForMedia", -ret);
		}

	} else if ((vDecoder != nullptr) && (rawMedia != nullptr)) {
		int ret = addMediaToAllVideoRenderers(source, rawMedia);
		if (ret < 0)
			ULOG_ERRNO("addMediaToAllVideoRenderers", -ret);
		ret = addMediaToAllToRawVideoSinks(source, rawMedia);
		if (ret < 0)
			ULOG_ERRNO("addMediaToAllToRawVideoSinks", -ret);
	} else if ((aDecoder != nullptr) && (audioMedia != nullptr)) {
		int ret = addMediaToAllAudioRenderers(source, audioMedia);
		if (ret < 0)
			ULOG_ERRNO("addMediaToAllAudioRenderers", -ret);
		ret = addMediaToAllToAudioSinks(source, audioMedia);
		if (ret < 0)
			ULOG_ERRNO("addMediaToAllToAudioSinks", -ret);
	}
}


void Session::PipelineFactory::onOutputMediaRemoved(const Source *source,
						    const Media *media) const
{
	PDRAW_UNUSED(source);
	PDRAW_UNUSED(media);

	return;
}


int Session::PipelineFactory::dumpPipeline(const std::string &fileName)
{
	int ret;
	FILE *f;

	f = fopen(fileName.c_str(), "w");
	if (f == nullptr) {
		ret = -errno;
		ULOG_ERRNO("fopen", -ret);
		return ret;
	}

	fprintf(f, "digraph {\n");
	fprintf(f, "\tnode [margin=0.2,fontsize=12];\n");

	mSession->mMutex.lock();

	/* First pass: list the elements with their sink and source medias */
	auto e = mSession->mElements.begin();
	while (e != mSession->mElements.end()) {
		unsigned int elmId = (e->get())->getId();
		const char *elmName = (e->get())->getName().c_str();
		fprintf(f, "\te%u [shape=record,label=\"", elmId);

		/* Element input medias */
		auto *sink = dynamic_cast<Sink *>(e->get());
		if (sink != nullptr) {
			unsigned int count = sink->getInputMediaCount();
			if (count > 0)
				fprintf(f, "{ ");
			for (unsigned int i = 0; i < count; i++) {
				const Media *media = sink->getInputMedia(i);
				if (media == nullptr)
					continue;
				fprintf(f,
					"%s<e%um%u> %s",
					(i > 0) ? " | " : "",
					elmId,
					media->id,
					media->getName().c_str());
			}
			if (count > 0)
				fprintf(f, " } | ");
		}

		/* Element name */
		fprintf(f, "<e%u> %s", elmId, elmName);

		/* Element output medias */
		auto *source = dynamic_cast<Source *>(e->get());
		if (source != nullptr) {
			unsigned int count = source->getOutputMediaCount();
			if (count > 0)
				fprintf(f, " | { ");
			for (unsigned int i = 0; i < count; i++) {
				const Media *media = source->getOutputMedia(i);
				if (media == nullptr)
					continue;
				fprintf(f,
					"%s<e%um%u> %s",
					(i > 0) ? " | " : "",
					elmId,
					media->id,
					media->getName().c_str());
			}
			if (count > 0)
				fprintf(f, " }");
		}

		fprintf(f, "\"];\n");
		e++;
	}

	/* Second pass: list the links between sources and sinks */
	e = mSession->mElements.begin();
	while (e != mSession->mElements.end()) {
		unsigned int dstElmId = (e->get())->getId();

		/* Element input medias */
		auto *sink = dynamic_cast<Sink *>(e->get());
		if (sink != nullptr) {
			unsigned int count = sink->getInputMediaCount();
			for (unsigned int i = 0; i < count; i++) {
				const Media *media = sink->getInputMedia(i);
				auto e2 = mSession->mElements.begin();
				while (e2 != mSession->mElements.end()) {
					auto *source = dynamic_cast<Source *>(
						e2->get());
					if (source != nullptr) {
						if (source->findOutputMedia(
							    media) == nullptr) {
							e2++;
							continue;
						}
						unsigned int srcElmId =
							(*e2)->getId();
						fprintf(f,
							"\te%u:e%um%u -> "
							"e%u:e%um%u;\n",
							srcElmId,
							srcElmId,
							media->id,
							dstElmId,
							dstElmId,
							media->id);
						break;
					}
					e2++;
				}
			}
		}

		e++;
	}

	mSession->mMutex.unlock();

	fprintf(f, "}");
	fclose(f);

	ULOGI("pipeline dumped to file '%s'", fileName.c_str());

	return 0;
}


int Session::PipelineFactory::addVideoDecoderForMedia(Source *source,
						      CodedVideoMedia *media)
{
	int ret;
	std::unique_ptr<VideoDecoder> decoder;

	try {
		decoder =
			make_unique<VideoDecoder>(mSession, mSession, mSession);
	} catch (const std::bad_alloc &) {
		ULOGE("decoder creation failed");
		return -ENOMEM;
	}

	ret = decoder->addInputMedia(media);
	if (ret < 0) {
		if (ret == -ENOSYS)
			ret = 0;
		else
			ULOG_ERRNO("decoder->addInputMedia", -ret);
		return ret;
	}
	mSession->mMutex.lock();
	mSession->mElements.push_back(std::move(decoder));
	const auto &elementPtr = mSession->mElements.back();
	auto *decoderPtr = dynamic_cast<VideoDecoder *>(elementPtr.get());
	if (!decoderPtr) {
		mSession->mMutex.unlock();
		ULOGE("element is not a VideoDecoder");
		return -EPROTO;
	}
	mSession->mMutex.unlock();
	ret = decoderPtr->start();
	if (ret < 0) {
		ULOG_ERRNO("decoder->start", -ret);
		return ret;
	}
	Channel *channel = decoderPtr->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get decoder input channel");
		return -EPROTO;
	}
	ret = source->addOutputChannel(media, channel);
	if (ret < 0) {
		ULOG_ERRNO("source->addOutputChannel", -ret);
		return ret;
	}
	/* Force a resync after linking the elements; this allows a H.264
	 * decoder to start on an IDR frame for example */
	decoderPtr->resync();

	return 0;
}


int Session::PipelineFactory::addAudioDecoderForMedia(Source *source,
						      AudioMedia *media)
{
	int ret;
	std::unique_ptr<AudioDecoder> decoder;

	try {
		decoder =
			make_unique<AudioDecoder>(mSession, mSession, mSession);
	} catch (const std::bad_alloc &) {
		ULOGE("decoder creation failed");
		return -ENOMEM;
	}

	ret = decoder->addInputMedia(media);
	if (ret < 0) {
		if (ret == -ENOSYS)
			ret = 0;
		else
			ULOG_ERRNO("decoder->addInputMedia", -ret);
		return ret;
	}
	mSession->mMutex.lock();
	mSession->mElements.push_back(std::move(decoder));
	const auto &elementPtr = mSession->mElements.back();
	auto *decoderPtr = dynamic_cast<AudioDecoder *>(elementPtr.get());
	if (!decoderPtr) {
		mSession->mMutex.unlock();
		ULOGE("element is not an AudioDecoder");
		return -EPROTO;
	}
	mSession->mMutex.unlock();
	ret = decoderPtr->start();
	if (ret < 0) {
		ULOG_ERRNO("decoder->start", -ret);
		return ret;
	}
	Channel *channel = decoderPtr->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get decoder input channel");
		return -EPROTO;
	}
	ret = source->addOutputChannel(media, channel);
	if (ret < 0) {
		ULOG_ERRNO("source->addOutputChannel", -ret);
		return ret;
	}

	return 0;
}


int Session::PipelineFactory::addVideoEncoderForMedia(
	Source *source,
	RawVideoMedia *media,
	const struct venc_config *params,
	IPdraw::IVideoEncoder::Listener *listener,
	VideoEncoder *encoder)
{
	int ret;
	bool allocated = false;
	Channel *channel = nullptr;
	std::unique_ptr<VideoEncoder> encoderPtr;

	if (encoder == nullptr) {
		try {
			encoderPtr = make_unique<VideoEncoder>(mSession,
							       mSession,
							       mSession,
							       listener,
							       nullptr,
							       params);
		} catch (const std::bad_alloc &) {
			ULOGE("encoder creation failed");
			return -ENOMEM;
		}
		encoder = encoderPtr.get();
		allocated = true;
	}

	mSession->mMutex.lock();
	if (!allocated)
		mSession->mElements.push_back(
			std::unique_ptr<VideoEncoder>(encoder));
	else
		mSession->mElements.push_back(std::move(encoderPtr));
	mSession->mMutex.unlock();

	ret = encoder->addInputMedia(media);
	if (ret < 0) {
		ULOG_ERRNO("VideoEncoder::addInputMedia", -ret);
		goto error;
	}
	ret = encoder->start();
	if (ret < 0) {
		ULOG_ERRNO("VideoEncoder::start", -ret);
		goto error;
	}
	channel = encoder->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get encoder input channel");
		ret = -EPROTO;
		goto error;
	}
	ret = source->addOutputChannel(media, channel);
	if (ret < 0) {
		ULOG_ERRNO("Source::addOutputChannel", -ret);
		goto error;
	}

	return 0;

error:
	if (encoder != nullptr) {
		if (channel != nullptr)
			source->removeOutputChannel(media, channel);
		if (allocated)
			mSession->deleteElement(encoder);
	}
	return ret;
}


int Session::PipelineFactory::addVideoScalerForMedia(
	Source *source,
	RawVideoMedia *media,
	const struct vscale_config *params,
	IPdraw::IVideoScaler::Listener *listener,
	Pdraw::VideoScaler *scaler)
{
	int ret;
	bool allocated = false;
	Channel *channel = nullptr;
	std::unique_ptr<VideoScaler> scalerPtr;

	if (scaler == nullptr) {
		try {
			scalerPtr = make_unique<VideoScaler>(mSession,
							     mSession,
							     mSession,
							     listener,
							     nullptr,
							     params);
		} catch (const std::bad_alloc &) {
			ULOGE("scaler creation failed");
			return -ENOMEM;
		}
		scaler = scalerPtr.get();
		allocated = true;
	}

	mSession->mMutex.lock();
	if (!allocated)
		mSession->mElements.push_back(
			std::unique_ptr<VideoScaler>(scaler));
	else
		mSession->mElements.push_back(std::move(scalerPtr));
	mSession->mMutex.unlock();

	ret = scaler->addInputMedia(media);
	if (ret < 0) {
		ULOG_ERRNO("scaler->addInputMedia", -ret);
		goto error;
	}
	ret = scaler->start();
	if (ret < 0) {
		ULOG_ERRNO("scaler->start", -ret);
		goto error;
	}
	channel = scaler->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get scaler input channel");
		ret = -EPROTO;
		goto error;
	}
	ret = source->addOutputChannel(media, channel);
	if (ret < 0) {
		ULOG_ERRNO("source->addOutputChannel", -ret);
		goto error;
	}

	return 0;

error:
	if (scaler != nullptr) {
		if (channel != nullptr)
			source->removeOutputChannel(media, channel);
		if (allocated)
			mSession->deleteElement(scaler);
	}
	return ret;
}


int Session::PipelineFactory::addAudioEncoderForMedia(
	Source *source,
	AudioMedia *media,
	const struct aenc_config *params,
	IPdraw::IAudioEncoder::Listener *listener,
	AudioEncoder *encoder)
{
	int ret;
	bool allocated = false;
	Channel *channel = nullptr;
	std::unique_ptr<AudioEncoder> encoderPtr;

	if (encoder == nullptr) {
		try {
			encoderPtr = make_unique<AudioEncoder>(mSession,
							       mSession,
							       mSession,
							       listener,
							       nullptr,
							       params);
		} catch (const std::bad_alloc &) {
			ULOGE("encoder creation failed");
			return -ENOMEM;
		}
		encoder = encoderPtr.get();
		allocated = true;
	}

	mSession->mMutex.lock();
	if (!allocated)
		mSession->mElements.push_back(
			std::unique_ptr<AudioEncoder>(encoder));
	else
		mSession->mElements.push_back(std::move(encoderPtr));
	mSession->mMutex.unlock();

	ret = encoder->addInputMedia(media);
	if (ret < 0) {
		ULOG_ERRNO("AudioEncoder::addInputMedia", -ret);
		goto error;
	}
	ret = encoder->start();
	if (ret < 0) {
		ULOG_ERRNO("AudioEncoder::start", -ret);
		goto error;
	}
	channel = encoder->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get encoder input channel");
		ret = -EPROTO;
		goto error;
	}
	ret = source->addOutputChannel(media, channel);
	if (ret < 0) {
		ULOG_ERRNO("Source::addOutputChannel", -ret);
		goto error;
	}

	return 0;

error:
	if (encoder != nullptr) {
		if (channel != nullptr)
			source->removeOutputChannel(media, channel);
		if (allocated)
			mSession->deleteElement(encoder);
	}
	return ret;
}


int Session::PipelineFactory::addMediaToVideoRenderer(
	Source *source,
	RawVideoMedia *media,
	Pdraw::VideoRenderer *renderer) const
{
	int ret;

	ret = renderer->addInputMedia(media);
	if ((ret == -EEXIST) || (ret == -EPERM)) {
		return 0;
	} else if (ret < 0) {
		ULOG_ERRNO("renderer->addInputMedia", -ret);
		return ret;
	}
	Channel *channel = renderer->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get renderer input channel");
		return -EPROTO;
	}
	ret = source->addOutputChannel(media, channel);
	if (ret < 0) {
		ULOG_ERRNO("source->addOutputChannel", -ret);
		return ret;
	}
	return 0;
}


int Session::PipelineFactory::addMediaToVideoRenderer(
	unsigned int mediaId,
	Pdraw::VideoRenderer *renderer)
{
	int ret;
	bool found = false;

	mSession->mMutex.lock();
	for (auto &e : mSession->mElements) {
		auto *source = dynamic_cast<Source *>(e.get());
		if (source == nullptr)
			continue;
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			auto *media = dynamic_cast<RawVideoMedia *>(m);
			if (media == nullptr)
				continue;
			if (media->id != mediaId)
				continue;
			ret = addMediaToVideoRenderer(source, media, renderer);
			if (ret < 0)
				ULOG_ERRNO("addMediaToVideoRenderer", -ret);
			found = true;
			break;
		}
		if (found)
			break;
	}
	mSession->mMutex.unlock();

	return 0;
}


int Session::PipelineFactory::addMediaToAllVideoRenderers(Source *source,
							  RawVideoMedia *media)
{
	int ret = 0;

	mSession->mMutex.lock();
	auto e = mSession->mElements.begin();
	while (e != mSession->mElements.end() && ret == 0) {
		auto *r = dynamic_cast<Pdraw::VideoRenderer *>(e->get());
		e++;
		if (r == nullptr)
			continue;
		ret = addMediaToVideoRenderer(source, media, r);
	}
	mSession->mMutex.unlock();

	return ret;
}


int Session::PipelineFactory::addAllMediaToVideoRenderer(
	Pdraw::VideoRenderer *renderer)
{
	int ret;

	mSession->mMutex.lock();
	for (auto &elem : mSession->mElements) {
		auto *source = dynamic_cast<Source *>(elem.get());
		if (source == nullptr)
			continue;
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			auto *media = dynamic_cast<RawVideoMedia *>(m);
			if (media == nullptr)
				continue;
			ret = addMediaToVideoRenderer(source, media, renderer);
			if (ret < 0)
				ULOG_ERRNO("addMediaToVideoRenderer", -ret);
		}
	}
	mSession->mMutex.unlock();

	return 0;
}


int Session::PipelineFactory::addMediaToAudioRenderer(
	Source *source,
	AudioMedia *media,
	Pdraw::AudioRenderer *renderer) const
{
	int ret;

	ret = renderer->addInputMedia(media);
	if ((ret == -EEXIST) || (ret == -EPERM)) {
		return 0;
	} else if (ret < 0) {
		ULOG_ERRNO("AudioRenderer::addInputMedia", -ret);
		return ret;
	}
	Channel *channel = renderer->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get renderer input channel");
		return -EPROTO;
	}
	ret = source->addOutputChannel(media, channel);
	if (ret < 0) {
		ULOG_ERRNO("source->addOutputChannel", -ret);
		return ret;
	}
	return 0;
}


int Session::PipelineFactory::addMediaToAudioRenderer(
	unsigned int mediaId,
	Pdraw::AudioRenderer *renderer)
{
	int ret;
	bool found = false;

	mSession->mMutex.lock();
	for (auto &e : mSession->mElements) {
		auto *source = dynamic_cast<Source *>(e.get());
		if (source == nullptr)
			continue;
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			auto *media = dynamic_cast<AudioMedia *>(m);
			if (media == nullptr)
				continue;
			if (media->id != mediaId)
				continue;
			ret = addMediaToAudioRenderer(source, media, renderer);
			if (ret < 0)
				ULOG_ERRNO("addMediaToAudioRenderer", -ret);
			found = true;
			break;
		}
		if (found)
			break;
	}
	mSession->mMutex.unlock();

	return 0;
}


int Session::PipelineFactory::addMediaToAllAudioRenderers(Source *source,
							  AudioMedia *media)
{
	int ret = 0;

	mSession->mMutex.lock();
	auto e = mSession->mElements.begin();
	while (e != mSession->mElements.end() && ret == 0) {
		auto *r = dynamic_cast<Pdraw::AudioRenderer *>(e->get());
		e++;
		if (r == nullptr)
			continue;
		ret = addMediaToAudioRenderer(source, media, r);
	}
	mSession->mMutex.unlock();

	return ret;
}


int Session::PipelineFactory::addAllMediaToAudioRenderer(
	Pdraw::AudioRenderer *renderer)
{
	int ret;

	mSession->mMutex.lock();
	for (auto &elem : mSession->mElements) {
		auto *source = dynamic_cast<Source *>(elem.get());
		if (source == nullptr)
			continue;
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			auto *media = dynamic_cast<AudioMedia *>(m);
			if (media == nullptr)
				continue;
			ret = addMediaToAudioRenderer(source, media, renderer);
			if (ret < 0)
				ULOG_ERRNO("addMediaToAudioRenderer", -ret);
		}
	}
	mSession->mMutex.unlock();

	return 0;
}


int Session::PipelineFactory::addMediaToCodedVideoSink(Source *source,
						       Media *media,
						       Pdraw::Sink *sink) const
{
	int ret;

	ret = sink->addInputMedia(media);
	if ((ret == -EEXIST) || (ret == -EPERM)) {
		return 0;
	} else if (ret < 0) {
		ULOG_ERRNO("CodedVideoSinkWrapper::addInputMedia", -ret);
		return ret;
	}
	Channel *channel = sink->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get sink input channel");
		return -EPROTO;
	}
	ret = source->addOutputChannel(media, channel);
	if (ret < 0) {
		ULOG_ERRNO("source->addOutputChannel", -ret);
		return ret;
	}
	return 0;
}


int Session::PipelineFactory::addMediaToCodedVideoSink(unsigned int mediaId,
						       Pdraw::Sink *sink)
{
	int ret;
	bool found = false;

	mSession->mMutex.lock();
	for (auto &e : mSession->mElements) {
		auto *source = dynamic_cast<Source *>(e.get());
		if (source == nullptr)
			continue;
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			auto *media = dynamic_cast<CodedVideoMedia *>(m);
			if (media == nullptr)
				continue;
			if (media->id != mediaId)
				continue;
			ret = addMediaToCodedVideoSink(source, media, sink);
			if (ret < 0)
				ULOG_ERRNO("addMediaToCodedVideoSink", -ret);
			found = true;
			break;
		}
		if (found)
			break;
	}
	mSession->mMutex.unlock();

	return 0;
}


int Session::PipelineFactory::addMediaToAllToCodedVideoSinks(
	Source *source,
	CodedVideoMedia *media)
{
	int ret = 0;

	mSession->mMutex.lock();
	auto e = mSession->mElements.begin();
	while (e != mSession->mElements.end() && ret == 0) {
		auto *rs =
			dynamic_cast<Pdraw::ExternalCodedVideoSink *>(e->get());
		e++;
		if (rs == nullptr)
			continue;
		ret = addMediaToCodedVideoSink(source, media, rs);
	}
	mSession->mMutex.unlock();

	return ret;
}


int Session::PipelineFactory::addAllMediaToCodedVideoSink(
	Pdraw::ExternalCodedVideoSink *sink)
{
	int ret;

	mSession->mMutex.lock();
	for (auto &elem : mSession->mElements) {
		auto *source = dynamic_cast<Source *>(elem.get());
		if (source == nullptr)
			continue;
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			auto *media = dynamic_cast<CodedVideoMedia *>(m);
			if (media == nullptr)
				continue;
			ret = addMediaToCodedVideoSink(source, media, sink);
			if (ret < 0)
				ULOG_ERRNO("addMediaToCodedVideoSink", -ret);
		}
	}
	mSession->mMutex.unlock();

	return 0;
}


int Session::PipelineFactory::addMediaToRawVideoSink(Source *source,
						     Media *media,
						     Pdraw::Sink *sink) const
{
	int ret;

	ret = sink->addInputMedia(media);
	if ((ret == -EEXIST) || (ret == -EPERM)) {
		return 0;
	} else if (ret < 0) {
		ULOG_ERRNO("RawVideoSinkWrapper::addInputMedia", -ret);
		return ret;
	}
	Channel *channel = sink->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get sink input channel");
		return -EPROTO;
	}
	ret = source->addOutputChannel(media, channel);
	if (ret < 0) {
		ULOG_ERRNO("source->addOutputChannel", -ret);
		return ret;
	}
	return 0;
}


int Session::PipelineFactory::addMediaToRawVideoSink(unsigned int mediaId,
						     Pdraw::Sink *sink)
{
	int ret;
	bool found = false;

	mSession->mMutex.lock();
	for (auto &e : mSession->mElements) {
		auto *source = dynamic_cast<Source *>(e.get());
		if (source == nullptr)
			continue;
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			auto *media = dynamic_cast<RawVideoMedia *>(m);
			if (media == nullptr)
				continue;
			if (media->id != mediaId)
				continue;
			ret = addMediaToRawVideoSink(source, media, sink);
			if (ret < 0)
				ULOG_ERRNO("addMediaToRawVideoSink", -ret);
			found = true;
			break;
		}
		if (found)
			break;
	}
	mSession->mMutex.unlock();

	return 0;
}


int Session::PipelineFactory::addMediaToAllToRawVideoSinks(Source *source,
							   RawVideoMedia *media)
{
	int ret = 0;

	mSession->mMutex.lock();
	auto e = mSession->mElements.begin();
	while (e != mSession->mElements.end() && ret == 0) {
		auto *rs =
			dynamic_cast<Pdraw::ExternalRawVideoSink *>(e->get());
		e++;
		if (rs == nullptr)
			continue;
		ret = addMediaToRawVideoSink(source, media, rs);
	}
	mSession->mMutex.unlock();

	return ret;
}


int Session::PipelineFactory::addAllMediaToRawVideoSink(
	Pdraw::ExternalRawVideoSink *sink)
{
	int ret;

	mSession->mMutex.lock();
	for (auto &e : mSession->mElements) {
		auto *source = dynamic_cast<Source *>(e.get());
		if (source == nullptr)
			continue;
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			auto *media = dynamic_cast<RawVideoMedia *>(m);
			if (media == nullptr)
				continue;
			ret = addMediaToRawVideoSink(source, media, sink);
			if (ret < 0)
				ULOG_ERRNO("addMediaToRawVideoSink", -ret);
		}
	}
	mSession->mMutex.unlock();

	return 0;
}


int Session::PipelineFactory::addMediaToAudioSink(Source *source,
						  Media *media,
						  Pdraw::Sink *sink) const
{
	int ret;

	ret = sink->addInputMedia(media);
	if ((ret == -EEXIST) || (ret == -EPERM)) {
		return 0;
	} else if (ret < 0) {
		ULOG_ERRNO("AudioSinkWrapper::addInputMedia", -ret);
		return ret;
	}
	Channel *channel = sink->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get sink input channel");
		return -EPROTO;
	}
	ret = source->addOutputChannel(media, channel);
	if (ret < 0) {
		ULOG_ERRNO("source->addOutputChannel", -ret);
		return ret;
	}
	return 0;
}


int Session::PipelineFactory::addMediaToAudioSink(unsigned int mediaId,
						  Pdraw::Sink *sink)
{
	int ret;
	bool found = false;

	mSession->mMutex.lock();
	for (auto &e : mSession->mElements) {
		auto *source = dynamic_cast<Source *>(e.get());
		if (source == nullptr)
			continue;
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			auto *media = dynamic_cast<AudioMedia *>(m);
			if (media == nullptr)
				continue;
			if (media->id != mediaId)
				continue;
			ret = addMediaToAudioSink(source, media, sink);
			if (ret < 0)
				ULOG_ERRNO("addMediaToAudioSink", -ret);
			found = true;
			break;
		}
		if (found)
			break;
	}
	mSession->mMutex.unlock();

	return 0;
}


int Session::PipelineFactory::addMediaToAllToAudioSinks(Source *source,
							AudioMedia *media)
{
	int ret = 0;

	mSession->mMutex.lock();
	auto e = mSession->mElements.begin();
	while (e != mSession->mElements.end() && ret == 0) {
		auto *rs = dynamic_cast<Pdraw::ExternalAudioSink *>(e->get());
		e++;
		if (rs == nullptr)
			continue;
		ret = addMediaToAudioSink(source, media, rs);
	}
	mSession->mMutex.unlock();

	return ret;
}


int Session::PipelineFactory::addAllMediaToAudioSink(
	Pdraw::ExternalAudioSink *sink)
{
	int ret;

	mSession->mMutex.lock();
	for (auto &e : mSession->mElements) {
		auto *source = dynamic_cast<Source *>(e.get());
		if (source == nullptr)
			continue;
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			auto *media = dynamic_cast<AudioMedia *>(m);
			if (media == nullptr)
				continue;
			ret = addMediaToAudioSink(source, media, sink);
			if (ret < 0)
				ULOG_ERRNO("addMediaToAudioSink", -ret);
		}
	}
	mSession->mMutex.unlock();

	return 0;
}


int Session::PipelineFactory::addMediaToMuxer(
	Source *source,
	Media *media,
	Pdraw::Muxer *muxer,
	const struct pdraw_muxer_media_params *params) const
{
	int ret;

	ret = muxer->addInputMedia(media, params);
	if (ret < 0) {
		ULOG_ERRNO("Muxer::addInputMedia", -ret);
		return ret;
	}
	Channel *channel = muxer->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get muxer input channel");
		ret = -EPROTO;
		return ret;
	}
	ret = source->addOutputChannel(media, channel);
	if (ret < 0) {
		ULOG_ERRNO("Source::addOutputChannel", -ret);
		return ret;
	}
	return 0;
}


int Session::PipelineFactory::addMediaToMuxer(
	unsigned int mediaId,
	Pdraw::Muxer *muxer,
	const struct pdraw_muxer_media_params *params)
{
	int ret;
	Source *source = nullptr;
	Media *media = nullptr;
	const CodedVideoMedia *codedMedia = nullptr;
	const RawVideoMedia *rawMedia = nullptr;
	const AudioMedia *audioMedia = nullptr;
	bool found = false;

	mSession->mMutex.lock();
	for (auto &e : mSession->mElements) {
		source = dynamic_cast<Source *>(e.get());
		if (source == nullptr)
			continue;
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			media = source->getOutputMedia(i);
			codedMedia = dynamic_cast<CodedVideoMedia *>(media);
			rawMedia = dynamic_cast<RawVideoMedia *>(media);
			audioMedia = dynamic_cast<AudioMedia *>(media);
			if (((codedMedia != nullptr) &&
			     (codedMedia->id == mediaId)) ||
			    ((rawMedia != nullptr) &&
			     (rawMedia->id == mediaId)) ||
			    ((audioMedia != nullptr) &&
			     (audioMedia->id == mediaId))) {
				found = true;
				break;
			}
		}
		if (found)
			break;
	}

	if ((!found) || (source == nullptr) ||
	    (codedMedia == nullptr && rawMedia == nullptr &&
	     audioMedia == nullptr)) {
		mSession->mMutex.unlock();
		return -ENOENT;
	}

	ret = addMediaToMuxer(source, media, muxer, params);
	if (ret < 0)
		ULOG_ERRNO("addMediaToMuxer", -ret);

	mSession->mMutex.unlock();

	return ret;
}


} /* namespace Pdraw */
