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
	IPdraw *pdraw = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(loop == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pdraw = new Session(loop, listener);
	if (pdraw == nullptr) {
		ULOGE("failed to create pdraw instance");
		return -EPROTO;
	}
	*retObj = pdraw;
	return 0;
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


const char *pdrawVideoTypeStr(enum pdraw_video_type val)
{
	return pdraw_videoTypeStr(val);
}


enum pdraw_video_type pdrawVideoTypeFromStr(const char *val)
{
	return pdraw_videoTypeFromStr(val);
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


int pdrawAlsaSourceGetCapabilities(const std::string &address,
				   struct pdraw_alsa_source_caps *caps)
{
#ifdef PDRAW_USE_ALSA
	return Pdraw::AlsaSource::getCapabilities(address, caps);
#else
	return -ENOSYS;
#endif
}


Session::Session(struct pomp_loop *loop, IPdraw::Listener *listener) :
		mFactory(this), mListener(listener), mState(STOPPED),
		mLoop(loop)

{
	int res;
	pthread_mutexattr_t attr;
	bool attr_created = false;
	bool mutex_created = false;

	mLoopThread = pthread_self();

	res = pthread_mutexattr_init(&attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutexattr_init", res);
		goto error;
	}
	attr_created = true;

	res = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutexattr_settype", res);
		goto error;
	}

	res = pthread_mutex_init(&mMutex, &attr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init(mMutex)", res);
		goto error;
	}
	pthread_mutexattr_destroy(&attr);
	mutex_created = true;

	res = pthread_mutex_init(&mAsyncMutex, nullptr);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init(mAsyncMutex)", res);
		goto error;
	}

	setState(READY);
	return;

error:
	if (mutex_created)
		pthread_mutex_destroy(&mMutex);
	else if (attr_created)
		pthread_mutexattr_destroy(&attr);
}


Session::~Session(void)
{
	if (mState != STOPPED)
		ULOGW("destroying while instance is still running");

	pthread_mutex_lock(&mMutex);
	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		delete *e;
		e++;
	}
	mElements.clear();
	pthread_mutex_unlock(&mMutex);

	/* Remove any leftover idle callbacks */
	if (mLoop != nullptr) {
		int err = pomp_loop_idle_remove_by_cookie(mLoop, this);
		if (err > 0)
			ULOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);
	}

	pthread_mutex_lock(&mAsyncMutex);
	while (!mMediaAddedInfoArgs.empty()) {
		struct pdraw_media_info info = mMediaAddedInfoArgs.front();
		mMediaAddedInfoArgs.pop();
		Media::cleanupMediaInfo(&info);
	}
	while (!mMediaRemovedInfoArgs.empty()) {
		struct pdraw_media_info info = mMediaRemovedInfoArgs.front();
		mMediaRemovedInfoArgs.pop();
		Media::cleanupMediaInfo(&info);
	}
	pthread_mutex_unlock(&mAsyncMutex);

	pthread_mutex_destroy(&mMutex);
	pthread_mutex_destroy(&mAsyncMutex);
}


/*
 * API methods
 */

int Session::stop(void)
{
	int ret;
	bool stopped = true;
	std::vector<Element *>::iterator e;

	if (mState == STOPPING) {
		/* Return without calling the stopResponse() function */
		ULOGI("%s: already in %s state, nothing to do",
		      __func__,
		      stateStr(mState));
		return 0;
	}

	if (mState == STOPPED) {
		/* Call the stopResponse() function with OK status */
		ULOGI("%s: state is %s, nothing to do",
		      __func__,
		      stateStr(mState));
		ret = 0;
		goto already_stopped;
	}

	if (mState != READY) {
		ULOGE("%s: invalid state (%s)", __func__, stateStr(mState));
		return -EPROTO;
	}

	setState(STOPPING);

	pthread_mutex_lock(&mMutex);
	e = mElements.begin();
	while (e != mElements.end()) {
		if ((*e)->getState() != Element::State::STOPPED) {
			stopped = false;
			int err = (*e)->stop();
			if (err < 0)
				ULOG_ERRNO("element->stop", -err);
		}
		e++;
	}
	pthread_mutex_unlock(&mMutex);

	if (stopped) {
		/* Call the stopResponse() function with OK status */
		ULOGI("%s: all elements are stopped, closing", __func__);
		setState(STOPPED);
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
	VideoRendererWrapper *renderer = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(renderPos == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);
	if (mState == STOPPING || mState == STOPPED) {
		ULOGE("renderer creation refused in %s state",
		      stateStr(mState));
		pthread_mutex_unlock(&mMutex);
		return -EPROTO;
	}

	renderer = new VideoRendererWrapper(
		this, mediaId, renderPos, params, listener);
	if (renderer == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the video renderer wrapper",
		      __func__);
		return -ENOMEM;
	}
	if (renderer->getElement() == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the video renderer", __func__);
		delete renderer;
		return -EPROTO;
	}

	mElements.push_back(renderer->getElement());
	pthread_mutex_unlock(&mMutex);

	res = renderer->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("renderer->start", -res);
		goto error;
	}

	*retObj = renderer;

	return 0;

error:
	delete renderer;
	return res;
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
	DemuxerWrapper *demuxer = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);
	if (mState == STOPPING || mState == STOPPED) {
		ULOGE("demuxer creation refused in %s state", stateStr(mState));
		pthread_mutex_unlock(&mMutex);
		return -EPROTO;
	}

	demuxer = new DemuxerWrapper(this,
				     localAddr,
				     localStreamPort,
				     localControlPort,
				     remoteAddr,
				     remoteStreamPort,
				     remoteControlPort,
				     params,
				     listener);
	if (demuxer == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the demuxer wrapper", __func__);
		return -ENOMEM;
	}
	if (demuxer->getElement() == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the demuxer", __func__);
		delete demuxer;
		return -EPROTO;
	}

	mElements.push_back(demuxer->getElement());
	pthread_mutex_unlock(&mMutex);

	res = demuxer->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("demuxer->start", -res);
		goto error;
	}

	*retObj = demuxer;

	/* Waiting for the asynchronous open; openResponse()
	 * will be called when it's done */
	return 0;

error:
	delete demuxer;
	return res;
}


int Session::createDemuxer(const std::string &url,
			   struct mux_ctx *mux,
			   const struct pdraw_demuxer_params *params,
			   IPdraw::IDemuxer::Listener *listener,
			   IPdraw::IDemuxer **retObj)
{
	int res;
	DemuxerWrapper *demuxer = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(url.length() == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);
	if (mState == STOPPING || mState == STOPPED) {
		ULOGE("demuxer creation refused in %s state", stateStr(mState));
		pthread_mutex_unlock(&mMutex);
		return -EPROTO;
	}

	demuxer = new DemuxerWrapper(this, url, mux, params, listener);
	if (demuxer == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the demuxer wrapper", __func__);
		return -ENOMEM;
	}
	if (demuxer->getElement() == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the demuxer", __func__);
		delete demuxer;
		return -EPROTO;
	}

	mElements.push_back(demuxer->getElement());
	pthread_mutex_unlock(&mMutex);

	res = demuxer->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("demuxer->start", -res);
		goto error;
	}

	*retObj = demuxer;

	/* Waiting for the asynchronous open; openResponse()
	 * will be called when it's done */
	return 0;

error:
	delete demuxer;
	return res;
}


int Session::createMuxer(const std::string &url,
			 const struct pdraw_muxer_params *params,
			 IPdraw::IMuxer::Listener *listener,
			 IPdraw::IMuxer **retObj)
{
	int res;
	MuxerWrapper *muxer = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(url.length() == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);
	if (mState == STOPPING || mState == STOPPED) {
		ULOGE("muxer creation refused in %s state", stateStr(mState));
		pthread_mutex_unlock(&mMutex);
		return -EPROTO;
	}

	muxer = new MuxerWrapper(this, url, params, listener);
	if (muxer == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the muxer wrapper", __func__);
		return -ENOMEM;
	}
	if (muxer->getElement() == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the muxer", __func__);
		delete muxer;
		return -EPROTO;
	}

	mElements.push_back(muxer->getElement());
	pthread_mutex_unlock(&mMutex);

	res = muxer->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("muxer->start", -res);
		goto error;
	}

	*retObj = muxer;

	return 0;

error:
	delete muxer;
	return res;
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
	CodedVideoSinkWrapper *sink = nullptr;
	Channel *channel = nullptr;

	sink = new CodedVideoSinkWrapper(this, params, listener);
	if (sink == nullptr) {
		ULOGE("%s: failed to create the sink wrapper", __func__);
		return -ENOMEM;
	}
	if (sink->getElement() == nullptr) {
		ULOGE("%s: failed to create the sink", __func__);
		delete sink;
		return -EPROTO;
	}

	mElements.push_back(sink->getElement());

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

	*retObj = sink;

	return 0;

error:
	if (sink != nullptr) {
		if (channel != nullptr) {
			/* removeOutputChannel must be called without mMutex
			 * being held, so release it here */
			pthread_mutex_unlock(&mMutex);
			source->removeOutputChannel(media, channel);
			pthread_mutex_lock(&mMutex);
		}
		delete sink;
	}
	return res;
}


int Session::internalCreateRawVideoSink(
	Source *source,
	RawVideoMedia *media,
	const struct pdraw_video_sink_params *params,
	IPdraw::IRawVideoSink::Listener *listener,
	IPdraw::IRawVideoSink **retObj)
{
	/* Note: mMutex is held while this function is called */
	int res;
	RawVideoSinkWrapper *sink = nullptr;
	Channel *channel = nullptr;

	sink = new RawVideoSinkWrapper(this, params, listener);
	if (sink == nullptr) {
		ULOGE("%s: failed to create the sink wrapper", __func__);
		return -ENOMEM;
	}
	if (sink->getElement() == nullptr) {
		ULOGE("%s: failed to create the sink", __func__);
		delete sink;
		return -EPROTO;
	}

	mElements.push_back(sink->getElement());

	res = sink->getSink()->addInputMedia(media);
	if (res < 0) {
		ULOG_ERRNO("rawVideoSink->addInputMedia", -res);
		goto error;
	}

	res = sink->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("rawVideoSink->start", -res);
		goto error;
	}

	channel = sink->getSink()->getInputChannel(media);
	if (channel == nullptr) {
		ULOGE("failed to get raw video sink input channel");
		res = -EPROTO;
		goto error;
	}

	res = source->addOutputChannel(media, channel);
	if (res < 0) {
		ULOG_ERRNO("source->addOutputChannel", -res);
		goto error;
	}

	*retObj = sink;

	return 0;

error:
	if (sink != nullptr) {
		if (channel != nullptr) {
			/* removeOutputChannel must be called without mMutex
			 * being held, so release it here */
			pthread_mutex_unlock(&mMutex);
			source->removeOutputChannel(media, channel);
			pthread_mutex_lock(&mMutex);
		}
		delete sink;
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
	AudioSinkWrapper *sink = nullptr;
	Channel *channel = nullptr;

	sink = new AudioSinkWrapper(this, listener);
	if (sink == nullptr) {
		ULOGE("%s: failed to create the sink wrapper", __func__);
		return -ENOMEM;
	}
	if (sink->getElement() == nullptr) {
		ULOGE("%s: failed to create the sink", __func__);
		delete sink;
		return -EPROTO;
	}

	mElements.push_back(sink->getElement());

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

	*retObj = sink;

	return 0;

error:
	if (sink != nullptr) {
		if (channel != nullptr) {
			/* removeOutputChannel must be called without mMutex
			 * being held, so release it here */
			pthread_mutex_unlock(&mMutex);
			source->removeOutputChannel(media, channel);
			pthread_mutex_lock(&mMutex);
		}
		delete sink;
	}
	return res;
}


int Session::createVipcSource(const struct pdraw_vipc_source_params *params,
			      IPdraw::IVipcSource::Listener *listener,
			      IPdraw::IVipcSource **retObj)
{
#ifdef BUILD_LIBVIDEO_IPC
	int res;
	VipcSourceWrapper *source = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->address == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);

	source = new VipcSourceWrapper(this, params, listener);
	if (source == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the video IPC source wrapper",
		      __func__);
		return -ENOMEM;
	}
	if (source->getElement() == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the video IPC source", __func__);
		delete source;
		return -EPROTO;
	}

	mElements.push_back(source->getElement());
	pthread_mutex_unlock(&mMutex);

	res = source->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("vipcSource->start", -res);
		goto error;
	}

	*retObj = source;

	return 0;

error:
	delete source;
	return res;
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
	CodedVideoSourceWrapper *source = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->video.format != VDEF_FRAME_TYPE_CODED,
				 EINVAL);

	pthread_mutex_lock(&mMutex);

	source = new CodedVideoSourceWrapper(this, params, listener);
	if (source == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the source wrapper", __func__);
		return -ENOMEM;
	}
	if (source->getElement() == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the source", __func__);
		delete source;
		return -EPROTO;
	}

	mElements.push_back(source->getElement());
	pthread_mutex_unlock(&mMutex);

	res = source->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("codedVideoSource->start", -res);
		goto error;
	}

	*retObj = source;

	return 0;

error:
	delete source;
	return res;
}


int Session::createRawVideoSource(
	const struct pdraw_video_source_params *params,
	IPdraw::IRawVideoSource::Listener *listener,
	IPdraw::IRawVideoSource **retObj)
{
	int res;
	RawVideoSourceWrapper *source = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->video.format != VDEF_FRAME_TYPE_RAW,
				 EINVAL);

	pthread_mutex_lock(&mMutex);

	source = new RawVideoSourceWrapper(this, params, listener);
	if (source == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the source wrapper", __func__);
		return -ENOMEM;
	}
	if (source->getElement() == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the source", __func__);
		delete source;
		return -EPROTO;
	}

	mElements.push_back(source->getElement());
	pthread_mutex_unlock(&mMutex);

	res = source->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("rawVideoSource->start", -res);
		goto error;
	}

	*retObj = source;

	return 0;

error:
	delete source;
	return res;
}


int Session::createCodedVideoSink(unsigned int mediaId,
				  const struct pdraw_video_sink_params *params,
				  IPdraw::ICodedVideoSink::Listener *listener,
				  IPdraw::ICodedVideoSink **retObj)
{
	int ret = 0;
	bool found = false;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);

	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		Media *media;
		CodedVideoMedia *codedMedia = nullptr;
		Source *source = dynamic_cast<Source *>(*e);
		if (source == nullptr) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			media = source->getOutputMedia(i);
			if ((media != nullptr) && (media->id == mediaId)) {
				codedMedia =
					dynamic_cast<CodedVideoMedia *>(media);
				found = true;
				break;
			}
		}
		if (found && codedMedia != nullptr) {
			ret = internalCreateCodedVideoSink(
				source, codedMedia, params, listener, retObj);
			goto exit;
		}
		e++;
	}
	ret = -ENOENT;
exit:
	pthread_mutex_unlock(&mMutex);
	return ret;
}


int Session::createRawVideoSink(unsigned int mediaId,
				const struct pdraw_video_sink_params *params,
				IPdraw::IRawVideoSink::Listener *listener,
				IPdraw::IRawVideoSink **retObj)
{
	int ret = 0;
	bool found = false;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);

	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		Media *media;
		RawVideoMedia *rawMedia = nullptr;
		Source *source = dynamic_cast<Source *>(*e);
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
			ret = internalCreateRawVideoSink(
				source, rawMedia, params, listener, retObj);
			goto exit;
		}
		e++;
	}
	ret = -ENOENT;
exit:
	pthread_mutex_unlock(&mMutex);
	return ret;
}


int Session::createAlsaSource(const struct pdraw_alsa_source_params *params,
			      IPdraw::IAlsaSource::Listener *listener,
			      IPdraw::IAlsaSource **retObj)
{
#ifdef PDRAW_USE_ALSA
	int res;
	AlsaSourceWrapper *source = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->address == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);

	source = new AlsaSourceWrapper(this, params, listener);
	if (source == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the ALSA source wrapper", __func__);
		return -ENOMEM;
	}
	if (source->getElement() == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the ALSA source", __func__);
		delete source;
		return -EPROTO;
	}

	mElements.push_back(source->getElement());
	pthread_mutex_unlock(&mMutex);

	res = source->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("alsaSource->start", -res);
		goto error;
	}

	*retObj = source;

	return 0;

error:
	delete source;
	return res;
#else
	return -ENOSYS;
#endif
}


int Session::createAudioSource(const struct pdraw_audio_source_params *params,
			       IPdraw::IAudioSource::Listener *listener,
			       IPdraw::IAudioSource **retObj)
{
	int res;
	AudioSourceWrapper *source = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);

	source = new AudioSourceWrapper(this, params, listener);
	if (source == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the source wrapper", __func__);
		return -ENOMEM;
	}
	if (source->getElement() == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the source", __func__);
		delete source;
		return -EPROTO;
	}

	mElements.push_back(source->getElement());
	pthread_mutex_unlock(&mMutex);

	res = source->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("Element::start", -res);
		goto error;
	}

	*retObj = source;

	return 0;

error:
	delete source;
	return res;
}


int Session::createAudioSink(unsigned int mediaId,
			     IPdraw::IAudioSink::Listener *listener,
			     IPdraw::IAudioSink **retObj)
{
	int ret = 0;
	bool found = false;

	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);

	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		Media *media;
		AudioMedia *audioMedia = nullptr;
		Source *source = dynamic_cast<Source *>(*e);
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
			ret = internalCreateAudioSink(
				source, audioMedia, listener, retObj);
			goto exit;
		}
		e++;
	}
	ret = -ENOENT;
exit:
	pthread_mutex_unlock(&mMutex);
	return ret;
}


int Session::createAudioRenderer(
	unsigned int mediaId,
	const struct pdraw_audio_renderer_params *params,
	IPdraw::IAudioRenderer::Listener *listener,
	IPdraw::IAudioRenderer **retObj)
{
	int res;
	AudioRendererWrapper *renderer = nullptr;

	ULOG_ERRNO_RETURN_ERR_IF(params == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(params->address == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(listener == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(retObj == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);
	if (mState == STOPPING || mState == STOPPED) {
		ULOGE("renderer creation refused in %s state",
		      stateStr(mState));
		pthread_mutex_unlock(&mMutex);
		return -EPROTO;
	}

	renderer = new AudioRendererWrapper(this, mediaId, params, listener);
	if (renderer == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the audio renderer", __func__);
		return -ENOMEM;
	}
	if (renderer->getElement() == nullptr) {
		pthread_mutex_unlock(&mMutex);
		ULOGE("%s: failed to create the audio renderer wrapper",
		      __func__);
		delete renderer;
		return -EPROTO;
	}

	mElements.push_back(renderer->getElement());
	pthread_mutex_unlock(&mMutex);

	res = renderer->getElement()->start();
	if (res < 0) {
		ULOG_ERRNO("AudioRenderer::start", -res);
		goto error;
	}

	*retObj = renderer;

	return 0;

error:
	delete renderer;
	return res;
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

	pthread_mutex_lock(&mMutex);

	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		Media *media;
		RawVideoMedia *rawMedia = nullptr;
		Source *source = dynamic_cast<Source *>(*e);
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
			VideoEncoderWrapper *wrapper =
				new VideoEncoderWrapper(this, params, listener);
			if (wrapper == nullptr) {
				ULOGE("%s: failed to create the "
				      "video encoder wrapper",
				      __func__);
				ret = -ENOMEM;
				goto exit;
			}
			if (wrapper->getElement() == nullptr) {
				ULOGE("%s: failed to create the video encoder",
				      __func__);
				delete wrapper;
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
				delete wrapper;
				goto exit;
			}
			*retObj = wrapper;
			goto exit;
		}
		e++;
	}
	ret = -ENOENT;
exit:
	pthread_mutex_unlock(&mMutex);
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

	pthread_mutex_lock(&mMutex);

	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		Media *media;
		RawVideoMedia *rawMedia = nullptr;
		Source *source = dynamic_cast<Source *>(*e);
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
			VideoScalerWrapper *wrapper =
				new VideoScalerWrapper(this, params, listener);
			if (wrapper == nullptr) {
				ULOGE("failed to create the "
				      "video scaler wrapper");
				ret = -ENOMEM;
				goto exit;
			}
			if (wrapper->getElement() == nullptr) {
				ULOGE("%s: failed to create the video scaler",
				      __func__);
				delete wrapper;
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
				delete wrapper;
				goto exit;
			}
			*retObj = wrapper;
			goto exit;
		}
		e++;
	}
	ret = -ENOENT;
exit:
	pthread_mutex_unlock(&mMutex);
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

	pthread_mutex_lock(&mMutex);

	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		Media *media;
		AudioMedia *audioMedia = nullptr;
		Source *source = dynamic_cast<Source *>(*e);
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
			AudioEncoderWrapper *wrapper =
				new AudioEncoderWrapper(this, params, listener);
			if (wrapper == nullptr) {
				ULOGE("failed to create the "
				      "audio encoder wrapper");
				ret = -ENOMEM;
				goto exit;
			}
			if (wrapper->getElement() == nullptr) {
				ULOGE("%s: failed to create the audio encoder",
				      __func__);
				delete wrapper;
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
				delete wrapper;
				goto exit;
			}
			*retObj = wrapper;
			goto exit;
		}
		e++;
	}
	ret = -ENOENT;
exit:
	pthread_mutex_unlock(&mMutex);
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
	pthread_mutex_lock(&mAsyncMutex);
	mElementStateChangeElementArgs.push(element);
	mElementStateChangeStateArgs.push(state);
	int err = pomp_loop_idle_add_with_cookie(
		mLoop, idleElementStateChange, this, this);
	if (err > 0)
		ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	pthread_mutex_unlock(&mAsyncMutex);
}


int Session::addMediaToVideoRenderer(unsigned int mediaId,
				     Pdraw::VideoRenderer *renderer)
{
	return mFactory.addMediaToVideoRenderer(mediaId, renderer);
}


int Session::addMediaToAudioRenderer(unsigned int mediaId,
				     Pdraw::AudioRenderer *renderer)
{
	return mFactory.addMediaToAudioRenderer(mediaId, renderer);
}


int Session::addMediaToMuxer(unsigned int mediaId,
			     Pdraw::Muxer *muxer,
			     const struct pdraw_muxer_media_params *params)
{
	return mFactory.addMediaToMuxer(mediaId, muxer, params);
}


void Session::asyncElementDelete(Element *element)
{
	pthread_mutex_lock(&mAsyncMutex);
	mElementDeleteElementArgs.push(element);
	int err = pomp_loop_idle_add_with_cookie(
		mLoop, idleElementDelete, this, this);
	if (err > 0)
		ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	pthread_mutex_unlock(&mAsyncMutex);
}


void Session::setState(enum State state)
{
	pthread_mutex_lock(&mMutex);
	if (state == mState) {
		pthread_mutex_unlock(&mMutex);
		return;
	}

	mState = state;
	pthread_mutex_unlock(&mMutex);
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
	Session *self = reinterpret_cast<Session *>(userdata);
	pthread_mutex_lock(&self->mAsyncMutex);
	Element *element = self->mElementStateChangeElementArgs.front();
	Element::State state = self->mElementStateChangeStateArgs.front();
	self->mElementStateChangeElementArgs.pop();
	self->mElementStateChangeStateArgs.pop();
	pthread_mutex_unlock(&self->mAsyncMutex);
	ULOG_ERRNO_RETURN_IF(element == nullptr, EINVAL);
	self->onElementStateChanged(element, state);
}


int Session::deleteElement(Element *element)
{
	int ret = 0;
	bool found = false;
	ULOG_ERRNO_RETURN_ERR_IF(element == nullptr, EINVAL);

	pthread_mutex_lock(&mMutex);
	std::vector<Element *>::iterator e = mElements.begin();
	while (e != mElements.end()) {
		if (*e != element) {
			e++;
			continue;
		}
		found = true;
		mElements.erase(e);
		delete element;
		break;
	}
	if (!found) {
		ret = -ENOENT;
		ULOGW("%s: element not found in the list", __func__);
		delete element;
	}
	pthread_mutex_unlock(&mMutex);
	return ret;
}


void Session::idleElementDelete(void *userdata)
{
	int err;
	Session *self = reinterpret_cast<Session *>(userdata);
	pthread_mutex_lock(&self->mAsyncMutex);
	Element *element = self->mElementDeleteElementArgs.front();
	self->mElementDeleteElementArgs.pop();
	pthread_mutex_unlock(&self->mAsyncMutex);

	err = self->deleteElement(element);
	if (err < 0)
		ULOG_ERRNO("deleteElement", -err);
}


void Session::callStopResponse(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	pthread_mutex_lock(&self->mAsyncMutex);
	int status = self->mStopRespStatusArgs.front();
	self->mStopRespStatusArgs.pop();
	pthread_mutex_unlock(&self->mAsyncMutex);
	if (self->mListener == nullptr)
		return;
	self->mListener->stopResponse(self, status);
}


void Session::callOnMediaAdded(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	pthread_mutex_lock(&self->mAsyncMutex);
	struct pdraw_media_info info = self->mMediaAddedInfoArgs.front();
	self->mMediaAddedInfoArgs.pop();
	void *elementUserData = self->mMediaAddedElementUserDataArgs.front();
	self->mMediaAddedElementUserDataArgs.pop();
	pthread_mutex_unlock(&self->mAsyncMutex);
	self->mListener->onMediaAdded(self, &info, elementUserData);
	Media::cleanupMediaInfo(&info);
}


void Session::callOnMediaRemoved(void *userdata)
{
	Session *self = reinterpret_cast<Session *>(userdata);
	pthread_mutex_lock(&self->mAsyncMutex);
	struct pdraw_media_info info = self->mMediaRemovedInfoArgs.front();
	self->mMediaRemovedInfoArgs.pop();
	void *elementUserData = self->mMediaRemovedElementUserDataArgs.front();
	self->mMediaRemovedElementUserDataArgs.pop();
	pthread_mutex_unlock(&self->mAsyncMutex);
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

		pthread_mutex_lock(&mMutex);

		curState = mState;

		std::vector<Element *>::iterator e = mElements.begin();
		while (e != mElements.end()) {
			if ((*e)->getState() != Element::State::STOPPED) {
				stopped = false;
				break;
			}
			e++;
		}

		pthread_mutex_unlock(&mMutex);

		asyncElementDelete(element);

		if (stopped && curState == STOPPING) {
			setState(STOPPED);

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
			pthread_mutex_lock(&mAsyncMutex);
			mMediaAddedInfoArgs.push(info);
			mMediaAddedElementUserDataArgs.push(elementUserData);
			int err = pomp_loop_idle_add_with_cookie(
				mLoop, callOnMediaAdded, this, this);
			if (err > 0) {
				ULOG_ERRNO("pomp_loop_idle_add_with_cookie",
					   -err);
			}
			pthread_mutex_unlock(&mAsyncMutex);
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
		pthread_mutex_lock(&mAsyncMutex);
		mMediaRemovedInfoArgs.push(info);
		mMediaRemovedElementUserDataArgs.push(elementUserData);
		int err = pomp_loop_idle_add_with_cookie(
			mLoop, callOnMediaRemoved, this, this);
		if (err > 0)
			ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
		pthread_mutex_unlock(&mAsyncMutex);
	}
}


void Session::stopResp(int status)
{
	pthread_mutex_lock(&mAsyncMutex);
	mStopRespStatusArgs.push(status);
	int err = pomp_loop_idle_add_with_cookie(
		mLoop, callStopResponse, this, this);
	if (err > 0)
		ULOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	pthread_mutex_unlock(&mAsyncMutex);
}


const char *Session::stateStr(enum State val)
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


Session::PipelineFactory::~PipelineFactory(void)
{
	return;
}


void Session::PipelineFactory::onElementStateChanged(Element *element,
						     Element::State state)
{
	if (state == Element::State::STARTED) {
		Pdraw::VideoRenderer *rv =
			dynamic_cast<Pdraw::VideoRenderer *>(element);
		Pdraw::AudioRenderer *ra =
			dynamic_cast<Pdraw::AudioRenderer *>(element);
		if (rv != nullptr) {
			int ret = addAllMediaToVideoRenderer(rv);
			if (ret < 0)
				ULOG_ERRNO("addAllMediaToVideoRenderer", -ret);
		} else if (ra != nullptr) {
			int ret = addAllMediaToAudioRenderer(ra);
			if (ret < 0)
				ULOG_ERRNO("addAllMediaToAudioRenderer", -ret);
		}
	}
}


void Session::PipelineFactory::onOutputMediaAdded(Source *source, Media *media)
{
	Pdraw::Demuxer *demuxer = dynamic_cast<Pdraw::Demuxer *>(source);
	VideoDecoder *vDecoder = dynamic_cast<VideoDecoder *>(source);
	AudioDecoder *aDecoder = dynamic_cast<AudioDecoder *>(source);
	CodedVideoMedia *codedMedia = dynamic_cast<CodedVideoMedia *>(media);
	RawVideoMedia *rawMedia = dynamic_cast<RawVideoMedia *>(media);
	AudioMedia *audioMedia = dynamic_cast<AudioMedia *>(media);
	if ((demuxer != nullptr) && (codedMedia != nullptr)) {
		if (demuxer->getParams()->autodecoding_mode ==
		    PDRAW_DEMUXER_AUTODECODING_MODE_DECODE_ALL) {
			int ret = addVideoDecoderForMedia(source, codedMedia);
			if (ret < 0)
				ULOG_ERRNO("addVideoDecoderForMedia", -ret);
		}
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
	} else if ((aDecoder != nullptr) && (audioMedia != nullptr)) {
		int ret = addMediaToAllAudioRenderers(source, audioMedia);
		if (ret < 0)
			ULOG_ERRNO("addMediaToAllAudioRenderers", -ret);
	}
}


void Session::PipelineFactory::onOutputMediaRemoved(Source *source,
						    Media *media)
{
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

	pthread_mutex_lock(&mSession->mMutex);

	/* First pass: list the elements with their sink and source medias */
	std::vector<Element *>::iterator e = mSession->mElements.begin();
	while (e != mSession->mElements.end()) {
		unsigned int elmId = (*e)->getId();
		const char *elmName = (*e)->getName().c_str();
		fprintf(f, "\te%u [shape=record,label=\"", elmId);

		/* Element input medias */
		Sink *sink = dynamic_cast<Sink *>(*e);
		if (sink != nullptr) {
			unsigned int count = sink->getInputMediaCount();
			if (count > 0)
				fprintf(f, "{ ");
			for (unsigned int i = 0; i < count; i++) {
				Media *media = sink->getInputMedia(i);
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
		Source *source = dynamic_cast<Source *>(*e);
		if (source != nullptr) {
			unsigned int count = source->getOutputMediaCount();
			if (count > 0)
				fprintf(f, " | { ");
			for (unsigned int i = 0; i < count; i++) {
				Media *media = source->getOutputMedia(i);
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
		unsigned int dstElmId = (*e)->getId();

		/* Element input medias */
		Sink *sink = dynamic_cast<Sink *>(*e);
		if (sink != nullptr) {
			unsigned int count = sink->getInputMediaCount();
			for (unsigned int i = 0; i < count; i++) {
				Media *media = sink->getInputMedia(i);
				std::vector<Element *>::iterator e2 =
					mSession->mElements.begin();
				while (e2 != mSession->mElements.end()) {
					Source *source =
						dynamic_cast<Source *>(*e2);
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

	pthread_mutex_unlock(&mSession->mMutex);

	fprintf(f, "}");
	fclose(f);

	ULOGI("pipeline dumped to file '%s'", fileName.c_str());

	return 0;
}


int Session::PipelineFactory::addVideoDecoderForMedia(Source *source,
						      CodedVideoMedia *media)
{
	int ret;

	VideoDecoder *decoder = new VideoDecoder(mSession, mSession, mSession);
	if (decoder == nullptr) {
		ULOGE("decoder creation failed");
		return -ENOMEM;
	}
	ret = decoder->addInputMedia(media);
	if (ret < 0) {
		if (ret == -ENOSYS)
			ret = 0;
		else
			ULOG_ERRNO("decoder->addInputMedia", -ret);
		delete decoder;
		return ret;
	}
	pthread_mutex_lock(&mSession->mMutex);
	mSession->mElements.push_back(decoder);
	pthread_mutex_unlock(&mSession->mMutex);
	ret = decoder->start();
	if (ret < 0) {
		ULOG_ERRNO("decoder->start", -ret);
		return ret;
	}
	Channel *channel = decoder->getInputChannel(media);
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
	decoder->resync();

	return 0;
}


int Session::PipelineFactory::addAudioDecoderForMedia(Source *source,
						      AudioMedia *media)
{
	int ret;

	AudioDecoder *decoder = new AudioDecoder(mSession, mSession, mSession);
	if (decoder == nullptr) {
		ULOGE("decoder creation failed");
		return -ENOMEM;
	}
	ret = decoder->addInputMedia(media);
	if (ret < 0) {
		if (ret == -ENOSYS)
			ret = 0;
		else
			ULOG_ERRNO("decoder->addInputMedia", -ret);
		delete decoder;
		return ret;
	}
	pthread_mutex_lock(&mSession->mMutex);
	mSession->mElements.push_back(decoder);
	pthread_mutex_unlock(&mSession->mMutex);
	ret = decoder->start();
	if (ret < 0) {
		ULOG_ERRNO("decoder->start", -ret);
		return ret;
	}
	Channel *channel = decoder->getInputChannel(media);
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

	if (encoder == nullptr) {
		encoder = new VideoEncoder(mSession,
					   mSession,
					   mSession,
					   listener,
					   nullptr,
					   params);
		if (encoder == nullptr) {
			ULOGE("encoder creation failed");
			return -ENOMEM;
		}
		allocated = true;
	}

	pthread_mutex_lock(&mSession->mMutex);
	mSession->mElements.push_back(encoder);
	pthread_mutex_unlock(&mSession->mMutex);

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

	if (scaler == nullptr) {
		scaler = new VideoScaler(mSession,
					 mSession,
					 mSession,
					 listener,
					 nullptr,
					 params);
		if (scaler == nullptr) {
			ULOGE("scaler creation failed");
			return -ENOMEM;
		}
		allocated = true;
	}

	pthread_mutex_lock(&mSession->mMutex);
	mSession->mElements.push_back(scaler);
	pthread_mutex_unlock(&mSession->mMutex);

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

	if (encoder == nullptr) {
		encoder = new AudioEncoder(mSession,
					   mSession,
					   mSession,
					   listener,
					   nullptr,
					   params);
		if (encoder == nullptr) {
			ULOGE("encoder creation failed");
			return -ENOMEM;
		}
		allocated = true;
	}

	pthread_mutex_lock(&mSession->mMutex);
	mSession->mElements.push_back(encoder);
	pthread_mutex_unlock(&mSession->mMutex);

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
	Pdraw::VideoRenderer *renderer)
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

	pthread_mutex_lock(&mSession->mMutex);
	std::vector<Element *>::iterator e = mSession->mElements.begin();
	while (e != mSession->mElements.end()) {
		Source *source = dynamic_cast<Source *>(*e);
		if (source == nullptr) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			RawVideoMedia *media = dynamic_cast<RawVideoMedia *>(m);
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
		e++;
	}
	pthread_mutex_unlock(&mSession->mMutex);

	return 0;
}


int Session::PipelineFactory::addMediaToAllVideoRenderers(Source *source,
							  RawVideoMedia *media)
{
	int ret = 0;

	pthread_mutex_lock(&mSession->mMutex);
	std::vector<Element *>::iterator e = mSession->mElements.begin();
	while (e != mSession->mElements.end() && ret == 0) {
		Pdraw::VideoRenderer *r =
			dynamic_cast<Pdraw::VideoRenderer *>(*e);
		e++;
		if (r == nullptr)
			continue;
		ret = addMediaToVideoRenderer(source, media, r);
	}
	pthread_mutex_unlock(&mSession->mMutex);

	return ret;
}


int Session::PipelineFactory::addAllMediaToVideoRenderer(
	Pdraw::VideoRenderer *renderer)
{
	int ret;

	pthread_mutex_lock(&mSession->mMutex);
	std::vector<Element *>::iterator e = mSession->mElements.begin();
	while (e != mSession->mElements.end()) {
		Source *source = dynamic_cast<Source *>(*e);
		if (source == nullptr) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			RawVideoMedia *media = dynamic_cast<RawVideoMedia *>(m);
			if (media == nullptr)
				continue;
			ret = addMediaToVideoRenderer(source, media, renderer);
			if (ret < 0)
				ULOG_ERRNO("addMediaToVideoRenderer", -ret);
		}
		e++;
	}
	pthread_mutex_unlock(&mSession->mMutex);

	return 0;
}


int Session::PipelineFactory::addMediaToAudioRenderer(
	Source *source,
	AudioMedia *media,
	Pdraw::AudioRenderer *renderer)
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

	pthread_mutex_lock(&mSession->mMutex);
	std::vector<Element *>::iterator e = mSession->mElements.begin();
	while (e != mSession->mElements.end()) {
		Source *source = dynamic_cast<Source *>(*e);
		if (source == nullptr) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			AudioMedia *media = dynamic_cast<AudioMedia *>(m);
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
		e++;
	}
	pthread_mutex_unlock(&mSession->mMutex);

	return 0;
}


int Session::PipelineFactory::addMediaToAllAudioRenderers(Source *source,
							  AudioMedia *media)
{
	int ret = 0;

	pthread_mutex_lock(&mSession->mMutex);
	std::vector<Element *>::iterator e = mSession->mElements.begin();
	while (e != mSession->mElements.end() && ret == 0) {
		Pdraw::AudioRenderer *r =
			dynamic_cast<Pdraw::AudioRenderer *>(*e);
		e++;
		if (r == nullptr)
			continue;
		ret = addMediaToAudioRenderer(source, media, r);
	}
	pthread_mutex_unlock(&mSession->mMutex);

	return ret;
}


int Session::PipelineFactory::addAllMediaToAudioRenderer(
	Pdraw::AudioRenderer *renderer)
{
	int ret;

	pthread_mutex_lock(&mSession->mMutex);
	std::vector<Element *>::iterator e = mSession->mElements.begin();
	while (e != mSession->mElements.end()) {
		Source *source = dynamic_cast<Source *>(*e);
		if (source == nullptr) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			Media *m = source->getOutputMedia(i);
			AudioMedia *media = dynamic_cast<AudioMedia *>(m);
			if (media == nullptr)
				continue;
			ret = addMediaToAudioRenderer(source, media, renderer);
			if (ret < 0)
				ULOG_ERRNO("addMediaToAudioRenderer", -ret);
		}
		e++;
	}
	pthread_mutex_unlock(&mSession->mMutex);

	return 0;
}


int Session::PipelineFactory::addMediaToMuxer(
	Source *source,
	Media *media,
	Pdraw::Muxer *muxer,
	const struct pdraw_muxer_media_params *params)
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
	CodedVideoMedia *codedMedia = nullptr;
	RawVideoMedia *rawMedia = nullptr;
	AudioMedia *audioMedia = nullptr;
	bool found = false;

	pthread_mutex_lock(&mSession->mMutex);
	std::vector<Element *>::iterator e = mSession->mElements.begin();
	while (e != mSession->mElements.end()) {
		source = dynamic_cast<Source *>(*e);
		if (source == nullptr) {
			e++;
			continue;
		}
		unsigned int mediaCount = source->getOutputMediaCount();
		for (unsigned int i = 0; i < mediaCount; i++) {
			media = source->getOutputMedia(i);
			codedMedia = dynamic_cast<CodedVideoMedia *>(media);
			rawMedia = dynamic_cast<RawVideoMedia *>(media);
			audioMedia = dynamic_cast<AudioMedia *>(media);
			if ((codedMedia != nullptr) &&
			    (codedMedia->id == mediaId)) {
				found = true;
				break;
			} else if ((rawMedia != nullptr) &&
				   (rawMedia->id == mediaId)) {
				found = true;
				break;
			} else if ((audioMedia != nullptr) &&
				   (audioMedia->id == mediaId)) {
				found = true;
				break;
			}
		}
		if (found)
			break;
		e++;
	}

	if ((!found) || (source == nullptr) ||
	    (codedMedia == nullptr && rawMedia == nullptr &&
	     audioMedia == nullptr)) {
		pthread_mutex_unlock(&mSession->mMutex);
		return -ENOENT;
	}

	ret = addMediaToMuxer(source, media, muxer, params);
	if (ret < 0)
		ULOG_ERRNO("addMediaToMuxer", -ret);

	pthread_mutex_unlock(&mSession->mMutex);

	return ret;
}

} /* namespace Pdraw */
