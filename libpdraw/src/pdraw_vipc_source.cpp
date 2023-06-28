/**
 * Parrot Drones Awesome Video Viewer Library
 * Video IPC source
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

#ifdef BUILD_LIBVIDEO_IPC

#	define ULOG_TAG pdraw_vipc_source
#	include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#	include <time.h>

#	if PDRAW_VIPC_BACKEND_HISI || PDRAW_VIPC_BACKEND_NETWORK_HISI
#		include <hi_buffer.h>
#		include <mpi_sys.h>
#		include <mpi_vb.h>
#		include <mpi_venc.h>
#		include <media-buffers/mbuf_mem_hisi.h>
#	endif

#	if PDRAW_VIPC_BACKEND_DMABUF
#		include <media-buffers/mbuf_mem_ion.h>
#	endif

#	include "pdraw_session.hpp"
#	include "pdraw_vipc_source.hpp"

#	define PDRAW_VIPC_SOURCE_ANCILLARY_KEY_INPUT_TIME                     \
		"pdraw.vipcsource.input_time"

#	define DEFAULT_BACKEND_NAME "default"
#	define DEFAULT_VIPC_MAX_FRAMES 30
#	define DEFAULT_IN_POOL_SIZE 10
#	define DEFAULT_TIMESCALE 1000000
#	define DEFAULT_TIMEOUT_MS 2000


namespace Pdraw {


/* TODO: Channel::DownstreamEvent::TIMEOUT? */


VipcSource::VipcSource(Session *session,
		       Element::Listener *elementListener,
		       Source::Listener *sourceListener,
		       IPdraw::IVipcSource::Listener *listener,
		       IPdraw::IVipcSource *source,
		       const struct pdraw_vipc_source_params *params) :
		SourceElement(session, elementListener, 1, sourceListener),
		mVipcSource(source), mVipcSourceListener(listener),
		mClient(nullptr),
		mLastEosReason(PDRAW_VIPC_SOURCE_EOS_REASON_NONE),
		mOutputMedia(nullptr), mOutputMediaChanging(false),
		mVipcConnected(false), mReady(false), mWasReady(false),
		mRunning(false), mWasRunning(false), mFirstFrame(true),
		mInputFramesCount(0), mFrameIndex(0),
		mTimescale(DEFAULT_TIMESCALE), mLastTimestamp(UINT64_MAX),
		mFlushPending(false), mWatchdogTimer(nullptr)
{
	Element::setClassName(__func__);

#	if PDRAW_VIPC_BACKEND_NETWORK_CBUF
	mPool = nullptr;
#	endif

	mBackendType = (VipcSource::BackendType)0;
	mStatus = {};

	mParams = *params;
	if (params->address != nullptr) {
		mAddress = params->address;
		mParams.address = mAddress.c_str();
	} else {
		mParams.address = nullptr;
	}
	if (params->friendly_name != nullptr) {
		mFriendlyName = params->friendly_name;
		mParams.friendly_name = mFriendlyName.c_str();
	} else {
		mParams.friendly_name = nullptr;
	}
	if (params->backend_name != nullptr) {
		mBackendName = params->backend_name;
		mParams.backend_name = mBackendName.c_str();
	} else {
		mParams.backend_name = nullptr;
	}
	if (mParams.frame_count == 0)
		mParams.frame_count = DEFAULT_VIPC_MAX_FRAMES;
	if ((mParams.crop.left == 0.f) && (mParams.crop.top == 0.f) &&
	    (mParams.crop.width == 0.f) && (mParams.crop.height == 0.f)) {
		mParams.crop.left = mParams.crop.top = 0.f;
		mParams.crop.width = mParams.crop.height = 1.f;
	}
	if (mParams.decimation == 0)
		mParams.decimation = 1;
	if (mParams.timeout_ms == 0)
		mParams.timeout_ms = DEFAULT_TIMEOUT_MS;

	setState(CREATED);
}


VipcSource::~VipcSource(void)
{
	int err;

	if (mState == STARTED)
		PDRAW_LOGW("VIPC source is still running");

	if (mClient != nullptr) {
		err = vipcc_destroy(mClient);
		if (err < 0)
			PDRAW_LOG_ERRNO("vipcc_destroy", -err);
	}

	if (mOutputMedia != nullptr)
		PDRAW_LOGW("output media was not properly removed");

	if (mWatchdogTimer != nullptr) {
		err = pomp_timer_clear(mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
	}
}


int VipcSource::start(void)
{
	int ret;

	if ((mState == STARTED) || (mState == STARTING))
		return 0;
	if (mState != CREATED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}
	setState(STARTING);

	const Backend *backend =
		getBackend(mParams.backend_name, true, &mBackendType);
	if (backend == nullptr) {
		PDRAW_LOGE("%s: unable to select backend '%s'",
			   __func__,
			   mParams.backend_name);
		ret = -ENOSYS;
		goto error;
	}

	/* Watchdog is only created if timeout is specified */
	if (mWatchdogTimer == nullptr && mParams.timeout_ms != -1) {
		mWatchdogTimer = pomp_timer_new(
			mSession->getLoop(), watchdogTimerCb, this);
		if (mWatchdogTimer == nullptr) {
			ret = -ENOMEM;
			PDRAW_LOGE("pomp_timer_new failed");
			goto error;
		}
	}

	mClient = vipcc_new2(mSession->getLoop(),
			     backend->clientCbs,
			     backend->beCbs,
			     mParams.address,
			     mParams.friendly_name,
			     this,
			     mParams.frame_count,
			     true);
	if (mClient == nullptr) {
		PDRAW_LOGE("%s: failed to create VIPC client", __func__);
		ret = -ENOMEM;
		goto error;
	}

	if (mWatchdogTimer != nullptr) {
		ret = pomp_timer_set(mWatchdogTimer, mParams.timeout_ms);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("pomp_timer_set", -ret);
			goto error;
		}
	}

	setState(STARTED);
	return 0;

error:
	(void)stop();
	return ret;
}


int VipcSource::stop(void)
{
	int ret, err;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if ((mState != STARTED) && (mState != STARTING)) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	setState(STOPPING);

	if (mRunning) {
		err = vipcc_stop(mClient);
		if (err < 0)
			PDRAW_LOG_ERRNO("vipcc_stop", -err);
		mRunning = false;
	}

	/* Flush everything */
	ret = flush();
	if (ret < 0)
		PDRAW_LOG_ERRNO("flush", -ret);

	/* When the flush is complete, stopping will be triggered */
	return ret;
}


int VipcSource::tryStop(void)
{
	int pendingCount;

	if (mState != STOPPING)
		return 0;

	pendingCount = teardownChannels();

	if (pendingCount == 0)
		completeStop();

	return 0;
}


void VipcSource::completeStop(void)
{
	int err;
	unsigned int outputChannelCount;

	Source::lock();

	if (mOutputMedia == nullptr)
		goto exit;

	outputChannelCount = getOutputChannelCount(mOutputMedia);
	if (outputChannelCount > 0) {
		Source::unlock();
		return;
	}

#	if PDRAW_VIPC_BACKEND_NETWORK_CBUF
	if (mPool) {
		int err = mbuf_pool_destroy(mPool);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_pool_destroy", -err);
	}
#	endif

	(void)destroyMedia();

exit:
	Source::unlock();

	if (mWatchdogTimer != nullptr) {
		err = pomp_timer_clear(mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
		mWatchdogTimer = nullptr;
	}

	setState(STOPPED);
}


void VipcSource::onChannelUnlink(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Source::onChannelUnlink(channel);

	if (mState == STOPPING) {
		completeStop();
		return;
	}

	if (mOutputMediaChanging) {
		Source::lock();

		unsigned int outputChannelCount =
			getOutputChannelCount(mOutputMedia);
		if (outputChannelCount > 0) {
			Source::unlock();
			return;
		}

		Source::unlock();

		(void)destroyMedia();
		mOutputMediaChanging = false;
		(void)setupMedia();
	}
}


int VipcSource::flush(void)
{
	int err;
	Channel *outputChannel;

	/* Flush the output channels (async) */
	Source::lock();
	if (mOutputMedia != nullptr) {
		unsigned int outputChannelCount =
			getOutputChannelCount(mOutputMedia);
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia, i);
			if (outputChannel == nullptr) {
				PDRAW_LOGW(
					"failed to get output channel "
					"at index %d",
					i);
				continue;
			}
			err = outputChannel->flush();
			if (err < 0)
				PDRAW_LOG_ERRNO("channel->flush", -err);
			else
				mFlushPending = true;
		}
	}
	Source::unlock();

	if (!mFlushPending)
		completeFlush();

	return 0;
}


void VipcSource::completeFlush(void)
{
	bool pending = false;

	Source::lock();
	if (mOutputMedia != nullptr) {
		unsigned int outputChannelCount =
			getOutputChannelCount(mOutputMedia);
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			Channel *outputChannel =
				getOutputChannel(mOutputMedia, i);
			if (outputChannel == nullptr) {
				PDRAW_LOGW(
					"failed to get output channel "
					"at index %d",
					i);
				continue;
			}
			if (outputChannel->isFlushPending()) {
				pending = true;
				break;
			}
		}
	}
	Source::unlock();

	if (pending)
		return;
	mFlushPending = false;

	if ((mState != STOPPING) && (!mOutputMediaChanging))
		return;

	int pendingCount = teardownChannels();
	if (pendingCount == 0) {
		if (mState == STOPPING) {
			completeStop();
		} else if (mOutputMediaChanging) {
			(void)destroyMedia();
			mOutputMediaChanging = false;
			(void)setupMedia();
		}
	}
}


void VipcSource::onChannelFlushed(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("media not found");
		return;
	}
	PDRAW_LOGD("'%s': channel flushed media name=%s (channel owner=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getOwner());

	completeFlush();
}


bool VipcSource::isReadyToPlay(void)
{
	return mReady;
}


bool VipcSource::isPaused(void)
{
	return !mRunning;
}


int VipcSource::play(void)
{
	if (mState != STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}
	if (!mReady) {
		PDRAW_LOGE("%s: not ready to play", __func__);
		return -EPROTO;
	}

	if (mRunning)
		return 0;

	int ret = vipcc_start(mClient);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vipcc_start", -ret);
		return ret;
	}

	if (mWatchdogTimer != nullptr) {
		int err = pomp_timer_set(mWatchdogTimer, mParams.timeout_ms);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set", -err);
	}

	mRunning = true;
	return 0;
}


int VipcSource::pause(void)
{
	if (mState != STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}
	if (!mReady) {
		PDRAW_LOGE("%s: not ready to play", __func__);
		return -EPROTO;
	}

	if (!mRunning)
		return 0;

	int ret = vipcc_stop(mClient);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vipcc_stop", -ret);
		return ret;
	}

	if (mWatchdogTimer != nullptr) {
		int err = pomp_timer_clear(mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
	}

	mRunning = false;
	return 0;
}


int VipcSource::configure(const struct vdef_dim *resolution,
			  const struct vdef_rectf *crop)
{
	/* TODO */
	return -ENOSYS;
}


int VipcSource::setSessionMetadata(const struct vmeta_session *meta)
{
	if (meta == nullptr)
		return -EINVAL;

	mParams.session_meta = *meta;

	Source::lock();
	if (mOutputMedia != nullptr) {
		mOutputMedia->sessionMeta = mParams.session_meta;
		sendDownstreamEvent(
			mOutputMedia,
			Channel::DownstreamEvent::SESSION_META_UPDATE);
	}
	Source::unlock();

	return 0;
}


int VipcSource::getSessionMetadata(struct vmeta_session *meta)
{
	if (meta == nullptr)
		return -EINVAL;

	*meta = mParams.session_meta;

	return 0;
}


int VipcSource::processFrame(const struct vipc_frame *vipcFrame,
			     struct mbuf_mem *mem)
{
	int ret, err;
	struct mbuf_raw_video_frame *mbufFrame = nullptr;
	struct vdef_raw_frame frameInfo = {};
	struct vdef_raw_format format = {};
	RawVideoMedia::Frame out_meta = {};
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	unsigned int outputChannelCount;
	IPdraw::IVipcSource::Listener *listener = getVipcSourceListener();

	if (mState != STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	if (mWatchdogTimer != nullptr) {
		err = pomp_timer_set(mWatchdogTimer, mParams.timeout_ms);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set", -err);
	}

	Source::lock();

	if (mOutputMedia == nullptr) {
		PDRAW_LOGE("%s: invalid output media", __func__);
		ret = -EPROTO;
		goto out;
	}

	if (mFirstFrame) {
		mFirstFrame = false;
		sendDownstreamEvent(mOutputMedia,
				    Channel::DownstreamEvent::SOS);
	}

	ret = vacq_pix_format_to_vdef_raw_format(vipcFrame->format,
						 &frameInfo.format);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vacq_pix_format_to_vdef_raw_format", -ret);
		goto out;
	}
	vdef_format_to_frame_info(&mOutputMedia->info, &frameInfo.info);
	frameInfo.info.timestamp =
		(vipcFrame->ts_sof_ns * mTimescale + 500000000) / 1000000000;
	frameInfo.info.timescale = mTimescale;
	frameInfo.info.capture_timestamp = vipcFrame->ts_sof_ns / 1000;
	frameInfo.info.index = mFrameIndex++;
	frameInfo.info.full_range =
		(vipcFrame->meta.flags & VIPC_META_FLAG_FULL_RANGE);
	frameInfo.info.resolution.width = vipcFrame->width;
	frameInfo.info.resolution.height = vipcFrame->height;
	for (unsigned int i = 0; i < vipcFrame->num_planes; i++)
		frameInfo.plane_stride[i] = vipcFrame->planes[i].stride;

	/* Check the frame format */
	if (!vdef_raw_format_cmp(&frameInfo.format, &mOutputMedia->format)) {
		char *fmt1 = vdef_raw_format_to_str(&format);
		char *fmt2 = vdef_raw_format_to_str(&mOutputMedia->format);
		PDRAW_LOGE("%s: invalid frame foramt (%s, expected %s)",
			   __func__,
			   fmt1,
			   fmt2);
		free(fmt1);
		free(fmt2);
		ret = -EINVAL;
		goto out;
	}

	/* Check the frame resolution */
	if (!vdef_dim_cmp(&mOutputMedia->info.resolution,
			  &frameInfo.info.resolution)) {
		PDRAW_LOGE(
			"%s: invalid frame resolution (%ux%u, expected %ux%u)",
			__func__,
			frameInfo.info.resolution.width,
			frameInfo.info.resolution.height,
			mOutputMedia->info.resolution.width,
			mOutputMedia->info.resolution.height);
		ret = -EINVAL;
		goto out;
	}

	/* Check the color range */
	if (mOutputMedia->info.full_range != frameInfo.info.full_range) {
		PDRAW_LOGE("%s: invalid range (%s, expected %s)",
			   __func__,
			   frameInfo.info.full_range ? "full" : "limited",
			   mOutputMedia->info.full_range ? "full" : "limited");
		ret = -EINVAL;
		goto out;
	}

	/* Check the frame timestamp */
	if ((mLastTimestamp != UINT64_MAX) &&
	    (frameInfo.info.timestamp <= mLastTimestamp)) {
		PDRAW_LOGE("%s: non-strictly-monotonic timestamp (%" PRIu64
			   " <= %" PRIu64 ")",
			   __func__,
			   frameInfo.info.timestamp,
			   mLastTimestamp);
		ret = -EINVAL;
		goto out;
	}

	/* Save frame timestamp as last_timestamp */
	mLastTimestamp = frameInfo.info.timestamp;

	/* Create the raw video frame */
	ret = mbuf_raw_video_frame_new(&frameInfo, &mbufFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_new", -ret);
		goto out;
	}
	for (unsigned int i = 0; i < vipcFrame->num_planes; i++) {
		size_t offset = vipcFrame->planes[i].virt_addr -
				vipcFrame->planes[0].virt_addr;
		ret = mbuf_raw_video_frame_set_plane(
			mbufFrame, i, mem, offset, vipcFrame->planes[i].size);
		if (ret < 0) {
			PDRAW_LOG_ERRNO(
				"mbuf_raw_video_frame_set_plane(%u)", -ret, i);
			goto out;
		}
	}

	/* Set the ancillary data */
	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTime);
	err = mbuf_raw_video_frame_add_ancillary_buffer(
		mbufFrame,
		PDRAW_VIPC_SOURCE_ANCILLARY_KEY_INPUT_TIME,
		&curTime,
		sizeof(curTime));
	if (err < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
				-err);
	}

	out_meta.ntpTimestamp = frameInfo.info.capture_timestamp;
	out_meta.ntpUnskewedTimestamp = frameInfo.info.capture_timestamp;
	out_meta.ntpRawTimestamp = frameInfo.info.capture_timestamp;
	out_meta.ntpRawUnskewedTimestamp = frameInfo.info.capture_timestamp;
	out_meta.demuxOutputTimestamp = curTime;
	out_meta.playTimestamp = frameInfo.info.capture_timestamp;
	out_meta.captureTimestamp = frameInfo.info.capture_timestamp;
	out_meta.localTimestamp = frameInfo.info.capture_timestamp;

	ret = mbuf_raw_video_frame_add_ancillary_buffer(
		mbufFrame,
		PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
				-ret);
		goto out;
	}

	/* Notify that a frame is ready */
	if (listener != nullptr) {
		listener->vipcSourceFrameReady(
			mSession, getVipcSource(), mbufFrame);
	}

	/* Finalize the frame */
	ret = mbuf_raw_video_frame_finalize(mbufFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_finalize", -ret);
		goto out;
	}

	/* Queue the frame in the output channels */
	outputChannelCount = getOutputChannelCount(mOutputMedia);
	for (unsigned int i = 0; i < outputChannelCount; i++) {
		Channel *c = getOutputChannel(mOutputMedia, i);
		RawVideoChannel *channel = dynamic_cast<RawVideoChannel *>(c);
		if (channel == nullptr) {
			PDRAW_LOGE("failed to get channel at index %d", i);
			continue;
		}
		err = channel->queue(mbufFrame);
		if (err < 0)
			PDRAW_LOG_ERRNO("channel->queue", -err);
	}

out:
	Source::unlock();

	if (mbufFrame != nullptr) {
		err = mbuf_raw_video_frame_unref(mbufFrame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref", -err);
	}

	return ret;
}


const VipcSource::Backend *VipcSource::getBackend(const char *name,
						  bool useDefault,
						  VipcSource::BackendType *type)
{
	if (BE_COUNT == 0)
		return nullptr;
	if (!name || strcmp(name, DEFAULT_BACKEND_NAME) == 0)
		goto default_backend;

	for (int i = 0; i < BE_COUNT; i++) {
		if (strcasecmp(name, cBackends[i].name) != 0)
			continue;
		if (type)
			*type = (VipcSource::BackendType)i;
		return &cBackends[i];
	}
	ULOGW("unknown backend type %s", name);

default_backend:
	if (useDefault) {
		if (type)
			*type = (VipcSource::BackendType)0;
		return &cBackends[0];
	}
	return nullptr;
}


int VipcSource::setupMedia(void)
{
	if (mState == STOPPED)
		return 0;

	if (mState != STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	Source::lock();

	if (mOutputMediaChanging) {
		PDRAW_LOGI("%s: output media is already pending change",
			   __func__);
		Source::unlock();
		return 0;
	}

	if (mOutputMedia != nullptr) {
		mOutputMediaChanging = true;
		int ret = flush();
		Source::unlock();
		return ret;
	}

	Source::unlock();

	if (mStatus.width > 0 && mStatus.height > 0)
		return createMedia();
	else
		return 0;
}


int VipcSource::createMedia(void)
{
	int ret;
	std::string path;

	if (mState != STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	Source::lock();

	if (mOutputMedia != nullptr) {
		Source::unlock();
		PDRAW_LOGE("already existing output media");
		return -EALREADY;
	}

	mOutputMedia = new RawVideoMedia(mSession);
	if (mOutputMedia == nullptr) {
		Source::unlock();
		PDRAW_LOGE("output media allocation failed");
		return -ENOMEM;
	}
	path = Element::getName() + "$" + mOutputMedia->getName();
	mOutputMedia->setPath(path);

	ret = addOutputPort(mOutputMedia);
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("addOutputPort", -ret);
		return ret;
	}

	ret = vacq_pix_format_to_vdef_raw_format(mStatus.format,
						 &mOutputMedia->format);
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("vacq_pix_format_to_vdef_raw_format", -ret);
		return ret;
	}
	mOutputMedia->info.framerate.num = mStatus.framerate_num;
	mOutputMedia->info.framerate.den =
		mStatus.framerate_den * mParams.decimation;
	mOutputMedia->info.bit_depth = mOutputMedia->format.pix_size;
	mOutputMedia->info.full_range = mStatus.full_range;
	mOutputMedia->info.color_primaries = mStatus.color_primaries;
	mOutputMedia->info.transfer_function = mStatus.transfer_function;
	mOutputMedia->info.matrix_coefs =
		VDEF_MATRIX_COEFS_UNKNOWN; /* Note: unavailable for now */
	mOutputMedia->info.dynamic_range = mStatus.dynamic_range;
	mOutputMedia->info.tone_mapping = mStatus.tone_mapping;
	mOutputMedia->info.resolution.width = mStatus.width;
	mOutputMedia->info.resolution.height = mStatus.height;
	mOutputMedia->info.sar.width = 1; /* Note: unavailable for now */
	mOutputMedia->info.sar.height = 1; /* Note: unavailable for now */
	mOutputMedia->sessionMeta = mParams.session_meta;
	mOutputMedia->playbackType = PDRAW_PLAYBACK_TYPE_LIVE;
	mOutputMedia->duration = 0;

	mParams.resolution.width = mStatus.width;
	mParams.resolution.height = mStatus.height;
	if (mParams.timescale == 0)
		mTimescale = mStatus.framerate_num;
	else
		mTimescale = mParams.timescale;
	/* Ensure that the timescale is precise enough to avoid non strictly
	 * monotonic timestamps when computing frame timestamps */
	if (mTimescale < 1000)
		mTimescale *= 1000;

	Source::unlock();

	PDRAW_LOGI("output media created");

	IPdraw::IVipcSource::Listener *listener = getVipcSourceListener();
	if (listener != nullptr) {
		listener->vipcSourceConfigured(mSession,
					       getVipcSource(),
					       0,
					       &mOutputMedia->info,
					       &mParams.crop);
	}

	if (Source::mListener) {
		Source::mListener->onOutputMediaAdded(
			this, mOutputMedia, getVipcSource());
	}

	if (!mReady) {
		mReady = true;
		if (listener != nullptr) {
			listener->vipcSourceReadyToPlay(
				mSession,
				getVipcSource(),
				mReady,
				PDRAW_VIPC_SOURCE_EOS_REASON_NONE);
		}
	}

	if (mWasRunning) {
		mWasRunning = false;
		int err = vipcc_start(mClient);
		if (err < 0)
			PDRAW_LOG_ERRNO("vipcc_start", -err);
		if (mWatchdogTimer != nullptr) {
			err = pomp_timer_set(mWatchdogTimer,
					     mParams.timeout_ms);
			if (err < 0)
				PDRAW_LOG_ERRNO("pomp_timer_set", -err);
		}
		mRunning = true;
	}

	return 0;
}


int VipcSource::destroyMedia(void)
{
	int ret;

	if ((mState != STARTED) && (mState != STOPPING) &&
	    (mState != STOPPED)) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	Source::lock();

	if (mOutputMedia == nullptr) {
		Source::unlock();
		PDRAW_LOGW("output media already destroyed");
		return 0;
	}

	if (Source::mListener) {
		Source::mListener->onOutputMediaRemoved(
			this, mOutputMedia, getVipcSource());
	}
	ret = removeOutputPort(mOutputMedia);
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("removeOutputPort", -ret);
		return ret;
	}

	delete mOutputMedia;
	mOutputMedia = nullptr;

	Source::unlock();

	PDRAW_LOGI("output media destroyed");

	if (mWasReady) {
		mWasReady = false;
		IPdraw::IVipcSource::Listener *listener =
			getVipcSourceListener();
		if ((listener != nullptr) && (mState == STARTED)) {
			listener->vipcSourceReadyToPlay(mSession,
							getVipcSource(),
							mReady,
							mLastEosReason);
		}
	}

	return 0;
}


int VipcSource::teardownChannels(void)
{
	unsigned int pendingCount = 0;

	Source::lock();

	if (mOutputMedia == nullptr) {
		Source::unlock();
		return 0;
	}

	/* Teardown the output channels
	 * Note: loop downwards because calling teardown on a channel may or
	 * may not synchronously remove the channel from the output port */

	unsigned int outputChannelCount = getOutputChannelCount(mOutputMedia);

	for (int i = outputChannelCount - 1; i >= 0; i--) {
		Channel *channel = getOutputChannel(mOutputMedia, i);
		if (channel == nullptr) {
			PDRAW_LOGW("failed to get channel at index %d", i);
			continue;
		}
		int err = channel->teardown();
		if (err < 0)
			PDRAW_LOG_ERRNO("channel::teardown", -err);
		else
			pendingCount++;
	}

	Source::unlock();

	return pendingCount;
}


void VipcSource::statusCb(struct vipcc_ctx *ctx,
			  const struct vipc_status *status,
			  void *userdata)
{
	VipcSource *self = reinterpret_cast<VipcSource *>(userdata);

	PDRAW_LOGN(
		"%s('%s'): %ux%u@%.2ffps (color=%s, transfer=%s, "
		"dynrange=%s tonemap=%s, %s range)",
		__func__,
		self->getSourceName(),
		status->width,
		status->height,
		(status->framerate_den != 0)
			? (float)status->framerate_num / status->framerate_den
			: 0.f,
		vdef_color_primaries_to_str(status->color_primaries),
		vdef_transfer_function_to_str(status->transfer_function),
		vdef_dynamic_range_to_str(status->dynamic_range),
		vdef_tone_mapping_to_str(status->tone_mapping),
		status->full_range ? "full" : "limited");

	if (self->mState != STARTED) {
		PDRAW_LOGI("%s: ignored in state %s",
			   __func__,
			   Element::getElementStateStr(self->mState));
		return;
	}

	if (self->mWatchdogTimer != nullptr) {
		int err = pomp_timer_clear(self->mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
	}

	self->mStatus = *status;

	/* TODO: apply initial resolution and crop configuration */

	if (!self->mOutputMediaChanging)
		(void)self->setupMedia();
}


/* Called on the loop thread */
void VipcSource::watchdogTimerCb(struct pomp_timer *timer, void *userdata)
{
	VipcSource *self = reinterpret_cast<VipcSource *>(userdata);

	if (self->mRunning)
		PDRAW_LOGW("timeout while collecting frames");
	else
		PDRAW_LOGW("timeout while waiting for %s",
			   self->mVipcConnected ? "status" : "connection");

	IPdraw::IVipcSource::Listener *listener = self->getVipcSourceListener();
	if (listener != nullptr) {
		listener->vipcSourceReadyToPlay(
			self->mSession,
			self->getVipcSource(),
			false,
			PDRAW_VIPC_SOURCE_EOS_REASON_TIMEOUT);
	}
}


void VipcSource::configureCb(struct vipcc_ctx *ctx,
			     const struct vipc_configure *config,
			     void *userdata)
{
	VipcSource *self = reinterpret_cast<VipcSource *>(userdata);

	if (config->flags & VIPC_CONFIGURE_FLAG_ERROR) {
		PDRAW_LOGE("%s('%s'): config error: %s%s%s%s",
			   __func__,
			   self->getSourceName(),
			   config->flags & VIPC_CONFIGURE_FLAG_RESET ? "reset "
								     : "",
			   config->flags & VIPC_CONFIGURE_FLAG_RESOLUTION
				   ? "resolution "
				   : "",
			   config->flags & VIPC_CONFIGURE_FLAG_CROP ? "crop "
								    : "",
			   config->flags & VIPC_CONFIGURE_FLAG_NOT_SUPPORTED
				   ? "not supported "
				   : "");
		return;
	}

	if (config->flags & VIPC_CONFIGURE_FLAG_RESOLUTION) {
		PDRAW_LOGN(
			"%s('%s'): resolution changed to %ux%u@%.2ffps",
			__func__,
			self->getSourceName(),
			config->resolution.width,
			config->resolution.height,
			((self->mOutputMedia != nullptr) &&
			 (self->mOutputMedia->info.framerate.den != 0))
				? (float)self->mOutputMedia->info.framerate
						  .num /
					  self->mOutputMedia->info.framerate.den
				: 0.f);
	}

	if (config->flags & VIPC_CONFIGURE_FLAG_CROP) {
		PDRAW_LOGN("%s('%s'): crop changed to %.2f,%.2f->%.2f,%.2f",
			   __func__,
			   self->getSourceName(),
			   config->crop.left,
			   config->crop.top,
			   config->crop.left + config->crop.width,
			   config->crop.top + config->crop.height);
	}

	if (self->mState != STARTED) {
		PDRAW_LOGI("%s: ignored in state %s",
			   __func__,
			   Element::getElementStateStr(self->mState));
		return;
	}

	/* TODO */
}


#	if PDRAW_VIPC_BACKEND_DMABUF || PDRAW_VIPC_BACKEND_HISI ||            \
		PDRAW_VIPC_BACKEND_NETWORK_HISI || PDRAW_VIPC_BACKEND_SHM
void VipcSource::releaseFrameCb(void *data, size_t len, void *userdata)
{
	FrameCtx *f = reinterpret_cast<FrameCtx *>(userdata);
	vipcc_release(f->client, f->frame);
	free(f);
}
#	endif


#	if PDRAW_VIPC_BACKEND_DMABUF
void VipcSource::dmabufFrameCb(struct vipcc_ctx *ctx,
			       const struct vipc_frame *frame,
			       void *be_frame,
			       void *userdata)
{
	VipcSource *self = reinterpret_cast<VipcSource *>(userdata);
	struct dmabuf_frame *dmabufFrame =
		reinterpret_cast<struct dmabuf_frame *>(be_frame);
	FrameCtx *f = nullptr;
	int err;
	struct mbuf_mem *mem = nullptr;
	size_t totalSize = 0;

	if (self->mState != STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(self->mState));
		goto unref;
	}
	if (!self->mReady) {
		PDRAW_LOGE("%s: frame received while not ready", __func__);
		goto unref;
	}
	if (!self->mRunning) {
		PDRAW_LOGE("%s: frame received while not running", __func__);
		goto unref;
	}

	/* Apply decimation */
	self->mInputFramesCount++;
	if ((self->mInputFramesCount - 1) % self->mParams.decimation != 0)
		goto unref;

	/* Wrap the frame */
	f = (FrameCtx *)calloc(1, sizeof(*f));
	if (f == nullptr) {
		PDRAW_LOG_ERRNO("calloc", ENOMEM);
		goto unref;
	}
	f->client = ctx;
	f->frame = frame;

	for (unsigned int i = 0; i < frame->num_planes; i++)
		totalSize += frame->planes[i].size;

	err = mbuf_mem_ion_wrap((void *)frame->planes[0].virt_addr,
				totalSize,
				dmabufFrame->fd[0],
				releaseFrameCb,
				f,
				&mem);
	if (err < 0) {
		PDRAW_LOG_ERRNO("mbuf_mem_ion_wrap", -err);
		goto unref;
	}

	err = self->processFrame(frame, mem);
	if (err < 0)
		goto unref;

unref:
	if (mem != nullptr) {
		err = mbuf_mem_unref(mem);
		if (err < 0)
			ULOG_ERRNO("mbuf_mem_unref", -err);
	} else {
		vipcc_release(ctx, frame);
		free(f);
	}
}
#	endif


#	if PDRAW_VIPC_BACKEND_HISI || PDRAW_VIPC_BACKEND_NETWORK_HISI
void VipcSource::hisiFrameCb(struct vipcc_ctx *ctx,
			     const struct vipc_frame *frame,
			     void *be_frame,
			     void *userdata)
{
	VipcSource *self = reinterpret_cast<VipcSource *>(userdata);
	VIDEO_FRAME_INFO_S *frameInfo =
		reinterpret_cast<VIDEO_FRAME_INFO_S *>(be_frame);
	FrameCtx *f = nullptr;
	int err;
	struct mbuf_mem *mem = nullptr;
	size_t totalSize = 0;

	if (self->mState != STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(self->mState));
		goto unref;
	}
	if (!self->mReady) {
		PDRAW_LOGE("%s: frame received while not ready", __func__);
		goto unref;
	}
	if (!self->mRunning) {
		PDRAW_LOGE("%s: frame received while not running", __func__);
		goto unref;
	}

	/* Apply decimation */
	self->mInputFramesCount++;
	if ((self->mInputFramesCount - 1) % self->mParams.decimation != 0)
		goto unref;

	/* Wrap the frame */
	f = (FrameCtx *)calloc(1, sizeof(*f));
	if (f == nullptr) {
		PDRAW_LOG_ERRNO("calloc", ENOMEM);
		goto unref;
	}
	f->client = ctx;
	f->frame = frame;

	for (unsigned int i = 0; i < frame->num_planes; i++)
		totalSize += frame->planes[i].size;

	err = mbuf_mem_hisi_wrap((void *)frame->planes[0].virt_addr,
				 totalSize,
				 frameInfo,
				 releaseFrameCb,
				 f,
				 &mem);
	if (err < 0) {
		PDRAW_LOG_ERRNO("mbuf_mem_hisi_wrap", -err);
		goto unref;
	}

	err = self->processFrame(frame, mem);
	if (err < 0)
		goto unref;

unref:
	if (mem != nullptr) {
		err = mbuf_mem_unref(mem);
		if (err < 0)
			ULOG_ERRNO("mbuf_mem_unref", -err);
	} else {
		vipcc_release(ctx, frame);
		free(f);
	}
}
#	endif


#	if PDRAW_VIPC_BACKEND_SHM
void VipcSource::shmFrameCb(struct vipcc_ctx *ctx,
			    const struct vipc_frame *frame,
			    void *be_frame,
			    void *userdata)
{
	VipcSource *self = reinterpret_cast<VipcSource *>(userdata);
	FrameCtx *f = nullptr;
	int err;
	struct mbuf_mem *mem = nullptr;
	size_t totalSize = 0;

	if (self->mState != STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(self->mState));
		goto unref;
	}
	if (!self->mReady) {
		PDRAW_LOGE("%s: frame received while not ready", __func__);
		goto unref;
	}
	if (!self->mRunning) {
		PDRAW_LOGE("%s: frame received while not running", __func__);
		goto unref;
	}

	/* Apply decimation */
	self->mInputFramesCount++;
	if ((self->mInputFramesCount - 1) % self->mParams.decimation != 0)
		goto unref;

	/* Wrap the frame */
	f = (FrameCtx *)calloc(1, sizeof(*f));
	if (f == nullptr) {
		PDRAW_LOG_ERRNO("calloc", ENOMEM);
		goto unref;
	}
	f->client = ctx;
	f->frame = frame;

	for (unsigned int i = 0; i < frame->num_planes; i++)
		totalSize += frame->planes[i].size;

	err = mbuf_mem_generic_wrap((void *)frame->planes[0].virt_addr,
				    totalSize,
				    releaseFrameCb,
				    f,
				    &mem);
	if (err < 0) {
		PDRAW_LOG_ERRNO("mbuf_mem_generic_wrap", -err);
		goto unref;
	}

	err = self->processFrame(frame, mem);
	if (err < 0)
		goto unref;

unref:
	if (mem != nullptr) {
		err = mbuf_mem_unref(mem);
		if (err < 0)
			ULOG_ERRNO("mbuf_mem_unref", -err);
	} else {
		vipcc_release(ctx, frame);
		free(f);
	}
}
#	endif


#	if PDRAW_VIPC_BACKEND_NETWORK_CBUF
void VipcSource::cbufFrameCb(struct vipcc_ctx *ctx,
			     const struct vipc_frame *frame,
			     void *be_frame,
			     void *userdata)
{
	VipcSource *self = reinterpret_cast<VipcSource *>(userdata);
	int err;
	struct mbuf_mem *mem = nullptr;
	size_t totalSize = 0, cap;
	void *data;

	if (self->mState != STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(self->mState));
		goto unref;
	}
	if (!self->mReady) {
		PDRAW_LOGE("%s: frame received while not ready", __func__);
		goto unref;
	}
	if (!self->mRunning) {
		PDRAW_LOGE("%s: frame received while not running", __func__);
		goto unref;
	}

	/* Apply decimation */
	self->mInputFramesCount++;
	if ((self->mInputFramesCount - 1) % self->mParams.decimation != 0)
		goto unref;

	/* The frame needs to have contiguous planes */
	for (unsigned int i = 0; i < frame->num_planes - 1; i++) {
		if (frame->planes[i].virt_addr + frame->planes[i].size !=
		    frame->planes[i + 1].virt_addr) {
			PDRAW_LOGW(
				"planes %d and %d are not contiguous, "
				"skip frame",
				i,
				i + 1);
			goto unref;
		}
	}

	for (unsigned int i = 0; i < frame->num_planes; i++)
		totalSize += frame->planes[i].size;

	/* Create mPool if needed */
	if (self->mPool == nullptr) {
		int err;
		size_t planeStride[VDEF_RAW_MAX_PLANE_COUNT] = {0};
		size_t planeSize[VDEF_RAW_MAX_PLANE_COUNT] = {0};
		struct vdef_raw_format format;
		struct vdef_dim resolution = {
			.width = frame->width,
			.height = frame->height,
		};
		unsigned int i;
		for (i = 0; i < VIPC_PLANE_COUNT; i++)
			planeStride[i] = frame->planes[i].stride;

		err = vacq_pix_format_to_vdef_raw_format(frame->format,
							 &format);
		if (err < 0) {
			PDRAW_LOG_ERRNO("vacq_pix_format_to_vdef_raw_format",
					-err);
			goto unref;
		}
		err = vdef_calc_raw_frame_size(&format,
					       &resolution,
					       planeStride,
					       nullptr,
					       nullptr,
					       nullptr,
					       planeSize,
					       nullptr);
		unsigned int frameSize = 0;
		for (i = 0; i < VDEF_RAW_MAX_PLANE_COUNT; i++)
			frameSize = planeSize[i];

		err = mbuf_pool_new(mbuf_mem_generic_impl,
				    frameSize,
				    DEFAULT_IN_POOL_SIZE,
				    MBUF_POOL_NO_GROW,
				    0,
				    self->getName().c_str(),
				    &self->mPool);
		if (err < 0) {
			PDRAW_LOG_ERRNO("mbuf_pool_new", -err);
			goto unref;
		}
	}

	/* Copy the frame */
	err = mbuf_pool_get(self->mPool, &mem);
	if (err < 0) {
		PDRAW_LOG_ERRNO("mbuf_pool_get", -err);
		goto unref;
	}
	err = mbuf_mem_get_data(mem, &data, &cap);
	if (err < 0) {
		PDRAW_LOG_ERRNO("mbuf_mem_get_data", -err);
		goto unref;
	}
	if (cap < totalSize) {
		PDRAW_LOGE("insufficient buffer space");
		goto unref;
	}
	memcpy(data, (void *)frame->planes[0].virt_addr, totalSize);

	err = self->processFrame(frame, mem);
	if (err < 0)
		goto unref;

unref:
	if (mem != nullptr) {
		err = mbuf_mem_unref(mem);
		if (err < 0)
			ULOG_ERRNO("mbuf_mem_unref", -err);
	}
	vipcc_release(ctx, frame);
}
#	endif


void VipcSource::connectionStatusCb(struct vipcc_ctx *ctx,
				    bool connected,
				    void *userdata)
{
	VipcSource *self = reinterpret_cast<VipcSource *>(userdata);
	PDRAW_LOGI("%s server %s",
		   connected ? "connected to" : "disconnected from",
		   self->mParams.friendly_name ? self->mParams.friendly_name
					       : self->mParams.address);
	self->mVipcConnected = connected;

	if (!connected && self->mWatchdogTimer != nullptr) {
		int err = pomp_timer_set(self->mWatchdogTimer,
					 self->mParams.timeout_ms);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set", -err);
	}
}


void VipcSource::eosCb(struct vipcc_ctx *ctx,
		       enum vipc_eos_reason reason,
		       void *userdata)
{
	VipcSource *self = reinterpret_cast<VipcSource *>(userdata);

	PDRAW_LOGN("%s('%s'): EOS reason %s (%u)",
		   __func__,
		   self->getSourceName(),
		   vipc_eos_reason_to_str(reason),
		   reason);

	if (self->mState != STARTED) {
		PDRAW_LOGI("%s: ignored in state %s",
			   __func__,
			   Element::getElementStateStr(self->mState));
		return;
	}

	if (self->mWatchdogTimer != nullptr) {
		int err = pomp_timer_clear(self->mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
	}

	self->mWasRunning = self->mRunning;
	self->mRunning = false;
	self->mWasReady = self->mReady;
	self->mReady = false;

	switch (reason) {
	default:
	case VIPC_EOS_REASON_NONE:
		self->mLastEosReason = PDRAW_VIPC_SOURCE_EOS_REASON_NONE;
		self->mFirstFrame = true;
		if (self->mOutputMedia != nullptr) {
			self->sendDownstreamEvent(
				self->mOutputMedia,
				Channel::DownstreamEvent::EOS);
		}
		break;
	case VIPC_EOS_REASON_RESTART:
		self->mLastEosReason = PDRAW_VIPC_SOURCE_EOS_REASON_RESTART;
		if (self->mOutputMedia != nullptr) {
			self->sendDownstreamEvent(
				self->mOutputMedia,
				Channel::DownstreamEvent::RECONFIGURE);
		}
		break;
	case VIPC_EOS_REASON_CONFIGURATION:
		self->mLastEosReason =
			PDRAW_VIPC_SOURCE_EOS_REASON_CONFIGURATION;
		self->mFirstFrame = true;
		if (self->mOutputMedia != nullptr) {
			self->sendDownstreamEvent(
				self->mOutputMedia,
				Channel::DownstreamEvent::EOS);
		}
		break;
	};

	/* Reset the status to avoid creating a new media */
	self->mStatus = {};
	(void)self->setupMedia();
}


const char *VipcSource::getSourceName(void)
{
	if (!mFriendlyName.empty())
		return mFriendlyName.c_str();
	if (!mAddress.empty())
		return mAddress.c_str();
	else
		return "<NULL>";
}


#	if PDRAW_VIPC_BACKEND_DMABUF
const struct vipcc_cb VipcSource::cDmabufCbs = {
	.status_cb = &VipcSource::statusCb,
	.configure_cb = &VipcSource::configureCb,
	.frame_cb = &VipcSource::dmabufFrameCb,
	.connection_status_cb = &VipcSource::connectionStatusCb,
	.eos_cb = &VipcSource::eosCb,
};
#	endif


#	if PDRAW_VIPC_BACKEND_HISI || PDRAW_VIPC_BACKEND_NETWORK_HISI
const struct vipcc_cb VipcSource::cHisiCbs = {
	.status_cb = &VipcSource::statusCb,
	.configure_cb = &VipcSource::configureCb,
	.frame_cb = &VipcSource::hisiFrameCb,
	.connection_status_cb = &VipcSource::connectionStatusCb,
	.eos_cb = &VipcSource::eosCb,
};
#	endif


#	if PDRAW_VIPC_BACKEND_SHM
const struct vipcc_cb VipcSource::cShmCbs = {
	.status_cb = &VipcSource::statusCb,
	.configure_cb = &VipcSource::configureCb,
	.frame_cb = &VipcSource::shmFrameCb,
	.connection_status_cb = &VipcSource::connectionStatusCb,
	.eos_cb = &VipcSource::eosCb,
};
#	endif


#	if PDRAW_VIPC_BACKEND_NETWORK_CBUF
const struct vipcc_cb VipcSource::cCbufCbs = {
	.status_cb = &VipcSource::statusCb,
	.configure_cb = &VipcSource::configureCb,
	.frame_cb = &VipcSource::cbufFrameCb,
	.connection_status_cb = &VipcSource::connectionStatusCb,
	.eos_cb = &VipcSource::eosCb,
};
#	endif


const VipcSource::Backend VipcSource::cBackends[BE_COUNT] = {
#	if PDRAW_VIPC_BACKEND_DMABUF
	[VipcSource::BackendType::DMABUF] = {"dmabuf",
					     &vipc_be_dmabuf_callbacks,
					     &VipcSource::cDmabufCbs},
#	endif
#	if PDRAW_VIPC_BACKEND_HISI
	[VipcSource::BackendType::HISI] = {"hisi",
					   &vipc_be_hisi_callbacks,
					   &VipcSource::cHisiCbs},
#	endif
#	if PDRAW_VIPC_BACKEND_SHM
	[VipcSource::BackendType::SHM] = {"shm",
					  &vipc_be_shm_callbacks,
					  &VipcSource::cShmCbs},
#	endif
#	if PDRAW_VIPC_BACKEND_NETWORK_HISI
	[VipcSource::BackendType::NETWORK_HISI] =
		{"network-hisi",
		 &vipc_be_network_hisi_callbacks,
		 &VipcSource::cHisiCbs},
#	endif
#	if PDRAW_VIPC_BACKEND_NETWORK_CBUF
	[VipcSource::BackendType::NETWORK_CBUF] =
		{"network-cbuf",
		 &vipc_be_network_cbuf_callbacks,
		 &VipcSource::cCbufCbs},
#	endif
};

} /* namespace Pdraw */

#endif /* BUILD_LIBVIDEO_IPC */
