/**
 * Parrot Drones Audio and Video Vector library
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

#define ULOG_TAG pdraw_vipc_source
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include <libmp4.h>
#include <time.h>

#include <new>

#include "pdraw_session.hpp"
#include "pdraw_vipc_source.hpp"

#ifdef BUILD_LIBVIDEO_IPC
#	if PDRAW_VIPC_BACKEND_HISI || PDRAW_VIPC_BACKEND_NETWORK_HISI
#		include <hi_buffer.h>
#		include <mpi_sys.h>
#		include <mpi_vb.h>
#		include <mpi_venc.h>
#		include <media-buffers/mbuf_mem_hisi.h>
#	endif

#	if PDRAW_VIPC_BACKEND_DMABUF
#		include <media-buffers/mbuf_mem_ion.h>
#		include <media-buffers/mbuf_mem_vacq.h>
#		include <linux/ion.h>
#		include <sys/ioctl.h>
#		define ION_BIT(nr) (1U << (nr))
#		define ION_HEAP(bit) ION_BIT(bit)
#		define ION_SECURE_HEAP_ID 9
#		define ION_SYSTEM_HEAP_ID 25
#	endif

constexpr const char *PDRAW_VIPC_SOURCE_ANCILLARY_KEY_INPUT_TIME =
	"pdraw.vipcsource.input_time";

constexpr const char *DEFAULT_BACKEND_NAME = "default";
constexpr size_t DEFAULT_VIPC_MAX_FRAMES = 30;
constexpr size_t DEFAULT_TIMEOUT_MS = 2000;
#	if PDRAW_VIPC_BACKEND_NETWORK_CBUF
constexpr size_t DEFAULT_IN_POOL_SIZE = 10;
#	endif
#endif

namespace Pdraw {


#ifdef BUILD_LIBVIDEO_IPC

/* TODO: Channel::DownstreamEvent::TIMEOUT? */

template <typename T> inline unsigned int getNumPlanes(const T *obj)
{
	return std::min(obj->num_planes,
			static_cast<unsigned int>(VIPC_PLANE_COUNT));
}


VipcSource::VipcSource(Session *session,
		       Element::Listener *elementListener,
		       Source::Listener *sourceListener,
		       IPdraw::IVipcSource::Listener *listener,
		       VipcSourceWrapper *wrapper,
		       const struct pdraw_vipc_source_params *params) :
		SourceElement(session,
			      elementListener,
			      wrapper,
			      1,
			      sourceListener),
		mVipcSource(wrapper), mVipcSourceListener(listener)
{
	Element::setClassName(__func__);

	struct pdraw_vipc_source_params *rawParams =
		pdraw_vipcSourceParamsDup(params);
	if (!rawParams) {
		ULOGE("pdraw_vipcSourceParamsDup");
		throw std::bad_alloc();
	}
	mParams.reset(rawParams);

	if (mParams->frame_count == 0)
		mParams->frame_count = DEFAULT_VIPC_MAX_FRAMES;
	if ((mParams->crop.left == 0.f) && (mParams->crop.top == 0.f) &&
	    (mParams->crop.width == 0.f) && (mParams->crop.height == 0.f)) {
		mParams->crop.left = mParams->crop.top = 0.f;
		mParams->crop.width = mParams->crop.height = 1.f;
	}
	if (mParams->decimation == 0)
		mParams->decimation = 1;
	if (mParams->connection_timeout_ms == 0)
		mParams->connection_timeout_ms = DEFAULT_TIMEOUT_MS;
	if (mParams->frame_timeout_ms == 0)
		mParams->frame_timeout_ms = DEFAULT_TIMEOUT_MS;

	setState(State::CREATED);
}


VipcSource::~VipcSource()
{
	int err;

	/* Make sure listener functions will no longer be called */
	mVipcSourceListener = nullptr;

	if (mState == State::STARTED)
		PDRAW_LOGW("VIPC source is still running");

	/* Remove any leftover idle callbacks */
	err = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

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

	vipc_status_free(mStatus);
}


bool VipcSource::isMbufMemImplemSupported(
	const enum mbuf_mem_implem_type *supportedMemImplems,
	size_t supportedMemImplemCount,
	enum mbuf_mem_implem_type implem,
	enum BackendType type)
{
	bool found = false;

	for (size_t i = 0; i < supportedMemImplemCount; i++) {
		if (supportedMemImplems[i] == implem) {
			found = true;
			break;
		}
	}
	if (!found)
		return false;

	switch (type) {
#	if PDRAW_VIPC_BACKEND_DMABUF
	case VipcSource::BackendType::DMABUF:
		return ((implem == MBUF_MEM_IMPLEM_TYPE_ION) ||
			(implem == MBUF_MEM_IMPLEM_TYPE_VACQ));
#	endif
#	if PDRAW_VIPC_BACKEND_SHM
	case BackendType::SHM:
		return (implem == MBUF_MEM_IMPLEM_TYPE_GENERIC);
#	endif
#	if PDRAW_VIPC_BACKEND_HISI
	case BackendType::HISI:
		return (implem == MBUF_MEM_IMPLEM_TYPE_HISI);
#	endif
#	if PDRAW_VIPC_BACKEND_NETWORK_CBUF
	case BackendType::NETWORK_CBUF:
		return (implem == MBUF_MEM_IMPLEM_TYPE_GENERIC);
#	endif
	default:
		return false;
	}
}


int VipcSource::start(void)
{
	int ret;
	const enum mbuf_mem_implem_type *supportedMemImplems = nullptr;

	if ((mState == State::STARTED) || (mState == State::STARTING))
		return 0;
	if (mState != State::CREATED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}
	setState(State::STARTING);

	const Backend *backend =
		getBackend(mParams->backend_name, true, &mBackendType);
	if (backend == nullptr) {
		PDRAW_LOGE("%s: unable to select backend '%s'",
			   __func__,
			   mParams->backend_name);
		ret = -ENOSYS;
		goto error;
	}

	if (mParams->mem_implem != MBUF_MEM_IMPLEM_TYPE_AUTO) {
		ret = mbuf_get_supported_implems(&supportedMemImplems);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_get_supported_implems", -ret);
			goto error;
		}
		int nbImplems = ret;
		if (!isMbufMemImplemSupported(supportedMemImplems,
					      nbImplems,
					      mParams->mem_implem,
					      mBackendType)) {
			PDRAW_LOGE("unsupported implem: '%s'",
				   mbuf_mem_implem_type_to_str(
					   mParams->mem_implem));
			ret = -EPROTO;
			goto error;
		}
	}

	/* Watchdog is only created if timeout is specified */
	if (mWatchdogTimer == nullptr &&
	    (mParams->connection_timeout_ms != -1 ||
	     mParams->frame_timeout_ms != -1)) {
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
			     mParams->address,
			     mParams->friendly_name,
			     this,
			     mParams->frame_count,
			     true);
	if (mClient == nullptr) {
		PDRAW_LOGE("%s: failed to create VIPC client", __func__);
		ret = -ENOMEM;
		goto error;
	}

	if (mWatchdogTimer != nullptr && mParams->connection_timeout_ms != -1) {
		ret = pomp_timer_set(mWatchdogTimer,
				     mParams->connection_timeout_ms);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("pomp_timer_set", -ret);
			goto error;
		}
	}

	setState(State::STARTED);
	return 0;

error:
	(void)stop();
	return ret;
}


int VipcSource::stop()
{
	int ret;
	int err;

	if ((mState == State::STOPPED) || (mState == State::STOPPING))
		return 0;
	if ((mState != State::STARTED) && (mState != State::STARTING)) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	setState(State::STOPPING);

	/* Make sure listener functions will no longer be called */
	mVipcSourceListener = nullptr;

	Source::lock();
	if (mOutputMedia != nullptr)
		mOutputMedia->setTearingDown();
	Source::unlock();

	if (mRunning) {
		err = vipcc_stop(mClient);
		if (err < 0)
			PDRAW_LOG_ERRNO("vipcc_stop", -err);
		mRunning = false;
	}

	/* Flush everything */
	ret = flush();
	if (ret < 0 && ret != -EALREADY)
		PDRAW_LOG_ERRNO("flush", -ret);
	else
		ret = 0;

	/* When the flush is complete, stopping will be triggered */
	return ret;
}


int VipcSource::tryStop()
{
	int pendingCount;

	if (mState != State::STOPPING)
		return 0;

	pendingCount = teardownChannels();

	if (pendingCount == 0)
		completeStop();

	return 0;
}


void VipcSource::completeStop()
{
	int err;
	unsigned int outputChannelCount;

	Source::lock();

	if (mOutputMedia == nullptr)
		goto exit;

	outputChannelCount = getOutputChannelCount(mOutputMedia.get());
	if (outputChannelCount > 0) {
		Source::unlock();
		return;
	}

	if (mUsedFrameCount > 0) {
		PDRAW_LOGI(
			"%s: %u frame(s) still not released,"
			" trying again later",
			__func__,
			mUsedFrameCount.load());
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

	if (mClient != nullptr) {
		err = vipcc_destroy(mClient);
		if (err < 0)
			PDRAW_LOG_ERRNO("vipcc_destroy", -err);
		mClient = nullptr;
	}

	if (mWatchdogTimer != nullptr) {
		err = pomp_timer_clear(mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		err = pomp_timer_destroy(mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_destroy", -err);
		mWatchdogTimer = nullptr;
	}

	setState(State::STOPPED);
}


void VipcSource::playResponse()
{
	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callPlayResponse, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


void VipcSource::pauseResponse()
{
	int err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callPauseResponse, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
}


void VipcSource::onChannelUnlink(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Source::onChannelUnlink(channel);

	if (mState == State::STOPPING) {
		completeStop();
		return;
	}

	if (mOutputMediaChanging) {
		Source::lock();

		unsigned int outputChannelCount =
			getOutputChannelCount(mOutputMedia.get());
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


int VipcSource::flush(bool discard)
{
	int err;
	int ret = 0;
	bool channelFound = false;
	Channel *outputChannel;

	switch (getFlushingState()) {
	case FlushingState::UNFLUSHED:
		/* OK */
		break;
	case FlushingState::FLUSHING:
		return -EALREADY;
	case FlushingState::FLUSHED:
		PDRAW_LOGD("vipc source is already flushed, nothing to do");
		ret = pomp_loop_idle_add_with_cookie(
			mSession->getLoop(), &idleCompleteFlush, this, this);
		if (ret < 0)
			PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -ret);
		else
			setFlushingState(FlushingState::FLUSHING, discard);
		return ret;
	default:
		break;
	}

	setFlushingState(FlushingState::FLUSHING, discard);

	/* Flush the output channels (async) */
	Source::lock();
	if (mOutputMedia != nullptr) {
		unsigned int outputChannelCount =
			getOutputChannelCount(mOutputMedia.get());
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia.get(), i);
			if (outputChannel == nullptr) {
				PDRAW_LOGW(
					"failed to get output channel "
					"at index %d",
					i);
				continue;
			}
			if (mFlushDiscard)
				err = outputChannel->flush();
			else
				err = outputChannel->drain();
			if (err < 0 && err != -EALREADY) {
				PDRAW_LOG_ERRNO(
					"channel->%s (channel index=%u)",
					-err,
					mFlushDiscard ? "flush" : "drain",
					i);
			} else {
				channelFound = true;
			}
		}
	}
	Source::unlock();

	if (!channelFound)
		completeFlush();

	return 0;
}


void VipcSource::completeFlush()
{
	bool pending = false;

	Source::lock();
	if (mOutputMedia != nullptr) {
		unsigned int outputChannelCount =
			getOutputChannelCount(mOutputMedia.get());
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			const Channel *outputChannel =
				getOutputChannel(mOutputMedia.get(), i);
			if (outputChannel == nullptr) {
				PDRAW_LOGW(
					"failed to get output channel "
					"at index %d",
					i);
				continue;
			}
			if (outputChannel->isFlushPending() ||
			    outputChannel->isDrainPending()) {
				pending = true;
				break;
			}
		}
	}
	Source::unlock();

	if (pending)
		return;

	setFlushingState(FlushingState::FLUSHED);

	if (mPausePending)
		pauseResponse();

	if ((mState != State::STOPPING) && (!mOutputMediaChanging))
		return;

	int pendingCount = teardownChannels();
	if (pendingCount == 0) {
		if (mState == State::STOPPING) {
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

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		return;
	}
	PDRAW_LOGD("'%s': channel flushed media name=%s (channel owner=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getOwner());

	completeFlush();
}


void VipcSource::onChannelDrained(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	const Media *media = getOutputMediaFromChannel(channel);
	if (media == nullptr) {
		PDRAW_LOGE("%s: output media not found", __func__);
		return;
	}
	PDRAW_LOGD("'%s': channel drained media name=%s (channel owner=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getOwner());

	completeFlush();
}


bool VipcSource::isReadyToPlay() const
{
	return mReady;
}


bool VipcSource::isPaused() const
{
	return !mRunning;
}


int VipcSource::play()
{
	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}
	if (!mReady) {
		PDRAW_LOGE("%s: not ready to play", __func__);
		return -EPROTO;
	}

	if (mPausePending)
		return -EBUSY;

	if (mRunning)
		return 0;

	int ret = vipcc_start(mClient);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vipcc_start", -ret);
		return ret;
	}

	if (mWatchdogTimer != nullptr && mParams->frame_timeout_ms != -1) {
		int err = pomp_timer_set(mWatchdogTimer,
					 mParams->frame_timeout_ms);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set", -err);
	}

	mRunning = true;
	mPushedFrameCount = 0;

	playResponse();

	return 0;
}


int VipcSource::pause()
{
	int ret;
	int err;

	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	if (mPausePending)
		return -EALREADY;

	if (mReady && mRunning) {
		err = vipcc_stop(mClient);
		if (err < 0)
			PDRAW_LOG_ERRNO("vipcc_stop", -err);

		if (mWatchdogTimer != nullptr) {
			err = pomp_timer_clear(mWatchdogTimer);
			if (err < 0)
				PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
		}
		mRunning = false;
	}

	/* Drain */
	ret = drain();
	if (ret < 0 && ret != -EALREADY) {
		PDRAW_LOG_ERRNO("drain", -ret);
		return ret;
	}

	mPausePending = true;

	return 0;
}


int VipcSource::configure(const struct vdef_dim *resolution,
			  const struct vdef_rectf *crop) const
{
	PDRAW_UNUSED(resolution);
	PDRAW_UNUSED(crop);

	/* TODO */
	return -ENOSYS;
}


int VipcSource::insertGreyFrame(uint64_t tsUs)
{
	int ret;
	int err;
	struct mbuf_raw_video_frame *frame = nullptr;
	unsigned int outputChannelCount;
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	struct vdef_raw_frame frameInfo = {};
	RawVideoMedia::Frame out_meta{};
	size_t frameSize = 0;
	struct mbuf_mem *mem = nullptr;
	uint8_t *data = nullptr;
	size_t size = 0;
	size_t offset = 0;

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mStatus == nullptr, EPROTO);

	ret = vacq_pix_format_to_vdef_raw_format(mStatus->format,
						 &frameInfo.format);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vacq_pix_format_to_vdef_raw_format", -ret);
		goto out;
	}
	vdef_format_to_frame_info(&mOutputMedia->info, &frameInfo.info);
	frameInfo.info.timestamp = mp4_usec_to_sample_time(tsUs, mTimescale);
	frameInfo.info.timescale = mTimescale;
	frameInfo.info.capture_timestamp = 0;
	/* Indicate that the frame is fake */
	frameInfo.info.flags = VDEF_FRAME_FLAG_FAKE;
	frameInfo.info.index = mNextFrameIndex;
	frameInfo.info.full_range = mStatus->full_range;
	frameInfo.info.resolution.width = mStatus->width;
	frameInfo.info.resolution.height = mStatus->height;
	for (size_t i = 0; i < getNumPlanes(mStatus); i++) {
		frameInfo.plane_stride[i] = mStatus->planes[i].stride;
		frameSize += mStatus->planes[i].size;
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

	ret = mbuf_raw_video_frame_new(&frameInfo, &frame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_new", -ret);
		goto out;
	}

	switch (mBackendType) {
#	if PDRAW_VIPC_BACKEND_DMABUF
	case BackendType::DMABUF: {
		struct mbuf_ion_heap_attr heapAttrs = {};
		heapAttrs.heap_id_mask = ION_HEAP(ION_SECURE_HEAP_ID) |
					 ION_HEAP(ION_SYSTEM_HEAP_ID);
		heapAttrs.flags = ION_FLAG_CACHED;
		ret = mbuf_mem_ion_new(&heapAttrs, frameSize, &mem);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_mem_ion_new", -ret);
			goto out;
		}
		break;
	}
#	endif
#	if PDRAW_VIPC_BACKEND_SHM || PDRAW_VIPC_BACKEND_NETWORK_CBUF
	case BackendType::SHM: {
		ret = mbuf_mem_generic_new(frameSize, &mem);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_mem_generic_new", -ret);
			goto out;
		}
		break;
	}
#	endif
#	if PDRAW_VIPC_BACKEND_HISI || PDRAW_VIPC_BACKEND_NETWORK_HISI
	case BackendType::HISI: {
		/* TODO: single hisi memory is not supported, a pool should be
		 * created */
		ret = -ENOSYS;
		goto out;
	}
#	endif
	default:
		ret = -ENOSYS;
		goto out;
	}

#	pragma GCC diagnostic push
#	pragma GCC diagnostic ignored "-Wunreachable-code"
	/* coverity[unreachable] */
	ret = mbuf_mem_get_data(mem, reinterpret_cast<void **>(&data), &size);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_mem_get_data", -ret);
		goto out;
	}
	if (size == 0 || size < frameSize) {
		PDRAW_LOGE("insufficient buffer space");
		goto out;
	}

	/* Fill the frame with grey */
	if (vdef_raw_format_cmp(&frameInfo.format, &vdef_nv12) ||
	    vdef_raw_format_cmp(&frameInfo.format, &vdef_nv21) ||
	    vdef_raw_format_cmp(&frameInfo.format, &vdef_i420)) {
		memset(data, 0x80, size);
	} else if (vdef_raw_format_cmp(&frameInfo.format, &vdef_raw16)) {
		const uint16_t RAW16_GREY = 0x8000;
		for (size_t i = 0; (i + 1) < size; i += 2) {
			memcpy(data + i, &RAW16_GREY, sizeof(RAW16_GREY));
		}
	} else {
		ret = -ENOSYS;
		goto out;
	}

	for (size_t i = 0; i < getNumPlanes(mStatus); i++) {
		size_t planeSize = mStatus->planes[i].size;
		ret = mbuf_raw_video_frame_set_plane(
			frame, i, mem, offset, planeSize);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_set_plane", -ret);
			goto out;
		}
		offset += planeSize;
	}

	/* Set the ancillary data */
	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTime);
	err = mbuf_raw_video_frame_add_ancillary_buffer(
		frame,
		PDRAW_VIPC_SOURCE_ANCILLARY_KEY_INPUT_TIME,
		&curTime,
		sizeof(curTime));
	if (err < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
				-err);
	}

	out_meta.ntpTimestamp = 0;
	out_meta.ntpUnskewedTimestamp = 0;
	out_meta.ntpRawTimestamp = 0;
	out_meta.ntpRawUnskewedTimestamp = 0;
	out_meta.demuxOutputTimestamp = curTime;
	out_meta.playTimestamp = 0;
	out_meta.captureTimestamp = 0;
	out_meta.localTimestamp = 0;

	ret = mbuf_raw_video_frame_add_ancillary_buffer(
		frame,
		PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
				-ret);
		goto out;
	}

	ret = mbuf_raw_video_frame_finalize(frame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_finalize", -ret);
		goto out;
	}

	Source::lock();

	/* Queue the frame in the output channels */
	outputChannelCount = getOutputChannelCount(mOutputMedia.get());
	for (unsigned int i = 0; i < outputChannelCount; i++) {
		Channel *c = getOutputChannel(mOutputMedia.get(), i);
		auto *channel = dynamic_cast<RawVideoChannel *>(c);
		if (channel == nullptr) {
			PDRAW_LOGE("failed to get channel at index %d", i);
			continue;
		}
		err = channel->queue(frame);
		if (err < 0)
			PDRAW_LOG_ERRNO("channel->queue", -err);
		else
			setFlushingState(FlushingState::UNFLUSHED);
	}

	/* Save frame timestamp as last_timestamp */
	mLastTimestamp = frameInfo.info.timestamp;
	/* Increment next frame index */
	mNextFrameIndex++;

	Source::unlock();

	ret = 0;

out:
	if (frame != nullptr) {
		err = mbuf_raw_video_frame_unref(frame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_unref", -err);
	}
	if (mem != nullptr) {
		err = mbuf_mem_unref(mem);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_mem_unref", -err);
	}
	return ret;
#	pragma GCC diagnostic pop
}


int VipcSource::setSessionMetadata(const struct vmeta_session *meta)
{
	if (meta == nullptr)
		return -EINVAL;

	mParams->session_meta = *meta;

	Source::lock();
	if (mOutputMedia != nullptr) {
		mOutputMedia->sessionMeta = mParams->session_meta;
		int err = sendDownstreamEvent(
			mOutputMedia.get(),
			Channel::DownstreamEvent::SESSION_META_UPDATE);
		if (err < 0)
			PDRAW_LOG_ERRNO("sendDownstreamEvent", -err);
	}
	Source::unlock();

	return 0;
}


int VipcSource::getSessionMetadata(struct vmeta_session *meta) const
{
	if (meta == nullptr)
		return -EINVAL;

	*meta = mParams->session_meta;

	return 0;
}


void VipcSource::incrementUsedFrameCount()
{
	mUsedFrameCount++;
}


void VipcSource::decrementUsedFrameCount()
{
	if (mUsedFrameCount <= 0) {
		PDRAW_LOGE("%s: mUsedFrameCount <= 0 (%d)",
			   __func__,
			   mUsedFrameCount.load());
	} else {
		mUsedFrameCount--;
	}
}


int VipcSource::processFrame(const struct vipc_frame *vipcFrame,
			     struct mbuf_mem *mem)
{
	int ret;
	int err;
	struct mbuf_raw_video_frame *mbufFrame = nullptr;
	struct vdef_raw_frame frameInfo = {};
	struct vdef_raw_format format = {};
	RawVideoMedia::Frame out_meta{};
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	unsigned int outputChannelCount;

	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	if (pushLimitReached()) {
		ret = -EPROTO;
		PDRAW_LOGW(
			"%s: push limit reached (%u/%u frames), dropping frame",
			__func__,
			mPushedFrameCount,
			mParams->max_pushed_frame_count);
		goto out;
	}

	if (mWatchdogTimer != nullptr && mParams->frame_timeout_ms != -1) {
		err = pomp_timer_set(mWatchdogTimer, mParams->frame_timeout_ms);
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
		err = sendDownstreamEvent(mOutputMedia.get(),
					  Channel::DownstreamEvent::SOS);
		if (err < 0)
			PDRAW_LOG_ERRNO("sendDownstreamEvent", -err);
	}

	ret = vacq_pix_format_to_vdef_raw_format(vipcFrame->format,
						 &frameInfo.format);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vacq_pix_format_to_vdef_raw_format", -ret);
		goto out;
	}
	vdef_format_to_frame_info(&mOutputMedia->info, &frameInfo.info);
	frameInfo.info.timestamp =
		((vipcFrame->ts_sof_ns / 1000) * mTimescale + 500000) / 1000000;
	frameInfo.info.timescale = mTimescale;
	frameInfo.info.capture_timestamp = vipcFrame->ts_sof_ns / 1000;
	frameInfo.info.index = mNextFrameIndex;
	frameInfo.info.full_range =
		(vipcFrame->meta.flags & VIPC_META_FLAG_FULL_RANGE);
	frameInfo.info.resolution.width = vipcFrame->width;
	frameInfo.info.resolution.height = vipcFrame->height;
	for (unsigned int i = 0; i < getNumPlanes(vipcFrame); i++)
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

	/* Create the raw video frame */
	ret = mbuf_raw_video_frame_new(&frameInfo, &mbufFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_new", -ret);
		goto out;
	}
	for (unsigned int i = 0; i < getNumPlanes(vipcFrame); i++) {
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
	if (mVipcSourceListener != nullptr) {
		mVipcSourceListener->vipcSourceFrameReady(
			mSession, getVipcSource(), mbufFrame);
	}

	/* Note: the vipcSourceFrameReady listener may call the insertGreyFrame
	 * API function, leading to non-monotonic frame indexes.
	 * Update the currently processed frame index if necessary before
	 * finalizing the frame. */
	if (mNextFrameIndex > frameInfo.info.index) {
		frameInfo.info.index = mNextFrameIndex;
		ret = mbuf_raw_video_frame_set_frame_info(mbufFrame,
							  &frameInfo);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_raw_video_frame_set_frame_info",
					-ret);
			goto out;
		}
	}

	/* Finalize the frame */
	ret = mbuf_raw_video_frame_finalize(mbufFrame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_finalize", -ret);
		goto out;
	}

	/* Queue the frame in the output channels */
	outputChannelCount = getOutputChannelCount(mOutputMedia.get());
	for (unsigned int i = 0; i < outputChannelCount; i++) {
		Channel *c = getOutputChannel(mOutputMedia.get(), i);
		auto *channel = dynamic_cast<RawVideoChannel *>(c);
		if (channel == nullptr) {
			PDRAW_LOGE("failed to get channel at index %d", i);
			continue;
		}
		err = channel->queue(mbufFrame);
		if (err < 0)
			PDRAW_LOG_ERRNO("channel->queue", -err);
		else
			setFlushingState(FlushingState::UNFLUSHED);
	}

	/* Save frame timestamp as last_timestamp */
	mLastTimestamp = frameInfo.info.timestamp;
	/* Increment next frame index */
	mNextFrameIndex++;

	mPushedFrameCount++;
	if (pushLimitReached()) {
		PDRAW_LOGI("required frame count reached (%u), pausing source",
			   mPushedFrameCount);
		pause();
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
#	pragma GCC diagnostic push
#	pragma GCC diagnostic ignored "-Wunreachable-code"
	/* coverity[unreachable] */
	if (!name || strcmp(name, DEFAULT_BACKEND_NAME) == 0)
		goto default_backend;

	for (size_t i = 0; i < BE_COUNT; i++) {
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
#	pragma GCC diagnostic pop
}


int VipcSource::setupMedia()
{
	bool validStatus = (mStatus != nullptr);

	if (mState == State::STOPPED)
		return 0;

	if (mState != State::STARTED) {
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

	if ((mOutputMedia != nullptr) &&
	    (!validStatus || (validStatus && !mKeepMedia))) {
		if (!mKeepMedia)
			mOutputMediaChanging = true;
		int ret = flush();
		if (ret < 0 && ret != -EALREADY)
			PDRAW_LOG_ERRNO("flush", -ret);
		else
			ret = 0;
		Source::unlock();
		return ret;
	}

	Source::unlock();

	if (validStatus && mStatus->width > 0 && mStatus->height > 0)
		return createMedia();
	else
		return 0;
}


/**
 * Vipc source listener calls from idle functions
 */
void VipcSource::callOnMediaAdded(void *userdata)
{
	auto *self = static_cast<VipcSource *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mOutputMedia == nullptr) {
		PDRAW_LOGW("%s: output media not found", __func__);
		return;
	}

	if (self->Source::mListener) {
		self->Source::mListener->onOutputMediaAdded(
			self, self->mOutputMedia.get(), self->getVipcSource());
	}
}


void VipcSource::idleCompleteFlush(void *userdata)
{
	auto *self = static_cast<VipcSource *>(userdata);
	(void)self->completeFlush();
}


void VipcSource::idleCompleteStop(void *userdata)
{
	auto *self = static_cast<VipcSource *>(userdata);
	(void)self->completeStop();
}


/* Listener call from an idle function */
void VipcSource::callPlayResponse(void *userdata)
{
	auto *self = static_cast<VipcSource *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mVipcSourceListener != nullptr) {
		self->mVipcSourceListener->vipcSourcePlayResponse(
			self->mSession, self->getVipcSource());
	}
}


/* Listener call from an idle function */
void VipcSource::callPauseResponse(void *userdata)
{
	auto *self = static_cast<VipcSource *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	self->mPausePending = false;

	if (self->mVipcSourceListener != nullptr) {
		self->mVipcSourceListener->vipcSourcePauseResponse(
			self->mSession, self->getVipcSource());
	}
}


int VipcSource::createMedia()
{
	int ret;
	int err;
	std::string path;

	if (mState != State::STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(mStatus == nullptr, EPROTO);

	Source::lock();

	if (mOutputMedia != nullptr) {
		if (mKeepMedia)
			goto media_created;

		Source::unlock();
		PDRAW_LOGE("already existing output media");
		return -EALREADY;
	}

	try {
		mOutputMedia = make_unique<RawVideoMedia>(mSession);
	} catch (const std::bad_alloc &) {
		Source::unlock();
		PDRAW_LOGE("output media allocation failed");
		return -ENOMEM;
	}
	path = Element::getName() + "$" + mOutputMedia->getName();
	mOutputMedia->setPath(path);

	ret = addOutputPort(mOutputMedia.get());
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("addOutputPort", -ret);
		return ret;
	}

	ret = vacq_pix_format_to_vdef_raw_format(mStatus->format,
						 &mOutputMedia->format);
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("vacq_pix_format_to_vdef_raw_format", -ret);
		return ret;
	}
	mOutputMedia->info.framerate.num = mStatus->framerate_num;
	mOutputMedia->info.framerate.den =
		mStatus->framerate_den * mParams->decimation;
	mOutputMedia->info.bit_depth = mOutputMedia->format.pix_size;
	mOutputMedia->info.full_range = mStatus->full_range;
	mOutputMedia->info.color_primaries = mStatus->color_primaries;
	mOutputMedia->info.transfer_function = mStatus->transfer_function;
	mOutputMedia->info.matrix_coefs =
		VDEF_MATRIX_COEFS_UNKNOWN; /* Note: unavailable for now */
	mOutputMedia->info.dynamic_range = mStatus->dynamic_range;
	mOutputMedia->info.tone_mapping = mStatus->tone_mapping;
	mOutputMedia->info.resolution.width = mStatus->width;
	mOutputMedia->info.resolution.height = mStatus->height;
	mOutputMedia->info.sar.width = 1; /* Note: unavailable for now */
	mOutputMedia->info.sar.height = 1; /* Note: unavailable for now */
	mOutputMedia->sessionMeta = mParams->session_meta;
	mOutputMedia->playbackType = PDRAW_PLAYBACK_TYPE_LIVE;
	mOutputMedia->duration = 0;

	mParams->resolution.width = mStatus->width;
	mParams->resolution.height = mStatus->height;
	if (mParams->timescale == 0)
		mTimescale = mStatus->framerate_num;
	else
		mTimescale = mParams->timescale;
	/* Ensure that the timescale is precise enough to avoid non strictly
	 * monotonic timestamps when computing frame timestamps */
	if (mTimescale < 1000)
		mTimescale *= 1000;

	Source::unlock();

	PDRAW_LOGI("output media created");

	if (mVipcSourceListener != nullptr) {
		mVipcSourceListener->vipcSourceConfigured(mSession,
							  getVipcSource(),
							  0,
							  &mOutputMedia->info,
							  &mParams->crop);
	}

	if (Source::mListener) {
		/* Call the onMediaAdded listener function from a fresh
		 * callstack as a direct call could ultimately be blocking
		 * in a pdraw-backend application calling another pdraw-backend
		 * function from the onMediaAdded listener function */
		err = pomp_loop_idle_add_with_cookie(
			mSession->getLoop(), callOnMediaAdded, this, this);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	}

media_created:
	if (!mReady) {
		mReady = true;
		if (mVipcSourceListener != nullptr) {
			mVipcSourceListener->vipcSourceReadyToPlay(
				mSession,
				getVipcSource(),
				mReady,
				PDRAW_VIPC_SOURCE_EOS_REASON_NONE);
		}
	}

	if (mWasRunning && mRunning) {
		mWasRunning = false;
		err = vipcc_start(mClient);
		if (err < 0)
			PDRAW_LOG_ERRNO("vipcc_start", -err);
		if (mWatchdogTimer != nullptr &&
		    mParams->connection_timeout_ms != -1) {
			err = pomp_timer_set(mWatchdogTimer,
					     mParams->connection_timeout_ms);
			if (err < 0)
				PDRAW_LOG_ERRNO("pomp_timer_set", -err);
		}
		mRunning = true;
	}

	return 0;
}


int VipcSource::destroyMedia()
{
	int ret;

	if ((mState != State::STARTED) && (mState != State::STOPPING) &&
	    (mState != State::STOPPED)) {
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
			this, mOutputMedia.get(), getVipcSource());
	}
	ret = removeOutputPort(mOutputMedia.get());
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("removeOutputPort", -ret);
		return ret;
	}

	mOutputMedia.reset();

	Source::unlock();

	PDRAW_LOGI("output media destroyed");

	if (mWasReady) {
		mWasReady = false;
		if ((mVipcSourceListener != nullptr) &&
		    (mState == State::STARTED)) {
			mVipcSourceListener->vipcSourceReadyToPlay(
				mSession,
				getVipcSource(),
				mReady,
				mLastEosReason);
		}
	}

	return 0;
}


int VipcSource::teardownChannels()
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

	int outputChannelCount = getOutputChannelCount(mOutputMedia.get());

	for (int i = outputChannelCount - 1; i >= 0; i--) {
		Channel *channel = getOutputChannel(mOutputMedia.get(), i);
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
	PDRAW_UNUSED(ctx);

	int err;
	auto *self = static_cast<VipcSource *>(userdata);
	unsigned int gcd;

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

	if (self->mState != State::STARTED) {
		PDRAW_LOGI("%s: ignored in state %s",
			   __func__,
			   Element::getElementStateStr(self->mState));
		return;
	}

	if (self->mStatus != nullptr) {
		struct vipc_status statusCopy = *self->mStatus;

		/* Check for framerate change only */
		statusCopy.framerate_num = status->framerate_num;
		statusCopy.framerate_den = status->framerate_den;

		if (vipc_status_cmp(self->mStatus, status)) {
			PDRAW_LOGW("%s: ignored (identical)", __func__);
			self->mWasRunning = self->mRunning;
			self->mRunning = false;
			goto ignore;
		} else if (self->mVipcSourceListener != nullptr &&
			   vipc_status_cmp(&statusCopy, status)) {
			const struct vdef_frac prevFramerate = {
				.num = self->mStatus->framerate_num,
				.den = self->mStatus->framerate_den,
			};
			const struct vdef_frac newFramerate = {
				.num = status->framerate_num,
				.den = status->framerate_den,
			};
			bool ignore = self->mVipcSourceListener
					      ->vipcSourceFramerateChanged(
						      self->mSession,
						      self->getVipcSource(),
						      &prevFramerate,
						      &newFramerate);
			if (ignore) {
				PDRAW_LOGI(
					"%s: ignored "
					"(framerate change "
					"from %u/%u to %u/%u)",
					__func__,
					prevFramerate.num,
					prevFramerate.den,
					newFramerate.num,
					newFramerate.den);
				self->mStatus->framerate_num =
					status->framerate_num;
				self->mStatus->framerate_den =
					status->framerate_den;
				self->mWasRunning = self->mRunning;
				self->mRunning = false;
				goto ignore;
			}
		}
	}

	if (self->mWatchdogTimer != nullptr) {
		err = pomp_timer_clear(self->mWatchdogTimer);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_clear", -err);
	}

	/* vipc_status deep copy */
	vipc_status_free(self->mStatus);
	self->mStatus = nullptr;
	err = vipc_status_copy(status, &self->mStatus);
	if (err < 0) {
		ULOG_ERRNO("vipc_status_copy", -err);
		return;
	}

	/* Ensure that the framerate fraction is reduced */
	gcd = pdraw_gcd(self->mStatus->framerate_num,
			self->mStatus->framerate_den);
	if (gcd > 0) {
		self->mStatus->framerate_num /= gcd;
		self->mStatus->framerate_den /= gcd;
	}

	/* TODO: apply initial resolution and crop configuration */

	if (!self->mOutputMediaChanging)
		(void)self->setupMedia();

	return;

ignore:
	if (self->mWasRunning) {
		self->mWasRunning = false;
		err = vipcc_start(self->mClient);
		if (err < 0)
			PDRAW_LOG_ERRNO("vipcc_start", -err);
		if (self->mWatchdogTimer != nullptr &&
		    self->mParams->connection_timeout_ms != -1) {
			err = pomp_timer_set(
				self->mWatchdogTimer,
				self->mParams->connection_timeout_ms);
			if (err < 0)
				PDRAW_LOG_ERRNO("pomp_timer_set", -err);
		}
		self->mRunning = true;
	}
}


/* Called on the loop thread */
void VipcSource::watchdogTimerCb(struct pomp_timer *timer, void *userdata)
{
	PDRAW_UNUSED(timer);

	auto *self = static_cast<VipcSource *>(userdata);

	if (self->mRunning)
		PDRAW_LOGW("timeout while collecting frames");
	else
		PDRAW_LOGW("timeout while waiting for %s",
			   self->mVipcConnected ? "status" : "connection");

	if (self->mVipcSourceListener != nullptr) {
		self->mVipcSourceListener->vipcSourceReadyToPlay(
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
	PDRAW_UNUSED(ctx);

	auto *self = static_cast<VipcSource *>(userdata);

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

	if (self->mState != State::STARTED) {
		PDRAW_LOGI("%s: ignored in state %s",
			   __func__,
			   Element::getElementStateStr(self->mState));
		return;
	}

	/* TODO */
}


#	if PDRAW_VIPC_BACKEND_DMABUF || PDRAW_VIPC_BACKEND_HISI ||            \
		PDRAW_VIPC_BACKEND_NETWORK_HISI || PDRAW_VIPC_BACKEND_SHM
/* Can be called from any thread */
void VipcSource::releaseFrameCb(void *data, size_t len, void *userdata)
{
	PDRAW_UNUSED(data);
	PDRAW_UNUSED(len);

	auto *f = static_cast<FrameCtx *>(userdata);
	VipcSource *self = f->self;
	vipcc_release_safe(f->frame);
	self->decrementUsedFrameCount();

	if ((self->mUsedFrameCount == 0) && (self->mState == State::STOPPING)) {
		int err = pomp_loop_idle_add_with_cookie(
			self->mSession->getLoop(),
			&idleCompleteStop,
			self,
			self);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	}

	free(f);
}


/* Can be called from any thread */
void VipcSource::releaseFdFrameCb(void *data,
				  size_t len,
				  int fd,
				  void *userdata)
{
	VipcSource::releaseFrameCb(data, len, userdata);
}
#	endif


#	if PDRAW_VIPC_BACKEND_DMABUF
void VipcSource::dmabufFrameCb(struct vipcc_ctx *ctx,
			       const struct vipc_frame *frame,
			       void *be_frame,
			       void *userdata)
{
	auto *self = static_cast<VipcSource *>(userdata);
	auto *dmabufFrame = static_cast<struct dmabuf_frame *>(be_frame);
	FrameCtx *f = nullptr;
	int err;
	struct mbuf_mem *mem = nullptr;
	size_t totalSize = 0;

	self->incrementUsedFrameCount();

	if (self->mState != State::STARTED) {
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
	if ((self->mInputFramesCount - 1) % self->mParams->decimation != 0)
		goto unref;

	/* Wrap the frame */
	f = static_cast<FrameCtx *>(calloc(1, sizeof(*f)));
	if (f == nullptr) {
		PDRAW_LOG_ERRNO("calloc", ENOMEM);
		goto unref;
	}
	f->self = self;
	f->client = ctx;
	f->frame = frame;

	for (unsigned int i = 0; i < getNumPlanes(frame); i++)
		totalSize += frame->planes[i].size;

	switch (self->mParams->mem_implem) {
	case MBUF_MEM_IMPLEM_TYPE_AUTO:
	case MBUF_MEM_IMPLEM_TYPE_ION:
		err = mbuf_mem_ion_wrap(
			reinterpret_cast<void *>(frame->planes[0].virt_addr),
			totalSize,
			dmabufFrame->fd[0],
			releaseFrameCb,
			f,
			&mem);
		if (err < 0) {
			PDRAW_LOG_ERRNO("mbuf_mem_ion_wrap", -err);
			goto unref;
		}
		break;
	case MBUF_MEM_IMPLEM_TYPE_VACQ:
		err = mbuf_mem_vacq_wrap_with_fd(
			reinterpret_cast<void *>(frame->planes[0].virt_addr),
			totalSize,
			dmabufFrame->fd[0],
			releaseFdFrameCb,
			f,
			&mem);
		if (err < 0) {
			PDRAW_LOG_ERRNO("mbuf_mem_vacq_wrap_with_fd", -err);
			goto unref;
		}
		break;
	default:
		PDRAW_LOGE(
			"invalid memory implem (%s)",
			mbuf_mem_implem_type_to_str(self->mParams->mem_implem));
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
		self->decrementUsedFrameCount();
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
	auto *self = static_cast<VipcSource *>(userdata);
	VIDEO_FRAME_INFO_S *frameInfo =
		static_cast<VIDEO_FRAME_INFO_S *>(be_frame);
	FrameCtx *f = nullptr;
	int err;
	struct mbuf_mem *mem = nullptr;
	size_t totalSize = 0;

	self->incrementUsedFrameCount();

	if (self->mState != State::STARTED) {
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
	if ((self->mInputFramesCount - 1) % self->mParams->decimation != 0)
		goto unref;

	/* Wrap the frame */
	f = static_cast<FrameCtx *>(calloc(1, sizeof(*f)));
	if (f == nullptr) {
		PDRAW_LOG_ERRNO("calloc", ENOMEM);
		goto unref;
	}
	f->self = self;
	f->client = ctx;
	f->frame = frame;

	for (unsigned int i = 0; i < getNumPlanes(frame); i++)
		totalSize += frame->planes[i].size;

	switch (self->mParams->mem_implem) {
	case MBUF_MEM_IMPLEM_TYPE_AUTO:
	case MBUF_MEM_IMPLEM_TYPE_HISI:
		err = mbuf_mem_hisi_wrap(
			reinterpret_cast<void *>(frame->planes[0].virt_addr),
			totalSize,
			frameInfo,
			releaseFrameCb,
			f,
			&mem);
		if (err < 0) {
			PDRAW_LOG_ERRNO("mbuf_mem_hisi_wrap", -err);
			goto unref;
		}
		break;
	default:
		PDRAW_LOGE(
			"invalid memory implem (%s)",
			mbuf_mem_implem_type_to_str(self->mParams->mem_implem));
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
		self->decrementUsedFrameCount();
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
	PDRAW_UNUSED(be_frame);

	auto *self = static_cast<VipcSource *>(userdata);
	FrameCtx *f = nullptr;
	int err;
	struct mbuf_mem *mem = nullptr;
	size_t totalSize = 0;

	self->incrementUsedFrameCount();

	if (self->mState != State::STARTED) {
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
	if ((self->mInputFramesCount - 1) % self->mParams->decimation != 0)
		goto unref;

	/* Wrap the frame */
	f = static_cast<FrameCtx *>(calloc(1, sizeof(*f)));
	if (f == nullptr) {
		PDRAW_LOG_ERRNO("calloc", ENOMEM);
		goto unref;
	}
	f->self = self;
	f->client = ctx;
	f->frame = frame;

	for (unsigned int i = 0; i < getNumPlanes(frame); i++)
		totalSize += frame->planes[i].size;

	switch (self->mParams->mem_implem) {
	case MBUF_MEM_IMPLEM_TYPE_AUTO:
	case MBUF_MEM_IMPLEM_TYPE_GENERIC:
		err = mbuf_mem_generic_wrap(
			reinterpret_cast<void *>(frame->planes[0].virt_addr),
			totalSize,
			releaseFrameCb,
			f,
			&mem);
		if (err < 0) {
			PDRAW_LOG_ERRNO("mbuf_mem_generic_wrap", -err);
			goto unref;
		}
		break;
	default:
		PDRAW_LOGE(
			"invalid memory implem (%s)",
			mbuf_mem_implem_type_to_str(self->mParams->mem_implem));
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
		self->decrementUsedFrameCount();
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
	auto *self = static_cast<VipcSource *>(userdata);
	int err;
	struct mbuf_mem *mem = nullptr;
	size_t totalSize = 0, cap;
	void *data;

	self->incrementUsedFrameCount();

	if (self->mState != State::STARTED) {
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
	if ((self->mInputFramesCount - 1) % self->mParams->decimation != 0)
		goto unref;

	/* The frame needs to have contiguous planes */
	for (unsigned int i = 0; i < getNumPlanes(frame); - 1; i++) {
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

	for (unsigned int i = 0; i < getNumPlanes(frame); i++)
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

		switch (self->mParams->mem_implem) {
		case MBUF_MEM_IMPLEM_TYPE_AUTO:
		case MBUF_MEM_IMPLEM_TYPE_GENERIC:
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
			break;
		default:
			PDRAW_LOGE("invalid memory implem (%s)",
				   mbuf_mem_implem_type_to_str(
					   self->mParams->mem_implem));
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
	memcpy(data,
	       reinterpret_cast<void *>(frame->planes[0].virt_addr),
	       totalSize);

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
	self->decrementUsedFrameCount();
}
#	endif


void VipcSource::connectionStatusCb(struct vipcc_ctx *ctx,
				    bool connected,
				    void *userdata)
{
	PDRAW_UNUSED(ctx);

	auto *self = static_cast<VipcSource *>(userdata);
	PDRAW_LOGI("%s server %s",
		   connected ? "connected to" : "disconnected from",
		   self->mParams->friendly_name ? self->mParams->friendly_name
						: self->mParams->address);
	self->mVipcConnected = connected;

	if (!connected && self->mWatchdogTimer != nullptr &&
	    self->mParams->connection_timeout_ms != -1) {
		int err = pomp_timer_set(self->mWatchdogTimer,
					 self->mParams->connection_timeout_ms);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_timer_set", -err);
	}
}


void VipcSource::eosCb(struct vipcc_ctx *ctx,
		       enum vipc_eos_reason reason,
		       void *userdata)
{
	PDRAW_UNUSED(ctx);

	int err;
	auto *self = static_cast<VipcSource *>(userdata);

	PDRAW_LOGN("%s('%s'): EOS reason %s (%u)",
		   __func__,
		   self->getSourceName(),
		   vipc_eos_reason_to_str(reason),
		   reason);

	if (self->mState != State::STARTED) {
		PDRAW_LOGI("%s: ignored in state %s",
			   __func__,
			   Element::getElementStateStr(self->mState));
		return;
	}

	if (self->mWatchdogTimer != nullptr) {
		err = pomp_timer_clear(self->mWatchdogTimer);
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
			err = self->sendDownstreamEvent(
				self->mOutputMedia.get(),
				Channel::DownstreamEvent::EOS);
			if (err < 0)
				PDRAW_LOG_ERRNO("sendDownstreamEvent", -err);
		}
		break;
	case VIPC_EOS_REASON_RESTART:
		self->mLastEosReason = PDRAW_VIPC_SOURCE_EOS_REASON_RESTART;
		if (self->mOutputMedia != nullptr) {
			err = self->sendDownstreamEvent(
				self->mOutputMedia.get(),
				Channel::DownstreamEvent::RECONFIGURE);
			if (err < 0)
				PDRAW_LOG_ERRNO("sendDownstreamEvent", -err);
		}
		break;
	case VIPC_EOS_REASON_CONFIGURATION:
		self->mLastEosReason =
			PDRAW_VIPC_SOURCE_EOS_REASON_CONFIGURATION;
		self->mFirstFrame = true;
		if (self->mOutputMedia != nullptr) {
			err = self->sendDownstreamEvent(
				self->mOutputMedia.get(),
				Channel::DownstreamEvent::EOS);
			if (err < 0)
				PDRAW_LOG_ERRNO("sendDownstreamEvent", -err);
		}
		break;
	}

	if (self->mVipcSourceListener != nullptr) {
		self->mKeepMedia =
			self->mVipcSourceListener->vipcSourceEndOfStream(
				self->mSession,
				self->getVipcSource(),
				self->mLastEosReason);
	}

	/* Notify not ready-to-play if eos is ignored */
	if (self->mKeepMedia && self->mWasReady) {
		self->mWasReady = false;
		if ((self->mVipcSourceListener != nullptr) &&
		    (self->mState == State::STARTED)) {
			self->mVipcSourceListener->vipcSourceReadyToPlay(
				self->mSession,
				self->getVipcSource(),
				self->mReady,
				self->mLastEosReason);
		}
	}

	/* Invalidate the status to avoid creating a new media */
	vipc_status_free(self->mStatus);
	self->mStatus = nullptr;

	(void)self->setupMedia();
}


const char *VipcSource::getSourceName() const
{
	if (mParams->friendly_name != nullptr &&
	    !std::string(mParams->friendly_name).empty())
		return mParams->friendly_name;
	if (mParams->address != nullptr &&
	    !std::string(mParams->address).empty())
		return mParams->address;
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
	[toIndex(VipcSource::BackendType::DMABUF)] =
		Backend("dmabuf",
			&vipc_be_dmabuf_callbacks,
			&VipcSource::cDmabufCbs),
#	endif
#	if PDRAW_VIPC_BACKEND_HISI
	[toIndex(VipcSource::BackendType::HISI)] =
		Backend("hisi", &vipc_be_hisi_callbacks, &VipcSource::cHisiCbs),
#	endif
#	if PDRAW_VIPC_BACKEND_SHM
	[toIndex(VipcSource::BackendType::SHM)] =
		Backend("shm", &vipc_be_shm_callbacks, &VipcSource::cShmCbs),
#	endif
#	if PDRAW_VIPC_BACKEND_NETWORK_HISI
	[toIndex(VipcSource::BackendType::NETWORK_HISI)] =
		Backend("network-hisi",
			&vipc_be_network_hisi_callbacks,
			&VipcSource::cHisiCbs),
#	endif
#	if PDRAW_VIPC_BACKEND_NETWORK_CBUF
	[toIndex(VipcSource::BackendType::NETWORK_CBUF)] =
		Backend("network-cbuf",
			&vipc_be_network_cbuf_callbacks,
			&VipcSource::cCbufCbs),
#	endif
};


#endif /* BUILD_LIBVIDEO_IPC */


VipcSourceWrapper::VipcSourceWrapper(
	Session *session,
	const struct pdraw_vipc_source_params *params,
	IPdraw::IVipcSource::Listener *listener)
#ifdef BUILD_LIBVIDEO_IPC
		:
		ElementWrapper(new Pdraw::VipcSource(session,
						     session,
						     session,
						     listener,
						     this,
						     params)),
		mSource(static_cast<Pdraw::VipcSource *>(mElement))
#else
		:
		ElementWrapper(nullptr)
#endif
{
#ifndef BUILD_LIBVIDEO_IPC
	PDRAW_UNUSED(session);
	PDRAW_UNUSED(params);
	PDRAW_UNUSED(listener);

	ULOGE("no video IPC source implementation found");
#endif
}


VipcSourceWrapper::~VipcSourceWrapper()
{
#ifdef BUILD_LIBVIDEO_IPC
	if (isElementStopped())
		return;
	int ret = mSource->stop();
	if (ret < 0)
		ULOG_ERRNO("source->stop", -ret);
#endif
}


bool VipcSourceWrapper::isReadyToPlay()
{
#ifdef BUILD_LIBVIDEO_IPC
	if (isElementStopped())
		return false;
	return mSource->isReadyToPlay();
#else
	return false;
#endif
}


bool VipcSourceWrapper::isPaused()
{
#ifdef BUILD_LIBVIDEO_IPC
	if (isElementStopped())
		return false;
	return mSource->isPaused();
#else
	return false;
#endif
}


int VipcSourceWrapper::play()
{
#ifdef BUILD_LIBVIDEO_IPC
	if (isElementStopped())
		return -EPROTO;
	return mSource->play();
#else
	return -ENOSYS;
#endif
}


int VipcSourceWrapper::pause()
{
#ifdef BUILD_LIBVIDEO_IPC
	if (isElementStopped())
		return -EPROTO;
	return mSource->pause();
#else
	return -ENOSYS;
#endif
}


int VipcSourceWrapper::configure(const struct vdef_dim *resolution,
				 const struct vdef_rectf *crop)
{
#ifdef BUILD_LIBVIDEO_IPC
	if (isElementStopped())
		return -EPROTO;
	return mSource->configure(resolution, crop);
#else
	return -ENOSYS;
#endif
}


int VipcSourceWrapper::insertGreyFrame(uint64_t tsUs)
{
#ifdef BUILD_LIBVIDEO_IPC
	if (isElementStopped())
		return -EPROTO;
	return mSource->insertGreyFrame(tsUs);
#else
	return -ENOSYS;
#endif
}


int VipcSourceWrapper::setSessionMetadata(const struct vmeta_session *meta)
{
#ifdef BUILD_LIBVIDEO_IPC
	if (isElementStopped())
		return -EPROTO;
	return mSource->setSessionMetadata(meta);
#else
	return -ENOSYS;
#endif
}


int VipcSourceWrapper::getSessionMetadata(struct vmeta_session *meta)
{
#ifdef BUILD_LIBVIDEO_IPC
	if (isElementStopped())
		return -EPROTO;
	return mSource->getSessionMetadata(meta);
#else
	return -ENOSYS;
#endif
}

} /* namespace Pdraw */
