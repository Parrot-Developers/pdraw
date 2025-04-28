/**
 * Parrot Drones Audio and Video Vector library
 * Application external coded video source
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

#define ULOG_TAG pdraw_external_coded_video_source
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_external_coded_video_source.hpp"
#include "pdraw_session.hpp"

#include <time.h>

#define PDRAW_EXT_CODED_VIDEO_SOURCE_ANCILLARY_KEY_INPUT_TIME                  \
	"pdraw.codedvideosource.input_time"

namespace Pdraw {


ExternalCodedVideoSource::ExternalCodedVideoSource(
	Session *session,
	Element::Listener *elementListener,
	Source::Listener *sourceListener,
	IPdraw::ICodedVideoSource::Listener *listener,
	CodedVideoSourceWrapper *wrapper,
	const struct pdraw_video_source_params *params) :
		SourceElement(session,
			      elementListener,
			      wrapper,
			      1,
			      sourceListener),
		mVideoSource(wrapper), mVideoSourceListener(listener),
		mParams(*params), mFrameQueue(nullptr), mOutputMedia(nullptr),
		mLastTimestamp(UINT64_MAX), mFlushPending(false)
{
	Element::setClassName(__func__);

	setState(CREATED);
}


ExternalCodedVideoSource::~ExternalCodedVideoSource(void)
{
	int err;

	if (mState == STARTED)
		PDRAW_LOGW("video source is still running");

	/* Make sure listener functions will no longer be called */
	mVideoSourceListener = nullptr;

	/* Remove any leftover idle callbacks */
	err = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	if (mFrameQueue != nullptr) {
		err = mbuf_coded_video_frame_queue_flush(mFrameQueue);
		if (err < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_flush",
					-err);
		}
		err = mbuf_coded_video_frame_queue_destroy(mFrameQueue);
		if (err < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_destroy",
					-err);
		}
	}

	if (mOutputMedia != nullptr)
		PDRAW_LOGW("output media was not properly removed");
}


int ExternalCodedVideoSource::start(void)
{
	int ret, err;
	struct pomp_evt *evt = nullptr;
	std::string path;

	if ((mState == STARTED) || (mState == STARTING))
		return 0;
	if (mState != CREATED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}
	setState(STARTING);

	mbuf_coded_video_frame_queue_args args = {
		.filter = &ExternalCodedVideoSource::inputFilter,
		.filter_userdata = this,
		.max_frames = mParams.queue_max_count,
	};
	ret = mbuf_coded_video_frame_queue_new_with_args(&args, &mFrameQueue);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_new_with_args",
				-ret);
		goto error;
	}

	ret = mbuf_coded_video_frame_queue_get_event(mFrameQueue, &evt);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_get_event", -ret);
		goto error;
	}

	ret = pomp_evt_attach_to_loop(
		evt, mSession->getLoop(), &queueEventCb, this);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto error;
	}

	setState(STARTED);

	Source::lock();

	mOutputMedia = new CodedVideoMedia(mSession);
	if (mOutputMedia == nullptr) {
		Source::unlock();
		PDRAW_LOGE("output media allocation failed");
		ret = -ENOMEM;
		goto error;
	}
	path = Element::getName() + "$" + mOutputMedia->getName();
	mOutputMedia->setPath(path);

	ret = addOutputPort(mOutputMedia);
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("addOutputPort", -ret);
		goto error;
	}

	mOutputMedia->format = mParams.video.coded.format;
	mOutputMedia->info = mParams.video.coded.info;
	mOutputMedia->sessionMeta = mParams.session_meta;
	mOutputMedia->playbackType = mParams.playback_type;
	mOutputMedia->duration = mParams.duration;
	switch (mOutputMedia->format.encoding) {
	case VDEF_ENCODING_H264:
		ret = mOutputMedia->setPs(nullptr,
					  0,
					  mParams.video.coded.h264.sps,
					  mParams.video.coded.h264.spslen,
					  mParams.video.coded.h264.pps,
					  mParams.video.coded.h264.ppslen);
		if (ret < 0) {
			Source::unlock();
			PDRAW_LOG_ERRNO("media->setPs", -ret);
			goto error;
		}
		break;
	case VDEF_ENCODING_H265:
		ret = mOutputMedia->setPs(mParams.video.coded.h265.vps,
					  mParams.video.coded.h265.vpslen,
					  mParams.video.coded.h265.sps,
					  mParams.video.coded.h265.spslen,
					  mParams.video.coded.h265.pps,
					  mParams.video.coded.h265.ppslen);
		if (ret < 0) {
			Source::unlock();
			PDRAW_LOG_ERRNO("media->setPs", -ret);
			goto error;
		}
		break;
	default:
		break;
	}

	Source::unlock();

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

	return 0;

error:
	(void)stop();
	return ret;
}


int ExternalCodedVideoSource::stop(void)
{
	int ret;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if (mState != STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(mState));
		return -EPROTO;
	}

	setState(STOPPING);

	/* Make sure listener functions will no longer be called */
	mVideoSourceListener = nullptr;

	Source::lock();
	if (mOutputMedia != nullptr)
		mOutputMedia->setTearingDown();
	Source::unlock();

	/* Flush everything */
	ret = flush();
	if (ret < 0)
		PDRAW_LOG_ERRNO("flush", -ret);

	/* When the flush is complete, stopping will be triggered */
	return ret;
}


int ExternalCodedVideoSource::tryStop(void)
{
	int ret;
	struct pomp_evt *evt = nullptr;
	int completeStopPendingCount;

	if (mState != STOPPING)
		return 0;

	if (mFrameQueue == nullptr)
		goto teardown;

	ret = mbuf_coded_video_frame_queue_get_event(mFrameQueue, &evt);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_get_event", -ret);
		goto queue_destroy;
	}

	ret = pomp_evt_detach_from_loop(evt, mSession->getLoop());
	if (ret < 0) {
		PDRAW_LOG_ERRNO("pomp_evt_detach_from_loop", -ret);
		goto queue_destroy;
	}

queue_destroy:
	ret = mbuf_coded_video_frame_queue_destroy(mFrameQueue);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_destroy", -ret);
	}
	mFrameQueue = nullptr;

teardown:
	/* Teardown the output channels
	 * Note: loop downwards because calling teardown on a channel may or
	 * may not synchronously remove the channel from the output port */
	completeStopPendingCount = 0;
	Source::lock();
	if (mOutputMedia != nullptr) {
		int outputChannelCount = getOutputChannelCount(mOutputMedia);

		for (int i = outputChannelCount - 1; i >= 0; i--) {
			Channel *channel = getOutputChannel(mOutputMedia, i);
			if (channel == nullptr) {
				PDRAW_LOGW("failed to get channel at index %d",
					   i);
				continue;
			}
			ret = channel->teardown();
			if (ret < 0)
				PDRAW_LOG_ERRNO("channel->teardown", -ret);
			else
				completeStopPendingCount++;
		}
	}
	Source::unlock();

	if (completeStopPendingCount == 0)
		completeStop();

	return 0;
}


void ExternalCodedVideoSource::completeStop(void)
{
	int ret;
	unsigned int outputChannelCount;

	Source::lock();

	if (mOutputMedia == nullptr)
		goto exit;

	outputChannelCount = getOutputChannelCount(mOutputMedia);
	if (outputChannelCount > 0) {
		Source::unlock();
		return;
	}

	/* Remove the output port */
	if (Source::mListener) {
		Source::mListener->onOutputMediaRemoved(
			this, mOutputMedia, getVideoSource());
	}
	ret = removeOutputPort(mOutputMedia);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("removeOutputPort", -ret);
	} else {
		delete mOutputMedia;
		mOutputMedia = nullptr;
	}

exit:
	Source::unlock();

	setState(STOPPED);
}


void ExternalCodedVideoSource::onChannelUnlink(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Source::onChannelUnlink(channel);

	if (mState == STOPPING)
		completeStop();
}


int ExternalCodedVideoSource::flush(void)
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

	/* Flush the queue */
	if (mFrameQueue != nullptr) {
		err = mbuf_coded_video_frame_queue_flush(mFrameQueue);
		if (err < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_flush",
					-err);
		}
	}

	if (!mFlushPending)
		completeFlush();

	return 0;
}


int ExternalCodedVideoSource::setSessionMetadata(
	const struct vmeta_session *meta)
{
	if (meta == nullptr)
		return -EINVAL;

	mParams.session_meta = *meta;

	Source::lock();
	if (mOutputMedia != nullptr) {
		mOutputMedia->sessionMeta = mParams.session_meta;
		int err = sendDownstreamEvent(
			mOutputMedia,
			Channel::DownstreamEvent::SESSION_META_UPDATE);
		if (err < 0)
			PDRAW_LOG_ERRNO("sendDownstreamEvent", -err);
	}
	Source::unlock();

	return 0;
}


int ExternalCodedVideoSource::getSessionMetadata(
	struct vmeta_session *meta) const
{
	if (meta == nullptr)
		return -EINVAL;

	*meta = mParams.session_meta;

	return 0;
}


void ExternalCodedVideoSource::completeFlush(void)
{
	int err;
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

	if (mState != STOPPING) {
		/* Signal to the application that flushing is done */
		err = pomp_loop_idle_add_with_cookie(mSession->getLoop(),
						     callVideoSourceFlushed,
						     this,
						     this);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
	}

	tryStop();
}


void ExternalCodedVideoSource::onChannelFlushed(Channel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel);
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


void ExternalCodedVideoSource::queueEventCb(struct pomp_evt *evt,
					    void *userdata)
{
	ExternalCodedVideoSource *self =
		reinterpret_cast<ExternalCodedVideoSource *>(userdata);
	int ret, err;
	struct mbuf_coded_video_frame *frame = nullptr;

	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mState != STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(self->mState));
		return;
	}
	if (self->mFrameQueue == nullptr) {
		PDRAW_LOGE("%s: invalid queue", __func__);
		return;
	}
	if (self->mFlushPending) {
		PDRAW_LOGI("%s: flush pending, discarding queue event",
			   __func__);
		return;
	}

	do {
		ret = mbuf_coded_video_frame_queue_pop(self->mFrameQueue,
						       &frame);
		if (ret < 0) {
			if (ret != -EAGAIN) {
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_queue_pop",
					-ret);
			}
			continue;
		}

		(void)self->processFrame(frame);

		err = mbuf_coded_video_frame_unref(frame);
		if (err < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_unref", -err);
	} while (ret == 0);
}


bool ExternalCodedVideoSource::inputFilter(struct mbuf_coded_video_frame *frame,
					   void *userdata)
{
	ExternalCodedVideoSource *self =
		reinterpret_cast<ExternalCodedVideoSource *>(userdata);
	int ret, err;
	bool accept = true;
	uint64_t ts_us;
	struct timespec cur_ts = {0, 0};
	struct vdef_coded_frame frame_info;

	PDRAW_LOG_ERRNO_RETURN_VAL_IF(self == nullptr, EINVAL, false);
	PDRAW_LOG_ERRNO_RETURN_VAL_IF(frame == nullptr, EINVAL, false);

	if (self->mState != STARTED) {
		PDRAW_LOGE("%s: invalid state (%s)",
			   __func__,
			   Element::getElementStateStr(self->mState));
		return false;
	}

	self->Source::lock();

	if (self->mOutputMedia == nullptr) {
		PDRAW_LOGE("%s: invalid output media", __func__);
		accept = false;
		goto out;
	}

	ret = mbuf_coded_video_frame_get_frame_info(frame, &frame_info);
	if (ret < 0) {
		accept = false;
		goto out;
	}

	/* Check the frame format */
	if (!vdef_coded_format_intersect(
		    &frame_info.format, &self->mOutputMedia->format, 1)) {
		PDRAW_LOGE(
			"%s: unsupported format:"
			" " VDEF_CODED_FORMAT_TO_STR_FMT,
			__func__,
			VDEF_CODED_FORMAT_TO_STR_ARG(&frame_info.format));
		accept = false;
		goto out;
	}

	/* Check the frame timestamp */
	if (frame_info.info.timestamp <= self->mLastTimestamp &&
	    self->mLastTimestamp != UINT64_MAX) {
		PDRAW_LOGE("%s: non-strictly-monotonic timestamp (%" PRIu64
			   " <= %" PRIu64 ")",
			   __func__,
			   frame_info.info.timestamp,
			   self->mLastTimestamp);
		accept = false;
		goto out;
	}

	/* Check the frame info */
	if ((self->mOutputMedia->info.bit_depth != frame_info.info.bit_depth) ||
	    (self->mOutputMedia->info.full_range !=
	     frame_info.info.full_range) ||
	    !vdef_dim_cmp(&self->mOutputMedia->info.resolution,
			  &frame_info.info.resolution)) {
		PDRAW_LOGE(
			"%s: invalid frame information "
			"expected (resolution:%ux%u, bit_depth:%d, range:%s) "
			"got (resolution:%ux%u, bit_depth:%d, range:%s)",
			__func__,
			self->mOutputMedia->info.resolution.width,
			self->mOutputMedia->info.resolution.height,
			self->mOutputMedia->info.bit_depth,
			self->mOutputMedia->info.full_range ? "full"
							    : "limited",
			frame_info.info.resolution.width,
			frame_info.info.resolution.height,
			frame_info.info.bit_depth,
			frame_info.info.full_range ? "full" : "limited");
		accept = false;
		goto out;
	}

	/* Save frame timestamp as last_timestamp */
	self->mLastTimestamp = frame_info.info.timestamp;

	/* Set the input time ancillary data */
	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);
	err = mbuf_coded_video_frame_add_ancillary_buffer(
		frame,
		PDRAW_EXT_CODED_VIDEO_SOURCE_ANCILLARY_KEY_INPUT_TIME,
		&ts_us,
		sizeof(ts_us));
	if (err < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
				-err);
	}

out:
	self->Source::unlock();

	return accept;
}


int ExternalCodedVideoSource::processFrame(struct mbuf_coded_video_frame *frame)
{
	int ret, err;
	struct vdef_coded_frame info;
	CodedVideoMedia::Frame out_meta = {};
	struct timespec ts = {0, 0};
	uint64_t curTime = 0;
	unsigned int outputChannelCount;

	Source::lock();

	if (mOutputMedia == nullptr) {
		PDRAW_LOGE("%s: invalid output media", __func__);
		ret = -EPROTO;
		goto out;
	}

	ret = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -ret);
		goto out;
	}

	time_get_monotonic(&ts);
	time_timespec_to_us(&ts, &curTime);
	out_meta.ntpTimestamp =
		(info.info.timestamp * 1000000 + info.info.timescale / 2) /
		info.info.timescale;
	out_meta.ntpUnskewedTimestamp = out_meta.ntpTimestamp;
	out_meta.ntpRawTimestamp = out_meta.ntpTimestamp;
	out_meta.ntpRawUnskewedTimestamp = out_meta.ntpTimestamp;
	out_meta.demuxOutputTimestamp = curTime;
	out_meta.playTimestamp = info.info.capture_timestamp;
	out_meta.captureTimestamp = info.info.capture_timestamp;
	out_meta.localTimestamp = info.info.capture_timestamp;

	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		frame,
		PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0 && ret != -EEXIST) {
		/* Ancillary buffer may already exist; ignore -EEXIST */
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
				-ret);
		goto out;
	}

	/* Queue the frame in the output channels */
	outputChannelCount = getOutputChannelCount(mOutputMedia);
	for (unsigned int i = 0; i < outputChannelCount; i++) {
		Channel *c = getOutputChannel(mOutputMedia, i);
		CodedVideoChannel *channel =
			dynamic_cast<CodedVideoChannel *>(c);
		if (channel == nullptr) {
			PDRAW_LOGE("failed to get channel at index %d", i);
			continue;
		}
		err = channel->queue(frame);
		if (err < 0)
			PDRAW_LOG_ERRNO("channel->queue", -err);
	}

out:
	Source::unlock();

	return ret;
}


/* Listener call from an idle function */
void ExternalCodedVideoSource::callOnMediaAdded(void *userdata)
{
	ExternalCodedVideoSource *self =
		reinterpret_cast<ExternalCodedVideoSource *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mOutputMedia == nullptr) {
		PDRAW_LOGW("%s: output media not found", __func__);
		return;
	}

	if (self->Source::mListener) {
		self->Source::mListener->onOutputMediaAdded(
			self, self->mOutputMedia, self->getVideoSource());
	}
}


/* Listener call from an idle function */
void ExternalCodedVideoSource::callVideoSourceFlushed(void *userdata)
{
	ExternalCodedVideoSource *self =
		reinterpret_cast<ExternalCodedVideoSource *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mVideoSourceListener != nullptr) {
		self->mVideoSourceListener->onCodedVideoSourceFlushed(
			self->mSession, self->getVideoSource());
	}
}


CodedVideoSourceWrapper::CodedVideoSourceWrapper(
	Session *session,
	const struct pdraw_video_source_params *params,
	IPdraw::ICodedVideoSource::Listener *listener)
{
	mElement = mSource = new Pdraw::ExternalCodedVideoSource(
		session, session, session, listener, this, params);
}


CodedVideoSourceWrapper::~CodedVideoSourceWrapper(void)
{
	if (mSource == nullptr)
		return;
	int ret = mSource->stop();
	if (ret < 0)
		ULOG_ERRNO("source->stop", -ret);
}


struct mbuf_coded_video_frame_queue *CodedVideoSourceWrapper::getQueue(void)
{
	if (mSource == nullptr)
		return nullptr;
	return mSource->getQueue();
}


int CodedVideoSourceWrapper::flush(void)
{
	if (mSource == nullptr)
		return -EPROTO;
	return mSource->flush();
}


int CodedVideoSourceWrapper::setSessionMetadata(
	const struct vmeta_session *meta)
{
	if (mSource == nullptr)
		return -EPROTO;
	return mSource->setSessionMetadata(meta);
}


int CodedVideoSourceWrapper::getSessionMetadata(struct vmeta_session *meta)
{
	if (mSource == nullptr)
		return -EPROTO;
	return mSource->getSessionMetadata(meta);
}

} /* namespace Pdraw */
