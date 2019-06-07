/**
 * Parrot Drones Awesome Video Viewer Library
 * Video scaler element
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

#define ULOG_TAG pdraw_vscale
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_scaler_video.hpp"
#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"

#include <time.h>
#include <unistd.h>

#include <vector>

namespace Pdraw {


const struct vscale_cbs VideoScaler::mScalerCbs = {
	.frame_output = &VideoScaler::frameOutputCb,
	.flush = &VideoScaler::flushCb,
	.stop = &VideoScaler::stopCb,
};


VideoScaler::VideoScaler(Session *session,
			 Element::Listener *elementListener,
			 RawSource::Listener *sourceListener) :
		RawToRawFilterElement(session,
				      elementListener,
				      1,
				      nullptr,
				      0,
				      1,
				      sourceListener),
		mInputMedia(nullptr), mOutputMedia(nullptr),
		mInputBufferPool(nullptr), mInputBufferQueue(nullptr),
		mVscale(nullptr), mIsFlushed(true),
		mInputChannelFlushPending(false), mVscaleFlushPending(false),
		mVscaleStopPending(false), mCompleteStopPendingCount(0)
{
	const struct vdef_raw_format *supportedInputFormats;
	int supportedInputFormatsCount;

	Element::setClassName(__func__);

	/* Supported input formats */
	supportedInputFormatsCount = vscale_get_supported_input_formats(
		VSCALE_SCALER_IMPLEM_AUTO, &supportedInputFormats);
	if (supportedInputFormatsCount < 0)
		PDRAW_LOG_ERRNO("vscale_get_supported_input_formats",
				-supportedInputFormatsCount);
	else
		setRawVideoMediaFormatCaps(supportedInputFormats,
					   supportedInputFormatsCount);

	setState(CREATED);
}


VideoScaler::~VideoScaler(void)
{
	int ret;

	if (mState != STOPPED)
		PDRAW_LOGW("scaler is still running");

	if (mVscale) {
		ret = vscale_destroy(mVscale);
		if (ret < 0)
			PDRAW_LOG_ERRNO("vscale_destroy", -ret);
	}

	if (mOutputMedia != nullptr)
		PDRAW_LOGW("output media was not properly removed");
}


int VideoScaler::start(void)
{
	int ret = 0;

	if ((mState == STARTED) || (mState == STARTING)) {
		return 0;
	}
	if (mState != CREATED) {
		PDRAW_LOGE("scaler is not created");
		return -EPROTO;
	}
	setState(STARTING);

	/* Get the input media and port */
	RawSink::lock();
	unsigned int inputMediaCount = getInputMediaCount();
	if (inputMediaCount != 1) {
		RawSink::unlock();
		PDRAW_LOGE("invalid input media count");
		return -EPROTO;
	}
	Media *media = getInputMedia(0);
	if (media == nullptr) {
		RawSink::unlock();
		PDRAW_LOGE("invalid input media");
		return -EPROTO;
	}
	mInputMedia = dynamic_cast<RawVideoMedia *>(media);
	if (mInputMedia == nullptr) {
		RawSink::unlock();
		PDRAW_LOGE("invalid input media");
		return -EPROTO;
	}
	InputPort *port = getInputPort(mInputMedia);
	if (port == nullptr) {
		RawSink::unlock();
		PDRAW_LOGE("invalid input port");
		return -EPROTO;
	}

	/* Initialize the scaler */
	struct vscale_config cfg = {};
	cfg.implem = VSCALE_SCALER_IMPLEM_AUTO;
	cfg.input.format = mInputMedia->format;
	cfg.input.info = mInputMedia->info;
	cfg.output.info = cfg.input.info;
	cfg.output.info.resolution.width = 1280; /* TODO */
	cfg.output.info.resolution.height = 720; /* TODO */
	ret = vscale_new(
		mSession->getLoop(), &cfg, &mScalerCbs, this, &mVscale);
	if (ret < 0) {
		RawSink::unlock();
		PDRAW_LOG_ERRNO("vscale_new", -ret);
		return ret;
	}

	/* Setup the input port */
	port->channel->setKey(this);
	mInputBufferQueue = vscale_get_input_buffer_queue(mVscale);
	port->channel->setQueue(mInputBufferQueue);
	mInputBufferPool = vscale_get_input_buffer_pool(mVscale);
	port->channel->setPool(mInputBufferPool);

	RawSink::unlock();

	setState(STARTED);

	return 0;
}


int VideoScaler::stop(void)
{
	int ret;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if (mState != STARTED) {
		PDRAW_LOGE("scaler is not started");
		return -EPROTO;
	}
	setState(STOPPING);
	mVscaleStopPending = true;

	/* Flush everything */
	ret = flush();
	if (ret < 0)
		PDRAW_LOG_ERRNO("flush", -ret);

	/* When the flush is complete, stopping will be triggered */
	return ret;
}


int VideoScaler::flush(void)
{
	int ret;
	unsigned int outputChannelCount, i;
	RawChannel *outputChannel;

	if (mIsFlushed) {
		PDRAW_LOGD("scaler is already flushed, nothing to do");
		completeFlush();
		return 0;
	}

	mVscaleFlushPending = true;

	/* Flush the output channels (async) */
	RawSource::lock();
	if (mOutputMedia != nullptr) {
		outputChannelCount = getOutputChannelCount(mOutputMedia);
		for (i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia, i);
			if (outputChannel == nullptr) {
				PDRAW_LOGW(
					"failed to get output channel "
					"at index %d",
					i);
				continue;
			}
			ret = outputChannel->flush();
			if (ret < 0)
				PDRAW_LOG_ERRNO("channel->flush", -ret);
		}
	}
	RawSource::unlock();

	/* Flush the scaler (async)
	 * (the input channel queue is flushed by vscale) */
	ret = vscale_flush(mVscale, 1);
	if (ret < 0)
		PDRAW_LOG_ERRNO("vscale_flush", -ret);

	return ret;
}


void VideoScaler::completeFlush(void)
{
	int ret;
	unsigned int outputChannelCount, i;
	RawChannel *outputChannel;
	bool pending = false;

	if (mVscaleFlushPending)
		return;

	RawSource::lock();
	if (mOutputMedia != nullptr) {
		outputChannelCount = getOutputChannelCount(mOutputMedia);
		for (i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia, i);
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
	RawSource::unlock();

	if (pending)
		return;

	RawSink::lock();
	if (mInputMedia != nullptr) {
		mIsFlushed = true;
		if (mInputChannelFlushPending) {
			mInputChannelFlushPending = false;
			RawChannel *inputChannel = getInputChannel(mInputMedia);
			if (inputChannel == nullptr) {
				PDRAW_LOGE("failed to get input channel");
			} else {
				ret = inputChannel->flushDone();
				if (ret < 0)
					PDRAW_LOG_ERRNO("channel->flushDone",
							-ret);
			}
		}
	}
	RawSink::unlock();

	tryStop();
}


int VideoScaler::tryStop(void)
{
	int ret;
	int outputChannelCount = 0, i;
	RawChannel *channel;

	if (mState != STOPPING)
		return 0;

	/* Remove the input port */
	RawSink::lock();
	if (mInputMedia != nullptr) {
		channel = getInputChannel(mInputMedia);
		if (channel == nullptr) {
			PDRAW_LOGE("failed to get channel");
		} else {
			channel->setQueue(nullptr);
			channel->setPool(nullptr);
		}

		ret = removeInputMedia(mInputMedia);
		if (ret < 0)
			PDRAW_LOG_ERRNO("removeInputMedia", -ret);
		else
			mInputMedia = nullptr;
	}
	RawSink::unlock();

	/* Teardown the output channels
	 * Note: loop downwards because calling teardown on a channel may or
	 * may not synchronously remove the channel from the output port */
	RawSource::lock();
	if (mOutputMedia != nullptr) {
		outputChannelCount = getOutputChannelCount(mOutputMedia);

		for (i = outputChannelCount - 1; i >= 0; i--) {
			channel = getOutputChannel(mOutputMedia, i);
			if (channel == nullptr) {
				PDRAW_LOGW("failed to get channel at index %d",
					   i);
				continue;
			}
			ret = channel->teardown();
			if (ret < 0)
				PDRAW_LOG_ERRNO("channel->teardown", -ret);
		}
	}
	RawSource::unlock();

	/* Stop the scaler */
	ret = vscale_stop(mVscale);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("vscale_stop", -ret);
		return ret;
	}

	return 0;
}


void VideoScaler::completeStop(void)
{
	int ret;
	unsigned int outputChannelCount;

	mCompleteStopPendingCount--;

	RawSource::lock();
	if (mOutputMedia == nullptr) {
		RawSource::unlock();
		goto exit;
	}
	outputChannelCount = getOutputChannelCount(mOutputMedia);
	if (outputChannelCount > 0) {
		RawSource::unlock();
		return;
	}

	/* Remove the output port */
	if (RawSource::mListener) {
		RawSource::mListener->onOutputMediaRemoved(this, mOutputMedia);
	}
	ret = removeOutputPort(mOutputMedia);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("removeOutputPort", -ret);
	} else {
		delete mOutputMedia;
		mOutputMedia = nullptr;
	}

	RawSource::unlock();

exit:
	if ((!mVscaleStopPending) && (mCompleteStopPendingCount == 0))
		setState(STOPPED);
}


int VideoScaler::createOutputMedia(struct vdef_raw_frame *frameInfo,
				   RawVideoMedia::Frame &frame)
{
	int ret;

	RawSource::lock();

	mOutputMedia = new RawVideoMedia(mSession);
	if (mOutputMedia == nullptr) {
		RawSource::unlock();
		PDRAW_LOGE("output media allocation failed");
		return -ENOMEM;
	}
	std::string path = mInputMedia->getPath() + ">" + Element::getName() +
			   "$" + mOutputMedia->getName();
	mOutputMedia->setPath(path);

	ret = addOutputPort(mOutputMedia);
	if (ret < 0) {
		RawSource::unlock();
		PDRAW_LOG_ERRNO("addOutputPort", -ret);
		return ret;
	}

	mOutputMedia->format = frameInfo->format;
	vdef_frame_to_format_info(&frameInfo->info, &mOutputMedia->info);
	mOutputMedia->info.framerate = mInputMedia->info.framerate;
	mOutputMedia->sessionMeta = mInputMedia->sessionMeta;
	mOutputMedia->playbackType = mInputMedia->playbackType;
	mOutputMedia->duration = mInputMedia->duration;

	RawSource::unlock();

	if (RawSource::mListener)
		RawSource::mListener->onOutputMediaAdded(this, mOutputMedia);

	return 0;
}


void VideoScaler::onChannelQueue(RawChannel *channel,
				 struct mbuf_raw_video_frame *frame)
{

	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}
	if (frame == nullptr) {
		PDRAW_LOG_ERRNO("frame", EINVAL);
		return;
	}
	if (mState != STARTED) {
		PDRAW_LOGE("scaler is not started");
		return;
	}
	if ((mVscaleFlushPending) || (mInputChannelFlushPending)) {
		PDRAW_LOGI("frame input: flush pending, discard frame");
		return;
	}
	RawSink::lock();
	struct mbuf_raw_video_frame_queue *queue = channel->getQueue();
	if (queue == nullptr) {
		RawSink::unlock();
		PDRAW_LOGE("invalid queue");
		return;
	}
	if (queue != mInputBufferQueue) {
		RawSink::unlock();
		PDRAW_LOGE("invalid input buffer queue");
		return;
	}

	RawSink::onChannelQueue(channel, frame);
	mIsFlushed = false;
	RawSink::unlock();
}


void VideoScaler::onChannelFlush(RawChannel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	PDRAW_LOGD("flushing input channel");
	mInputChannelFlushPending = true;

	int ret = flush();
	if (ret < 0)
		PDRAW_LOG_ERRNO("flush", -ret);
}


void VideoScaler::onChannelFlushed(RawChannel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Media *media = getOutputMediaFromChannel(channel->getKey());
	if (media == nullptr) {
		PDRAW_LOGE("media not found");
		return;
	}
	PDRAW_LOGD("'%s': channel flushed media name=%s (channel key=%p)",
		   Element::getName().c_str(),
		   media->getName().c_str(),
		   channel->getKey());

	completeFlush();
}


void VideoScaler::onChannelTeardown(RawChannel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	PDRAW_LOGD("tearing down input channel");

	int ret = stop();
	if (ret < 0)
		PDRAW_LOG_ERRNO("stop", -ret);
}


void VideoScaler::onChannelUnlink(RawChannel *channel)
{
	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	RawSource::onChannelUnlink(channel);

	if (mState == STOPPING) {
		mCompleteStopPendingCount++;
		completeStop();
	}
}


void VideoScaler::frameOutputCb(struct vscale_scaler *scaler,
				int status,
				struct mbuf_raw_video_frame *out_frame,
				void *userdata)
{
	int ret;
	VideoScaler *self = (VideoScaler *)userdata;
	struct vdef_raw_frame info;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	RawVideoMedia::Frame *in_meta;
	RawVideoMedia::Frame out_meta;
	unsigned int outputChannelCount, i;
	RawChannel *channel;

	if (status != 0) {
		PDRAW_LOGE("scaler error: %d(%s)", -status, strerror(-status));
		return;
	}

	if (userdata == nullptr) {
		PDRAW_LOG_ERRNO("userdata", EINVAL);
		return;
	}
	if (out_frame == nullptr) {
		PDRAW_LOG_ERRNO("out_frame", EINVAL);
		return;
	}
	if (self->mState != STARTED) {
		PDRAW_LOGE("scaler is not started");
		return;
	}
	if ((self->mVscaleFlushPending) || (self->mInputChannelFlushPending)) {
		PDRAW_LOGI("frame output: flush pending, discard frame");
		return;
	}
	ret = mbuf_raw_video_frame_get_frame_info(out_frame, &info);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		return;
	}

	self->RawSink::lock();
	if (self->mInputMedia == nullptr) {
		self->RawSink::unlock();
		PDRAW_LOG_ERRNO("invalid input media", EPROTO);
		return;
	}

	ret = mbuf_raw_video_frame_get_ancillary_data(
		out_frame,
		PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME,
		&ancillaryData);
	if (ret < 0) {
		self->RawSink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_ancillary_data",
				-ret);
		return;
	}
	in_meta = (RawVideoMedia::Frame *)mbuf_ancillary_data_get_buffer(
		ancillaryData, NULL);
	out_meta = *in_meta;
	out_meta.scalerOutputTimestamp = pdraw_getTimestampFromMbufFrame(
		out_frame, VSCALE_ANCILLARY_KEY_OUTPUT_TIME);
	mbuf_ancillary_data_unref(ancillaryData);
	ancillaryData = NULL;

	ret = mbuf_raw_video_frame_remove_ancillary_data(
		out_frame, PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME);
	if (ret < 0) {
		self->RawSink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_remove_ancillary_data",
				-ret);
		return;
	}

	ret = mbuf_raw_video_frame_add_ancillary_buffer(
		out_frame,
		PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0) {
		self->RawSink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
				-ret);
		return;
	}

	self->RawSink::unlock();
	self->RawSource::lock();

	if (self->mOutputMedia == nullptr) {
		ret = self->createOutputMedia(&info, out_meta);
		if (ret < 0) {
			self->RawSource::unlock();
			PDRAW_LOG_ERRNO("createOutputMedia", -ret);
			return;
		}
	}

	/* Push the frame (unless it is silent) */
	if (!(info.info.flags & VDEF_FRAME_FLAG_SILENT)) {
		outputChannelCount =
			self->getOutputChannelCount(self->mOutputMedia);
		for (i = 0; i < outputChannelCount; i++) {
			channel = self->getOutputChannel(self->mOutputMedia, i);
			if (channel == nullptr) {
				PDRAW_LOGE("failed to get channel at index %d",
					   i);
				continue;
			}
			ret = channel->queue(out_frame);
			if (ret < 0)
				PDRAW_LOG_ERRNO("channel->queue", -ret);
		}
	} else {
		PDRAW_LOGD("silent frame (ignored)");
	}

	self->RawSource::unlock();
}


void VideoScaler::flushCb(struct vscale_scaler *scaler, void *userdata)
{
	VideoScaler *self = (VideoScaler *)userdata;

	if (userdata == nullptr) {
		PDRAW_LOG_ERRNO("userdata", EINVAL);
		return;
	}

	PDRAW_LOGD("scaler is flushed");
	self->mVscaleFlushPending = false;

	self->completeFlush();
}


void VideoScaler::stopCb(struct vscale_scaler *scaler, void *userdata)
{
	VideoScaler *self = (VideoScaler *)userdata;

	if (userdata == nullptr) {
		PDRAW_LOG_ERRNO("userdata", EINVAL);
		return;
	}

	PDRAW_LOGD("scaler is stopped");
	self->mVscaleStopPending = false;

	self->mCompleteStopPendingCount++;
	self->completeStop();
}

} /* namespace Pdraw */
