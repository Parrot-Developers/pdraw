/**
 * Parrot Drones Audio and Video Vector library
 * Video decoder element
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

#define ULOG_TAG pdraw_vdec
#include <ulog.h>

#include "pdraw_decoder_video.hpp"
#include "pdraw_session.hpp"
#include "pdraw_utils.hpp"
#if BUILD_LIBVIDEO_DECODE_MEDIACODEC
#	include <video-decode/vdec_mediacodec.h>
#endif

#include <time.h>
#include <unistd.h>

#include <vector>

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {


const struct vdec_cbs VideoDecoder::mDecoderCbs = {
	.frame_output = &VideoDecoder::frameOutputCb,
	.flush = &VideoDecoder::flushCb,
	.stop = &VideoDecoder::stopCb,
};


VideoDecoder::VideoDecoder(Session *session,
			   Element::Listener *elementListener,
			   Source::Listener *sourceListener) :
		FilterElement(session,
			      elementListener,
			      nullptr,
			      1,
			      nullptr,
			      0,
			      nullptr,
			      0,
			      nullptr,
			      0,
			      1,
			      sourceListener)
{
	const struct vdef_coded_format *supportedInputFormats;
	int supportedInputFormatsCount;

	Element::setClassName(__func__);

	mCompleteFlushHandler.set([this] { idleCompleteFlush(); });

	/* Supported input formats */
	supportedInputFormatsCount =
		vdec_get_all_supported_input_formats(&supportedInputFormats);
	if (supportedInputFormatsCount < 0) {
		PDRAW_LOG_ERRNO("vdec_get_supported_input_formats",
				-supportedInputFormatsCount);
	} else {
		setCodedVideoMediaFormatCaps(supportedInputFormats,
					     supportedInputFormatsCount);
	}

	setState(State::CREATED);
}


VideoDecoder::~VideoDecoder()
{
	int ret;

	if (mState != State::STOPPED && mState != State::CREATED)
		PDRAW_LOGW("decoder is still running");

	/* Remove any leftover idle callbacks */
	mSession->getPompLoop()->idleRemove(this);

	if (mVdec != nullptr) {
		ret = vdec_destroy(mVdec);
		if (ret < 0)
			PDRAW_LOG_ERRNO("vdec_destroy", -ret);
	}

	if (mOutputMedia != nullptr)
		PDRAW_LOGW("output media was not properly removed");
}


std::vector<uint8_t>
VideoDecoder::preparePsVector(const uint8_t *ps,
			      size_t psSize,
			      enum vdef_coded_data_format fmt)
{
	size_t prefixSize = (fmt == VDEF_CODED_DATA_FORMAT_RAW_NALU) ? 0 : 4;
	size_t totalSize = prefixSize + psSize;
	uint32_t start;

	ULOG_ERRNO_RETURN_VAL_IF(
		psSize > UINT32_MAX, EINVAL, std::vector<uint8_t>{});

	std::vector<uint8_t> psVector(totalSize);

	if (fmt != VDEF_CODED_DATA_FORMAT_RAW_NALU) {
		start = (fmt == VDEF_CODED_DATA_FORMAT_BYTE_STREAM)
				? htonl(0x00000001)
				: htonl(static_cast<uint32_t>(psSize));
		memcpy(psVector.data(), &start, sizeof(start));
	}
	memcpy(psVector.data() + prefixSize, ps, psSize);

	return psVector;
}


int VideoDecoder::start()
{
	int ret = 0;
	int err;
	enum vdef_coded_data_format fmt = VDEF_CODED_DATA_FORMAT_UNKNOWN;
	const uint8_t *vps = nullptr;
	const uint8_t *sps = nullptr;
	const uint8_t *pps = nullptr;
	size_t vpsSize = 0;
	size_t spsSize = 0;
	size_t ppsSize = 0;
	const InputPort *port = nullptr;
	struct vdec_config cfg = {};
	Channel *c = nullptr;
	CodedVideoChannel *channel = nullptr;

	if ((mState == State::STARTED) || (mState == State::STARTING)) {
		return 0;
	}
	if (mState != State::CREATED) {
		PDRAW_LOGE("%s: decoder is not created", __func__);
		return -EPROTO;
	}
	setState(State::STARTING);

	/* Get the input media and port */
	Sink::lock();
	unsigned int inputMediaCount = getInputMediaCount();
	if (inputMediaCount != 1) {
		Sink::unlock();
		PDRAW_LOGE("invalid input media count");
		ret = -EPROTO;
		goto error;
	}
	mInputMedia = dynamic_cast<CodedVideoMedia *>(getInputMedia(0));
	if (mInputMedia == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input media");
		ret = -EPROTO;
		goto error;
	}
	port = getInputPort(mInputMedia);
	if (port == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input port");
		ret = -EPROTO;
		goto error;
	}

	fmt = mInputMedia->format.data_format;
	if (fmt == VDEF_CODED_DATA_FORMAT_UNKNOWN) {
		Sink::unlock();
		PDRAW_LOGE("invalid input data format");
		ret = -EPROTO;
		goto error;
	}

	/* Initialize the decoder */
	cfg.implem = vdec_get_auto_implem_by_coded_format(&mInputMedia->format);
	if (cfg.implem == VDEC_DECODER_IMPLEM_AUTO) {
		Sink::unlock();
		PDRAW_LOGE(
			"no implementation found "
			"for format " VDEF_CODED_FORMAT_TO_STR_FMT,
			VDEF_CODED_FORMAT_TO_STR_ARG(&mInputMedia->format));
		ret = -EPROTO;
		goto error;
	}
	cfg.encoding = mInputMedia->format.encoding;
	cfg.low_delay = 1;
	cfg.gen_grey_idr = 1;
	ret = vdec_new(mSession->getLoop(), &cfg, &mDecoderCbs, this, &mVdec);
	if (ret < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("vdec_new", -ret);
		goto error;
	}

	/* Configure the decoder */
	switch (mInputMedia->format.encoding) {
	case VDEF_ENCODING_H264: {
		ret = mInputMedia->getPs(
			nullptr, nullptr, &sps, &spsSize, &pps, &ppsSize);
		if (ret < 0) {
			Sink::unlock();
			PDRAW_LOG_ERRNO("media->getPs", -ret);
			goto error;
		}

		try {
			std::vector<uint8_t> spsBuffer =
				VideoDecoder::preparePsVector(
					sps, spsSize, fmt);
			std::vector<uint8_t> ppsBuffer =
				VideoDecoder::preparePsVector(
					pps, ppsSize, fmt);

			ret = vdec_set_h264_ps(mVdec,
					       spsBuffer.data(),
					       spsBuffer.size(),
					       ppsBuffer.data(),
					       ppsBuffer.size(),
					       &mInputMedia->format);
			if (ret < 0) {
				Sink::unlock();
				PDRAW_LOG_ERRNO("vdec_set_h264_ps", -ret);
				goto error;
			}
		} catch (const std::bad_alloc &) {
			Sink::unlock();
			ret = -ENOMEM;
			PDRAW_LOG_ERRNO("std::vector allocation failed", -ret);
			goto error;
		}
		break;
	}
	case VDEF_ENCODING_H265: {
		ret = mInputMedia->getPs(
			&vps, &vpsSize, &sps, &spsSize, &pps, &ppsSize);
		if (ret < 0) {
			Sink::unlock();
			PDRAW_LOG_ERRNO("media->getPs", -ret);
			goto error;
		}

		try {
			std::vector<uint8_t> vpsBuffer =
				VideoDecoder::preparePsVector(
					vps, vpsSize, fmt);
			std::vector<uint8_t> spsBuffer =
				VideoDecoder::preparePsVector(
					sps, spsSize, fmt);
			std::vector<uint8_t> ppsBuffer =
				VideoDecoder::preparePsVector(
					pps, ppsSize, fmt);

			ret = vdec_set_h265_ps(mVdec,
					       vpsBuffer.data(),
					       vpsBuffer.size(),
					       spsBuffer.data(),
					       spsBuffer.size(),
					       ppsBuffer.data(),
					       ppsBuffer.size(),
					       &mInputMedia->format);
			if (ret < 0) {
				Sink::unlock();
				PDRAW_LOG_ERRNO("vdec_set_h265_ps", -ret);
				goto error;
			}
		} catch (const std::bad_alloc &) {
			Sink::unlock();
			ret = -ENOMEM;
			PDRAW_LOG_ERRNO("std::vector allocation failed", -ret);
			goto error;
		}
		break;
	}
	case VDEF_ENCODING_JPEG:
		ret = vdec_set_jpeg_params(mVdec, &mInputMedia->info);
		if (ret < 0) {
			Sink::unlock();
			PDRAW_LOG_ERRNO("vdec_set_jpeg_params", -ret);
			goto error;
		}
		break;

	default:
		Sink::unlock();
		PDRAW_LOGE("unsupported input media encoding (%s)",
			   vdef_encoding_to_str(mInputMedia->format.encoding));
		ret = -EPROTO;
		goto error;
	}

	/* Setup the input port */
	c = port->channel.get();
	channel = dynamic_cast<CodedVideoChannel *>(c);
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input channel");
		ret = -EPROTO;
		goto error;
	}
	mInputBufferQueue = mbuf::Queue::wrapExisting(
		vdec_get_input_buffer_queue(mVdec), false);
	channel->setQueue(this, mInputBufferQueue.get());
	mInputBufferPool = vdec_get_input_buffer_pool(mVdec);
	channel->setPool(this, mInputBufferPool);

	Sink::unlock();

	setState(State::STARTED);

	return 0;

error:
	if (mInputMedia != nullptr) {
		/* mInputMedia must be removed synchronously to avoid holding a
		 * reference to a media that can be destroyed at any moment by
		 * the Source element */
		err = removeInputMedia(mInputMedia);
		if (err < 0)
			PDRAW_LOG_ERRNO("removeInputMedia", -err);
		else
			mInputMedia = nullptr;
	}
	err = stop();
	if (err < 0)
		PDRAW_LOG_ERRNO("stop", -err);

	return ret;
}


int VideoDecoder::stop()
{
	int ret;

	if ((mState == State::STOPPED) || (mState == State::STOPPING))
		return 0;
	if (mState == State::CREATED) {
		/* Skip flush/drain for unstarted elements to prevent them from
		 * getting stuck in CREATED and blocking session stop */
		setState(State::STOPPED);
		return 0;
	}
	if ((mState != State::STARTING) && (mState != State::STARTED)) {
		PDRAW_LOGE("%s: decoder is not started", __func__);
		return -EPROTO;
	}
	setState(State::STOPPING);
	mVdecStopPending = true;

	Source::lock();
	if (mOutputMedia != nullptr)
		mOutputMedia->setTearingDown();
	Source::unlock();

	/* Flush everything */
	ret = flush();
	if (ret < 0 && ret != -EALREADY)
		PDRAW_LOG_ERRNO("flush", -ret);
	else
		ret = 0;

	/* When the flush is complete, stopping will be triggered */
	return ret;
}


int VideoDecoder::flush(bool discard)
{
	int ret = 0;
	int err;
	unsigned int outputChannelCount;
	Channel *outputChannel;

	switch (getFlushingState()) {
	case FlushingState::UNFLUSHED:
		/* OK */
		break;
	case FlushingState::FLUSHING:
		return -EALREADY;
	case FlushingState::FLUSHED:
		if (mInputBufferQueue != nullptr) {
			ret = mInputBufferQueue->getCount();
			if (ret < 0) {
				PDRAW_LOG_ERRNO("queue::getCount", -ret);
				return ret;
			} else if (ret > 0) {
				setFlushingState(FlushingState::UNFLUSHED);
				break;
			}
		}
		PDRAW_LOGD("decoder is already %s, nothing to do",
			   discard ? "flushed" : "drained");
		ret = mSession->getPompLoop()->idleAdd(&mCompleteFlushHandler,
						       this);
		if (ret < 0)
			PDRAW_LOG_ERRNO("Loop::idleAdd", -ret);
		else
			setFlushingState(FlushingState::FLUSHING, discard);
		return ret;
	default:
		break;
	}

	setFlushingState(FlushingState::FLUSHING, discard);

	Source::lock();
	if (mOutputMedia != nullptr) {
		if (mFlushDiscard) {
			/* Flush the output channels (async) */
			outputChannelCount =
				getOutputChannelCount(mOutputMedia.get());
			for (unsigned int i = 0; i < outputChannelCount; i++) {
				outputChannel =
					getOutputChannel(mOutputMedia.get(), i);
				if (outputChannel == nullptr) {
					PDRAW_LOGW(
						"failed to get output channel "
						"at index %d",
						i);
					continue;
				}
				err = outputChannel->flush();
				if (err < 0 && err != -EALREADY) {
					PDRAW_LOG_ERRNO(
						"channel->flush "
						"(channel index=%u)",
						-err,
						i);
				}
			}
		} else {
			/* Drain event is called once flush is complete */
			mOutputChannelDrainRequired = true;
		}
	}
	Source::unlock();

	/* Flush the decoder (async)
	 * (the input channel queue is flushed by vdec) */
	if (mVdec != nullptr) {
		if (!mVdecFlushPending) {
			ret = vdec_flush(mVdec, mFlushDiscard);
			if (ret < 0)
				PDRAW_LOG_ERRNO("vdec_flush", -ret);
			else
				mVdecFlushPending = true;
		}
	} else {
		completeFlush();
	}

	return ret;
}


void VideoDecoder::completeFlush()
{
	int ret;
	int err;
	unsigned int outputChannelCount;
	Channel *outputChannel;
	bool pending = false;

	if (mVdecFlushPending)
		return;

	/* Drain the output channels (async) */
	Source::lock();
	if (!mFlushDiscard && mOutputChannelDrainRequired &&
	    mOutputMedia != nullptr) {
		mOutputChannelDrainRequired = false;
		outputChannelCount = getOutputChannelCount(mOutputMedia.get());
		for (unsigned i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia.get(), i);
			if (outputChannel == nullptr) {
				PDRAW_LOGW(
					"failed to get output channel "
					"at index %d",
					i);
				continue;
			}
			err = outputChannel->drain();
			if (err < 0 && err != -EALREADY) {
				PDRAW_LOG_ERRNO(
					"channel->drain (channel index=%u)",
					-err,
					i);
			}
		}
	}
	if (mOutputMedia != nullptr) {
		outputChannelCount = getOutputChannelCount(mOutputMedia.get());
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			outputChannel = getOutputChannel(mOutputMedia.get(), i);
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

	Sink::lock();
	if ((mInputMedia != nullptr) && mInputChannelFlushPending) {
		mInputChannelFlushPending = false;
		auto *inputChannel = dynamic_cast<CodedVideoChannel *>(
			getInputChannel(mInputMedia));
		if (inputChannel == nullptr) {
			PDRAW_LOGE("failed to get input channel");
		} else {
			if (mFlushDiscard)
				ret = inputChannel->flushDone();
			else
				ret = inputChannel->drainDone();
			if (ret < 0) {
				PDRAW_LOG_ERRNO("channel->%s",
						-ret,
						mFlushDiscard ? "flushDone"
							      : "drainDone");
			}
		}
	}
	Sink::unlock();

	completeResync();

	tryStop();
}


void VideoDecoder::idleCompleteFlush()
{
	completeFlush();
}


int VideoDecoder::tryStop()
{
	int ret;
	int outputChannelCount = 0;

	if (mState != State::STOPPING)
		return 0;

	/* Teardown the output channels
	 * Note: loop downwards because calling teardown on a channel may or
	 * may not synchronously remove the channel from the output port */
	Source::lock();
	if (mOutputMedia != nullptr) {
		outputChannelCount = getOutputChannelCount(mOutputMedia.get());

		for (int i = outputChannelCount - 1; i >= 0; i--) {
			Channel *channel =
				getOutputChannel(mOutputMedia.get(), i);
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
	Source::unlock();

	/* Stop the decoder
	 * tryStop() can be re-entered while STOPPING; guard against issuing
	 * vdec_stop() more than once (see VideoScaler/vscale_libyuv). */
	if ((mVdec != nullptr) && (!mVdecStopIssued)) {
		mVdecStopIssued = true;
		ret = vdec_stop(mVdec);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("vdec_stop", -ret);
			return ret;
		}
	}
	/* Else, delay completeStop() after removing the input port */

	/* Remove the input port */
	Sink::lock();
	if (mInputMedia != nullptr) {
		auto *channel = dynamic_cast<CodedVideoChannel *>(
			getInputChannel(mInputMedia));
		if (channel == nullptr) {
			PDRAW_LOGE("failed to get channel");
		} else {
			channel->setQueue(this, nullptr);
			channel->setPool(this, nullptr);
		}

		ret = removeInputMedia(mInputMedia);
		if (ret < 0)
			PDRAW_LOG_ERRNO("removeInputMedia", -ret);
		else
			mInputMedia = nullptr;
	}
	Sink::unlock();

	if (mVdec == nullptr) {
		mVdecStopPending = false;
		completeStop();
	}

	return 0;
}


void VideoDecoder::completeStop()
{
	int ret;
	unsigned int outputChannelCount;

	Source::lock();
	if (mOutputMedia == nullptr) {
		Source::unlock();
		goto exit;
	}
	outputChannelCount = getOutputChannelCount(mOutputMedia.get());
	if (outputChannelCount > 0) {
		Source::unlock();
		return;
	}

	/* Remove the output port */
	if (Source::mListener) {
		Source::mListener->onOutputMediaRemoved(
			this, mOutputMedia.get(), nullptr);
	}
	ret = removeOutputPort(mOutputMedia.get());
	if (ret < 0) {
		PDRAW_LOG_ERRNO("removeOutputPort", -ret);
	} else {
		mOutputMedia.reset();
	}

	Source::unlock();

exit:
	if ((!mVdecStopPending) && (mOutputMedia == nullptr))
		setState(State::STOPPED);
}


void VideoDecoder::resync()
{
	int ret;

	Sink::lock();

	if (mResyncPending) {
		Sink::unlock();
		PDRAW_LOGD(
			"%s: decoder is already synchronizing, nothing to do",
			__func__);
		return;
	}

	if (getFlushingState() == FlushingState::FLUSHED) {
		Sink::unlock();
		PDRAW_LOGD("%s: decoder is already flushed, nothing to do",
			   __func__);
		return;
	}

	mResyncPending = true;

	ret = vdec_flush(mVdec, 1);
	if (ret < 0)
		PDRAW_LOG_ERRNO("vdec_flush", -ret);
	else
		mVdecFlushPending = true;

	Sink::unlock();
}


void VideoDecoder::completeResync()
{
	int ret;

	Sink::lock();

	if (!mResyncPending) {
		Sink::unlock();
		return;
	}

	auto *inputChannel =
		dynamic_cast<CodedVideoChannel *>(getInputChannel(mInputMedia));
	if (inputChannel == nullptr) {
		PDRAW_LOGE("failed to get input channel");
	} else {
		ret = inputChannel->resync();
		if (ret < 0)
			PDRAW_LOG_ERRNO("channel->resync", -ret);
	}

	mResyncPending = false;
	Sink::unlock();
}


int VideoDecoder::createOutputMedia(
	const struct vdef_raw_frame *frameInfo,
	[[maybe_unused]] const RawVideoMedia::Frame &frame)
{

	int ret;

	Source::lock();

	try {
		mOutputMedia = std::make_unique<RawVideoMedia>(mSession);
	} catch (const std::bad_alloc &) {
		Source::unlock();
		PDRAW_LOGE("output media allocation failed");
		return -ENOMEM;
	}
	std::string path = mInputMedia->getPath() + ">" + Element::getName() +
			   "$" + mOutputMedia->getName();
	mOutputMedia->setPath(path);

	ret = addOutputPort(mOutputMedia.get());
	if (ret < 0) {
		Source::unlock();
		PDRAW_LOG_ERRNO("addOutputPort", -ret);
		return ret;
	}

	mOutputMedia->format = frameInfo->format;
	vdef_frame_to_format_info(&frameInfo->info, &mOutputMedia->info);
	mOutputMedia->info.framerate = mInputMedia->info.framerate;
	mOutputMedia->sessionMeta = mInputMedia->sessionMeta;
	mOutputMedia->copyPropertiesFrom(mInputMedia);

	Source::unlock();

	if (Source::mListener)
		Source::mListener->onOutputMediaAdded(
			this, mOutputMedia.get(), nullptr);

	return 0;
}


void VideoDecoder::onCodedVideoChannelQueue(
	CodedVideoChannel *channel,
	struct mbuf_coded_video_frame *frame)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_IF(frame == nullptr, EINVAL);

	if (mState != State::STARTED) {
		PDRAW_LOGE("frame input: decoder is not started");
		return;
	}
	if (mVdecFlushPending || mInputChannelFlushPending) {
		PDRAW_LOGI("frame input: flush pending, discard frame");
		return;
	}
	Sink::lock();
	if (mInputBufferQueue == nullptr ||
	    !channel->hasQueue(mInputBufferQueue.get())) {
		Sink::unlock();
		PDRAW_LOGE("invalid queue");
		return;
	}

	Sink::onCodedVideoChannelQueue(channel, frame);
	setFlushingState(FlushingState::UNFLUSHED);
	Sink::unlock();
}


void VideoDecoder::onChannelFlush(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	PDRAW_LOGD("flushing input channel");
	mInputChannelFlushPending = true;

	int ret = flush();
	if (ret < 0 && ret != -EALREADY)
		PDRAW_LOG_ERRNO("flush", -ret);
}


void VideoDecoder::onChannelDrain(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	PDRAW_LOGD("draining input channel");
	mInputChannelFlushPending = true;

	int ret = drain();
	if (ret < 0 && ret != -EALREADY)
		PDRAW_LOG_ERRNO("drain", -ret);
}


void VideoDecoder::onChannelFlushed(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

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


void VideoDecoder::onChannelDrained(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

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


void VideoDecoder::onChannelTeardown(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	PDRAW_LOGD("tearing down input channel");

	int ret = stop();
	if (ret < 0)
		PDRAW_LOG_ERRNO("stop", -ret);
}


void VideoDecoder::onChannelUnlink(Channel *channel)
{
	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	Source::onChannelUnlink(channel);

	if (mState == State::STOPPING)
		completeStop();
}


void VideoDecoder::onChannelSessionMetaUpdate(Channel *channel)
{
	struct vmeta_session tmpSessionMeta;

	ULOG_ERRNO_RETURN_IF(channel == nullptr, EINVAL);

	Sink::lock();
	if (mInputMedia == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("%s: input media not found", __func__);
		return;
	}
	tmpSessionMeta = mInputMedia->sessionMeta;
	Sink::unlock();

	Source::lock();
	if (mOutputMedia == nullptr) {
		Source::unlock();
		PDRAW_LOGE("%s: output media not found", __func__);
		return;
	}
	mOutputMedia->sessionMeta = tmpSessionMeta;
	Source::unlock();

	PDRAW_LOGD("updating session metadata");

	FilterElement::onChannelSessionMetaUpdate(channel);
}


void VideoDecoder::frameOutputCb([[maybe_unused]] struct vdec_decoder *dec,
				 int status,
				 struct mbuf_raw_video_frame *out_frame,
				 void *userdata)
{

	int ret;
	auto *self = static_cast<VideoDecoder *>(userdata);
	struct vdef_raw_frame info;
	struct mbuf_ancillary_data *ancillaryData;
	const CodedVideoMedia::Frame *in_meta;
	RawVideoMedia::Frame out_meta{};
	unsigned int outputChannelCount;

	if (status != 0) {
		PDRAW_LOGE("decoder error %d(%s), resync required",
			   -status,
			   strerror(-status));
		self->resync();
		return;
	}

	PDRAW_LOG_ERRNO_RETURN_IF(userdata == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_IF(out_frame == nullptr, EINVAL);

	if (self->mState != State::STARTED) {
		PDRAW_LOGE("frame output: decoder is not started");
		return;
	}
	if (self->mFlushDiscard &&
	    (self->mVdecFlushPending || self->mInputChannelFlushPending)) {
		PDRAW_LOGI("frame output: flush pending, discard frame");
		return;
	}

	self->Sink::lock();
	if (self->mInputMedia == nullptr) {
		self->Sink::unlock();
		PDRAW_LOG_ERRNO("invalid input media", EPROTO);
		return;
	}

	ret = mbuf_raw_video_frame_get_frame_info(out_frame, &info);
	if (ret < 0) {
		self->Sink::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		return;
	}
	ret = mbuf_raw_video_frame_get_ancillary_data(
		out_frame,
		PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME,
		&ancillaryData);
	if (ret < 0) {
		self->Sink::unlock();
		PDRAW_LOG_ERRNO(
			"mbuf_raw_video_frame_get_ancillary_data:pdraw_in",
			-ret);
		return;
	}
	in_meta = static_cast<const CodedVideoMedia::Frame *>(
		mbuf_ancillary_data_get_buffer(ancillaryData, nullptr));
	out_meta.ntpTimestamp = in_meta->ntpTimestamp;
	out_meta.ntpUnskewedTimestamp = in_meta->ntpUnskewedTimestamp;
	out_meta.ntpRawTimestamp = in_meta->ntpRawTimestamp;
	out_meta.ntpRawUnskewedTimestamp = in_meta->ntpRawUnskewedTimestamp;
	out_meta.playTimestamp = in_meta->playTimestamp;
	out_meta.captureTimestamp = in_meta->captureTimestamp;
	out_meta.localTimestamp = in_meta->localTimestamp;
	out_meta.localTimestampPrecision = in_meta->localTimestampPrecision;
	out_meta.recvStartTimestamp = in_meta->recvStartTimestamp;
	out_meta.recvEndTimestamp = in_meta->recvEndTimestamp;
	out_meta.demuxOutputTimestamp = in_meta->demuxOutputTimestamp;
	out_meta.decoderOutputTimestamp = pdraw_getTimestampFromMbufFrame(
		out_frame, VDEC_ANCILLARY_KEY_OUTPUT_TIME);
	ret = mbuf_ancillary_data_unref(ancillaryData);
	if (ret < 0)
		PDRAW_LOG_ERRNO("mbuf_ancillary_data_unref", -ret);

	/* Remove the PDrAW input ancillary data */
	ret = mbuf_raw_video_frame_remove_ancillary_data(
		out_frame, PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME);
	if (ret < 0)
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_remove_ancillary_data",
				-ret);

	self->Sink::unlock();
	self->Source::lock();

	if (self->mOutputMedia == nullptr) {
		/* Create the output media now that the format is known */
		ret = self->createOutputMedia(&info, out_meta);
		if (ret < 0) {
			self->Source::unlock();
			PDRAW_LOG_ERRNO("createOutputMedia", -ret);
			return;
		}
	} else {
		/* TODO: This should be generic for every filter element */
		/* Update the output media metadata */
		self->mOutputMedia->sessionMeta =
			self->mInputMedia->sessionMeta;
	}

	ret = mbuf_raw_video_frame_add_ancillary_buffer(
		out_frame,
		PDRAW_ANCILLARY_DATA_KEY_RAWVIDEOFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0) {
		self->Source::unlock();
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
				-ret);
		return;
	}

	/* Push the frame (unless it is silent) */
	if (!(info.info.flags & VDEF_FRAME_FLAG_SILENT)) {
		outputChannelCount =
			self->getOutputChannelCount(self->mOutputMedia.get());
		for (unsigned int i = 0; i < outputChannelCount; i++) {
			Channel *c = self->getOutputChannel(
				self->mOutputMedia.get(), i);
			auto *channel = dynamic_cast<RawVideoChannel *>(c);
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

	self->Source::unlock();
}


void VideoDecoder::flushCb([[maybe_unused]] struct vdec_decoder *dec,
			   void *userdata)
{

	auto *self = static_cast<VideoDecoder *>(userdata);

	PDRAW_LOG_ERRNO_RETURN_IF(userdata == nullptr, EINVAL);

	PDRAW_LOGD("decoder is flushed");
	self->mVdecFlushPending = false;

	self->completeFlush();
}


void VideoDecoder::stopCb([[maybe_unused]] struct vdec_decoder *dec,
			  void *userdata)
{

	auto *self = static_cast<VideoDecoder *>(userdata);

	PDRAW_LOG_ERRNO_RETURN_IF(userdata == nullptr, EINVAL);

	PDRAW_LOGD("decoder is stopped");
	self->mVdecStopPending = false;
	self->completeStop();
}

} /* namespace Pdraw */
