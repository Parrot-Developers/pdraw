/**
 * Parrot Drones Audio and Video Vector
 * Video sink wrapper library
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

#include "pdraw_vsink_priv.hpp"

#include <errno.h>
#include <string.h>

namespace PdrawVsink {

void Vsink::rawQueueEventCb([[maybe_unused]] struct pomp_evt *evt,
			    void *userdata)
{
	int res = 0;
	struct mbuf_raw_video_frame *frame = nullptr;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	const struct pdraw_video_frame *frameInfo = nullptr;
	struct pdraw_vsink_frame vframe = {};
	auto *self = static_cast<Vsink *>(userdata);

	if (self->mListener == nullptr) {
		std::scoped_lock lock(self->mMutex);
		self->mFrameReady = true;
		self->mCond.notify_one();
		return;
	}

	res = mbuf_raw_video_frame_queue_pop(self->mRaw.queue, &frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_pop", -res);
		goto out;
	}

	res = mbuf_raw_video_frame_get_ancillary_data(
		frame, PDRAW_ANCILLARY_DATA_KEY_VIDEOFRAME, &ancillaryData);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_ancillary_data", -res);
		goto out;
	}
	frameInfo = static_cast<const struct pdraw_video_frame *>(
		mbuf_ancillary_data_get_buffer(ancillaryData, nullptr));

	vframe.type = PDRAW_VSINK_VIDEO_MEDIA_TYPE_RAW;
	vframe.raw = frame;
	self->mListener->onFrameReady(self, &vframe, frameInfo);

out:
	if (ancillaryData != nullptr)
		mbuf_ancillary_data_unref(ancillaryData);
	if (frame != nullptr)
		mbuf_raw_video_frame_unref(frame);
}


void Vsink::onRawVideoSinkMediaAdded(
	[[maybe_unused]] Pdraw::IPdraw *pdraw,
	[[maybe_unused]] Pdraw::IPdraw::IRawVideoSink *sink,
	const struct pdraw_media_info *info)
{
	ULOGI("%s: id=%d", __func__, info->id);
}


void Vsink::onRawVideoSinkMediaRemoved(
	[[maybe_unused]] Pdraw::IPdraw *pdraw,
	[[maybe_unused]] Pdraw::IPdraw::IRawVideoSink *sink,
	const struct pdraw_media_info *info,
	bool restart)
{
	ULOGI("%s: id=%d (restart: %d)", __func__, info->id, (int)restart);
}


void Vsink::onRawVideoSinkFlush([[maybe_unused]] Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IRawVideoSink *sink)
{
	int res;

	struct mbuf_raw_video_frame_queue *queue = sink->getQueue();
	if (queue == nullptr) {
		ULOG_ERRNO("IRawVideoSink::getQueue", EPROTO);
		return;
	}

	res = mbuf_raw_video_frame_queue_flush(queue);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_flush", -res);
		return;
	}

	res = sink->queueFlushed();
	if (res < 0) {
		ULOG_ERRNO("IRawVideoSink::queueFlushed", -res);
		return;
	}
}


void Vsink::onRawVideoSinkDrain([[maybe_unused]] Pdraw::IPdraw *pdraw,
				Pdraw::IPdraw::IRawVideoSink *sink)
{
	int res;

	struct mbuf_raw_video_frame_queue *queue = sink->getQueue();
	if (queue == nullptr) {
		ULOG_ERRNO("IRawVideoSink::getQueue", EPROTO);
		return;
	}

	res = mbuf_raw_video_frame_queue_flush(queue);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_flush", -res);
		return;
	}

	res = sink->queueDrained();
	if (res < 0) {
		ULOG_ERRNO("IRawVideoSink::queueDrained", -res);
		return;
	}
}


int Vsink::processRawMediaAdded(const struct pdraw_media_info *info)
{
	int res = 0;
	struct pomp_evt *evt = nullptr;

	if (mRaw.sink == nullptr) {
		struct pdraw_video_sink_params params;
		memset(&params, 0, sizeof(params));
		params.queue_max_count = 1;
		Pdraw::IPdraw::IRawVideoSink *rawSink = nullptr;
		res = mPdraw->createRawVideoSink(
			info->id, &params, this, &rawSink);
		if (res < 0) {
			ULOG_ERRNO("IPdraw::createRawVideoSink", -res);
			goto out;
		}
		mRaw.sink.reset(rawSink);

		mRaw.queue = mRaw.sink->getQueue();
		if (mRaw.queue == nullptr) {
			ULOG_ERRNO("IRawVideoSink::getQueue", EPROTO);
			res = -EPROTO;
			goto out;
		}

		res = mbuf_raw_video_frame_queue_get_event(mRaw.queue, &evt);
		if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_get_event",
				   -res);
			goto out;
		}
		res = pomp_evt_attach_to_loop(
			evt, mLoop->get(), &Vsink::rawQueueEventCb, this);
		if (res < 0) {
			ULOG_ERRNO("pomp_evt_attach_to_loop", -res);
			goto out;
		}
	} else {
		res = mRaw.sink->setMediaId(info->id);
		if (res < 0) {
			ULOG_ERRNO("IRawVideoSink::setMediaId", -res);
			goto out;
		}
	}

out:
	return res;
}


int Vsink::getRawFrame(int timeoutMs,
		       struct mbuf_mem *frameMemory,
		       struct pdraw_video_frame *frameInfo,
		       struct mbuf_raw_video_frame **retFrame)
{
	int res;
	struct mbuf_raw_video_frame *inFrame = nullptr;
	struct mbuf_mem *memory = frameMemory;
	struct mbuf_ancillary_data *ancillaryData = nullptr;
	const struct pdraw_video_frame *inFrameInfo = nullptr;
	bool ownMem = false;

	ULOG_ERRNO_RETURN_ERR_IF(mRaw.queue == nullptr, EAGAIN);

	if (mListener != nullptr) {
		ULOGE("%s is unavailable when a Listener is registered",
		      __func__);
		return -EPERM;
	}

	*retFrame = nullptr;
	for (;;) {
		res = mbuf_raw_video_frame_queue_pop(mRaw.queue, &inFrame);
		if (res == -EAGAIN) {
			if (timeoutMs == 0)
				return -EAGAIN;

			std::unique_lock lock(mMutex);
			mWaiters++;
			bool ready;
			if (timeoutMs > 0) {
				ready = mCond.wait_for(
					lock,
					std::chrono::milliseconds(timeoutMs),
					[this] {
						return mFrameReady || mStopping;
					});
			} else {
				mCond.wait(lock, [this] {
					return mFrameReady || mStopping;
				});
				ready = true;
			}
			if (mStopping) {
				mWaiters--;
				mCond.notify_all();
				return -ECANCELED;
			}
			if (!ready) {
				mFrameReady = false;
				mWaiters--;
				mCond.notify_all();
				return -ETIMEDOUT;
			}
			mFrameReady = false;
			mWaiters--;
			mCond.notify_all();
		} else if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_pop", -res);
			return res;
		} else {
			break;
		}
	}

	if (!memory) {
		ssize_t len;
		/* Need to allocate our own memory */
		len = mbuf_raw_video_frame_get_packed_size(inFrame, false);
		if (len <= 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_get_packed_size",
				   -len);
			goto out;
		}
		ownMem = true;
		res = mbuf_mem_generic_new(len, &memory);
		if (res < 0) {
			ULOG_ERRNO("mbuf_mem_generic_new", -res);
			goto out;
		}
	}

	res = mbuf_raw_video_frame_get_ancillary_data(
		inFrame, PDRAW_ANCILLARY_DATA_KEY_VIDEOFRAME, &ancillaryData);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_ancillary_data", -res);
		goto out;
	}
	inFrameInfo = static_cast<const struct pdraw_video_frame *>(
		mbuf_ancillary_data_get_buffer(ancillaryData, nullptr));
	if (inFrameInfo != nullptr)
		*frameInfo = *inFrameInfo;

	res = mbuf_raw_video_frame_copy(inFrame, memory, true, retFrame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_copy", -res);
		goto out;
	}
	res = mbuf_raw_video_frame_finalize(*retFrame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_copy", -res);
		goto out;
	}

out:
	if (ownMem && memory)
		mbuf_mem_unref(memory);
	mbuf_raw_video_frame_unref(inFrame);
	mbuf_ancillary_data_unref(ancillaryData);
	if (res < 0) {
		mbuf_raw_video_frame_unref(*retFrame);
		*retFrame = nullptr;
	}
	return res;
}


int Vsink::detachRawEvent()
{
	struct pomp_evt *evt = nullptr;
	int res = 0;

	if (mRaw.queue == nullptr)
		return 0;

	res = mbuf_raw_video_frame_queue_get_event(mRaw.queue, &evt);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_get_event", -res);
	} else {
		res = pomp_evt_detach_from_loop(evt, mLoop->get());
		if (res < 0)
			ULOG_ERRNO("pomp_evt_detach_from_loop", -res);
	}
	mRaw.queue = nullptr;

	return res;
}


int Vsink::destroyRawSink()
{
	mRaw.sink.reset();
	return 0;
}

} /* namespace PdrawVsink */
