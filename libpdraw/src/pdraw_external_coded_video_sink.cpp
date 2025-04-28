/**
 * Parrot Drones Audio and Video Vector library
 * Application external coded video sink
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

#define ULOG_TAG pdraw_external_coded_video_sink
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_external_coded_video_sink.hpp"
#include "pdraw_session.hpp"

#include <time.h>

namespace Pdraw {


const struct h264_ctx_cbs ExternalCodedVideoSink::mH264ReaderCbs = {};


#define NB_SUPPORTED_FORMATS 4
static struct vdef_coded_format supportedFormats[NB_SUPPORTED_FORMATS];
static pthread_once_t supportedFormatsIsInit = PTHREAD_ONCE_INIT;
static void initializeSupportedFormats(void)
{
	supportedFormats[0] = vdef_h264_byte_stream;
	supportedFormats[1] = vdef_h264_avcc;
	supportedFormats[2] = vdef_h265_byte_stream;
	supportedFormats[3] = vdef_h265_hvcc;
}


ExternalCodedVideoSink::ExternalCodedVideoSink(
	Session *session,
	const struct vdef_coded_format *requiredCodedFormat,
	Element::Listener *elementListener,
	IPdraw::ICodedVideoSink::Listener *listener,
	CodedVideoSinkWrapper *wrapper,
	const struct pdraw_video_sink_params *params) :
		SinkElement(session,
			    elementListener,
			    wrapper,
			    1,
			    nullptr,
			    0,
			    nullptr,
			    0,
			    nullptr,
			    0)
{
	Element::setClassName(__func__);
	mVideoSinkListener = listener;
	mVideoSink = wrapper;
	mParams = *params;
	mInputMedia = nullptr;
	mInputFrameQueue = nullptr;
	mIsFlushed = true;
	mInputChannelFlushPending = false;
	mTearingDown = false;
	mNeedSync = true;
	mH264Reader = nullptr;

	(void)pthread_once(&supportedFormatsIsInit, initializeSupportedFormats);

	if (requiredCodedFormat &&
	    requiredCodedFormat->data_format !=
		    VDEF_CODED_DATA_FORMAT_UNKNOWN &&
	    requiredCodedFormat->encoding != VDEF_ENCODING_UNKNOWN)
		setCodedVideoMediaFormatCaps(requiredCodedFormat, 1);
	else
		setCodedVideoMediaFormatCaps(supportedFormats,
					     NB_SUPPORTED_FORMATS);

	setState(CREATED);
}


ExternalCodedVideoSink::~ExternalCodedVideoSink(void)
{
	int ret;

	if (mState == STARTED)
		PDRAW_LOGW("video sink is still running");

	/* Make sure listener functions will no longer be called */
	mVideoSinkListener = nullptr;

	/* Remove any leftover idle callbacks */
	ret = pomp_loop_idle_remove_by_cookie(mSession->getLoop(), this);
	if (ret < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -ret);

	/* Flush and destroy the queue */
	if (mInputFrameQueue != nullptr) {
		ret = mbuf_coded_video_frame_queue_flush(mInputFrameQueue);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_flush",
					-ret);
		ret = mbuf_coded_video_frame_queue_destroy(mInputFrameQueue);
		if (ret < 0)
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_destroy",
					-ret);
		mInputFrameQueue = nullptr;
	}

	if (mH264Reader) {
		ret = h264_reader_destroy(mH264Reader);
		if (ret < 0)
			PDRAW_LOG_ERRNO("h264_reader_destroy", -ret);
		mH264Reader = nullptr;
	}
}


int ExternalCodedVideoSink::start(void)
{
	if ((mState == STARTED) || (mState == STARTING)) {
		return 0;
	}
	if (mState != CREATED) {
		PDRAW_LOGE("%s: video sink is not created", __func__);
		return -EPROTO;
	}
	setState(STARTING);

	/* Get the input media and port */
	Sink::lock();
	unsigned int inputMediaCount = getInputMediaCount();
	if (inputMediaCount != 1) {
		Sink::unlock();
		PDRAW_LOGE("invalid input media count");
		return -EPROTO;
	}
	mInputMedia = dynamic_cast<CodedVideoMedia *>(getInputMedia(0));
	if (mInputMedia == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input media");
		return -EPROTO;
	}
	InputPort *port;
	port = getInputPort(mInputMedia);
	if (port == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input port");
		return -EPROTO;
	}

	/* Create the queue */
	struct mbuf_coded_video_frame_queue_args queueArgs = {};
	queueArgs.max_frames = mParams.queue_max_count;
	int res = mbuf_coded_video_frame_queue_new_with_args(&queueArgs,
							     &mInputFrameQueue);
	if (res < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_queue_new_with_args",
				-res);
		return res;
	}

	res = h264_reader_new(&mH264ReaderCbs, this, &mH264Reader);
	if (res < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("h264_reader_new", -res);
		return res;
	}

	/* Setup the input port */
	Channel *c = port->channel;
	CodedVideoChannel *channel = dynamic_cast<CodedVideoChannel *>(c);
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("invalid input channel");
		return -EPROTO;
	}
	channel->setQueue(this, mInputFrameQueue);

	Sink::unlock();

	setState(STARTED);

	return 0;
}


int ExternalCodedVideoSink::stop(void)
{
	int ret;
	CodedVideoChannel *channel = nullptr;

	if ((mState == STOPPED) || (mState == STOPPING))
		return 0;
	if (mState != STARTED) {
		PDRAW_LOGE("%s: video sink is not started", __func__);
		return -EPROTO;
	}
	setState(STOPPING);

	/* Make sure listener functions will no longer be called.
	 * Note: the ICodedVideoSink::Listener::onCodedVideoSinkFlush function
	 * will not be called, but the ExternalCodedVideoSink::stop function
	 * is only called by the API object destructor, and this is
	 * precisely when we do not want to call listener functions any more;
	 * when destroying the API object, outstanding video frames should have
	 * been previously released by the caller anyway. */
	mVideoSinkListener = nullptr;

	Sink::lock();

	if (mInputMedia == nullptr) {
		Sink::unlock();
		setState(STOPPED);
		return 0;
	}

	channel =
		dynamic_cast<CodedVideoChannel *>(getInputChannel(mInputMedia));
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("failed to get channel");
		return -EPROTO;
	}

	Sink::unlock();

	ret = channelTeardown(channel);
	if (ret < 0)
		PDRAW_LOG_ERRNO("channelTeardown", -ret);

	return 0;
}


int ExternalCodedVideoSink::resync(void)
{
	int ret;
	Channel *channel = nullptr;

	if (mState != STARTED) {
		PDRAW_LOGE("%s: video sink is not started", __func__);
		return -EPROTO;
	}

	Sink::lock();

	ret = flush();
	if (ret < 0) {
		Sink::unlock();
		PDRAW_LOG_ERRNO("flush", -ret);
		return ret;
	}

	channel = getInputChannel(mInputMedia);
	if (channel == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("failed to get channel");
		return -EPROTO;
	}

	ret = channel->resync();
	if (ret < 0)
		PDRAW_LOG_ERRNO("channel->resync", -ret);

	mNeedSync = true;
	Sink::unlock();

	return ret;
}


int ExternalCodedVideoSink::flush(void)
{
	int err;

	if (mIsFlushed) {
		PDRAW_LOGD("video sink is already flushed, nothing to do");
		err = pomp_loop_idle_add_with_cookie(
			mSession->getLoop(), &idleFlushDone, this, this);
		if (err < 0)
			PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);
		return 0;
	}

	/* Signal the application for flushing */
	err = pomp_loop_idle_add_with_cookie(
		mSession->getLoop(), callVideoSinkFlush, this, this);
	if (err < 0)
		PDRAW_LOG_ERRNO("pomp_loop_idle_add_with_cookie", -err);

	return 0;
}


int ExternalCodedVideoSink::flushDone(void)
{
	int ret;

	Sink::lock();

	if (mInputMedia == nullptr)
		goto exit;

	if (mInputChannelFlushPending) {
		Channel *channel = getInputChannel(mInputMedia);
		if (channel == nullptr) {
			PDRAW_LOGE("failed to get channel");
		} else {
			mIsFlushed = true;
			mInputChannelFlushPending = false;
			ret = channel->flushDone();
			if (ret < 0)
				PDRAW_LOG_ERRNO("channel->flushDone", -ret);
		}
	}

exit:
	Sink::unlock();

	if (mState == STOPPING)
		setState(STOPPED);

	return 0;
}


void ExternalCodedVideoSink::idleFlushDone(void *userdata)
{
	ExternalCodedVideoSink *self = (ExternalCodedVideoSink *)userdata;
	(void)self->flushDone();
}


int ExternalCodedVideoSink::prepareCodedVideoFrame(
	CodedVideoChannel *channel,
	struct mbuf_coded_video_frame *frame)
{
	int ret;
	CodedVideoMedia::Frame *in_meta;
	struct pdraw_video_frame out_meta = {};
	struct mbuf_ancillary_data *ancillaryData = nullptr;

	struct mbuf_coded_video_frame_queue *queue = channel->getQueue(this);
	if (queue == nullptr) {
		PDRAW_LOGE("invalid queue");
		return -ENOENT;
	}
	if (queue != mInputFrameQueue) {
		PDRAW_LOGE("invalid input buffer queue");
		return -EPROTO;
	}

	ret = mbuf_coded_video_frame_get_frame_info(frame, &out_meta.coded);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -ret);
		return ret;
	}

	/* Get the CodedVideoMedia::Frame input metadata */
	ret = mbuf_coded_video_frame_get_ancillary_data(
		frame,
		PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME,
		&ancillaryData);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_ancillary_data",
				-ret);
		return ret;
	}

	in_meta = (CodedVideoMedia::Frame *)mbuf_ancillary_data_get_buffer(
		ancillaryData, nullptr);

	if (!vdef_coded_format_intersect(&out_meta.coded.format,
					 mCodedVideoMediaFormatCaps,
					 mCodedVideoMediaFormatCapsCount)) {
		PDRAW_LOGE("unsupported coded video input format");
		ret = -EPROTO;
		goto out;
	}

	out_meta.format = VDEF_FRAME_TYPE_CODED;
	out_meta.ntp_timestamp = in_meta->ntpTimestamp;
	out_meta.ntp_unskewed_timestamp = in_meta->ntpUnskewedTimestamp;
	out_meta.ntp_raw_timestamp = in_meta->ntpRawTimestamp;
	out_meta.ntp_raw_unskewed_timestamp = in_meta->ntpRawUnskewedTimestamp;
	out_meta.play_timestamp = in_meta->playTimestamp;
	out_meta.capture_timestamp = in_meta->captureTimestamp;
	out_meta.local_timestamp = in_meta->localTimestamp;
	out_meta.is_ref = in_meta->isRef;
	out_meta.is_sync = in_meta->isSync;

	/* If the frame is handled by multiple external video sinks, this key
	 * might already have been filled by another sink, so we don't consider
	 * -EEXIST as an error */
	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		frame,
		PDRAW_ANCILLARY_DATA_KEY_VIDEOFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0 && ret != -EEXIST) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
				-ret);
		goto out;
	}
	ret = 0;

out:
	if (ancillaryData != nullptr)
		mbuf_ancillary_data_unref(ancillaryData);
	return ret;
}


int ExternalCodedVideoSink::writeGreyIdr(CodedVideoChannel *channel,
					 struct CodedVideoMedia::Frame *inFrame,
					 struct vdef_coded_frame *inInfo,
					 uint64_t *ntpDelta,
					 uint64_t *ntpUnskewedDelta,
					 uint64_t *ntpRawDelta,
					 uint64_t *ntpRawUnskewedDelta,
					 uint64_t *playDelta)
{
	int ret;
	struct mbuf_mem *idr_mem = nullptr;
	struct mbuf_coded_video_frame *idr_frame = nullptr;
	struct h264_bitstream bs = {};
	struct h264_slice_header *sh = nullptr;
	uint32_t mbTotal, sc;
	const uint8_t *sps, *pps;
	uint8_t *data;
	size_t len, spsSize, ppsSize;
	struct h264_nalu_header nh = {};
	struct vdef_coded_frame idr_info;
	struct pdraw_video_frame out_meta = {};
	struct vdef_nalu nalu = {};

	PDRAW_LOG_ERRNO_RETURN_ERR_IF(channel == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(inFrame == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(inInfo == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(ntpDelta == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(ntpUnskewedDelta == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(ntpRawDelta == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(ntpRawUnskewedDelta == nullptr, EINVAL);
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(playDelta == nullptr, EINVAL);

	ret = mInputMedia->getPs(
		nullptr, nullptr, &sps, &spsSize, &pps, &ppsSize);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("media->getPs", -ret);
		return ret;
	}

	/* Give SPS and PPS to h264_reader */
	ret = h264_reader_parse_nalu(mH264Reader, 0, sps, spsSize);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("h264_reader_parse_nalu", -ret);
		return ret;
	}

	ret = h264_reader_parse_nalu(mH264Reader, 0, pps, ppsSize);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("h264_reader_parse_nalu", -ret);
		return ret;
	}

	struct h264_ctx *ctx = h264_reader_get_ctx(mH264Reader);
	size_t outBufSize = mInputMedia->info.resolution.width *
			    mInputMedia->info.resolution.height * 3 / 4;

	if (!vdef_coded_format_intersect(&inInfo->format,
					 mCodedVideoMediaFormatCaps,
					 mCodedVideoMediaFormatCapsCount)) {
		PDRAW_LOGE("unsupported coded video input format");
		return -EPROTO;
	}

	/* Create buffer */
	ret = mbuf_mem_generic_new(outBufSize, &idr_mem);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_mem_generic_new", -ret);
		goto out;
	}

	idr_info = *inInfo;
	idr_info.type = VDEF_CODED_FRAME_TYPE_IDR;
	idr_info.info.flags |= VDEF_FRAME_FLAG_SILENT;

	out_meta.format = VDEF_FRAME_TYPE_CODED;
	out_meta.is_ref = 1;
	out_meta.is_sync = 1;
	out_meta.ntp_timestamp = inFrame->ntpTimestamp;
	out_meta.ntp_unskewed_timestamp = inFrame->ntpUnskewedTimestamp;
	out_meta.ntp_raw_timestamp = inFrame->ntpRawTimestamp;
	out_meta.ntp_raw_unskewed_timestamp = inFrame->ntpRawUnskewedTimestamp;
	out_meta.play_timestamp = inFrame->playTimestamp;
	out_meta.capture_timestamp = 0;
	out_meta.local_timestamp = 0;

	*ntpDelta = (inFrame->ntpTimestamp == 0) ? 1 : 0;
	if (inFrame->ntpTimestamp != 0)
		out_meta.ntp_timestamp--;

	*ntpUnskewedDelta = (inFrame->ntpTimestamp == 0) ? 1 : 0;
	if (inFrame->ntpTimestamp != 0)
		out_meta.ntp_unskewed_timestamp--;

	*ntpRawDelta = (inFrame->ntpTimestamp == 0) ? 1 : 0;
	if (inFrame->ntpTimestamp != 0)
		out_meta.ntp_raw_timestamp--;

	*ntpRawUnskewedDelta = (inFrame->ntpTimestamp == 0) ? 1 : 0;
	if (inFrame->ntpTimestamp != 0)
		out_meta.ntp_raw_unskewed_timestamp--;

	*playDelta = (inFrame->ntpTimestamp == 0) ? 1 : 0;
	if (inFrame->ntpTimestamp != 0)
		out_meta.play_timestamp--;

	/* Start NALU */
	ret = h264_ctx_clear_nalu(ctx);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("h264_ctx_clear_nalu", -ret);
		goto out;
	}

	/* Setup NALU header */
	nh.nal_ref_idc = 3;
	nh.nal_unit_type = H264_NALU_TYPE_SLICE_IDR;
	ret = h264_ctx_set_nalu_header(ctx, &nh);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("h264_ctx_set_nalu_header", -ret);
		goto out;
	}

	/* Setup slice header */
	sh = (struct h264_slice_header *)calloc(1, sizeof(*sh));
	if (sh == nullptr) {
		ret = -ENOMEM;
		PDRAW_LOG_ERRNO("calloc", -ret);
		goto out;
	}
	sh->first_mb_in_slice = 0;
	sh->slice_type = H264_SLICE_TYPE_I;
	sh->frame_num = 0;
	sh->pic_order_cnt_lsb = 0;
	sh->redundant_pic_cnt = 0;
	sh->direct_spatial_mv_pred_flag = 0;
	sh->slice_qp_delta = 0;
	sh->disable_deblocking_filter_idc = 2;
	sh->slice_alpha_c0_offset_div2 = 0;
	sh->slice_beta_offset_div2 = 0;
	ret = h264_ctx_set_slice_header(ctx, sh);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("h264_ctx_set_slice_header", -ret);
		goto out;
	}

	/* Setup bitstream */
	ret = mbuf_mem_get_data(idr_mem, (void **)&data, &len);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_mem_get_data", -ret);
		goto out;
	}

	if (len <= 4) {
		ret = -ENOBUFS;
		PDRAW_LOG_ERRNO("", -ret);
		goto out;
	}

	h264_bs_init(&bs, data + 4, len - 4, 1);

	/* Write slice */
	mbTotal = (mInputMedia->info.resolution.width + 15) / 16 *
		  (mInputMedia->info.resolution.height + 15) / 16;

	ret = h264_write_grey_i_slice(&bs, ctx, mbTotal);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("h264_write_grey_i_slice", -ret);
		goto out;
	}

	switch (inInfo->format.data_format) {
	case VDEF_CODED_DATA_FORMAT_AVCC:
		sc = htonl(bs.off);
		memcpy(data, &sc, sizeof(sc));
		break;
	case VDEF_CODED_DATA_FORMAT_BYTE_STREAM:
		sc = htonl(0x00000001);
		memcpy(data, &sc, sizeof(sc));
		break;
	default:
		ret = -ENOSYS;
		PDRAW_LOG_ERRNO("unsupported data format", -ret);
		goto out;
	}

	ret = mbuf_coded_video_frame_new(&idr_info, &idr_frame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_new", -ret);
		goto out;
	}
	nalu.size = bs.off + 4;
	nalu.h264.type = H264_NALU_TYPE_SLICE_IDR;
	nalu.h264.slice_type = H264_SLICE_TYPE_I;
	nalu.h264.slice_mb_count = mbTotal;
	ret = mbuf_coded_video_frame_add_nalu(idr_frame, idr_mem, 0, &nalu);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_add_nalu", -ret);
		goto out;
	}
	ret = mbuf_coded_video_frame_finalize(idr_frame);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_finalize", -ret);
		goto out;
	}
	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		idr_frame,
		PDRAW_ANCILLARY_DATA_KEY_VIDEOFRAME,
		&out_meta,
		sizeof(out_meta));
	if (ret < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
				-ret);
		goto out;
	}

	Sink::onCodedVideoChannelQueue(channel, idr_frame);
	mIsFlushed = false;
out:
	free(sh);
	if (idr_mem)
		mbuf_mem_unref(idr_mem);
	if (idr_frame)
		mbuf_coded_video_frame_unref(idr_frame);
	return ret;
}


void ExternalCodedVideoSink::onCodedVideoChannelQueue(
	CodedVideoChannel *channel,
	struct mbuf_coded_video_frame *frame)
{
	int ret;
	uint64_t ntpDelta = 0, ntpUnskewedDelta = 0, ntpRawDelta = 0,
		 ntpRawUnskewedDelta = 0, playDelta = 0;
	bool isIdr = false;

	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}
	if (frame == nullptr) {
		PDRAW_LOG_ERRNO("frame", EINVAL);
		return;
	}
	if (mState != STARTED) {
		PDRAW_LOGE("%s: video sink is not started", __func__);
		return;
	}
	if (mInputChannelFlushPending) {
		PDRAW_LOGI("frame input: flush pending, discard frame");
		return;
	}
	Sink::lock();

	if (mInputMedia->format.encoding == VDEF_ENCODING_H264) {
		struct vdef_coded_frame frame_info;

		ret = mbuf_coded_video_frame_get_frame_info(frame, &frame_info);
		if (ret < 0) {
			PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info",
					-ret);
			goto end;
		}

		isIdr = (frame_info.type == VDEF_CODED_FRAME_TYPE_IDR);

		if (isIdr) {
			mNeedSync = false;
		} else if (mNeedSync) {
			struct mbuf_ancillary_data *adata;
			struct CodedVideoMedia::Frame *meta;
			ret = mbuf_coded_video_frame_get_ancillary_data(
				frame,
				PDRAW_ANCILLARY_DATA_KEY_CODEDVIDEOFRAME,
				&adata);
			if (ret < 0) {
				PDRAW_LOG_ERRNO(
					"mbuf_coded_video_frame_get_ancillary_data",
					-ret);
				goto end;
			}
			meta = (struct CodedVideoMedia::Frame *)
				mbuf_ancillary_data_get_buffer(adata, nullptr);
			ret = writeGreyIdr(channel,
					   meta,
					   &frame_info,
					   &ntpDelta,
					   &ntpUnskewedDelta,
					   &ntpRawDelta,
					   &ntpRawUnskewedDelta,
					   &playDelta);
			mbuf_ancillary_data_unref(adata);
			if (ret < 0) {
				PDRAW_LOG_ERRNO("writeGreyIdr", -ret);
				goto end;
			}
			mNeedSync = false;

			meta->ntpTimestamp += ntpDelta;
			meta->ntpUnskewedTimestamp += ntpUnskewedDelta;
			meta->ntpRawTimestamp += ntpRawDelta;
			meta->ntpRawUnskewedTimestamp += ntpRawUnskewedDelta;
			meta->playTimestamp += playDelta;
		}
	}
	ret = prepareCodedVideoFrame(channel, frame);

end:
	if (ret < 0) {
		Sink::unlock();
		return;
	}

	Sink::onCodedVideoChannelQueue(channel, frame);
	mIsFlushed = false;
	Sink::unlock();
}


void ExternalCodedVideoSink::onChannelFlush(Channel *channel)
{
	int ret;

	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	PDRAW_LOGD("flushing input channel");
	mInputChannelFlushPending = true;

	ret = flush();
	if (ret < 0)
		PDRAW_LOG_ERRNO("flush", -ret);
}


void ExternalCodedVideoSink::onChannelTeardown(Channel *channel)
{
	CodedVideoChannel *c = dynamic_cast<CodedVideoChannel *>(channel);
	if (c == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	PDRAW_LOGD("tearing down input channel");

	int ret = channelTeardown(c);
	if (ret < 0)
		PDRAW_LOG_ERRNO("channelTeardown", -ret);
}


void ExternalCodedVideoSink::onChannelSessionMetaUpdate(Channel *channel)
{
	struct vmeta_session tmpSessionMeta;
	Sink::onChannelSessionMetaUpdate(channel);

	if (channel == nullptr) {
		PDRAW_LOG_ERRNO("channel", EINVAL);
		return;
	}

	Sink::lock();
	if (mInputMedia == nullptr) {
		Sink::unlock();
		PDRAW_LOGE("%s: input media not found", __func__);
		return;
	}
	tmpSessionMeta = mInputMedia->sessionMeta;
	Sink::unlock();

	if (mVideoSinkListener != nullptr) {
		mVideoSinkListener->onCodedVideoSinkSessionMetaUpdate(
			mSession, getVideoSink(), &tmpSessionMeta);
	}
}


int ExternalCodedVideoSink::channelTeardown(CodedVideoChannel *channel)
{
	int ret;

	if (channel == nullptr)
		return -EINVAL;

	Sink::lock();

	if (mInputMedia == nullptr) {
		/* The channel is already torn down, nothing more to do */
		Sink::unlock();
		return 0;
	}

	if (mTearingDown) {
		/* The teardown may already be in progress but mInputMedia
		 * is not yet set to nullptr.
		 * Eg. removeInputMedia() utimately calls the app's
		 * mediaRemoved() callback, which can call the VideoSink
		 * stop() function, which calls channelTeardown() again. */
		Sink::unlock();
		return 0;
	}
	mTearingDown = true;

	/* Remove the input port */
	channel->setQueue(this, nullptr);

	ret = removeInputMedia(mInputMedia);
	if (ret < 0)
		PDRAW_LOG_ERRNO("removeInputMedia", -ret);
	else
		mInputMedia = nullptr;

	mTearingDown = false;
	Sink::unlock();

	ret = flush();
	if (ret < 0)
		PDRAW_LOG_ERRNO("flush", -ret);

	return ret;
}


/* Listener call from an idle function */
void ExternalCodedVideoSink::callVideoSinkFlush(void *userdata)
{
	ExternalCodedVideoSink *self =
		reinterpret_cast<ExternalCodedVideoSink *>(userdata);
	PDRAW_LOG_ERRNO_RETURN_IF(self == nullptr, EINVAL);

	if (self->mVideoSinkListener == nullptr) {
		self->flushDone();
	} else {
		self->mVideoSinkListener->onCodedVideoSinkFlush(
			self->mSession, self->getVideoSink());
	}
}


CodedVideoSinkWrapper::CodedVideoSinkWrapper(
	Session *session,
	const struct pdraw_video_sink_params *params,
	IPdraw::ICodedVideoSink::Listener *listener)
{
	mElement = mSink = new Pdraw::ExternalCodedVideoSink(
		session,
		&params->required_coded_format,
		session,
		listener,
		this,
		params);
}


CodedVideoSinkWrapper::~CodedVideoSinkWrapper(void)
{
	if (mSink == nullptr)
		return;
	int ret = mSink->stop();
	if (ret < 0)
		ULOG_ERRNO("sink->stop", -ret);
}


int CodedVideoSinkWrapper::resync(void)
{
	if (mSink == nullptr)
		return -EPROTO;
	return mSink->resync();
}


struct mbuf_coded_video_frame_queue *CodedVideoSinkWrapper::getQueue(void)
{
	if (mSink == nullptr)
		return nullptr;
	return mSink->getQueue();
}


int CodedVideoSinkWrapper::queueFlushed(void)
{
	if (mSink == nullptr)
		return -EPROTO;
	return mSink->flushDone();
}

} /* namespace Pdraw */
