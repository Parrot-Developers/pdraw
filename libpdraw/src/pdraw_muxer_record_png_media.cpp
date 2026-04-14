/**
 * Parrot Drones Audio and Video Vector library
 * Record muxer
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

#define ULOG_TAG pdraw_recmux_png_media
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_muxer_record_png.hpp"
#include "pdraw_muxer_record_png_media.hpp"

#include <media-buffers/mbuf_coded_video_frame.h>

namespace Pdraw {


#define PDRAW_CHECK_MUXER_WRITER_THREAD(expectWriter)                          \
	mPngMuxer->logThreadCheckWarning(__func__, expectWriter)


PngRecordMuxer::PngMuxerMedia::PngMuxerMedia(PngRecordMuxer *muxer,
					     const MuxerMediaConfig &cfg) :
		PhotoRecordMuxer::PhotoMuxerCodedVideoMedia(muxer, cfg),
		mPngMuxer(muxer)
{
	std::string name = muxer->getName() + "#PngMuxerMedia";
	Loggable::setName(name);
}


/* Called on the writer thread */
int PngRecordMuxer::PngMuxerMedia::setup(
	const struct pdraw_media_info *mediaInfo,
	const struct pdraw_muxer_media_params *params)
{
	PDRAW_CHECK_MUXER_WRITER_THREAD(true);

	return 0;
}


/* Called on the writer thread */
int PngRecordMuxer::PngMuxerMedia::processFrame(
	struct mbuf_coded_video_frame *frame)
{
	int res = 0;
	const void *buf = nullptr;
	size_t len;
	struct vdef_coded_frame info = {};
	int naluCount;

	PDRAW_CHECK_MUXER_WRITER_THREAD(true);

	naluCount = mbuf_coded_video_frame_get_nalu_count(frame);
	if (naluCount <= 0) {
		PDRAW_LOGE("Invalid NALU count: %d", naluCount);
		return -EINVAL;
	}

	res = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		return res;
	}

	res = mbuf_coded_video_frame_get_packed_buffer(frame, &buf, &len);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_packed_buffer",
				-res);
		return res;
	}

	struct iovec iov[1];
	iov[0].iov_base = const_cast<void *>(buf);
	iov[0].iov_len = len;
	int iovcnt = 1;

	res = mPngMuxer->saveToDiskIov(
		iov, iovcnt, mPngMuxer->mStats.record.coded_video_frames);

	if (buf)
		mbuf_coded_video_frame_release_packed_buffer(frame, buf);

	return res;
}

} /* namespace Pdraw */
