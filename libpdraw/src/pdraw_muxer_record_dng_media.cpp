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

#define ULOG_TAG pdraw_recmux_dng_media
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "pdraw_muxer_record_dng.hpp"
#include "pdraw_muxer_record_dng_media.hpp"

#ifdef BUILD_LIBDNG_PARROT

#	include <media-buffers/mbuf_raw_video_frame.h>

namespace Pdraw {


#	define PDRAW_CHECK_MUXER_WRITER_THREAD(expectWriter)                  \
		mDngMuxer->logThreadCheckWarning(__func__, expectWriter)


void DngRecordMuxer::DngMuxerMedia::photoMetaWriteFileCb(
	enum pmeta_defs_dest dest,
	const struct pmeta_defs_exif_def *exifDef,
	const struct pmeta_defs_xmp_def *xmpDef,
	const char *value,
	void *userdata)
{
	auto *mux = static_cast<struct dng_mux *>(userdata);

	ULOG_ERRNO_RETURN_IF(mux == nullptr, EINVAL);
	ULOG_ERRNO_RETURN_IF(value == nullptr, EINVAL);

	int res = -EINVAL;
	const char *errKey = "unknown";

	switch (dest) {
	case PMETA_DEFS_DEST_EXIF:
		ULOG_ERRNO_RETURN_IF(exifDef == nullptr, EINVAL);
		res = dng_mux_add_exif(mux, exifDef, value);
		errKey = exifDef->tag_name;
		break;
	case PMETA_DEFS_DEST_XMP:
		ULOG_ERRNO_RETURN_IF(xmpDef == nullptr, EINVAL);
		res = dng_mux_add_xmp(mux, xmpDef, value);
		errKey = xmpDef->full_key;
		break;
	default:
		ULOGE("unsupported metadata destination: %d", dest);
		return;
	}

	if (res < 0)
		ULOG_ERRNO("dng_mux insertion failed for '%s'", -res, errKey);
}


DngRecordMuxer::DngMuxerMedia::DngMuxerMedia(DngRecordMuxer *muxer,
					     const MuxerMediaConfig &cfg) :
		PhotoRecordMuxer::PhotoMuxerRawVideoMedia(muxer, cfg),
		mDngMuxer(muxer)
{
	std::string name = muxer->getName() + "#DngMuxerMedia";
	Loggable::setName(name);
}


/* Called on the writer thread */
int DngRecordMuxer::DngMuxerMedia::setup(
	const struct pdraw_media_info *mediaInfo,
	const struct pdraw_muxer_media_params *params)
{
	PDRAW_CHECK_MUXER_WRITER_THREAD(true);

	return 0;
}


/* Called on the writer thread */
int DngRecordMuxer::DngMuxerMedia::processFrame(
	struct mbuf_raw_video_frame *frame)
{
	int res = 0;
	const void *buf = nullptr;
	size_t len;
	struct vdef_raw_frame info = {};
	struct vmeta_frame *metadata = nullptr;

	PDRAW_CHECK_MUXER_WRITER_THREAD(true);

	if (mDngMuxer->mDngMux == nullptr)
		return -EPROTO;

	res = mbuf_raw_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -res);
		return res;
	}

	struct dng_mux_format dngFormat {
		.width = info.info.resolution.width,
		.height = info.info.resolution.height,
		.stride = static_cast<unsigned int>(info.plane_stride[0]),
		.bayer_phase = mapVdefToDngPhase(info.format.pix_order),
		.bit_depth = info.format.pix_size,
	};

	res = dng_mux_set_format(mDngMuxer->mDngMux, &dngFormat);
	if (res < 0) {
		PDRAW_LOG_ERRNO("dng_mux_set_format", -res);
		return res;
	}

	dng_mux_clear_metadata(mDngMuxer->mDngMux);

	res = vmeta_session_photo_write(
		&mSessionMeta,
		&DngRecordMuxer::DngMuxerMedia::photoMetaWriteFileCb,
		mDngMuxer->mDngMux);
	if (res < 0) {
		PDRAW_LOG_ERRNO("vmeta_session_photo_write", -res);
	}

	res = mbuf_raw_video_frame_get_packed_buffer(frame, &buf, &len);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_packed_buffer", -res);
		return res;
	}

	res = mbuf_raw_video_frame_get_metadata(frame, &metadata);
	if (res == 0 && metadata != nullptr) {
		updateMetadata(metadata);
		res = vmeta_frame_photo_write(
			metadata,
			&DngRecordMuxer::DngMuxerMedia::photoMetaWriteFileCb,
			mDngMuxer->mDngMux);
		if (res < 0) {
			PDRAW_LOG_ERRNO("vmeta_frame_photo_write", -res);
		}
	} else if (res < 0 && res != -ENOENT) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_metadata", -res);
	}

	struct iovec iov[1];
	int iovcnt = 0;
	uint8_t *headerBuf = nullptr;

	res = dng_mux_serialize_iov(mDngMuxer->mDngMux,
				    (const uint8_t *)buf,
				    len,
				    iov,
				    &iovcnt,
				    &headerBuf);
	if (res < 0) {
		PDRAW_LOG_ERRNO("dng_mux_serialize", -res);
	} else {
		res = mDngMuxer->saveToDiskIov(
			iov, iovcnt, mDngMuxer->mStats.record.raw_video_frames);

		if (headerBuf)
			free(headerBuf);
	}

	if (metadata)
		vmeta_frame_unref(metadata);
	if (buf)
		mbuf_raw_video_frame_release_packed_buffer(frame, buf);

	return res;
}

} /* namespace Pdraw */

#endif /* BUILD_LIBDNG_PARROT */
