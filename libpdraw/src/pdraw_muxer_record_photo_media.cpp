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

#define ULOG_TAG pdraw_recmux_photo_media
#include <ulog.h>

#include "pdraw_muxer_record_photo_media.hpp"

#include <futils/futils.h>
#include <memory>
#include <time.h>
#include <video-metadata/vmeta_photo.h>

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {

#define XMP_(_name) PMETA_DEFS_XMP_IDX_##_name

PhotoRecordMuxer::PhotoMuxerMedia::PhotoMuxerMedia(
	PhotoRecordMuxer *muxer,
	const MuxerMediaConfig &cfg) :
		RecordMuxer::MuxerMedia(muxer, cfg),
		mPhotoMuxer(muxer)
{
}


void PhotoRecordMuxer::PhotoMuxerMedia::appendIovec(
	std::vector<struct iovec> &iov,
	const uint8_t *buf,
	size_t len)
{
	struct iovec i;
	i.iov_base = const_cast<uint8_t *>(buf);
	i.iov_len = len;
	iov.push_back(i);
}


void PhotoRecordMuxer::PhotoMuxerMedia::updateMetadata(
	const struct vmeta_frame *meta) const
{
	if (!meta)
		return;
}


void PhotoRecordMuxer::PhotoMuxerMedia::photoMetaWriteFileCb(
	enum pmeta_defs_dest dest,
	const struct pmeta_defs_exif_def *exifDef,
	const struct pmeta_defs_xmp_def *xmpDef,
	const char *value,
	void *userdata)
{
	auto *media = static_cast<PhotoMuxerMedia *>(userdata);

	ULOG_ERRNO_RETURN_IF(media == nullptr, EINVAL);

	int res = -EINVAL;
	const char *errKey;

	switch (dest) {
	case PMETA_DEFS_DEST_EXIF:
		ULOG_ERRNO_RETURN_IF(exifDef == nullptr, EINVAL);
		res = media->internalAddExif(exifDef, value);
		errKey = exifDef->tag_name;
		break;
	case PMETA_DEFS_DEST_XMP:
		ULOG_ERRNO_RETURN_IF(xmpDef == nullptr, EINVAL);
		res = media->internalAddXmp(xmpDef, value);
		errKey = xmpDef->full_key;
		break;
	default:
		ULOGE("unsupported metadata destination: %d", dest);
		return;
	}

	if (res < 0 && res != -ENOSYS)
		ULOG_ERRNO("metadata insertion failed for '%s'", -res, errKey);
}


int PhotoRecordMuxer::PhotoMuxerMedia::writeMetadata(
	struct vmeta_session &session,
	struct vmeta_frame *frame,
	const char *mimeType)
{
	int res;

	res = mPhotoMuxer->updateMediaDate();
	if (res < 0)
		PDRAW_LOG_ERRNO("updateMediaDate", -res);

	internalClearMetadata();

	mPhotoMuxer->sessionSetMediaDate(session, true);

	if (session.title[0] == '\0') {
		int err = time_local_format(session.media_date,
					    session.media_date_gmtoff,
					    TIME_FMT_RFC1123,
					    session.title,
					    sizeof(session.title));
		if (err < 0)
			PDRAW_LOG_ERRNO("time_local_format", -err);
	}

	if (frame != nullptr)
		updateMetadata(frame);

	res = vmeta_photo_write(
		&session,
		frame,
		&PhotoRecordMuxer::PhotoMuxerMedia::photoMetaWriteFileCb,
		this);
	if (res < 0) {
		PDRAW_LOG_ERRNO("vmeta_photo_write", -res);
	}

	if (mimeType != nullptr) {
		enum pmeta_defs_xmp_idx idx = XMP_(DC_FORMAT);
		const struct pmeta_defs_xmp_def *def =
			pmeta_defs_get_xmp_tag_by_idx(idx);
		if (def != nullptr) {
			res = internalAddXmp(def, mimeType);
			if (res < 0 && res != -ENOSYS)
				PDRAW_LOG_ERRNO("internalAddXmp", -res);
		}
	}

	return 0;
}


/* Called on the writer thread */
int PhotoRecordMuxer::PhotoMuxerCodedVideoMedia::process()
{
	struct mbuf_coded_video_frame *frame = nullptr;
	int res = mQueue->popFrame(&frame);
	if (res < 0)
		return res;

	res = processFrame(frame);

	mbuf_coded_video_frame_unref(frame);
	return res;
}


int PhotoRecordMuxer::PhotoMuxerCodedVideoMedia::processFrame(
	struct mbuf_coded_video_frame *frame)
{
	int res = 0;
	const void *buf = nullptr;
	size_t len;
	struct vdef_coded_frame info = {};
	struct vmeta_frame *metadata = nullptr;
	struct vmeta_session sessionMeta = {};
	std::vector<struct iovec> iov;
	unique_c_ptr<uint8_t> headerBuf;

	res = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		return res;
	}

	sessionMeta = mSessionMeta;
	res = mbuf_coded_video_frame_get_metadata(frame, &metadata);
	if (res < 0 && res != -ENOENT)
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_metadata", -res);

	writeMetadata(sessionMeta, metadata, internalGetMimeType());

	res = mbuf_coded_video_frame_get_packed_buffer(frame, &buf, &len);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_coded_video_frame_get_packed_buffer",
				-res);
		goto out;
	}

	{
		uint8_t *rawBuf = nullptr;
		res = internalSerialize(
			static_cast<const uint8_t *>(buf), len, iov, &rawBuf);
		if (res < 0) {
			PDRAW_LOG_ERRNO("internalSerialize", -res);
		} else {
			headerBuf.reset(rawBuf);
			res = mPhotoMuxer->saveToDiskIov(
				iov.data(),
				static_cast<int>(iov.size()),
				mPhotoMuxer->mStats.record.coded_video_frames);
		}
	}


out:
	if (metadata != nullptr)
		vmeta_frame_unref(metadata);
	if (buf != nullptr)
		mbuf_coded_video_frame_release_packed_buffer(frame, buf);

	return res;
}


/* Called on the writer thread */
int PhotoRecordMuxer::PhotoMuxerRawVideoMedia::process()
{
	struct mbuf_raw_video_frame *frame = nullptr;
	int res = mQueue->popFrame(&frame);
	if (res < 0)
		return res;

	res = processFrame(frame);

	mbuf_raw_video_frame_unref(frame);
	return res;
}


int PhotoRecordMuxer::PhotoMuxerRawVideoMedia::processFrame(
	struct mbuf_raw_video_frame *frame)
{
	int res = 0;
	const void *buf = nullptr;
	size_t len;
	struct vdef_raw_frame info = {};
	struct vmeta_frame *metadata = nullptr;
	struct vmeta_session sessionMeta = {};
	std::vector<struct iovec> iov;
	unique_c_ptr<uint8_t> headerBuf;

	res = mbuf_raw_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -res);
		return res;
	}

	res = internalSetupFormat(&info);
	if (res < 0) {
		PDRAW_LOG_ERRNO("internalSetupFormat", -res);
		return res;
	}

	sessionMeta = mSessionMeta;
	res = mbuf_raw_video_frame_get_metadata(frame, &metadata);
	if (res < 0 && res != -ENOENT)
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_metadata", -res);

	writeMetadata(sessionMeta, metadata, internalGetMimeType());

	res = mbuf_raw_video_frame_get_packed_buffer(frame, &buf, &len);
	if (res < 0) {
		PDRAW_LOG_ERRNO("mbuf_raw_video_frame_get_packed_buffer", -res);
		goto out;
	}

	{
		uint8_t *rawBuf = nullptr;
		res = internalSerialize(
			static_cast<const uint8_t *>(buf), len, iov, &rawBuf);
		if (res < 0) {
			PDRAW_LOG_ERRNO("internalSerialize", -res);
		} else {
			headerBuf.reset(rawBuf);
			res = mPhotoMuxer->saveToDiskIov(
				iov.data(),
				static_cast<int>(iov.size()),
				mPhotoMuxer->mStats.record.raw_video_frames);
		}
	}


out:
	if (metadata != nullptr)
		vmeta_frame_unref(metadata);
	if (buf != nullptr)
		mbuf_raw_video_frame_release_packed_buffer(frame, buf);

	return res;
}

} /* namespace Pdraw */
