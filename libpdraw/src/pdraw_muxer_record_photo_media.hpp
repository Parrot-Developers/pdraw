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

#pragma once

#include "pdraw_muxer_record_media.hpp"
#include "pdraw_muxer_record_photo.hpp"

#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_raw_video_frame.h>

#include <vector>
#include <video-metadata/vmeta_photo.h>

namespace Pdraw {


class PhotoRecordMuxer::PhotoMuxerMedia : public RecordMuxer::MuxerMedia {
public:
	PhotoMuxerMedia(PhotoRecordMuxer *muxer, const MuxerMediaConfig &cfg);

	~PhotoMuxerMedia() override = default;

	void updateMetadata(const struct vmeta_frame *meta) const;

	int writeMetadata(struct vmeta_session &session,
			  struct vmeta_frame *frame,
			  const char *mimeType = nullptr);

	static void
	photoMetaWriteFileCb(enum pmeta_defs_dest dest,
			     const struct pmeta_defs_exif_def *exifDef,
			     const struct pmeta_defs_xmp_def *xmpDef,
			     const char *value,
			     void *userdata);

protected:
	static void appendIovec(std::vector<struct iovec> &iov,
				const uint8_t *buf,
				size_t len);

	virtual int internalAddExif(const struct pmeta_defs_exif_def *exifDef,
				    const char *value)
	{
		return -ENOSYS;
	}

	virtual int internalAddXmp(const struct pmeta_defs_xmp_def *xmpDef,
				   const char *value)
	{
		return -ENOSYS;
	}

	virtual void internalClearMetadata()
	{
		/* Intentional no-op. */
	}

	virtual int internalSerialize(const uint8_t *buf,
				      size_t len,
				      std::vector<struct iovec> &iov,
				      uint8_t **headerBuf) = 0;

	virtual const char *internalGetMimeType() const = 0;

	PhotoRecordMuxer *mPhotoMuxer = nullptr;
};

class PhotoRecordMuxer::PhotoMuxerCodedVideoMedia
		: public PhotoRecordMuxer::PhotoMuxerMedia {
public:
	using PhotoMuxerMedia::PhotoMuxerMedia;
	int process() override;

protected:
	virtual int processFrame(struct mbuf_coded_video_frame *frame);
};


class PhotoRecordMuxer::PhotoMuxerRawVideoMedia
		: public PhotoRecordMuxer::PhotoMuxerMedia {
public:
	using PhotoMuxerMedia::PhotoMuxerMedia;
	int process() override;

protected:
	virtual int processFrame(struct mbuf_raw_video_frame *frame);

	virtual int internalSetupFormat(const struct vdef_raw_frame *info)
	{
		return 0;
	}
};

} /* namespace Pdraw */
