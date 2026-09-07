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

#define ULOG_TAG pdraw_recmux_jfif_media
#include <ulog.h>

#include "pdraw_muxer_record_jfif.hpp"
#include "pdraw_muxer_record_jfif_media.hpp"

#ifdef BUILD_LIBJFIF
#	include <time.h>
#	include <media-buffers/mbuf_coded_video_frame.h>
#endif

ULOG_DECLARE_TAG(ULOG_TAG);

#ifdef BUILD_LIBJFIF


namespace Pdraw {


#	define PDRAW_CHECK_MUXER_WRITER_THREAD(expectWriter)                  \
		mJfifMuxer->logThreadCheckWarning(__func__, expectWriter)


JfifRecordMuxer::JfifMuxerMedia::JfifMuxerMedia(JfifRecordMuxer *muxer,
						const MuxerMediaConfig &cfg) :
		PhotoRecordMuxer::PhotoMuxerCodedVideoMedia(muxer, cfg),
		mJfifMuxer(muxer)
{
	std::string name = muxer->getName() + "#JfifMuxerMedia";
	Loggable::setName(name);
}


/* Called on the writer thread */
int JfifRecordMuxer::JfifMuxerMedia::setup(
	const struct pdraw_media_info *mediaInfo,
	const struct pdraw_muxer_media_params *params)
{
	PDRAW_CHECK_MUXER_WRITER_THREAD(true);

	return 0;
}


int JfifRecordMuxer::JfifMuxerMedia::internalAddExif(
	const struct pmeta_defs_exif_def *exifDef,
	const char *value)
{
	return jfif_mux_add_exif(mJfifMuxer->mJfifMux, exifDef, value);
}


int JfifRecordMuxer::JfifMuxerMedia::internalAddXmp(
	const struct pmeta_defs_xmp_def *xmpDef,
	const char *value)
{
	return jfif_mux_add_xmp(mJfifMuxer->mJfifMux, xmpDef, value);
}


void JfifRecordMuxer::JfifMuxerMedia::internalClearMetadata()
{
	jfif_mux_clear_metadata(mJfifMuxer->mJfifMux);
}


int JfifRecordMuxer::JfifMuxerMedia::internalSerialize(
	const uint8_t *buf,
	size_t len,
	std::vector<struct iovec> &iov,
	uint8_t **headerBuf)
{
	int iovcnt = 0;
	iov.resize(3);
	int res = jfif_mux_serialize_iov(
		mJfifMuxer->mJfifMux, buf, len, iov.data(), &iovcnt, headerBuf);
	if (res >= 0)
		iov.resize(iovcnt);
	return res;
}


} /* namespace Pdraw */


#endif /* BUILD_LIBJFIF */
