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

#define ULOG_TAG pdraw_recmux_jfif
#include <ulog.h>

#include "pdraw_muxer_record_jfif.hpp"
#include "pdraw_muxer_record_jfif_media.hpp"

#ifdef BUILD_LIBJFIF
#	include <array>
#endif

ULOG_DECLARE_TAG(ULOG_TAG);

#ifdef BUILD_LIBJFIF

namespace Pdraw {


static const std::array<vdef_coded_format, 1> &getSupportedCodedFormats()
{
	static const std::array<vdef_coded_format, 1> formats = {{
		vdef_jpeg_jfif,
	}};
	return formats;
}


JfifRecordMuxer::JfifRecordMuxer(Session *session,
				 Element::Listener *elementListener,
				 IPdraw::IMuxer::Listener *listener,
				 MuxerWrapper *wrapper,
				 const std::string &fileName,
				 const struct pdraw_muxer_params *params) :
		PhotoRecordMuxer(session,
				 elementListener,
				 listener,
				 wrapper,
				 fileName,
				 params)
{
	Element::setClassName(__func__);

	setCodedVideoMediaFormatCaps(
		getSupportedCodedFormats().data(),
		static_cast<int>(getSupportedCodedFormats().size()));
}


/* Called on the writer thread */
std::unique_ptr<RecordMuxer::MuxerMedia>
JfifRecordMuxer::JfifRecordMuxer::createMedia(const MuxerMediaConfig &cfg)
{
	try {
		switch (cfg.type) {
		case Media::Type::CODED_VIDEO:
			return std::make_unique<JfifMuxerMedia>(this, cfg);
		default:
			PDRAW_LOGE("unsupported media type: %d",
				   static_cast<int>(cfg.type));
			return nullptr;
		}
	} catch (const std::bad_alloc &) {
		PDRAW_LOGE("output media allocation failed");
		return nullptr;
	}
}


/* Called on the writer thread */
int JfifRecordMuxer::internalSetThumbnail(enum pdraw_muxer_thumbnail_type type,
					  const uint8_t *data,
					  size_t size)
{
	PDRAW_CHECK_WRITER_THREAD(true);

	int ret = jfif_mux_set_thumbnail(mJfifMux, data, size);
	if (ret < 0) {
		PDRAW_LOG_ERRNO("jfif_mux_set_thumbnail", -ret);
		return ret;
	}

	return 0;
}


/* Called on the writer thread */
void JfifRecordMuxer::onInternalStopThread()
{
	PhotoRecordMuxer::onInternalStopThread();
	if (mJfifMux) {
		jfif_mux_close(mJfifMux);
		mJfifMux = nullptr;
	}
}


/* Called on the writer thread */
int JfifRecordMuxer::onBeforeAddMuxerMedias()
{
	struct jfif_mux_config config = {};
	config.filemode = mFileMode;
	snprintf(config.xmptk, sizeof(config.xmptk), "Pdraw_JfifRecordMuxer");
	config.flags = 0;

	int ret = jfif_mux_open(&config, &mJfifMux);
	if (ret < 0) {
		ULOG_ERRNO("jfif_mux_open", -ret);
		return ret;
	}

	return PhotoRecordMuxer::onBeforeAddMuxerMedias();
}

} /* namespace Pdraw */

#endif /* BUILD_LIBJFIF */
