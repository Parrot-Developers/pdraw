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

#include "pdraw_muxer_record.hpp"

namespace Pdraw {

class PhotoRecordMuxer : public RecordMuxer {
public:
	PhotoRecordMuxer(Session *session,
			 Element::Listener *elementListener,
			 IPdraw::IMuxer::Listener *listener,
			 MuxerWrapper *wrapper,
			 const std::string &fileName,
			 const struct pdraw_muxer_params *params);

	~PhotoRecordMuxer() override = default;

	int getDynParams(struct pdraw_muxer_dyn_params *dyn_params) override;

	int generateFileName(std::string &fileName) const;

	void notifyMediaReadyIov(const std::string &fileName,
				 const struct iovec *iov,
				 int iovcnt);

	void notifyMediaSaved(const std::string &fileName);

	int writeToFileIov(const std::string &fileName,
			   const struct iovec *iov,
			   int iovcnt) const;

	int saveToDiskIov(const struct iovec *iov,
			  int iovcnt,
			  uint32_t &frame_counter);

protected:
	class PhotoMuxerMedia;
	class PhotoMuxerCodedVideoMedia;
	class PhotoMuxerRawVideoMedia;

	std::string getDefaultMediaName(Media::Type type) override;

	int onBeforeMediaCreation(
		MuxerMediaConfig &cfg,
		const struct pdraw_media_info *mediaInfo,
		const struct pdraw_muxer_media_params *params) override;

	int onAfterMediaCreation(MuxerMedia *track,
				 const MuxerMediaConfig &cfg) override;

	int internalSetThumbnail(enum pdraw_muxer_thumbnail_type type,
				 const uint8_t *data,
				 size_t size) override;

	int internalSetFileMetadata(
		const struct pdraw_muxer_metadata_params *params,
		const uint8_t *data,
		size_t size) override;

	void onInternalStopThread() override;

	int onWriterLoopInit() override;

	int onWriterLoopCleanup() override;

	const char *getThreadName() const override
	{
		static constexpr char name[] = "pdraw_recmx_pho";
		static_assert(sizeof(name) <= 16,
			      "Thread name is too long for pthread_setname_np");
		return name;
	}

	int onBeforeAddMuxerMedias() override;

	int internalSetDynParams(
		const struct pdraw_muxer_dyn_params *dyn_params) override;

	std::string mFileNamePattern{};
	uint32_t mNextFileIndex = 0;
};

} /* namespace Pdraw */
