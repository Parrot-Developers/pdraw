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

#define ULOG_TAG pdraw_recmux_photo
#include <ulog.h>

#include "pdraw_muxer_record_photo.hpp"
#include "pdraw_session.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <futils/futils.h>

ULOG_DECLARE_TAG(ULOG_TAG);

namespace Pdraw {


PhotoRecordMuxer::PhotoRecordMuxer(Session *session,
				   Element::Listener *elementListener,
				   IPdraw::IMuxer::Listener *listener,
				   MuxerWrapper *wrapper,
				   const std::string &fileName,
				   const struct pdraw_muxer_params *params) :
		RecordMuxer(session,
			    elementListener,
			    listener,
			    wrapper,
			    fileName,
			    params),
		mFileNamePattern(fileName),
		mNextFileIndex(params->initial_file_index)
{
}


/* Called on the writer thread */
int PhotoRecordMuxer::generateFileName(std::string &fileName) const
{
	/* Compute size */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
	int size =
		snprintf(nullptr, 0, mFileNamePattern.c_str(), mNextFileIndex);
#pragma GCC diagnostic pop

	if (size < 0) {
		int err = errno;
		PDRAW_LOGE("snprintf failed to calculate size");
		return -err;
	}

	fileName.resize(size);

	/* Write buf to string */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
	snprintf(&fileName[0],
		 size + 1,
		 mFileNamePattern.c_str(),
		 mNextFileIndex);
#pragma GCC diagnostic pop

	return 0;
}


/* Called on the writer thread */
void PhotoRecordMuxer::notifyMediaReadyIov(const std::string &fileName,
					   const struct iovec *iov,
					   int iovcnt)
{
	mNextFileIndex++;
	if (mMuxerListener != nullptr) {
		mMuxerListener->onMuxerMediaReady(
			mSession, mMuxer, fileName.c_str(), iov, iovcnt);
	}
}


/* Called on the writer thread */
void PhotoRecordMuxer::notifyMediaSaved(const std::string &fileName)
{
	if (mMuxerListener != nullptr) {
		mMuxerListener->onMuxerMediaSaved(
			mSession, mMuxer, fileName.c_str());
	}
}


#ifdef _WIN32
static ssize_t writev(int fd, const struct iovec *iov, int iovcnt)
{
	ssize_t total = 0;
	ssize_t ret;
	for (int i = 0; i < iovcnt; i++) {
		ret = write(fd,
			    iov[i].iov_base,
			    static_cast<unsigned int>(iov[i].iov_len));
		if (ret < 0)
			return total == 0 ? ret : total;
		total += ret;
		if ((size_t)ret != iov[i].iov_len)
			return total;
	}
	return total;
}
#endif


/* Called on the writer thread */
int PhotoRecordMuxer::writeToFileIov(const std::string &fileName,
				     const struct iovec *iov,
				     int iovcnt) const
{
	/* 0 means "use the default" (documented on filemode), matching
	 * mp4_mux_open()'s own fallback. */
	mode_t mode = mFileMode ? mFileMode : (S_IRUSR | S_IWUSR);
	int fd = open(fileName.c_str(), O_WRONLY | O_CREAT | O_TRUNC, mode);
	if (fd < 0) {
		return -errno;
	}

	ssize_t written = writev(fd, iov, iovcnt);
	int err = 0;

	if (written < 0) {
		err = -errno;
	}

	close(fd);
	return err;
}


/* Called on the writer thread */
int PhotoRecordMuxer::saveToDiskIov(const struct iovec *iov,
				    int iovcnt,
				    uint32_t &frame_counter)
{
	std::string fileName;
	int res = generateFileName(fileName);
	if (res < 0) {
		PDRAW_LOG_ERRNO("generateFileName", -res);
		return res;
	}

	size_t size = 0;
	for (int i = 0; i < iovcnt; i++)
		size += iov[i].iov_len;

	res = ensureFreeSpace(size);
	if (res < 0) {
		if (res != -ENOSPC)
			PDRAW_LOG_ERRNO("ensureFreeSpace", -res);
		onUnrecoverableError(res);
		return res;
	}

	frame_counter++;
	notifyMediaReadyIov(fileName, iov, iovcnt);

	res = writeToFileIov(fileName, iov, iovcnt);
	if (res < 0)
		PDRAW_LOG_ERRNO("writeToFileIov", -res);
	else
		notifyMediaSaved(fileName);

	return res;
}


/* Called on the loop thread */
int PhotoRecordMuxer::getDynParams(struct pdraw_muxer_dyn_params *dyn_params)
{
	PDRAW_LOG_ERRNO_RETURN_ERR_IF(dyn_params == nullptr, EINVAL);

	PDRAW_CHECK_WRITER_THREAD(false);

	/* TODO */

	return 0;
}


/* Called on the writer thread */
std::string PhotoRecordMuxer::getDefaultMediaName(Media::Type type)
{
	switch (type) {
	case Media::Type::CODED_VIDEO:
		return "DefaultVideo";
	default:
		return "Track";
	}
}


/* Called on the writer thread */
int PhotoRecordMuxer::onBeforeMediaCreation(
	MuxerMediaConfig &cfg,
	const struct pdraw_media_info *mediaInfo,
	const struct pdraw_muxer_media_params *params)
{
	/* Add track to muxer */
	return 0;
}


/* Called on the writer thread */
int PhotoRecordMuxer::onAfterMediaCreation(MuxerMedia *track,
					   const MuxerMediaConfig &cfg)
{
	return 0;
}


/* Called on the writer thread */
int PhotoRecordMuxer::internalSetThumbnail(enum pdraw_muxer_thumbnail_type type,
					   const uint8_t *data,
					   size_t size)
{
	PDRAW_CHECK_WRITER_THREAD(true);

	return 0;
}


/* Called on the writer thread */
int PhotoRecordMuxer::internalSetFileMetadata(
	const struct pdraw_muxer_metadata_params *params,
	const uint8_t *data,
	size_t size)
{
	return -ENOSYS;
}


/* Called on the writer thread */
void PhotoRecordMuxer::onInternalStopThread()
{
	PDRAW_CHECK_WRITER_THREAD(true);

	mIsMuxerReady = false;
}


/* Called on the writer thread */
int PhotoRecordMuxer::onWriterLoopInit()
{
	return 0;
}


/* Called on the writer thread */
int PhotoRecordMuxer::onWriterLoopCleanup()
{
	return 0;
}


/* Must be called on the loop thread */
int PhotoRecordMuxer::onBeforeAddMuxerMedias()
{
	PDRAW_CHECK_WRITER_THREAD(false);

	mIsMuxerReady = true;

	return 0;
}


/* Called on the writer thread */
int PhotoRecordMuxer::internalSetDynParams(
	const struct pdraw_muxer_dyn_params *dyn_params)
{
	PDRAW_CHECK_WRITER_THREAD(true);

	if (dyn_params->file_name_pattern == nullptr ||
	    dyn_params->file_name_pattern[0] == '\0')
		return -EINVAL;

	mFileNamePattern = std::string(dyn_params->file_name_pattern);
	mNextFileIndex = dyn_params->next_file_index;

	return 0;
}

} /* namespace Pdraw */
