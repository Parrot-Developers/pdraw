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

#include "pdraw_muxer.hpp"
#include <futils/futils.h>
#include <libpomp.hpp>

#include <array>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace Pdraw {


/* Forward declaration */
struct MuxerMediaConfig;


#define PDRAW_CHECK_WRITER_THREAD(expectWriter)                                \
	logThreadCheckWarning(__func__, expectWriter)

#define PDRAW_CHECK_WRITER_THREAD_SELF(self_ptr, expectWriter)                 \
	(self_ptr)->logThreadCheckWarning(__func__, expectWriter)


enum class CmdType : unsigned int {
	/* RecordMuxer commands */
	ADD_TRACK,
	ADD_QUEUE_EVENT,
	REMOVE_QUEUE_EVENT,
	SET_THUMBNAIL,
	SET_METADATA,
	FLUSH,
	DRAIN,
	STOP_THREAD,
	SET_DYN_PARAMS,
	SET_FILE_METADATA,

	/* IsoBmffRecordMuxer commands */
	ADD_CHAPTER,
	FORCE_SYNC,
};


using RecordTask = std::function<void()>;


class RecordMuxer : public Muxer {
public:
	RecordMuxer(Session *session,
		    Element::Listener *elementListener,
		    IPdraw::IMuxer::Listener *listener,
		    MuxerWrapper *wrapper,
		    const std::string &fileName,
		    const struct pdraw_muxer_params *params);

	~RecordMuxer() override;

	int
	addInputMedia(Media *media,
		      const struct pdraw_muxer_media_params *params) override;

	int addInputMedia(Media *media) override
	{
		return addInputMedia(media, nullptr);
	}

	int removeInputMedia(Media *media) override;

	int setThumbnail(enum pdraw_muxer_thumbnail_type type,
			 const uint8_t *data,
			 size_t size) override;

	int setFileMetadata(enum pdraw_muxer_metadata_type type,
			    const uint8_t *data,
			    size_t size,
			    const void *params,
			    size_t paramsSize) override;

	int
	setDynParams(const struct pdraw_muxer_dyn_params *dyn_params) override;

	int getDynParams(struct pdraw_muxer_dyn_params *dyn_params) override;

	int getStats(struct pdraw_muxer_stats *stats) override;

protected:
	class MuxerMedia;
	class MuxerCodedVideoMedia;
	class MuxerRawVideoMedia;
	class MuxerAudioMedia;

	virtual int onWriterLoopInit() = 0;

	virtual int onWriterLoopCleanup() = 0;

	virtual std::string getDefaultMediaName(Media::Type type) = 0;

	virtual int onBeforeAddMuxerMedias() = 0;

	virtual std::unique_ptr<RecordMuxer::MuxerMedia>
	createMedia(const MuxerMediaConfig &cfg) = 0;

	virtual int onBeforeMediaCreation(
		MuxerMediaConfig &cfg,
		const struct pdraw_media_info *mediaInfo,
		const struct pdraw_muxer_media_params *params) = 0;

	virtual int onAfterMediaCreation(MuxerMedia *track,
					 const MuxerMediaConfig &cfg) = 0;

	virtual int internalSetThumbnail(enum pdraw_muxer_thumbnail_type type,
					 const uint8_t *data,
					 size_t size) = 0;

	virtual int internalSetFileMetadata(enum pdraw_muxer_metadata_type type,
					    const uint8_t *data,
					    size_t size,
					    const void *params,
					    size_t paramsSize) = 0;

	virtual int internalSetDynParams(
		const struct pdraw_muxer_dyn_params *dyn_params) = 0;

	virtual int internalSetMetadata(uint32_t mediaId,
					const struct vmeta_session *metadata);

	virtual void onInternalStopThread() = 0;

	bool isThreadRunning() const
	{
		return (getThreadState() == State::STARTED);
	}

	void logThreadCheckWarning(const char *funcName,
				   bool shouldBeWriterThread) const;

	virtual bool canPostTask(bool needsMux) const;

	int postTask(CmdType type, const RecordTask &task);

	int ensureFreeSpace(size_t spaceNeeded);

	pomp::Loop *getThreadLoop() const
	{
		return mThread.loop.get();
	}

	std::string mFileName{};
	std::string mStorageDirPath{};
	mode_t mFileMode = 0;
	std::vector<std::unique_ptr<RecordMuxer::MuxerMedia>> mMedias;
	pdraw_muxer_stats mStats{};
	std::vector<uint8_t> mMetaBuffer{};
	uint64_t mMediaDate = 0;
	int32_t mMediaDateGmtOff = 0;
	std::atomic_bool mIsMuxerReady{false};
	std::atomic_bool mMetadataChanged{true};

private:
	int addMuxerMedia(Media *media,
			  const struct pdraw_muxer_media_params *params);

	int
	internalAddMuxerMedia(uint32_t mediaId,
			      Media::Type mediaType,
			      const struct pdraw_media_info *mediaInfo,
			      const struct pdraw_muxer_media_params *params);

	int internalAddQueueEvtToLoop(Media::Type type, mbuf::Queue *queue);

	int internalRemoveQueueEvtFromLoop(Media::Type type,
					   mbuf::Queue *queue);

	int internalStart() override;

	int internalStop() override;

	void onChannelFlush(Channel *channel) override;

	void onChannelDrain(Channel *channel) override;

	void onChannelSessionMetaUpdate(Channel *channel) override;

	int internalFlush(Channel *channel, bool discard);

	int process() override;

	void writerThread();

	void processTasks();

	int internalStopThread();

	static void callCompleteStop(void *userdata);

	Element::State getThreadState() const;

	void setThreadState(Element::State state);

	bool isThreadAlive() const;

	bool isThreadJoinable() const
	{
		return (isThreadAlive() || getThreadState() == State::STOPPED);
	}

	std::mutex mTasksMutex;
	std::queue<RecordTask> mPendingTasks;
	struct {
		std::thread thread{};
		std::thread::id id{};
		std::atomic<Element::State> state{State::INVALID};
		std::atomic_bool shouldStop{false};
		size_t taskIdCounter = 0;
		std::unique_ptr<pomp::Loop> loop{};
		std::unique_ptr<pomp::Event> evt{};
		pomp::Event::HandlerFunc evtHandler;
	} mThread;
	size_t mFreeSpaceLeft = 0;
	uint64_t mLastCheckFreeSpaceTime = 0;
	std::atomic_bool mPendingStop{false};
	std::atomic_bool mStopThreadReceived{false};
};

} /* namespace Pdraw */
