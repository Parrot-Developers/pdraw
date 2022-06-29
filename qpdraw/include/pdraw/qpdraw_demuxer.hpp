/**
 * Parrot Drones Awesome Video Viewer
 * Qt PDrAW demuxer object
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

#ifndef _QPDRAW_DEMUXER_HPP_
#define _QPDRAW_DEMUXER_HPP_

#include <QObject>

#include <pdraw/pdraw_defs.h>
#include <pdraw/qpdraw.hpp>

namespace QPdraw {

/* Forward declarations */
namespace Internal {
class QPdrawDemuxerPriv;
}


class QPdrawDemuxer : public QObject {
	Q_OBJECT

public:
	explicit QPdrawDemuxer(QPdraw *parent);

	~QPdrawDemuxer();

	/**
	 * Open a demuxer on a URL (stream or local file).
	 * The URL can be either an RTSP URL (starting with "rtsp://") or a
	 * local file path (either absolute or relative).
	 * The function returns before the actual opening is done. If the
	 * function returns 0, the openResponse() signal will be emitted once
	 * the open operation is successful (0 status) or has failed (negative
	 * errno status). If the function returns a negative errno value
	 * (immediate failure), the openResponse() signal will not be emitted.
	 * Once a demuxer is no longer used, it must be closed and then
	 * destroyed (@see the close() function).
	 * @param url: URL of the resource to open
	 * @return 0 on success, negative errno value in case of error
	 */
	int open(const std::string &url);

	/**
	 * Open a demuxer on a single stream.
	 * This function opens an RTP/AVP stream. No session management is done:
	 * it is the application's responsibility to handle the ports
	 * negociation with the sender. If null local ports are given as
	 * parameter, the effective port numbers used can be retrieved using the
	 * getSingleStreamLocalStreamPort() for the RTP port and
	 * getSingleStreamLocalControlPort() for the RTCP port functions.
	 * If the localAddr parameter is left empty, any local network interface
	 * will be used. The remoteAddr, remoteStreamPort and remoteControlPort
	 * parameters can be left null/empty if unknown; they will be known once
	 * the stream is being received.
	 * The function returns before the actual opening is done. If the
	 * function returns 0, the openResponse() signal will be emitted once
	 * the open operation is successful (0 status) or has failed (negative
	 * errno status). If the function returns a negative errno value
	 * (immediate failure), the openResponse() signal will not be emitted.
	 * Once a demuxer is no longer used, it must be closed and then
	 * destroyed (@see the close() function).
	 * @param url: URL of the resource to open
	 * @return 0 on success, negative errno value in case of error
	 */
	int open(const std::string &localAddr,
		 uint16_t localStreamPort,
		 uint16_t localControlPort,
		 const std::string &remoteAddr,
		 uint16_t remoteStreamPort,
		 uint16_t remoteControlPort);

	/**
	 * Open a demuxer on a stream URL through a mux channel.
	 * The URL must be an RTSP URL (starting with "rtsp://"). Mux channels
	 * are used to transfer data between a SkyController remote and a
	 * smartphone through USB; see Parrot's libmux for more information.
	 * No concurrent sessions can run on the mux channel; therefore the user
	 * must take care of limiting the number of PDrAW instances and demuxer
	 * objects running on the mux channel to only one.
	 * The function returns before the actual opening is done. If the
	 * function returns 0, the openResponse() signal will be emitted once
	 * the open operation is successful (0 status) or has failed (negative
	 * errno status). If the function returns a negative errno value
	 * (immediate failure), the openResponse() signal will not be emitted.
	 * Once a demuxer is no longer used, it must be closed and then
	 * destroyed (@see the close() function).
	 * @param url: URL of the resource to open
	 * @return 0 on success, negative errno value in case of error
	 */
	int open(const std::string &url, struct mux_ctx *mux);

	/**
	 * Close a demuxer.
	 * This function closes a previously opened demuxer. The function
	 * returns before the actual closing is done. If the function returns 0,
	 * the closeResponse() signal will be emitted once the close is
	 * successful (0 status) or has failed (negative errno status). If the
	 * function returns a negative errno value (immediate failure), the
	 * closeResponse() signal will not be emitted. After a successful close,
	 * the demuxer must be destroyed.
	 * @return 0 on success, negative errno value in case of error
	 */
	int close(void);

	/**
	 * Get the single stream local stream port.
	 * This function returns the local stream (RTP) port currently in use
	 * after a succesful open operation on a single stream (RTP/AVP) or an
	 * RTSP URL. If no open operation has been done, or if an open operation
	 * has been done on a mux channel, 0 is returned. This is useful when
	 * opening a single stream with null local ports to let PDrAW open
	 * sockets on any available port.
	 * @return the stream port on success, 0 in case of error
	 */
	uint16_t getSingleStreamLocalStreamPort(void);

	/**
	 * Get the single stream local control port.
	 * This function returns the local control (RTCP) port currently in use
	 * after a succesful open operation on a single stream (RTP/AVP) or an
	 * RTSP URL. If no open operation has been done, or if an open operation
	 * has been done on a mux channel, 0 is returned. This is useful when
	 * opening a single stream with null local ports to let PDrAW open
	 * sockets on any available port.
	 * @return the stream port on success, 0 in case of error
	 */
	uint16_t getSingleStreamLocalControlPort(void);

	/**
	 * Get the ready to play status.
	 * This function returns true if a successful open operation has been
	 * completed and if the playback is ready to start, false otherwise. One
	 * case when a successful open operation is complete but the playback is
	 * not ready is when connected to a SkyController's RTSP server but the
	 * SkyController itself is not yet connected to a drone. The value
	 * returned by this function is identical to the ready parameter passed
	 * to the readyToPlay() listener function when it is called.
	 * @return the ready to play status on success, false in case of error
	 */
	bool isReadyToPlay(void);

	/**
	 * Get the pause status.
	 * This function returns true if the playback is currently paused, false
	 * otherwise.
	 * @return the pause status on success, false in case of error
	 */
	bool isPaused(void);

	/**
	 * Play at the given speed.
	 * This function starts the playback of the video. The speed parameter
	 * is optional and defaults to 1.0. If the speed parameter is negative,
	 * the video is played backward. If the speed is greater than or equal
	 * to PDRAW_PLAY_SPEED_MAX, the speed is ignored and the video is played
	 * at the maximum speed achievable. If the speed is less than or equal
	 * to -PDRAW_PLAY_SPEED_MAX, the speed is ignored and the video is
	 * played backward at the maximum speed achievable. A 0.0 speed
	 * has the same effet as calling the pause() function. On a live
	 * stream, the speed parameter has no effect. The function returns
	 * before the actual operation is done. If the function returns 0, the
	 * playResponse() signal will be emitted once the play is successful (0
	 * status) or has failed (negative errno status). If the function
	 * returns a negative errno value (immediate failure), the
	 * playResponse() signal will not be emitted.
	 * @param speed: playback speed (0.0 means pause, negative value
	 *               means play backward)
	 * @return 0 on success, negative errno value in case of error
	 */
	int play(float speed = 1.0f);

	/**
	 * Pause the playback.
	 * This function suspends the playback of the video. The session is not
	 * closed and the playback can be resumed using the play() function. The
	 * function returns before the actual operation is done. If the function
	 * returns 0, the pauseResponse() signal will be emitted once the pause
	 * is successful (0 status) or has failed (negative errno status). If
	 * the function returns a negative errno value (immediate failure), the
	 * pauseResponse() signal will not be emitted.
	 * @return 0 on success, negative errno value in case of error
	 */
	int pause(void);

	/**
	 * Go to previous frame in frame-by-frame playback.
	 * This function plays the previous frame while the playback is paused.
	 * If the playback is not currently paused an error is returned.
	 * Frame-by-frame is only available on local replays (MP4 records). The
	 * function returns before the actual operation is done. If the function
	 * returns 0, the seekResponse() signal will be emitted once the seek is
	 * successful (0 status) or has failed (negative errno status). If the
	 * function returns a negative errno value (immediate failure), the
	 * seekResponse() signal will not be emitted.
	 * @return 0 on success, negative errno value in case of error
	 */
	int previousFrame(void);

	/**
	 * Go to next frame in frame-by-frame playback.
	 * This function plays the next frame while the playback is paused. If
	 * the playback is not currently paused an error is returned.
	 * Frame-by-frame is only available on local replays (MP4 records). The
	 * function returns before the actual operation is done. If the function
	 * returns 0, the seekResponse() signal will be emitted once the seek is
	 * successful (0 status) or has failed (negative errno status). If the
	 * function returns a negative errno value (immediate failure), the
	 * seekResponse() signal will not be emitted.
	 * @return 0 on success, negative errno value in case of error
	 */
	int nextFrame(void);

	/**
	 * Seek forward or backward.
	 * This function seeks forward (positive delta) or backward (negative
	 * delta). The delta parameter is in microseconds. When exact is false
	 * the seek is done to the nearest synchronization sample preceeding the
	 * delta, otherwise the seek is done to the sample nearest to the delta.
	 * Seeking is only available on replays (either local or streamed), not
	 * on live streams. The function returns before the actual operation is
	 * done. If the function returns 0, the seekResponse() signal will be
	 * emitted once the seek is successful (0 status) or has failed
	 * (negative errno status). If the function returns a negative errno
	 * value (immediate failure), the seekResponse() signal will not be
	 * emitted.
	 * @param delta: time delta in microseconds (positive or
	 *               negative)
	 * @param exact: true means seek to the sample closest to the
	 *               delta, false means seek to the nearest
	 *               synchronization sample preceeding the delta
	 * @return 0 on success, negative errno value in case of error
	 */
	int seek(int64_t delta, bool exact = false);

	/**
	 * Seek forward.
	 * This function seeks forward by delta microseconds. It has the same
	 * behavior as calling seek() with a positive delta. When exact is false
	 * the seek is done to the nearest synchronization sample preceeding the
	 * delta, otherwise the seek is done to the sample nearest to the delta.
	 * Seeking is only available on replays (either local or streamed), not
	 * on live streams. The function returns before the actual operation is
	 * done. If the function returns 0, the seekResponse() signal will be
	 * emitted once the seek is successful (0 status) or has failed
	 * (negative errno status). If the function returns a negative errno
	 * value (immediate failure), the seekResponse() signal will not be
	 * emitted.
	 * @param delta: positive time delta forward in microseconds
	 * @param exact: true means seek to the sample closest to the
	 *               delta, false means seek to the nearest
	 *               synchronization sample preceeding the delta
	 * @return 0 on success, negative errno value in case of error
	 */
	int seekForward(uint64_t delta, bool exact = false);

	/**
	 * Seek backward.
	 * This function seeks backward by delta microseconds (postive). It has
	 * the same behavior as calling seek() with a negative delta. When exact
	 * is false the seek is done to the nearest synchronization sample
	 * preceeding the delta, otherwise the seek is done to the sample
	 * nearest to the delta. Seeking is only available on replays (either
	 * local or streamed), not on live streams. The function returns before
	 * the actual operation is done. If the function returns 0, the
	 * seekResponse() signal will be emitted once the seek is successful
	 * (0 status) or has failed (negative errno status). If the function
	 * returns a negative errno value (immediate failure), the
	 * seekResponse() signal will not be emitted.
	 * @param delta: positive time delta backward in microseconds
	 * @param exact: true means seek to the sample closest to the
	 *               delta, false means seek to the nearest
	 *               synchronization sample preceeding the delta
	 * @return 0 on success, negative errno value in case of error
	 */
	int seekBack(uint64_t delta, bool exact = false);

	/**
	 * Seek to a given time.
	 * This function seeks to the given play timestamp in microseconds. When
	 * exact is false the seek is done to the nearest synchronization sample
	 * preceeding the timestamp, otherwise the seek is done to the sample
	 * nearest to the timestamp. Seeking is only available on replays
	 * (either local or streamed), not on live streams. The function returns
	 * before the actual operation is done. If the function returns 0, the
	 * seekResponse() signal will be emitted once the seek is successful
	 * (0 status) or has failed (negative errno status). If the function
	 * returns a negative errno value (immediate failure), the
	 * seekResponse() signal will not be emitted.
	 * @param timestamp: play timestamp in microseconds
	 * @param exact: true means seek to the sample closest to the
	 *               timestamp, false means seek to the nearest
	 *               synchronization sample preceeding the timestamp
	 * @return 0 on success, negative errno value in case of error
	 */
	int seekTo(uint64_t timestamp, bool exact = false);

	/**
	 * Get the playback duration.
	 * This function returns the playback duration in microseconds.
	 * The duration is only available on replays (either local or streamed),
	 * not on live streams.
	 * @return the duration in microseconds on success,
	 *         0 in case of error
	 */
	uint64_t getDuration(void);

	/**
	 * Get the playback current time.
	 * This function returns the current playback position in microseconds.
	 * On replays (either local or streamed) this is the position between 0
	 * and the duration; on live streams this is the time since the start of
	 * the stream session.
	 * @return the current time in microseconds on success,
	 *         0 in case of error
	 */
	uint64_t getCurrentTime(void);

signals:
	/**
	 * Open response signal, called when an open operation is complete or
	 * has failed. The status parameter is the open operation status: 0 on
	 * success, or a negative errno value in case of error.
	 * If this function reports an error, the close() function must be
	 * called and one must wait for the demuxerCloseResponse() listener
	 * function to be called prior to destroying the demuxer.
	 * @param status: 0 on success, negative errno value in
	 *                case of error
	 */
	void openResponse(int status);

	/**
	 * Close response signal, called when a close operation is complete or
	 * has failed. The status parameter is the close operation status: 0 on
	 * success, or a negative errno value in case of error.
	 * @param status: 0 on success, negative errno value in
	 *                case of error
	 */
	void closeResponse(int status);

	/**
	 * Unrecoverable error signal, called when a previously opened demuxer
	 * is no longer running. When this function is called, the demuxer is no
	 * longer running; the close() function must be called and one must wait
	 * for the demuxerCloseResponse() listener function to be called prior
	 * to destroying the demuxer.
	 */
	void onUnrecoverableError(void);

	/**
	 * Demuxer media selection signal, emitted with a list of video medias
	 * found from which the application must choose one or more to process
	 * in the pipeline. The retVal parameter must be a bitfield of the
	 * identifiers of the chosen medias (from the pdraw_demuxer_media
	 * structure), or 0 to choose the default media. If the retVal value is
	 * -ENOSYS, the callback is considered not implemented and the default
	 * media is chosen. If the retVal value is -ECANCELED no media is chosen
	 * and the open operation is aborted. If the retVal value is another
	 * negative errno or an invalid media identifier the openResponse()
	 * signal will be emitted if an open operation is in progress, or the
	 * onUnrecoverableError() signal otherwise.
	 * Note: this signal must be connected using a Qt::DirectConnection,
	 * to ensure that the slot is executed synchronously and that the
	 * retVal parameter is set before this function returns.
	 * @param medias: array of demuxer media
	 * @param count: demuxer media array element count
	 * @param retVal: a bitfield of the identifiers of the chosen chosen
	 *                medias, 0 or -ENOSYS to choose the default media,
	 *                -ECANCELED to choose no media and abort the open
	 *                operation, or another negative errno value in case
	 *                of error
	 */
	void selectMedia(const struct pdraw_demuxer_media *medias,
			 unsigned int count,
			 int *retVal);

	/**
	 * Ready to play signal, emitted when the playback is ready to start.
	 * This signal is emitted to indicate that the demuxer is ready to
	 * process play operations. Generally the demuxer is ready to play as
	 * soon as the openResponse() signal has been emitted with a success
	 * status. One case when the open operation was successful and the
	 * playback is not ready is when connected to a SkyController's RTSP
	 * server but the SkyController itself is not yet connected to a drone.
	 * Similarly, when connected to a drone's stream through
	 * a SkyController's RTSP server, if the drone is
	 * disconnected from the SkyController, this function
	 * will be called with a false value in the ready
	 * parameter.
	 * @param ready: true if the session is ready to play,
	 *               false otherwise
	 */
	void readyToPlay(bool ready);

	/**
	 * End of range signal, emitted when the playback is suspended after
	 * having reached the end of the playback duration. This signal is only
	 * emitted for replays (either local or streamed), not for live streams.
	 * The timestamp parameter is the current play time in microseconds at
	 * the moment the playback is suspended.
	 * @param timestamp: current playback time in
	 *                   microseconds
	 */
	void onEndOfRange(quint64 timestamp);

	/**
	 * Play response signal, emitted when a play operation is complete (the
	 * playback has started) or has failed. The status parameter is the play
	 * operation status: 0 on success, or a negative errno value in case of
	 * error. The timestamp parameter is the current play time in
	 * microseconds at the moment the playback is started. The speed
	 * parameter is the current playback speed; a negative value means
	 * playing backward.
	 * @param status: 0 on success, negative errno value
	 *                in case of error
	 * @param timestamp: current playback time in
	 *                   microseconds
	 * @param speed: current playback speed, negative means
	 *               backward
	 */
	void playResponse(int status, quint64 timestamp, float speed);

	/**
	 * Pause response signal, emitted when a pause operation is complete
	 * (the playback is suspended) or has failed. The status parameter is
	 * the pause operation status: 0 on success, or a negative errno value
	 * in case of error. The timestamp parameter is the current play time in
	 * microseconds at the moment the playback is paused.
	 * @param status: 0 on success, negative errno value
	 *                in case of error
	 * @param timestamp: current playback time in
	 *                   microseconds
	 */
	void pauseResponse(int status, quint64 timestamp);

	/**
	 * Seek response signal, emitted when a seek operation is complete or
	 * has failed. The status parameter is the seek operation status: 0 on
	 * success, or a negative errno value in case of error. The timestamp
	 * parameter is the current play time in microseconds after seeking. The
	 * speed parameter is the current playback speed; a negative value means
	 * playing backward.
	 * @param status: 0 on success, negative errno value
	 *                in case of error
	 * @param timestamp: current playback time in
	 *                   microseconds
	 * @param speed: current playback speed, negative means
	 *               backward
	 */
	void seekResponse(int status, quint64 timestamp, float speed);

private:
	Internal::QPdrawDemuxerPriv *mPriv;
};

} /* namespace QPdraw */

#endif /* !_QPDRAW_DEMUXER_HPP_ */
