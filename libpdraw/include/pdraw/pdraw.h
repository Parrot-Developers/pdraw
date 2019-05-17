/**
 * Parrot Drones Awesome Video Viewer Library
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

#ifndef _PDRAW_H_
#define _PDRAW_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <inttypes.h>

#include "pdraw_defs.h"


/* Forward declaration */
struct pdraw;


/* General callback functions */
struct pdraw_cbs {
	/* Open response callback function, called when an open operation is
	 * complete or has failed (optional, can be null, but highly recommended
	 * for correct PDrAW session management).
	 * The status parameter is the open operation status: 0 on success,
	 * or a negative errno value in case of error.
	 * @param pdraw: PDrAW instance handle
	 * @param status: 0 on success, negative errno value in case of error
	 * @param userdata: user data pointer */
	void (*open_resp)(struct pdraw *pdraw, int status, void *userdata);

	/* Close response callback function, called when a close operation is
	 * complete or has failed (optional, can be null, but highly recommended
	 * for correct PDrAW session management).
	 * The status parameter is the close operation status: 0 on success,
	 * or a negative errno value in case of error.
	 * @param pdraw: PDrAW instance handle
	 * @param status: 0 on success, negative errno value in case of error
	 * @param userdata: user data pointer */
	void (*close_resp)(struct pdraw *pdraw, int status, void *userdata);

	/* Unrecoverable error callback function, called when a previously
	 * opened session is no longer running. This happens only on streaming
	 * sessions, for example if the server has ended the stream. When this
	 * function is called, the PDrAW session is no longer running;
	 * pdraw_close() must be called and one must wait for the close_resp
	 * callback function to be issued prior to calling pdraw_destroy().
	 * @param pdraw: PDrAW instance handle
	 * @param userdata: user data pointer */
	void (*unrecoverable_error)(struct pdraw *pdraw, void *userdata);

	/* Demuxer media selection callback function, called with a list of
	 * video medias found from which the application must choose one
	 * to process in the pipeline. Generally, only 1 media will be
	 * available but there are cases when multiple medias can be available
	 * (e.g. local or streamed replay of an Anafi Thermal video). The return
	 * value of the callback function must be the identifier of the chosen
	 * media (from the pdraw_demuxer_media structure), or 0 to choose the
	 * default media. If the return value is -ENOSYS, the callback is
	 * considered not implemented and the default media is chosen. If the
	 * return value is another negative errno or an invalid media identifier
	 * the open_resp callback function will be called if an open operation
	 * is in progress, or the unrecoverable_error callback function
	 * otherwise.
	 * @param pdraw: PDrAW instance handle
	 * @param medias: array of demuxer media
	 * @param count: demuxer media array element count
	 * @param userdata: user data pointer
	 * @return the identifier of the chosen media, 0 or -ENOSYS to choose
	 *         the default media, or another negative errno value in case
	 *         of error */
	int (*select_demuxer_media)(struct pdraw *pdraw,
				    const struct pdraw_demuxer_media *medias,
				    size_t count,
				    void *userdata);

	/* Media added callback function, called when a media has been added
	 * internally in the PDrAW pipeline. Medias are for example YUV or H.264
	 * video medias. The info structure gives the media identifier that can
	 * be used to create a video sink on this media.
	 * @param pdraw: PDrAW instance handle
	 * @param info: pointer on the media information
	 * @param userdata: user data pointer */
	void (*media_added)(struct pdraw *pdraw,
			    const struct pdraw_media_info *info,
			    void *userdata);

	/* Media removed callback function, called when a media has been removed
	 * internally from the PDrAW pipeline. Medias are for example YUV or
	 * H.264 video medias. When a media is removed, any video sink created
	 * on this media must then be stopped.
	 * @param pdraw: PDrAW instance handle
	 * @param info: pointer on the media information
	 * @param userdata: user data pointer */
	void (*media_removed)(struct pdraw *pdraw,
			      const struct pdraw_media_info *info,
			      void *userdata);

	/* Ready to play callback function, called when the playback is ready
	 * to start (optional, can be null). This function is called to indicate
	 * that the PDrAW session is ready to process play operations.
	 * Generally the session is ready to play as soon as the open_resp
	 * callback function has been called with a success status. One case
	 * when the open operation was successful and the playback is not ready
	 * is when connected to a SkyController's RTSP server but the
	 * SkyController itself is not yet connected to a drone. Similarly,
	 * when connected to a drone's stream through a SkyController's RTSP
	 * server, if the drone is disconnected from the SkyController, this
	 * function will be called with a 0 value in the ready parameter.
	 * @param pdraw: PDrAW instance handle
	 * @param ready: 1 if the session is ready to play, 0 otherwise
	 * @param userdata: user data pointer */
	void (*ready_to_play)(struct pdraw *pdraw, int ready, void *userdata);

	/* End of range callback function, called when the playback is suspended
	 * after having reached the end of the playback duration (optional,
	 * can be null). This function is only called for replays (either local
	 * or streamed), not for live streams. The timestamp parameter is the
	 * current play time in microseconds at the moment the playback is
	 * suspended.
	 * @param pdraw: PDrAW instance handle
	 * @param timestamp: current playback time in microseconds
	 * @param userdata: user data pointer */
	void (*end_of_range)(struct pdraw *pdraw,
			     uint64_t timestamp,
			     void *userdata);

	/* Play response callback function, called when a play operation is
	 * complete (the playback has started) or has failed (optional, can be
	 * null). The status parameter is the play operation status: 0 on
	 * success, or a negative errno value in case of error. The timestamp
	 * parameter is the current play time in microseconds at the moment the
	 * playback is started. The speed parameter is the current playback
	 * speed; a negative value means playing backward.
	 * @param pdraw: PDrAW instance handle
	 * @param status: 0 on success, negative errno value in case of error
	 * @param timestamp: current playback time in microseconds
	 * @param speed: current playback speed, negative means backward
	 * @param userdata: user data pointer */
	void (*play_resp)(struct pdraw *pdraw,
			  int status,
			  uint64_t timestamp,
			  float speed,
			  void *userdata);

	/* Pause response callback function, called when a pause operation is
	 * complete (the playback is suspended) or has failed (optional, can be
	 * null). The status parameter is the pause operation status: 0 on
	 * success, or a negative errno value in case of error. The timestamp
	 * parameter is the current play time in microseconds at the moment the
	 * playback is paused.
	 * @param pdraw: PDrAW instance handle
	 * @param status: 0 on success, negative errno value in case of error
	 * @param timestamp: current playback time in microseconds
	 * @param userdata: user data pointer */
	void (*pause_resp)(struct pdraw *pdraw,
			   int status,
			   uint64_t timestamp,
			   void *userdata);

	/* Seek response callback function, called when a seek operation is
	 * complete or has failed (optional, can be null).
	 * The status parameter is the seek operation status: 0 on success,
	 * or a negative errno value in case of error. The timestamp parameter
	 * is the current play time in microseconds after seeking. The speed
	 * parameter is the current playback speed; a negative value means
	 * playing backward.
	 * @param pdraw: PDrAW instance handle
	 * @param status: 0 on success, negative errno value in case of error
	 * @param timestamp: current playback time in microseconds
	 * @param speed: current playback speed, negative means backward
	 * @param userdata: user data pointer */
	void (*seek_resp)(struct pdraw *pdraw,
			  int status,
			  uint64_t timestamp,
			  float speed,
			  void *userdata);

	/* Socket creation callback function, called immediately after a
	 * socket creation with its file descriptor as parameter (optional,
	 * can be null).
	 * @param pdraw: PDrAW instance handle
	 * @param fd: socket file descriptor
	 * @param userdata: user data pointer */
	void (*socket_created)(struct pdraw *pdraw, int fd, void *userdata);
};


/* Video renderer callback functions */
struct pdraw_video_renderer_cbs {
	/* Render ready callback function, called both when a new frame is
	 * ready for rendering and periodically (optional, can be null).
	 * This function is called from the pomp_loop thread.
	 * @param pdraw: PDrAW instance handle
	 * @param renderer: renderer handle
	 * @param userdata: user data pointer */
	void (*render_ready)(struct pdraw *pdraw,
			     struct pdraw_video_renderer *renderer,
			     void *userdata);

	/* External texture loading callback function (optional, can be null).
	 * This function is called before the rendering of the video frame in
	 * order to override the frame loading as a texture. This can be used
	 * to transform the video frames outside of PDrAW before resuming the
	 * rendering. This function is called from the rendering thread.
	 * @param pdraw: PDrAW instance handle
	 * @param renderer: renderer handle
	 * @param texture_width: texture width in pixels
	 * @param texture_height: texture height in pixels
	 * @param session_info: session information
	 * @param session_meta: session metadata
	 * @param frame: frame information
	 * @param frame_userdata: frame user data buffer
	 * @param frame_userdata_len: frame user data buffer size in bytes
	 * @param userdata: user data pointer
	 * @return 0 on success, negative errno value in case of error */
	int (*load_texture)(struct pdraw *pdraw,
			    struct pdraw_video_renderer *renderer,
			    unsigned int texture_width,
			    unsigned int texture_height,
			    const struct pdraw_session_info *session_info,
			    const struct vmeta_session *session_meta,
			    const struct pdraw_video_frame *frame,
			    const void *frame_userdata,
			    size_t frame_userdata_len,
			    void *userdata);

	/* Overlay rendering callback function (optional, can be null).
	 * This function is called after the rendering of the video frame in
	 * order to render an application overlay on top of the video. When
	 * HMD distorsion correction is enabled in the renderer, it is applied
	 * after the overlay rendering. This function is called from the
	 * rendering thread.
	 * @param pdraw: PDrAW instance handle
	 * @param renderer: renderer handle
	 * @param render_pos: rendering position
	 * @param content_pos: video content position
	 * @param view_mat: 4x4 view matrix
	 * @param proj_mat: 4x4 projection matrix
	 * @param session_info: session information
	 * @param session_meta: session metadata
	 * @param frame_meta: frame metadata
	 * @param frame_extra: frame extra information
	 * @param userdata: user data pointer */
	void (*render_overlay)(
		struct pdraw *pdraw,
		struct pdraw_video_renderer *renderer,
		const struct pdraw_rect *render_pos,
		const struct pdraw_rect *content_pos,
		const float *view_mat,
		const float *proj_mat,
		const struct pdraw_session_info *session_info,
		const struct vmeta_session *session_meta,
		const struct vmeta_frame *frame_meta,
		const struct pdraw_video_frame_extra *frame_extra,
		void *userdata);
};


/* Video sink callback functions */
struct pdraw_video_sink_cbs {
	/* Video sink flush callback function, called when flushing is required
	 * (mandatory). When this function is called, the application must flush
	 * the sink queue by calling vbuf_queue_flush() and must return all
	 * buffers outside of the queue by calling vbuf_unref(); once the
	 * flushing is done, the pdraw_video_sink_queue_flushed() function
	 * must be called.
	 * @param pdraw: PDrAW instance handle
	 * @param sink: video sink handle
	 * @param userdata: user data pointer */
	void (*flush)(struct pdraw *pdraw,
		      struct pdraw_video_sink *sink,
		      void *userdata);
};


/**
 * Instance management API
 */

/**
 * Create a PDrAW instance.
 * A running pomp_loop must be provided; all functions (with the exception of
 * rendering functions) must be called from the pomp_loop thread; all callback
 * functions (with the exception of rendering callbacks) are issued from the
 * pomp_loop thread. The callbacks structure must be provided but all callback
 * functions are optional. However, for correct management of the session it is
 * highly recommended to implement at least the open_resp and close_resp
 * functions. The instance handle is returned through the ret_obj parameter.
 * When no longer needed, the instance must be freed using the pdraw_destroy()
 * function. If a successful pdraw_open_*() was made, the pdraw_close()
 * function must be called and one must wait for the close_resp callback
 * function to be issued prior to calling pdraw_destroy().
 * @param loop: pomp_loop to use
 * @param cbs: PDrAW callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: PDrAW instance handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_new(struct pomp_loop *loop,
			const struct pdraw_cbs *cbs,
			void *userdata,
			struct pdraw **ret_obj);


/**
 * Free a PDrAW instance.
 * This function frees all resources associated with a PDrAW instance.
 * If a successful pdraw_open_*() was made, the pdraw_close() function must
 * be called and one must wait for the close_resp callback function to be
 * issued prior to calling pdraw_destroy().
 * @param pdraw: PDrAW instance handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_destroy(struct pdraw *pdraw);


/**
 * Session management API
 */

/**
 * Open a session on a URL (stream or local file).
 * The URL can be either an RTSP URL (starting with "rtsp://") or a local file
 * path (either absolute or relative).
 * The function returns before the actual opening is done. If the function
 * returns 0, the open_resp callback function will be issued once the open
 * operation is successful (0 status) or has failed (negative errno status).
 * If the function returns a negative errno value (immediate failure), the
 * open_resp callback function will not be issued. A failed open operation can
 * be retried, i.e. the open functions can be called again on the same PDrAW
 * instance.
 * @param pdraw: PDrAW instance handle
 * @param url: URL of the resource to open
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_open_url(struct pdraw *pdraw, const char *url);


/**
 * Open a session on a single stream.
 * This function opens an RTP/AVP stream. No session management is done: it
 * is the application's responsibility to handle the ports negociation with
 * the sender. If null local ports are given as parameter, the effective port
 * numbers used can be retrieved using the
 * pdraw_get_single_stream_local_stream_port() for the RTP port and
 * pdraw_get_single_stream_local_control_port() for the RTCP functions.
 * If the local_addr parameter is left null, any local network interface will
 * be used. The remote_addr, remote_stream_port and remote_control_port
 * parameters can be left null if unknown; they will be known once the stream
 * is being received.
 * The function returns before the actual opening is done. If the function
 * returns 0, the open_resp callback function will be issued once the open
 * operation is successful (0 status) or has failed (negative errno status).
 * If the function returns a negative errno value (immediate failure), the
 * open_resp callback function will not be issued. A failed open operation can
 * be retried, i.e. the open functions can be called again on the same PDrAW
 * instance.
 * @param pdraw: PDrAW instance handle
 * @param local_addr: local IP address (optional, can be NULL)
 * @param local_stream_port: local stream (RTP) port (optional, can be 0)
 * @param local_control_port: local control (RTCP) port (optional, can be 0)
 * @param remote_addr: remote IP address (optional, can be NULL)
 * @param remote_stream_port: remote stream (RTP) port (optional, can be 0)
 * @param remote_control_port: remote control (RTCP) port (optional, can be 0)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_open_single_stream(struct pdraw *pdraw,
				       const char *local_addr,
				       uint16_t local_stream_port,
				       uint16_t local_control_port,
				       const char *remote_addr,
				       uint16_t remote_stream_port,
				       uint16_t remote_control_port);


/**
 * Open a session on a stream URL through a mux channel.
 * The URL must be an RTSP URL (starting with "rtsp://"). Mux channels are used
 * to transfer data between a SkyController remote and a smartphone through USB;
 * see Parrot's libmux for more information.
 * No concurrent sessions can run on the mux channel; therefore the user must
 * take care of limiting the number of PDrAW instances running on the mux
 * channel to only one.
 * The function returns before the actual opening is done. If the function
 * returns 0, the open_resp callback function will be issued once the open
 * operation is successful (0 status) or has failed (negative errno status).
 * If the function returns a negative errno value (immediate failure), the
 * open_resp callback function will not be issued. A failed open operation can
 * be retried, i.e. the open functions can be called again on the same PDrAW
 * instance.
 * @param pdraw: PDrAW instance handle
 * @param url: URL of the resource to open
 * @param mux: mux instance handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int
pdraw_open_url_mux(struct pdraw *pdraw, const char *url, struct mux_ctx *mux);


/**
 * Close a session.
 * This function closes a previously opened session (either record or stream).
 * The function returns before the actual closing is done. If the function
 * returns 0, the close_resp callback function will be issued once the close is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the close_resp callback
 * function will not be issued. After a successful close, the PDrAW instance
 * must be destroyed by calling pdraw_destroy().
 * @param pdraw: PDrAW instance handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_close(struct pdraw *pdraw);


/**
 * Get the single stream local stream port.
 * This function returns the local stream (RTP) port currently in use after
 * a succesful open operation on a single stream (RTP/AVP) or an RTSP URL.
 * If no open operation has been done, or if an open operation has been done
 * on a mux channel, 0 is returned.
 * This is useful when calling pdraw_open_single_stream() with null local
 * ports to let PDrAW open sockets on any available port.
 * @param pdraw: PDrAW instance handle
 * @return the stream port on success, 0 in case of error
 */
PDRAW_API uint16_t
pdraw_get_single_stream_local_stream_port(struct pdraw *pdraw);


/**
 * Get the single stream local control port.
 * This function returns the local control (RTCP) port currently in use after
 * a succesful open operation on a single stream (RTP/AVP) or an RTSP URL.
 * If no open operation has been done, or if an open operation has been done
 * on a mux channel, 0 is returned.
 * This is useful when calling pdraw_open_single_stream() with null local
 * ports to let PDrAW open sockets on any available port.
 * @param pdraw: PDrAW instance handle
 * @return the stream port on success, 0 in case of error
 */
PDRAW_API uint16_t
pdraw_get_single_stream_local_control_port(struct pdraw *pdraw);


/**
 * Playback control API
 */

/**
 * Get the ready to play status.
 * This function returns 1 if a successful open operation has been completed
 * and if the playback is ready to start, 0 otherwise. One case when a
 * successful open operation is complete but the playback is not ready is when
 * connected to a SkyController's RTSP server but the SkyController itself is
 * not yet connected to a drone.
 * The value returned by this function is identical to the ready parameter
 * passed to the ready_to_play callback function when it is issued.
 * @param pdraw: PDrAW instance handle
 * @return the ready to play status on success, 0 in case of error
 */
PDRAW_API int pdraw_is_ready_to_play(struct pdraw *pdraw);


/**
 * Get the pause status.
 * This function returns 1 if the playback is currently paused, 0 otherwise.
 * @param pdraw: PDrAW instance handle
 * @return the pause status on success, 0 in case of error
 */
PDRAW_API int pdraw_is_paused(struct pdraw *pdraw);


/**
 * Play at normal speed (x1.0).
 * This function starts the playback of the video.
 * The function returns before the actual operation is done. If the function
 * returns 0, the play_resp callback function will be issued once the play is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the play_resp callback
 * function will not be issued.
 * @param pdraw: PDrAW instance handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_play(struct pdraw *pdraw);


/**
 * Play at the given speed.
 * This function starts the playback of the video. If the speed parameter is
 * negative, the video is played backward. If the speed is greater than or
 * equal to PDRAW_PLAY_SPEED_MAX, the speed is ignored and the video is played
 * at the maximum speed achievable. If the speed is less than or equal to
 * -PDRAW_PLAY_SPEED_MAX, the speed is ignored and the video is played backward
 * at the maximum speed achievable. A 0.0 speed has the same effet as calling
 * the pdraw_pause() function. On a live stream, the speed parameter has
 * no effect.
 * The function returns before the actual operation is done. If the function
 * returns 0, the play_resp callback function will be issued once the play is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the play_resp callback
 * function will not be issued.
 * @param pdraw: PDrAW instance handle
 * @param speed: playback speed (0.0 means pause, negative value means
 *               play backward)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_play_with_speed(struct pdraw *pdraw, float speed);


/**
 * Pause the playback.
 * This function suspends the playback of the video. The session is not closed
 * and the playback can be resumed using the play functions.
 * The function returns before the actual operation is done. If the function
 * returns 0, the pause_resp callback function will be issued once the pause is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the pause_resp callback
 * function will not be issued.
 * @param pdraw: PDrAW instance handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_pause(struct pdraw *pdraw);


/**
 * Go to previous frame in frame-by-frame playback.
 * This function plays the previous frame while the playback is paused. If the
 * playback is not currently paused an error is returned. Frame-by-frame is
 * only available on local replays (MP4 records).
 * The function returns before the actual operation is done. If the function
 * returns 0, the seek_resp callback function will be issued once the seek is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the seek_resp callback
 * function will not be issued.
 * @param pdraw: PDrAW instance handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_previous_frame(struct pdraw *pdraw);


/**
 * Go to next frame in frame-by-frame playback.
 * This function plays the next frame while the playback is paused. If the
 * playback is not currently paused an error is returned. Frame-by-frame is
 * only available on local replays (MP4 records).
 * The function returns before the actual operation is done. If the function
 * returns 0, the seek_resp callback function will be issued once the seek is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the seek_resp callback
 * function will not be issued.
 * @param pdraw: PDrAW instance handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_next_frame(struct pdraw *pdraw);


/**
 * Seek forward or backward.
 * This function seeks forward (positive delta) or backward (negative delta).
 * The delta parameter is in microseconds. The exact parameter is a boolean
 * value; when exact is 0 the seek is done to the nearest synchronization sample
 * preceeding the delta, otherwise the seek is done to the sample nearest to the
 * delta. Seeking is only available on replays (either local or streamed),
 * not on live streams.
 * The function returns before the actual operation is done. If the function
 * returns 0, the seek_resp callback function will be issued once the seek is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the seek_resp callback
 * function will not be issued.
 * @param pdraw: PDrAW instance handle
 * @param delta: time delta in microseconds (positive or negative)
 * @param exact: 1 means seek to the sample closest to the delta, 0 means seek
 *               to the nearest synchronization sample preceeding the delta
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_seek(struct pdraw *pdraw, int64_t delta, int exact);


/**
 * Seek forward.
 * This function seeks forward by delta microseconds. It has the same
 * behavior as calling pdraw_seek() with a positive delta. The exact parameter
 * is a boolean value; when exact is 0 the seek is done to the nearest
 * synchronization sample preceeding the delta, otherwise the seek is done to
 * the sample nearest to the delta. Seeking is only available on replays
 * (either local or streamed), not on live streams.
 * The function returns before the actual operation is done. If the function
 * returns 0, the seek_resp callback function will be issued once the seek is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the seek_resp callback
 * function will not be issued.
 * @param pdraw: PDrAW instance handle
 * @param delta: positive time delta forward in microseconds
 * @param exact: 1 means seek to the sample closest to the delta, 0 means seek
 *               to the nearest synchronization sample preceeding the delta
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int
pdraw_seek_forward(struct pdraw *pdraw, uint64_t delta, int exact);


/**
 * Seek backward.
 * This function seeks backward by delta microseconds (postive). It has the same
 * behavior as calling pdraw_seek() with a negative delta. The exact parameter
 * is a boolean value; when exact is 0 the seek is done to the nearest
 * synchronization sample preceeding the delta, otherwise the seek is done to
 * the sample nearest to the delta. Seeking is only available on replays
 * (either local or streamed), not on live streams.
 * The function returns before the actual operation is done. If the function
 * returns 0, the seek_resp callback function will be issued once the seek is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the seek_resp callback
 * function will not be issued.
 * @param pdraw: PDrAW instance handle
 * @param delta: positive time delta backward in microseconds
 * @param exact: 1 means seek to the sample closest to the delta, 0 means seek
 *               to the nearest synchronization sample preceeding the delta
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_seek_back(struct pdraw *pdraw, uint64_t delta, int exact);


/**
 * Seek to a given time.
 * This function seeks to the given play timestamp in microseconds. The exact
 * parameter is a boolean value; when exact is 0 the seek is done to the nearest
 * synchronization sample preceeding the timestamp, otherwise the seek is done
 * to the sample nearest to the timestamp. Seeking is only available on replays
 * (either local or streamed), not on live streams.
 * The function returns before the actual operation is done. If the function
 * returns 0, the seek_resp callback function will be issued once the seek is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the seek_resp callback
 * function will not be issued.
 * @param pdraw: PDrAW instance handle
 * @param timestamp: play timestamp in microseconds
 * @param exact: 1 means seek to the sample closest to the timestamp,
 *               0 means seek to the nearest synchronization sample
 *               preceeding the timestamp
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_seek_to(struct pdraw *pdraw, uint64_t timestamp, int exact);


/**
 * Get the playback duration.
 * This function returns the playback duration in microseconds. The duration is
 * only available on replays (either local or streamed), not on live streams.
 * @param pdraw: PDrAW instance handle
 * @return the duration in microseconds on success, 0 in case of error
 */
PDRAW_API uint64_t pdraw_get_duration(struct pdraw *pdraw);


/**
 * Get the playback current time.
 * This function returns the current playback position in microseconds.
 * On replays (either local or streamed) this is the position between 0 and
 * the duration; on live streams this is the time since the start of the
 * stream session.
 * @param pdraw: PDrAW instance handle
 * @return the current time in microseconds on success, 0 in case of error
 */
PDRAW_API uint64_t pdraw_get_current_time(struct pdraw *pdraw);


/**
 * Video renderer API
 * @warning: all functions must be called from the application's
 * rendering thread
 */

/**
 * Start a video renderer.
 * This function creates a video renderer. Once the renderer is created, the
 * rendering is done by calling the pdraw_render_video*() functions. Once a
 * renderer is no longer used it must be destroyed by calling the
 * pdraw_stop_video_renderer() function.
 * The render_pos parameter sets the position and size of the rendering in the
 * window/view; these coordinates are in pixels from the bottom-left corner
 * (OpenGL coordinates). The params structure must be provided but all
 * parameters are optional and can be left null.
 * The callbacks structure must be provided but all callback functions are
 * optional; all callback functions are called from the rendering thread,
 * except the render_ready function which is called from the pomp_loop thread.
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW instance handle
 * @param render_pos: rendering position and size
 * @param params: renderer parameters
 * @param cbs: renderer callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: renderer handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int
pdraw_start_video_renderer(struct pdraw *pdraw,
			   const struct pdraw_rect *render_pos,
			   const struct pdraw_video_renderer_params *params,
			   const struct pdraw_video_renderer_cbs *cbs,
			   void *userdata,
			   struct pdraw_video_renderer **ret_obj);


/**
 * Start a video renderer on an EGL display.
 * This function creates a video renderer on an active EGL display context.
 * Once the renderer is created, the rendering is done by calling the
 * pdraw_render_video*() functions. Once a renderer is no longer used it must
 * be destroyed by calling the pdraw_stop_video_renderer() function.
 * The render_pos parameter sets the position and size of the rendering in the
 * window/view; these coordinates are in pixels from the bottom-left corner
 * (OpenGL coordinates). The params structure must be provided but all
 * parameters are optional and can be left null.
 * The callbacks structure must be provided but all callback functions are
 * optional; all callback functions are called from the rendering thread,
 * except the render_ready function which is called from the pomp_loop thread.
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW instance handle
 * @param render_pos: rendering position and size
 * @param params: renderer parameters
 * @param cbs: renderer callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param egl_display: EGL display context
 * @param ret_obj: renderer handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int
pdraw_start_video_renderer_egl(struct pdraw *pdraw,
			       const struct pdraw_rect *render_pos,
			       const struct pdraw_video_renderer_params *params,
			       const struct pdraw_video_renderer_cbs *cbs,
			       void *userdata,
			       struct egl_display *egl_display,
			       struct pdraw_video_renderer **ret_obj);


/**
 * Stop a video renderer.
 * This function stops a running video renderer and frees the associated
 * resources.
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW instance handle
 * @param renderer: renderer handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_stop_video_renderer(struct pdraw *pdraw,
					struct pdraw_video_renderer *renderer);


/**
 * Resize a video renderer.
 * This function updates the rendering position on a running video renderer.
 * The render_pos parameter sets the position and size of the rendering in the
 * window/view; these coordinates are in pixels from the bottom-left corner
 * (OpenGL coordinates).
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW instance handle
 * @param renderer: renderer handle
 * @param render_pos: rendering position and size
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_resize_video_renderer(struct pdraw *pdraw,
					  struct pdraw_video_renderer *renderer,
					  const struct pdraw_rect *render_pos);


/**
 * Set the video renderer parameters.
 * This function updates the rendering parameters on a running video renderer.
 * The params structure must be provided but all parameters are optional and
 * can be left null.
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW instance handle
 * @param renderer: renderer handle
 * @param params: renderer parameters
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_set_video_renderer_params(
	struct pdraw *pdraw,
	struct pdraw_video_renderer *renderer,
	const struct pdraw_video_renderer_params *params);


/**
 * Get the video renderer parameters.
 * This function retrieves the rendering parameters on a running video renderer.
 * The provided params structure is filled by the function.
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW instance handle
 * @param renderer: renderer handle
 * @param params: renderer parameters (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int
pdraw_get_video_renderer_params(struct pdraw *pdraw,
				struct pdraw_video_renderer *renderer,
				struct pdraw_video_renderer_params *params);


/**
 * Render the video.
 * This function renders the video with the current rendering position and
 * rendering parameters.
 * For render-on-demand, consider using the render_ready callback function
 * which is called both when a new frame is ready for rendering and
 * periodically. Note that the render_ready callback function is called from
 * the pomp_loop thread, not the rendering thread; the synchronization is up
 * to the caller.
 * If a content_pos structure is provided, it is filled with the actual position
 * and size of the video within the rendering position; these coordinates are
 * in pixels from the bottom-left corner (OpenGL coordinates).
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW instance handle
 * @param renderer: renderer handle
 * @param content_pos: video content position (output; optional, can be null)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_render_video(struct pdraw *pdraw,
				 struct pdraw_video_renderer *renderer,
				 struct pdraw_rect *content_pos);


/**
 * Render the video with provided matrices.
 * This function renders the video with the current rendering position and
 * rendering parameters, and the provided view and projection matrices.
 * The view_mat and proj_mat matrices are 4x4 OpenGL matrices.
 * For render-on-demand, consider using the render_ready callback function
 * which is called both when a new frame is ready for rendering and
 * periodically. Note that the render_ready callback function is called from
 * the pomp_loop thread, not the rendering thread; the synchronization is up
 * to the caller.
 * If a content_pos structure is provided, it is filled with the actual position
 * and size of the video within the rendering position; these coordinates are
 * in pixels from the bottom-left corner (OpenGL coordinates).
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW instance handle
 * @param renderer: renderer handle
 * @param content_pos: video content position (output; optional, can be null)
 * @param view_mat: 4x4 view matrix
 * @param proj_mat: 4x4 projection matrix
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_render_video_mat(struct pdraw *pdraw,
				     struct pdraw_video_renderer *renderer,
				     struct pdraw_rect *content_pos,
				     const float *view_mat,
				     const float *proj_mat);


/**
 * Video sink API
 */

/**
 * Start a video sink.
 * This function creates a video sink on a media of the given media_id.
 * The media idenfifiers are known when the media_added or media_removed
 * general callback functions are called.
 * Once the sink is created, video frames are retrieved by getting buffers
 * from the buffer queue returned by the pdraw_get_video_sink_queue() function.
 * Once a video sink is no longer used, it must be destroyed by calling the
 * pdraw_stop_video_sink() function.
 * The params structure must be provided but all parameters are optional and
 * can be left null.
 * The callbacks structure must be provided and the flush callback function is
 * required to be implemented; all callback functions are called from the
 * pomp_loop thread. When the flush callback function is called, the
 * application must flush the sink queue by calling vbuf_queue_flush() and
 * must return all buffers outside of the queue by calling vbuf_unref();
 * once the flushing is complete, the pdraw_video_sink_queue_flushed()
 * function must be called.
 * @param pdraw: PDrAW instance handle
 * @param media_id: identifier of the media on which to create the sink
 * @param params: video sink parameters
 * @param cbs: video sink callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: video sink handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int
pdraw_start_video_sink(struct pdraw *pdraw,
		       unsigned int media_id,
		       const struct pdraw_video_sink_params *params,
		       const struct pdraw_video_sink_cbs *cbs,
		       void *userdata,
		       struct pdraw_video_sink **ret_obj);


/**
 * Stop a video sink.
 * This function stops a running video sink and frees the associated resources.
 * A video sink must not be stopped unless all buffers outside of the queue
 * have been returned by calling vbuf_unref(). Once a video sink is stopped
 * the queue returned by pdraw_get_video_sink_queue() must no longer be used.
 * @param pdraw: PDrAW instance handle
 * @param sink: video sink handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_stop_video_sink(struct pdraw *pdraw,
				    struct pdraw_video_sink *sink);


/**
 * Resynchronize a video sink.
 * This function schedules the output of a synchronization frame (IDR) for a
 * running video sink. This is only useful for video medias of type
 * PDRAW_VIDEO_MEDIA_FORMAT_H264. It can be used for example in case of
 * unrecoverable video decoder errors to restart decoding. After a video sink
 * creation, the first frame that is output is always a synchronization frame;
 * therefore it is not necessary to call this function immediately after a
 * video sink creation.
 * @param pdraw: PDrAW instance handle
 * @param sink: video sink handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_resync_video_sink(struct pdraw *pdraw,
				      struct pdraw_video_sink *sink);


/**
 * Get the video sink buffer queue.
 * This function returns the buffer queue to use in order to retrieve frames
 * from a running video sink. Buffers are retrieved from the queue by using the
 * vbuf_queue_pop() function.
 * @param pdraw: PDrAW instance handle
 * @param sink: video sink handle
 * @return a pointer on a vbuf_queue object on success, NULL in case of error
 */
PDRAW_API struct vbuf_queue *
pdraw_get_video_sink_queue(struct pdraw *pdraw, struct pdraw_video_sink *sink);


/**
 * Signal that a video sink has been flushed.
 * This function is used to signal that flushing is complete. When the flush
 * video sink callback function is called, the application must flush the sink
 * queue by calling vbuf_queue_flush() and must return all buffers outside of
 * the queue by calling vbuf_unref(); once the flushing is complete, this
 * function must be called.
 * @param pdraw: PDrAW instance handle
 * @param sink: video sink handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_video_sink_queue_flushed(struct pdraw *pdraw,
					     struct pdraw_video_sink *sink);


/**
 * Metadata API
 */

/**
 * Get the session type.
 * This function returns the session type, which is either live or replay,
 * for an opened PDrAW session.
 * @param pdraw: PDrAW instance handle
 * @return the session type, or PDRAW_SESSION_TYPE_UNKNOWN in case of error
 */
PDRAW_API enum pdraw_session_type pdraw_get_session_type(struct pdraw *pdraw);


/**
 * Get the PDrAW instance friendly name.
 * This function fills the str array with the null-terminated friendly name.
 * The string must have been previously allocated. The function writes up to
 * len characters. The friendly name is generally either the device's friendly
 * name (e.g. "Bob's phone") or the application's name (e.g.
 * "MyDroneControllerApp"). It is used in metadata exchanged with a streaming
 * server.
 * @param pdraw: PDrAW instance handle
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int
pdraw_get_self_friendly_name(struct pdraw *pdraw, char *str, size_t len);


/**
 * Set the PDrAW instance friendly name.
 * The friendly_name string is copied internally. The friendly name is generally
 * either the device's friendly name (e.g. "Bob's phone") or the application's
 * name (e.g. "MyDroneControllerApp"). It is used in metadata exchanged with a
 * streaming server. Setting the friendly name value is optional.
 * @param pdraw: PDrAW instance handle
 * @param friendly_name: pointer to the friendly name string
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_set_self_friendly_name(struct pdraw *pdraw,
					   const char *friendly_name);


/**
 * Get the PDrAW instance serial number.
 * This function fills the str array with the null-terminated serial number.
 * The string must have been previously allocated. The function writes up to
 * len characters. The serial number is generally the unique serial number of
 * the device on which PDrAW is running. It is used in metadata exchanged with
 * a streaming server.
 * @param pdraw: PDrAW instance handle
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int
pdraw_get_self_serial_number(struct pdraw *pdraw, char *str, size_t len);


/**
 * Set the PDrAW instance serial number.
 * The serial_number string is copied internally. The serial number is generally
 * the unique serial number of the device on which PDrAW is running. It is used
 * in metadata exchanged with a streaming server. Setting the serial number
 * value is recommended as it is used as a unique identifier in the streaming
 * protocols.
 * @param pdraw: PDrAW instance handle
 * @param serial_number: pointer to the serial number string
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_set_self_serial_number(struct pdraw *pdraw,
					   const char *serial_number);


/**
 * Get the PDrAW instance software version.
 * This function fills the str array with the null-terminated software version.
 * The string must have been previously allocated. The function writes up to
 * len characters. The software version is generally the version number of the
 * application running PDrAW (e.g. "MyApp v1.2.3"). It is used in metadata
 * exchanged with a streaming server.
 * @param pdraw: PDrAW instance handle
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int
pdraw_get_self_software_version(struct pdraw *pdraw, char *str, size_t len);


/**
 * Set the PDrAW instance software version.
 * The software_version string is copied internally. The software version is
 * generally the version number of the application running PDrAW (e.g.
 * "MyApp v1.2.3"). It is used in metadata exchanged with a streaming server.
 * Setting the software version value is optional.
 * @param pdraw: PDrAW instance handle
 * @param software_version: pointer to the software version string
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_set_self_software_version(struct pdraw *pdraw,
					      const char *software_version);


/**
 * Get the pilot status.
 * This function returns 1 if the PDrAW session corresponds to the drone's
 * pilot, 0 otherwise. This value must be set by the application by using the
 * pdraw_set_self_pilot() function. The default value is 1. This value is
 * mostly used as metadata for the video rendering callback functions in the
 * pdraw_session_info structure.
 * @param pdraw: PDrAW instance handle
 * @return the pilot status on success, 0 in case of error
 */
PDRAW_API int pdraw_is_self_pilot(struct pdraw *pdraw);


/**
 * Set the pilot status.
 * The application should set is_pilot to 1 if the PDrAW session corresponds
 * to the drone's pilot, 0 otherwise. The default value is 1. This value is
 * mostly used as metadata for the video rendering callback functions in the
 * pdraw_session_info structure.
 * @param pdraw: PDrAW instance handle
 * @param is_pilot: pilot status
 * @return the pilot status on success, 0 in case of error
 */
PDRAW_API int pdraw_set_self_pilot(struct pdraw *pdraw, int is_pilot);


/**
 * Get the peer session metadata.
 * This function returns the drone's session (i.e. untimed) metadata on a stream
 * or record. For the session metadata to be known, a session must be opened.
 * The session structure is filled by the function and must have been previously
 * allocated.
 * @param pdraw: PDrAW instance handle
 * @param session: pointer to a session metadata structure (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_get_peer_session_metadata(struct pdraw *pdraw,
					      struct vmeta_session *session);


/**
 * Get the peer drone model.
 * This function returns the drone model of a stream or record if it is known.
 * For the drone model to be known, a session must be opened.
 * @param pdraw: PDrAW instance handle
 * @return the drone model, or PDRAW_DRONE_MODEL_UNKNOWN in case of error
 */
PDRAW_API enum pdraw_drone_model
pdraw_get_peer_drone_model(struct pdraw *pdraw);


/**
 * Settings API
 */

/**
 * Get the pipeline mode setting.
 * This function returns the pipeline mode of a PDrAW instance. The pipeline
 * mode controls whether to decode the selected video media (for full processing
 * up to the rendering), or to disable video decoding (e.g. when no rendering
 * is required, only an H.264 video sink).
 * @param pdraw: PDrAW instance handle
 * @return the pipeline mode, or PDRAW_PIPELINE_MODE_DECODE_ALL in case of error
 */
PDRAW_API enum pdraw_pipeline_mode
pdraw_get_pipeline_mode_setting(struct pdraw *pdraw);


/**
 * Set the pipeline mode setting.
 * This function sets the pipeline mode of a PDrAW instance. This function can
 * be called only prior to any open operation. The pipeline mode controls
 * whether to decode the selected video media (for full processing up to the
 * rendering), or to disable video decoding (e.g. when no rendering is required,
 * only an H.264 video sink).
 * @param pdraw: PDrAW instance handle
 * @param mode: pipeline mode
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_set_pipeline_mode_setting(struct pdraw *pdraw,
					      enum pdraw_pipeline_mode mode);


/**
 * Get the display screen settings.
 * This function returns the display screen settings through the xdpi, ydpi and
 * device_margin_* parameters. This is only useful if HMD distortion correction
 * is enabled in the video rendering. The xdpi and ydpi are pixel densities in
 * dots per inches. The device margins are in millimeters.
 * @param pdraw: PDrAW instance handle
 * @param xdpi: horizontal pixel density (output)
 * @param ydpi: vertical pixel density (output)
 * @param device_margin_top: top device margin (output)
 * @param device_margin_bottom: bottom device margin (output)
 * @param device_margin_left: left device margin (output)
 * @param device_margin_right: right device margin (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_get_display_screen_settings(struct pdraw *pdraw,
						float *xdpi,
						float *ydpi,
						float *device_margin_top,
						float *device_margin_bottom,
						float *device_margin_left,
						float *device_margin_right);


/**
 * Set the display screen settings.
 * This function sets the display screen settings. This is only useful if HMD
 * distortion correction is enabled in the video rendering. The xdpi and ydpi
 * are pixel densities in dots per inches. The device margins are in
 * millimeters.
 * @param pdraw: PDrAW instance handle
 * @param xdpi: horizontal pixel density
 * @param ydpi: vertical pixel density
 * @param device_margin_top: top device margin
 * @param device_margin_bottom: bottom device margin
 * @param device_margin_left: left device margin
 * @param device_margin_right: right device margin
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_set_display_screen_settings(struct pdraw *pdraw,
						float xdpi,
						float ydpi,
						float device_margin_top,
						float device_margin_bottom,
						float device_margin_left,
						float device_margin_right);


/**
 * Get the HMD model setting.
 * This function returns the head-mounted display (HMD) model. This is only
 * useful if HMD distortion correction is enabled in the video rendering.
 * @param pdraw: PDrAW instance handle
 * @return the HMD model, or PDRAW_HMD_MODEL_UNKNOWN in case of error
 */
PDRAW_API enum pdraw_hmd_model pdraw_get_hmd_model_setting(struct pdraw *pdraw);


/**
 * Set the HMD model setting.
 * This function sets the head-mounted display (HMD) model. This is only
 * useful if HMD distortion correction is enabled in the video rendering.
 * @param pdraw: PDrAW instance handle
 * @param hmd_model: HMD model
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_set_hmd_model_setting(struct pdraw *pdraw,
					  enum pdraw_hmd_model hmd_model);

/**
 * Platform-specific API
 */

/**
 * Set the Android JVM pointer.
 * This function sets the JVM pointer for internal calls to the Android SDK API.
 * This is only useful on Android and is ignored on other platforms. If the
 * JVM pointer is not provided on Android platforms, some features may not be
 * available.
 * @param pdraw: PDrAW instance handle
 * @param jvm: JVM pointer
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_set_android_jvm(struct pdraw *pdraw, void *jvm);


/**
 * Helpers
 */

/**
 * ToString function for enum pdraw_drone_model.
 * @param val: drone model value to convert
 * @return a string description of the drone model
 */
PDRAW_API const char *pdraw_drone_model_str(enum pdraw_drone_model val);


/**
 * ToString function for enum pdraw_hmd_model.
 * @param val: HMD model value to convert
 * @return a string description of the HMD model
 */
PDRAW_API const char *pdraw_hmd_model_str(enum pdraw_hmd_model val);


/**
 * ToString function for enum pdraw_pipeline_mode.
 * @param val: pipeline mode value to convert
 * @return a string description of the pipeline mode
 */
PDRAW_API const char *pdraw_pipeline_mode_str(enum pdraw_pipeline_mode val);


/**
 * ToString function for enum pdraw_session_type.
 * @param val: session type value to convert
 * @return a string description of the session type
 */
PDRAW_API const char *pdraw_session_type_str(enum pdraw_session_type val);


/**
 * ToString function for enum pdraw_media_type.
 * @param val: media type value to convert
 * @return a string description of the media type
 */
PDRAW_API const char *pdraw_media_type_str(enum pdraw_media_type val);


/**
 * ToString function for enum pdraw_video_media_format.
 * @param val: video media format value to convert
 * @return a string description of the video media format
 */
PDRAW_API const char *
pdraw_video_media_format_str(enum pdraw_video_media_format val);


/**
 * ToString function for enum pdraw_yuv_format.
 * @param val: YUV format value to convert
 * @return a string description of the YUV format
 */
PDRAW_API const char *pdraw_yuv_format_str(enum pdraw_yuv_format val);


/**
 * ToString function for enum pdraw_h264_format.
 * @param val: H.264 format value to convert
 * @return a string description of the H.264 format
 */
PDRAW_API const char *pdraw_h264_format_str(enum pdraw_h264_format val);


/**
 * ToString function for enum pdraw_video_type.
 * @param val: video type value to convert
 * @return a string description of the video type
 */
PDRAW_API const char *pdraw_video_type_str(enum pdraw_video_type val);


/**
 * ToString function for enum pdraw_histogram_channel.
 * @param val: histogram channel value to convert
 * @return a string description of the histogram channel
 */
PDRAW_API const char *
pdraw_histogram_channel_str(enum pdraw_histogram_channel val);


/**
 * ToString function for enum pdraw_video_renderer_fill_mode.
 * @param val: video renderer fill mode value to convert
 * @return a string description of the video renderer fill mode
 */
PDRAW_API const char *
pdraw_video_renderer_fill_mode_str(enum pdraw_video_renderer_fill_mode val);


/**
 * ToString function for enum pdraw_video_renderer_transition_flag.
 * @param val: video renderer transition flag value to convert
 * @return a string description of the video renderer transition flag
 */
PDRAW_API const char *pdraw_video_renderer_transition_flag_str(
	enum pdraw_video_renderer_transition_flag val);


/**
 * Convert a video frame metadata structure to a JSON object.
 * The JSON object must have been previously created. The function appends
 * new elements in this object.
 * @param frame: pointer to a video frame structure
 * @param jobj: pointer to a JSON object to fill (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_video_frame_to_json(const struct pdraw_video_frame *frame,
					struct json_object *jobj);


/**
 * Convert a video frame metadata structure to a JSON string.
 * This function fills the str array with the null-terminated JSON string.
 * The string must have been previously allocated. The function writes
 * up to len characters.
 * @param frame: pointer to a video frame structure
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int
pdraw_video_frame_to_json_str(const struct pdraw_video_frame *frame,
			      char *str,
			      unsigned int len);


/**
 * Pack an YUV video frame structure that points to non-contiguous planes into
 * a contiguous buffer.
 * This function changes the the output buffer capacity if necessary, copies
 * the input video frame into the output buffer and fills the output video
 * frame structure that will point the the output buffer.
 * @param in_frame: pointer to a non-contiguous video frame structure
 * @param out_frame: pointer to an output video frame structure
 * @param out_buf: pointer to an output buffer
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_pack_yuv_frame(const struct pdraw_video_frame *in_frame,
				   struct pdraw_video_frame *out_frame,
				   struct vbuf_buffer *out_buf);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_PDRAW_H_ */
