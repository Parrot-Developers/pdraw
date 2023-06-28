/**
 * Parrot Drones Awesome Video Viewer
 * PDrAW back-end library
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

#ifndef _PDRAW_BACKEND_H_
#define _PDRAW_BACKEND_H_

#include <inttypes.h>

#include <libpomp.h>
#include <pdraw/pdraw.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* To be used for all public API */
#ifdef PDRAW_BACKEND_API_EXPORTS
#	ifdef _WIN32
#		define PDRAW_BACKEND_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define PDRAW_BACKEND_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !PDRAW_BACKEND_API_EXPORTS */
#	define PDRAW_BACKEND_API
#endif /* !PDRAW_BACKEND_API_EXPORTS */


/* Forward declarations */
struct pdraw_backend;


/* General callback functions */
struct pdraw_backend_cbs {
	/* Stop response callback function, called when a stop operation is
	 * complete or has failed (optional, can be null, but highly recommended
	 * for correct PDrAW session management).
	 * The status parameter is the stop operation status: 0 on success,
	 * or a negative errno value in case of error.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param status: 0 on success, negative errno value in case of error
	 * @param userdata: user data pointer */
	void (*stop_resp)(struct pdraw_backend *pdraw,
			  int status,
			  void *userdata);

	/* Media added callback function, called when a media has been added
	 * internally in the PDrAW pipeline. Medias are for example raw or coded
	 * video medias. The info structure gives the media identifier that can
	 * be used for example to create a video sink on this media.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param info: pointer on the media information
	 * @param element_userdata: optional element user data pointer
	 *                          that corresponds to the pipeline
	 *                          element that created the media
	 * @param userdata: user data pointer */
	void (*media_added)(struct pdraw_backend *pdraw,
			    const struct pdraw_media_info *info,
			    void *element_userdata,
			    void *userdata);

	/* Media removed callback function, called when a media has been removed
	 * internally from the PDrAW pipeline. Medias are for example raw or
	 * coded video medias. When a media is removed, any video sink created
	 * on this media must then be stopped.
	 * @warning when this function is called the pipeline element that
	 * created the media is likely being destroyed, therefore the
	 * element_userdata should not be casted to the pipeline element object.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param info: pointer on the media information
	 * @param element_userdata: optional element user data pointer
	 *                          that corresponds to the pipeline
	 *                          element that created the media
	 * @param userdata: user data pointer */
	void (*media_removed)(struct pdraw_backend *pdraw,
			      const struct pdraw_media_info *info,
			      void *element_userdata,
			      void *userdata);

	/* Socket creation callback function, called immediately after a
	 * socket creation with its file descriptor as parameter (optional,
	 * can be null).
	 * @param pdraw: PDrAW back-end instance handle
	 * @param fd: socket file descriptor
	 * @param userdata: user data pointer */
	void (*socket_created)(struct pdraw_backend *pdraw,
			       int fd,
			       void *userdata);
};


/* Demuxer callback functions */
struct pdraw_backend_demuxer_cbs {
	/* Open response callback function, called when an open operation is
	 * complete or has failed (optional, can be null, but highly recommended
	 * for correct PDrAW session management).
	 * The status parameter is the open operation status: 0 on success,
	 * or a negative errno value in case of error.
	 * If this function reports an error, the demuxer still needs to be
	 * closed: pdraw_be_demuxer_close() must be called and one must wait for
	 * the close_resp callback function to be issued prior to calling
	 * pdraw_be_demuxer_destroy().
	 * @param pdraw: PDrAW back-end instance handle
	 * @param demuxer: demuxer handle
	 * @param status: 0 on success, negative errno value in case of error
	 * @param userdata: user data pointer */
	void (*open_resp)(struct pdraw_backend *pdraw,
			  struct pdraw_demuxer *demuxer,
			  int status,
			  void *userdata);

	/* Close response callback function, called when a close operation is
	 * complete or has failed (optional, can be null, but highly recommended
	 * for correct PDrAW session management).
	 * The status parameter is the close operation status: 0 on success,
	 * or a negative errno value in case of error.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param demuxer: demuxer handle
	 * @param status: 0 on success, negative errno value in case of error
	 * @param userdata: user data pointer */
	void (*close_resp)(struct pdraw_backend *pdraw,
			   struct pdraw_demuxer *demuxer,
			   int status,
			   void *userdata);

	/* Unrecoverable error callback function, called when a previously
	 * opened session is no longer running. When this function is called,
	 * the demuxer is no longer running; pdraw_be_demuxer_close() must be
	 * called and one must wait for the close_resp callback function to be
	 * issued prior to calling pdraw_be_demuxer_destroy().
	 * @param pdraw: PDrAW back-end instance handle
	 * @param demuxer: demuxer handle
	 * @param userdata: user data pointer */
	void (*unrecoverable_error)(struct pdraw_backend *pdraw,
				    struct pdraw_demuxer *demuxer,
				    void *userdata);

	/* Demuxer media selection callback function, called with a list of
	 * video medias found from which the application must choose one or more
	 * to process in the pipeline. The return value of the callback function
	 * must be a bitfield of the identifiers of the chosen medias (from the
	 * pdraw_demuxer_media structure), or 0 to choose the default medias. If
	 * the return value is -ENOSYS, the callback is considered not
	 * implemented and the default medias are chosen. If the return value is
	 * -ECANCELED no media is chosen and the open operation is aborted. If
	 * the return value is another negative errno or an invalid bitfield the
	 * open_resp callback function will be called if an open operation is in
	 * progress, or the unrecoverable_error callback function otherwise.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param demuxer: demuxer handle
	 * @param medias: array of demuxer media
	 * @param count: demuxer media array element count
	 * @param userdata: user data pointer
	 * @return a bitfield of the identifiers of the chosen medias,
	 *         0 or -ENOSYS to choose the default medias,
	 *         -ECANCELED to choose no media and abort the open operation,
	 *         or another negative errno value in case of error */
	int (*select_media)(struct pdraw_backend *pdraw,
			    struct pdraw_demuxer *demuxer,
			    const struct pdraw_demuxer_media *medias,
			    size_t count,
			    void *userdata);

	/* Ready to play callback function, called when the playback is ready
	 * to start (optional, can be null). This function is called to indicate
	 * that the PDrAW session is ready to process play operations.
	 * Generally, the session is ready to play as soon as the open_resp
	 * callback function has been called with a success status. One case
	 * when the open operation was successful and the playback is not ready
	 * is when connected to a SkyController's RTSP server but the
	 * SkyController itself is not yet connected to a drone. Similarly,
	 * when connected to a drone's stream through a SkyController's RTSP
	 * server, if the drone is disconnected from the SkyController, this
	 * function will be called with a 0 value in the ready parameter.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param demuxer: demuxer handle
	 * @param ready: 1 if the session is ready to play, 0 otherwise
	 * @param userdata: user data pointer */
	void (*ready_to_play)(struct pdraw_backend *pdraw,
			      struct pdraw_demuxer *demuxer,
			      int ready,
			      void *userdata);

	/* End of range callback function, called when the playback is suspended
	 * after having reached the end of the playback duration (optional,
	 * can be null). This function is only called for replays (either local
	 * or streamed), not for live streams. The timestamp parameter is the
	 * current play time in microseconds at the moment the playback is
	 * suspended.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param demuxer: demuxer handle
	 * @param timestamp: current playback time in microseconds
	 * @param userdata: user data pointer */
	void (*end_of_range)(struct pdraw_backend *pdraw,
			     struct pdraw_demuxer *demuxer,
			     uint64_t timestamp,
			     void *userdata);

	/* Play response callback function, called when a play operation is
	 * complete (the playback has started) or has failed (optional, can be
	 * null). The status parameter is the play operation status: 0 on
	 * success, or a negative errno value in case of error. The timestamp
	 * parameter is the current play time in microseconds at the moment the
	 * playback is started. The speed parameter is the current playback
	 * speed; a negative value means playing backward.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param demuxer: demuxer handle
	 * @param status: 0 on success, negative errno value in case of error
	 * @param timestamp: current playback time in microseconds
	 * @param speed: current playback speed, negative means backward
	 * @param userdata: user data pointer */
	void (*play_resp)(struct pdraw_backend *pdraw,
			  struct pdraw_demuxer *demuxer,
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
	 * @param pdraw: PDrAW back-end instance handle
	 * @param demuxer: demuxer handle
	 * @param status: 0 on success, negative errno value in case of error
	 * @param timestamp: current playback time in microseconds
	 * @param userdata: user data pointer */
	void (*pause_resp)(struct pdraw_backend *pdraw,
			   struct pdraw_demuxer *demuxer,
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
	 * @param pdraw: PDrAW back-end instance handle
	 * @param demuxer: demuxer handle
	 * @param status: 0 on success, negative errno value in case of error
	 * @param timestamp: current playback time in microseconds
	 * @param speed: current playback speed, negative means backward
	 * @param userdata: user data pointer */
	void (*seek_resp)(struct pdraw_backend *pdraw,
			  struct pdraw_demuxer *demuxer,
			  int status,
			  uint64_t timestamp,
			  float speed,
			  void *userdata);
};


/* Muxer callback functions */
struct pdraw_backend_muxer_cbs {
	/* No space left function, called when the amount of free space on the
	 * storage where the MP4 file written falls below a threshold provided
	 * at the muxer object creation. This function is called from the
	 * pomp_loop thread.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param muxer: muxer handle
	 * @param limit: minimum required free space on the storage to write
	 * the MP4 file (byte)
	 * @param left: current free space on the storage (byte)
	 * @param userdata: user data pointer */
	void (*no_space_left)(struct pdraw_backend *pdraw,
			      struct pdraw_muxer *muxer,
			      size_t limit,
			      size_t left,
			      void *userdata);
};


/* Video renderer callback functions */
struct pdraw_backend_video_renderer_cbs {
	/* Media added callback function, called when a media has been added
	 * internally to the renderer. Medias are raw video medias.
	 * This function is called from the internal pomp_loop thread.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param renderer: renderer handle
	 * @param info: pointer on the media information
	 * @param userdata: user data pointer */
	void (*media_added)(struct pdraw_backend *pdraw,
			    struct pdraw_video_renderer *renderer,
			    const struct pdraw_media_info *info,
			    void *userdata);

	/* Media removed callback function, called when a media has been removed
	 * internally from the renderer. Medias are raw video medias.
	 * This function is called from the internal pomp_loop thread.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param renderer: renderer handle
	 * @param info: pointer on the media information
	 * @param userdata: user data pointer */
	void (*media_removed)(struct pdraw_backend *pdraw,
			      struct pdraw_video_renderer *renderer,
			      const struct pdraw_media_info *info,
			      void *userdata);

	/* Render ready callback function, called both when a new frame is
	 * ready for rendering and periodically (optional, can be null).
	 * This function is called from the internal pomp_loop thread.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param renderer: renderer handle
	 * @param userdata: user data pointer */
	void (*render_ready)(struct pdraw_backend *pdraw,
			     struct pdraw_video_renderer *renderer,
			     void *userdata);

	/* External texture loading callback function (optional, can be null).
	 * This function is called before the rendering of the video frame in
	 * order to override the frame loading as a texture. This can be used
	 * to transform the video frames outside of PDrAW before resuming the
	 * rendering. This function is called from the rendering thread.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param renderer: renderer handle
	 * @param texture_width: texture width in pixels
	 * @param texture_height: texture height in pixels
	 * @param media_info: media information
	 * @param frame: frame information
	 * @param frame_userdata: frame user data buffer
	 * @param frame_userdata_len: frame user data buffer size in bytes
	 * @param userdata: user data pointer
	 * @return 0 on success, negative errno value in case of error */
	int (*load_texture)(struct pdraw_backend *pdraw,
			    struct pdraw_video_renderer *renderer,
			    unsigned int texture_width,
			    unsigned int texture_height,
			    const struct pdraw_media_info *media_info,
			    struct mbuf_raw_video_frame *frame,
			    const void *frame_userdata,
			    size_t frame_userdata_len,
			    void *userdata);

	/* Overlay rendering callback function (optional, can be null).
	 * This function is called after the rendering of the video frame
	 * (if one is available) in order to render an application overlay
	 * on top of the video. When HMD distorsion correction is enabled
	 * in the renderer, it is applied after the overlay rendering.
	 * This function is called from the rendering thread.
	 * When no frame is available for the rendering, the frame_meta
	 * and frame_extra parameters are NULL.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param renderer: renderer handle
	 * @param render_pos: rendering position
	 * @param content_pos: video content position
	 * @param view_mat: 4x4 view matrix
	 * @param proj_mat: 4x4 projection matrix
	 * @param media_info: media information
	 * @param frame_meta: frame metadata (optional, can be NULL)
	 * @param frame_extra: frame extra information (optional, can be NULL)
	 * @param userdata: user data pointer */
	void (*render_overlay)(
		struct pdraw_backend *pdraw,
		struct pdraw_video_renderer *renderer,
		const struct pdraw_rect *render_pos,
		const struct pdraw_rect *content_pos,
		const float *view_mat,
		const float *proj_mat,
		const struct pdraw_media_info *media_info,
		struct vmeta_frame *frame_meta,
		const struct pdraw_video_frame_extra *frame_extra,
		void *userdata);
};


/* Video IPC source callback functions */
struct pdraw_backend_vipc_source_cbs {
	/* Ready to play function, called when the video IPC is ready to
	 * start receiving frames (the ready parameter is 1), or when the
	 * video IPC cannot receive frames any more (end of stream; the ready
	 * parameter is 0). When the ready parameter is 0, the eos_reason
	 * parameter indicates the reason why the end of stream was received
	 * on the video IPC.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param source: video IPC source handle
	 * @param ready: 1 if the session is ready to play, 0 otherwise
	 * @param eos_reason: end of stream reason if the ready parameter is 0
	 * @param userdata: user data pointer */
	void (*ready_to_play)(struct pdraw_backend *pdraw,
			      struct pdraw_vipc_source *source,
			      int ready,
			      enum pdraw_vipc_source_eos_reason eos_reason,
			      void *userdata);

	/* Configured function, called when a video IPC has been
	 * configured (initially or reconfigured) or when a configuration
	 * has failed. The status parameter is the configuration operation
	 * status: 0 on success, or a negative errno value in case of error.
	 * The info and crop parameters are the current video IPC configuration.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param source: video IPC source handle
	 * @param status: 0 on success, negative errno value in case of error
	 * @param info: current video format info
	 * @param crop: current crop configuration
	 * @param userdata: user data pointer */
	void (*configured)(struct pdraw_backend *pdraw,
			   struct pdraw_vipc_source *source,
			   int status,
			   const struct vdef_format_info *info,
			   const struct vdef_rectf *crop,
			   void *userdata);

	/* Frame ready function, called when a video frame has been received
	 * and before it is propagated downstream in the pipeline. This can be
	 * used to associate metadata with the frame.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param source: video IPC source handle
	 * @param frame: frame information
	 * @param userdata: user data pointer */
	void (*frame_ready)(struct pdraw_backend *pdraw,
			    struct pdraw_vipc_source *source,
			    struct mbuf_raw_video_frame *frame,
			    void *userdata);
};


/* Coded video source callback functions */
struct pdraw_backend_coded_video_source_cbs {
	/* Coded video source flushed callback function (mandatory),
	 * called to signal that flushing is complete after the
	 * pdraw_be_coded_video_source_flush() function has been called.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param source: coded video source handle
	 * @param userdata: user data pointer */
	void (*flushed)(struct pdraw_backend *pdraw,
			struct pdraw_coded_video_source *source,
			void *userdata);
};


/* Raw video source callback functions */
struct pdraw_backend_raw_video_source_cbs {
	/* Raw video source flushed callback function (mandatory),
	 * called to signal that flushing is complete after the
	 * pdraw_be_raw_video_source_flush() function has been called.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param source: raw video source handle
	 * @param userdata: user data pointer */
	void (*flushed)(struct pdraw_backend *pdraw,
			struct pdraw_raw_video_source *source,
			void *userdata);
};


/* Coded video sink callback functions */
struct pdraw_backend_coded_video_sink_cbs {
	/* Video sink flush callback function, called when flushing is required
	 * (mandatory). When this function is called, the application must flush
	 * the sink queue by calling mbuf_coded_video_frame_queue_flush() and
	 * must return all frames outside of the queue by calling
	 * mbuf_coded_video_frame_unref(); once the flushing is done, the
	 * pdraw_be_coded_video_sink_queue_flushed() function must be called.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param sink: video sink handle
	 * @param userdata: user data pointer */
	void (*flush)(struct pdraw_backend *pdraw,
		      struct pdraw_coded_video_sink *sink,
		      void *userdata);
};


/* Video sink callback functions */
struct pdraw_backend_raw_video_sink_cbs {
	/* Video sink flush callback function, called when flushing is required
	 * (mandatory). When this function is called, the application must flush
	 * the sink queue by calling mbuf_raw_video_frame_queue_flush() and
	 * must return all frames outside of the queue by calling
	 * mbuf_raw_video_frame_unref(); once the flushing is done, the
	 * pdraw_be_raw_video_sink_queue_flushed() function must be called.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param sink: video sink handle
	 * @param userdata: user data pointer */
	void (*flush)(struct pdraw_backend *pdraw,
		      struct pdraw_raw_video_sink *sink,
		      void *userdata);
};


/* Video encoder callback functions */
struct pdraw_backend_video_encoder_cbs {
	/* Frame output function, called when a video frame is output from the
	 * encoder. This can be used to extract frame info and ancillary data
	 * from the frame. This function is usually called from the pomp_loop
	 * thread, but it may depend on the video encoder implementation.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param encoder: video encoder handle
	 * @param frame: frame information
	 * @param userdata: user data pointer */
	void (*frame_output)(struct pdraw_backend *pdraw,
			     struct pdraw_video_encoder *encoder,
			     struct mbuf_coded_video_frame *frame,
			     void *userdata);

	/* Frame pre-release function, called before a video frame is released.
	 * This can be used to extract frame info and ancillary data from the
	 * frame. This function is called from the thread that unrefs the frame;
	 * it can be any thread.
	 * @param pdraw: PDrAW back-end instance handle
	 * @param encoder: video encoder handle
	 * @param frame: frame information
	 * @param userdata: user data pointer */
	void (*frame_pre_release)(struct pdraw_backend *pdraw,
				  struct pdraw_video_encoder *encoder,
				  struct mbuf_coded_video_frame *frame,
				  void *userdata);
};


/**
 * Instance management API
 */

/**
 * Create a PDrAW back-end instance.
 * All API functions can be called from any thread, with the exception of
 * rendering functions which must still be called from the rendering thread.
 * All callback functions with the exception of video renderer callback
 * functions are called from the internal pomp_loop thread; synchronization
 * is up to the application.
 * The callbacks structure must be provided but all callback functions are
 * optional. However, for correct management of the session it is highly
 * recommended to implement at least the open_resp and close_resp functions.
 * The instance handle is returned through the ret_obj parameter. When no
 * longer needed, the instance must be freed using the pdraw_be_destroy()
 * function. The pdraw_be_stop() function must be called and one must wait
 * for the stop_resp callback function to be issued prior to calling
 * pdraw_be_destroy().
 * @param cbs: back-end instance callback functions
 * @param userdata: callback functions user data pointer
 * @param ret_obj: PDrAW back-end instance handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_new(const struct pdraw_backend_cbs *cbs,
				   void *userdata,
				   struct pdraw_backend **ret_obj);


/**
 * Free a PDrAW back-end instance.
 * This function frees all resources associated with a back-end instance.
 * If a successful pdraw_be_open_*() was made, the pdraw_be_close() function
 * must be called and one must wait for the close_resp callback function to
 * be issued prior to calling pdraw_be_destroy().
 * @param self: PDrAW back-end instance handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_destroy(struct pdraw_backend *self);


/**
 * Stop a PDrAW back-end instance.
 * This function stops a PDrAW back-end instance and all the associated objects.
 * The function returns before the actual stopping is done. If the function
 * returns 0, the stop_resp callback function will be called once the stopping
 * is successful (0 status) or has failed (negative errno status). If the
 * function returns a negative errno value (immediate failure), the stop_resp
 * callback function will not be called. After a successful stop, the PDrAW
 * instance must be destroyed by calling pdraw_destroy().
 * @param self: PDrAW back-end instance handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_stop(struct pdraw_backend *self);


/**
 * Get the PDrAW back-end internal event loop.
 * This function returns a pointer to the event loop used internally.
 * @param self: PDrAW back-end instance handle
 * @return a pointer on the internal loop on success, NULL in case of error
 */
PDRAW_BACKEND_API struct pomp_loop *
pdraw_be_get_loop(struct pdraw_backend *self);


/**
 * Demuxer API
 */

/**
 * Create a demuxer on a URL (stream or local file).
 * The URL can be either an RTSP URL (starting with "rtsp://") or a local file
 * path (either absolute or relative).
 * The function returns before the actual opening is done. If the function
 * returns 0, the open_resp callback function will be issued once the open
 * operation is successful (0 status) or has failed (negative errno status).
 * If the function returns a negative errno value (immediate failure), the
 * open_resp callback function will not be issued.
 * Once a demuxer is no longer used, it must be closed and then destroyed
 * (@see the pdraw_be_demuxer_close() function).
 * @param pdraw: PDrAW back-end instance handle
 * @param url: URL of the resource to open
 * @param cbs: demuxer callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: demuxer handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_demuxer_new_from_url(struct pdraw_backend *self,
			      const char *url,
			      const struct pdraw_backend_demuxer_cbs *cbs,
			      void *userdata,
			      struct pdraw_demuxer **ret_obj);


/**
 * Create a demuxer on a single stream.
 * This function opens an RTP/AVP stream. No session management is done: it
 * is the application's responsibility to handle the ports negociation with
 * the sender. If null local ports are given as parameter, the effective port
 * numbers used can be retrieved using the
 * pdraw_be_get_single_stream_local_stream_port() for the RTP port and
 * pdraw_be_get_single_stream_local_control_port() for the RTCP functions.
 * If the local_addr parameter is left null, any local network interface will
 * be used. The remote_addr, remote_stream_port and remote_control_port
 * parameters can be left null if unknown; they will be known once the stream
 * is being received.
 * The function returns before the actual opening is done. If the function
 * returns 0, the open_resp callback function will be issued once the open
 * operation is successful (0 status) or has failed (negative errno status).
 * If the function returns a negative errno value (immediate failure), the
 * open_resp callback function will not be issued.
 * Once a demuxer is no longer used, it must be closed and then destroyed
 * (@see the pdraw_be_demuxer_close() function).
 * @param pdraw: PDrAW back-end instance handle
 * @param local_addr: local IP address (optional, can be NULL)
 * @param local_stream_port: local stream (RTP) port (optional, can be 0)
 * @param local_control_port: local control (RTCP) port (optional, can be 0)
 * @param remote_addr: remote IP address (optional, can be NULL)
 * @param remote_stream_port: remote stream (RTP) port (optional, can be 0)
 * @param remote_control_port: remote control (RTCP) port (optional, can be 0)
 * @param cbs: demuxer callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: demuxer handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_demuxer_new_single_stream(struct pdraw_backend *self,
				   const char *local_addr,
				   uint16_t local_stream_port,
				   uint16_t local_control_port,
				   const char *remote_addr,
				   uint16_t remote_stream_port,
				   uint16_t remote_control_port,
				   const struct pdraw_backend_demuxer_cbs *cbs,
				   void *userdata,
				   struct pdraw_demuxer **ret_obj);


/**
 * Create a demuxer on a stream URL through a mux channel.
 * The URL must be an RTSP URL (starting with "rtsp://"). Mux channels are used
 * to transfer data between a SkyController remote and a smartphone through USB;
 * see Parrot's libmux for more information.
 * No concurrent sessions can run on the mux channel; therefore the user must
 * take care of limiting the number of PDrAW instances and demuxer objects
 * running on the mux channel to only one.
 * The function returns before the actual opening is done. If the function
 * returns 0, the open_resp callback function will be issued once the open
 * operation is successful (0 status) or has failed (negative errno status).
 * If the function returns a negative errno value (immediate failure), the
 * open_resp callback function will not be issued.
 * Once a demuxer is no longer used, it must be closed and then destroyed
 * (@see the pdraw_be_demuxer_close() function).
 * @param pdraw: PDrAW back-end instance handle
 * @param url: URL of the resource to open
 * @param mux: mux instance handle
 * @param cbs: demuxer callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: demuxer handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_demuxer_new_from_url_on_mux(
	struct pdraw_backend *self,
	const char *url,
	struct mux_ctx *mux,
	const struct pdraw_backend_demuxer_cbs *cbs,
	void *userdata,
	struct pdraw_demuxer **ret_obj);


/**
 * Destroy a demuxer.
 * This function stops a running demuxer and frees the associated resources.
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_demuxer_destroy(struct pdraw_backend *self,
					       struct pdraw_demuxer *demuxer);


/**
 * Close a demuxer.
 * This function closes a previously opened demuxer (either record or stream).
 * The function returns before the actual closing is done. If the function
 * returns 0, the close_resp callback function will be issued once the close is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the close_resp callback
 * function will not be issued. After a successful close, the demuxer must be
 * destroyed by calling pdraw_be_demuxer_destroy().
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_demuxer_close(struct pdraw_backend *self,
					     struct pdraw_demuxer *demuxer);


/**
 * Get the single stream local stream port.
 * This function returns the local stream (RTP) port currently in use after
 * a succesful open operation on a single stream (RTP/AVP) or an RTSP URL.
 * If no open operation has been done, or if an open operation has been done
 * on a mux channel, 0 is returned.
 * This is useful when calling pdraw_be_open_single_stream() with null local
 * ports to let PDrAW open sockets on any available port.
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @return the stream port on success, 0 in case of error
 */
PDRAW_BACKEND_API uint16_t pdraw_be_demuxer_get_single_stream_local_stream_port(
	struct pdraw_backend *self,
	struct pdraw_demuxer *demuxer);


/**
 * Get the single stream local control port.
 * This function returns the local control (RTCP) port currently in use after
 * a succesful open operation on a single stream (RTP/AVP) or an RTSP URL.
 * If no open operation has been done, or if an open operation has been done
 * on a mux channel, 0 is returned.
 * This is useful when calling pdraw_be_open_single_stream() with null local
 * ports to let PDrAW open sockets on any available port.
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @return the stream port on success, 0 in case of error
 */
PDRAW_BACKEND_API uint16_t
pdraw_be_demuxer_get_single_stream_local_control_port(
	struct pdraw_backend *self,
	struct pdraw_demuxer *demuxer);


/**
 * Get the ready to play status.
 * This function returns 1 if a successful open operation has been completed
 * and if the playback is ready to start, 0 otherwise. One case when a
 * successful open operation is complete but the playback is not ready is when
 * connected to a SkyController's RTSP server but the SkyController itself is
 * not yet connected to a drone.
 * The value returned by this function is identical to the ready parameter
 * passed to the ready_to_play callback function when it is issued.
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @return the ready to play status on success, 0 in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_demuxer_is_ready_to_play(struct pdraw_backend *self,
				  struct pdraw_demuxer *demuxer);


/**
 * Get the pause status.
 * This function returns 1 if the playback is currently paused, 0 otherwise.
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @return the pause status on success, 0 in case of error
 */
PDRAW_BACKEND_API int pdraw_be_demuxer_is_paused(struct pdraw_backend *self,
						 struct pdraw_demuxer *demuxer);


/**
 * Play at normal speed (x1.0).
 * This function starts the playback of the video.
 * The function returns before the actual operation is done. If the function
 * returns 0, the play_resp callback function will be issued once the play is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the play_resp callback
 * function will not be issued.
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_demuxer_play(struct pdraw_backend *self,
					    struct pdraw_demuxer *demuxer);


/**
 * Play at the given speed.
 * This function starts the playback of the video. If the speed parameter is
 * negative, the video is played backward. If the speed is greater than or
 * equal to PDRAW_PLAY_SPEED_MAX, the speed is ignored and the video is played
 * at the maximum speed achievable. If the speed is less than or equal to
 * -PDRAW_PLAY_SPEED_MAX, the speed is ignored and the video is played backward
 * at the maximum speed achievable. A 0.0 speed has the same effet as calling
 * the pdraw_be_pause() function. On a live stream, the speed parameter has
 * no effect.
 * The function returns before the actual operation is done. If the function
 * returns 0, the play_resp callback function will be issued once the play is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the play_resp callback
 * function will not be issued.
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @param speed: playback speed (0.0 means pause, negative value means
 *               play backward)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_demuxer_play_with_speed(struct pdraw_backend *self,
				 struct pdraw_demuxer *demuxer,
				 float speed);


/**
 * Pause the playback.
 * This function suspends the playback of the video. The session is not closed
 * and the playback can be resumed using the play functions.
 * The function returns before the actual operation is done. If the function
 * returns 0, the pause_resp callback function will be issued once the pause is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the pause_resp callback
 * function will not be issued.
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_demuxer_pause(struct pdraw_backend *self,
					     struct pdraw_demuxer *demuxer);


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
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_demuxer_previous_frame(struct pdraw_backend *self,
				struct pdraw_demuxer *demuxer);


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
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_demuxer_next_frame(struct pdraw_backend *self,
			    struct pdraw_demuxer *demuxer);


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
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @param delta: time delta in microseconds (positive or negative)
 * @param exact: 1 means seek to the sample closest to the delta, 0 means seek
 *               to the nearest synchronization sample preceeding the delta
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_demuxer_seek(struct pdraw_backend *self,
					    struct pdraw_demuxer *demuxer,
					    int64_t delta,
					    int exact);


/**
 * Seek forward.
 * This function seeks forward by delta microseconds. It has the same
 * behavior as calling pdraw_be_seek() with a positive delta. The exact
 * parameter is a boolean value; when exact is 0 the seek is done to the
 * nearest synchronization sample preceeding the delta, otherwise the seek
 * is done to the sample nearest to the delta. Seeking is only available on
 * replays (either local or streamed), not on live streams.
 * The function returns before the actual operation is done. If the function
 * returns 0, the seek_resp callback function will be issued once the seek is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the seek_resp callback
 * function will not be issued.
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @param delta: positive time delta forward in microseconds
 * @param exact: 1 means seek to the sample closest to the delta, 0 means seek
 *               to the nearest synchronization sample preceeding the delta
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_demuxer_seek_forward(struct pdraw_backend *self,
			      struct pdraw_demuxer *demuxer,
			      uint64_t delta,
			      int exact);


/**
 * Seek backward.
 * This function seeks backward by delta microseconds (postive). It has the same
 * behavior as calling pdraw_be_seek() with a negative delta. The exact
 * parameter is a boolean value; when exact is 0 the seek is done to the nearest
 * synchronization sample preceeding the delta, otherwise the seek is done to
 * the sample nearest to the delta. Seeking is only available on replays
 * (either local or streamed), not on live streams.
 * The function returns before the actual operation is done. If the function
 * returns 0, the seek_resp callback function will be issued once the seek is
 * successful (0 status) or has failed (negative errno status). If the function
 * returns a negative errno value (immediate failure), the seek_resp callback
 * function will not be issued.
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @param delta: positive time delta backward in microseconds
 * @param exact: 1 means seek to the sample closest to the delta, 0 means seek
 *               to the nearest synchronization sample preceeding the delta
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_demuxer_seek_back(struct pdraw_backend *self,
						 struct pdraw_demuxer *demuxer,
						 uint64_t delta,
						 int exact);


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
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @param timestamp: play timestamp in microseconds
 * @param exact: 1 means seek to the sample closest to the timestamp,
 *               0 means seek to the nearest synchronization sample
 *               preceeding the timestamp
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_demuxer_seek_to(struct pdraw_backend *self,
					       struct pdraw_demuxer *demuxer,
					       uint64_t timestamp,
					       int exact);


/**
 * Get the playback duration.
 * This function returns the playback duration in microseconds. The duration is
 * only available on replays (either local or streamed), not on live streams.
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @return the duration in microseconds on success, 0 in case of error
 */
PDRAW_BACKEND_API uint64_t
pdraw_be_demuxer_get_duration(struct pdraw_backend *self,
			      struct pdraw_demuxer *demuxer);


/**
 * Get the playback current time.
 * This function returns the current playback position in microseconds.
 * On replays (either local or streamed) this is the position between 0 and
 * the duration; on live streams this is the time since the start of the
 * stream session.
 * @param pdraw: PDrAW back-end instance handle
 * @param demuxer: demuxer handle
 * @return the current time in microseconds on success, 0 in case of error
 */
PDRAW_BACKEND_API uint64_t
pdraw_be_demuxer_get_current_time(struct pdraw_backend *self,
				  struct pdraw_demuxer *demuxer);


/**
 * Muxer API
 */

/**
 * Create a muxer.
 * This function creates a muxer with a given URL. The url parameter is a path
 * to an MP4 file (the file will be created/overwritten).
 * Once the muxer is created medias can be added by id using the
 * pdraw_be_muxer_add_media() function. Once a muxer is no longer used it must
 * be destroyed by calling the pdraw_be_muxer_destroy() function. If writing to
 * an MP4 file, the file is finalized in the pdraw_be_muxer_destroy() function.
 * The params structure must be provided but all parameters are optional and can
 * be left null. The callbacks structure must be provided; all callback
 * functions are called from the pomp_loop thread.
 * @param pdraw: PDrAW back-end instance handle
 * @param url: destination URL
 * @param params: muxer parameters
 * @param cbs: muxer callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: muxer handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_muxer_new(struct pdraw_backend *self,
		   const char *url,
		   const struct pdraw_muxer_params *params,
		   const struct pdraw_backend_muxer_cbs *cbs,
		   void *userdata,
		   struct pdraw_muxer **ret_obj);


/**
 * Destroy a muxer.
 * This function stops a running muxer and frees the associated resources.
 * @param pdraw: PDrAW back-end instance handle
 * @param muxer: muxer handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_muxer_destroy(struct pdraw_backend *self,
					     struct pdraw_muxer *muxer);


/**
 * Add a media to a muxer.
 * This function adds a media to the muxer by its media_id. The media
 * idenfifiers are known when the media_added or media_removed general
 * callback functions are called.
 * The params structure is only relevant for video medias; the structure must
 * then be provided but all parameters are optional and can be left null.
 * @param pdraw: PDrAW back-end instance handle
 * @param muxer: muxer handle
 * @param media_id: identifier of the media to add to the muxer
 * @param params: muxer video media parameters
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_muxer_add_media(struct pdraw_backend *self,
			 struct pdraw_muxer *muxer,
			 unsigned int media_id,
			 const struct pdraw_muxer_video_media_params *params);


/**
 * Set the thumbnail of the MP4 file written by the muxer.
 * This function is available on a record muxer only; on another type of muxer
 * -ENOSYS is returned
 * @param pdraw: PDrAW back-end instance handle
 * @param muxer: muxer handle
 * @param type: type of the thumbnail (JPEG, PNG, etc...)
 * @param data: thumbnail data, must be of length size
 * @param size: size of data
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API
int pdraw_be_muxer_set_thumbnail(struct pdraw_backend *pdraw,
				 struct pdraw_muxer *muxer,
				 enum pdraw_muxer_thumbnail_type type,
				 const uint8_t *data,
				 size_t size);


/**
 * Video renderer API
 * @warning: all functions must be called from the application's
 * rendering thread
 */

/**
 * Create a video renderer.
 * This function creates a video renderer on a media of the given media id;
 * if the media id is zero the first raw media encountered is used. Once
 * the renderer is created, the rendering is done by calling the
 * pdraw_video_renderer_render*() functions. Once a renderer is no longer used
 * it must be destroyed by calling the pdraw_be_video_renderer_destroy()
 * function. The render_pos parameter sets the position and size of the
 * rendering in the window/view; these coordinates are in pixels from the
 * bottom-left corner (OpenGL coordinates). The params structure must be
 * provided but all parameters are optional and can be left null. The callbacks
 * structure must be provided but all callback functions are optional; all
 * callback functions are called from the rendering thread, except the
 * render_ready function which is called from the internal pomp_loop thread.
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW back-end instance handle
 * @param media_id: identifier of the raw media to render (from a
 *                  pdraw_media_info structure); if zero the first
 *                  raw media found is used for rendering
 * @param render_pos: rendering position and size
 * @param params: renderer parameters
 * @param cbs: renderer callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: renderer handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_video_renderer_new(struct pdraw_backend *self,
			    unsigned int media_id,
			    const struct pdraw_rect *render_pos,
			    const struct pdraw_video_renderer_params *params,
			    const struct pdraw_backend_video_renderer_cbs *cbs,
			    void *userdata,
			    struct pdraw_video_renderer **ret_obj);


/**
 * Create a video renderer on an EGL display.
 * This function creates a video renderer on an active EGL display context on a
 * media of the given media id; if the media id is zero the first raw
 * media encountered is used. Once the renderer is created, the rendering is
 * done by calling the pdraw_video_renderer_render*() functions. Once a renderer
 * is no longer used it must be destroyed by calling the
 * pdraw_be_video_renderer_destroy() function. The render_pos parameter sets the
 * position and size of the rendering in the window/view; these coordinates are
 * in pixels from the bottom-left corner (OpenGL coordinates). The params
 * structure must be provided but all parameters are optional and can be left
 * null. The callbacks structure must be provided but all callback functions are
 * optional; all callback functions are called from the rendering thread,
 * except the render_ready function which is called from the internal pomp_loop
 * thread.
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW back-end instance handle
 * @param media_id: identifier of the raw media to render (from a
 *                  pdraw_media_info structure); if zero the first
 *                  raw media found is used for rendering
 * @param render_pos: rendering position and size
 * @param params: renderer parameters
 * @param cbs: renderer callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param egl_display: EGL display context
 * @param ret_obj: renderer handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_video_renderer_new_egl(
	struct pdraw_backend *self,
	unsigned int media_id,
	const struct pdraw_rect *render_pos,
	const struct pdraw_video_renderer_params *params,
	const struct pdraw_backend_video_renderer_cbs *cbs,
	void *userdata,
	struct egl_display *egl_display,
	struct pdraw_video_renderer **ret_obj);


/**
 * Stop a video renderer.
 * This function stops a running video renderer and frees the associated
 * resources.
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW back-end instance handle
 * @param renderer: renderer handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_video_renderer_destroy(struct pdraw_backend *self,
				struct pdraw_video_renderer *renderer);


/**
 * Resize a video renderer.
 * This function updates the rendering position on a running video renderer.
 * The render_pos parameter sets the position and size of the rendering in the
 * window/view; these coordinates are in pixels from the bottom-left corner
 * (OpenGL coordinates).
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW back-end instance handle
 * @param renderer: renderer handle
 * @param render_pos: rendering position and size
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_video_renderer_resize(struct pdraw_backend *self,
			       struct pdraw_video_renderer *renderer,
			       const struct pdraw_rect *render_pos);


/**
 * Set the video renderer media identifier.
 * This function updates the identifier of the media on which the rendering is
 * done; if the media id is zero the first raw media encountered is used.
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW back-end instance handle
 * @param renderer: renderer handle
 * @param media_id: identifier of the raw media to render (from a
 *                  pdraw_media_info structure); if zero the first raw media
 *                  found is used for rendering
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_video_renderer_set_media_id(struct pdraw_backend *self,
				     struct pdraw_video_renderer *renderer,
				     unsigned int media_id);


/**
 * Get the video renderer media identifier.
 * This function retrieves the identifier of the media on which the rendering
 * is done.
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW back-end instance handle
 * @param renderer: renderer handle
 * @return the identifier of the media on success, 0 if no media is being
 *         renderered or in case of error
 */
PDRAW_BACKEND_API unsigned int
pdraw_be_video_renderer_get_media_id(struct pdraw_backend *self,
				     struct pdraw_video_renderer *renderer);


/**
 * Set the video renderer parameters.
 * This function updates the rendering parameters on a running video renderer.
 * The params structure must be provided but all parameters are optional and
 * can be left null.
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW back-end instance handle
 * @param renderer: renderer handle
 * @param params: renderer parameters
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_video_renderer_set_params(
	struct pdraw_backend *self,
	struct pdraw_video_renderer *renderer,
	const struct pdraw_video_renderer_params *params);


/**
 * Get the video renderer parameters.
 * This function retrieves the rendering parameters on a running video renderer.
 * The provided params structure is filled by the function.
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW back-end instance handle
 * @param renderer: renderer handle
 * @param params: renderer parameters (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_video_renderer_get_params(struct pdraw_backend *self,
				   struct pdraw_video_renderer *renderer,
				   struct pdraw_video_renderer_params *params);


/**
 * Render the video.
 * This function renders the video with the current rendering position and
 * rendering parameters.
 * For render-on-demand, consider using the render_ready callback function
 * which is called both when a new frame is ready for rendering and
 * periodically. Note that the render_ready callback function is called from
 * the internal pomp_loop thread, not the rendering thread; the synchronization
 * is up to the caller.
 * If a content_pos structure is provided, it is filled with the actual position
 * and size of the video within the rendering position; these coordinates are
 * in pixels from the bottom-left corner (OpenGL coordinates).
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW back-end instance handle
 * @param renderer: renderer handle
 * @param content_pos: video content position (output; optional, can be null)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_video_renderer_render(struct pdraw_backend *self,
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
 * the internal pomp_loop thread, not the rendering thread; the synchronization
 * is up to the caller.
 * If a content_pos structure is provided, it is filled with the actual position
 * and size of the video within the rendering position; these coordinates are
 * in pixels from the bottom-left corner (OpenGL coordinates).
 * @warning: this function must be called from the application's rendering
 * thread.
 * @param pdraw: PDrAW back-end instance handle
 * @param renderer: renderer handle
 * @param content_pos: video content position (output; optional, can be null)
 * @param view_mat: 4x4 view matrix
 * @param proj_mat: 4x4 projection matrix
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_video_renderer_render_mat(struct pdraw_backend *self,
				   struct pdraw_video_renderer *renderer,
				   struct pdraw_rect *content_pos,
				   const float *view_mat,
				   const float *proj_mat);


/**
 * Video IPC source API
 */

/**
 * Create a video IPC source.
 * This function creates a video IPC source. Once a video source is no longer
 * used, it must be destroyed by calling the pdraw_be_vipc_source_destroy()
 * function. The video IPC address must be provided. The callbacks structure
 * must be provided; all callback functions are called from the pomp_loop
 * thread.
 * @param self: PDrAW back-end instance handle
 * @param params: video IPC source parameters
 * @param cbs: video IPC source callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: video IPC source handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_vipc_source_new(struct pdraw_backend *self,
			 const struct pdraw_vipc_source_params *params,
			 const struct pdraw_backend_vipc_source_cbs *cbs,
			 void *userdata,
			 struct pdraw_vipc_source **ret_obj);


/**
 * Destroy a video IPC source.
 * This function stops a running video IPC source and frees the associated
 * resources.
 * @param self: PDrAW back-end instance handle
 * @param source: video IPC source handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_vipc_source_destroy(struct pdraw_backend *self,
			     struct pdraw_vipc_source *source);


/**
 * Get the ready to play status.
 * This function returns 1 if the video IPC is ready to start, 0 otherwise.
 * The value returned by this function is identical to the ready parameter
 * passed to the ready_to_play() video IPC source callback function when it
 * is called.
 * @param self: PDrAW back-end instance handle
 * @param source: video IPC source handle
 * @return the ready to play status on success, 0 in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_vipc_source_is_ready_to_play(struct pdraw_backend *self,
				      struct pdraw_vipc_source *source);


/**
 * Get the pause status.
 * This function returns 1 if the video IPC is currently paused, 0 otherwise.
 * @param self: PDrAW back-end instance handle
 * @param source: video IPC source handle
 * @return the pause status on success, 0 in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_vipc_source_is_paused(struct pdraw_backend *self,
			       struct pdraw_vipc_source *source);


/**
 * Start receiving frames on the video IPC.
 * This function starts the video IPC if it is ready to play. Otherwise
 * an error is returned. Receiving frames can be halted by calling the
 * pdraw_be_vipc_source_pause() function.
 * @param self: PDrAW back-end instance handle
 * @param source: video IPC source handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_vipc_source_play(struct pdraw_backend *self,
			  struct pdraw_vipc_source *source);


/**
 * Stop receiving frames on the video IPC.
 * This function halts the video IPC to stop receiving frames. Receiving
 * frames can be resumed by calling the pdraw_be_vipc_source_play() function.
 * @param self: PDrAW back-end instance handle
 * @param source: video IPC source handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_vipc_source_pause(struct pdraw_backend *self,
			   struct pdraw_vipc_source *source);


/**
 * Configure the video IPC.
 * This function can be used to dynamically reconfigure a video IPC. The
 * resolution and crop can be specified; if either parameter is NULL or
 * values are 0, it is ignored.
 * The function returns before the actual operation is done. If the function
 * returns 0, the configured() video IPC source callback function will be
 * called once the configuration is successful (0 status) or has failed
 * (negative errno status). If the function returns a negative errno value
 * (immediate failure), the configured() callback function will not be called.
 * @param self: PDrAW back-end instance handle
 * @param source: video IPC source handle
 * @param resolution: new video IPC resolution to apply (optional, can be NULL)
 * @param crop: new video IPC crop to apply (optional, can be NULL)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_vipc_source_configure(struct pdraw_backend *self,
			       struct pdraw_vipc_source *source,
			       const struct vdef_dim *resolution,
			       const struct vdef_rectf *crop);


/**
 * Set the session metadata of the video IPC source.
 * This function updates the session metadata on a running video IPC source, and
 * propgates this structure to all elements downstream in the pipeline. The
 * structure is copied internally and ownership stays with the caller.
 * @param self: PDrAW back-end instance handle
 * @param source: video IPC source handle
 * @param meta: new session metadata
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_vipc_source_set_session_metadata(struct pdraw_backend *self,
					  struct pdraw_vipc_source *source,
					  const struct vmeta_session *meta);


/**
 * Get the session metadata of the video IPC source.
 * This function retrieves the session metadata on a running video IPC source.
 * The provided sessionMeta structure is filled by the function.
 * @param self: PDrAW back-end instance handle
 * @param source: video IPC source handle
 * @param meta: session metadata (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_vipc_source_get_session_metadata(struct pdraw_backend *self,
					  struct pdraw_vipc_source *source,
					  struct vmeta_session *meta);


/**
 * Video source API
 */

/**
 * Create a coded video source.
 * This function creates a coded video source to push frames to. Once the
 * source is created, video frames are to be pushed into the frame queue
 * returned by the pdraw_be_coded_video_source_get_queue() function.
 * Once a video source is no longer used, it must be destroyed by calling the
 * pdraw_be_coded_video_source_destroy() function.
 * The params structure must be provided and must be filled.
 * The callbacks structure must be provided and the flushed callback function
 * is required to be implemented; all callback functions are called from the
 * pomp_loop thread.
 * @param pdraw: PDrAW back-end instance handle
 * @param params: video source parameters
 * @param cbs: video source callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: video source handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_coded_video_source_new(
	struct pdraw_backend *self,
	const struct pdraw_video_source_params *params,
	const struct pdraw_backend_coded_video_source_cbs *cbs,
	void *userdata,
	struct pdraw_coded_video_source **ret_obj);


/**
 * Destroy a coded video source.
 * This function stops a running video source and frees the associated
 * resources. Once a video source is destroyed the queue returned by
 * pdraw_be_coded_video_source_get_queue() must no longer be used.
 * @param pdraw: PDrAW back-end instance handle
 * @param source: video source handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_coded_video_source_destroy(struct pdraw_backend *self,
				    struct pdraw_coded_video_source *source);


/**
 * Get the coded video source frame queue.
 * This function returns the frame queue to use in order to push frames to a
 * running coded video source. Frames are pushed into the queue by using the
 * mbuf_coded_video_frame_queue_push() function.
 * @param pdraw: PDrAW back-end instance handle
 * @param source: video source handle
 * @return a pointer on a mbuf_coded_video_frame_queue object on success, NULL
 * in case of error
 */
PDRAW_BACKEND_API struct mbuf_coded_video_frame_queue *
pdraw_be_coded_video_source_get_queue(struct pdraw_backend *self,
				      struct pdraw_coded_video_source *source);


/**
 * Coded video source flush function.
 * This function is to be called when flushing is required. When this function
 * is called, all frames previously pushed to the queue will be returned; once
 * the flushing is done, the flushed() coded video source callback function
 * will be called.
 * @param pdraw: PDrAW back-end instance handle
 * @param source: video source handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_coded_video_source_flush(struct pdraw_backend *self,
				  struct pdraw_coded_video_source *source);


/**
 * Set the session metadata of the coded video source.
 * This function updates the session metadata on a running coded video source,
 * and propgates this structure to all elements downstream in the pipeline. The
 * structure is copied internally and ownership stays with the caller.
 * @param self: PDrAW back-end instance handle
 * @param source: coded video source handle
 * @param meta: new session metadata
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_be_coded_video_source_set_session_metadata(
	struct pdraw_backend *self,
	struct pdraw_coded_video_source *source,
	const struct vmeta_session *meta);


/**
 * Get the session metadata of the coded video source.
 * This function retrieves the session metadata on a running coded video source.
 * The provided meta structure is filled by the function.
 * @param self: PDrAW back-end instance handle
 * @param source: coded video source handle
 * @param meta: session metadata (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_be_coded_video_source_get_session_metadata(
	struct pdraw_backend *self,
	struct pdraw_coded_video_source *source,
	struct vmeta_session *meta);


/**
 * Create a raw video source.
 * This function creates a raw video source to push frames to. Once the
 * source is created, video frames are to be pushed into the frame queue
 * returned by the pdraw_be_raw_video_source_get_queue() function.
 * Once a video source is no longer used, it must be destroyed by calling the
 * pdraw_be_raw_video_source_destroy() function.
 * The params structure must be provided and must be filled.
 * The callbacks structure must be provided and the flushed callback function
 * is required to be implemented; all callback functions are called from the
 * pomp_loop thread.
 * @param pdraw: PDrAW back-end instance handle
 * @param params: video source parameters
 * @param cbs: video source callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: video source handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_raw_video_source_new(
	struct pdraw_backend *self,
	const struct pdraw_video_source_params *params,
	const struct pdraw_backend_raw_video_source_cbs *cbs,
	void *userdata,
	struct pdraw_raw_video_source **ret_obj);


/**
 * Destroy a raw video source.
 * This function stops a running video source and frees the associated
 * resources. Once a video source is destroyed the queue returned by
 * pdraw_be_raw_video_source_get_queue() must no longer be used.
 * @param pdraw: PDrAW back-end instance handle
 * @param source: video source handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_raw_video_source_destroy(struct pdraw_backend *self,
				  struct pdraw_raw_video_source *source);


/**
 * Get the raw video source frame queue.
 * This function returns the frame queue to use in order to push frames to a
 * running raw video source. Frames are pushed into the queue by using the
 * mbuf_raw_video_frame_queue_push() function.
 * @param pdraw: PDrAW back-end instance handle
 * @param source: video source handle
 * @return a pointer on a mbuf_raw_video_frame_queue object on success, NULL
 * in case of error
 */
PDRAW_BACKEND_API struct mbuf_raw_video_frame_queue *
pdraw_be_raw_video_source_get_queue(struct pdraw_backend *self,
				    struct pdraw_raw_video_source *source);


/**
 * Raw video source flush function.
 * This function is to be called when flushing is required. When this function
 * is called, all frames previously pushed to the queue will be returned; once
 * the flushing is done, the flushed() raw video source callback function
 * will be called.
 * @param pdraw: PDrAW back-end instance handle
 * @param source: video source handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_raw_video_source_flush(struct pdraw_backend *self,
				struct pdraw_raw_video_source *source);


/**
 * Set the session metadata of the raw video source.
 * This function updates the session metadata on a running raw video source,
 * and propgates this structure to all elements downstream in the pipeline. The
 * structure is copied internally and ownership stays with the caller.
 * @param self: PDrAW back-end instance handle
 * @param source: raw video source handle
 * @param meta: new session metadata
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_be_raw_video_source_set_session_metadata(
	struct pdraw_backend *self,
	struct pdraw_raw_video_source *source,
	const struct vmeta_session *meta);


/**
 * Get the session metadata of the raw video source.
 * This function retrieves the session metadata on a running raw video source.
 * The provided meta structure is filled by the function.
 * @param self: PDrAW back-end instance handle
 * @param source: raw video source handle
 * @param meta: session metadata (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_API int pdraw_be_raw_video_source_get_session_metadata(
	struct pdraw_backend *self,
	struct pdraw_raw_video_source *source,
	struct vmeta_session *meta);


/**
 * Video sink API
 */

/**
 * Create a coded video sink.
 * This function creates a video sink on a media of the given media_id.
 * The media idenfifiers are known when the media_added or media_removed
 * general callback functions are called.
 * Once the sink is created, video frames are retrieved by getting buffers
 * from the buffer queue returned by the pdraw_be_coded_video_sink_get_queue()
 * function. Once a video sink is no longer used, it must be destroyed by
 * calling the pdraw_be_coded_video_sink_destroy() function.
 * The params structure must be provided but all parameters are optional and
 * can be left null.
 * The callbacks structure must be provided and the flush callback function is
 * required to be implemented; all callback functions are called from the
 * internal pomp_loop thread. When the flush callback function is called, the
 * application must flush the sink queue by calling
 * mbuf_coded_video_frame_queue_flush() and must return all frames outside of
 * the queue by calling mbuf_coded_video_frame_unref(); once the flushing is
 * complete, the pdraw_be_coded_video_sink_queue_flushed() function must be
 * called.
 * @note media_id must refer to a coded video media.
 * @param pdraw: PDrAW back-end instance handle
 * @param media_id: identifier of the media on which to create the sink
 * @param params: video sink parameters
 * @param cbs: video sink callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: video sink handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_coded_video_sink_new(
	struct pdraw_backend *self,
	unsigned int media_id,
	const struct pdraw_video_sink_params *params,
	const struct pdraw_backend_coded_video_sink_cbs *cbs,
	void *userdata,
	struct pdraw_coded_video_sink **ret_obj);


/**
 * Destroy a coded video sink.
 * This function stops a running video sink and frees the associated resources.
 * A video sink must not be destroyed unless all frames outside of the queue
 * have been returned by calling mbuf_coded_video_frame_unref(). Once a video
 * sink is destroyed the queue returned by pdraw_be_coded_video_sink_get_queue()
 * must no longer be used.
 * @param pdraw: PDrAW back-end instance handle
 * @param sink: video sink handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_coded_video_sink_destroy(struct pdraw_backend *self,
				  struct pdraw_coded_video_sink *sink);


/**
 * Resynchronize a coded video sink.
 * This function schedules the output of a synchronization frame (IDR) for a
 * running video sink. It can be used for example in case of
 * unrecoverable video decoder errors to restart decoding. After a video sink
 * creation, the first frame that is output is always a synchronization frame;
 * therefore it is not necessary to call this function immediately after a
 * video sink creation.
 * @param pdraw: PDrAW back-end instance handle
 * @param sink: video sink handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_coded_video_sink_resync(struct pdraw_backend *self,
				 struct pdraw_coded_video_sink *sink);


/**
 * Get the coded video sink frame queue.
 * This function returns the frame queue to use in order to retrieve frames
 * from a running video sink. Frames are retrieved from the queue by using the
 * mbuf_coded_video_frame_queue_pop() function.
 * @param pdraw: PDrAW back-end instance handle
 * @param sink: video sink handle
 * @return a pointer on a mbuf_coded_video_frame_queue object on success, NULL
 * in case of error
 */
PDRAW_BACKEND_API struct mbuf_coded_video_frame_queue *
pdraw_be_coded_video_sink_get_queue(struct pdraw_backend *self,
				    struct pdraw_coded_video_sink *sink);


/**
 * Signal that a coded video sink has been flushed.
 * This function is used to signal that flushing is complete. When the flush
 * video sink callback function is called, the application must flush the sink
 * queue by calling mbuf_coded_video_frame_queue_flush() and must return all
 * frames outside of the queue by calling mbuf_coded_video_frame_unref(); once
 * the flushing is complete, this function must be called.
 * @param pdraw: PDrAW back-end instance handle
 * @param sink: video sink handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_coded_video_sink_queue_flushed(struct pdraw_backend *self,
					struct pdraw_coded_video_sink *sink);


/**
 * Create a raw video sink.
 * This function creates a video sink on a media of the given media_id.
 * The media idenfifiers are known when the media_added or media_removed
 * general callback functions are called.
 * Once the sink is created, video frames are retrieved by getting buffers
 * from the buffer queue returned by the pdraw_be_raw_video_sink_get_queue()
 * function. Once a video sink is no longer used, it must be destroyed by
 * calling the pdraw_be_raw_video_sink_destroy() function.
 * The params structure must be provided but all parameters are optional and
 * can be left null.
 * The callbacks structure must be provided and the flush callback function is
 * required to be implemented; all callback functions are called from the
 * internal pomp_loop thread. When the flush callback function is called, the
 * application must flush the sink queue by calling
 * mbuf_raw_video_frame_queue_flush() and must return all frames outside of
 * the queue by calling mbuf_raw_video_frame_unref(); once the flushing is
 * complete, the pdraw_be_raw_video_sink_queue_flushed() function must be
 * called.
 * @note media_id must refer to a raw video media.
 * @param pdraw: PDrAW back-end instance handle
 * @param media_id: identifier of the media on which to create the sink
 * @param params: video sink parameters
 * @param cbs: video sink callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: video sink handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_raw_video_sink_new(struct pdraw_backend *self,
			    unsigned int media_id,
			    const struct pdraw_video_sink_params *params,
			    const struct pdraw_backend_raw_video_sink_cbs *cbs,
			    void *userdata,
			    struct pdraw_raw_video_sink **ret_obj);


/**
 * Destroy a raw video sink.
 * This function stops a running video sink and frees the associated resources.
 * A video sink must not be destroyed unless all frames outside of the queue
 * have been returned by calling mbuf_raw_video_frame_unref(). Once a video
 * sink is destroyed the queue returned by pdraw_be_raw_video_sink_get_queue()
 * must no longer be used.
 * @param pdraw: PDrAW back-end instance handle
 * @param sink: video sink handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_raw_video_sink_destroy(struct pdraw_backend *self,
				struct pdraw_raw_video_sink *sink);


/**
 * Get the raw video sink frame queue.
 * This function returns the frame queue to use in order to retrieve frames
 * from a running video sink. Frames are retrieved from the queue by using the
 * mbuf_raw_video_frame_queue_pop() function.
 * @param pdraw: PDrAW back-end instance handle
 * @param sink: video sink handle
 * @return a pointer on a mbuf_raw_video_frame_queue object on success, NULL
 * in case of error
 */
PDRAW_BACKEND_API struct mbuf_raw_video_frame_queue *
pdraw_be_raw_video_sink_get_queue(struct pdraw_backend *self,
				  struct pdraw_raw_video_sink *sink);


/**
 * Signal that a raw video sink has been flushed.
 * This function is used to signal that flushing is complete. When the flush
 * video sink callback function is called, the application must flush the sink
 * queue by calling mbuf_raw_video_frame_queue_flush() and must return all
 * frames outside of the queue by calling mbuf_raw_video_frame_unref(); once
 * the flushing is complete, this function must be called.
 * @param pdraw: PDrAW back-end instance handle
 * @param sink: video sink handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_raw_video_sink_queue_flushed(struct pdraw_backend *self,
				      struct pdraw_raw_video_sink *sink);


/**
 * Video encoder API
 */

/**
 * Create a video encoder.
 * This function creates a video encoder on a media of the given media_id.
 * The media idenfifiers are known when the media_added or media_removed
 * general callback functions are called.
 * The params structure must be provided but some parameters are optional and
 * can be left null. The input sub-structure in the params structure is ignored.
 * Once a video encoder is no longer used, it must be destroyed by calling the
 * pdraw_be_video_encoder_destroy() function. The callbacks structure must be
 * provided but all callback functions are optional; the frame_output callback
 * is usually called from the pomp_loop thread, but it may depend on the video
 * encoder implementation. The frame_pre_release callback is called from the
 * thread that unrefs the frame; it can be any thread.
 * @note media_id must refer to a raw video media.
 * @param self: PDrAW back-end instance handle
 * @param media_id: identifier of the media on which to create the encoder
 * @param params: video encoder parameters
 * @param cbs: video encoder callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: video encoder handle (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_video_encoder_new(struct pdraw_backend *self,
			   unsigned int media_id,
			   const struct venc_config *params,
			   const struct pdraw_backend_video_encoder_cbs *cbs,
			   void *userdata,
			   struct pdraw_video_encoder **ret_obj);


/**
 * Destroy a video encoder.
 * This function stops a running video encoder and frees the associated
 * resources.
 * @param self: PDrAW back-end instance handle
 * @param encoder: video encoder handle
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_video_encoder_destroy(struct pdraw_backend *self,
			       struct pdraw_video_encoder *encoder);


/**
 * Configure the video encoder.
 * This function can be used to dynamically reconfigure a video encoder.
 * @param self: PDrAW back-end instance handle
 * @param encoder: video encoder handle
 * @param config: video encoder configuration
 * @return a pointer on a mbuf_raw_video_frame_queue object on success, NULL
 * in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_video_encoder_configure(struct pdraw_backend *self,
				 struct pdraw_video_encoder *encoder,
				 const struct venc_dyn_config *config);


/**
 * Settings API
 */

/**
 * Get the PDrAW instance friendly name.
 * This function fills the str array with the null-terminated friendly name.
 * The string must have been previously allocated. The function writes up to
 * len characters. The friendly name is generally either the device's friendly
 * name (e.g. "Bob's phone") or the application's name (e.g.
 * "MyDroneControllerApp"). It is used for example in metadata exchanged with a
 * streaming server.
 * @param pdraw: PDrAW back-end instance handle
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_get_friendly_name_setting(struct pdraw_backend *self,
				   char *str,
				   size_t len);


/**
 * Set the PDrAW instance friendly name.
 * The friendly_name string is copied internally. The friendly name is generally
 * either the device's friendly name (e.g. "Bob's phone") or the application's
 * name (e.g. "MyDroneControllerApp"). It is used for example in metadata
 * exchanged with a streaming server. Setting the friendly name value is
 * optional.
 * @param pdraw: PDrAW back-end instance handle
 * @param friendly_name: pointer to the friendly name string
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_set_friendly_name_setting(struct pdraw_backend *self,
				   const char *friendly_name);


/**
 * Get the PDrAW instance serial number.
 * This function fills the str array with the null-terminated serial number.
 * The string must have been previously allocated. The function writes up to
 * len characters. The serial number is generally the unique serial number of
 * the device on which PDrAW is running. It is used for example in metadata
 * exchanged with a streaming server.
 * @param pdraw: PDrAW back-end instance handle
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_get_serial_number_setting(struct pdraw_backend *self,
				   char *str,
				   size_t len);


/**
 * Set the PDrAW instance serial number.
 * The serial_number string is copied internally. The serial number is generally
 * the unique serial number of the device on which PDrAW is running. It is used
 * for example in metadata exchanged with a streaming server. Setting the serial
 * number value is recommended as it is used as a unique identifier in the
 * streaming protocols.
 * @param pdraw: PDrAW back-end instance handle
 * @param serial_number: pointer to the serial number string
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_set_serial_number_setting(struct pdraw_backend *self,
				   const char *serial_number);


/**
 * Get the PDrAW instance software version.
 * This function fills the str array with the null-terminated software version.
 * The string must have been previously allocated. The function writes up to
 * len characters. The software version is generally the version number of the
 * application running PDrAW (e.g. "MyApp v1.2.3"). It is used for example in
 * metadata exchanged with a streaming server.
 * @param pdraw: PDrAW back-end instance handle
 * @param str: pointer to the string to write to (output)
 * @param len: maximum length of the string
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_get_software_version_setting(struct pdraw_backend *self,
				      char *str,
				      size_t len);


/**
 * Set the PDrAW instance software version.
 * The software_version string is copied internally. The software version is
 * generally the version number of the application running PDrAW (e.g.
 * "MyApp v1.2.3"). It is used for example in metadata exchanged with a
 * streaming server. Setting the software version value is optional.
 * @param pdraw: PDrAW back-end instance handle
 * @param software_version: pointer to the software version string
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_set_software_version_setting(struct pdraw_backend *self,
				      const char *software_version);


/**
 * Get the pipeline mode setting.
 * This function returns the pipeline mode of a PDrAW instance. The pipeline
 * mode controls whether to decode the selected video media (for full processing
 * up to the rendering), or to disable video decoding (e.g. when no rendering
 * is required, only a coded video sink).
 * @param pdraw: PDrAW back-end instance handle
 * @return the pipeline mode, or PDRAW_PIPELINE_MODE_DECODE_ALL in case of error
 */
PDRAW_BACKEND_API enum pdraw_pipeline_mode
pdraw_be_get_pipeline_mode_setting(struct pdraw_backend *self);


/**
 * Set the pipeline mode setting.
 * This function sets the pipeline mode of a PDrAW instance. This function can
 * be called only prior to any open operation. The pipeline mode controls
 * whether to decode the selected video media (for full processing up to the
 * rendering), or to disable video decoding (e.g. when no rendering is required,
 * only a coded video sink).
 * @param pdraw: PDrAW back-end instance handle
 * @param mode: pipeline mode
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_set_pipeline_mode_setting(struct pdraw_backend *self,
				   enum pdraw_pipeline_mode mode);


/**
 * Get the display screen settings.
 * This function returns the display screen settings through the xdpi, ydpi and
 * device_margin_* parameters. This is only useful if HMD distortion correction
 * is enabled in the video rendering. The xdpi and ydpi are pixel densities in
 * dots per inches. The device margins are in millimeters.
 * @param pdraw: PDrAW back-end instance handle
 * @param xdpi: horizontal pixel density (output)
 * @param ydpi: vertical pixel density (output)
 * @param device_margin_top: top device margin (output)
 * @param device_margin_bottom: bottom device margin (output)
 * @param device_margin_left: left device margin (output)
 * @param device_margin_right: right device margin (output)
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_get_display_screen_settings(struct pdraw_backend *self,
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
 * @param pdraw: PDrAW back-end instance handle
 * @param xdpi: horizontal pixel density
 * @param ydpi: vertical pixel density
 * @param device_margin_top: top device margin
 * @param device_margin_bottom: bottom device margin
 * @param device_margin_left: left device margin
 * @param device_margin_right: right device margin
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_set_display_screen_settings(struct pdraw_backend *self,
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
 * @param pdraw: PDrAW back-end instance handle
 * @return the HMD model, or PDRAW_HMD_MODEL_UNKNOWN in case of error
 */
PDRAW_BACKEND_API enum pdraw_hmd_model
pdraw_be_get_hmd_model_setting(struct pdraw_backend *self);


/**
 * Set the HMD model setting.
 * This function sets the head-mounted display (HMD) model. This is only
 * useful if HMD distortion correction is enabled in the video rendering.
 * @param pdraw: PDrAW back-end instance handle
 * @param hmd_model: HMD model
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int
pdraw_be_set_hmd_model_setting(struct pdraw_backend *self,
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
 * @param pdraw: PDrAW back-end instance handle
 * @param jvm: JVM pointer
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_set_android_jvm(struct pdraw_backend *self,
					       void *jvm);


/**
 * Debug API
 */

/**
 * Dump the current pipeline as a directed graph using the DOT file format.
 * @param pdraw: PDrAW back-end instance handle
 * @param file_name: DOT file to write to
 * @return 0 on success, negative errno value in case of error
 */
PDRAW_BACKEND_API int pdraw_be_dump_pipeline(struct pdraw_backend *self,
					     const char *file_name);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_PDRAW_BACKEND_H_ */
