set(LIB_SOURCES
        src/pdraw_channel_coded_video.cpp
        src/pdraw_channel_raw_video.cpp
        src/pdraw_decoder_video.cpp
        src/pdraw_demuxer.cpp
        src/pdraw_demuxer_record.cpp
        src/pdraw_demuxer_stream_mux.cpp
        src/pdraw_demuxer_stream_net.cpp
        src/pdraw_demuxer_stream.cpp
        src/pdraw_element.cpp
        src/pdraw_encoder_video.cpp
        src/pdraw_external_coded_video_sink.cpp
        src/pdraw_external_raw_video_sink.cpp
        src/pdraw_gles2_hmd_colors.cpp
        src/pdraw_gles2_hmd_indices.cpp
        src/pdraw_gles2_hmd_positions_cockpitglasses.cpp
        src/pdraw_gles2_hmd_positions_cockpitglasses2.cpp
        src/pdraw_gles2_hmd_shaders.cpp
        src/pdraw_gles2_hmd_texcoords_cockpitglasses_blue.cpp
        src/pdraw_gles2_hmd_texcoords_cockpitglasses_red.cpp
        src/pdraw_gles2_hmd_texcoords.cpp
        src/pdraw_gles2_hmd.cpp
        src/pdraw_gles2_video.cpp
        src/pdraw_media.cpp
        src/pdraw_muxer_record.cpp
        src/pdraw_muxer_stream_rtmp.cpp
        src/pdraw_muxer.cpp
        src/pdraw_renderer_gles2.cpp
        src/pdraw_renderer_videocoreegl.cpp
        src/pdraw_renderer.cpp
        src/pdraw_scaler_video.cpp
        src/pdraw_session.cpp
        src/pdraw_settings.cpp
        src/pdraw_sink_coded_video.cpp
        src/pdraw_sink_raw_video.cpp
        src/pdraw_source_coded_video.cpp
        src/pdraw_source_raw_video.cpp
        src/pdraw_utils.cpp
        src/pdraw_video_pres_stats.cpp
        src/pdraw_wrapper.cpp
)


add_library(pdraw ${LIB_SOURCES})
add_library(pdraw-metadata_compiler_flags INTERFACE)
target_compile_features(pdraw-metadata_compiler_flags INTERFACE cxx_std_11)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

target_include_directories(pdraw PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(pdraw PRIVATE "PDRAW_API_EXPORTS")
target_compile_definitions(pdraw PRIVATE "_USE_MATH_DEFINES")
target_compile_definitions(pdraw PRIVATE "_GNU_SOURCE")
target_compile_options(pdraw PRIVATE "-fvisibility=hidden")

find_package(Eigen3 CONFIG REQUIRED)
target_link_libraries(pdraw Eigen3::Eigen)

target_link_libraries(pdraw
                        futils
                        h264
                        h265
                        media-buffers
                        media-buffers-memory
                        media-buffers-memory-generic
                        mp4
                        pomp
                        rtp
                        rtsp
                        sdp
                        transport-packet
                        transport-socket
                        ulog
                        video-decode
                        video-defs
                        video-encode
                        video-metadata
                        video-scale
                        video-streaming
                        pdraw-metadata_compiler_flags
)
