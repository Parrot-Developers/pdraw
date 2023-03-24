cmake_minimum_required(VERSION 3.15)
project(pdraw VERSION 1.0)



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

add_library(${PROJECT_NAME} SHARED ${LIB_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES
                    POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
)

# checks if set up rpath exists for install
if(COMMAND set_up_rpath)
    set_up_rpath()
else()
    message("Set up rpath not defined!")
endif()

target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

target_include_directories(${PROJECT_NAME} PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "PDRAW_API_EXPORTS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "_USE_MATH_DEFINES")
target_compile_definitions(${PROJECT_NAME} PRIVATE "_GNU_SOURCE")
target_compile_options(${PROJECT_NAME} PRIVATE "-fvisibility=hidden")
target_compile_options(${PROJECT_NAME} PRIVATE "-std=c++11")

find_package(Eigen3 CONFIG REQUIRED)

target_link_libraries(${PROJECT_NAME}

                     PRIVATE
                      Eigen3::Eigen
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
                      video-decode-core
                      video-defs

                      video-encode
                      video-encode-core

                      video-metadata
                      video-scale
                      video-scale-core
                      video-streaming

)

set(${PROJECT_NAME}-headers
    ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# to get all header files correctly
install(
    DIRECTORY ${${PROJECT_NAME}-headers} DESTINATION include/${PROJECT_NAME}
)
