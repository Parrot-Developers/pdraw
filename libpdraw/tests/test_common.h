/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — shared declarations
 *
 * Copyright (c) 2026 Parrot Drones SAS
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

#ifndef TEST_COMMON_H_
#define TEST_COMMON_H_

#include <CUnit/Automated.h>
#include <CUnit/Basic.h>
#include <CUnit/CUnit.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Cast string literal to char* for CU_SuiteInfo / CU_TestInfo name fields */
#define FN(_name) ((char *)(_name))

#ifdef __cplusplus
extern "C" {
#endif

/* Test arrays exported by each suite file */
extern CU_TestInfo g_pdraw_test_utils[];
extern CU_TestInfo g_pdraw_test_settings[];
extern CU_TestInfo g_pdraw_test_video_pres_stats[];
extern CU_TestInfo g_pdraw_test_media[];
extern CU_TestInfo g_pdraw_test_element[];
extern CU_TestInfo g_pdraw_test_source[];
extern CU_TestInfo g_pdraw_test_sink[];
extern CU_TestInfo g_pdraw_test_channel[];
extern CU_TestInfo g_pdraw_test_channel_audio[];
extern CU_TestInfo g_pdraw_test_channel_coded_video[];
extern CU_TestInfo g_pdraw_test_channel_raw_video[];
extern CU_TestInfo g_pdraw_test_element_filter[];
extern CU_TestInfo g_pdraw_test_pipeline_decoder_video[];
extern CU_TestInfo g_pdraw_test_pipeline_decoder_audio[];
extern CU_TestInfo g_pdraw_test_pipeline_scaler[];
extern CU_TestInfo g_pdraw_test_pipeline_encoder_video[];
extern CU_TestInfo g_pdraw_test_pipeline_encoder_audio[];
extern CU_TestInfo g_pdraw_test_pipeline_vipc[];
extern CU_TestInfo g_pdraw_test_alsa_audio_format[];
extern CU_TestInfo g_pdraw_test_pipeline_alsa_source[];
extern CU_TestInfo g_pdraw_test_pipeline_renderer_video[];
extern CU_TestInfo g_pdraw_test_pipeline_renderer_audio[];
extern CU_TestInfo g_pdraw_test_pipeline_muxer_record_isobmff[];
extern CU_TestInfo g_pdraw_test_pipeline_muxer_record_photo[];
extern CU_TestInfo g_pdraw_test_pipeline_sourcesink_coded[];
extern CU_TestInfo g_pdraw_test_pipeline_sourcesink_raw[];
extern CU_TestInfo g_pdraw_test_pipeline_sourcesink_audio[];
extern CU_TestInfo g_pdraw_test_pipeline_demuxer_stream_net[];
extern CU_TestInfo g_pdraw_test_pipeline_muxer_stream_rtsp_net[];
extern CU_TestInfo g_pdraw_test_pipeline_muxer_stream_rtsp_mux[];
extern CU_TestInfo g_pdraw_test_pipeline_muxer_stream_rtmp[];
extern CU_TestInfo g_pdraw_test_pipeline_demuxer_stream_mux[];

/* Tier B suite fixture — shared pomp::Loop + Session */
int pdraw_test_loop_init(void);
int pdraw_test_loop_cleanup(void);

/* API input-validation test arrays (Tier C) */
extern CU_TestInfo g_pdraw_test_api_session[];
extern CU_TestInfo g_pdraw_test_api_demuxer[];
extern CU_TestInfo g_pdraw_test_api_muxer[];
extern CU_TestInfo g_pdraw_test_api_renderer_video[];
extern CU_TestInfo g_pdraw_test_api_renderer_audio[];
extern CU_TestInfo g_pdraw_test_api_coded_video_source[];
extern CU_TestInfo g_pdraw_test_api_raw_video_source[];
extern CU_TestInfo g_pdraw_test_api_coded_video_sink[];
extern CU_TestInfo g_pdraw_test_api_raw_video_sink[];
extern CU_TestInfo g_pdraw_test_api_vipc_source[];
extern CU_TestInfo g_pdraw_test_api_alsa_source[];
extern CU_TestInfo g_pdraw_test_api_audio_source[];
extern CU_TestInfo g_pdraw_test_api_audio_sink[];
extern CU_TestInfo g_pdraw_test_api_encoder_video[];
extern CU_TestInfo g_pdraw_test_api_encoder_audio[];
extern CU_TestInfo g_pdraw_test_api_scaler_video[];
extern CU_TestInfo g_pdraw_test_api_misc[];

/* Tier C suite fixture — loop + C++ session + public C handle */
int pdraw_test_api_init(void);
int pdraw_test_api_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif /* TEST_COMMON_H_ */
