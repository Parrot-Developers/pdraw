/**
 * Parrot Drones Audio and Video Vector library
 * libpdraw unit tests — CUnit runner
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

#include "test_common.h"


#define SUITE(name, init, cleanup)                                             \
	{                                                                      \
		FN(#name), init, cleanup, g_pdraw_test_##name                  \
	}
#define SUITE_NOFIXTURE(name) SUITE(name, NULL, NULL)
#define SUITE_LOOP(name)                                                       \
	SUITE(name, pdraw_test_loop_init, pdraw_test_loop_cleanup)
#define SUITE_API(name) SUITE(name, pdraw_test_api_init, pdraw_test_api_cleanup)


static CU_SuiteInfo s_suites[] = {
	/* Tier A — no fixture needed */
	SUITE_NOFIXTURE(utils),
	SUITE_NOFIXTURE(settings),
	SUITE_NOFIXTURE(video_pres_stats),
	SUITE_NOFIXTURE(alsa_audio_format),

	/* Tier B — pomp::Loop + Session fixture */
	SUITE_LOOP(media),
	SUITE_LOOP(element),
	SUITE_LOOP(source),
	SUITE_LOOP(sink),
	SUITE_LOOP(channel),
	SUITE_LOOP(channel_audio),
	SUITE_LOOP(channel_coded_video),
	SUITE_LOOP(channel_raw_video),
	SUITE_LOOP(element_filter),

	/* pipeline_decoder_video / pipeline_decoder_audio / pipeline_scaler /
	 * pipeline_encoder_video / pipeline_encoder_audio / pipeline_vipc /
	 * pipeline_renderer_video / pipeline_muxer: fully self-contained
	 * (each test builds its own loop + session + listener) */
	SUITE_NOFIXTURE(pipeline_decoder_video),
	SUITE_NOFIXTURE(pipeline_decoder_audio),
	SUITE_NOFIXTURE(pipeline_scaler),
	SUITE_NOFIXTURE(pipeline_encoder_video),
	SUITE_NOFIXTURE(pipeline_encoder_audio),
	SUITE_NOFIXTURE(pipeline_vipc),
	SUITE_NOFIXTURE(pipeline_alsa_source),
	SUITE_NOFIXTURE(pipeline_renderer_video),
	SUITE_NOFIXTURE(pipeline_renderer_audio),
	SUITE_NOFIXTURE(pipeline_muxer_record_isobmff),
	SUITE_NOFIXTURE(pipeline_muxer_record_photo),
	SUITE_NOFIXTURE(pipeline_sourcesink_coded),
	SUITE_NOFIXTURE(pipeline_sourcesink_raw),
	SUITE_NOFIXTURE(pipeline_sourcesink_audio),
	SUITE_NOFIXTURE(pipeline_demuxer_stream_net),
	SUITE_NOFIXTURE(pipeline_demuxer_stream_mux),
	SUITE_NOFIXTURE(pipeline_muxer_stream_rtsp_net),
	SUITE_NOFIXTURE(pipeline_muxer_stream_rtsp_mux),
	SUITE_NOFIXTURE(pipeline_muxer_stream_rtmp),

	/* Tier C — loop + C++ session + public C handle */
	SUITE_API(api_session),
	SUITE_API(api_demuxer),
	SUITE_API(api_muxer),
	SUITE_API(api_renderer_video),
	SUITE_API(api_renderer_audio),
	SUITE_API(api_coded_video_source),
	SUITE_API(api_raw_video_source),
	SUITE_API(api_coded_video_sink),
	SUITE_API(api_raw_video_sink),
	SUITE_API(api_vipc_source),
	SUITE_API(api_alsa_source),
	SUITE_API(api_audio_source),
	SUITE_API(api_audio_sink),
	SUITE_API(api_encoder_video),
	SUITE_API(api_encoder_audio),
	SUITE_API(api_scaler_video),
	SUITE_NOFIXTURE(api_misc),

	CU_SUITE_INFO_NULL,
};


#undef SUITE
#undef SUITE_NOFIXTURE
#undef SUITE_LOOP
#undef SUITE_API


int main(void)
{
	const char *automated = getenv("CUNIT_AUTOMATED");
	const char *outname = getenv("CUNIT_OUT_NAME");
	unsigned int failures;
	int ret;

	CU_initialize_registry();

	ret = CU_register_suites(s_suites);
	if (ret != CUE_SUCCESS) {
		fprintf(stderr, "CU_register_suites: %s\n", CU_get_error_msg());
		CU_cleanup_registry();
		return EXIT_FAILURE;
	}

	if (automated) {
		if (outname)
			CU_set_output_filename(outname);
		CU_automated_run_tests();
		CU_list_tests_to_file();
	} else {
		CU_basic_set_mode(CU_BRM_VERBOSE);
		CU_basic_run_tests();
	}

	failures = CU_get_number_of_failures();
	CU_cleanup_registry();
	return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
