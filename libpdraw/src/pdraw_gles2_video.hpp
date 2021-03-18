/**
 * Parrot Drones Awesome Video Viewer Library
 * OpenGL ES 2.0 video rendering
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

#ifndef _PDRAW_GLES2_VIDEO_HPP_
#define _PDRAW_GLES2_VIDEO_HPP_

#ifdef USE_GLES2

#	include "pdraw_gles2_common.hpp"
#	include "pdraw_utils.hpp"

#	ifdef BCM_VIDEOCORE
#		include <EGL/egl.h>
#		include <EGL/eglext.h>
#	endif /* BCM_VIDEOCORE */

#include "LFClientEngine.h"

namespace Pdraw {


#	define GLES2_VIDEO_TEX_UNIT_COUNT 3
#	define GLES2_VIDEO_FBO_TEX_UNIT_COUNT 1
#	define GLES2_VIDEO_BLUR_FBO_TARGET_SIZE 512
#	define GLES2_VIDEO_BLUR_TAP_COUNT 15
#	define GLES2_VIDEO_PADDING_FBO_TARGET_SIZE_1 256
#	define GLES2_VIDEO_PADDING_FBO_TARGET_SIZE_2 16
#	define GLES2_VIDEO_HISTOGRAM_FBO_TARGET_SIZE 256


enum gles2_video_color_conversion {
	GLES2_VIDEO_COLOR_CONVERSION_NONE = 0,
	GLES2_VIDEO_COLOR_CONVERSION_I420_TO_RGB,
	GLES2_VIDEO_COLOR_CONVERSION_NV12_TO_RGB,
	GLES2_VIDEO_COLOR_CONVERSION_MAX,
};

enum gles2_video_yuv_range {
	GLES2_VIDEO_YUV_LIMITED_RANGE = 0,
	GLES2_VIDEO_YUV_FULL_RANGE,
};

enum gles2_video_transition {
	GLES2_VIDEO_TRANSITION_NONE = 0,
	GLES2_VIDEO_TRANSITION_FADE_TO_BLACK,
	GLES2_VIDEO_TRANSITION_FADE_FROM_BLACK,
	GLES2_VIDEO_TRANSITION_FADE_TO_WHITE,
	GLES2_VIDEO_TRANSITION_FADE_FROM_WHITE,
	GLES2_VIDEO_TRANSITION_FADE_TO_BLACK_AND_WHITE,
	GLES2_VIDEO_TRANSITION_FADE_FROM_BLACK_AND_WHITE,
	GLES2_VIDEO_TRANSITION_FADE_TO_BLUR,
	GLES2_VIDEO_TRANSITION_FADE_FROM_BLUR,
};

class Session;
class VideoMedia;


class Gles2Video {
public:
	Gles2Video(Session *session,
		   GLuint defaultFbo,
		   unsigned int firstTexUnit);

	~Gles2Video(void);

	static int getTexUnitCount(void)
	{
		return GLES2_VIDEO_TEX_UNIT_COUNT +
		       GLES2_VIDEO_FBO_TEX_UNIT_COUNT;
	}

	GLuint getDefaultFbo(void)
	{
		return mDefaultFbo;
	}

	void setDefaultFbo(GLuint defaultFbo)
	{
		mDefaultFbo = defaultFbo;
	}

	float getSatCoef(void)
	{
		return mBaseSatCoef;
	}

	void setSatCoef(float coef)
	{
		mBaseSatCoef = coef < 0. ? 0. : (coef > 1. ? 1. : coef);
	}

	float getLightCoef(void)
	{
		return mBaseLightCoef;
	}

	void setLightCoef(float coef)
	{
		mBaseLightCoef = coef < 0. ? 0. : (coef > 1. ? 1. : coef);
	}

	float getDarkCoef(void)
	{
		return mBaseDarkCoef;
	}

	void setDarkCoef(float coef)
	{
		mBaseDarkCoef = coef < 0. ? 0. : (coef > 1. ? 1. : coef);
	}

	void startTransition(enum gles2_video_transition transition,
			     uint64_t duration,
			     bool hold);

	void abortTransition(void);

	int loadFrame(const uint8_t *frameData,
		      size_t framePlaneOffset[3],
		      size_t framePlaneStride[3],
		      unsigned int frameWidth,
		      unsigned int frameHeight,
		      enum gles2_video_color_conversion colorConversion,
		      struct egl_display *eglDisplay = NULL);

	int renderFrame(size_t framePlaneStride[3],
			unsigned int frameHeight,
			unsigned int cropLeft,
			unsigned int cropTop,
			unsigned int cropWidth,
			unsigned int cropHeight,
			unsigned int sarWidth,
			unsigned int sarHeight,
			const struct pdraw_rect *renderPos,
			struct pdraw_rect *contentPos,
			Eigen::Matrix4f &viewProjMat,
			enum gles2_video_color_conversion colorConversion,
			enum gles2_video_yuv_range yuvRange,
			const struct vmeta_frame *metadata,
			const struct pdraw_video_renderer_params *params);

	void setVideoMedia(VideoMedia *media);

	void setExtTexture(GLuint texture);

	void getHistograms(float *histogram[PDRAW_HISTOGRAM_CHANNEL_MAX],
			   size_t histogramLen[PDRAW_HISTOGRAM_CHANNEL_MAX]);

private:
    LFClientEngine _lfClientEngine;
	int setupBlur(void);

	void cleanupBlur(void);

	int setupBlurFbo(unsigned int cropWidth, unsigned int cropHeight);

	void cleanupBlurFbo(void);

	void renderBlur(size_t framePlaneStride[3],
			unsigned int frameHeight,
			unsigned int cropLeft,
			unsigned int cropTop,
			unsigned int cropWidth,
			unsigned int cropHeight,
			const struct pdraw_rect *render_pos,
			float videoW,
			float videoH,
			enum gles2_video_color_conversion colorConversion,
			enum gles2_video_yuv_range yuvRange,
			Eigen::Matrix4f &viewProjMat);

	int setupPaddingFbo(unsigned int cropWidth,
			    unsigned int cropHeight,
			    enum pdraw_video_renderer_fill_mode fillMode);

	void cleanupPaddingFbo(void);

	void renderPadding(size_t framePlaneStride[3],
			   unsigned int frameHeight,
			   unsigned int cropLeft,
			   unsigned int cropTop,
			   unsigned int cropWidth,
			   unsigned int cropHeight,
			   const struct pdraw_rect *renderPos,
			   float videoW,
			   float videoH,
			   float videoW2,
			   float videoH2,
			   float videoAR,
			   float windowAR,
			   enum pdraw_video_renderer_fill_mode fillMode,
			   enum gles2_video_color_conversion colorConversion,
			   enum gles2_video_yuv_range yuvRange,
			   bool immersive,
			   Eigen::Matrix4f &viewProjMat);

	void setupZebra(enum gles2_video_color_conversion colorConversion);

	void updateZebra(struct pdraw_rect *contentPos,
			 enum gles2_video_color_conversion colorConversion,
			 bool enable,
			 float threshold);

	int setupHistograms(void);

	void cleanupHistograms(void);

	void
	computeHistograms(size_t framePlaneStride[3],
			  unsigned int frameHeight,
			  unsigned int cropLeft,
			  unsigned int cropTop,
			  unsigned int cropWidth,
			  unsigned int cropHeight,
			  const struct pdraw_rect *renderPos,
			  enum gles2_video_color_conversion colorConversion,
			  enum gles2_video_yuv_range yuvRange,
			  bool enable);

	void updateTransition(void);

	Session *mSession;
	VideoMedia *mMedia;
	unsigned int mVideoWidth;
	unsigned int mVideoHeight;
	unsigned int mFirstTexUnit;
	GLuint mDefaultFbo;
	enum gles2_video_transition mTransition;
	uint64_t mTransitionStartTime;
	uint64_t mTransitionDuration;
	bool mTransitionHold;
	GLint mProgram[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mProgramTransformMatrix[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mProgramYuv2RgbMatrix[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mProgramYuv2RgbOffset[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mProgramStride[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mProgramSatCoef[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mProgramLightCoef[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mProgramDarkCoef[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mProgramZebraEnable[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mProgramZebraThreshold[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mProgramZebraPhase[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mProgramZebraWeight[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLuint mTextures[GLES2_VIDEO_TEX_UNIT_COUNT];
	GLuint mExtTexture;
	GLint mUniformSamplers[GLES2_VIDEO_COLOR_CONVERSION_MAX]
			      [GLES2_VIDEO_TEX_UNIT_COUNT];
	GLint mPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	bool mBlurInit;
	bool mApplyBlur;
	float mBlurWeights[GLES2_VIDEO_BLUR_TAP_COUNT];
	unsigned int mBlurFboWidth;
	unsigned int mBlurFboHeight;
	GLuint mBlurFbo[2];
	GLuint mBlurFboTexture[2];
	GLint mBlurProgram[2];
	GLint mBlurUniformPixelSize[2];
	GLint mBlurUniformWeights[2];
	GLint mBlurUniformSampler[2];
	GLint mBlurPositionHandle[2];
	unsigned int mPaddingPass1Width;
	unsigned int mPaddingPass1Height;
	unsigned int mPaddingPass2Width;
	unsigned int mPaddingPass2Height;
	float mPaddingBlurWeights[GLES2_VIDEO_BLUR_TAP_COUNT];
	GLuint mPaddingFbo[4];
	GLuint mPaddingFboTexture[4];
	bool mHistogramInit;
	uint64_t mHistogramLastComputeTime;
	GLint mHistogramProgram[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mHistogramYuv2RgbMatrix[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mHistogramYuv2RgbOffset[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mHistogramUniformSampler[GLES2_VIDEO_COLOR_CONVERSION_MAX]
				      [GLES2_VIDEO_TEX_UNIT_COUNT];
	GLint mHistogramPositionHandle[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLint mHistogramTexcoordHandle[GLES2_VIDEO_COLOR_CONVERSION_MAX];
	GLuint mHistogramFbo;
	GLuint mHistogramFboTexture;
	uint8_t *mHistogramBuffer;
	bool mHistogramValid[PDRAW_HISTOGRAM_CHANNEL_MAX];
	uint32_t *mHistogram[PDRAW_HISTOGRAM_CHANNEL_MAX];
	float *mHistogramNorm[PDRAW_HISTOGRAM_CHANNEL_MAX];
	float mSatCoef; /* 0.0 (greyscale) .. 1.0 (original video) */
	float mBaseSatCoef;
	float mLightCoef; /* 0.0 (white) .. 1.0 (video) */
	float mBaseLightCoef;
	float mDarkCoef; /* 0.0 (black) .. 1.0 (video) */
	float mBaseDarkCoef;
#	ifdef BCM_VIDEOCORE
	EGLImageKHR mEglImage;
#	endif /* BCM_VIDEOCORE */
};

} /* namespace Pdraw */

#endif /* USE_GLES2 */

#endif /* !_PDRAW_GLES2_VIDEO_HPP_ */
