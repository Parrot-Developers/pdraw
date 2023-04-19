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


namespace Pdraw {


#	define GLES2_VIDEO_TEX_UNIT_COUNT 3
#	define GLES2_VIDEO_FBO_TEX_UNIT_COUNT 1
#	define GLES2_VIDEO_BLUR_FBO_TARGET_SIZE 512
#	define GLES2_VIDEO_BLUR_TAP_COUNT 15
#	define GLES2_VIDEO_PADDING_FBO_TARGET_SIZE_1 256
#	define GLES2_VIDEO_PADDING_FBO_TARGET_SIZE_2 16
#	define GLES2_VIDEO_HISTOGRAM_FBO_TARGET_SIZE 256


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
	GLES2_VIDEO_TRANSITION_FLASH,
};

class Session;


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

	int loadFrame(const uint8_t *framePlanes[3],
		      size_t framePlaneStride[3],
		      const struct vdef_raw_format *format,
		      const struct vdef_frame_info *info,
		      struct egl_display *eglDisplay = nullptr);

	int renderFrame(const struct pdraw_rect *renderPos,
			struct pdraw_rect *contentPos,
			Eigen::Matrix4f &viewProjMat,
			size_t framePlaneStride[3],
			const struct vdef_raw_format *format,
			const struct vdef_frame_info *info,
			const struct vdef_rect *crop,
			struct vmeta_frame *metadata,
			const struct pdraw_video_renderer_params *params);

	int clear(Eigen::Matrix4f &viewProjMat);

	void setExtTexture(GLuint texture);

	void getHistograms(float *histogram[PDRAW_HISTOGRAM_CHANNEL_MAX],
			   size_t histogramLen[PDRAW_HISTOGRAM_CHANNEL_MAX]);

private:
	enum program {
		PROGRAM_NOCONV = 0,
		PROGRAM_YUV_TO_RGB_PLANAR,
		PROGRAM_YUV_TO_RGB_PLANAR_10_16LE,
		PROGRAM_YUV_TO_RGB_SEMIPLANAR,
		PROGRAM_YUV_TO_RGB_SEMIPLANAR_10_16LE_HIGH,
		PROGRAM_GRAY_TO_RGB_PLANAR,
		PROGRAM_MAX,
	};

	enum program getProgram(const struct vdef_raw_format *format,
				bool *swapUv);

	int setupBlur(void);

	void cleanupBlur(void);

	int setupBlurFbo(void);

	void cleanupBlurFbo(void);

	void renderBlur(size_t framePlaneStride[3],
			const struct vdef_raw_format *format,
			const struct vdef_frame_info *info,
			const struct vdef_rect *crop,
			const struct pdraw_rect *renderPos,
			float videoW,
			float videoH,
			Eigen::Matrix4f &viewProjMat);

	int setupPaddingFbo(enum pdraw_video_renderer_fill_mode fillMode);

	void cleanupPaddingFbo(void);

	void renderPadding(size_t framePlaneStride[3],
			   const struct vdef_raw_format *format,
			   const struct vdef_frame_info *info,
			   const struct vdef_rect *crop,
			   const struct pdraw_rect *renderPos,
			   float videoW,
			   float videoH,
			   float videoW2,
			   float videoH2,
			   float videoAR,
			   float windowAR,
			   enum pdraw_video_renderer_fill_mode fillMode,
			   bool immersive,
			   Eigen::Matrix4f &viewProjMat);

	void setupZebra(enum program prog);

	void updateZebra(struct pdraw_rect *contentPos,
			 enum program prog,
			 bool enable,
			 float threshold);

	int setupHistograms(void);

	void cleanupHistograms(void);

	void computeHistograms(size_t framePlaneStride[3],
			       const struct vdef_raw_format *format,
			       const struct vdef_frame_info *info,
			       const struct vdef_rect *crop,
			       const struct pdraw_rect *renderPos,
			       bool enable);

	void updateTransition(void);

	Session *mSession;
	unsigned int mVideoWidth;
	unsigned int mVideoHeight;
	unsigned int mFirstTexUnit;
	GLuint mDefaultFbo;
	enum gles2_video_transition mTransition;
	uint64_t mTransitionStartTime;
	uint64_t mTransitionDuration;
	bool mTransitionHold;
	GLint mProgram[PROGRAM_MAX];
	GLint mProgramTransformMatrix[PROGRAM_MAX];
	GLint mProgramYuv2RgbMatrix[PROGRAM_MAX];
	GLint mProgramYuv2RgbOffset[PROGRAM_MAX];
	GLint mProgramStride[PROGRAM_MAX];
	GLint mProgramMaxCoords[PROGRAM_MAX];
	GLint mProgramSatCoef[PROGRAM_MAX];
	GLint mProgramLightCoef[PROGRAM_MAX];
	GLint mProgramDarkCoef[PROGRAM_MAX];
	GLint mProgramSwapUv[PROGRAM_MAX];
	GLint mProgramZebraEnable[PROGRAM_MAX];
	GLint mProgramZebraThreshold[PROGRAM_MAX];
	GLint mProgramZebraPhase[PROGRAM_MAX];
	GLint mProgramZebraWeight[PROGRAM_MAX];
	GLuint mTextures[GLES2_VIDEO_TEX_UNIT_COUNT];
	GLuint mExtTexture;
	GLint mUniformSamplers[PROGRAM_MAX][GLES2_VIDEO_TEX_UNIT_COUNT];
	GLint mPositionHandle[PROGRAM_MAX];
	GLint mTexcoordHandle[PROGRAM_MAX];
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
	GLint mHistogramProgram[PROGRAM_MAX];
	GLint mHistogramYuv2RgbMatrix[PROGRAM_MAX];
	GLint mHistogramYuv2RgbOffset[PROGRAM_MAX];
	GLint mHistogramUniformSampler[PROGRAM_MAX][GLES2_VIDEO_TEX_UNIT_COUNT];
	GLint mHistogramPositionHandle[PROGRAM_MAX];
	GLint mHistogramTexcoordHandle[PROGRAM_MAX];
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

	static const GLchar *videoFragmentShaders[PROGRAM_MAX][3];
	static const GLchar *histogramFragmentShaders[PROGRAM_MAX][2];
};

} /* namespace Pdraw */

#endif /* USE_GLES2 */

#endif /* !_PDRAW_GLES2_VIDEO_HPP_ */
