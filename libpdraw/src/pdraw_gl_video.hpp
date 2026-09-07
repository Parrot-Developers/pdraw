/**
 * Parrot Drones Audio and Video Vector library
 * OpenGL video rendering
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

#ifdef PDRAW_USE_GL

#	include "pdraw_gl_common.hpp"
#	include "pdraw_utils.hpp"

#	include <algorithm>
#	include <array>
#	include <vector>


namespace Pdraw {


constexpr size_t GL_VIDEO_TEX_UNIT_COUNT = 3;
constexpr size_t GL_VIDEO_FBO_TEX_UNIT_COUNT = 1;
constexpr size_t GL_VIDEO_MBSTATUS_TEX_UNIT_COUNT = 1;
constexpr size_t GL_VIDEO_BLUR_FBO_TARGET_SIZE = 512;
constexpr size_t GL_VIDEO_BLUR_TAP_COUNT = 15;
constexpr size_t GL_VIDEO_PADDING_FBO_TARGET_SIZE_1 = 256;
constexpr size_t GL_VIDEO_PADDING_FBO_TARGET_SIZE_2 = 16;
constexpr size_t GL_VIDEO_HISTOGRAM_FBO_TARGET_SIZE = 256;


enum class GlVideoTransition : unsigned int {
	NONE = 0,
	FADE_TO_BLACK,
	FADE_FROM_BLACK,
	FADE_TO_WHITE,
	FADE_FROM_WHITE,
	FADE_TO_BLACK_AND_WHITE,
	FADE_FROM_BLACK_AND_WHITE,
	FADE_TO_BLUR,
	FADE_FROM_BLUR,
	FLASH,
};

class Session;


class GlVideo {
	PDRAW_DISABLE_COPY(GlVideo)

public:
	GlVideo(Session *session,
		GLuint defaultFbo,
		unsigned int firstTexUnit,
		bool simplified);

	~GlVideo();

	static int getTexUnitCount()
	{
		return GL_VIDEO_TEX_UNIT_COUNT + GL_VIDEO_FBO_TEX_UNIT_COUNT +
		       GL_VIDEO_MBSTATUS_TEX_UNIT_COUNT;
	}

	GLuint getDefaultFbo() const
	{
		return mDefaultFbo;
	}

	void setDefaultFbo(GLuint defaultFbo)
	{
		mDefaultFbo = defaultFbo;
	}

	float getBrightnessCoef() const
	{
		return mBrightnessCoef;
	}

	void setBrightnessCoef(float coef)
	{
		mBrightnessCoef = coef;
	}

	float getContrastCoef() const
	{
		return mContrastCoef;
	}

	void setContrastCoef(float coef)
	{
		mContrastCoef = coef;
	}

	float getGammaCoef() const
	{
		return mGammaCoef;
	}

	void setGammaCoef(float coef)
	{
		mGammaCoef = coef;
	}

	float getSatCoef() const
	{
		return mBaseSatCoef;
	}

	void setSatCoef(float coef)
	{
		mBaseSatCoef = std::clamp(coef, 0.f, 1.f);
	}

	float getLightCoef() const
	{
		return mBaseLightCoef;
	}

	void setLightCoef(float coef)
	{
		mBaseLightCoef = std::clamp(coef, 0.f, 1.f);
	}

	float getDarkCoef() const
	{
		return mBaseDarkCoef;
	}

	void setDarkCoef(float coef)
	{
		mBaseDarkCoef = std::clamp(coef, 0.f, 1.f);
	}

	void startTransition(GlVideoTransition transition,
			     uint64_t duration,
			     bool hold);

	void abortTransition();

	int loadFrame(const uint8_t *framePlanes[VDEF_RAW_MAX_PLANE_COUNT],
		      const size_t framePlaneStride[VDEF_RAW_MAX_PLANE_COUNT],
		      const struct vdef_raw_format *format,
		      const struct vdef_frame_info *info,
		      const uint8_t *mbStatus = nullptr);

	int renderFrame(const struct pdraw_rect *renderPos,
			struct pdraw_rect *contentPos,
			const Eigen::Matrix4f &viewProjMat,
			const size_t framePlaneStride[VDEF_RAW_MAX_PLANE_COUNT],
			const struct vdef_raw_format *format,
			const struct vdef_frame_info *info,
			const struct vdef_rect *crop,
			const struct pdraw_video_renderer_params *params);

	int clear(const Eigen::Matrix4f &viewProjMat) const;

	void setExtTexture(GLuint texture);

	void
	getHistograms(float *histogram[PDRAW_HISTOGRAM_CHANNEL_MAX],
		      size_t histogramLen[PDRAW_HISTOGRAM_CHANNEL_MAX]) const;

private:
	enum class Program : unsigned int {
		NOCONV = 0,
		YUV_TO_RGB_PLANAR,
		YUV_TO_RGB_PLANAR_10_16LE,
		YUV_TO_RGB_SEMIPLANAR,
		YUV_TO_RGB_SEMIPLANAR_10_16LE_HIGH,
		GRAY_TO_RGB_PLANAR,
		GRAY16_TO_RGB_PLANAR,
		GRAY32_TO_RGB_PLANAR,
		MAX,
	};
	static constexpr size_t PROGRAM_MAX = static_cast<size_t>(Program::MAX);

	static constexpr size_t toIndex(Program p) noexcept
	{
		return static_cast<size_t>(p);
	}

	Program getProgram(const struct vdef_raw_format *format,
			   bool *swapUv) const;

	void fillYuv2RgbMatrix(enum vdef_matrix_coefs matrixCoefs,
			       bool fullRange,
			       bool swapUv,
			       std::array<float, 9> &yuv2RgbMatrix,
			       std::array<float, 3> &yuv2RgbOffset) const;

	int setupBlur();

	void cleanupBlur();

	int setupBlurFbo();

	void cleanupBlurFbo();

	void renderBlur(const size_t framePlaneStride[VDEF_RAW_MAX_PLANE_COUNT],
			const struct vdef_raw_format *format,
			const struct vdef_frame_info *info,
			const struct vdef_rect *crop,
			const struct pdraw_rect *renderPos,
			float videoW,
			float videoH,
			bool verticalMirror,
			const Eigen::Matrix4f &viewProjMat) const;

	int setupPaddingFbo();

	void cleanupPaddingFbo();

	void
	renderPadding(const size_t framePlaneStride[VDEF_RAW_MAX_PLANE_COUNT],
		      const struct vdef_raw_format *format,
		      const struct vdef_frame_info *info,
		      const struct vdef_rect *crop,
		      const struct pdraw_rect *renderPos,
		      float videoW,
		      float videoH,
		      float videoW2,
		      float videoH2,
		      bool verticalMirror,
		      bool immersive,
		      const Eigen::Matrix4f &viewProjMat) const;

	void setupZebra(Program prog) const;

	void updateZebra(const struct pdraw_rect *contentPos,
			 Program prog,
			 bool enable,
			 float threshold) const;

	int setupHistograms();

	void cleanupHistograms();

	void computeHistograms(
		const size_t framePlaneStride[VDEF_RAW_MAX_PLANE_COUNT],
		const struct vdef_raw_format *format,
		const struct vdef_frame_info *info,
		const struct vdef_rect *crop,
		const struct pdraw_rect *renderPos,
		bool verticalMirror,
		bool enable);

	void updateTransition();

	static unsigned int getTextureMaxUnpackAlignment(unsigned int width);

	Session *mSession = nullptr;
	unsigned int mVideoWidth = 0;
	unsigned int mVideoHeight = 0;
	enum pdraw_video_renderer_fill_mode mFillMode =
		PDRAW_VIDEO_RENDERER_FILL_MODE_FIT;
	unsigned int mFirstTexUnit = 0;
	GLuint mDefaultFbo = 0;
	GlVideoTransition mTransition = GlVideoTransition::NONE;
	uint64_t mTransitionStartTime = 0;
	uint64_t mTransitionDuration = 0;
	bool mTransitionHold = false;
	std::array<GLint, PROGRAM_MAX> mProgram{};
	std::array<GLint, PROGRAM_MAX> mProgramTransformMatrix{};
	std::array<GLint, PROGRAM_MAX> mProgramYuv2RgbMatrix{};
	std::array<GLint, PROGRAM_MAX> mProgramYuv2RgbOffset{};
	std::array<GLint, PROGRAM_MAX> mProgramStride{};
	std::array<GLint, PROGRAM_MAX> mProgramMaxCoordsRatio{};
	std::array<GLint, PROGRAM_MAX> mProgramMaxClamp{};
	std::array<GLint, PROGRAM_MAX> mProgramBrightnessCoef{};
	std::array<GLint, PROGRAM_MAX> mProgramContrastCoef{};
	std::array<GLint, PROGRAM_MAX> mProgramGammaCoef{};
	std::array<GLint, PROGRAM_MAX> mProgramSatCoef{};
	std::array<GLint, PROGRAM_MAX> mProgramLightCoef{};
	std::array<GLint, PROGRAM_MAX> mProgramDarkCoef{};
	std::array<GLint, PROGRAM_MAX> mProgramZebraEnable{};
	std::array<GLint, PROGRAM_MAX> mProgramZebraThreshold{};
	std::array<GLint, PROGRAM_MAX> mProgramZebraPhase{};
	std::array<GLint, PROGRAM_MAX> mProgramZebraWeight{};
	std::array<GLint, PROGRAM_MAX> mProgramMbStatusEnable{};
	GLint mSimpleProgram = 0;
	GLint mSimpleProgramTransformMatrix = 0;
	GLint mSimpleProgramUniformSampler = 0;
	GLint mSimpleProgramPositionHandle = 0;
	GLint mSimpleProgramTexcoordHandle = 0;
	GLint mClearProgram = 0;
	GLint mClearProgramTransformMatrix = 0;
	GLint mClearProgramPositionHandle = 0;
	GLint mClearProgramTexcoordHandle = 0;
	GLint mClearProgramColor = 0;
	std::array<GLuint, GL_VIDEO_TEX_UNIT_COUNT> mTextures{};
	GLuint mMbStatusTexture = 0;
	GLuint mExtTexture = 0;
	std::array<std::array<GLint, GL_VIDEO_TEX_UNIT_COUNT>, PROGRAM_MAX>
		mUniformSamplers{};
	std::array<GLint, PROGRAM_MAX> mPositionHandle{};
	std::array<GLint, PROGRAM_MAX> mTexcoordHandle{};
	std::array<GLint, PROGRAM_MAX> mMbStatusUniformSampler{};
	bool mBlurInit = false;
	bool mApplyBlur = false;
	std::array<float, GL_VIDEO_BLUR_TAP_COUNT> mBlurWeights{};
	unsigned int mBlurFboWidth = 0;
	unsigned int mBlurFboHeight = 0;
	std::array<GLuint, 2> mBlurFbo{};
	std::array<GLuint, 2> mBlurFboTexture{};
	std::array<GLint, 2> mBlurProgram{};
	std::array<GLint, 2> mBlurUniformPixelSize{};
	std::array<GLint, 2> mBlurUniformWeights{};
	std::array<GLint, 2> mBlurUniformSampler{};
	std::array<GLint, 2> mBlurPositionHandle{};
	unsigned int mPaddingPass1Width = 0;
	unsigned int mPaddingPass1Height = 0;
	unsigned int mPaddingPass2Width = 0;
	unsigned int mPaddingPass2Height = 0;
	std::array<float, GL_VIDEO_BLUR_TAP_COUNT> mPaddingBlurWeights{};
	std::array<GLuint, 4> mPaddingFbo{};
	std::array<GLuint, 4> mPaddingFboTexture{};
	bool mHistogramInit = false;
	uint64_t mHistogramLastComputeTime = 0;
	std::array<GLint, PROGRAM_MAX> mHistogramProgram{};
	std::array<GLint, PROGRAM_MAX> mHistogramYuv2RgbMatrix{};
	std::array<GLint, PROGRAM_MAX> mHistogramYuv2RgbOffset{};
	std::array<GLint, PROGRAM_MAX> mHistogramRgb2LumaMatrix{};
	std::array<GLint, PROGRAM_MAX> mHistogramRgb2LumaOffset{};
	std::array<GLint, PROGRAM_MAX> mHistogramBrightnessCoef{};
	std::array<GLint, PROGRAM_MAX> mHistogramContrastCoef{};
	std::array<GLint, PROGRAM_MAX> mHistogramGammaCoef{};
	std::array<GLint, PROGRAM_MAX> mHistogramStride{};
	std::array<GLint, PROGRAM_MAX> mHistogramMaxCoordsRatio{};
	std::array<GLint, PROGRAM_MAX> mHistogramMaxClamp{};
	std::array<std::array<GLint, GL_VIDEO_TEX_UNIT_COUNT>, PROGRAM_MAX>
		mHistogramUniformSampler{};
	std::array<GLint, PROGRAM_MAX> mHistogramPositionHandle{};
	std::array<GLint, PROGRAM_MAX> mHistogramTexcoordHandle{};
	GLuint mHistogramFbo = 0;
	GLuint mHistogramFboTexture = 0;
	std::vector<uint8_t> mHistogramBuffer;
	std::array<bool, PDRAW_HISTOGRAM_CHANNEL_MAX> mHistogramValid{};
	std::array<std::array<uint32_t, 256>, PDRAW_HISTOGRAM_CHANNEL_MAX>
		mHistogram{};
	std::array<std::array<float, 256>, PDRAW_HISTOGRAM_CHANNEL_MAX>
		mHistogramNorm{};
	float mBrightnessCoef = 0.f;
	float mContrastCoef = 1.f;
	float mGammaCoef = 1.f;
	float mSatCoef = 1.f; /* 0.0 (greyscale) .. 1.0 (original video) */
	float mBaseSatCoef = 1.f;
	float mLightCoef = 1.f; /* 0.0 (white) .. 1.0 (video) */
	float mBaseLightCoef = 1.f;
	float mDarkCoef = 1.f; /* 0.0 (black) .. 1.0 (video) */
	float mBaseDarkCoef = 1.f;
	bool mHasMbStatus = false;

	static const std::
		array<std::array<std::array<const GLchar *, 5>, PROGRAM_MAX>, 2>
			videoFragmentShaders;
	static const std::array<std::array<const GLchar *, 3>, PROGRAM_MAX>
		histogramFragmentShaders;
};

} /* namespace Pdraw */

#endif /* PDRAW_USE_GL */
