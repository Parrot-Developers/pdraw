/**
 * @file Pdraw.java
 * @brief Parrot Drones Awesome Video Viewer - Android Java implementation
 * @date 05/11/2016
 * @author aurelien.barre@akaaba.net
 *
 * Copyright (c) 2016 Aurelien Barre <aurelien.barre@akaaba.net>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *
 *   * Neither the name of the copyright holder nor the names of the
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package net.akaaba.libpdraw;

import android.view.Surface;
import com.parrot.mux.Mux;
import java.nio.ByteBuffer;

public class Pdraw {
    public static final float PDRAW_PLAY_SPEED_MAX = 1000.f;

    private static final String LIBRARY_NAME = "pdraw_android";
    private long pdrawCtx;
    private VideoFrameListener mListener;

    static {
        System.loadLibrary(LIBRARY_NAME);
    }

    public class DisplayScreenSettings {
        public DisplayScreenSettings() {}
        public float xdpi = 0.f;
        public float ydpi = 0.f;
        public float deviceMargin = 0.f;
    };

    public class HmdDistorsionCorrectionSettings {
        public HmdDistorsionCorrectionSettings() {}
        public int hmdModel= 0;
        public float ipd = 0.f;
        public float scale = 0.f;
        public float panH = 0.f;
        public float panV = 0.f;
    };

    public class Location {
        public Location() {}
        public boolean valid = false;
        public double latitude = 500.0;
        public double longitude = 500.0;
        public double altitude = 0.0;
        public int svCount = 0;
    };

    public class Quaternion {
        public Quaternion() {}
        public float w = 0.0f;
        public float x = 0.0f;
        public float y = 0.0f;
        public float z = 0.0f;
    };

    public class Euler {
        public Euler() {}
        public float phi = 0.0f;      /* roll */
        public float theta = 0.0f;    /* pitch */
        public float psi = 0.0f;      /* yaw */
    };

    public class CameraOrientation {
        public CameraOrientation() {}
        public float pan = 0.f;
        public float tilt = 0.f;
    };

    public class Speed {
        public Speed() {}
        public float north = 0.0f;
        public float east = 0.0f;
        public float down = 0.0f;
    };

    public static class VideoFrame {
        private int mColorFormat;
        private int mWidth;
        private int mHeight;
        private ByteBuffer mPixels;
        private int mStride;
        private long mTimestamp;
        private long mUserData;
        private int mUserDataSize;

        private VideoFrame(int colorFormat, int width, int height, ByteBuffer pixels, int stride, long timestamp, long userData, int userDataSize) {
            mColorFormat = colorFormat;
            mWidth = width;
            mHeight = height;
            mPixels = pixels;
            mStride = stride;
            mTimestamp = timestamp;
            mUserData = userData;
            mUserDataSize = userDataSize;
        }

        public int getColorFormat() {
            return mColorFormat;
        }

        public int getWidth() {
            return mWidth;
        }

        public int getHeight() {
            return mHeight;
        }

        public ByteBuffer getPixels() {
            return mPixels;
        }

        public int getStride() {
            return mStride;
        }

        public long getTimestamp() {
            return mTimestamp;
        }

        public long getUserData() {
            return mUserData;
        }

        public int getUserDataSize() {
            return mUserDataSize;
        }
    }

    public interface VideoFrameListener {
        public void onFrameReceived(VideoFrame frame);
    }


    public Pdraw() {
        this.pdrawCtx = nativeNew();
    }

    private boolean isValid() {
        return pdrawCtx != 0;
    }

    public void destroy() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeDispose(pdrawCtx);
    }

    public void open(String url) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeOpenUrl(pdrawCtx, url);
    }

    public void open(String url, String ifaceAddr) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeOpenUrlMcast(pdrawCtx, url, ifaceAddr);
    }

    public void open(
        String srcAddr,
        String ifaceAddr,
        int srcStreamPort,
        int srcControlPort,
        int dstStreamPort,
        int dstControlPort,
        int qosMode) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeOpenSingleStream(pdrawCtx, srcAddr, ifaceAddr,
            srcStreamPort, srcControlPort,
            dstStreamPort, dstControlPort, qosMode);
    }

    public void open(Mux mux) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        Mux.Ref muxRef = mux.newMuxRef();
        nativeOpenMux(pdrawCtx, muxRef.getCPtr());
        muxRef.release();
    }

    public void openSdp(String sdp, String ifaceAddr) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeOpenSdp(pdrawCtx, sdp, ifaceAddr);
    }

    public void play() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativePlay(pdrawCtx);
    }

    public void playWithSpeed(float speed) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativePlayWithSpeed(pdrawCtx, speed);
    }

    public void pause() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativePause(pdrawCtx);
    }

    public boolean isPaused() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        int ret = nativeIsPaused(pdrawCtx);
        return (ret == 1) ? true : false;
    }

    public void previousFrame() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativePreviousFrame(pdrawCtx);
    }

    public void nextFrame() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeNextFrame(pdrawCtx);
    }

    public void stop() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeStop(pdrawCtx);
    }

    public void seekTo(long timestamp, boolean exact) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSeekTo(pdrawCtx, timestamp, exact);
    }

    public void seekForward(long delta, boolean exact) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSeekForward(pdrawCtx, delta, exact);
    }

    public void seekBack(long delta, boolean exact) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSeekBack(pdrawCtx, delta, exact);
    }

    public long getDuration() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetDuration(pdrawCtx);
    }

    public long getCurrentTime() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetCurrentTime(pdrawCtx);
    }

    public void startRecorder(String fileName) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeStartRecorder(pdrawCtx, fileName);
    }

    public void stopRecorder() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeStopRecorder(pdrawCtx);
    }

    public void startResender(
        String dstAddr,
        String ifaceAddr,
        int srcStreamPort,
        int srcControlPort,
        int dstStreamPort,
        int dstControlPort) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeStartResender(pdrawCtx, dstAddr, ifaceAddr,
            srcStreamPort, srcControlPort, dstStreamPort, dstControlPort);
    }

    public void stopResender() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeStopResender(pdrawCtx);
    }

    public void startRenderer(
        int renderX,
        int renderY,
        int renderWidth,
        int renderHeight,
        boolean hmdDistorsionCorrection,
        boolean headtracking,
        Surface surface) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeStartRenderer(pdrawCtx, renderX, renderY,
            renderWidth, renderHeight, hmdDistorsionCorrection,
            headtracking, surface);
    }

    public void stopRenderer() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeStopRenderer(pdrawCtx);
    }

    public void render(long lastRenderTime) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeRender(pdrawCtx, lastRenderTime);
    }

    public void setVideoFrameListener(VideoFrameListener listener) {
        VideoFrameListener old = mListener;
        mListener = listener;
        if (old == null) {
            nativeRegisterListener(pdrawCtx);
        }
        if (mListener == null) {
            nativeUnregisterListener(pdrawCtx);
        }
    }

    public String getSelfFriendlyName() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetSelfFriendlyName(pdrawCtx);
    }

    public void setSelfFriendlyName(String friendlyName) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetSelfFriendlyName(pdrawCtx, friendlyName);
    }

    public String getSelfSerialNumber() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetSelfSerialNumber(pdrawCtx);
    }

    public void setSelfSerialNumber(String serialNumber) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetSelfSerialNumber(pdrawCtx, serialNumber);
    }

    public String getSelfSoftwareVersion() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetSelfSoftwareVersion(pdrawCtx);
    }

    public void setSelfSoftwareVersion(String softwareVersion) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetSelfSoftwareVersion(pdrawCtx, softwareVersion);
    }

    public boolean isSelfPilot() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        int ret = nativeIsSelfPilot(pdrawCtx);
        return (ret == 1) ? true : false;
    }

    public void setSelfPilot(boolean isPilot) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetSelfPilot(pdrawCtx, isPilot);
    }

    public Location getSelfLocation() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetSelfLocation(pdrawCtx);
    }

    public void setSelfLocation(Location loc) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetSelfLocation(pdrawCtx, loc);
    }

    public int getSelfControllerBatteryLevel() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetSelfControllerBatteryLevel(pdrawCtx);
    }

    public void setSelfControllerBatteryLevel(int batteryLevel) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetSelfControllerBatteryLevel(pdrawCtx, batteryLevel);
    }

    public Quaternion getSelfControllerOrientationQuat() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetSelfControllerOrientationQuat(pdrawCtx);
    }

    public Euler getSelfControllerOrientationEuler() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetSelfControllerOrientationEuler(pdrawCtx);
    }

    public void setSelfControllerOrientation(Quaternion quat) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetSelfControllerOrientationQuat(pdrawCtx, quat);
    }

    public void setSelfControllerOrientation(Euler euler) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetSelfControllerOrientationEuler(pdrawCtx, euler);
    }

    public Quaternion getSelfHeadOrientationQuat() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetSelfHeadOrientationQuat(pdrawCtx);
    }

    public Euler getSelfHeadOrientationEuler() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetSelfHeadOrientationEuler(pdrawCtx);
    }

    public void setSelfHeadOrientation(Quaternion quat) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetSelfHeadOrientationQuat(pdrawCtx, quat);
    }

    public void setSelfHeadOrientation(Euler euler) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetSelfHeadOrientationEuler(pdrawCtx, euler);
    }

    public Quaternion getSelfHeadRefOrientationQuat() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetSelfHeadRefOrientationQuat(pdrawCtx);
    }

    public Euler getSelfHeadRefOrientationEuler() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetSelfHeadRefOrientationEuler(pdrawCtx);
    }

    public void setSelfHeadRefOrientation(Quaternion quat) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetSelfHeadRefOrientationQuat(pdrawCtx, quat);
    }

    public void setSelfHeadRefOrientation(Euler euler) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetSelfHeadRefOrientationEuler(pdrawCtx, euler);
    }

    public void resetSelfHeadRefOrientation() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeResetSelfHeadRefOrientation(pdrawCtx);
    }

    public String getPeerFriendlyName() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerFriendlyName(pdrawCtx);
    }

    public String getPeerMaker() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerMaker(pdrawCtx);
    }

    public String getPeerModel() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerModel(pdrawCtx);
    }

    public String getPeerModelId() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerModelId(pdrawCtx);
    }

    public String getPeerSerialNumber() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerSerialNumber(pdrawCtx);
    }

    public String getPeerSoftwareVersion() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerSoftwareVersion(pdrawCtx);
    }

    public String getPeerBuildId() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerFriendlyName(pdrawCtx);
    }

    public String getPeerTitle() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerFriendlyName(pdrawCtx);
    }

    public String getPeerComment() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerFriendlyName(pdrawCtx);
    }

    public String getPeerCopyright() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerFriendlyName(pdrawCtx);
    }

    public String getPeerRunDate() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerFriendlyName(pdrawCtx);
    }

    public String getPeerRunUuid() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerFriendlyName(pdrawCtx);
    }

    public String getPeerMediaDate() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerFriendlyName(pdrawCtx);
    }

    public Location getPeerTakeoffLocation() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerTakeoffLocation(pdrawCtx);
    }

    public void setPeerTakeoffLocation(Location loc) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetPeerTakeoffLocation(pdrawCtx, loc);
    }

    public Location getPeerHomeLocation() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerHomeLocation(pdrawCtx);
    }

    public void setPeerHomeLocation(Location loc) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetPeerHomeLocation(pdrawCtx, loc);
    }

    public long getPeerRecordingDuration() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetPeerRecordingDuration(pdrawCtx);
    }

    public void setPeerRecordingDuration(long duration) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetPeerRecordingDuration(pdrawCtx, duration);
    }

    public CameraOrientation getCameraOrientationForHeadtracking() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetCameraOrientationForHeadtracking(pdrawCtx);
    }

    public float getControllerRadarAngleSetting() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetControllerRadarAngleSetting(pdrawCtx);
    }

    public void setControllerRadarAngleSetting(float angle) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetControllerRadarAngleSetting(pdrawCtx, angle);
    }

    public DisplayScreenSettings getDisplayScreenSettings() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetDisplayScreenSettings(pdrawCtx);
    }

    public void setDisplayScreenSettings(DisplayScreenSettings scr) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetDisplayScreenSettings(pdrawCtx, scr);
    }

    public HmdDistorsionCorrectionSettings getHmdDistorsionCorrectionSettings() {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        return nativeGetHmdDistorsionCorrectionSettings(pdrawCtx);
    }

    public void setHmdDistorsionCorrectionSettings(HmdDistorsionCorrectionSettings hmd) {
        if (!isValid()) {
            throw new RuntimeException("invalid pdraw instance");
        }
        nativeSetHmdDistorsionCorrectionSettings(pdrawCtx, hmd);
    }


    private native long nativeNew();

    private native int nativeDispose(
        long pdrawCtx);

    private native int nativeOpenUrl(
        long pdrawCtx,
        String url);

    private native int nativeOpenUrlMcast(
        long pdrawCtx,
        String url,
        String ifaceAddr);

    private native int nativeOpenSingleStream(
        long pdrawCtx,
        String srcAddr,
        String ifaceAddr,
        int srcStreamPort,
        int srcControlPort,
        int dstStreamPort,
        int dstControlPort,
        int qosMode);

    private native int nativeOpenMux(
        long pdrawCtx,
        long mux);

    private native int nativeOpenSdp(
        long pdrawCtx,
        String sdp,
        String ifaceAddr);

    private native int nativePlay(
        long pdrawCtx);

    private native int nativePlayWithSpeed(
        long pdrawCtx,
        float speed);

    private native int nativePause(
        long pdrawCtx);

    private native int nativeIsPaused(
        long pdrawCtx);

    private native int nativePreviousFrame(
        long pdrawCtx);

    private native int nativeNextFrame(
        long pdrawCtx);

    private native int nativeStop(
        long pdrawCtx);

    private native int nativeSeekTo(
        long pdrawCtx,
        long timestamp,
        boolean exact);

    private native int nativeSeekForward(
        long pdrawCtx,
        long delta,
        boolean exact);

    private native int nativeSeekBack(
        long pdrawCtx,
        long delta,
        boolean exact);

    private native long nativeGetDuration(
        long pdrawCtx);

    private native long nativeGetCurrentTime(
        long pdrawCtx);

    private native int nativeStartRecorder(
        long pdrawCtx,
        String fileName);

    private native int nativeStopRecorder(
        long pdrawCtx);

    private native int nativeStartResender(
        long pdrawCtx,
        String dstAddr,
        String ifaceAddr,
        int srcStreamPort,
        int srcControlPort,
        int dstStreamPort,
        int dstControlPort);

    private native int nativeStopResender(
        long pdrawCtx);

    private native int nativeStartRenderer(
        long pdrawCtx,
        int renderX,
        int renderY,
        int renderWidth,
        int renderHeight,
        boolean hmdDistorsionCorrection,
        boolean headtracking,
        Surface surface);

    private native int nativeStopRenderer(
        long pdrawCtx);

    private native int nativeRender(
        long pdrawCtx,
        long lastRenderTime);

    private native String nativeGetSelfFriendlyName(
        long pdrawCtx);
    private native int nativeSetSelfFriendlyName(
        long pdrawCtx,
        String friendlyName);

    private native String nativeGetSelfSerialNumber(
        long pdrawCtx);
    private native int nativeSetSelfSerialNumber(
        long pdrawCtx,
        String serialNumber);

    private native String nativeGetSelfSoftwareVersion(
        long pdrawCtx);
    private native int nativeSetSelfSoftwareVersion(
        long pdrawCtx,
        String softwareVersion);

    private native int nativeIsSelfPilot(
        long pdrawCtx);
    private native int nativeSetSelfPilot(
        long pdrawCtx,
        boolean isPilot);

    private native Location nativeGetSelfLocation(
        long pdrawCtx);
    private native int nativeSetSelfLocation(
        long pdrawCtx,
        Location loc);

    private native int nativeGetSelfControllerBatteryLevel(
        long pdrawCtx);
    private native int nativeSetSelfControllerBatteryLevel(
        long pdrawCtx,
        int batteryLevel);

    private native Quaternion nativeGetSelfControllerOrientationQuat(
        long pdrawCtx);
    private native Euler nativeGetSelfControllerOrientationEuler(
        long pdrawCtx);
    private native int nativeSetSelfControllerOrientationQuat(
        long pdrawCtx,
        Quaternion quat);
    private native int nativeSetSelfControllerOrientationEuler(
        long pdrawCtx,
        Euler euler);

    private native Quaternion nativeGetSelfHeadOrientationQuat(
        long pdrawCtx);
    private native Euler nativeGetSelfHeadOrientationEuler(
        long pdrawCtx);
    private native int nativeSetSelfHeadOrientationQuat(
        long pdrawCtx,
        Quaternion quat);
    private native int nativeSetSelfHeadOrientationEuler(
        long pdrawCtx,
        Euler euler);

    private native Quaternion nativeGetSelfHeadRefOrientationQuat(
        long pdrawCtx);
    private native Euler nativeGetSelfHeadRefOrientationEuler(
        long pdrawCtx);
    private native int nativeSetSelfHeadRefOrientationQuat(
        long pdrawCtx,
        Quaternion quat);
    private native int nativeSetSelfHeadRefOrientationEuler(
        long pdrawCtx,
        Euler euler);
    private native int nativeResetSelfHeadRefOrientation(
        long pdrawCtx);

    private native String nativeGetPeerFriendlyName(
        long pdrawCtx);

    private native String nativeGetPeerMaker(
        long pdrawCtx);

    private native String nativeGetPeerModel(
        long pdrawCtx);

    private native String nativeGetPeerModelId(
        long pdrawCtx);

    private native String nativeGetPeerSerialNumber(
        long pdrawCtx);

    private native String nativeGetPeerSoftwareVersion(
        long pdrawCtx);

    private native String nativeGetPeerBuildId(
        long pdrawCtx);

    private native String nativeGetPeerTitle(
        long pdrawCtx);

    private native String nativeGetPeerComment(
        long pdrawCtx);

    private native String nativeGetPeerCopyright(
        long pdrawCtx);

    private native String nativeGetPeerRunDate(
        long pdrawCtx);

    private native String nativeGetPeerRunUuid(
        long pdrawCtx);

    private native String nativeGetPeerMediaDate(
        long pdrawCtx);

    private native Location nativeGetPeerTakeoffLocation(
        long pdrawCtx);
    private native int nativeSetPeerTakeoffLocation(
        long pdrawCtx,
        Location loc);

    private native Location nativeGetPeerHomeLocation(
        long pdrawCtx);
    private native int nativeSetPeerHomeLocation(
        long pdrawCtx,
        Location loc);

    private native long nativeGetPeerRecordingDuration(
        long pdrawCtx);
    private native int nativeSetPeerRecordingDuration(
        long pdrawCtx,
        long duration);

    private native CameraOrientation nativeGetCameraOrientationForHeadtracking(
        long pdrawCtx);

/* TODO
    private native int nativeGetMediaCount(
        long pdrawCtx);

    private native int nativeGetMediaInfo(
        long pdrawCtx,
        unsigned int index,
        struct pdraw_media_info *info);

    private native long nativeAddVideoFrameFilterCallback(
        long pdrawCtx,
        unsigned int mediaId,
        pdraw_video_frame_filter_callback_t cb,
        void *userPtr);

    private native int nativeRemoveVideoFrameFilterCallback(
        long pdrawCtx,
        unsigned int mediaId,
        long filterCtx);

    private native long nativeAddVideoFrameProducer(
        long pdrawCtx,
        unsigned int mediaId);

    private native int nativeRemoveVideoFrameProducer(
        long pdrawCtx,
        long producerCtx);

    private native int nativeGetProducerLastFrame(
        long pdrawCtx,
        long producerCtx,
        struct pdraw_video_frame *frame,
        int timeout);
*/

    private native float nativeGetControllerRadarAngleSetting(
        long pdrawCtx);
    private native int nativeSetControllerRadarAngleSetting(
        long pdrawCtx,
        float angle);

    private native DisplayScreenSettings nativeGetDisplayScreenSettings(
        long pdrawCtx);
    private native int nativeSetDisplayScreenSettings(
        long pdrawCtx,
        DisplayScreenSettings scr);

    private native HmdDistorsionCorrectionSettings nativeGetHmdDistorsionCorrectionSettings(
        long pdrawCtx);
    private native int nativeSetHmdDistorsionCorrectionSettings(
        long pdrawCtx,
        HmdDistorsionCorrectionSettings hmd);

    private void notifyNewFrame(VideoFrame frame) {
       if (mListener != null) {
           mListener.onFrameReceived(frame);
       }
    }

    private native void nativeRegisterListener(long pdrawCtx);

    private native void nativeUnregisterListener(long pdrawCtx);


}
