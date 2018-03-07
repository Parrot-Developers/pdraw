/**
 * Parrot Drones Awesome Video Viewer Library
 * RaspberryPi application
 *
 * Copyright (c) 2016 Aurelien Barre
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PDRAW_RASPI_H_
#define _PDRAW_RASPI_H_

#include <pthread.h>
#include <libARNetwork/ARNetwork.h>
#include <libARNetworkAL/ARNetworkAL.h>
#include <libARCommands/ARCommands.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARDiscovery/ARDISCOVERY_AvahiDiscovery.h>
#include <pdraw/pdraw.h>
#include <json-c/json.h>
#include <curl/curl.h>
#include <png.h>
#include <bcm_host.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>


#define PDRAW_ARSDK_DISCOVERY_PORT 44444
#define PDRAW_ARSDK_D2C_PORT 43210

#define PDRAW_ARSDK_CD_NONACK_ID 10
#define PDRAW_ARSDK_CD_ACK_ID 11
#define PDRAW_ARSDK_CD_EMERGENCY_ID 12
#define PDRAW_ARSDK_DC_NAVDATA_ID 127
#define PDRAW_ARSDK_DC_EVENT_ID 126

#define PDRAW_ARSDK_VIDEO_LOCAL_STREAM_PORT 55004
#define PDRAW_ARSDK_VIDEO_LOCAL_CONTROL_PORT 55005


struct arcmd_reader_data;


struct ardiscovery_browser_device
{
    char *serviceName;
    char *serviceType;
    char *ipAddr;
    uint16_t port;
    eARDISCOVERY_PRODUCT product;

    struct ardiscovery_browser_device *prev;
    struct ardiscovery_browser_device *next;
};


struct arcmd_reader_data
{
    struct pdraw_app *app;
    int readerBufferId;
};


struct ui_background_layer
{
    int32_t layer;
    DISPMANX_RESOURCE_HANDLE_T resource;
    DISPMANX_ELEMENT_HANDLE_T element;
};


struct ui_image_layer
{
    uint8_t *buffer;
    int width;
    int height;
    int isRgba;
    int32_t layer;
    DISPMANX_RESOURCE_HANDLE_T resource;
    DISPMANX_ELEMENT_HANDLE_T element;
};


struct pdraw_app
{
    struct pdraw *pdraw;
    pthread_mutex_t pdrawMutex;
    pthread_cond_t pdrawCond;
    int pdrawRunning;
    int run;

    char url[500];
    char ipAddr[16];
    char ifaceAddr[16];
    int playRecord;
    int daemon;
    int arsdkBrowse;
    int arsdkConnect;
    int arsdkStartStream;
    int scRestream;
    int receiveStream;

    ARDISCOVERY_AvahiDiscovery_BrowserData_t *ardiscoveryBrowserData;
    ARNETWORKAL_Manager_t *arnetworkalManager;
    ARNETWORK_Manager_t *arnetworkManager;
    ARCOMMANDS_Decoder_t *arcmdDecoder;
    pthread_mutex_t ardiscoveryBrowserMutex;
    pthread_t ardiscoveryBrowserThread;
    int ardiscoveryBrowserThreadLaunched;
    pthread_t arnetworkRxThread;
    int arnetworkRxThreadLaunched;
    pthread_t arnetworkTxThread;
    int arnetworkTxThreadLaunched;
    pthread_t *arcmdReaderThreads;
    int *arcmdReaderThreadsLaunched;
    struct arcmd_reader_data *arcmdThreadData;
    int arsdkDiscoveryPort;
    int arsdkD2CPort;
    int arsdkC2DPort;

    struct ardiscovery_browser_device *ardiscoveryDeviceList;
    int ardiscoveryDeviceCount;

    int localStreamPort;
    int localControlPort;
    int remoteStreamPort;
    int remoteControlPort;
    int disconnected;

    uint32_t screenWidth;
    uint32_t screenHeight;
    DISPMANX_DISPLAY_HANDLE_T dispmanDisplay;
    DISPMANX_ELEMENT_HANDLE_T dispmanPdrawElement;
    struct ui_background_layer dispmanBackgroundLayer;
    struct ui_image_layer dispmanSplashLayer;
    EGLDisplay display;
    EGLSurface surface;
    EGLContext context;
};


int loadPngFile(const char *fileName, uint8_t **buffer, int *width, int *height, int *isRgba);
int startUi(struct pdraw_app *app);
void resetUi(struct pdraw_app *app);
void stopUi(struct pdraw_app *app);

int startPdraw(struct pdraw_app *app);
void stopPdraw(struct pdraw_app *app);
void pdrawStateChanged(struct pdraw *pdraw,
    enum pdraw_state state, void *userdata);
void pdrawOpenResp(struct pdraw *pdraw, int status, void *userdata);
void pdrawCloseResp(struct pdraw *pdraw, int status, void *userdata);
void pdrawPlayResp(struct pdraw *pdraw, int status,
    uint64_t timestamp, void *userdata);
void pdrawPauseResp(struct pdraw *pdraw, int status,
    uint64_t timestamp, void *userdata);
void pdrawSeekResp(struct pdraw *pdraw, int status,
    uint64_t timestamp, void *userdata);
void pdrawSocketCreated(struct pdraw *pdraw, int fd, void *userdata);

int startArdiscoveryBrowse(struct pdraw_app *app);
void stopArdiscoveryBrowse(struct pdraw_app *app);
eARDISCOVERY_ERROR ardiscoveryBrowserCallback(void* userdata, uint8_t state, const char* serviceName, const char* serviceType, const char* ipAddr, uint16_t port);
int ardiscoveryConnect(struct pdraw_app *app);
eARDISCOVERY_ERROR ardiscoveryConnectionSendJsonCallback(uint8_t *dataTx, uint32_t *dataTxSize, void *customData);
eARDISCOVERY_ERROR ardiscoveryConnectionReceiveJsonCallback(uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData);
int startArnetwork(struct pdraw_app *app);
void stopArnetwork(struct pdraw_app *app);
void arnetworkOnDisconnectCallback(ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *customData);
eARNETWORK_MANAGER_CALLBACK_RETURN arnetworkCmdCallback(int buffer_id, uint8_t *data, void *custom, eARNETWORK_MANAGER_CALLBACK_STATUS cause);
void *arnetworkCmdReaderRun(void* data);
int startArcommand(struct pdraw_app *app);
void stopArcommand(struct pdraw_app *app);

int sendDateAndTime(struct pdraw_app *app);
int sendAllStates(struct pdraw_app *app);
int sendStreamingVideoEnable(struct pdraw_app *app);

int skyControllerRestreamConnect(struct pdraw_app *app);


#endif /* !_PDRAW_RASPI_H_ */
