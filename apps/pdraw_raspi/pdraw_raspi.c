/**
 * @file pdraw_raspi.c
 * @brief Parrot Drones Awesome Video Viewer RaspberryPi Application
 * @date 05/12/2016
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

#include "pdraw_raspi.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <getopt.h>

#define ULOG_TAG pdraw_app
#include <ulog.h>
ULOG_DECLARE_TAG(pdraw_app);


static const char short_options[] = "hdf:bk:K:i:m:s:c:S:C:n:";


static const struct option long_options[] =
{
    { "help"            , no_argument        , NULL, 'h' },
    { "daemon"          , no_argument        , NULL, 'd' },
    { "file"            , required_argument  , NULL, 'f' },
    { "arsdk-browse"    , no_argument        , NULL, 'b' },
    { "arsdk"           , required_argument  , NULL, 'k' },
    { "arsdk-start"     , required_argument  , NULL, 'K' },
    { "ip"              , required_argument  , NULL, 'i' },
    { "miface"          , required_argument  , NULL, 'm' },
    { "srcstrmp"        , required_argument  , NULL, 's' },
    { "srcctrlp"        , required_argument  , NULL, 'c' },
    { "dststrmp"        , required_argument  , NULL, 'S' },
    { "dstctrlp"        , required_argument  , NULL, 'C' },
    { "screstream"      , required_argument  , NULL, 'n' },
    { 0, 0, 0, 0 }
};


static ARNETWORK_IOBufferParam_t c2dParams[] =
{
    /* Non-acknowledged commands. */
    {
        .ID = PDRAW_ARSDK_CD_NONACK_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 2,
        .dataCopyMaxSize = 128,
        .isOverwriting = 1,
    },
    /* Acknowledged commands. */
    {
        .ID = PDRAW_ARSDK_CD_ACK_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = 500,
        .numberOfRetry = 3,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    /* Emergency commands. */
    {
        .ID = PDRAW_ARSDK_CD_EMERGENCY_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 10,
        .ackTimeoutMs = 100,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 1,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
};

static const size_t c2dParamsCount = sizeof(c2dParams) / sizeof(ARNETWORK_IOBufferParam_t);


static ARNETWORK_IOBufferParam_t d2cParams[] =
{
    {
        .ID = PDRAW_ARSDK_DC_NAVDATA_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfRetry = ARNETWORK_IOBUFFERPARAM_INFINITE_NUMBER,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
    {
        .ID = PDRAW_ARSDK_DC_EVENT_ID,
        .dataType = ARNETWORKAL_FRAME_TYPE_DATA_WITH_ACK,
        .sendingWaitTimeMs = 20,
        .ackTimeoutMs = 500,
        .numberOfRetry = 3,
        .numberOfCell = 20,
        .dataCopyMaxSize = 128,
        .isOverwriting = 0,
    },
};

static const size_t d2cParamsCount = sizeof(d2cParams) / sizeof(ARNETWORK_IOBufferParam_t);


static int commandBufferIds[] = {
    PDRAW_ARSDK_DC_NAVDATA_ID,
    PDRAW_ARSDK_DC_EVENT_ID,
};

static const size_t commandBufferIdsCount = sizeof(commandBufferIds) / sizeof(int);


static int stopping = 0;


static void sighandler(int signum)
{
    printf("Stoping PDrAW...\n");
    ULOGI("Stopping...");
    stopping = 1;
    signal(SIGINT, SIG_DFL);
}


static void welcome()
{
    printf("  ___ ___       ___      __ \n");
    printf(" | _ \\   \\ _ _ /_\\ \\    / / \n");
    printf(" |  _/ |) | '_/ _ \\ \\/\\/ /  \n");
    printf(" |_| |___/|_|/_/ \\_\\_/\\_/   \n");
    printf("\nParrot Drones Awesome Video Viewer\n\n");
}


static void usage(int argc, char *argv[])
{
    printf("Usage: %s [options]\n"
            "Options:\n"
            "-h | --help                        Print this message\n"
            "-d | --daemon                      Daemon mode (wait for available connection)\n"
            "-f | --file <file_name>            Offline MP4 file playing\n"
            "-b | --arsdk-browse                Browse for ARSDK devices (discovery)\n"
            "-k | --arsdk <ip_address>          ARSDK connection to drone with its IP address\n"
            "-K | --arsdk-start <ip_address>    ARSDK connection to drone with its IP address (connect only, do not process the stream)\n"
            "-i | --ip <ip_address>             Direct RTP/AVP H.264 reception with an IP address (ports must also be configured)\n"
            "-m | --miface <ip_address>         Multicast interface address (only in case of multicast reception)\n"
            "-s | --srcstrmp <port>             Source stream port for direct RTP/AVP reception\n"
            "-c | --srcctrlp <port>             Source control port for direct RTP/AVP reception\n"
            "-S | --dststrmp <port>             Destination stream port for direct RTP/AVP reception\n"
            "-C | --dstctrlp <port>             Destination control port for direct RTP/AVP reception\n"
            "-n | --screstream <ip_address>     Connexion to a RTP restream from a SkyController\n"
            "\n",
            argv[0]);
}


static void summary(struct pdraw_app* app, int afterBrowse)
{
    if ((app->daemon) && (!afterBrowse))
    {
        printf("Daemon mode. Waiting for connection...\n\n");
    }
    else if ((app->arsdkBrowse) && (!afterBrowse))
    {
        printf("Browse for ARSDK devices (discovery)\n\n");
    }
    else if (app->scRestream)
    {
        printf("Connection to a restream from SkyController (address %s)\n\n", app->ipAddr);
    }
    else if (app->arsdkConnect)
    {
        printf("ARSDK connection to address %s", app->ipAddr);
        if (!app->receiveStream) printf(" (start stream only)");
        printf("\n\n");
    }
    else if (app->receiveStream)
    {
        printf("Direct RTP/AVP H.264 reception from address %s\n", app->ipAddr);
        printf("source ports: %d, %d | destination ports: %d, %d\n\n",
               app->srcStreamPort, app->srcControlPort, app->dstStreamPort, app->dstControlPort);
    }
    else if (app->playRecord)
    {
        printf("Offline playing of MP4 file '%s'\n\n", app->url);
    }
    else
    {
        printf("Nothing to do...\n\n");
    }
}


int main(int argc, char *argv[])
{
    int failed = 0, loop = 1;
    int idx, c;
    struct pdraw_app *app;

    welcome();

    if (argc < 2)
    {
        usage(argc, argv);
        exit(EXIT_FAILURE);
    }

    app = (struct pdraw_app*)malloc(sizeof(struct pdraw_app));
    if (app != NULL)
    {
        /* Initialize configuration */
        memset(app, 0, sizeof(struct pdraw_app));
        app->arsdkDiscoveryPort = PDRAW_ARSDK_DISCOVERY_PORT;
        app->arsdkD2CPort = PDRAW_ARSDK_D2C_PORT;
        app->arsdkC2DPort = 0; // Will be read from json
        app->dstStreamPort = PDRAW_ARSDK_VIDEO_DST_STREAM_PORT;
        app->dstControlPort = PDRAW_ARSDK_VIDEO_DST_CONTROL_PORT;
        app->run = 1;
    }
    else
    {
        failed = 1;
        ULOGE("pdraw app alloc error!");
    }

    if (!failed)
    {
        /* Command-line parameters */
        while ((c = getopt_long(argc, argv, short_options, long_options, &idx)) != -1)
        {
            switch (c)
            {
                case 0:
                    break;

                case 'h':
                    usage(argc, argv);
                    free(app);
                    exit(EXIT_SUCCESS);
                    break;

                case 'd':
                    app->daemon = 1;
                    app->arsdkBrowse = 1;
                    break;

                case 'n':
                    strncpy(app->ipAddr, optarg, sizeof(app->ipAddr));
                    app->scRestream = 1;
                    app->receiveStream = 1;
                    break;

                case 'b':
                    app->arsdkBrowse = 1;
                    break;

                case 'k':
                    strncpy(app->ipAddr, optarg, sizeof(app->ipAddr));
                    app->arsdkConnect = 1;
                    app->arsdkStartStream = 1;
                    app->receiveStream = 1;
                    break;

                case 'K':
                    strncpy(app->ipAddr, optarg, sizeof(app->ipAddr));
                    app->arsdkConnect = 1;
                    app->arsdkStartStream = 1;
                    break;

                case 'i':
                    strncpy(app->ipAddr, optarg, sizeof(app->ipAddr));
                    app->receiveStream = 1;
                    break;

                case 'm':
                    strncpy(app->ifaceAddr, optarg, sizeof(app->ifaceAddr));
                    break;

                case 'f':
                    strncpy(app->url, optarg, sizeof(app->url));
                    app->playRecord = 1;
                    break;

                case 's':
                    sscanf(optarg, "%d", &app->srcStreamPort);
                    break;

                case 'c':
                    sscanf(optarg, "%d", &app->srcControlPort);
                    break;

                case 'S':
                    sscanf(optarg, "%d", &app->dstStreamPort);
                    break;

                case 'C':
                    sscanf(optarg, "%d", &app->dstControlPort);
                    break;

                default:
                    usage(argc, argv);
                    free(app);
                    exit(EXIT_FAILURE);
                    break;
            }
        }
    }

    if ((!app->daemon) && (!app->arsdkBrowse) && (((!app->ipAddr) || (!strlen(app->ipAddr)))
            && ((!app->url) || (!strlen(app->url)))))
    {
        failed = 1;
        fprintf(stderr, "Invalid address or file name\n\n");
        usage(argc, argv);
        ULOGE("invalid address or file name!");
    }

    if (!failed)
    {
        summary(app, 0);

        printf("Starting PDrAW...\n");
        ULOGI("Starting...");

        signal(SIGINT, sighandler);
    }

    if ((!failed) && (app->daemon))
    {
        pid_t pid = fork();
        if (pid == 0)
        {
            ULOGI("successfully forked");
        }
        else if (pid > 0)
        {
            ULOGI("terminating parent process");
            free(app);
            exit(EXIT_SUCCESS);
        }
        else
        {
            ULOGE("fork() failed");
            failed = 1;
        }
    }

    if (!failed)
    {
        failed = startUi(app);
    }

    while ((!failed) && (!stopping) && (loop))
    {
        app->disconnected = 0;

        if ((!failed) && (app->arsdkBrowse))
        {
            failed = startArdiscoveryBrowse(app);

            int selected = 0;

            while ((!failed) && (!stopping) && (!selected))
            {
                int idx = -1;
                //TODO: catch keyboard press

                pthread_mutex_lock(&app->ardiscoveryBrowserMutex);

                if (app->daemon)
                {
                    idx = 0;
                }

                if ((idx >= 0) && (idx < app->ardiscoveryDeviceCount))
                {
                    struct ardiscovery_browser_device *device;
                    int i;
                    for (device = app->ardiscoveryDeviceList, i = 0; device && i < idx; device = device->next, i++);

                    if ((device) && (i == idx))
                    {
                        switch (device->product)
                        {
                            case ARDISCOVERY_PRODUCT_ARDRONE:
                            case ARDISCOVERY_PRODUCT_BEBOP_2:
                            case ARDISCOVERY_PRODUCT_EVINRUDE:
                            case ARDISCOVERY_PRODUCT_SKYCONTROLLER:
                                strncpy(app->ipAddr, device->ipAddr, sizeof(app->ipAddr));
                                app->arsdkDiscoveryPort = device->port;
                                app->arsdkConnect = 1;
                                app->arsdkStartStream = 1;
                                app->receiveStream = 1;
                                selected = 1;
                                break;
                            case ARDISCOVERY_PRODUCT_SKYCONTROLLER_NG:
                            case ARDISCOVERY_PRODUCT_SKYCONTROLLER_2:
                                strncpy(app->ipAddr, device->ipAddr, sizeof(app->ipAddr));
                                app->scRestream = 1;
                                app->receiveStream = 1;
                                selected = 1;
                                break;
                            default:
                                break;
                        }
                    }
                }

                pthread_mutex_unlock(&app->ardiscoveryBrowserMutex);

                sleep(1);
            }

            if (selected)
            {
                ARDISCOVERY_AvahiDiscovery_StopBrowsing(app->ardiscoveryBrowserData);
                summary(app, 1);
            }
            else
            {
                failed = 1;
            }
        }

        if ((!failed) && (app->scRestream))
        {
            failed = skyControllerRestreamConnect(app);
        }

        if (app->arsdkConnect)
        {
            if (!failed)
            {
                failed = ardiscoveryConnect(app);
            }

            if (!failed)
            {
                failed = startArnetwork(app);
            }
        }

        if ((!failed) && ((app->receiveStream) || (app->playRecord)))
        {
            failed = startPdraw(app);
        }

        if (app->arsdkConnect)
        {
            if (!failed)
            {
                int cmdSend = 0;

                cmdSend = sendDateAndTime(app);
            }

            if (!failed)
            {
                int cmdSend = 0;

                cmdSend = sendAllStates(app);
            }

            if ((!failed) && (app->arsdkStartStream))
            {
                int cmdSend = 0;

                cmdSend = sendStreamingVideoEnable(app);
            }

            if (!failed)
            {
                failed = startArcommand(app);
            }
        }

        if (!failed)
        {
            printf("Rock'n'roll!\n");
            ULOGI("Running...");
        }

        /* Run until interrupted */
        while ((!failed) && (!stopping) && (!app->disconnected))
        {
            if (app->pdraw)
            {
                int ret = pdraw_render(app->pdraw, 0);
                if (ret != 0)
                {
                    ULOGE("pdraw_render() failed (%d)", ret);
                    failed = 1;
                }
            }
        }

        printf("Closing PDrAW...\n");
        ULOGI("Closing...");

        if (app != NULL)
        {
            app->run = 0; // break threads loops

            resetUi(app);
            stopPdraw(app);
            stopArcommand(app);
            stopArnetwork(app);
            stopArdiscoveryBrowse(app);
        }

        if ((stopping) || (!app->daemon))
        {
            loop = 0;
        }
        else
        {
            failed = 0;
        }
    }

    printf("Terminating PDrAW...\n");
    ULOGI("Terminating...");

    if (app != NULL)
    {
        app->run = 0; // break threads loops

        stopUi(app);
        free(app);
    }

    printf("Hasta la vista, PDrAW!\n");
    ULOGI("Finished");

    exit(EXIT_SUCCESS);
}


int loadPngFile(const char *fileName, uint8_t **buffer, int *width, int *height, int *isRgba)
{
    int ret = 0;
    png_byte header[8]; // 8 is the maximum size that can be checked
    FILE *fp = NULL;
    uint8_t *raw_image = NULL;
    png_bytep *row_pointers = NULL;
    int w = 0, h = 0;
    png_structp png_ptr = NULL;
    png_infop info_ptr = NULL;
    int rgba = 0;

    if ((!fileName) || (!buffer) || (!width) || (!height) || (!isRgba))
    {
        ULOGE("invalid pointer");
        return -1;
    }

    /* open file and test for it being a png */
    if (ret == 0)
    {
        fp = fopen(fileName, "rb");
        if (!fp)
        {
            ULOGE("failed to open file '%s'", fileName);
            ret = -1;
        }
    }

    if (ret == 0)
    {
        fread(header, 1, 8, fp);
        if (png_sig_cmp(header, 0, 8))
        {
            ULOGE("file '%s' is not recognized as a PNG file", fileName);
            ret = -1;
        }
    }

    /* initialize stuff */
    if (ret == 0)
    {
        png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
        if (!png_ptr)
        {
            ULOGE("png_create_read_struct() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        info_ptr = png_create_info_struct(png_ptr);
        if (!info_ptr)
        {
            ULOGE("png_create_info_struct() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        if (setjmp(png_jmpbuf(png_ptr)))
        {
            ULOGE("error during init_io()");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        png_init_io(png_ptr, fp);
        png_set_sig_bytes(png_ptr, 8);

        png_read_info(png_ptr, info_ptr);

        w = png_get_image_width(png_ptr, info_ptr);
        h = png_get_image_height(png_ptr, info_ptr);
        png_byte color_type = png_get_color_type(png_ptr, info_ptr);
        png_byte bit_depth = png_get_bit_depth(png_ptr, info_ptr);

        if (color_type & PNG_COLOR_MASK_ALPHA)
        {
            rgba = 1;
        }

        if (color_type == PNG_COLOR_TYPE_PALETTE)
        {
            png_set_palette_to_rgb(png_ptr);
        }
        /* Expand grayscale images to the full 8 bits from 1, 2, or 4 bits/pixel */
        if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
        {
            png_set_expand_gray_1_2_4_to_8(png_ptr);
        }

        /* Expand paletted or RGB images with transparency to full alpha channels
         * so the data will be available as RGBA quartets. */
        if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS))
        {
            png_set_tRNS_to_alpha(png_ptr);
        }

        /* Round to 8 bit */
        if (bit_depth == 16)
        {
#ifdef PNG_READ_SCALE_16_TO_8_SUPPORTED
            png_set_scale_16(png_ptr);
#else
            png_set_strip_16(png_ptr);
#endif
        }

        png_read_update_info(png_ptr, info_ptr);

        if (setjmp(png_jmpbuf(png_ptr)))
        {
            ULOGE("error during read_image()");
        }
    }

    if (ret == 0)
    {
        raw_image = (uint8_t*)malloc(png_get_rowbytes(png_ptr, info_ptr) * h);
        if (!raw_image)
        {
            ULOGE("allocation failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * h);
        if (!row_pointers)
        {
            ULOGE("allocation failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        int y;
        for (y = 0; y < h; y++)
        {
            row_pointers[y] = (png_byte*)(raw_image + y * png_get_rowbytes(png_ptr,info_ptr));
        }
    }

    /* read file */
    if (ret == 0)
    {
        png_read_image(png_ptr, row_pointers);
    }

    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
    fclose(fp);
    free(row_pointers);

    *width = (int)w;
    *height = (int)h;
    *buffer = raw_image;
    *isRgba = rgba;
    return 0;
}


int startUi(struct pdraw_app *app)
{
    int ret = 0;

    int32_t success = 0;
    EGLBoolean result;
    EGLint numConfig;

    static EGL_DISPMANX_WINDOW_T nativeWindow;

    DISPMANX_UPDATE_HANDLE_T dispmanUpdate;
    VC_RECT_T dstRect;
    VC_RECT_T srcRect;

    static const EGLint attributeList[] =
    {
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_ALPHA_SIZE, 8,
        EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
        EGL_NONE
    };

    static const EGLint contextAttributes[] =
    {
        EGL_CONTEXT_CLIENT_VERSION, 2,
        EGL_NONE
    };

    EGLConfig config;

    ULOGI("Start UI");

    bcm_host_init();

    if (ret == 0)
    {
        // get an EGL display connection
        app->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (app->display == EGL_NO_DISPLAY)
        {
            ULOGE("Failed to get display");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        // initialize the EGL display connection
        result = eglInitialize(app->display, NULL, NULL);
        if (result == EGL_FALSE)
        {
            ULOGE("eglInitialize() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        // get an appropriate EGL frame buffer configuration
        result = eglSaneChooseConfigBRCM(app->display, attributeList, &config, 1, &numConfig);
        if (result == EGL_FALSE)
        {
            ULOGE("eglSaneChooseConfigBRCM() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        // get an appropriate EGL frame buffer configuration
        result = eglBindAPI(EGL_OPENGL_ES_API);
        if (result == EGL_FALSE)
        {
            ULOGE("eglBindAPI() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        // create an EGL rendering context
        app->context = eglCreateContext(app->display, config, EGL_NO_CONTEXT, contextAttributes);
        if (app->context == EGL_NO_CONTEXT)
        {
            ULOGE("eglCreateContext() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        success = graphics_get_display_size(0 /* LCD */, &app->screenWidth, &app->screenHeight);
        if (success < 0)
        {
            ULOGE("graphics_get_display_size() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        // create the background layer
        memset(&app->dispmanBackgroundLayer, 0, sizeof(struct ui_background_layer));
        app->dispmanBackgroundLayer.layer = 0;
        VC_IMAGE_TYPE_T type = VC_IMAGE_RGBA16;
        uint16_t colour = 0x000F;
        uint32_t vc_image_ptr;

        app->dispmanBackgroundLayer.resource = vc_dispmanx_resource_create(type, 1, 1, &vc_image_ptr);
        if (app->dispmanBackgroundLayer.resource == 0)
        {
            ULOGE("vc_dispmanx_resource_create() failed");
            ret = -1;
        }
        else
        {
            VC_RECT_T dst_rect;
            vc_dispmanx_rect_set(&dst_rect, 0, 0, 1, 1);
            result = vc_dispmanx_resource_write_data(app->dispmanBackgroundLayer.resource,
                                                     type, sizeof(colour), &colour, &dst_rect);
            if (result != 0)
            {
                ULOGE("vc_dispmanx_resource_write_data() failed");
                ret = -1;
            }
        }
    }

    if (ret == 0)
    {
        // create the splash layer
        memset(&app->dispmanSplashLayer, 0, sizeof(struct ui_image_layer));

        result = loadPngFile("pdraw_splash.png", &app->dispmanSplashLayer.buffer, &app->dispmanSplashLayer.width,
                             &app->dispmanSplashLayer.height, &app->dispmanSplashLayer.isRgba);
        if (result != 0)
        {
            ULOGE("loadPngFile() failed");
            ret = -1;
        }
        else
        {
            app->dispmanSplashLayer.layer = 1;
            VC_IMAGE_TYPE_T type = (app->dispmanSplashLayer.isRgba) ? VC_IMAGE_RGBA32 : VC_IMAGE_RGB888;
            uint32_t vc_image_ptr;

            app->dispmanSplashLayer.resource = vc_dispmanx_resource_create(type, app->dispmanSplashLayer.width,
                                                                           app->dispmanSplashLayer.height, &vc_image_ptr);
            if (app->dispmanSplashLayer.resource == 0)
            {
                ULOGE("vc_dispmanx_resource_create() failed");
                ret = -1;
            }
            else
            {
                VC_RECT_T dst_rect;
                vc_dispmanx_rect_set(&dst_rect, 0, 0, app->dispmanSplashLayer.width, app->dispmanSplashLayer.height);
                result = vc_dispmanx_resource_write_data(app->dispmanSplashLayer.resource, type,
                                                         app->dispmanSplashLayer.width * ((app->dispmanSplashLayer.isRgba) ? 4 : 3),
                                                         app->dispmanSplashLayer.buffer, &dst_rect);
                if (result != 0)
                {
                    ULOGE("vc_dispmanx_resource_write_data() failed");
                    ret = -1;
                }
            }
        }
    }

    if (ret == 0)
    {
        app->dispmanDisplay = vc_dispmanx_display_open(0 /* LCD */);
        dispmanUpdate = vc_dispmanx_update_start(0);
    }

    if (ret == 0)
    {
        // display background layer
        VC_DISPMANX_ALPHA_T alpha = { DISPMANX_FLAGS_ALPHA_FROM_SOURCE, 255, 0 };

        VC_RECT_T src_rect;
        vc_dispmanx_rect_set(&src_rect, 0, 0, 1, 1);

        VC_RECT_T dst_rect;
        vc_dispmanx_rect_set(&dst_rect, 0, 0, 0, 0);

        app->dispmanBackgroundLayer.element = vc_dispmanx_element_add(dispmanUpdate, app->dispmanDisplay,
                                                                      app->dispmanBackgroundLayer.layer,
                                                                      &dst_rect,
                                                                      app->dispmanBackgroundLayer.resource,
                                                                      &src_rect,
                                                                      DISPMANX_PROTECTION_NONE,
                                                                      &alpha, NULL, DISPMANX_NO_ROTATE);
        if (app->dispmanBackgroundLayer.element == 0)
        {
            ULOGE("vc_dispmanx_element_add() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        // display splash layer
        VC_DISPMANX_ALPHA_T alpha = { DISPMANX_FLAGS_ALPHA_FROM_SOURCE, 255, 0 };

        VC_RECT_T src_rect;
        vc_dispmanx_rect_set(&src_rect, 0, 0, app->dispmanSplashLayer.width << 16, app->dispmanSplashLayer.height << 16);

        VC_RECT_T dst_rect;
        // logo dimensions are the smallest screen dimension / 2
        int32_t dim = (app->screenHeight < app->screenWidth) ? app->screenHeight / 2 : app->screenWidth / 2;
        dim = ((dim + 15) & ~15); // align to 16 pixels
        int32_t xOffset = (app->screenWidth - dim) / 2;
        int32_t yOffset = (app->screenHeight - dim) / 2;
        vc_dispmanx_rect_set(&dst_rect, xOffset, yOffset, dim, dim);

        app->dispmanSplashLayer.element = vc_dispmanx_element_add(dispmanUpdate, app->dispmanDisplay,
                                                                  app->dispmanSplashLayer.layer,
                                                                  &dst_rect,
                                                                  app->dispmanSplashLayer.resource,
                                                                  &src_rect,
                                                                  DISPMANX_PROTECTION_NONE,
                                                                  &alpha, NULL, DISPMANX_NO_ROTATE);
        if (app->dispmanSplashLayer.element == 0)
        {
            ULOGE("vc_dispmanx_element_add() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        // create an EGL window surface
        dstRect.x = 0;
        dstRect.y = 0;
        dstRect.width = app->screenWidth;
        dstRect.height = app->screenHeight;

        srcRect.x = 0;
        srcRect.y = 0;
        srcRect.width = app->screenWidth << 16;
        srcRect.height = app->screenHeight << 16;

        app->dispmanPdrawElement = vc_dispmanx_element_add(dispmanUpdate, app->dispmanDisplay,
                                                           2/*layer*/, &dstRect, 0/*src*/,
                                                           &srcRect, DISPMANX_PROTECTION_NONE,
                                                           0 /*alpha*/, 0/*clamp*/, (DISPMANX_TRANSFORM_T)0/*transform*/);

        nativeWindow.element = app->dispmanPdrawElement;
        nativeWindow.width = app->screenWidth;
        nativeWindow.height = app->screenHeight;

        app->surface = eglCreateWindowSurface(app->display, config, &nativeWindow, NULL);
        if (app->surface == EGL_NO_SURFACE)
        {
            ULOGE("VideoCoreEglRenderer: eglCreateWindowSurface() failed");
            ret = -1;
        }
    }

    if (ret == 0)
    {
        vc_dispmanx_update_submit_sync(dispmanUpdate);
    }

    if (ret == 0)
    {
        // connect the context to the surface
        result = eglMakeCurrent(app->display, app->surface, app->surface, app->context);
        if (result == EGL_FALSE)
        {
            ULOGE("VideoCoreEglRenderer: eglMakeCurrent() failed");
            ret = -1;
        }
    }

    /*if (ret == 0)
    {
        eglSwapInterval(app->display, 1);
    }*/

    ULOGI("UI started; width=%d height=%d", app->screenWidth, app->screenHeight);

    return ret;
}


void resetUi(struct pdraw_app *app)
{
    DISPMANX_UPDATE_HANDLE_T dispmanUpdate;
    int s;

    dispmanUpdate = vc_dispmanx_update_start(0);

    s = vc_dispmanx_element_remove(dispmanUpdate, app->dispmanPdrawElement);
    if (s != 0)
    {
        ULOGE("vc_dispmanx_element_remove() failed");
    }

    vc_dispmanx_update_submit_sync(dispmanUpdate);
}


void stopUi(struct pdraw_app *app)
{
    DISPMANX_UPDATE_HANDLE_T dispmanUpdate;
    int s;

    eglSwapBuffers(app->display, app->surface);

    eglDestroySurface(app->display, app->surface);

    dispmanUpdate = vc_dispmanx_update_start(0);

    s = vc_dispmanx_element_remove(dispmanUpdate, app->dispmanPdrawElement);
    if (s != 0)
    {
        ULOGE("vc_dispmanx_element_remove() failed");
    }

    s = vc_dispmanx_element_remove(dispmanUpdate, app->dispmanSplashLayer.element);
    if (s != 0)
    {
        ULOGE("vc_dispmanx_element_remove() failed");
    }

    s = vc_dispmanx_element_remove(dispmanUpdate, app->dispmanBackgroundLayer.element);
    if (s != 0)
    {
        ULOGE("vc_dispmanx_element_remove() failed");
    }

    vc_dispmanx_update_submit_sync(dispmanUpdate);

    s = vc_dispmanx_resource_delete(app->dispmanBackgroundLayer.resource);
    if (s != 0)
    {
        ULOGE("vc_dispmanx_resource_delete() failed");
    }

    s = vc_dispmanx_resource_delete(app->dispmanSplashLayer.resource);
    if (s != 0)
    {
        ULOGE("vc_dispmanx_resource_delete() failed");
    }
    free(app->dispmanSplashLayer.buffer);

    s = vc_dispmanx_display_close(app->dispmanDisplay);
    if (s != 0)
    {
        ULOGE("vc_dispmanx_display_close() failed");
    }

    eglMakeCurrent(app->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(app->display, app->context);
    eglTerminate(app->display);
}


int startPdraw(struct pdraw_app *app)
{
    int ret = 0;

    ULOGI("Start libpdraw");

    app->pdraw = pdraw_new();
    if (app->pdraw == NULL)
    {
        ULOGE("pdraw_new() failed");
        ret = -1;
    }

    if (ret == 0)
    {
        ret = pdraw_setup(app->pdraw, "PDrAW", "PDrAW", "PDrAW");
        if (ret != 0)
        {
            ULOGE("pdraw_setup() failed (%d)", ret);
        }
    }

    if (ret == 0)
    {
        if (app->receiveStream)
        {
            ret = pdraw_open_single_stream(app->pdraw, app->ipAddr, app->ifaceAddr, app->srcStreamPort, app->srcControlPort,
                                           app->dstStreamPort, app->dstControlPort, app->qosMode);
        }
        else if (app->playRecord)
        {
            ret = pdraw_open_url(app->pdraw, app->url);
        }
        if (ret != 0)
        {
            ULOGE("pdraw_open() failed (%d)", ret);
        }
    }

    if (ret == 0)
    {
        struct uiParams_s
        {
            EGLDisplay display;
            EGLSurface surface;
            EGLContext context;
        };
        struct uiParams_s uiParams = { app->display, app->surface, app->context };

        ret = pdraw_set_renderer_params(app->pdraw,
                                        app->screenWidth, app->screenHeight, 0, 0,
                                        app->screenWidth, app->screenHeight, (void*)&uiParams);
        if (ret != 0)
        {
            ULOGE("pdraw_set_renderer_params() failed (%d)", ret);
        }

        eglMakeCurrent(app->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    }

    if (ret == 0)
    {
        ret = pdraw_start(app->pdraw);
        if (ret != 0)
        {
            ULOGE("pdraw_start() failed (%d)", ret);
        }
    }

    return ret;
}


void stopPdraw(struct pdraw_app *app)
{
    if (app->pdraw)
    {
        int ret;

        ULOGI("Stop libpdraw");

        ret = pdraw_destroy(app->pdraw);
        if (ret != 0)
        {
            ULOGE("pdraw_destroy() failed (%d)", ret);
        }

        app->pdraw = NULL;
    }
}


int startArdiscoveryBrowse(struct pdraw_app *app)
{
    int failed = 0;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;

    ULOGI("Start ARDiscovery browsing");

    if (!failed)
    {
        int ret = pthread_mutex_init(&app->ardiscoveryBrowserMutex, NULL);
        if (ret != 0)
        {
            ULOGE("Mutex creation failed (%d)", ret);
            failed = 1;
        }
    }

    if (!failed)
    {
        char serviceTypes[6][128];
        char *serviceTypesPtr[6];
        uint8_t serviceTypesNb = 6, i;

        snprintf(serviceTypes[0], 128, ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT, ARDISCOVERY_getProductID(ARDISCOVERY_PRODUCT_ARDRONE));
        snprintf(serviceTypes[1], 128, ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT, ARDISCOVERY_getProductID(ARDISCOVERY_PRODUCT_BEBOP_2));
        snprintf(serviceTypes[2], 128, ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT, ARDISCOVERY_getProductID(ARDISCOVERY_PRODUCT_EVINRUDE));
        snprintf(serviceTypes[3], 128, ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT, ARDISCOVERY_getProductID(ARDISCOVERY_PRODUCT_SKYCONTROLLER));
        snprintf(serviceTypes[4], 128, ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT, ARDISCOVERY_getProductID(ARDISCOVERY_PRODUCT_SKYCONTROLLER_NG));
        snprintf(serviceTypes[5], 128, ARDISCOVERY_SERVICE_NET_DEVICE_FORMAT, ARDISCOVERY_getProductID(ARDISCOVERY_PRODUCT_SKYCONTROLLER_2));
        for (i = 0; i < serviceTypesNb; i++)
        {
            serviceTypesPtr[i] = serviceTypes[i];
        }

        app->ardiscoveryBrowserData = ARDISCOVERY_AvahiDiscovery_Browser_New(ardiscoveryBrowserCallback, (void*)app, serviceTypesPtr, serviceTypesNb, &err);
        if (!app->ardiscoveryBrowserData)
        {
            ULOGE("ARDISCOVERY_AvahiDiscovery_Browser_New() failed: %d", err);
            failed = 1;
        }
    }

    if (!failed)
    {
        if (pthread_create(&(app->ardiscoveryBrowserThread), NULL, (void*(*)(void *))ARDISCOVERY_AvahiDiscovery_Browse, app->ardiscoveryBrowserData) != 0)
        {
            ULOGE("Creation of discovery browser thread failed");
            failed = 1;
        }
        else
        {
            app->ardiscoveryBrowserThreadLaunched = 1;
        }
    }

    return failed;
}


void stopArdiscoveryBrowse(struct pdraw_app *app)
{
    ULOGI("Stop ARDiscovery browsing");

    if (app->ardiscoveryBrowserData != NULL)
    {
        ARDISCOVERY_AvahiDiscovery_StopBrowsing(app->ardiscoveryBrowserData);
        if (app->ardiscoveryBrowserThreadLaunched)
        {
            pthread_join(app->ardiscoveryBrowserThread, NULL);
            app->ardiscoveryBrowserThreadLaunched = 0;
        }
    }

    ARDISCOVERY_AvahiDiscovery_Browser_Delete(&app->ardiscoveryBrowserData);

    struct ardiscovery_browser_device *device, *next;
    for (device = app->ardiscoveryDeviceList; device; device = next)
    {
        next = device->next;
        free(device->serviceName);
        free(device->serviceType);
        free(device->ipAddr);
        free(device);
    }
    app->ardiscoveryDeviceList = NULL;

    pthread_mutex_destroy(&app->ardiscoveryBrowserMutex);
}


eARDISCOVERY_ERROR ardiscoveryBrowserCallback(void *userdata, uint8_t state, const char *serviceName, const char *serviceType, const char *ipAddr, uint16_t port)
{
    struct pdraw_app *app = (struct pdraw_app*)userdata;
    struct ardiscovery_browser_device *device = NULL;

    if (state)
    {
        ULOGI("ARSDK device discovered: %s %s %s %d", serviceName, serviceType, ipAddr, port);
        device = malloc(sizeof(*device));
        if (!device)
        {
            ULOGE("Allocation failed");
            return ARDISCOVERY_ERROR;
        }
        else
        {
            memset(device, 0, sizeof(*device));
            device->serviceName = strdup(serviceName);
            device->serviceType = strdup(serviceType);
            device->ipAddr = strdup(ipAddr);
            device->port = port;
            unsigned int productId = 0;
            sscanf(serviceType, "_arsdk-%04x", &productId);
            device->product = ARDISCOVERY_getProductFromProductID(productId);

            pthread_mutex_lock(&app->ardiscoveryBrowserMutex);

            device->next = app->ardiscoveryDeviceList;
            if (app->ardiscoveryDeviceList)
            {
                app->ardiscoveryDeviceList->prev = device;
            }
            app->ardiscoveryDeviceList = device;
            app->ardiscoveryDeviceCount++;

            pthread_mutex_unlock(&app->ardiscoveryBrowserMutex);
        }
    }
    else
    {
        ULOGI("ARSDK device removed: %s %s %s %d", serviceName, serviceType, ipAddr, port);
        int found = 0;

        pthread_mutex_lock(&app->ardiscoveryBrowserMutex);

        for (device = app->ardiscoveryDeviceList; device; device = device->next)
        {
            if ((device->serviceName) && (!strcmp(device->serviceName, serviceName))
                    && (device->serviceType) && (!strcmp(device->serviceType, serviceType))
                    && (device->ipAddr) && (!strcmp(device->ipAddr, ipAddr))
                    && (device->port == port))
            {
                found = 1;
                break;
            }
        }
        if (found)
        {
            free(device->serviceName);
            free(device->serviceType);
            free(device->ipAddr);
            if (device->prev)
            {
                device->prev->next = device->next;
            }
            else
            {
                app->ardiscoveryDeviceList = device->next;
            }
            if (device->next)
            {
                device->next->prev = device->prev;
            }
            free(device);
            app->ardiscoveryDeviceCount--;
        }

        pthread_mutex_unlock(&app->ardiscoveryBrowserMutex);
    }

    pthread_mutex_lock(&app->ardiscoveryBrowserMutex);

    printf("\nDevice discovery:\n");
    int i;
    for (device = app->ardiscoveryDeviceList, i = 1; device; device = device->next, i++)
    {
        printf("  - %d - %s (%s) %s:%d\n", i, device->serviceName, device->serviceType, device->ipAddr, device->port);
    }

    if (app->ardiscoveryDeviceCount == 0)
    {
        printf("  - none\n");
    }

    pthread_mutex_unlock(&app->ardiscoveryBrowserMutex);

    return ARDISCOVERY_OK;
}


int ardiscoveryConnect(struct pdraw_app *app)
{
    int failed = 0;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;
    ARDISCOVERY_Connection_ConnectionData_t *discoveryData;

    ULOGI("ARDiscovery connection");

    discoveryData = ARDISCOVERY_Connection_New(ardiscoveryConnectionSendJsonCallback,
                                               ardiscoveryConnectionReceiveJsonCallback, app, &err);
    if (discoveryData == NULL || err != ARDISCOVERY_OK)
    {
        ULOGE("Error while creating discoveryData: %s", ARDISCOVERY_Error_ToString(err));
        failed = 1;
    }

    if (!failed)
    {
        err = ARDISCOVERY_Connection_ControllerConnection(discoveryData, app->arsdkDiscoveryPort, app->ipAddr);
        if (err != ARDISCOVERY_OK)
        {
            ULOGE("Error while opening discovery connection: %s", ARDISCOVERY_Error_ToString(err));
            failed = 1;
        }
    }

    ARDISCOVERY_Connection_Delete(&discoveryData);

    return failed;
}


eARDISCOVERY_ERROR ardiscoveryConnectionSendJsonCallback(uint8_t *dataTx, uint32_t *dataTxSize, void *customData)
{
    struct pdraw_app *app = (struct pdraw_app*)customData;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;

    if ((dataTx != NULL) && (dataTxSize != NULL) && (app != NULL))
    {
        *dataTxSize = sprintf((char*)dataTx, "{ \"%s\": %d, \"%s\": \"%s\", \"%s\": \"%s\", \"%s\": %d, \"%s\": %d, \"%s\": %d, \"%s\": %d }",
                              ARDISCOVERY_CONNECTION_JSON_D2CPORT_KEY, app->arsdkD2CPort,
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_NAME_KEY, "PDrAW",
                              ARDISCOVERY_CONNECTION_JSON_CONTROLLER_TYPE_KEY, "Unix",
                              ARDISCOVERY_CONNECTION_JSON_QOS_MODE_KEY, 1,
                              ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_CLIENT_STREAM_PORT_KEY, app->dstStreamPort,
                              ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_CLIENT_CONTROL_PORT_KEY, app->dstControlPort,
                              ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SUPPORTED_METADATA_VERSION_KEY, 1) + 1;
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }

    return err;
}


eARDISCOVERY_ERROR ardiscoveryConnectionReceiveJsonCallback(uint8_t *dataRx, uint32_t dataRxSize, char *ip, void *customData)
{
    struct pdraw_app *app = (struct pdraw_app*)customData;
    eARDISCOVERY_ERROR err = ARDISCOVERY_OK;

    if ((dataRx != NULL) && (dataRxSize != 0) && (app != NULL))
    {
        char *json = (char*)malloc(dataRxSize + 1);
        strncpy(json, (char*)dataRx, dataRxSize);
        json[dataRxSize] = '\0';

        ULOGD("Received JSON: %s", json);

        free(json);

        int error = 0;
        json_object* jsonObj_All;
        json_object* jsonObj_Item;
        int value;

        /* Parse the whole Rx buffer */
        if (error == 0)
        {
            jsonObj_All = json_tokener_parse((const char*)dataRx);
            if (jsonObj_All == NULL)
            {
                error = -1;
            }
        }

        /* Find the c2dPort */
        if (error == 0)
        {
            jsonObj_Item = json_object_object_get(jsonObj_All, ARDISCOVERY_CONNECTION_JSON_C2DPORT_KEY);
            if (jsonObj_Item != NULL)
            {
                value = json_object_get_int(jsonObj_Item);
                app->arsdkC2DPort = value;
            }
        }

        /* Find the QoS mode */
        if (error == 0)
        {
            jsonObj_Item = json_object_object_get(jsonObj_All, ARDISCOVERY_CONNECTION_JSON_QOS_MODE_KEY);
            if (jsonObj_Item != NULL)
            {
                value = json_object_get_int(jsonObj_Item);
                app->qosMode = value;
            }
        }

        /* Find the srcStreamPort */
        if (error == 0)
        {
            jsonObj_Item = json_object_object_get(jsonObj_All, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SERVER_STREAM_PORT_KEY);
            if (jsonObj_Item != NULL)
            {
                value = json_object_get_int(jsonObj_Item);
                app->srcStreamPort = value;
            }
        }

        /* Find the srcControlPort */
        if (error == 0)
        {
            jsonObj_Item = json_object_object_get(jsonObj_All, ARDISCOVERY_CONNECTION_JSON_ARSTREAM2_SERVER_CONTROL_PORT_KEY);
            if (jsonObj_Item != NULL)
            {
                value = json_object_get_int(jsonObj_Item);
                app->srcControlPort = value;
            }
        }
    }
    else
    {
        err = ARDISCOVERY_ERROR;
    }

    return err;
}


int startArnetwork(struct pdraw_app *app)
{
    int failed = 0;
    eARNETWORK_ERROR netError = ARNETWORK_OK;
    eARNETWORKAL_ERROR netAlError = ARNETWORKAL_OK;
    int pingDelay = 0; // 0 means default, -1 means no ping

    ULOGI("Start ARNetwork");

    app->arnetworkalManager = ARNETWORKAL_Manager_New(&netAlError);
    if (netAlError != ARNETWORKAL_OK)
    {
        failed = 1;
    }

    if (!failed)
    {
        netAlError = ARNETWORKAL_Manager_InitWifiNetwork(app->arnetworkalManager, app->ipAddr, app->arsdkC2DPort, app->arsdkD2CPort, 1);
        if (netAlError != ARNETWORKAL_OK)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        if (app->qosMode == 1)
        {
            netAlError = ARNETWORKAL_Manager_SetSendClassSelector(app->arnetworkalManager, ARSAL_SOCKET_CLASS_SELECTOR_CS6);
            if (netAlError != ARNETWORKAL_OK)
            {
                failed = 1;
            }
            netAlError = ARNETWORKAL_Manager_SetRecvClassSelector(app->arnetworkalManager, ARSAL_SOCKET_CLASS_SELECTOR_CS6);
            if (netAlError != ARNETWORKAL_OK)
            {
                failed = 1;
            }
        }
    }

    if (!failed)
    {
        app->arnetworkManager = ARNETWORK_Manager_New(app->arnetworkalManager, c2dParamsCount, c2dParams, d2cParamsCount, d2cParams, pingDelay, arnetworkOnDisconnectCallback, app, &netError);
        if (netError != ARNETWORK_OK)
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        if (pthread_create(&(app->arnetworkRxThread), NULL, ARNETWORK_Manager_ReceivingThreadRun, app->arnetworkManager) != 0)
        {
            ULOGE("Creation of rx thread failed");
            failed = 1;
        }
        else
        {
            app->arnetworkRxThreadLaunched = 1;
        }

        if (pthread_create(&(app->arnetworkTxThread), NULL, ARNETWORK_Manager_SendingThreadRun, app->arnetworkManager) != 0)
        {
            ULOGE("Creation of tx thread failed");
            failed = 1;
        }
        else
        {
            app->arnetworkTxThreadLaunched = 1;
        }
    }

    if (failed)
    {
        if (netAlError != ARNETWORKAL_OK)
        {
            ULOGE("ARNetWorkAL Error: %s", ARNETWORKAL_Error_ToString(netAlError));
        }

        if (netError != ARNETWORK_OK)
        {
            ULOGE("ARNetWork Error: %s", ARNETWORK_Error_ToString(netError));
        }
    }

    return failed;
}


void stopArnetwork(struct pdraw_app *app)
{
    ULOGI("Stop ARNetwork");

    if (app->arnetworkManager != NULL)
    {
        ARNETWORK_Manager_Stop(app->arnetworkManager);
        if (app->arnetworkRxThreadLaunched)
        {
            pthread_join(app->arnetworkRxThread, NULL);
            app->arnetworkRxThreadLaunched = 0;
        }

        if (app->arnetworkTxThreadLaunched)
        {
            pthread_join(app->arnetworkTxThread, NULL);
            app->arnetworkTxThreadLaunched = 0;
        }
    }

    if (app->arnetworkalManager != NULL)
    {
        ARNETWORKAL_Manager_Unlock(app->arnetworkalManager);

        ARNETWORKAL_Manager_CloseWifiNetwork(app->arnetworkalManager);
    }

    ARNETWORK_Manager_Delete(&(app->arnetworkManager));
    ARNETWORKAL_Manager_Delete(&(app->arnetworkalManager));
}


void arnetworkOnDisconnectCallback(ARNETWORK_Manager_t *manager, ARNETWORKAL_Manager_t *alManager, void *customData)
{
    ULOGD("ARNetwork disconnection callback");

    struct pdraw_app *app = (struct pdraw_app*)customData;

    if (!app)
    {
        return;
    }

    app->disconnected = 1;
}


eARNETWORK_MANAGER_CALLBACK_RETURN arnetworkCmdCallback(int buffer_id, uint8_t *data, void *custom, eARNETWORK_MANAGER_CALLBACK_STATUS cause)
{
    eARNETWORK_MANAGER_CALLBACK_RETURN retval = ARNETWORK_MANAGER_CALLBACK_RETURN_DEFAULT;

    ULOGD("ARNetwork command callback %d, cause:%d ", buffer_id, cause);

    if (cause == ARNETWORK_MANAGER_CALLBACK_STATUS_TIMEOUT)
    {
        retval = ARNETWORK_MANAGER_CALLBACK_RETURN_DATA_POP;
    }

    return retval;
}


void *arnetworkCmdReaderRun(void* data)
{
    struct pdraw_app *app = NULL;
    int bufferId = 0;
    int failed = 0;
    
    const size_t maxLength = 128 * 1024;
    void *readData = malloc(maxLength);
    if (readData == NULL)
    {
        failed = 1;
    }
    
    if (!failed)
    {
        if (data != NULL)
        {
            bufferId = ((struct arcmd_reader_data*)data)->readerBufferId;
            app = ((struct arcmd_reader_data*)data)->app;
            
            if (app == NULL)
            {
                failed = 1;
            }
        }
        else
        {
            failed = 1;
        }
    }
    
    if (!failed)
    {
        while (app->run)
        {
            eARNETWORK_ERROR netError = ARNETWORK_OK;
            int length = 0;
            int skip = 0;
            
            netError = ARNETWORK_Manager_ReadDataWithTimeout(app->arnetworkManager, bufferId, (uint8_t*)readData, maxLength, &length, 1000);
            if (netError != ARNETWORK_OK)
            {
                if (netError != ARNETWORK_ERROR_BUFFER_EMPTY)
                {
                    ULOGE("ARNETWORK_Manager_ReadDataWithTimeout () failed: %s", ARNETWORK_Error_ToString(netError));
                }
                skip = 1;
            }
            
            if (!skip)
            {
                eARCOMMANDS_DECODER_ERROR cmdError = ARCOMMANDS_DECODER_OK;
                cmdError = ARCOMMANDS_Decoder_DecodeCommand(app->arcmdDecoder, (uint8_t*)readData, length);
                if ((cmdError != ARCOMMANDS_DECODER_OK) && (cmdError != ARCOMMANDS_DECODER_ERROR_NO_CALLBACK))
                {
                    char msg[128];
                    ARCOMMANDS_Decoder_DescribeBuffer((uint8_t*)readData, length, msg, sizeof(msg));
                    ULOGE("ARCOMMANDS_Decoder_DecodeBuffer () failed: %d %s", cmdError, msg);
                }
            }
        }
    }
    
    if (readData != NULL)
    {
        free(readData);
        readData = NULL;
    }
    
    return NULL;
}


int startArcommand(struct pdraw_app *app)
{
    int failed = 0;

    app->arcmdDecoder = ARCOMMANDS_Decoder_NewDecoder(NULL);
    if (app->arcmdDecoder == NULL)
    {
        ULOGE("Failed to create decoder");
        failed = 1;
    }

    app->arcmdReaderThreads = (pthread_t*)calloc(commandBufferIdsCount, sizeof(pthread_t));
    if (app->arcmdReaderThreads == NULL)
    {
        ULOGE("Allocation of reader threads failed");
        failed = 1;
    }

    app->arcmdReaderThreadsLaunched = (int*)calloc(commandBufferIdsCount, sizeof(int));
    if (app->arcmdReaderThreadsLaunched == NULL)
    {
        ULOGE("Allocation of reader threads failed");
        failed = 1;
    }

    if (!failed)
    {
        app->arcmdThreadData = (struct arcmd_reader_data*)calloc(commandBufferIdsCount, sizeof(struct arcmd_reader_data));
        if (app->arcmdThreadData == NULL)
        {
            ULOGE("Allocation of reader threads data failed");
            failed = 1;
        }
    }

    if (!failed)
    {
        size_t readerThreadIndex = 0;
        for (readerThreadIndex = 0 ; readerThreadIndex < commandBufferIdsCount ; readerThreadIndex++)
        {
            app->arcmdThreadData[readerThreadIndex].app = app;
            app->arcmdThreadData[readerThreadIndex].readerBufferId = commandBufferIds[readerThreadIndex];

            if (pthread_create(&(app->arcmdReaderThreads[readerThreadIndex]), NULL, arnetworkCmdReaderRun, &(app->arcmdThreadData[readerThreadIndex])) != 0)
            {
                ULOGE("Creation of reader thread failed");
                failed = 1;
            }
            else
            {
                app->arcmdReaderThreadsLaunched[readerThreadIndex] = 1;
            }
        }
    }

    return failed;
}


void stopArcommand(struct pdraw_app *app)
{
    if ((app->arcmdReaderThreads != NULL) && (app->arcmdReaderThreadsLaunched != NULL))
    {
        size_t readerThreadIndex = 0;
        for (readerThreadIndex = 0 ; readerThreadIndex < commandBufferIdsCount ; readerThreadIndex++)
        {
            if (app->arcmdReaderThreadsLaunched[readerThreadIndex])
            {
                pthread_join(app->arcmdReaderThreads[readerThreadIndex], NULL);
                app->arcmdReaderThreadsLaunched[readerThreadIndex] = 0;
            }
        }
        
        free(app->arcmdReaderThreads);
        app->arcmdReaderThreads = NULL;
        free(app->arcmdReaderThreadsLaunched);
        app->arcmdReaderThreadsLaunched = NULL;
    }
    
    if (app->arcmdThreadData != NULL)
    {
        free(app->arcmdThreadData);
        app->arcmdThreadData = NULL;
    }

    ARCOMMANDS_Decoder_DeleteDecoder(&app->arcmdDecoder);
}


int sendDateAndTime(struct pdraw_app *app)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ULOGD("Send date and time commands");

    char strDate[30];
    char strTime[30];
    time_t rawDate;
    struct tm* tmDateTime;

    time(&rawDate);
    tmDateTime = localtime(&rawDate);
    strftime(strDate, 30, "%F", tmDateTime);
    strftime(strTime, 30, "T%H%M%S%z", tmDateTime);

    /* Send date command */
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonCurrentDate(cmdBuffer, sizeof(cmdBuffer), &cmdSize, strDate);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(app->arnetworkManager, PDRAW_ARSDK_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ULOGW("Failed to send date command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    /* Send time command */
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonCurrentTime(cmdBuffer, sizeof(cmdBuffer), &cmdSize, strTime);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(app->arnetworkManager, PDRAW_ARSDK_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ULOGW("Failed to send time command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}


int sendAllStates(struct pdraw_app *app)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;

    ULOGD("Send all states command");

    /* Send all states command */
    cmdError = ARCOMMANDS_Generator_GenerateCommonCommonAllStates(cmdBuffer, sizeof(cmdBuffer), &cmdSize);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(app->arnetworkManager, PDRAW_ARSDK_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }

    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ULOGW("Failed to send all states command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }

    return sentStatus;
}


int sendStreamingVideoEnable(struct pdraw_app *app)
{
    int sentStatus = 1;
    u_int8_t cmdBuffer[128];
    int32_t cmdSize = 0;
    eARCOMMANDS_GENERATOR_ERROR cmdError;
    eARNETWORK_ERROR netError = ARNETWORK_ERROR;
    
    ULOGD("Send streaming video enable command");
    
    /* Send streaming begin command */
    cmdError = ARCOMMANDS_Generator_GenerateARDrone3MediaStreamingVideoEnable(cmdBuffer, sizeof(cmdBuffer), &cmdSize, 1);
    if (cmdError == ARCOMMANDS_GENERATOR_OK)
    {
        netError = ARNETWORK_Manager_SendData(app->arnetworkManager, PDRAW_ARSDK_CD_ACK_ID, cmdBuffer, cmdSize, NULL, &(arnetworkCmdCallback), 1);
    }
    
    if ((cmdError != ARCOMMANDS_GENERATOR_OK) || (netError != ARNETWORK_OK))
    {
        ULOGW("Failed to send streaming video enable command. cmdError:%d netError:%s", cmdError, ARNETWORK_Error_ToString(netError));
        sentStatus = 0;
    }
    
    return sentStatus;
}


int skyControllerRestreamConnect(struct pdraw_app *app)
{
    int failed = 0;
    char url[100];
    snprintf(url, sizeof(url), "http://%s:7711/video", app->ipAddr);

    ULOGI("SkyController restream connection: %s", url);

    CURL *curl;
    CURLcode res;

    curl_global_init(CURL_GLOBAL_DEFAULT);

    curl = curl_easy_init();
    if (curl)
    {
        curl_easy_setopt(curl, CURLOPT_URL, url);
        FILE *devnull = fopen("/dev/null", "w+");
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, devnull);

        res = curl_easy_perform(curl);
        if (res != CURLE_OK)
        {
            ULOGE("curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
            failed = 1;
        }

        curl_easy_cleanup(curl);
        fclose(devnull);
    }

    curl_global_cleanup();

    app->srcStreamPort = 5004;
    app->srcControlPort = 5005;
    app->dstStreamPort = 55004;
    app->dstControlPort = 55005;

    return failed;
}
