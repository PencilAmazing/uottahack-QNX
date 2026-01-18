/*
 * Copyright (c) 2024, BlackBerry Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>
#include <termios.h>
#include <pthread.h>

#include <camera/camera_api.h>
#include <sys/neutrino.h>
#include "gpio.h"

#include <opencv2/opencv.hpp>

#define DEBUG

static const int MIDDLE = 900;

/**
 * @brief Number of channels for supported frametypes
 */
#define NUM_CHANNELS            (3)

/**
 * @brief Pulse event codes
 */
#define PULSE_CODE_DATA         (_PULSE_CODE_MINAVAIL + 1)
#define PULSE_CODE_STATUS       (_PULSE_CODE_MINAVAIL + 2)
#define PULSE_CODE_TERMINATE    (_PULSE_CODE_MINAVAIL + 3)

#define NS_PER_MS               (1000000UL)
#define US_PER_MS               (1000UL)

/**
 * @brief Maximum time to wait for a pulse: must be greater than 1 frame period
 */
#define PULSE_TIMEOUT_NS        (500 * NS_PER_MS)

/**
 * @brief Time to wait after starting the events before checking thread error
 */
#define START_WAIT_TIME_US      (100 * US_PER_MS)

/**
 * @brief List of frametypes that @c processCameraData can operate on
 */
const camera_frametype_t cSupportedFrametypes[] = {
    CAMERA_FRAMETYPE_YCBYCR,
    CAMERA_FRAMETYPE_CBYCRY,
    CAMERA_FRAMETYPE_RGB8888,
    CAMERA_FRAMETYPE_BGR8888,
};
#define NUM_SUPPORTED_FRAMETYPES (sizeof(cSupportedFrametypes) / sizeof(cSupportedFrametypes[0]))

/**
 * @brief State for the thread that receives events
 */
typedef struct {
    camera_handle_t handle;
    pthread_t tid;
    bool started;

    // This mutex protects all below variables up to the next empty line
    pthread_mutex_t mutex;
    int chid;
    int coid;
    int error;
} event_thread_state_t;

/**
 * @brief If the camera does not default to a supported frametype,
 *        then change the frametype to an available supported frametype.
 */
static int configureFrametype(camera_handle_t handle);

/**
 * @brief Start the event processing thread
 */
static int startEventProcessingThread(event_thread_state_t* state);

/**
 * @brief Stop the event processing thread
 */
static int stopEventProcessingThread(event_thread_state_t* state);

/**
 * @brief Entrypoint for the thread which receives events and processes the buffers
 */
static void* eventProcessingThread(void* arg);

/**
 * @brief Prints a list of available cameras
 */
static void listAvailableCameras(void);

/**
 * @brief Function for processing the camera data
 */
static void processCameraData(camera_buffer_t* buffer);

/**
 * @brief Blocks until the user presses any key
 */
static void blockOnKeyPress(void);

int main(int argc, char* argv[])
{
    int err;
    int opt;
    camera_unit_t unit = CAMERA_UNIT_NONE;
    camera_handle_t handle = CAMERA_HANDLE_INVALID;
    event_thread_state_t eventThreadState;

    // Read command line options
    while ((opt = getopt(argc, argv, "u:")) != -1 || (optind < argc)) {
        switch (opt) {
        case 'u':
            unit = (camera_unit_t)strtol(optarg, NULL, 10);
            break;
        default:
            printf("Ignoring unrecognized option: %s\n", optarg);
            break;
        }
    }

    // If no camera unit has been specified, list the options and exit
    if ((unit == CAMERA_UNIT_NONE) || (unit >= CAMERA_UNIT_NUM_UNITS)) {
        listAvailableCameras();
        printf("Please provide camera unit with -u option\n");
        exit(EXIT_SUCCESS);
    }

    // Open a data read-only, config read-write handle for the specified camera unit.
    // With this access we can change the frametype to a supported one if available.
    uint32_t mode = (CAMERA_MODE_DREAD | CAMERA_MODE_PREAD | CAMERA_MODE_PWRITE);
    err = camera_open(unit, mode, &handle);
    if ((err != CAMERA_EOK) || (handle == CAMERA_HANDLE_INVALID)) {
        printf("Failed to open CAMERA_UNIT_%d: err = %d\n", (int)unit, err);
        exit(EXIT_FAILURE);
    }

    // Ensure the camera is configured to use a frametype that is supported by
    // processCameraData. If that's not possible, this function returns an error.
    err = configureFrametype(handle);
    if (err != EOK) {
        (void)camera_close(handle);
        exit(EXIT_FAILURE);
    }
    printf("\n");

    // GPIO setup
    init_gpio();

    // Start the event thread
    memset(&eventThreadState, 0x0, sizeof(eventThreadState));
    eventThreadState.handle = handle;
    err = startEventProcessingThread(&eventThreadState);
    if (err != EOK) {
        printf("Failed to start event thread: err = %d\n", err);
        (void)camera_close(handle);
        exit(EXIT_FAILURE);
    }

    // Start the camera streaming: events will start being received
    err = camera_start_viewfinder(handle, NULL, NULL, NULL);
    if (err != CAMERA_EOK) {
        printf("Failed to start CAMERA_UNIT_%d: err = %d\n", (int)unit, err);
        (void)stopEventProcessingThread(&eventThreadState);
        (void)camera_close(handle);
        exit(EXIT_FAILURE);
    }

    // Wait for any initialization errors in the event processing thread
    usleep(START_WAIT_TIME_US);
    (void)pthread_mutex_lock(&eventThreadState.mutex);
    bool block = (eventThreadState.error == EOK);
    (void)pthread_mutex_unlock(&eventThreadState.mutex);
    if (block) {
        blockOnKeyPress();
    }

    // Stop the camera streaming: no more events will be received
    err = camera_stop_viewfinder(handle);
    printf("\r\n");
    if (err != CAMERA_EOK) {
        printf("Failed to stop CAMERA_UNIT_%d: err = %d\n", (int)unit, err);
        (void)stopEventProcessingThread(&eventThreadState);
        (void)camera_close(handle);
        exit(EXIT_FAILURE);
    }

    // Stop the event processing thread
    err = stopEventProcessingThread(&eventThreadState);
    if (err != EOK) {
        printf("Failed to stop event processing thread: err = %d\n", err);
        (void)camera_close(handle);
        exit(EXIT_FAILURE);
    }

    // Close the camera handle
    err = camera_close(handle);
    if (err != CAMERA_EOK) {
        printf("Failed to close CAMERA_UNIT_%d: err = %d\n", (int)unit, err);
        exit(EXIT_FAILURE);
    }

    // Set GPIO pins low
    deinit_gpio();
    
    exit(EXIT_SUCCESS);
}

static int configureFrametype(camera_handle_t handle)
{
    int err;
    camera_frametype_t frametype = CAMERA_FRAMETYPE_UNSPECIFIED;

    // Check if the camera is already set to a supported frametype
    err = camera_get_vf_property(handle, CAMERA_IMGPROP_FORMAT, &frametype);
    if ((err != CAMERA_EOK) || (frametype == CAMERA_FRAMETYPE_UNSPECIFIED)) {
        printf("Failed to get frametype: err = %d\n", err);
        return err;
    }
    bool unsupportedFrametype = true;
    for (uint i = 0; i < NUM_SUPPORTED_FRAMETYPES; i++) {
        if (frametype == cSupportedFrametypes[i]) {
            unsupportedFrametype = false;
            break;
        }
    }

    if (unsupportedFrametype) {
        // Get the list of available frametypes for this camera unit
        uint32_t numsupported;
        err = camera_get_supported_vf_frame_types(
            handle, 0, &numsupported, NULL
        );
        if (err != CAMERA_EOK) {
            printf("Failed to number of supported frametypes: err = %d\n", err);
            return err;
        }
        camera_frametype_t* availableFrametypes = (camera_frametype_t*)calloc(
            numsupported, sizeof(camera_frametype_t)
        );
        if (availableFrametypes == NULL) {
            printf("No memory for available frametype list\n");
            return ENOMEM;
        }
        err = camera_get_supported_vf_frame_types(
            handle, numsupported, &numsupported, availableFrametypes
        );
        if (err != CAMERA_EOK) {
            printf("Failed to get supported frametypes: err = %d\n", err);
            free(availableFrametypes);
            return err;
        }

        // Find intersection between cSupportedFrametypes and availableFrametypes
        frametype = CAMERA_FRAMETYPE_UNSPECIFIED;
        for (uint i = 0; i < NUM_SUPPORTED_FRAMETYPES; i++) {
            for (uint j = 0; i < numsupported; j++) {
                if (cSupportedFrametypes[i] == availableFrametypes[j]) {
                    frametype = availableFrametypes[j];
                    break;
                }
            }
            if (frametype != CAMERA_FRAMETYPE_UNSPECIFIED) {
                break;
            }
        }
        free(availableFrametypes);
        if (frametype == CAMERA_FRAMETYPE_UNSPECIFIED) {
            printf("No available frametype found for this example\n");
            return ENOTSUP;
        }

        // Set the discovered frametype
        err = camera_set_vf_property(
            handle, CAMERA_IMGPROP_FORMAT, frametype
        );
        if (err != CAMERA_EOK) {
            printf("Failed to set frametype of %d: err = %d\n", (int)frametype, err);
            return err;
        }
    }

    return EOK;
}

static int startEventProcessingThread(event_thread_state_t* state)
{
    int err;

    // Initialize the mutex
    err = pthread_mutex_init(&state->mutex, NULL);
    if (err != EOK) {
        printf("Failed to initialize mutex: err = %d\n", err);
        return err;
    }

    // Launch the thread
    err = pthread_create(&state->tid, NULL, eventProcessingThread, state);
    if (err != EOK) {
        printf("Failed to start event thread: err = %d\n", err);
        (void)pthread_mutex_destroy(&state->mutex);
        return err;
    }

    state->started = true;

    return EOK;
}

static int stopEventProcessingThread(event_thread_state_t* state)
{
    int err;
    int rc = EOK;

    if (!state->started) {
        return EALREADY;
    }

    (void)pthread_mutex_lock(&state->mutex);
    if (state->coid > 0) {
        err = MsgSendPulse(state->coid, -1, PULSE_CODE_TERMINATE, 0);
        if (err != EOK) {
            rc = errno;
        }
    }
    (void)pthread_mutex_unlock(&state->mutex);
    if (rc != EOK) {
        rc = err;
        printf("Failed to deliver pulse: err = %d\n", rc);
    }

    err = pthread_join(state->tid, NULL);
    if (err != EOK) {
        rc = err;
        printf("Failed to join event processing thread: err = %d\n", rc);
    }
    (void)pthread_mutex_destroy(&state->mutex);
    state->started = false;

    return rc;
}

static void* eventProcessingThread(void* arg)
{
    int err;
    struct sigevent dataEvent;
    camera_eventkey_t dataKey = -1;
    struct sigevent statusEvent;
    camera_eventkey_t statusKey = -1;
    struct _pulse pulse;
    event_thread_state_t* state = (event_thread_state_t*)arg;

    // _NTO_THREAD_NAME_MAX will be greater than 22
    (void)pthread_setname_np(0, "eventProcessingThread");
    printf("Event processing thread starting\n\n");

    (void)pthread_mutex_lock(&state->mutex);

    state->chid = -1;
    state->coid = -1;

    do {

        // Create the necessary QNX IPC architecture to support receiving events
        state->chid = ChannelCreate(_NTO_CHF_PRIVATE);
        if (state->chid < 0) {
            state->error = errno;
            printf("Failed to create a channel: err = %d\n", state->error);
            break;
        }
        state->coid = ConnectAttach(0, 0, state->chid, _NTO_SIDE_CHANNEL, 0);
        if (state->coid < 0) {
            state->error = errno;
            printf("Failed to attach a channel: err = %d\n", state->error);
            break;
        }

        // Configure the events to be provided
        SIGEV_PULSE_INIT(&dataEvent, state->coid, -1, PULSE_CODE_DATA, NULL);
        SIGEV_PULSE_INIT(&statusEvent, state->coid, -1, PULSE_CODE_STATUS, NULL);

        // Enable the delivery of the data and status events
        err = camera_enable_viewfinder_event(
            state->handle, CAMERA_EVENTMODE_READONLY, &dataKey, &dataEvent
        );
        if (err != CAMERA_EOK) {
            state->error = err;
            printf("Failed to enable data event: err = %d\n", state->error);
            break;
        }
        err = camera_enable_status_event(state->handle, &statusKey, &statusEvent);
        if (err != CAMERA_EOK) {
            state->error = err;
            printf("Failed to enable status event: err = %d\n", state->error);
            break;
        }

        (void)pthread_mutex_unlock(&state->mutex);

        for(;;) {

            // Wait for a pulse
            uint64_t timeoutNs = PULSE_TIMEOUT_NS;
            (void)TimerTimeout(
                CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE, NULL, &timeoutNs, NULL
            );
            err = MsgReceivePulse(state->chid, &pulse, sizeof(pulse), NULL);
            if (err != EOK) {
                state->error = err;
                printf("Failed to receive pulse: err = %d\n", err);
                break;
            }

            if (pulse.code == PULSE_CODE_DATA) {
                // Get the available buffer
                camera_buffer_t inBuf;
                err = camera_get_viewfinder_buffers(
                    state->handle, dataKey, &inBuf, NULL
                );
                if (err != CAMERA_EOK) {
                    state->error = err;
                    printf("Failed to get buffer: err = %d\n", state->error);
                    break;
                }

                processCameraData(&inBuf);

                // We must explicitly return the buffer when we are done with it
                err = camera_return_buffer(state->handle, &inBuf);
                if (err != CAMERA_EOK) {
                    state->error = err;
                    printf("Failed to return buffer: err = %d\n", state->error);
                    break;
                }

            } else if (pulse.code == PULSE_CODE_STATUS) {
                // Get the status details
                camera_devstatus_t status;
                uint16_t extra;
                err = camera_get_status_details(
                    state->handle, pulse.value, &status, &extra
                );
                if (err != CAMERA_EOK) {
                    printf("Failed to get status details: err = %d\n", state->error);
                    break;
                }

                switch (status) {
                // Status codes when starting streaming
                case CAMERA_STATUS_VIDEOVF:
                    printf("\rCAMERA_STATUS_VIDEOVF\n");
                    break;
                case CAMERA_STATUS_VIEWFINDER_ACTIVE:
                    printf("\rCAMERA_STATUS_VIEWFINDER_ACTIVE\n");
                    break;
                // Status code when stopping streaming
                case CAMERA_STATUS_CONNECTED:
                    printf("\r\nCAMERA_STATUS_CONNECTED\n");
                    break;
                // Status codes in error or warning cases
                case CAMERA_STATUS_VIEWFINDER_ERROR:
                    printf("\r\nCAMERA_STATUS_VIEWFINDER_ERROR\n");
                    break;
                case CAMERA_STATUS_FRAME_DROPPED:
                {
                  break; /* NOTE: ignnore dropped frames */
                    printf("\r\nCAMERA_STATUS_FRAME_DROPPED:\n");
                    uint64_t droppedFrameCount;
                    uint64_t totalFrameCount;
                    err = camera_get_frame_stats_property(
                        state->handle, (camera_eventkey_t)extra,
                        CAMERA_FRAME_STATS_DROPPED_COUNT, &droppedFrameCount,
                        CAMERA_FRAME_STATS_TOTAL_COUNT, &totalFrameCount
                    );
                    if (err == EOK) {
                        printf("\r%" PRIu64 " / %" PRIu64 "\n", droppedFrameCount, totalFrameCount);
                    }
                    break;
                }
                case CAMERA_STATUS_USER_BUFFER_OVERFLOW:
                {
                    printf("\r\nCAMERA_STATUS_USER_BUFFER_OVERFLOW:\n");
                    uint64_t skippedFrameCount;
                    uint64_t totalFrameCount;
                    err = camera_get_frame_stats_property(
                        state->handle, (camera_eventkey_t)extra,
                        CAMERA_FRAME_STATS_SKIPPED_COUNT, &skippedFrameCount,
                        CAMERA_FRAME_STATS_TOTAL_COUNT, &totalFrameCount
                    );
                    if (err == EOK) {
                        printf("\r%" PRIu64 " / %" PRIu64 "\n", skippedFrameCount, totalFrameCount);
                    }
                    break;
                }
                default:
                    printf("\rOther status code = %d\n", (int)status);
                    break;
                }

            } else if (pulse.code == PULSE_CODE_TERMINATE) {
                break;
            }

        }

        (void)pthread_mutex_lock(&state->mutex);

    } while (false);

    // Cleanup with mutex locked
    if (dataKey != -1) {
        (void)camera_disable_event(state->handle, dataKey);
    }
    if (statusKey != -1) {
        (void)camera_disable_event(state->handle, statusKey);
    }
    if (state->coid != -1) {
        (void)ConnectDetach(state->coid);
    }
    if (state->chid != -1) {
        (void)ChannelDestroy(state->chid);
    }
    (void)pthread_mutex_unlock(&state->mutex);

    printf("Event processing thread exited\n");
    return NULL;
}

static void listAvailableCameras(void)
{
    int err;
    uint numSupported;
    camera_unit_t* supportedCameras;

    // Determine how many cameras are supported
    err = camera_get_supported_cameras(0, &numSupported, NULL);
    if (err != CAMERA_EOK) {
        printf("Failed to get number of supported cameras: err = %d\n", err);
        return;
    }

    if (numSupported == 0) {
        printf("No supported cameras detected!\n");
        return;
    }

    // Allocate an array big enough to hold all camera units
    supportedCameras = (camera_unit_t*)calloc(numSupported, sizeof(camera_unit_t));
    if (supportedCameras == NULL) {
        printf("Failed to allocate memory for supported cameras\n");
        return;
    }

    // Get the list of supported cameras
    err = camera_get_supported_cameras(numSupported, &numSupported, supportedCameras);
    if (err != CAMERA_EOK) {
        printf("Failed to get list of supported cameras: err = %d\n", err);
    } else {
        printf("Available camera units:\n");
        for (uint i = 0; i < numSupported; i++) {
            printf("\tCAMERA_UNIT_%d", supportedCameras[i]);
            printf(" (specify -u %d)\n", supportedCameras[i]);
        }
    }

    free(supportedCameras);
    return;
}

static const int HISTORY_SIZE = 16;
static int historyPointer = 0;
cv::Point point_history[HISTORY_SIZE] = {};

static const int FRAMES_TO_SKIP = 40;
static int frameCounterTillSkip = 0;

struct Puck {
    int x;
    int y;
    float radius;
    bool found;
    
    int velocity_x;
    int velocity_y;
};
Puck detectGreenPuck(const unsigned char* imageData, int width, int height, int stride) {
     Puck puck = {0, 0, 0, false};
    
     // Create cv::Mat from raw image data
     /* printf("stride %d", stride); */
     /* fflush(stdout); */
     // Convert YUYV to BGR
    cv::Mat yuyv(height, width, CV_8UC2, (void*)imageData, stride);
    cv::Mat img;
    cv::cvtColor(yuyv, img, cv::COLOR_YUV2BGR_YUYV);

   
    // Preprocess: Enhance brightness and contrast for dark images

    // cv::Mat enhanced;
    
    // // Gamma correction - increase gamma for brighter image
    // double gamma = 3; // Increased for much brighter image
    // cv::Mat lookUpTable(1, 256, CV_8U);
    // uchar* p = lookUpTable.ptr();
    // for(int i = 0; i < 256; ++i)
    //     p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, 1.0 / gamma) * 255.0);
    // cv::LUT(img, lookUpTable, enhanced);

    // // Use enhanced image for detection
    // img = enhanced;

    // Convert to HSV color space
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    
    cv::Scalar lowerRed1(0, 20, 30);
    cv::Scalar upperRed1(10, 255, 255);
    cv::Scalar lowerRed2(170, 20, 30);
    cv::Scalar upperRed2(180, 255, 255);
    
    // Define range for green color in HSV
    // Adjusted for darker images - lower saturation and value thresholds
    cv::Scalar lowerGreen(35, 25, 25);  // Lowered S and V for dark conditions
    // Upper bound
    cv::Scalar upperGreen(85, 255, 255);
    
    // Threshold the image to get only green colors
    cv::Mat mask;
    cv::inRange(hsv, lowerGreen, upperGreen, mask);
    
    #ifdef DEBUG
    cv::imwrite("mask.tiff", mask);
    #endif
    
    // Apply morphological operations to remove noise
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // Filter contours to find the largest circular puck
    double maxArea = 0;
    
    int bigenough = 0;
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        
        // Filter by area (adjust threshold as needed)
        if (area < 100) {
            // printf("Skipping contour %d", count);
            continue;
        }

        bigenough += 1;
        
        // Fit circle to contour
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);
        
        // Check circularity
        double perimeter = cv::arcLength(contour, true);
        double circularity = 4 * CV_PI * area / (perimeter * perimeter);

        // Filter by circularity and keep the largest one
        if (circularity > 0.5 && area > maxArea) {
            // Found the puck, push to history
            frameCounterTillSkip += 1;
            if (frameCounterTillSkip > FRAMES_TO_SKIP) {
                frameCounterTillSkip = 0;

                point_history[historyPointer] = cv::Point((int)center.x, (int)center.y);
                historyPointer = (historyPointer + 1) % HISTORY_SIZE;
            }
            
            maxArea = area;
            puck.x = (int)center.x;
            puck.y = (int)center.y;
            puck.radius = radius;
            puck.found = true;
        };

        cv::Point p(center.x, center.y);

        #ifdef DEBUG
        /* DEBUG: Draw some annotations */
        cv::circle(img, p, radius, {0, 255, 0}, 4);

        cv::putText(
            img,
            cv::format("radius: %.2f, circle: %.02f, area: %.02f ", radius, circularity, area),
            p + cv::Point(25, -10),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            {0, 255, 0},
            1,
            cv::LINE_AA
        );
        #endif
    }

    
    
    // DEBUG: Save enhanced image
    #ifdef DEBUG
    cv::circle(img, cv::Point(puck.x, puck.y), puck.radius, {0, 0, 225}, 4);

    cv::circle(img, cv::Point(10, 10), 4, {0,0,0}, 4);
    cv::circle(img, cv::Point(img.size().width-10, img.size().height-10), 4, {0,0,0}, 4);
    
    cv::imwrite("debug_enhanced.tiff", img);
    #endif
    
    return puck;
}

static void sendPaddle(Puck puck, int paddlePosition) {
    int delta = puck.x - MIDDLE;

    if (delta > 100) {
        paddle_left();
    } else if(delta < -100) {
        paddle_right();
    }
    
}

static void processCameraData(camera_buffer_t* buffer)
{
    clock_t begin;
    clock_t end;
    double channelAverage[NUM_CHANNELS];
    Puck puck;

    // Camera data is buffer->framebuf and described by buffer->framedesc.
    // As an example, let's compute channel averages by iterating over the
    // bytes in every other line and determining which channel the byte belongs to.
    begin = clock();
    memset(channelAverage, 0x0, sizeof(channelAverage));

  /* YCBYCR is camera type, 14 */
  
    switch (buffer->frametype) {
    case CAMERA_FRAMETYPE_RGB8888:
    {
      // Channel averages ordering: R, G, B
      uint32_t width = buffer->framedesc.rgb8888.width;
      uint32_t height = buffer->framedesc.rgb8888.height;
      uint32_t stride = buffer->framedesc.rgb8888.stride;

      puck = detectGreenPuck(buffer->framebuf, width, height, stride);
      
      for (uint y = 0; y < height; y += 2) {
        uint8_t *linePointer = buffer->framebuf + y * stride;
        for (uint i = 0; i < 4 * width; i++) {
          uint chan = i % 4;
          if (chan == 3) {
            continue;
          }
          channelAverage[chan] += (double)*(linePointer + i);
        }
        }
        for (uint chan = 0; chan < NUM_CHANNELS; chan++) {
            channelAverage[chan] /= (double)(width * height);
        }
        break;
    }
    case CAMERA_FRAMETYPE_BGR8888:
    {
        // Channel averages ordering: R, G, B
        uint32_t width = buffer->framedesc.bgr8888.width;
        uint32_t height = buffer->framedesc.bgr8888.height;
        uint32_t stride = buffer->framedesc.bgr8888.stride;
      
        puck = detectGreenPuck(buffer->framebuf, width, height, stride);
      
        for (uint y = 0; y < height; y += 2) {
            uint8_t* linePointer = buffer->framebuf + y * stride;
            for (uint i = 0; i < 4 * width; i++) {
                if (i % 4 == 3) {
                    continue;
                }
                uint chan = (4 * width - i + 3) % 4 - 1;
                channelAverage[chan] += (double)*(linePointer + i);
            }
        }
        for (uint chan = 0; chan < NUM_CHANNELS; chan++) {
            channelAverage[chan] /= (double)(width * height);
        }
        break;
    }
    case CAMERA_FRAMETYPE_YCBYCR:
    {
      /* NOTE: THIS IS WHERE THE MAGIC HAPPENS */
      // Channel averages ordering: Y, Cb, Cr
      uint32_t width = buffer->framedesc.ycbycr.width;
      uint32_t height = buffer->framedesc.ycbycr.height;
      uint32_t stride = buffer->framedesc.ycbycr.stride;
      
      puck = detectGreenPuck(buffer->framebuf, width, height, stride);

      for (uint y = 0; y < height; y += 2) {
        uint8_t *linePointer = buffer->framebuf + y * stride;
        for (uint i = 0; i < 2 * width; i++) {
          uint chan = i % 2;
          if (chan == 1) {
            chan = (i - 1) % 4 / 2 + 1;
          }
          channelAverage[chan] += (double)*(linePointer + i);
        }
        }
        channelAverage[0] /= (double)(width * height);
        channelAverage[1] /= (double)(width / 2 * height);
        channelAverage[2] /= (double)(width / 2 * height);
        break;
    }
    case CAMERA_FRAMETYPE_CBYCRY:
    {
        // Channel averages ordering: Y, Cb, Cr
        uint32_t width = buffer->framedesc.cbycry.width;
        uint32_t height = buffer->framedesc.cbycry.height;
        uint32_t stride = buffer->framedesc.cbycry.stride;

      puck = detectGreenPuck(buffer->framebuf, width, height, stride);

        for (uint y = 0; y < height; y += 2) {
            uint8_t* linePointer = buffer->framebuf + y * stride;
            for (uint i = 0; i < 2 * width; i++) {
                uint chan = (i + 1) % 2;
                if (chan == 1) {
                    chan = ((i + 1) % 4 + 1) / 2;
                }
                channelAverage[chan] += (double)*(linePointer + i);
            }
        }
        channelAverage[0] /= (double)(width * height);
        channelAverage[1] /= (double)(width / 2 * height);
        channelAverage[2] /= (double)(width / 2 * height);
        break;
    }
    default:
        printf("\r");
        printf("Frametype %d is not suppported!", (int)buffer->frametype);
        printf(" (press any key to stop example)");
        fflush(stdout);
        return;
    }
    end = clock();

    double delta_time = (double)(end - begin) / CLOCKS_PER_SEC * 1000;

    printf("\r");
    printf("Channel averages: ");
    printf("%.3f, %.3f, %.3f", channelAverage[0], channelAverage[1], channelAverage[2]);
    printf(" took %.3f ms", delta_time);
    printf(" frametype %d", buffer->frametype);
    if (puck.found) {
      printf(" Puck: (%d, %d) - radius %f", puck.x, puck.y, puck.radius);
    } else {
      printf(" Puck not found");
    }
    printf(" (press any key to stop example)     ");
    fflush(stdout);

    /* We know where th puck is on the x-axis, control paddle */
    sendPaddle(puck, 800);

    return;
}

static void blockOnKeyPress(void)
{
    struct termios oldterm;
    struct termios newterm;
    char key;

    (void)tcgetattr(STDIN_FILENO, &oldterm);
    newterm = oldterm;
    newterm.c_lflag &= ~(ECHO | ICANON);
    (void)tcsetattr(STDIN_FILENO, TCSANOW, &newterm);
    // Blocking call: wait for 1 byte of data to become available
    (void)read(STDIN_FILENO, &key, 1);
    (void)tcsetattr(STDIN_FILENO, TCSANOW, &oldterm);

    return;
}
