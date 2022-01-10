#include <stdio.h>
#include <iostream>
#include <windows.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#ifdef _DEBUG
    #pragma comment(lib, "opencv_world455d.lib")
#else
    #pragma comment(lib, "opencv_world455.lib")
#endif

#define BASE_OFFSET(ADDRESS, OFFSET) (((ULONG_PTR)(ADDRESS)) + ((ULONG_PTR)OFFSET))

#define FLOAT_COMPARE(X1, X2, EPSILON) (((X1) == (X2)) && (((X1 + EPSILON) > (X2)) && ((X1 - EPSILON) < (X2))))

namespace MotionHighlight {
    #define MOTION_HIGHLIGHT_ELEMENT_SIZE          3
    #define MOTION_HIGHLIGHT_PIXEL_COMPARE_EPSILON 0.20f
    
    static ULONG  VideoFrameWidth;
    static ULONG  VideoFrameHeight;
    static ULONG  VideoFrameElementsCount;
    static PUCHAR ActiveFrameBuffer = NULL;
    static PUCHAR PreviousFrameBuffer = NULL;
    static PUCHAR FirstFrameBuffer;
    static PUCHAR SecondFrameBuffer;

    VOID InitializeFrame(IN cv::Mat * pFrame);
    VOID FrameProcedure(IN cv::Mat * pFrame);
    VOID FirstFrameProcedure(IN cv::Mat * pFrame);
    BOOL Initalize(IN ULONG dwFrameWidth, IN ULONG dwFrameHeight);
}

VOID MotionHighlight::InitializeFrame(IN cv::Mat * pFrame) {
    PULONG32 pPixelBuffer = (PULONG32)(pFrame->data + pFrame->step.p[0]);

    RtlCopyMemory(ActiveFrameBuffer, pPixelBuffer, MotionHighlight::VideoFrameElementsCount * MOTION_HIGHLIGHT_ELEMENT_SIZE);
}

VOID MotionHighlight::FrameProcedure(IN cv::Mat * pFrame) {
    #define COMPARE_PIXEL(PIX1, PIX2, EPSILON) ( \
        FLOAT_COMPARE((PIX1)[0], (PIX2)[0], EPSILON) && \
        FLOAT_COMPARE((PIX1)[1], (PIX2)[1], EPSILON) && \
        FLOAT_COMPARE((PIX1)[2], (PIX2)[2], EPSILON) \
    )

    BOOL   bMotionFound = FALSE;
    PUCHAR sModifiedFrameBuffer;
    ULONG  dwMotionX1 = -1;
    ULONG  dwMotionY1 = -1;
    ULONG  dwMotionX2 = 0;
    ULONG  dwMotionY2 = 0;

    sModifiedFrameBuffer = (PUCHAR)pFrame->data;

    if (MotionHighlight::ActiveFrameBuffer == MotionHighlight::FirstFrameBuffer) {
        MotionHighlight::ActiveFrameBuffer = MotionHighlight::SecondFrameBuffer;
        MotionHighlight::PreviousFrameBuffer = MotionHighlight::FirstFrameBuffer;
    } else {
        MotionHighlight::ActiveFrameBuffer = MotionHighlight::FirstFrameBuffer;
        MotionHighlight::PreviousFrameBuffer = MotionHighlight::SecondFrameBuffer;
    }

    MotionHighlight::InitializeFrame(pFrame);

    for (ULONG_PTR y = 0; y < MotionHighlight::VideoFrameHeight; y++) {
        ULONG_PTR dwHeightIndex = y * pFrame->step.p[0];

        for (ULONG_PTR x = 0; x < MotionHighlight::VideoFrameWidth; x++) {
            ULONG dwPixel;
            FLOAT fActivePixel[3];
            FLOAT fPreviousPixel[3];

            dwPixel = *((PULONG32)&MotionHighlight::ActiveFrameBuffer[dwHeightIndex + x * MOTION_HIGHLIGHT_ELEMENT_SIZE]) & 0x00FFFFFF;
            fActivePixel[0] = (FLOAT)(dwPixel & 0xFF) / 255.0f;
            fActivePixel[1] = (FLOAT)((dwPixel >>  8) & 0xFF) / 255.0f;
            fActivePixel[2] = (FLOAT)((dwPixel >> 16) & 0xFF) / 255.0f;

            dwPixel = *((PULONG32)&MotionHighlight::PreviousFrameBuffer[dwHeightIndex + x * MOTION_HIGHLIGHT_ELEMENT_SIZE]) & 0x00FFFFFF;
            fPreviousPixel[0] = (FLOAT)(dwPixel & 0xFF) / 255.0f;
            fPreviousPixel[1] = (FLOAT)((dwPixel >>  8) & 0xFF) / 255.0f;
            fPreviousPixel[2] = (FLOAT)((dwPixel >> 16) & 0xFF) / 255.0f;

            /* if there is a motion */
            if (!COMPARE_PIXEL(fActivePixel, fPreviousPixel, MOTION_HIGHLIGHT_PIXEL_COMPARE_EPSILON)) {
                ULONG dwMotionWidth = 0;
                ULONG dwMotionHeight = 0;

                /* find the final x of the motion */
                for (ULONG_PTR x1 = x + 1; x1 < MotionHighlight::VideoFrameWidth; x1++) {
                    dwPixel = *((PULONG32)&MotionHighlight::ActiveFrameBuffer[dwHeightIndex + x1 * MOTION_HIGHLIGHT_ELEMENT_SIZE]) & 0x00FFFFFF;
                    fActivePixel[0] = (FLOAT)(dwPixel & 0xFF) / 255.0f;
                    fActivePixel[1] = (FLOAT)((dwPixel >>  8) & 0xFF) / 255.0f;
                    fActivePixel[2] = (FLOAT)((dwPixel >> 16) & 0xFF) / 255.0f;

                    dwPixel = *((PULONG32)&MotionHighlight::PreviousFrameBuffer[dwHeightIndex + x1 * MOTION_HIGHLIGHT_ELEMENT_SIZE]) & 0x00FFFFFF;
                    fPreviousPixel[0] = (FLOAT)(dwPixel & 0xFF) / 255.0f;
                    fPreviousPixel[1] = (FLOAT)((dwPixel >>  8) & 0xFF) / 255.0f;
                    fPreviousPixel[2] = (FLOAT)((dwPixel >> 16) & 0xFF) / 255.0f;

                    if (!COMPARE_PIXEL(fActivePixel, fPreviousPixel, MOTION_HIGHLIGHT_PIXEL_COMPARE_EPSILON)) {
                        if (x1 - x > dwMotionWidth) {
                            dwMotionWidth = x1 - x;
                        }

                        break;
                    }

                    /* find the final y of the motion */
                    for (ULONG_PTR y1 = y + 1; y1 < MotionHighlight::VideoFrameHeight; y1++) {
                        ULONG dwHeightIndex1 = y1 * MotionHighlight::VideoFrameWidth;

                        dwPixel = *((PULONG32)&MotionHighlight::ActiveFrameBuffer[dwHeightIndex1 + x1 * MOTION_HIGHLIGHT_ELEMENT_SIZE]) & 0x00FFFFFF;
                        fActivePixel[0] = (FLOAT)(dwPixel & 0xFF) / 255.0f;
                        fActivePixel[1] = (FLOAT)((dwPixel >>  8) & 0xFF) / 255.0f;
                        fActivePixel[2] = (FLOAT)((dwPixel >> 16) & 0xFF) / 255.0f;

                        dwPixel = *((PULONG32)&MotionHighlight::PreviousFrameBuffer[dwHeightIndex1 + x1 * MOTION_HIGHLIGHT_ELEMENT_SIZE]) & 0x00FFFFFF;
                        fPreviousPixel[0] = (FLOAT)(dwPixel & 0xFF) / 255.0f;
                        fPreviousPixel[1] = (FLOAT)((dwPixel >>  8) & 0xFF) / 255.0f;
                        fPreviousPixel[2] = (FLOAT)((dwPixel >> 16) & 0xFF) / 255.0f;

                        if (!COMPARE_PIXEL(fActivePixel, fPreviousPixel, MOTION_HIGHLIGHT_PIXEL_COMPARE_EPSILON)) {
                            if ((y1 - y) * 2 > dwMotionHeight) {
                                dwMotionHeight = (y1 - y) * 2;
                            }

                            break;
                        }
                    }
                }

                x += dwMotionWidth;

                if (dwMotionWidth > 10 || dwMotionHeight > 10) {
                    bMotionFound = TRUE;

                    if (x < dwMotionX1) {
                        dwMotionX1 = x;
                    }

                    if (y < dwMotionY1) {
                        dwMotionY1 = y;
                    }

                    if (x + dwMotionWidth > dwMotionX2) {
                        dwMotionX2 = x + dwMotionWidth;
                    }

                    if (y + dwMotionHeight > dwMotionY2) {
                        dwMotionY2 = y + dwMotionHeight;
                    }
                }
            }
        }
    }

    for (ULONG_PTR y = 0; y != MotionHighlight::VideoFrameHeight; y++) {
        ULONG_PTR dwHeightIndex = y * pFrame->step.p[0];

        for (ULONG_PTR x = 0; x != MotionHighlight::VideoFrameWidth; x++) {
            ULONG dwSubPixel;
            ULONG dwPixel = *((PULONG32)&sModifiedFrameBuffer[dwHeightIndex + x * MOTION_HIGHLIGHT_ELEMENT_SIZE]);
            FLOAT fBrightness = (FLOAT)(dwPixel & 0xFFFFFF) / (FLOAT)0xFFFFFF;

            dwSubPixel = (ULONG)(fBrightness * 255.0f);

            if (bMotionFound && x >= dwMotionX1 && x <= dwMotionX2 && y >= dwMotionY1 && y <= dwMotionY2) {
                dwPixel = (dwPixel & 0xFF000000) | (dwSubPixel << 16);
            } else {
                dwPixel = (dwPixel & 0xFF000000) | ((dwSubPixel) | (dwSubPixel << 8) | (dwSubPixel << 16));
            }

            *((PULONG32)&sModifiedFrameBuffer[dwHeightIndex + x * MOTION_HIGHLIGHT_ELEMENT_SIZE]) = dwPixel;
        }
    }

    #undef COMPARE_PIXEL
}

VOID MotionHighlight::FirstFrameProcedure(IN cv::Mat * pFrame) {
    MotionHighlight::InitializeFrame(pFrame);
}

BOOL MotionHighlight::Initalize(IN ULONG dwFrameWidth, IN ULONG dwFrameHeight) {
    SIZE_T cbVideFrameBuffer = (dwFrameWidth * dwFrameHeight) * MOTION_HIGHLIGHT_ELEMENT_SIZE;

    MotionHighlight::VideoFrameWidth = dwFrameWidth;
    MotionHighlight::VideoFrameHeight = dwFrameHeight;
    MotionHighlight::VideoFrameElementsCount = dwFrameWidth * dwFrameHeight;

    if (!(MotionHighlight::FirstFrameBuffer = (PUCHAR)malloc(cbVideFrameBuffer))) {
        return FALSE;
    }

    if (!(MotionHighlight::SecondFrameBuffer = (PUCHAR)malloc(cbVideFrameBuffer))) {
        return FALSE;
    }

    MotionHighlight::ActiveFrameBuffer = MotionHighlight::FirstFrameBuffer;
    MotionHighlight::PreviousFrameBuffer = MotionHighlight::SecondFrameBuffer;

    return TRUE;
}

INT wmain() {
    BOOL             bFirstFrame = TRUE;
    ULONG            dwFrameWidth;
    ULONG            dwFrameHeight;
    ULONG            dwFrameCount;
    ULONG            dwFrameNumber = 0;
    std::string      Filename = "sample video.mp4";
    cv::Mat          VideoFrame;
    cv::VideoCapture Capture;

    std::vector<int> CaptureParams = {
        cv::CAP_PROP_AUDIO_STREAM,     cv::CAP_PROP_POS_MSEC,
        cv::CAP_PROP_VIDEO_STREAM,     cv::CAP_PROP_POS_MSEC,
        cv::CAP_PROP_AUDIO_DATA_DEPTH, CV_16S
    };

    Capture.open(Filename, cv::CAP_MSMF, CaptureParams);

    if (!Capture.isOpened()) {
        printf("failed to open '%s'\n", Filename.c_str());

        return 1;
    }

    dwFrameWidth = (ULONG)Capture.get(cv::CAP_PROP_FRAME_WIDTH);
    dwFrameHeight = (ULONG)Capture.get(cv::CAP_PROP_FRAME_HEIGHT);
    dwFrameCount = (ULONG)Capture.get(cv::CAP_PROP_FRAME_COUNT);

    if (!MotionHighlight::Initalize(dwFrameWidth, dwFrameHeight)) {
        printf("failed to initialize the motion-highlighter");

        return 3;
    }

    /* BUG: grab creates a crash in debug */
    while (Capture.grab()) {
        Capture.retrieve(VideoFrame);

        if (VideoFrame.empty()) {
            break;
        }

        if (bFirstFrame) {
            bFirstFrame = FALSE;

            MotionHighlight::FirstFrameProcedure(&VideoFrame);
        } else {
            MotionHighlight::FrameProcedure(&VideoFrame);
        }

        imshow("Motion Highlight", VideoFrame);

        if (cv::waitKey('\n') >= 0) {
            break;
        }
    }

    return 0;
}
