#include "common_function.h"
#include "opencv2/opencv.hpp"

#include "System.h"
#include <chrono>

// 处理ctrl+c中断
#include <csignal>

using namespace cv;
using namespace std;

bool bEnd = false;

void signalHandler(int sig) {
    if (sig == SIGINT) {
        bEnd = true;
        printf("\nORB_SLAM End and Camera Close!\n");
    }
}

string vocFile = "/home/young/SLAM/ORB_SLAM3/Vocabulary/ORBvoc.txt";
string settingFile = "/home/young/SLAM/ORB_SLAM3/METOAK/U60.yaml";

int32_t main(int32_t argc, char **argv) {
    signal(SIGINT, signalHandler);

    if (argc != 3) {
        cerr << endl << "Usage: ./moStereo.cpp vocabularyPath settingPath" << endl;
        return 1;
    }

    // Init ORB-SLAM3
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);
    auto start = chrono::system_clock::now();

    printf("The version of SDK is %s\n", moGetSdkVersion());

    MO_CAMERA_HANDLE hCameraHandle = MO_INVALID_HANDLE;

    int32_t n32Result = 0;
    char caCameraPath[64];
    // Locate camera device name by running arguments
    if (0 != cameraSampleInit(argc, argv, caCameraPath)) {
        return -1;
    }
    // Open camera by path
    n32Result = moOpenUVCCameraByPath(caCameraPath, &hCameraHandle);
    if (0 != n32Result) {
        printf("Open Camera Failed %d\n", n32Result);
        return -1;
    }

    // 设置补光灯
    /*
     * MFT_OFF = 0,                // Fill light off
     * MFT_ON = 1,                 // Fill light on
     * MFT_EXPOSURE_SYNC = 2,      // Fill light synchronizes with exposure
     * MFT_ON_OFF_ALTERNATION = 3  // Fill light alternates between on and off
     */
    moSetFilllightType(hCameraHandle, MFT_OFF);

    uint64_t u64ImageFrameNum = 0;
    uint8_t *pu8FrameBuffer = NULL;
    uint8_t *pu8OnlyY = NULL;
    uint8_t *pu8RectifiedRightYUVI420Img = NULL;
    double d8FPS = 0.0f;
    char caFPS[24] = {0};
#ifdef PRINT_INFO
    // 相机内参
    float fFocus, fCx, fCy;
    float fBxf, fBase;
#endif
    // Get video resolution
    uint16_t u16VideoFrameWidth = 0;
    uint16_t u16VideoFrameHeight = 0;
    n32Result = moGetVideoResolution(hCameraHandle, &u16VideoFrameWidth, &u16VideoFrameHeight);
    if (0 != n32Result) {
        printf("Get video resolution failed %d\n", n32Result);
        return -1;
    }

    // Set video mode to RECTIFIED mode
    n32Result = moSetVideoMode(hCameraHandle, MVM_RECTIFIED);
    if (0 != n32Result) {
        printf("Error: moSetVideoMode failed %d\n", n32Result);
        return -4;
    }
    printf("Press ENTER to quit the running process\n");

    while (0 == getchar_nb(FETCH_AND_DISPLAY_TIME_LENGTH)) {
        // Get current video frame
        n32Result = moGetCurrentFrame(hCameraHandle, &u64ImageFrameNum, &pu8FrameBuffer);
        if (0 != n32Result) {
            printf("Get current frame failed.Try Again...\n");
            continue;
        }

#ifdef PRINT_INFO
        // 打印相机参数
        printf("FPS: %f\n", d8FPS);
        printf("Frame Width: %d, Height: %d\n", u16VideoFrameWidth, u16VideoFrameHeight);

        // 获取相机内参
         int moGetRectifyCamParam(IN const MO_CAMERA_HANDLE hCameraHandle, OUT float* _pfFocus, OUT float* _pfCx, OUT float* _pfCy);
        moGetRectifyCamParam(hCameraHandle, &fFocus, &fCx, &fCy);
        printf("Focus: %f, Cx: %f, Cy: %f\n", fFocus, fCx, fCy);

        // int32_t moGetBxfAndBase(IN const MO_CAMERA_HANDLE hCameraHandle, OUT float* const pfBxf, OUT float* const pfBase);
        moGetBxfAndBase(hCameraHandle, &fBxf, &fBase);
        printf("BF: %f, Base: %f", fBxf, fBase);
#endif

        // Get real-time FPS
        n32Result = moGetRealTimeFPS(hCameraHandle, &d8FPS);
        // Failed to get FPS(usually caused by insufficient frame counts during initial period)
        if (0 != n32Result) {
            sprintf(caFPS, "FPS: Calculating...");
        } else {
            sprintf(caFPS, "FPS: %f", d8FPS);
        }
#ifdef PRINT_INFO
        printf("\nFrameNum: %lu  %s\n", u64ImageFrameNum, caFPS);
#endif
        // Get rectified left and right image
        n32Result = moGetRectifiedImage(hCameraHandle, pu8FrameBuffer, &pu8OnlyY, &pu8RectifiedRightYUVI420Img);

        // Use OpenCV
#ifdef SHOW_IMG
        waitKey(FETCH_AND_DISPLAY_TIME_LENGTH);
#endif
        Mat matLeftBGR;
        Mat matLeftRectify(u16VideoFrameHeight, u16VideoFrameWidth, CV_8UC1, pu8OnlyY);
        cvtColor(matLeftRectify, matLeftBGR, COLOR_GRAY2BGR);
#ifdef SHOW_IMG
        // Mark fps at top left corner
        putText(matLeftBGR, string(caFPS), Point(50, 60), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
        imshow("LeftRectify", matLeftBGR);
#endif
        Mat matRightBGR;  // 1.5 = 12bits / 8bits
        Mat matRightRectify(u16VideoFrameHeight * 1.5, u16VideoFrameWidth, CV_8UC1, pu8RectifiedRightYUVI420Img);
        cvtColor(matRightRectify, matRightBGR, COLOR_YUV2BGR_I420);
#ifdef SHOW_IMG
        // Mark fps at top left corner
        putText(matRightBGR, string(caFPS), Point(50, 60), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
        imshow("RightRectify", matRightBGR);
#endif
        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        SLAM.TrackStereo(matLeftBGR, matRightBGR, double(timestamp.count() / 1000.0));

        if (bEnd) {
            // Stop all threads
            SLAM.Shutdown();
            // Save camera trajectory
            SLAM.SaveTrajectoryKITTI("moCameraTrajectory.txt");
            break;
        }

    }

    // Close camera
    moCloseCamera(&hCameraHandle);
    return n32Result;
}
