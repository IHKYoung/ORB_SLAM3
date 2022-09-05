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
string settingFile = "/home/young/SLAM/ORB_SLAM3/METOAK/U60RGBD.yaml";

int32_t main(int32_t argc, char **argv) {
    signal(SIGINT, signalHandler);

    if (argc != 3) {
        cerr << endl << "Usage: ./moRGBD.cpp vocabularyPath settingPath" << endl;
        return 1;
    }

    // Init ORB-SLAM3
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);
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
    // for RGBD
    uint16_t *pu16RGBDDisparityData = NULL;
    uint8_t *pu8RGBDYUVI420Img = NULL;
    double d8FPS = 0.0f;
    char caFPS[24] = {0};

    // Get video resolution
    uint16_t u16VideoFrameWidth = 0;
    uint16_t u16VideoFrameHeight = 0;
    n32Result = moGetVideoResolution(hCameraHandle, &u16VideoFrameWidth, &u16VideoFrameHeight);
    if (0 != n32Result) {
        printf("Get video resolution failed %d\n", n32Result);
        return -1;
    }

    // Set video mode to RGBD DENSE mode
    n32Result = moSetVideoMode(hCameraHandle, MVM_RGBD_DENSE);
    if (0 != n32Result) {
        printf("Error: moSetVideoMode to RGBD failed %d\n", n32Result);
        return -4;
    }

#ifdef PRINT_INFO
    // 相机内参
    float fBxf = 0.0f;
    float fBase = 0.0f;
    // Get Bxf and Baseline
    n32Result = moGetBxfAndBase(hCameraHandle, &fBxf, &fBase);
    if (n32Result == 0) {
        printf("Bxf: %f, Baseline: %f\n", fBxf, fBase);
    } else {
        printf("Get camera bxf base failed %d\n", n32Result);
    }
#endif
    printf("Press ENTER to quit the running process\n");

    while (0 == getchar_nb(FETCH_AND_DISPLAY_TIME_LENGTH)) {
        // Get current video frame
        n32Result = moGetCurrentFrame(hCameraHandle, &u64ImageFrameNum, &pu8FrameBuffer);
        if (0 != n32Result) {
            printf("Get current frame failed. Try Again...\n");
            continue;
        }

        // Get real-time FPS
        n32Result = moGetRealTimeFPS(hCameraHandle, &d8FPS);
        // Failed to get FPS(usually caused by insufficient frame counts during initial period)
        if (0 != n32Result) {
            sprintf(caFPS, "FPS: Calculating...");
        } else {
            sprintf(caFPS, "FPS: %f", d8FPS);
        }
#ifdef PRINT_INFO
        //printf("\nFrameNum: %lu  %s\n", u64ImageFrameNum, caFPS);
#endif
        // Get RGBD image
        n32Result = moGetRGBDImage(hCameraHandle, pu8FrameBuffer, &pu16RGBDDisparityData, &pu8RGBDYUVI420Img);

        // Use OpenCV
        Mat matYUV2BGR;  // 12bits / 8bits
        Mat matYUVI420(u16VideoFrameHeight * 1.5, u16VideoFrameWidth, CV_8UC1, pu8RGBDYUVI420Img);
        cvtColor(matYUVI420, matYUV2BGR, COLOR_YUV2BGR_I420);
#ifdef SHOW_IMG
        // Mark fps at top left corner
        putText(matYUV2BGR, string(caFPS), Point(50, 60), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
        imshow("RGB", matYUV2BGR);
#endif
        Mat matDisparity(u16VideoFrameHeight, u16VideoFrameWidth, CV_8UC3);
        GetDisparityImage(matDisparity.rows, matDisparity.cols, matDisparity.step, pu16RGBDDisparityData,
                          matDisparity.data);
#ifdef SHOW_IMG
        // Mark fps at top left corner
        putText(matDisparity, string(caFPS), Point(50, 60), FONT_HERSHEY_COMPLEX, 1, Scalar(255, 0, 0));
        imshow("Disparity", matDisparity);
#endif
        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        SLAM.TrackRGBD(matYUV2BGR, matDisparity, double(timestamp.count() / 1000.0));

        if (bEnd) {
            // Stop all threads
            SLAM.Shutdown();
            // Save camera trajectory
            SLAM.SaveTrajectoryTUM("moRGBDCameraTrajectory.txt");
            SLAM.SaveKeyFrameTrajectoryTUM("moRGBDKeyFrameTrajectory.txt");
            break;
        }

    }

    // Close camera
    moCloseCamera(&hCameraHandle);
    return n32Result;
}
