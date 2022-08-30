#include "common_function.h"
#include "opencv2/opencv.hpp"

#include "System.h"
#include <chrono>

// 处理ctrl+c中断
#include <csignal>
// 视频流
#include <fstream>

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
ifstream myFile("/home/young/METOAK/moWork.bin", ios::in | ios::binary);

int32_t main(int32_t argc, char **argv) {
    signal(SIGINT, signalHandler);
    myFile.seekg(0);

    // Init ORB-SLAM3
    ORB_SLAM3::System SLAM(vocFile,settingFile,ORB_SLAM3::System::STEREO,true);
    auto start = chrono::system_clock::now();

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

    // Get video resolution
    uint16_t u16VideoFrameWidth = 0;
    uint16_t u16VideoFrameHeight = 0;
    n32Result = moGetVideoResolution(hCameraHandle, &u16VideoFrameWidth, &u16VideoFrameHeight);
    if (0 != n32Result) {
        printf("Get video resolution failed %d\n", n32Result);
        return -1;
    }

    while (myFile) {

        int leftLength = sizeof(uint8_t) * u16VideoFrameHeight * u16VideoFrameWidth;
        int rightLength = sizeof(uint8_t) * 1.5 * u16VideoFrameHeight * u16VideoFrameWidth;
        char pu8OnlyY[leftLength];
        char pu8RectifiedRightYUVI420Img[rightLength];
        myFile.read (pu8OnlyY, leftLength);
        myFile.read (pu8RectifiedRightYUVI420Img, rightLength);

        // 20 Hz
        // 大约等待一半的时间
        // usleep(25000);

        Mat matLeftBGR;
        Mat matLeftRectify(u16VideoFrameHeight, u16VideoFrameWidth, CV_8UC1, pu8OnlyY);
        cvtColor(matLeftRectify, matLeftBGR, COLOR_GRAY2BGR);

        Mat matRightBGR;  // 1.5 = 12bits / 8bits
        Mat matRightRectify(u16VideoFrameHeight * 1.5, u16VideoFrameWidth, CV_8UC1, pu8RectifiedRightYUVI420Img);
        cvtColor(matRightRectify, matRightBGR, COLOR_YUV2BGR_I420);

        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        SLAM.TrackStereo(matLeftBGR, matRightBGR, double(timestamp.count() / 1000.0));

        if(bEnd) {
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
