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
string settingFile = "/home/young/SLAM/ORB_SLAM3/METOAK/U60RGBD.yaml";
string streamFile = "/home/young/METOAK/moRGBD.bin";

int32_t main(int32_t argc, char **argv) {
    // 处理Ctrl+C
    signal(SIGINT, signalHandler);

    if (argc != 4) {
        cerr << endl << "Usage: ./moLocalRGBD.cpp vocabularyPath settingPath streamPath" << endl;
        return 1;
    }

    ifstream myFile(argv[3], ios::in | ios::binary);
    myFile.seekg(0);

    // Init ORB-SLAM3
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);
    auto start = chrono::system_clock::now();

    MO_CAMERA_HANDLE hCameraHandle = MO_INVALID_HANDLE;

    int32_t n32Result = 0;

    // Get video resolution
    uint16_t u16VideoFrameWidth = 1280;
    uint16_t u16VideoFrameHeight = 720;

    while (myFile) {

        // Width * Height * 2 bytes
        int lengthRGB = 2 * u16VideoFrameHeight * u16VideoFrameWidth;
        // Width * Height * 1.5 bytes
        int lengthD = 1.5 * u16VideoFrameHeight * u16VideoFrameWidth;
        uint16_t pu16RGBDDisparityData[lengthRGB];
        uint8_t pu8RGBDYUVI420Img[lengthD];
        myFile.read ((char*)pu16RGBDDisparityData, lengthRGB);
        myFile.read ((char*)pu8RGBDYUVI420Img, lengthD);

        Mat matYUV2BGR;  // 12bits / 8bits
        Mat matYUVI420(u16VideoFrameHeight * 1.5, u16VideoFrameWidth, CV_8UC1, pu8RGBDYUVI420Img);
        cvtColor(matYUVI420, matYUV2BGR, COLOR_YUV2BGR_I420);

        Mat matDisparity(u16VideoFrameHeight, u16VideoFrameWidth, CV_8UC3);
        GetDisparityImage(matDisparity.rows, matDisparity.cols, matDisparity.step, pu16RGBDDisparityData,
                          matDisparity.data);

        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        SLAM.TrackRGBD(matYUV2BGR, matDisparity, double(timestamp.count() / 1000.0));

        // 20 Hz
        // 大约等待一半的时间
         usleep(25000);
        
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
