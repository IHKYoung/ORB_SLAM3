#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>

#include "mo_stereo_camera_driver_c.h"
#include "mo_stereo_camera_driver_c_reserved.h"

#define STDIN 0
#define MAXFD 1  // STDIN + 1

#define FETCH_AND_DISPLAY_TIME_LENGTH 10
#define FETCH_IMU_TIME_LENGTH 3

// make the array of colors table
extern unsigned char* g_caColorTable;

void ms_sleep(uint16_t u16Millisecond);

int getchar_nb(uint16_t u16Timeout);

unsigned char* getColorTable();

int cameraSampleInit(int32_t argc, char** argv, char* caCameraPath);

int32_t GetDisparityImage(IN const uint16_t u16Height, IN const uint16_t u16Width, IN const uint16_t u16Step,
                          IN const uint16_t* pu16DepthData, OUT uint8_t* const pu8DepthImage);

#define BEGIN_TO_WORK(argc, argv1)                                                                    \
    int32_t s32Result = 0;                                                                            \
    do {                                                                                              \
        char caCameraPath[16];                                                                        \
        if ((1 < argc) && ((('0' <= argv[1][0]) && ('9' >= argv[1][0])) &&                            \
                           (('\0' == argv[1][1]) || (('0' <= argv[1][1]) && ('9' >= argv[1][1]))))) { \
            sprintf(caCameraPath, "/dev/video%s", argv1);                                             \
        } else {                                                                                      \
            sprintf(caCameraPath, "/dev/video0");                                                     \
        }

#define BEGIN_GET_VIDEO_FRAME_LOOP       \
    printf("press ENTER key to quit\n"); \
    while (0 == getchar_nb(FETCH_AND_DISPLAY_TIME_LENGTH)) {
#define END_GET_VIDEO_FRAME_LOOP }

#define END_TO_WORK \
    }               \
    while (0)       \
        ;
