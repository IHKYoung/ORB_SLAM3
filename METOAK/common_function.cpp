#include "common_function.h"

void ms_sleep(uint16_t u16Millisecond) {
    struct timeval stTV;
    stTV.tv_sec = 0;
    stTV.tv_usec = u16Millisecond * 1000;
    select(0, NULL, NULL, NULL, &stTV);
}

int getchar_nb(uint16_t u16Timeout)  // millisecond
{
    fd_set fdReadSet;
    struct timeval sTimeout;

    FD_ZERO(&fdReadSet);
    FD_SET(STDIN, &fdReadSet);

    sTimeout.tv_sec = u16Timeout / 1000;
    sTimeout.tv_usec = (u16Timeout % 1000) * 1000;

    // check whether any key has pressed
    if ((0 < select(MAXFD, &fdReadSet, NULL, NULL, &sTimeout)) && (0 != FD_ISSET(STDIN, &fdReadSet))) {
        return getchar();
    }

    return 0;
}

int cameraSampleInit(int32_t argc, char** argv, char* caCameraPath) {
    if (argc < 2) {
        // Find Camera automatically
        moCameraList* pCamList;
        moQueryCameraList(&pCamList);
        if (pCamList->size < 1) {
            printf("Failed to locate camera automatically\n");
            return -1;
        } else {
            moPrintCameraList(pCamList);
            uint32_t u32ListIdx = 0;
            if (pCamList->size > 1) {
                printf("Please select a camera by index and use enter key to confirm\n");
                scanf("%d", &u32ListIdx);

                if (u32ListIdx >= pCamList->size) {
                    printf("Selected index [%d] out of range\n", u32ListIdx);
                    return -1;
                }
            }
            printf("\nSelected %d, dev %s\n", u32ListIdx, pCamList->pCameraInfos[u32ListIdx].devName);
            strcpy(caCameraPath, pCamList->pCameraInfos[u32ListIdx].devName);
        }

        moReleaseCameraList(&pCamList);
    } else {
        // Open Camera by specified video idx
        sprintf(caCameraPath, "/dev/video%s", argv[1]);
    }
    printf("Opening Camera Device: %s\n", caCameraPath);
    return 0;
}

unsigned char* getColorTable() {
    int idx;
    // 24576 = 3 * 8192 * sizeof(unsigned char)
    static unsigned char baColorTable[24576];

    baColorTable[0] = 0;
    baColorTable[1] = 0;
    baColorTable[2] = 0;

    for (idx = 1; idx <= 24; idx++) {
        baColorTable[idx * 3] = 255;
        baColorTable[idx * 3 + 1] = 255;
        baColorTable[idx * 3 + 2] = 255;
    }

    for (idx = 25; idx <= 40; idx++) {
        // 7.9375 = ((255.0 - 128.0) / (40.0 - 24.0))
        baColorTable[idx * 3] = (int)(255 - (int)(7.9375 * (idx - 24)));
        baColorTable[idx * 3 + 1] = (int)(255 - (int)(7.9375 * (idx - 24)));
        baColorTable[idx * 3 + 2] = (int)(255 - (int)(7.9375 * (idx - 24)));
    }

    for (idx = 41; idx <= 64; idx++) {
        baColorTable[idx * 3] = (int)(128 + (int)(5.291668 * (idx - 41)));
        baColorTable[idx * 3 + 1] = (int)(128 - (int)(5.291668 * (idx - 41)));
        baColorTable[idx * 3 + 2] = (int)(128 + (int)(5.291668 * (idx - 41)));
    }

    for (idx = 65; idx <= 120; idx++) {
        baColorTable[idx * 3] = (int)(255 - (int)(4.553571 * (idx - 64)));
        baColorTable[idx * 3 + 1] = 0;
        baColorTable[idx * 3 + 2] = 255;
    }

    for (idx = 121; idx <= 176; idx++) {
        baColorTable[idx * 3] = 0;
        baColorTable[idx * 3 + 1] = (int)(0 + (int)(4.553571 * (idx - 120)));
        baColorTable[idx * 3 + 2] = 255;
    }

    for (idx = 177; idx <= 320; idx++) {
        baColorTable[idx * 3] = 0;
        baColorTable[idx * 3 + 1] = 255;
        baColorTable[idx * 3 + 2] = (int)(255 - (int)(1.770833 * (idx - 176)));
    }

    for (idx = 321; idx <= 800; idx++) {
        baColorTable[idx * 3] = (int)(0 + (int)(0.53125 * (idx - 320)));
        baColorTable[idx * 3 + 1] = 255;
        baColorTable[idx * 3 + 2] = 0;
    }

    for (idx = 801; (idx <= 2048); idx++) {
        baColorTable[idx * 3] = 255;
        baColorTable[idx * 3 + 1] = (int)(255 - (int)(0.204327 * (idx - 800)));
        baColorTable[idx * 3 + 2] = 0;
    }

    for (idx = 2049; idx < 8192; idx++) {
        baColorTable[idx * 3] = 255;
        baColorTable[idx * 3 + 1] = 0;
        baColorTable[idx * 3 + 2] = 0;
    }

    return baColorTable;
}

// make the array of colors table
unsigned char* g_caColorTable = getColorTable();

/** \brief Get the image of disparity data
 *
 * \param u16Height - the height of video frame
 * \param u16Width - the widht of video frame
 * \param u16Step - the step length of depth image data buffer
 * \param pu16DepthData - points depth data of video frame
 * \param pu8DepthImage - points colored image data of depth data
 * \return 0 - success, else - failure
 *        -1 - parameter is invalid
 */
int32_t GetDisparityImage(IN const uint16_t u16Height, IN const uint16_t u16Width, IN const uint16_t u16Step,
                          IN const uint16_t* pu16DepthData, OUT uint8_t* const pu8DepthImage) {
    int32_t s32Result = 0;

    do {
        if ((0 >= u16Width) || (0 >= u16Height) || (0 >= u16Step) || (NULL == pu16DepthData) ||
            (NULL == pu8DepthImage)) {
            s32Result = -1;
            break;
        }

        for (uint16_t u16IdxH = 0; u16IdxH < u16Height; u16IdxH++) {
            unsigned char* pu8ImageData = pu8DepthImage + u16IdxH * u16Step;

            for (uint16_t u16IdxW = 0; u16IdxW < u16Width; u16IdxW++) {
                uint16_t u32RawDisparityValue = (*(pu16DepthData++)) & ((unsigned short)0x1fff);

                if (0 < u32RawDisparityValue) {
                    *(pu8ImageData++) = g_caColorTable[u32RawDisparityValue * 3 + 2];
                    *(pu8ImageData++) = g_caColorTable[u32RawDisparityValue * 3 + 1];
                    *(pu8ImageData++) = g_caColorTable[u32RawDisparityValue * 3];
                } else {
                    *(pu8ImageData++) = 0;
                    *(pu8ImageData++) = 0;
                    *(pu8ImageData++) = 0;
                }
            }
        }

    } while (0);

    return s32Result;
}
