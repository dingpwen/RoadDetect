#include <jni.h>
#include <string>
#include <cmath>
#include <android/bitmap.h>
#include <opencv2/opencv.hpp>
#include "LoadDetect.h"
#include "LaneDetector.h"
#include "RoadEdge.h"

using namespace cv;
using namespace std;

extern "C" JNIEXPORT void JNICALL
Java_com_stu_opencv_MainActivity_getEdge(
        JNIEnv *env,
        jobject /* this */,
        jobject bitmap,
        jint hough,
        jboolean showGray) {
    AndroidBitmapInfo info;
    void *pixels;
    int ret;

    CV_Assert(AndroidBitmap_getInfo(env, bitmap, &info) >= 0);
    CV_Assert(info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 ||
              info.format == ANDROID_BITMAP_FORMAT_RGB_565);
    CV_Assert(AndroidBitmap_lockPixels(env, bitmap, &pixels) >= 0);
    CV_Assert(pixels);
    if (info.format == ANDROID_BITMAP_FORMAT_RGBA_8888) {
        Mat rgbMat(info.height, info.width, CV_8UC4, pixels);
        //LoadDetect ld(rgbMat, info.width, info.height);
        //LaneDetector ld(rgbMat, info.width, info.height);
        RoadEdge ld(rgbMat, info.width, info.height);
        ld.setShowGray(showGray);
        ld.setHoughVote(hough);
        ret = ld.detect();
    } else {
        Mat rgbMat(info.height, info.width, CV_8UC2, pixels);
        RoadEdge ld(rgbMat, info.width, info.height);
        ld.setShowGray(showGray);
        ret = ld.detect();
    }
    AndroidBitmap_unlockPixels(env, bitmap);
    switch(ret) {
        case 0:
            LOGI("line0-detect: go straight");
            break;
        case 1:
            LOGI("line0-detect: turn left");
            break;
        case 2:
            LOGI("line0-detect: turn right");
            break;
        case 3:
            LOGI("line0-detect: Obstacles on the left");
            break;
        case 4:
            LOGI("line0-detect: Obstacles on the right");
            break;
        case 5:
            LOGI("line0-detect:  Obstacles on the front");
            break;
        case 6:
            LOGI("line0-detect: crossed road");
            break;
        default:
            break;
    }
}
