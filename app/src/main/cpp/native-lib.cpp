#include <jni.h>
#include <string>
#include <cmath>
#include <android/bitmap.h>
#include <opencv2/opencv.hpp>
#include "LoadDetect.h"
#include "LaneDetector.h"

using namespace cv;
using namespace std;

extern "C" JNIEXPORT void JNICALL
Java_com_stu_opencv_MainActivity_getEdge(
        JNIEnv *env,
        jobject /* this */,
        jobject bitmap,
        jboolean showGray) {
    AndroidBitmapInfo info;
    void *pixels;
    int i;

    CV_Assert(AndroidBitmap_getInfo(env, bitmap, &info) >= 0);
    CV_Assert(info.format == ANDROID_BITMAP_FORMAT_RGBA_8888 ||
              info.format == ANDROID_BITMAP_FORMAT_RGB_565);
    CV_Assert(AndroidBitmap_lockPixels(env, bitmap, &pixels) >= 0);
    CV_Assert(pixels);
    if (info.format == ANDROID_BITMAP_FORMAT_RGBA_8888) {
        Mat rgbMat(info.height, info.width, CV_8UC4, pixels);
        //LoadDetect ld = LoadDetect(rgbMat, info.width, info.height);
        LaneDetector ld = LaneDetector(rgbMat, info.width, info.height);
        ld.setShowGray(showGray);
        ld.detect();
    } else {
        Mat temp(info.height, info.width, CV_8UC2, pixels);
        Mat gray;
        cvtColor(temp, gray, COLOR_RGB2GRAY);
        Canny(gray, gray, 125, 225);
        cvtColor(gray, temp, COLOR_GRAY2RGB);
    }
    AndroidBitmap_unlockPixels(env, bitmap);
}
