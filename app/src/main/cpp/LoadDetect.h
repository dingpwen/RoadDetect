//
// Created by wenpd on 2020/2/6.
//
#include <jni.h>
#include <string>
#include <android/log.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#ifndef OPENCV_APPLICATION_LOADDETECT_H
#define OPENCV_APPLICATION_LOADDETECT_H

#define TAG "LoadDetect-JNI" // 这个是自定义的LOG的标识
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,TAG ,__VA_ARGS__) // 定义LOGD类型
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,TAG ,__VA_ARGS__) // 定义LOGI类型
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,TAG ,__VA_ARGS__) // 定义LOGW类型
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,TAG ,__VA_ARGS__) // 定义LOGE类型

class Line{
public:
    Point2f mStart, mEnd;
    float mLen;
    Line(Point2f start, Point2f end);
    float getMidX() const;
    float getMidY() const;
    bool operator<(const Line& line);
    bool operator==(const Line& line);
};

class LoadDetect {
private:
    int mWidth, mHeight;
    Mat mRgbMat;
    vector<Line> mLines;
    bool mShowGray;
    bool filter(float x1, float y1, float x2, float y2);
    float getAngle(const vector<Line> &lines);
    Line leastSquaresTechnique(const vector<float> &pointX, const vector<float> &pointY);
    vector<Line> getTwoLines(const vector<Line> &lines,const vector<Line>::iterator &mid);
    static bool compareByLen(const Line& lLine, const Line& rLine);
    static bool compareByMidX(const Line& lLine, const Line& rLine);

public:
    LoadDetect(Mat rgbMat, int width, int height);
    void setShowGray(bool gray);
    void detect();
};


#endif //OPENCV_APPLICATION_LOADDETECT_H
