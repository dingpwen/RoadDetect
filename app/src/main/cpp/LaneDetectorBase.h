//
// Created by wenpd on 2020/2/12.
//
#include <jni.h>
#include <opencv2/opencv.hpp>

using namespace cv;

#ifndef OPENCV_APPLICATION_LANEDETECTORBASE_H
#define OPENCV_APPLICATION_LANEDETECTORBASE_H


class LaneDetectorBase {
protected:
    int mWidth, mHeight;
    Mat mRgbMat;
    bool mShowGray;

public:
    LaneDetectorBase(Mat rgbMat, int width, int height):mRgbMat(rgbMat), mWidth(width), mHeight(height){}
    void setShowGray(bool gray){ mShowGray = gray; }
    virtual void processImage(Mat &img) = 0;
    virtual void detect() { processImage(mRgbMat); }
};


#endif //OPENCV_APPLICATION_LANEDETECTORBASE_H
