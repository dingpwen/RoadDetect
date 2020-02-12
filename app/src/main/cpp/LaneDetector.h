//
// Created by wenpd on 2020/2/12.
//
#include <jni.h>
#include <string>
#include "log.h"
#include <opencv2/opencv.hpp>
#include "LaneDetectorBase.h"

using namespace cv;
using namespace std;

#ifndef OPENCV_APPLICATION_LANEDETECTOR_H
#define OPENCV_APPLICATION_LANEDETECTOR_H


class LaneDetector:public LaneDetectorBase {
private:
    int mHoughValue;

public:
    LaneDetector(Mat rgbMat, int width, int height):LaneDetectorBase(rgbMat, width, height){
        mHoughValue = 150;
    }
    virtual void detect();
};


#endif //OPENCV_APPLICATION_LANEDETECTOR_H
