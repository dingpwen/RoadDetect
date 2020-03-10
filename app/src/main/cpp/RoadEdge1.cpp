//
// Created by wenpd on 2020/2/16.
//

#include "RoadEdge.h"
#include "ImageProcess.h"

void RoadEdge::grayScale(Mat &src, Mat &dst) {
    cvtColor(src, dst, COLOR_RGBA2GRAY);
}

void RoadEdge::regionOfInterest(Mat &img, const Point **pts, const int *npts) {
    Mat mask(img.size(), CV_8U, kColorGrayDefault);
    fillPoly(mask, pts, npts, 1, kColorGrayMax);
    bitwise_and(img, mask, img);
}

void RoadEdge::drawLines(Mat &img, const vector<Vec4f> &lines, const Scalar& color, int thickness) {
    if(lines.empty()) {
        return ;
    }
    LOGI("line0 size:%d", lines.size());
    vector<float> slopeL, slopeR;
    vector<float> bL, bR;
    float slope, b;
    bool presentL = false, presentR = false;
    float x1, y1, x2, y2;
    float halfW = mWidth/(float)2;
    float halfH = mHeight/(float)2;
    for(Vec4f curLine:lines) {
        x1 = curLine[0], y1 = curLine[1], x2 = curLine[2], y2 = curLine[3];
        if(x1 == x2) continue;
        slope = ((y2-y1)/(x2-x1));
        b = y1 - slope * x1;
        if (slope > MIN_SLOPE && slope < MAX_SLOPE && (x1 > halfW && x2 > halfW) && (y1 > halfH && y2 > halfH)) {
            //Right part
            presentR = true;
            slopeR.push_back(slope);
            bR.push_back(b);
            line(img, Point2f(x1, y1), Point2f(x2, y2), Scalar(0,255,0,255), thickness);
        } else if (slope < -MIN_SLOPE && slope > -MAX_SLOPE &&(x1 < halfW && x2 < halfW) && (y1 > halfH && y2 > halfH)) {
            //Left part
            presentL = true;
            slopeL.push_back(slope);
            bL.push_back(b);
            line(img, Point2f(x1, y1), Point2f(x2, y2), Scalar(255,0,255,255), thickness);
        }
    }
    if(presentR) {
        float meanSlopR = getMean(slopeR);
        float meanBR = getMean(bR);
        LOGI("line0-meanSlopR:%.2f", angle(meanSlopR));
        x1 = mWidth;
        y1 = meanSlopR * x1 + meanBR;
        y2 = mHeight / (float)2;
        x2 = ( y2 - meanBR ) / meanSlopR;
        line(img, Point2f(x1, y1), Point2f(x2, y2), color, thickness);
    }
    if(presentL) {
        float meanSlopL = getMean(slopeL);
        float meanBL = getMean(bL);
        LOGI("line0-meanSlopL:%.2f", angle(meanSlopL));
        x1 = 0;
        y1 = meanSlopL * x1 + meanBL;
        y2 = mHeight / (float)2;
        x2 = (y2 - meanBL) / meanSlopL;
        line(img, Point2f(x1, y1), Point2f(x2, y2), color, thickness);
    }

    line(img, Point2f(0, halfH), Point2f(mWidth, halfH), Scalar(0,255,255,0), thickness *2);
}

template <typename T>
T RoadEdge::getMean(vector<T> values) {
    if(values.empty()) {
        return 0;
    }
    T mean = 0;
    for(T value:values) {
        mean += value;
    }
    mean /= values.size();
    return mean;
}

Mat RoadEdge::houghLines(Mat &img, int threshold, double minLineLen, double maxLineGap) {
    vector<Vec4f> lines;
    HoughLinesP(img, lines, 1, CV_PI / 180, threshold, minLineLen, maxLineGap);
    //Mat lineImage(img.size(), CV_8UC4, kColorDefault);
    Mat lineImage = Mat::zeros(img.size(), CV_8UC4);
    drawLines(lineImage, lines);
    return lineImage;
}

Mat RoadEdge::enhance(const Mat &img) {
    Mat enhanceImage;
    Mat kernel = (Mat_<float>(3, 3) << 0, -1, 0, -1, 6, -1, 0, -1, 0);
    filter2D(img, enhanceImage, img.depth(), kernel);
    LOGI("line0 enhance:%d", img.depth());
    return enhanceImage;
}

int RoadEdge::processImage(Mat &img) {
    Mat blur, gray;
    LOGI("line0-size:%d&%d", mHeight, mWidth);
    //Mat enhanceImage = enhance(blur);
    GaussianBlur(img, blur, Size(kSize, kSize), 0);
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(15, 20));
    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(15, 20));
    dilate(blur, blur, dilateElement);
    erode(blur, blur, erodeElement);
    Canny(blur, gray, 100, mHoughVote);
    if (mShowGray) {
        //Mat kernel = (Mat_<float>(3, 3) << 0, -1, 0, -1, 3, -1, 0, -1, 0);
        //filter2D(img, img, img.depth(), kernel);
        //cvtColor(enhanceImage, img, COLOR_RGB2RGBA);
        //ImageProcess(img).adaptiveContrastEnhancement(20, 20, 0.2);
        Mat src;
        cvtColor(img, src, COLOR_RGBA2RGB);
        Mat Sal = Mat::zeros(src.size(), CV_8UC1);
        int R2 = (mHeight > mWidth) ? mWidth : mHeight;
        LOGI("line0-AC start");
        ImageProcess::SalientRegionDetectionBasedonAC(src, Sal, R2 >> 3, R2 >> 1);
        LOGI("line0-AC end");
        Mat dst = Mat::zeros(src.size(), CV_8UC3);
        ImageProcess::salientSegment(src, Sal, dst);
        //cvtColor(dst, img, COLOR_RGB2RGBA);

        cvtColor(dst, img, COLOR_RGB2RGBA);
    } else {
        Mat src;
        cvtColor(img, src, COLOR_RGBA2RGB);
        //Mat dst = Mat::zeros(src.size(), CV_8UC3);
        //ImageProcess::washedSegment(src, dst);
        //ImageProcess::meanShift(src, dst);
        //cvtColor(dst, img, COLOR_RGB2RGBA);
        Mat dst = Mat::zeros(src.size(), CV_8UC1);
        ImageProcess::kMean(src, dst);
        cvtColor(dst, img, COLOR_GRAY2RGBA);

        /*GaussianBlur(img, blur, Size(kSize, kSize), 0);
        Mat dilateElement = getStructuringElement(MORPH_RECT, Size(15, 20));
        Mat erodeElement = getStructuringElement(MORPH_RECT, Size(15, 20));
        //erode(gray, gray, erodeElement);
        dilate(blur, blur, dilateElement);
        erode(blur, blur, erodeElement);

        Canny(blur, gray, 100, 150);*/
    }

    Point points[1][6];
    points[0][0] = Point(0, mHeight);
    points[0][1] = Point(0, mHeight/2);
    points[0][2] = Point(mWidth/3, mHeight/3);
    points[0][3] = Point(2*mWidth/3, mHeight/3);
    points[0][4] = Point(mWidth, mHeight/2);
    points[0][5] = Point(mWidth, mHeight);
    Point *pts[1] = { points[0] };
    int npt[] = { 6 };
    regionOfInterest(gray, (const Point **)pts, npt);
    Mat lineImage = houghLines(gray, 35, 15, 100);
    addWeighted(lineImage, 0.8, img, 1.0, 0, img);
    return 0;
}