//
// Created by wenpd on 2020/2/16.
//

#include "RoadEdge.h"

void RoadEdge::grayScale(Mat &src, Mat &dst) {
    cvtColor(src, dst, COLOR_RGBA2GRAY, 4);
}

void RoadEdge::regionOfInterest(Mat &img, const Point **pts, const int *npts) {
    Mat mask(img.size(), CV_8U, Scalar(0));
    fillPoly(mask, pts, npts, 1, Scalar(255));
    bitwise_and(img, mask, img);
}

void RoadEdge::drawLines(Mat &img, vector<Vec4f> lines, const Scalar& color, int thickness) {
    if(lines.empty()) {
        return ;
    }
    vector<float> slopeL, slopeR;
    vector<float> bL, bR;
    float slope, b;
    bool presentL = false, presentR = false;
    float x1, y1, x2, y2;
    for(Vec4f curLine:lines) {
        x1 = curLine[0], y1 = curLine[1], x2 = curLine[2], y2 = curLine[3];
        if(x1 == x2) continue;
        slope = ((y2-y1)/(x2-x1));
        b = y1 - slope * x1;
        if (slope > MIN_SLOPE && slope < MAX_SLOPE && (x1 > mWidth/2 && x2 > mWidth/2)) {
            //Right part
            presentR = true;
            slopeR.push_back(slope);
            bR.push_back(b);
            line(img, Point2f(x1, y1), Point2f(x2, y2), Scalar(255,255,0,255), thickness);
        } else if (slope < -MIN_SLOPE && slope > -MAX_SLOPE &&(x1 < mWidth/2 && x2 < mWidth/2)) {
            //Left part
            presentL = true;
            slopeL.push_back(slope);
            bL.push_back(b);
            line(img, Point2f(x1, y1), Point2f(x2, y2), Scalar(255,255,0,255), thickness);
        }
    }
    if(presentR) {
        float meanSlopR = getMean(slopeR);
        float meanBR = getMean(bR);
        x1 = mWidth;
        y1 = meanSlopR * x1 + meanBR;
        y2 = mHeight / (float)3;
        x2 = ( y2 - meanBR ) / meanSlopR;
        line(img, Point2f(x1, y1), Point2f(x2, y2), color, thickness);
    }
    if(presentL) {
        float meanSlopL = getMean(slopeL);
        float meanBL = getMean(bL);
        x1 = 0;
        y1 = meanSlopL * x1 + meanBL;
        y2 = mHeight / (float)3;
        x2 = (y2 - meanBL) / meanSlopL;
        line(img, Point2f(x1, y1), Point2f(x2, y2), color, thickness);
    }
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
    Mat lineImage(img.size(), CV_8UC4, Scalar(0, 0, 0, 0));
    drawLines(lineImage, lines);
    return lineImage;
}

void RoadEdge::processImage(Mat &img) {
    Mat blur, gray;
    GaussianBlur(img, blur, Size(kSize, kSize), 0);
    Canny(blur, gray, 100, 150);

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
}