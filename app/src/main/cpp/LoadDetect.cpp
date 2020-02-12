//
// Created by wenpd on 2020/2/6.
//

#include "LoadDetect.h"
#include <cmath>

Line::Line(Point2f start, Point2f end) {
    mStart = start;
    mEnd = end;
    mLen = pow(start.x - end.x, 2) + pow(start.y - end.y, 2);
}

float Line::getMidX() const{
    return (mStart.x + mEnd.x) / 2;
}

float Line::getMidY() const{
    return (mStart.y + mEnd.y) / 2;
}

bool Line::operator<(const Line &line) {
    if(mLen > line.mLen){
        return true;
    }
    return false;
}
bool Line::operator==(const Line& line){
    if(mStart.x == line.mStart.x && mStart.y == line.mStart.y &&
            mEnd.x == line.mEnd.x && mEnd.y == line.mEnd.y) {
        return true;
    }
    return false;
}

bool LoadDetect::compareByLen(const Line& lLine, const Line& rLine){
    if(lLine.mLen > rLine.mLen){
        return true;
    }
    return false;
}

bool LoadDetect::compareByMidX(const Line& lLine, const Line& rLine) {
    if(lLine.getMidX() < rLine.getMidX()) {
        return true;
    }
    return false;
}

LoadDetect::LoadDetect(Mat rgbMat, int width, int height) {
    mRgbMat = rgbMat;
    mWidth = width;
    mHeight = height;
    mShowGray = true;
}

void LoadDetect::setShowGray(bool gray) {
    mShowGray = gray;
}

void LoadDetect::detect() {
    Mat tmp, gray;
    cvtColor(mRgbMat, tmp, COLOR_RGBA2GRAY, 4);
    GaussianBlur(tmp, tmp, Size(5, 5), 0);
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(15, 20));
    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(15, 20));
    //erode(gray, gray, erodeElement);
    dilate(tmp, tmp, dilateElement);
    erode(tmp, tmp, erodeElement);
    Canny(tmp, gray, 50, 50);

    vector<Vec4f> lines;
    int threshold = 150;
    int minLineSize = 50;
    int lineGap = 35;
    int number = 0;
    float av_x1 =0, av_y1 = 0,av_x2 =0, av_y2 = 0;
    HoughLinesP(gray, lines, 1, CV_PI / 180, threshold, minLineSize, lineGap);
    if(lines.size() < 200) {
        Canny(tmp, gray, 20, 20);
        lines.clear();
        HoughLinesP(gray, lines, 1, CV_PI / 180, threshold, minLineSize, lineGap);
    }
    if(mShowGray){
        cvtColor(gray, mRgbMat, COLOR_GRAY2RGBA);
    }
    LOGI("Line0 HoughLinesP-=%d", lines.size());
    mLines.clear();
    for(int i=0; i<lines.size(); ++i){
        Vec4f hLine = lines[i];
        float x1 = hLine[0], y1 = hLine[1], x2 = hLine[2], y2 = hLine[3];
        if(filter(x1, y1, x2, y2)) {
            Point2f start = Point2f(x1, y1);
            Point2f end = Point2f(x2, y2);
            mLines.push_back(Line(start, end));
            number++;
            if(x1 + x2 < mWidth) {
                line(mRgbMat, start, end, Scalar(255,0,0,255), 3);
            } else
            line(mRgbMat, start, end, Scalar(255,0,0,255), 3);

            av_x1 += x1;
            av_y1 += y1;
            av_x2 += x2;
            av_y2 += y2;
        }
    }
    LOGI("Line0 number-=%d", number);
    if(number != 0) {
        av_x1 /= number;
        av_y1 /= number;
        av_x2 /= number;
        av_y2 /= number;
        Point2f start(av_x1, av_y1);
        Point2f end(av_x2, av_y2);
        //line(mRgbMat, start, end, Scalar(0,255,0,255), 3);
        float angle = getAngle(mLines);
        //line(mRgbMat, mLines[0].mStart, mLines[0].mEnd, Scalar(0,255,255,255), 6);
        //LOGI("Line0 len-=%.2f", mLines[0].mLen);
        //sort(mLines.begin(), mLines.end(), compareByMidX);
        //line(mRgbMat, mLines[0].mStart, mLines[0].mEnd, Scalar(255,255,0,255), 6);
        LOGI("Line0 angle-=%.2f", angle);
        if(fabs(angle) < 20) {
            Line midLine(start, end);
            mLines.push_back(midLine);
            sort(mLines.begin(), mLines.end(), compareByMidX);
            auto startIter = std::begin(mLines);
            auto endIter = std::end(mLines);
            startIter = find(startIter, endIter, midLine);
            LOGI("Line0 startIter-=%d", startIter);
            if(startIter != endIter){
                vector<Line> twoLine = getTwoLines(mLines, startIter);
                start = Point2f((twoLine[0].mStart.x + twoLine[1].mStart.x)/2, (twoLine[0].mStart.y + twoLine[1].mStart.y)/2);
                end = Point2f((twoLine[0].mEnd.x + twoLine[1].mEnd.x)/2, (twoLine[0].mEnd.y + twoLine[1].mEnd.y)/2);
                midLine = Line(start, end);
                line(mRgbMat, twoLine[0].mStart, twoLine[0].mEnd, Scalar(0,255,0,255), 6);
                line(mRgbMat, twoLine[1].mStart, twoLine[1].mEnd, Scalar(0,255,0,255), 6);
                line(mRgbMat, midLine.mStart, midLine.mEnd, Scalar(255,255,0,255), 6);

            }
            //line(mRgbMat, start, end, Scalar(0,255,0,255), 3);
        }
    }
}

bool LoadDetect::filter(float x1, float y1, float x2, float y2) {
    float minYGap = 20, minXGap = 20;
    float minYXGap = 0.5;
    float yMax = 0.5 * mHeight;
    if(fabs(y2 - y1)< minYGap || fabs(x2 - x1)< minXGap) {
        return false;
    }
    if(abs((y2 - y1)/(x2 - x1))< minYXGap) {
        //return false;
    }
    if((y2 + y1) / 2 < yMax) {
        return false;
    }
    return true;
}

float LoadDetect::getAngle(const vector<Line> &lines) {
    float angle = 0.0;
    float angleTemp= 0.0;
    float a_cos = 0.0;
    float mark = 1.0;
    int number = 0;
    float length;

    for(int index=0;index<lines.size();index+=1) {
        Point2f start = lines[index].mStart, end = lines[index].mEnd;
        if((end.x - start.x)!=0 && (end.y-start.y)!=0) {
            length = sqrt(pow(fabs(end.y - start.y), 2) + pow(fabs(end.x - start.x), 2));
            a_cos = (end.x - start.x)/length;
            mark = (end.y - start.y)/( end.x - start.x);
            angleTemp = acos(a_cos)*180/CV_PI;
            if(angleTemp < 0) {
                angleTemp = -angleTemp;
            }
            if(mark>0) {
                angle += angleTemp;
            }else {
                angle -= angleTemp;
            }
            number++;
        }
    }
    if(number!=0) {
        angle = angle/number;
    }else {
        angle = 0.0;
    }
    return angle;
}

vector<Line> LoadDetect::getTwoLines(const vector<Line> &lines, const vector<Line>::iterator &mid) {
    vector<Line> result;
    vector<float> pointX;
    vector<float> pointY;
    for(auto iter = lines.begin(); iter < mid; ++iter) {
        pointX.push_back((*iter).mStart.x);
        pointX.push_back((*iter).mEnd.x);
        pointY.push_back((*iter).mStart.y);
        pointY.push_back((*iter).mEnd.y);
    }
    Line leftLine = leastSquaresTechnique(pointX, pointY);
    result.push_back(leftLine);
    pointX.clear();
    pointY.clear();
    for(auto iter = mid + 1; iter < lines.end(); ++iter) {
        pointX.push_back((*iter).mStart.x);
        pointX.push_back((*iter).mEnd.x);
        pointY.push_back((*iter).mStart.y);
        pointY.push_back((*iter).mEnd.y);
    }
    Line rLine = leastSquaresTechnique(pointX, pointY);
    result.push_back(rLine);
    return result;
}

Line LoadDetect::leastSquaresTechnique(const vector<float> &pointX, const vector<float> &pointY) {
    Point2f points[2];
    float sumx = 0.0, sumy = 0.0, sumx2 = 0.0;
    int number=0;
    while (number < pointX.size()) {
        sumx += pointX[number];
        sumx2 += pointX[number] * pointX[number];
        sumy += pointY[number];
        number++;
    }

    float xbar = sumx / number;
    float ybar = sumy / number;
    float xxbar = 0.0, yybar = 0.0, xybar = 0.0;
    for (int i = 0; i < number; i++) {
        xxbar += (pointX[i] - xbar) * (pointX[i]  - xbar);
        yybar += (pointY[i] - ybar) * (pointY[i] - ybar);
        xybar += (pointX[i]  - xbar) *(pointY[i] - ybar);
    }
    float k = xybar / xxbar;
    float b = ybar - k * xbar;

    if(k < 0 && b > 0) {
        if(b < mHeight && (-b/k) < mWidth) {
            points[0] = Point2f(0, b);
            points[1] = Point2f(-b/k, 0);
        }else if(b > mHeight && (-b/k) < mWidth){
            points[0] = Point2f((mHeight-b)/k, mHeight);
            points[1] = Point2f(-b/k, 0);
        }else if(b > mHeight && (-b/k) > mWidth){
            points[0] = Point2f((mHeight-b)/k, mHeight);
            points[1] = Point2f(mWidth, k*mWidth+b);
        }else if(b < mHeight && (-b/k) > mWidth) {
            points[0] = Point2f(0, b);
            points[1] = Point2f(mWidth, k*mWidth+b);
        }
    }else if(k > 0 && b < 0) {
        if(-b/k < mWidth && (mHeight - b)/mWidth < k) {
            points[0] = Point2f((mHeight-b)/mWidth, mHeight);
            points[1] = Point2f(-b/k, 0);
        }else if(-b/k < mWidth && (mHeight - b)/mWidth > k) {
            points[0] = Point2f(mWidth, mWidth*k+b);
            points[1] = Point2f(-b/k, 0);
        }
    }else if(k > 0 && b > 0) {
        if(b < mHeight && mWidth*k+b < mHeight) {
            points[0] = Point2f(0, b);
            points[1] = Point2f(mWidth, mWidth*k+b);
        }else if(b < mHeight && mWidth*k+b > mHeight){
            points[0] = Point2f(0, b);
            points[1] = Point2f((mHeight-b)/k, mHeight);
        }
    }

    LOGI("line0 y=%.2f x +  %.2f", k, b);
    return Line(points[0], points[1]);
}