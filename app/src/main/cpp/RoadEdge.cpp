//
// Created by wenpd on 2020/2/16.
//

#include "RoadEdge.h"

void RoadEdge::grayScale(Mat &src, Mat &dst) {
    cvtColor(src, dst, COLOR_RGBA2GRAY);
}

void RoadEdge::regionOfInterest(Mat &img, const Point **pts, const int *npts) {
    Mat mask(img.size(), CV_8U, kColorGrayDefault);
    fillPoly(mask, pts, npts, 1, kColorGrayMax);
    bitwise_and(img, mask, img);
}

/**
 * find lines to get the orientation of the road
 * @param img draw lines in the img
 * @param lines hough lines
 * @param color
 * @param thickness
 * @return -1:not confirm;0:go straight;1:turn left;2:turn right;3:Obstacles on the left;4:Obstacles on the right;5:Obstacles on the front;6:cross load
 */
int RoadEdge::findLines(Mat &img, const vector<Vec4f> &lines, const Scalar& color, int thickness) {
    int ret = RESULT_NONE;
    if(lines.empty()) {
        return ret;
    }
    LOGI("line0 size:%d", lines.size());
    vector<float> slopeL, slopeR;
    vector<float> bL, bR;
    float slope, b;
    bool presentL = false, presentR = false;
    float x1, y1, x2, y2;
    float halfW = mWidth/(float)2;
    float halfH = mHeight/(float)2;
    float centerLx = 0, centerLy = 0, centerRx = 0, centerRy = 0;
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
            centerRx += ((x1 + x2)/2);
            centerRy += ((y1 + y2)/2);
            line(img, Point2f(x1, y1), Point2f(x2, y2), Scalar(0,255,0,255), thickness);
        } else if (slope < -MIN_SLOPE && slope > -MAX_SLOPE &&(x1 < halfW && x2 < halfW) && (y1 > halfH && y2 > halfH)) {
            //Left part
            presentL = true;
            slopeL.push_back(slope);
            bL.push_back(b);
            centerLx += ((x1 + x2)/2);
            centerLy += ((y1 + y2)/2);
            line(img, Point2f(x1, y1), Point2f(x2, y2), Scalar(255,0,255,255), thickness);
        }
    }
    float gap = 0;
    float lenL = 0, lenR = 0;
    float angleL = 0, angleR = 0;
    if(presentR) {
        centerRx /= slopeR.size();
        centerRy /= slopeR.size();
        float meanSlopR = getMean(slopeR);
        float meanBR = getMean(bR);
        angleR = angle(meanSlopR);
        LOGI("line0-meanSlopR:%.2f", angleR);
        x1 = mWidth;
        y1 = meanSlopR * x1 + meanBR;
        y2 = mHeight / (float)2;
        x2 = ( y2 - meanBR ) / meanSlopR;
        gap = x2;
        lenL = (float)sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
        line(img, Point2f(x1, y1), Point2f(x2, y2), color, thickness);
    }
    if(presentL) {
        centerLx /= slopeL.size();
        centerLy /= slopeL.size();
        float meanSlopL = getMean(slopeL);
        float meanBL = getMean(bL);
        angleL = angle(meanSlopL);
        LOGI("line0-meanSlopL:%.2f", angleL);
        x1 = 0;
        y1 = meanSlopL * x1 + meanBL;
        y2 = mHeight / (float)2;
        x2 = (y2 - meanBL) / meanSlopL;
        gap -= x2;
        lenR = (float)sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
        line(img, Point2f(x1, y1), Point2f(x2, y2), color, thickness);
    }

    if(presentL && presentR) {
        line(img, Point2f(centerLx, centerLy), Point2f(centerRx, centerRy), Scalar(255,255,0,255), thickness);
        if(gap > 0) {
            if((lenL > halfH/2) && (lenR  > halfH/2)) {
                return 0;
            } else if(lenL > halfH/2) {
                if(-angleL > VALID_ANGLE) {
                    return RESULT_GO_STRAIGHT;
                }
                return RESULT_TURN_RIGHT;
            } else if(lenR  > halfH/2){
                if(angleR > VALID_ANGLE) {
                    return RESULT_GO_STRAIGHT;
                }
                return RESULT_TURN_LEFT;
            } else {
                return ret;//not confirm
            }
        } else {//two line crossed
            LOGI("line0-gap:%.2f", gap);
            int xValid = validX(centerLx, centerRx);
            int yValid = validY(centerLy, centerRy);
            if(xValid == -3) {
                return RESULT_OBST_FRONT;
            }
            if(yValid == -3) {
                return ret;
            }
            if(-angleL > VALID_ANGLE && angleR > VALID_ANGLE) {
                if(xValid == 0) {
                    if(fabs(angleR + angleL) < 10) {
                        return RESULT_GO_STRAIGHT;
                    }
                    if(yValid == -1) {
                        return RESULT_TURN_RIGHT;
                    } else if(yValid == -2) {
                        return RESULT_TURN_LEFT;
                    }
                    return RESULT_GO_STRAIGHT;
                } else if(yValid == 0) {
                    if(xValid == -1) {
                        return RESULT_OBST_RIGHT;
                    } else {
                        return RESULT_OBST_LEFT;
                    }
                } else {
                    if(yValid == -1) {
                        return RESULT_TURN_RIGHT;
                    } else if(yValid == -2) {
                        return RESULT_TURN_LEFT;
                    }
                }
            } else {
                if(yValid == -1) {//left is invalid
                    LOGI("line0-left is invalid, xValid:%d, centerRx=%.2f", xValid, centerRx);
                    if(angleR > VALID_ANGLE && xValid != -2) {
                        return RESULT_GO_STRAIGHT;
                    }
                    return RESULT_TURN_LEFT;
                } else if(yValid == -2) {//right is invalid
                    LOGI("line0-right is invalid, xValid:%d, centerLx=%.2f", xValid, centerLx);
                    if(-angleL > VALID_ANGLE && xValid != -1) {
                        return RESULT_GO_STRAIGHT;
                    }
                    return RESULT_TURN_RIGHT;
                }
                if(xValid == 0) {
                    return RESULT_GO_STRAIGHT;
                } else if(xValid == -1) {
                    return RESULT_OBST_RIGHT;
                } else {
                    return RESULT_OBST_LEFT;
                }
            }
        }
    } else if(presentL) {
        int xValid = validX(centerLx, mWidth);
        int yValid = validY(centerLy, mHeight);
        if(xValid != 0 && yValid != 0) {
            return RESULT_NONE;
        }
        if(xValid == -1 || -angleL > VALID_ANGLE) {
            return RESULT_TURN_RIGHT;
        } else {
            return RESULT_GO_STRAIGHT;
        }
    } else if(presentR) {
        int xValid = validX(0, centerRx);
        int yValid = validY(mHeight, centerRy);
        if(xValid != 0 && yValid != 0) {
            return RESULT_NONE;
        }
        if(xValid == -2 || -angleL > VALID_ANGLE) {
            return RESULT_TURN_LEFT;
        } else {
            return RESULT_GO_STRAIGHT;
        }
    }

    return ret;
}

int RoadEdge::validX(float x1, float x2) {
    float validLw = mWidth * VALID_X;
    float validRw = mWidth * (1 - VALID_X);
    if(x1 < validLw  && x2 > validRw) {
        return 0;
    } else if(x1 >= validLw && x2 <= validRw) {
        return -3;
    }
    if(x1 >= validLw) {
        return -1;
    }
    return -2;
}

int RoadEdge::validY(float y1, float y2) {
    float validH = (float)mHeight * 2 / 3;
    if(y1 > validH && y2 > validH) {
        return 0;
    } else if(y1 <= validH && y2 <= validH) {
        return -3;
    }
    if(y1 <= validH) {
        return -1;
    }
    return -2;
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

int RoadEdge::houghLines(const Mat &img, Mat &lineImage, int threshold, double minLineLen, double maxLineGap, bool retry) {
    vector<Vec4f> lines;
    HoughLinesP(img, lines, 1, CV_PI / 180, threshold, minLineLen, maxLineGap);
    if(retry && lines.size() < 100) {
        return -2;
    }
    //Mat lineImage(img.size(), CV_8UC4, Scalar(0, 0, 0, 0));
    return findLines(lineImage, lines);
}

int RoadEdge::processImage(Mat &img) {
    Mat blur, gray;
    GaussianBlur(img, blur, Size(kSize, kSize), 0);
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(15, 20));
    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(15, 20));
    dilate(blur, blur, dilateElement);
    erode(blur, blur, erodeElement);
    Canny(blur, gray, 100, mHoughVote);
    //Canny(blur, gray, 100, 150);

    Point points[1][6];
    points[0][0] = Point(0, mHeight);
    points[0][1] = Point(0, mHeight/2);
    points[0][2] = Point(mWidth/3, mHeight/3);
    points[0][3] = Point(2*mWidth/3, mHeight/3);
    points[0][4] = Point(mWidth, mHeight/2);
    points[0][5] = Point(mWidth, mHeight);
    Point *pts[1] = { points[0] };
    int npt[] = { 6 };
    Mat lineImage(img.size(), CV_8UC4, Scalar(0, 0, 0, 0));
    int ret = -1;
    regionOfInterest(gray, (const Point **)pts, npt);
    ret = houghLines(gray, lineImage, 35, 15, 100);
    if(ret == -2) {
        Canny(blur, gray, 50, 50);
        regionOfInterest(gray, (const Point **)pts, npt);
        ret = houghLines(gray, lineImage, 35, 15, 100, false);
    }
    addWeighted(lineImage, 0.8, img, 1.0, 0, img);
    return ret;
}