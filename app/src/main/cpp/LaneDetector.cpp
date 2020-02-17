//
// Created by wenpd on 2020/2/12.
//

#include "LaneDetector.h"
#include "log.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <iostream>
#include <vector>
#include "linefinder.h"
#include "lanemetrics.h"

void LaneDetector::processImage(Mat &imgIn) {
    //////reducing the image size////
    Mat image;
    resize(imgIn, image, Size(),1,1, cv::INTER_LINEAR); //to reduce it to quarter of size

    int houghVote = mHoughValue;
    if (image.empty()){
        LOGE("LANEDETECTOR++", "%s", "Cannot access the camera");
       return;
    }

    Mat gray;
    cvtColor(image,gray,COLOR_RGBA2GRAY); //convert image to grayscale
    //vector<string> codes;
    //Mat corners;
    //findDataMatrix(gray, codes, corners);
    //drawDataMatrixCodes(image, codes, corners);

    /*ROI returns a matrix that is pointing to the ROI of the original image, located at the place specified by the rectangle.
    so imageROI is really the Region of Interest (or subimage/submatrix) of the original image "image".
    If you modify imageROI it will consequently modify the original, larger matrix.
    Rect region_of_interest = Rect(x, y, w, h); current parameters try to take the lower half of the image on vertical frame*/

    Rect roi(0,image.cols/3,image.cols-1,image.rows - image.cols/3);// set the ROI (region of interest) for the image
    //Rect roi(0,image.cols/2,image.cols-1,image.rows - image.cols/2);// set the ROI (region of interest) for the image

    Mat imgROI = image(roi);
    /* Canny Edge Detector algorithm : canny (source image, edge output image, first threshold for the hysteresis procedure,
		  , second threshold for the hysteresis procedure, apersturesize i.e set to be 3 by default , L2gradient )
		  threshold2 is recommended to be 3 times to threshold1 */

    Mat contours;
    Canny(imgROI,contours, 80,250,3); //50, 150,3); //in original code => 50,250);
    LOGI("line0 after canny");

    /* Thresholding is to differentiate the pixels we are interested in from the rest */
    Mat contoursInv;
    threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);

    /*
		Hough tranform for line detection with feedback
		Increase by 25 for the next frame if we found some lines.
		This is so we don't miss other lines that may crop up in the next frame
		but at the same time we don't want to start the feed back loop from scratch.
	*/


    vector<Vec2f> lines;
    if (houghVote < 1 or lines.size() > 2){ // we lost all lines. reset
        houghVote = mHoughValue; //previously it was set to 200 always
    }
    else{ houghVote += 25;}

    //ensuring lines size is 5
    while(lines.size() < 5 && houghVote > 0){
        HoughLines(contours,lines,1,PI/180, houghVote);
        houghVote -= 5;
    }
    LOGI("line0 start HoughLines:%d", lines.size());

    //cout << houghVote << "\n";
    Mat result(imgROI.size(),CV_8U,Scalar(255));
    imgROI.copyTo(result);

    // Draw the lines
    vector<Vec2f>::const_iterator it= lines.begin();
    Mat hough(imgROI.size(),CV_8U,Scalar(0));
    LOGI("line0 start line");
    while (it != lines.end()){

        //__android_log_print(ANDROID_LOG_INFO, "LANEDETECTOR++", "%s", "while of lines");
        float rho= (*it)[0];   // first element is distance rho
        float theta= (*it)[1]; // second element is angle theta

        // filter to remove vertical and horizontal lines
        //if(theta is between 5 degrees and 84 degrees) or if(theta is between 95 degrees and 180 degrees )
        if((theta > 0.09 && theta < 1.48) || (theta < 3.14 && theta > 1.66))
        {
            // point of intersection of the line with first row
            Point pt1(rho/cos(theta),0);//y = 0
            // point of intersection of the line with last row
            Point pt2((rho-result.rows*sin(theta))/cos(theta),result.rows);//y = row count


            //This draws lines but without ends and in shape of X
            line( hough, pt1, pt2, Scalar(255, 0, 0, 255), 10); //this is working and shows red lines of thickness 10
        }

        // cout << "line: (" << rho << "," << theta << ")\n";
        LOGI("line: %.2, %.2f", rho, theta);
        ++it;
    }

    // Create LineFinder instance
    LineFinder ld;

    // Set probabilistic Hough parameters
    ld.setLineLengthAndGap(60,10);
    ld.setMinVote(4);

    // Detect lines
    vector<Vec4i> li= ld.findLines(contours); //applying probablity hough transform
    LOGI("line0 start findLines:%d", li.size());

    Mat houghP(imgROI.size(),CV_8U,Scalar(0));
    ld.setShift(0);
    ld.drawDetectedLines(houghP);

    // bitwise AND of the two hough images.
    //1) Normal Hough -> without end points 2) Probabilistic Hough -> with end points

    bitwise_and(houghP,hough,houghP);
    Mat houghPinv(imgROI.size(),CV_8U,Scalar(0));
    Mat dst(imgROI.size(),CV_8U,Scalar(0));
    threshold(houghP,houghPinv,150,255,THRESH_BINARY_INV); // threshold and invert to black lines

    Canny(houghPinv,contours, 100,350);//100,150); //100,350);
    li= ld.findLines(contours);
    LOGI("line0 start findLines2:%d", li.size());
    if(true){
        imwrite("contours.bmp", contoursInv);
    }

    // Set probabilistic Hough parameters
    //These parameters set min and max values. If you set minvote as 100 then even if you set 60 from slider, it won't
    //show any lines

    ld.setLineLengthAndGap(5,2); //5,2 original
    ld.setMinVote(1); //1 original

    ld.setShift(image.cols/3);
    ld.drawDetectedLines(imgIn);

    //to show the number of line segments found in a frame
    //putText(image, stream.str(), Point(10,image.rows-10), 4, 1, Scalar(0,255,255),0);
    lines.clear();

    //Hough Processing ends here
    //resize(image, image, Size(),4,4, cv::INTER_LINEAR);
    //mRgbMat = image.clone();
}
