//
// Created by wenpd on 2020/2/25.
//

#include <opencv2/opencv.hpp>
#include "log.h"

using namespace cv;

#ifndef OPENCV_APPLICATION_IMAGEPROCESS_H
#define OPENCV_APPLICATION_IMAGEPROCESS_H

class ImageProcess{
private:
    Mat mImg;

private:
    // delta^2 = delta0^2 + [sumRight(f^2) - sumLeft(f^2)]/N + M0^2 - M^2
    void getMeanFromPreviousCol(const Mat &img, Mat &mean, Mat &sigma, const Mat &sumM, const Mat &sumS, int m, int n, int pI, int pJ) {
        int total = (2*m + 1) * (2*n + 1);
        /*float M = 0;
        for(int i = pI - m; i <= pI + m; ++i) {
            M += (img.at<uchar>(i, pJ + n) - img.at<uchar>(i, pJ -n -1));
        }*/
        float M = sumM.at<float>(pI, pJ + n) - sumM.at<float>(pI, pJ -n -1);
        M /= total;
        mean.at<float>(pI, pJ) = M + mean.at<float>(pI, pJ -1);

        /*float S = 0;
        for(int i = pI - m; i <= pI + m; ++i) {
            S += (pow(img.at<uchar>(i, pJ + n), 2) - pow(img.at<uchar>(i, pJ -n -1), 2));
        }*/
        float S = sumS.at<float>(pI, pJ + n) - sumS.at<float>(pI, pJ -n -1);
        S /= total;
        S += pow(mean.at<float>(pI, pJ -1), 2) - pow(mean.at<float>(pI, pJ), 2);
        sigma.at<float>(pI, pJ) = S + sigma.at<float>(pI, pJ -1);
    }

    void getMeanFromPreviousRow(const Mat &img, Mat &mean, Mat &sigma, const vector<float> &sumM, const vector<float> &sumS,
            int m, int n, int pI, int pJ) {
        if(pJ != n) {
            LOGE("line0-something is wrong!with pos=(%d, %d)", pI, pJ);
            return;
        }
        int total = (2*m + 1) * (2*n + 1);
        /*float M = 0;
        for(int j = pJ - n; j <= pJ + n; ++j) {
            M += (img.at<uchar>(pI + n, j) - img.at<uchar>(pI - n -1, j));
        }*/
        float M = sumM[pI + n] - sumM[pI - n -1];
        M /= total;
        mean.at<float>(pI, pJ) = M + mean.at<float>(pI - 1, pJ);

        /*float S = 0;
        for(int j = pJ - n; j <= pJ + n; ++j) {
            S += (pow(img.at<uchar>(pI + n, j), 2) - pow(img.at<uchar>(pI - n -1, j), 2));
        }
        S /= total;*/
        float S = (sumS[pI + n] - sumS[pI - n -1])/total;
        S += pow(mean.at<float>(pI - 1, pJ), 2) - pow(mean.at<float>(pI, pJ), 2);
        sigma.at<float>(pI, pJ) = S + sigma.at<float>(pI - 1, pJ);
    }

    //Count col sum
    void preCountColSum(const Mat &img, Mat &sumM, Mat &sumS, int m) {
        int colSize = (m << 1) + 1;
        for(int j=0; j<img.cols; ++j) {
            float tM = 0;
            float tS = 0;
            for(int i=0; i<colSize; ++i) {
                tM += img.at<uchar>(i, j);
                tS += (float)pow(img.at<uchar>(i, j), 2);
            }
            sumM.at<float>(m, j) = tM;
            sumS.at<float>(m, j) = tS;
            for(int i=m+1; i<img.rows - m; ++i) {
                sumM.at<float>(i, j) = sumM.at<float>(i-1, j) + img.at<uchar>(i + m, j) - img.at<uchar>(i - m - 1, j);
                sumS.at<float>(i, j) = sumS.at<float>(i-1, j) + (float)pow(img.at<uchar>(i + m, j), 2) - (float)pow(img.at<uchar>(i - m - 1, j), 2);
            }
        }
    }

    void preCountRowSum(const Mat &img, vector<float> &sumM, vector<float> &sumS, int n) {
        int rowSize = (n << 1) + 1;
        for(int i=0; i<img.rows; ++i) {
            sumM[i] = 0;
            sumS[i] = 0;
            for(int j=0; j<rowSize; ++j) {
                sumM[i] += img.at<uchar>(i, j);
                sumS[i] += (float)pow(img.at<uchar>(i, j), 2);
            }
        }
    }

    void getMean(const Mat &img, Mat &mean, Mat &sigma, const Mat &sumM, const Mat &sumS, vector<float> &rowSumM, vector<float> &rowSumS,
            int m, int n, int pI, int pJ) {
        if(pI - m < 0 || pI + m >= img.rows || pJ - n < 0 || pJ + n >= img.cols) {
            mean.at<float>(pI, pJ) = -1;
            sigma.at<float>(pI, pJ) = 0;
            return ;
        }
        if(pJ == n && pI > m) {
            return getMeanFromPreviousRow(img, mean, sigma, rowSumM, rowSumS, m, n, pI, pJ);
        } else if(pJ > n) {
            return getMeanFromPreviousCol(img, mean, sigma, sumM, sumS, m, n, pI, pJ);
        }
        float M = 0;
        int total = (2*m + 1) * (2*n + 1);
        for(int i=pI -m; i<= pI + m; ++i) {
            for(int j=pJ - n; j <= pJ + n; ++j) {
                M += img.at<uchar>(i, j);
            }
        }
        M /= total;
        mean.at<float>(pI, pJ) = M;

        float S = 0;
        float diff;
        for(int i=pI -m; i<= pI + m; ++i) {
            for(int j=pJ - n; j <= pJ + n; ++j) {
                diff =  img.at<uchar>(i, j) - M;
                S += (diff * diff);
            }
        }
        S /= total;
        sigma.at<float>(pI, pJ) = S;
    }

    void getMean(const Mat &img, Mat &mean, Mat &sigma, int m, int n){
        Mat colSumM = Mat::zeros(img.rows, img.cols, CV_32FC1);
        Mat colSumS = Mat::zeros(img.rows, img.cols, CV_32FC1);
        preCountColSum(img, colSumM, colSumS, m);

        vector<float> rowSumM;
        vector<float> rowSumS;
        rowSumM.reserve((unsigned)img.rows);
        rowSumS.reserve((unsigned)img.rows);
        preCountRowSum(img, rowSumM, rowSumS, n);

        for(int i=0;i<mImg.rows; ++i) {
            //LOGI("line0-row:%d", i);
            for (int j = 0; j < mImg.cols; ++j) {
                getMean(img, mean, sigma, colSumM, colSumS, rowSumM, rowSumS, m, n, i, j);
            }
        }
        float value;
        for(int i=0;i<mImg.rows; ++i) {
            for (int j = 0; j < mImg.cols; ++j) {
                value = sigma.at<float>(i, j);
                sigma.at<float>(i, j) = sqrt(value);
            }
        }
    }

    float getMean(const Mat &mat) {
        Scalar mean;
        Scalar dev;
        meanStdDev(mat, mean, dev);

        return (float)mean.val[0];
    }


public:
    ImageProcess(const Mat &img) {
        mImg = img;
    }

    void adaptiveContrastEnhancement(int m, int n, int alpha, float maxG = 5) {
        Mat rgbImg;
        cvtColor(mImg, rgbImg, COLOR_RGBA2RGB);
        Mat ycc;
        cvtColor(rgbImg, ycc, COLOR_RGB2YCrCb);
        vector<Mat> channels(3);        //分离通道；
        split(ycc, channels);

        Mat meanMat(mImg.rows, mImg.cols, CV_32FC1);
        Mat sigmaMat(mImg.rows, mImg.cols, CV_32FC1);

       getMean(channels[0], meanMat, sigmaMat, m, n);
        Mat tmp = channels[0].clone();
        float M = getMean(tmp);
        Mat enhance(mImg.rows, mImg.cols, CV_8UC1);

        for(int i=0;i<mImg.rows; ++i) {
            for(int j=0; j<mImg.cols; ++j) {
                if(sigmaMat.at<float>(i, j) < (float)0.01) {
                    enhance.at<uchar>(i, j) = tmp.at<uchar>(i, j);
                } else {
                    float G = alpha * M / sigmaMat.at<float>(i, j);
                    if(G < 1) {
                        G = 1;
                    } else if(G > maxG) {
                        G = maxG;
                    }
                    float result = meanMat.at<float>(i, j) + G * (tmp.at<uchar>(i, j) - meanMat.at<float>(i, j));
                    if(result > 255) {
                        result = 255;
                    } else if(result < 0) {
                        result = 0;
                    }
                    enhance.at<uchar>(i, j) = (uchar)result;
                }
            }
        }

        channels[0] = enhance;   //合并通道，转换颜色空间回到RGB
        merge(channels, ycc);
        cvtColor(ycc, rgbImg, COLOR_YCrCb2RGB);
        cvtColor(rgbImg, mImg, COLOR_RGB2RGBA);
    }

    static float getMean2(const Mat &mat) {
        Scalar mean;
        Scalar dev;
        meanStdDev(mat, mean, dev);

        return (float)mean.val[0];
    }

    static void SalientRegionDetectionBasedonAC(Mat &src, Mat &sal, int MinR2, int MaxR2,int Scale = 3){
        Mat Lab;
        LOGI("line0-cvtColor");
        cvtColor(src, Lab, COLOR_RGB2Lab);

        int row=src.rows,col=src.cols;
        int **Sal_org = new int*[row];
        for(int i=0; i<row; ++i) {
            Sal_org[i] = new int[col]{0};
        }
        //memset(Sal_org,0,sizeof(int)*row*col);

        Point3_<uchar>* p;
        Point3_<uchar>* p1;

        int val;
        Mat filter;

        int max_v=0;
        int min_v=1<<28;
        for (int k=0;k<Scale;k++){
            int len=(MaxR2 - MinR2) * k / (Scale - 1) + MinR2;
            blur(Lab, filter, Size(len,len ));
            LOGI("line0-k=%d", k);
            for (int i=0;i<row;i++){
                for (int j=0;j<col;j++){
                    p=Lab.ptr<Point3_<uchar> > (i,j);
                    p1=filter.ptr<Point3_<uchar> > (i,j);

                    val=(int)sqrt( (p->x - p1->x)*(p->x - p1->x)+ (p->y - p1->y)*(p->y-p1->y) + (p->z - p1->z)*(p->z - p1->z) );

                    Sal_org[i][j] += val;
                    if(k==Scale-1){
                        max_v=max(max_v, Sal_org[i][j]);
                        min_v=min(min_v, Sal_org[i][j]);
                    }
                }
            }
        }

        //cout<<max_v<<" "<<min_v<<endl;
        LOGI("line0-max-min=%d, %d", max_v, min_v);
        int value;
        for (int i = 0; i < row; i++){
            for (int j = 0; j < col; j++){
                value = (Sal_org[i][j] - min_v)*255/(max_v - min_v);
                sal.at<uchar>(i, j) = (uchar)value;        //    计算全图每个像素的显著性
                //Sal.at<uchar>(Y,X) = (Dist[gray[Y][X]])*255/(max_gray);        //    计算全图每个像素的显著性
            }
        }

        for(int i=0; i<row; ++i) {
            delete Sal_org[i];
            Sal_org[i] = NULL;
        }
        delete Sal_org;
        Sal_org = NULL;
        //imshow("sal",sal);
        //waitKey(0);
    }

    static void washedSegment(const Mat &src, Mat &dst) {
        Mat gray;
        Mat blur;
        cvtColor(src, gray, COLOR_RGB2GRAY);
        GaussianBlur(gray, blur, Size(5, 5), 0);
        Canny(blur, gray, 80, 150);
        //Canny(blur, gray, 100, 150);

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(gray, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
        Mat marker = Mat::zeros(gray.size(), CV_32S);
        for (size_t i = 0; i < contours.size(); i++){
            drawContours(marker, contours, static_cast<int>(i) ,Scalar(static_cast<int>(i)+1), -1);
        }
        watershed(src, marker);
        vector<Vec3b> colors;
        for (size_t i = 0; i < contours.size(); i++) {
            int r = theRNG().uniform(0, 255);
            int g = theRNG().uniform(0, 255);
            int b = theRNG().uniform(0, 255);
            colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
        }
        //Mat dst = Mat::zeros(marker.size(), CV_8UC3);
        for (int row = 0; row < marker.rows; row++) {
            for (int col = 0; col < marker.cols; col++) {
                int index = marker.at<int>(row, col);
                if (index > 0 && index <= static_cast<int>(contours.size())) {
                    dst.at<Vec3b>(row, col) = colors[index - 1];
                } else {
                    dst.at<Vec3b>(row, col) = Vec3b(0, 0, 0);
                }
            }
        }
        addWeighted(dst, 0.4, src, 0.6, 0, dst);
    }

    static void meanShift(const Mat &src, Mat &dst) {
        Mat dst_img;                    //分割后图像
        int spatialRad = 20;        //空间窗口大小
        int colorRad = 20;          //色彩窗口大小
        int maxPyrLevel = 1;        //金字塔层数
        pyrMeanShiftFiltering(src, dst, spatialRad, colorRad, maxPyrLevel);
        RNG rng = theRNG();
        Mat mask(dst.rows + 2, dst.cols + 2, CV_8UC1, Scalar::all(0));  //掩模
        for (int y = 0; y < dst.rows; y++) {
            for (int x = 0; x < dst.cols; x++) {
                if (mask.at<uchar>(y + 1, x + 1) == 0)  //非0处即为1，表示已经经过填充，不再处理
                {
                    Scalar newVal(rng(256), rng(256), rng(256));
                    floodFill(dst, mask, Point(x, y), newVal, 0, Scalar::all(5),
                              Scalar::all(5)); //执行漫水填充
                }
            }
        }
    }

    static void kMean(const Mat &src, Mat &dst, int clusterCount = 4) {
        Mat lab;
        cvtColor(src, lab, COLOR_RGB2Lab);
        Mat samples(src.cols * src.rows, 1, CV_32FC3);
        Mat labels(src.cols * src.rows, 1, CV_32SC1);
        const uchar* p;
        int k=0;
        for(int i=0; i < src.rows; i++)
        {
            p = lab.ptr<uchar>(i);
            for(int j=0; j< src.cols; j++){
                samples.at<Vec3f>(k,0)[0] = float(p[j*3]);
                samples.at<Vec3f>(k,0)[1] = float(p[j*3+1]);
                samples.at<Vec3f>(k,0)[2] = float(p[j*3+2]);
                k++;
            }
        }

        Mat centers(clusterCount, 1, samples.type());
        kmeans(samples, clusterCount, labels,
               TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 10, 1.0),
               3, KMEANS_PP_CENTERS, centers);

        float step = 255/(clusterCount + 1);
        k=0;
        uchar *dstP;
        for(int i=0; i < dst.rows; i++){
            dstP = dst.ptr<uchar>(i);
            for(int j=0; j< dst.cols; j++)
            {
                int tt = labels.at<int>(k, 0);
                k++;
                dstP[j] = 255 - (tt + 1)*step;
            }
        }
    }

    static void salientSegment(const Mat &src, const Mat &sal, Mat &dst, int T = 31) {
        Mat lab;
        cvtColor(src, lab, COLOR_RGB2Lab);
        Mat samples(src.cols * src.rows, 1, CV_32FC3);
        Mat labels(src.cols * src.rows, 1, CV_32SC1);
        const uchar* p;
        int k=0;
        for(int i=0; i < src.rows; i++){
            p = lab.ptr<uchar>(i);
            for(int j=0; j< src.cols; j++){
                samples.at<Vec3f>(k,0)[0] = float(p[j*3]);
                samples.at<Vec3f>(k,0)[1] = float(p[j*3+1]);
                samples.at<Vec3f>(k,0)[2] = float(p[j*3+2]);
                k++;
            }
        }

        int clusterCount = 4;
        Mat centers(clusterCount, 1, samples.type());
        kmeans(samples, clusterCount, labels,
               TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 10, 1.0),
               3, KMEANS_PP_CENTERS, centers);
        int *count = new int[clusterCount]{0};
        float *avSal = new float[clusterCount]{0};
        k=0;
        for(int i=0; i < dst.rows; i++){
            for(int j=0; j< dst.cols; j++)
            {
                int tt = labels.at<int>(k, 0);
                avSal[tt] += sal.at<uchar>(i, j);
                ++count[tt];
                k++;
            }
        }
        float av_sal = getMean2(sal);
        float avMax = 0;
        for(int i=0; i<clusterCount; ++i) {
            avSal[i] /= count[i];
            avMax = max(avMax, avSal[i]);
            LOGI("line0-sal:%.2f", avSal[i]);
        }
        LOGI("line0-avsal:%.2f", av_sal);
        k=0;
        float threshHold = av_sal;
        if(av_sal > avMax) {
            threshHold = avMax / 1.2;
        }
        for(int i=0; i < src.rows; i++){
            for(int j=0; j< src.cols; j++){
                int tt = labels.at<int>(k, 0);
                if(avSal[tt] > threshHold) {
                    dst.at<Vec3b>(i, j) = src.at<Vec3b>(i, j);
                } else {
                    dst.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
                }
                k++;
            }
        }
        delete count;
        delete avSal;
    }
};
#endif //OPENCV_APPLICATION_IMAGEPROCESS_H
