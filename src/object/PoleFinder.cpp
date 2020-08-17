#include <iostream>
#include <vector>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "../include/PoleFinder.h"

using namespace std;
using namespace cv;
using namespace Robot2019;

// #define DEBUG_GOALFINDER

#ifdef DEBUG_POLEFINDER
#define printExpr(EXPR) cout << #EXPR << " : " << EXPR << endl;
#define printExprC(S, EXPR) cout << S << " : " << EXPR << endl;
#define printStr(S) cout << S << endl;
#else
#define printExpr(EXPR)
#define printStr(S)
#endif

// GoalPerceptor *GoalPerceptor::m_UniqueInstance = new GoalPerceptor();

////////////////////////////////////
//Bagian GoalPerceptor
//Constructor
PoleFinder::PoleFinder() {
    namedWindow("Deteksi Gawang", WINDOW_NORMAL);
    // namedWindow("Pre-Processing", WINDOW_NORMAL);
    // namedWindow("Properties Gawang", WINDOW_NORMAL);
    namedWindow("Pre-Proc Final", WINDOW_NORMAL);
    // namedWindow("Adaptive Thresold", WINDOW_NORMAL);
}

PoleDetector::~PoleDetector() {}

void PoleDetector::init()
{

    initWindow();
    // initTrackbar();
}

void PoleFinder::paramCallback(daho_vision::PoleFinderConfig &config, uint32_t level) {
    //Color filter
    minB = config.minB;
    maxB = config.maxB;
    minG = config.minG;
    maxG = config.maxG;
    minR = config.minR;
    maxR = config.maxR;

    // Pre-Processing
    yCrop = config.yCrop;
    sigma = config.sigma;
    tetha = config.tetha;
    lambd = config.lambd;
    gamma = config.gamma;

    // Seleksi
    minFieldRatio = config.minFieldRatio;
    maxFieldRatio = config.maxFieldRatio;

    // Statistik
    minVarian = config.minVarian;
    maxVarian = config.maxVarian;
}

void PoleFinder::initWindow()
{
    namedWindow("Deteksi Gawang", WINDOW_NORMAL);
    namedWindow("Pre-Processing", WINDOW_NORMAL);
    namedWindow("Properties Gawang", WINDOW_NORMAL);
    namedWindow("Pre-Proc Final", WINDOW_NORMAL);
    namedWindow("Adaptive Thresold", WINDOW_NORMAL);
}

// Helper function min & max
template <class T>
T min4(T a, T b, T c, T d)
{
    return MIN(MIN(a, b), MIN(c, d));
}

template <class T>
T max4(T a, T b, T c, T d)
{
    return MAX(MAX(a, b), MAX(c, d));
}

//Cek apakah tiang valid dengan mengecek orientasinya
//TODO : Atur batas orientasi sesuai pose kamera
bool tiangValid(RotatedRect boundRect)
{
    Point2f p[4];
    boundRect.points(p);
    float minX = min4(p[0].x, p[1].x, p[2].x, p[3].x);
    float minY = min4(p[0].y, p[1].y, p[2].y, p[3].y);
    float maxX = max4(p[0].x, p[1].x, p[2].x, p[3].x);
    float maxY = max4(p[0].y, p[1].y, p[2].y, p[3].y);
    return (abs(boundRect.angle) - 90 < 10) && ((maxY - minY) - (maxX - minX) > 30); //&& (maxX-minX) < 60;
}

//Cek apakah bagian atas tiang bukan lapangan dan bagian bawah ada di sekitar lapangan
bool PoleFinder::cekLapangan(RotatedRect boundRect, Mat field)
{
    Point2f p[4];
    boundRect.points(p);
    float maxY = MIN(MAX(MAX(p[0].y, p[1].y), MAX(p[2].y, p[3].y)), 480);
    float minY = MAX(MIN(MIN(p[0].y, p[1].y), MIN(p[2].y, p[3].y)), 0);
    float maxX = MIN(MAX(MAX(p[0].x, p[1].x), MAX(p[2].x, p[3].x)), 640);
    float minX = MAX(MIN(MIN(p[0].y, p[1].y), MIN(p[2].y, p[3].y)), 0);

    Rect boundingBox(minX, minY, maxX-minX, maxY-minY);
    // rectangle(image, boundingBox, Scalar(255, 200, 200), 2);
    // rectangle(image, boundingBoxAtas, Scalar(0, 0, 0), 5);
    // imshow("Debug gawang", image);
    Mat maskField = field(boundingBox);
    int countNZero = countNonZero(maskField);
    double rasioLapangan = (double)countNZero / (lebarBoundBox * tinggiBoundBox) * 100.0;
    return rasioLapangan >= minFieldRatio && rasioLapangan <= maxFieldRatio;
}

//Deteksi tiang gawang, m = img dari webcam, field = mask lapangan
void PoleFinder::process(Mat m, Mat field)
{
    Mat fieldCopy;
    cvtColor(field, fieldCopy, COLOR_BGR2GRAY);
    //Image Processing
    Mat pp, hasil, gray; //pp = Mat temp untuk pre-processing
    //--------------------------------Color thresholding--------------------------------
    m.copyTo(hasil);
    GaussianBlur(m, pp, Size(3, 3), 0);
    inRange(pp, Scalar(minB, minG, minR), Scalar(maxB, maxG, maxR), pp);
    m.copyTo(pp, pp);
    // imshow("Pre-Processing", pp);
    //--------------------------------Adaptive thresolding--------------------------------
    cvtColor(pp, pp, COLOR_BGR2GRAY);
    GaussianBlur(pp, pp, Size(3, 3), 0);
    GaussianBlur(pp, pp, Size(9, 9), 0);
    adaptiveThreshold(pp, pp, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 5, 0);
    GaussianBlur(pp, pp, Size(5, 5), 0);
    //--------------------------------Crop setengah atas--------------------------------
    Mat roi = pp(Rect(0, 0, 640, yCrop));
    roi.setTo(Scalar(0, 0, 0));
    //--------------------------------Gabor filter--------------------------------
    Mat hasilGabor1, hasilGabor2;
    Size2i sizeKernel(3, 3);
    Mat gaborKernel1 = getGaborKernel(sizeKernel, sigma, (double)tetha / 360 * CV_2PI, lambd, gamma);
    filter2D(pp, hasilGabor1, pp.depth(), gaborKernel1);
    Mat gaborKernel2 = getGaborKernel(sizeKernel, sigma, (double)(tetha + 180) / 360 * CV_2PI, lambd, gamma);
    filter2D(pp, hasilGabor2, pp.depth(), gaborKernel2);
    bitwise_or(hasilGabor1, hasilGabor2, pp);
    bitwise_not(fieldCopy, fieldCopy);
    bitwise_and(pp, fieldCopy, pp);
    bitwise_not(fieldCopy, fieldCopy);
    Mat ppDebug;
    m.copyTo(ppDebug, pp);
    // cout << "Ini Gambarnya" << endl;
    imshow("Pre-Proc Final", ppDebug);
    // resize(gab, gab, Size(300, 300));
    // imshow("Gabor filter", gab);
    //--------------------------------Contour detection--------------------------------
    vector<vector<Point>> contours;
    findContours(pp, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
    //--------------------------------Seleksi contours yang sesuai & tampilkan--------------------------------
    xGoalPost.clear();
    cArea.clear();
    totalAreaContour = 0;
    for (vector<Point> c : contours)
    {
        RotatedRect boundRect = minAreaRect(c);
        if (tiangValid(boundRect))
        {
            //Cek lapangan
            if (cekLapangan(boundRect, fieldCopy))
            {
                xGoalPost.push_back(boundRect.center.x);
                double area = contourArea(c);
                cArea.push_back(area);
                totalAreaContour += area;
                Point2f rect_points[4];
                boundRect.points(rect_points);
                for (int j = 0; j < 4; j++)
                {
                    line(hasil, rect_points[j], rect_points[(j + 1) % 4], Scalar(255, 0, 0), 4);
                }
                putText(hasil, "Tiang", boundRect.center, FONT_HERSHEY_PLAIN, 2, Scalar(0, 255, 0), 2);
            }
        }
    }
    imshow("Deteksi Gawang", hasil);
    waitKey(1);
}

//Probability Mass Function setiap kandidat gawang
//Digunakan di Behavior bersamaan dengan minVarian & maxVarian
double PoleFinder::getContourPMF(int idx)
{
    if (idx >= 0 && idx < cArea.size())
    {
        return cArea[idx] / totalAreaContour;
    }
    else
    {
        return 0;
    }
}

//Helper function Linear Interpolation
double normValue(double from, double fromMin, double fromMax, double toMin, double toMax)
{
    return fromMin + from * (toMax - toMin) / (fromMax - fromMin);
}