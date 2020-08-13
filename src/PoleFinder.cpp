/*
/*  Author :
 *  Aditya Putra Santosa
 *  13517013
 */
#include <iostream>
#include <vector>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "/include/PoleFinder.h"

using namespace std;
using namespace cv;
using namespace Robot2019;

// #define DEBUG_GOALFINDER

#ifdef DEBUG_GOALFINDER
#define printExpr(EXPR) cout << #EXPR << " : " << EXPR << endl;
#define printExprC(S, EXPR) cout << S << " : " << EXPR << endl;
#define printStr(S) cout << S << endl;
#else
#define printExpr(EXPR)
#define printStr(S)
#endif

GoalPerceptor *GoalPerceptor::m_UniqueInstance = new GoalPerceptor();

////////////////////////////////////
//Bagian GoalPerceptor
//Constructor
GoalPerceptor::GoalPerceptor() {
    namedWindow("Deteksi Gawang", WINDOW_NORMAL);
    namedWindow("Pre-Processing", WINDOW_NORMAL);
    namedWindow("Properties Gawang", WINDOW_NORMAL);
    namedWindow("Pre-Proc Final", WINDOW_NORMAL);
    namedWindow("Adaptive Thresold", WINDOW_NORMAL);
}

GoalPerceptor::~GoalPerceptor() {}

void GoalPerceptor::init()
{

    initWindow();
    // initTrackbar();
}

void GoalPerceptor::paramCallback(daho_vision::GoalPostConfig &config, uint32_t level) {
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
    minFieldAtasR = config.minFieldAtasR;
    maxFieldAtasR = config.maxFieldAtasR;

    // Statistik
    minVarian = config.minVarian;
    maxVarian = config.maxVarian;
}

void GoalPerceptor::initWindow()
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
bool GoalPerceptor::cekLapangan(RotatedRect boundRect, Mat field)
{
    Point2f p[4];
    boundRect.points(p);
    float maxY = MIN(MAX(MAX(p[0].y, p[1].y), MAX(p[2].y, p[3].y)), 480);
    float minY = MAX(MIN(MIN(p[0].y, p[1].y), MIN(p[2].y, p[3].y)), 0);
    Point2f center = Point2f(boundRect.center.x, maxY);
    Point2f centerAtas = Point2f(boundRect.center.x, minY);
    int lebarBoundBox = 50, lBBAtas = 50;
    int tinggiBoundBox = 10, tBBAtas = 10;
    int awalX = center.x - lebarBoundBox / 2;
    int awalY = center.y - tinggiBoundBox / 2;
    int awalXAtas = centerAtas.x - lebarBoundBox / 2;
    int awalYAtas = centerAtas.y + tinggiBoundBox / 2;

    // printStr("Awal")
    // printExpr(awalX)
    // printExpr(awalY)
    // printExpr(lebarBoundBox)
    // printExpr(tinggiBoundBox)
    if (awalX < 0)
    {
        lebarBoundBox += awalX;
        awalX = 0;
    }
    if (awalX + lebarBoundBox > field.cols)
    {
        lebarBoundBox -= (awalX + lebarBoundBox - field.cols);
    }
    if (awalY < 0)
    {
        awalY = 0;
        tinggiBoundBox += awalY;
    }
    if (awalY + tinggiBoundBox > field.rows)
    {
        tinggiBoundBox -= (awalY + tinggiBoundBox - field.rows);
    }
    // printStr("===============")
    // printExpr(awalX)
    // printExpr(awalY)
    // printExpr(lebarBoundBox)
    // printExpr(tinggiBoundBox)
    // printStr("\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\")
    if (awalXAtas < 0)
    {
        awalXAtas = 0;
        lBBAtas += awalXAtas;
    }
    if (awalXAtas + lBBAtas > field.cols)
    {
        lBBAtas -= (awalXAtas + lBBAtas - field.cols);
    }
    if (awalYAtas < 0)
    {
        awalYAtas = 0;
        tBBAtas += awalYAtas;
    }
    if (awalYAtas + tBBAtas > field.rows)
    {
        tBBAtas -= (awalYAtas + tBBAtas - field.rows);
    }

    Rect boundingBox(awalX, awalY, lebarBoundBox, tinggiBoundBox);
    Rect boundingBoxAtas(awalXAtas, awalYAtas, lBBAtas, tBBAtas);
    // rectangle(image, boundingBox, Scalar(255, 200, 200), 2);
    // rectangle(image, boundingBoxAtas, Scalar(0, 0, 0), 5);
    // imshow("Debug gawang", image);
    Mat maskField = field(boundingBox);
    Mat maskFieldAtas = field(boundingBoxAtas);
    int countNZero = countNonZero(maskField);
    int cNZAtas = countNonZero(maskFieldAtas);
    double rasioLapangan = (double)countNZero / (lebarBoundBox * tinggiBoundBox) * 100.0;
    double rasioLapanganAtas = (double)cNZAtas / (lebarBoundBox * tinggiBoundBox) * 100.0;
    return rasioLapangan >= minFieldRatio && rasioLapangan <= maxFieldRatio && rasioLapanganAtas >= minFieldAtasR && rasioLapanganAtas <= maxFieldAtasR;
}

//Deteksi tiang gawang, m = img dari webcam, field = mask lapangan
void GoalPerceptor::process(Mat m, Mat field)
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
            // if (cekLapangan(boundRect, fieldCopy))
            // {
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
                putText(hasil, to_string(boundRect.angle), boundRect.center, FONT_HERSHEY_PLAIN, 2, Scalar(0, 255, 0), 2);
            // }
        }
    }
    imshow("Deteksi Gawang", hasil);
    // waitKey(1);
}

//Probability Mass Function setiap kandidat gawang
//Digunakan di Behavior bersamaan dengan minVarian & maxVarian
double GoalPerceptor::getContourPMF(int idx)
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