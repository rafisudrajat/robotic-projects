// Soccer Vision
// Author: Andreas Ekadinata Widodo <ekadinataa@gmail.com>
//         Ayrton Cyril
//         Aditya Putra Santosa <adityaputra159@gmail.com>
/**
* KRSBI ITB
* Dago Hoogeschool Team
**/

#include "ros/ros.h"
#include "daho_vision/BallFinder2019.h"
#define DEBUGBALL
using namespace std;
using namespace Robot;

/*
// Const like Walking
BallFinder::BallFinder()
{
    min_Y = 0;
    max_Y = 255;
    min_U = 0;
    max_U = 255;
    min_V = 0;
    max_V = 255;

    lowThreshold = 150;
    ratio = 1;
    kernel_size = 3;

    min_dist = 3;
    upper_threshold = 50;
    center_threshold = 45;
    min_r = 10;
    max_r = 400;   
}
//////////////////////
*/

BallFinder::BallFinder()
{
    namedWindow("Thresold", WINDOW_NORMAL);
    namedWindow("Con", WINDOW_NORMAL);
}
/////////////////////////////////////

BallFinder::~BallFinder()
{ /* NONE */
}

void BallFinder::on_trackbar(int value, void *userdata) {}

void BallFinder::morphOps(Mat &thresh)
{
    Mat element = getStructuringElement(0, Size(2 * countErode + 1, 2 * countErode + 1), Point(countErode, countErode));

    morphologyEx(thresh, thresh, MORPH_CLOSE, element);
}

bool BallFinder::isDetectBall(Point2d center)
{
    if (center.x > 0)
    {
        ROS_DEBUG_NAMED("Ball Finder", "Bola di : ( %.2f , %.2f )", center.x, center.y);
        return true;
    }
    else
    {
        ROS_DEBUG_NAMED("Ball Finder", "Bola tidak ada");
        return false;
    }
}

bool compareBola(Point3f p1, Point3f p2)
{
    return p1.z < p2.z;
}

void BallFinder::Process(Mat image, Mat field, bool cannyOn, bool checkEllipse)
{
    //--------------------------------Validity Checking-----------------------------
    bool imageValid = image.rows > 0 && image.cols > 0;
    bool fieldValid = field.rows > 0 && field.cols > 0;
    if(!imageValid || !fieldValid)
    {
        if(!imageValid)
        {
            ROS_ERROR("Image tidak valid");
        }
        if(!fieldValid)
        {
            ROS_ERROR("Field tidak valid");
        }
        return;
    }
    //--------------------------------Pre Processing--------------------------------
    Mat debug_thresold, fieldCopy;
    cvtColor(field, fieldCopy, COLOR_BGR2GRAY);
    GaussianBlur(image, img, Size(3, 3), 2, 2);
    GaussianBlur(img, img, Size(9, 9), 2, 2, 4);
    cvtColor(img, yuv, COLOR_BGR2HSV);
    // cvtColor(img, hsv, CV_BGR2HSV);
    inRange(yuv, Scalar(min_Y, min_U, min_V), Scalar(max_Y, max_U, max_V), th);
    // inRange(hsv, Scalar(fmin_H, fmin_S, fmin_V), Scalar(fmax_H, fmax_S, fmax_V), field);
    morphOps(th);
    GaussianBlur(th, th, Size(9, 9), 2, 2, 4);
    image.copyTo(debug_thresold, th);
    imshow("Thresold", debug_thresold);
    //--------------------------------Ball Detection--------------------------------
    vector<Point3f> kandidatBola; //Vector titik 3 dimensi (x, y, r), r = radius

    //Menggunakkan 2 cara : Contour Detection & Hough Transform
    //Step 1 : Ambil semua contour yang kemungkinan bola
    //Step 2 : Ambil semua objek yang bulat (Hough Transform)
    //Step 3 : Untuk semua kandidat, cek apakah diatas lapangan
    //Step 4 : Sort berdasar berapa persen putihnya & berapa persen lapangan disekitarnya
    //--------------------------------Contour Finding--------------------------------
    vector<vector<Point>> contours;
    findContours(th, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
    image.copyTo(contoursImg);
    for (size_t i = 0; i < contours.size(); i++)
    {
        Point2f center;
        float radius;
        minEnclosingCircle(contours[i], center, radius);
        RotatedRect minRect = minAreaRect(contours[i]);
        RotatedRect minEllips = RotatedRect(minRect);
        //Prelim 1
        //--------------------------------Cek apakah rasio rect/circle > epsBoundRC--------------------------------
        //Jika iya maka bola
        float luasCirlce = radius * radius * CV_PI;
        float luasRect = minRect.size.area();
        bool passed = false;
        if (100 * luasRect / luasCirlce > epsBoundRC)
        {
            // ROS_ERROR_STREAM("Con " << i << "Pass RC" << endl);
            if (checkEllipse)
            {
                if (contours[i].size() >= 5)
                {
                    minEllips = fitEllipse(contours[i]);
#ifdef DEBUGBALL
                    ellipse(contoursImg, minEllips, Scalar(0, 255, 0), 3);
#endif
                }
                //--------------------------------Cek apakah rasio ellipse/circle > epsBoundEC--------------------------------
                Point2f elips_points[4];
                minEllips.points(elips_points);
                float minor = norm(elips_points[0] - elips_points[1]) / 2;
                float major = norm(elips_points[1] - elips_points[2]) / 2;
                float luasEllips = minor * major * CV_PI;
                if (100 * luasEllips / luasCirlce > epsBoundEC)
                {
                    // ROS_ERROR_STREAM("Con " << i << "Pass EC" << endl);
                    passed = true;
                }
            }
            else
            {
                passed = true;
            }
#ifdef DEBUGBALL
            Point2f rect_points[4];
            minRect.points(rect_points);
            for (int j = 0; j < 4; j++)
            {
                line(contoursImg, rect_points[j], rect_points[(j + 1) % 4], Scalar(100, 0, 255), 2);
            }
#endif
        }
        if (passed)
        {
            circle(contoursImg, center, radius, Scalar(255, 0, 100), 2);
            //Tambahkan ke kandidat
            kandidatBola.push_back(Point3f(center.x, center.y, radius));
        }
    }
    if (cannyOn)
    {
        //--------------------------------Hough Circle--------------------------------
        //PreProcessing lagi
        Canny(th, edge, lowThreshold, lowThreshold * ratio, 7);
        GaussianBlur(edge, edge, Size(9, 9), 2, 2, 4);
        vector<Vec3f> circles;
        HoughCircles(edge, circles, HOUGH_GRADIENT, 1, edge.rows / min_dist,
                     upper_threshold, center_threshold, min_r, max_r);
        for (Vec3f c : circles)
        {
#ifdef DEBUGBALL
            circle(contoursImg, Point(c[0], c[1]), c[2] / 2, Scalar(255, 0, 0), 8);
#endif
            kandidatBola.push_back(Point3f(c[0], c[1], c[2]));
        }
    }
    //--------------------------------Cek lapangan semua kandidat bola--------------------------------
    vector<Point3f> bola;
    for (Point3f kandidat : kandidatBola)
    {
        float radius = kandidat.z;
        Point2f center = Point2f(kandidat.x, kandidat.y);
        int lebarBoundBox = radius * 4;
        int tinggiBoundBox = radius;
        int awalX = center.x - lebarBoundBox / 2;
        int awalY = center.y + tinggiBoundBox / 4; //turun 1/2 tinggi
        //Cek jika nilai awalX keluar dari batas kiri, jika iya, geser ke 0, kemudian panjang bounding box dikurangi sebanyak geser
	if (awalX < 0)
        {
            awalX = 0;
            lebarBoundBox += awalX;
        }
        if (awalX + lebarBoundBox > fieldCopy.cols)
        {
            lebarBoundBox -= (awalX + lebarBoundBox - fieldCopy.cols);
        }
	lebarBoundBox = max(0, lebarBoundBox);

	//Nilai awalY sama juga idenya dengan awalX
        if (awalY < 0)
        {
            awalY = 0;
            tinggiBoundBox += awalY;
        }
        if (awalY + tinggiBoundBox > fieldCopy.rows)
        {
            tinggiBoundBox -= (awalY + tinggiBoundBox - fieldCopy.rows);
        }
	tinggiBoundBox = max(0, tinggiBoundBox);
	// Cek validitas nilai
	bool awalXValid = awalX > 0 && awalX < fieldCopy.cols;
	bool awalYValid = awalY > 0 && awalY < fieldCopy.rows;
	bool tinggiValid = tinggiBoundBox > 0 && tinggiBoundBox < fieldCopy.rows;
	bool lebarValid = lebarBoundBox > 0 && lebarBoundBox < fieldCopy.cols;
	if(awalXValid && awalYValid && tinggiValid && lebarValid)
	{
        	Rect boundingBox(awalX, awalY, lebarBoundBox, tinggiBoundBox);
        	Mat maskField = fieldCopy(boundingBox);
        	int countNZero = countNonZero(maskField);
        	int countMax = lebarBoundBox * tinggiBoundBox;
        	double rasioFieldCon = (double)countNZero / countMax * 100;
        	rectangle(contoursImg, boundingBox, Scalar(0,0,0), 6);
        	// ROS_ERROR_STREAM("rfc : " << rasioFieldCon << endl);
        	// ROS_ERROR_STREAM("cnz : " << countNZero << endl);
        	// ROS_ERROR_STREAM("cnm : " << countMax << endl);
        	// imshow("Mask Field", maskField);
        	if (rasioFieldCon > epsFieldCheck)
        	{
            		// ROS_ERROR_STREAM("Kandidat" << kandidat.x << " " << kandidat.y << "Pass RC" << endl);
            		if (radius >= minRContour && radius <= maxRContour)
            		{
                		circle(contoursImg, center, radius, Scalar(22, 12, 123), 7);
                		bola.push_back(kandidat);
            		}
        	}
	}
    }
    imshow("Con", contoursImg);
    //--------------------------------Sort bola berdasar radius--------------------------------
    sort(bola.begin(), bola.end(), compareBola);
    if (bola.size() >= 1)
    {
        pos = Point2d(bola[0].x, bola[0].y);
        // cout << pos.X << ", " << pos.Y << endl;
    }
    else
    {
        pos = Point2d(-1, -1);
    }
    waitKey(1);
}

Point2d &BallFinder::getPosition()
{
    return pos;
}

void BallFinder::paramCallback(daho_vision::BallConfig &config, uint32_t level)
{
    min_Y = config.min_Y;
    max_Y = config.max_Y;
    min_U = config.min_U;
    max_U = config.max_U;
    min_V = config.min_V;
    max_V = config.max_V;
    countErode = config.countErode;
    min_dist = config.min_dist;
    upper_threshold = config.upper_threshold;
    center_threshold = config.center_threshold;
    min_r = config.min_r;
    max_r = config.max_r;
    epsBoundRC = config.epsBoundRC;
    epsBoundEC = config.epsBoundEC;
    epsFieldCheck = config.epsFieldCheck;
    minRContour = config.minRContour;
    maxRContour = config.maxRContour;
}

// void BallFinder::SaveINISettings(minIni *ini, const std::string &section)
// {
//     ini->put(section, "min_Y", min_Y);
//     ini->put(section, "max_Y", max_Y);
//     ini->put(section, "min_U", min_U);
//     ini->put(section, "max_U", max_U);
//     ini->put(section, "min_V", min_V);
//     ini->put(section, "max_V", max_V);
//     ini->put(section, "fmin_H", fmin_H);
//     ini->put(section, "fmax_H", fmax_H);
//     ini->put(section, "fmin_S", fmin_S);
//     ini->put(section, "fmax_S", fmax_S);
//     ini->put(section, "fmin_V", fmin_V);
//     ini->put(section, "fmax_V", fmax_V);
//     ini->put(section, "Threshold", detectionThreshold);
//     ini->put(section, "countDilate", countDilate);
//     ini->put(section, "countErode", countErode);
//     ini->put(section, "lowThreshold", lowThreshold);
//     ini->put(section, "ratio", ratio);
//     ini->put(section, "kernel_size", kernel_size);
//     ini->put(section, "min_dist", min_dist);
//     ini->put(section, "upper_threshold", upper_threshold);
//     ini->put(section, "center_threshold", center_threshold);
//     ini->put(section, "min_r", min_r);
//     ini->put(section, "max_r", max_r);
//     ini->put(section, "epsBoundRC", epsBoundRC);
//     ini->put(section, "epsBoundEC", epsBoundEC);
//     ini->put(section, "epsFieldCheck", epsFieldCheck);
//     ini->put(section, "minRContour", minRContour);
//     ini->put(section, "maxRContour", maxRContour);

//     // ini->put(section, "minEpsCon", minEpsCon);
//     // ini->put(section, "maxEpsCon", maxEpsCon);
//     // ini->put(section, "minEpsRect", minEpsCon);
//     // ini->put(section, "maxEpsRect", maxEpsCon);
//     // ini->put(section, "epsConField", epsConField);
//     // ini->put(section, "min_r_contour", min_r_contour);
//     // ini->put(section, "max_r_contour", max_r_contour);

//     color_section = section;
// }
