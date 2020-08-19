// Soccer Vision
// Author: Andreas Ekadinata Widodo <ekadinataa@gmail.com>
//         Ayrton Cyril
//         Aditya Putra Santosa <adityaputra159@gmail.com>

/**
* KRSBI ITB
* Dago Hoogeschool Team
**/

#ifndef BallFinder_H
#define BallFinder_H

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "daho_vision/BallConfig.h"

#include <stdio.h>

#define max_val 255
#define max_lowThreshold 500

using namespace cv;

namespace Robot
{

class BallFinder
{
private:
  bool flagContour, flagHough;

  Mat img;
  Mat yuv;
  Mat th;
  Mat hsv;
  Mat gray;
  Mat edge;
  Mat ball;
  Mat contoursImg;

  static void on_trackbar(int value, void *userdata);
  // void CreateTrackbar();
  // void CreateTrackbarField();
  void morphOps(Mat &thresh);

public:
  std::string color_section;

  int min_Y;
  int max_Y;
  int min_U;
  int max_U;
  int min_V;
  int max_V;

  int fmin_H;
  int fmax_H;
  int fmin_S;
  int fmax_S;
  int fmin_V;
  int fmax_V;

  int countErode;
  int countDilate;

  int detectionThreshold;
  double count;

  int lowThreshold;
  int ratio;
  int kernel_size;

  int min_dist;
  int upper_threshold;
  int center_threshold;
  int min_r;
  int max_r;
  int radius;

  int epsBoundRC, epsBoundEC;
  int epsFieldCheck;
  int minRContour, maxRContour;

  BallFinder();
  virtual ~BallFinder();

  Point2d pos;
  bool isDetectBall(Point2d center);
  void printParam();
  void Process(Mat image, Mat field, bool cannyOn = false, bool checkEllipse = false);
  // void ControlPanel(minIni* ini);
  Point2d &getPosition();
  void paramCallback(daho_vision::BallConfig &config, uint32_t level);
  // void showResult(Mat image);
  // void LoadINISettings(minIni* ini);
  // void LoadINISettings(minIni* ini, const std::string &section);
  // void SaveINISettings(minIni* ini);
  // void SaveINISettings(minIni* ini, const std::string &section);
  int getRadius();
};
} // namespace Robot

#endif