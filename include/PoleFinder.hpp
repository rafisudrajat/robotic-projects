#ifndef POLE_DETECTOR
#define POLE_DETECTOR

#include "ros/ros.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "daho_vision/PoleFinderConfig.h"
#include <vector>

// TODO : Crete pole detector config file

using namespace std;
using namespace cv;

// #define INVALID_VALUE -1024
#define v2f vector<Vec2f>
#define v6f vector<Vec6f>
#define v3f vector<Vec3f>
#define v4i vector<Vec4i>
#define v4f vector<Vec4f>
#define v2i vector<Vec2i>

namespace Robot2019
{
class PoleFinder
{
private:
  // static GoalPerceptor *m_UniqueInstance;
  vector<int> xPole;
  vector<double> cArea;
  double totalAreaContour;
  int minB, maxB, minG, maxG, minR, maxR;
  int yCrop;
  int sigma, tetha, lambd, gamma;
  int minFieldRatio, maxFieldRatio;
  // int minFieldAtasR, maxFieldAtasR;
  bool cekLapangan(RotatedRect boundRect, Mat field);

public:
  int minVarian, maxVarian;
  //Constructor
  PoleFinder();
  //Destructor
  ~PoleFinder();
  //Setter
  void init();
  // static GoalPerceptor *GetInstance() { return m_UniqueInstance; }

  // Attribute config
  void paramCallback(daho_vision::PoleFinderConfig &config, uint32_t level);

  //Init GUI
  static void on_trackbar(int, void *);
  void initWindow();
  // void initTrackbar();

  void process(Mat m, Mat field);

  vector<int> getXPole() { return xPole; }

  double getContourPMF(int idx);
};
}
#endif