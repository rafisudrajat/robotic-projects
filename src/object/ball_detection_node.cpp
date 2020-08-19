#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "daho_vision/BallFinder2019.h"
// #include "daho_vision/HasilField.h"
#include "daho_vision/PosisiBola.h"
#include "dynamic_reconfigure/server.h"

class ball_detection_node
{
private:
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  ros::Publisher posisi_bola;
  image_transport::Subscriber real_image, field_mask;
  Robot::BallFinder ball_finder;
  Mat realImage, fieldMask;

public:
  ball_detection_node() : it(nh)
  {
    real_image = it.subscribe("/cv_camera/image_raw", 1, &ball_detection_node::realImageImageCallback, this);
    field_mask = it.subscribe("/field_detection/field_mask", 1, &ball_detection_node::fieldMaskImageCallback, this);
    posisi_bola = nh.advertise<daho_vision::PosisiBola>("/ball_detection/posisi_bola", 1);
  }

  void realImageImageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr_image_real;
    try
    {
      cv_ptr_image_real = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      //ROS_INFO("Berhasil copy gambar camera di ball detection");
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    realImage = cv_ptr_image_real->image;
  }

  void fieldMaskImageCallback(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr_field_mask;
    try
    {
      cv_ptr_field_mask = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      //ROS_INFO("Berhasil copy field mask di ball detection");
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    fieldMask = cv_ptr_field_mask->image;
    // imshow("Field ROS", fieldMask);
    processImage();
    // Proses image
  }

  void processImage()
  {
    ball_finder.Process(realImage, fieldMask, true, true);

    daho_vision::PosisiBola pb;
    pb.x = ball_finder.getPosition().x;
    pb.y = ball_finder.getPosition().y;
    posisi_bola.publish(pb);
  }

  void paramCallback(daho_vision::BallConfig &config, uint32_t level)
  {
    ball_finder.paramCallback(config, level);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ball_detection_node");
  ball_detection_node bdn;

  dynamic_reconfigure::Server<daho_vision::BallConfig> server;
  dynamic_reconfigure::Server<daho_vision::BallConfig>::CallbackType f;

  f = boost::bind(&ball_detection_node::paramCallback, &bdn, _1, _2);
  server.setCallback(f);

  ros::spin();

  return 0;
}
