# include <ros/ros.h>
# include <image_transport/image_transport.h>
# include <cv_bridge/cv_bridge.h>
# include <sensor_msgs/image_encodings.h>
# include "daho_vision/PoleFinder.h"
#include "daho_vision/ContourPMF.h"
#include "daho_vision/arrOfxPole.h"
#include "dynamic_reconfigure/server.h"

# include <iostream>

# include <opencv2/imgproc/imgproc.hpp>
# include <opencv2/highgui/highgui.hpp>

// TODO : Bikin cfg poleFinder
// TODO : Bikin msg arrOfxPole

using namespace std;

class PoleNode {
    private :
        ros::NodeHandle nh;
        ros::Publisher xPole;
        image_transport::Subscriber Field_Mask, Camera;
        Robot2019::PoleFinder poleFinder;
        Mat CameraImage, FieldMaskImage;
        image_transport::ImageTransport it;
        ros::ServiceServer ContourPMF;

    public :
        // Constructor
        PoleNode() : it(nh) {
            Camera = it.subscribe("/cv_camera/image_raw", 1, &PoleNode::CameraCallBack, this);
            Field_Mask = it.subscribe("/field_detection/field_mask", 1, &PoleNode::Field_MaskCallBack, this);
            xPole = nh.advertise<daho_vision::arrOfxPole>("/poleFinder/xPole", 1);
            ContourPMF = nh.advertiseService("/poleFinder/GetContourPMF", &PoleNode::getContour_PMF, this);
        }

        //Callback Function
        bool getContour_PMF(daho_vision::ContourPMF::Request &req, daho_vision::ContourPMF::Response &res){
            
            // res.ratio = poleNode.getContourPMF(req.idx);

            return true;
        }

        void CameraCallBack(const sensor_msgs::ImageConstPtr &msg) {
            cv_bridge::CvImagePtr Camera_ptr;

            try {
                Camera_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                CameraImage = Camera_ptr->image;
            } catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception : %s", e.what());
                return;
            }
        }

        void Field_MaskCallBack(const sensor_msgs::ImageConstPtr &msg) {
            cv_bridge::CvImagePtr Field_Mask_ptr;

            try {
                Field_Mask_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                FieldMaskImage = Field_Mask_ptr->image;
                ProcessAndPublish();
            } catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception : %s", e.what());
                return;
            }
        }

        // Main Process
        void ProcessAndPublish() {
            poleFinder.process(CameraImage, FieldMaskImage);

            daho_vision::arrOfxPole data;
            data.xComponent = poleFinder.getXPole();

            xPole.publish(data);
        }

        void paramCallback(daho_vision::PoleFinderConfig &config, uint32_t level) {
            poleFinder.paramCallback(config, level);
        }
};

int main (int argc, char **argv) {
    cout << "It's starting" << endl;
    ros::init(argc, argv, "pole_detection_node");
    PoleNode poleNode;

    dynamic_reconfigure::Server<daho_vision::PoleFinderConfig> server;
    dynamic_reconfigure::Server<daho_vision::PoleFinderConfig>::CallbackType f;

    f = boost::bind(&PoleNode::paramCallback, &poleNode, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}