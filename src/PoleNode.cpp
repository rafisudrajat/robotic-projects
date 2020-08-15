# include <ros/ros.h>
# include <image_transport/image_transport.h>
# include <cv_bridge/cv_bridge.h>
# include <sensor_msgs/image_encodings.h>
# include "daho_vision/PoleFinder.h"
#include "daho_vision/ContourPMF.h"
#include "daho_vision/arrOfxPole.h"
#include "dynamic_reconfigure/server.h"

# include <opencv2/imgproc/imgproc.hpp>
# include <opencv2/highgui/highgui.hpp>

// TODO : Bikin cfg poleFinder
// TODO : Bikin msg arrOfxPole

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
        goal_post_detection_node() : it(nh) {
            Camera = it.subscribe("/cv_camera/image_raw", 1, &goal_post_detection_node::CameraCallBack, this);
            Field_Mask = it.subscribe("/field_detection/field_mask", 1, &goal_post_detection_node::Field_MaskCallBack, this);
            xPole = nh.advertise<daho_vision::arrOfxPole>("/poleFinder/xPole", 1);
            ContourPMF = N.advertiseService("/poleFinder/GetContourPMF", &goal_post_detection_node::getContour_PMF, this);
        }

        //Callback Function
        bool getContour_PMF(daho_vision::ContourPMF::Request &req, daho_vision::ContourPMF::Response &res){
            
            res.ratio = goalPerceptor.getContourPMF(req.idx);

            return true;
        }

        void CameraCallBack(const sensor_msgs::ImageConstPtr &msg) {
            cv_bridge::CvImagePtr Camera_ptr;

            try {
                Camera_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception : %s", e.what());
                return;
            }

            CameraImage = Camera_ptr->image;

        }

        void Field_MaskCallBack(const sensor_msgs::ImageConstPtr &msg) {
            cv_bridge::CvImagePtr Field_Mask_ptr;

            try {
                Field_Mask_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception &e) {
                ROS_ERROR("cv_bridge exception : %s", e.what());
                return;
            }

            FieldMaskImage = Field_Mask_ptr->image;

            ProcessAndPublish();
        }

        // Main Process
        void ProcessAndPublish() {
            poleFinder.process(CameraImage, FieldMaskImage);

            daho_vision::arrOfxPole data;
            Data.xComponent = poleFinder.getXPole();

            xPole.publish(data);
        }

        void paramCallback(daho_vision::PoleFinderConfig &config, uint32_t level) {
            goalPerceptor.paramCallback(config, level);
        }
};

int main (int argc, char **argv) {

    ros::init(argc, argv, "goal_post_detection_node");
    PoleNode poleNode;

    dynamic_reconfigure::Server<daho_vision::PoleFinderConfig> server;
    dynamic_reconfigure::Server<daho_vision::PoleFinderConfig>::CallbackType f;

    f = boost::bind(&poleNode::paramCallback, &poleNode, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}