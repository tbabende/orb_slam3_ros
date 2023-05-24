/**
* 
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_mono.cc
*
*/

#include "common.h"

#include "tf/transform_datatypes.h"
#include "DenseInput.h"
#include "MaskImage.h"

using namespace std;



class ImageGrabber
{
public:
    ImageGrabber(){};

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageAndMask(const detectron2_ros::MaskImageConstPtr& msg);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }



    std::string node_name = ros::this_node::getName();

    ros::NodeHandle node_handler;
    image_transport::ImageTransport image_transport(node_handler);

    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");       
        ros::shutdown();
        return 1;
    }

    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "map");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");

    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::MONOCULAR;
    pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, enable_pangolin);
    ImageGrabber igb;

    // ros::Subscriber sub_img = node_handler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);
    ros::Subscriber sub_img_and_mask = node_handler.subscribe("/camera/image_and_mask", 1, &ImageGrabber::GrabImageAndMask, &igb);

    setup_publishers(node_handler, image_transport, node_name);
    setup_services(node_handler, node_name);

    ros::spin();

    // Stop all threads
    pSLAM->Shutdown();
    ros::shutdown();

    return 0;
}

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr_im;
    try
    {
        cv_ptr_im = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // ORB-SLAM3 runs in TrackMonocular()
    Sophus::SE3f Tcw = pSLAM->TrackMonocular(cv::Mat::ones(cv_ptr_im->image.size(), CV_8UC1), cv_ptr_im->image, msg->header.stamp.toSec());

    ros::Time msg_time = msg->header.stamp;

    publish_topics(msg_time);
}

void ImageGrabber::GrabImageAndMask(const detectron2_ros::MaskImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr_im;
    cv_bridge::CvImageConstPtr cv_ptr_mask;
    try
    {
        cv_ptr_im = cv_bridge::toCvShare(msg->image, msg);
        cv_ptr_mask = cv_bridge::toCvShare(msg->mask, msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // ORB-SLAM3 runs in TrackMonocular()
    Sophus::SE3f Tcw = pSLAM->TrackMonocular(cv_ptr_mask->image, cv_ptr_im->image, msg->header.stamp.toSec());

    ros::Time msg_time = msg->header.stamp;

    publish_topics(msg_time);
}