#include "lane_detection/lane_detection_node.h"

using namespace std;
using namespace cv;

// when using usb_cam or camera
LaneDetectorNode::LaneDetectorNode()
{
    n = ros::NodeHandle();
    steer_pub = n.advertise<std_msgs::Float32>("steer", 1000);
    image_sub = n.subscribe("/usb_cam/image_raw", 1, &LaneDetectorNode::imageCallback, this);
    //image_sub = n.subscribe("/camera/image_color", 1, &LaneDetectorNode::imageCallback, this);
}

// when using video
LaneDetectorNode::LaneDetectorNode(String path)
	: test_video_path(path)
{}

void LaneDetectorNode::parseRawimg(const sensor_msgs::ImageConstPtr& ros_img, cv::Mat& cv_img)
{
    //ros_img : image, cv_img : frame
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);
	cv_img = cv_ptr->image;
	if (cv_img.empty()) throw std::runtime_error("frame is empty!");
}

// calculate angle
double LaneDetectorNode::LaneDetecting()
{   
    img_denoise = lanedetector.deNoise(frame);
    img_edges = lanedetector.edgeDetector(img_denoise);
    img_mask = lanedetector.mask(img_edges);
    lines = lanedetector.houghLines(img_mask);
    if (!lines.empty()) 
    {
        left_right_lines = lanedetector.lineSeparation(lines, img_mask);
        lane = lanedetector.regression(left_right_lines, frame);
        lanedetector.plotLane(frame, lane);
        angle = lanedetector.angleCal(frame, lane);
        cout << "angle : " << angle << endl;
    }
    
    return angle;
}

void LaneDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
    try {parseRawimg(image, frame);} 
    catch(const cv_bridge::Exception& e)
    {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return ;
	}
    catch(const std::runtime_error& e) {cerr << e.what() << endl;}

    double angle = LaneDetecting();
    cv::waitKey(3);

    std_msgs::Float32 control_msg;
    control_msg.data = angle;
    steer_pub.publish(control_msg);
}

bool LaneDetectorNode::run_test()
{
	if(test_video_path.empty())
	{
		ROS_ERROR("Test is failed. video path is empty! you should set video path by constructor argument");
		return false;
	}

	VideoCapture cap;
	cap.open(test_video_path);

	if (!cap.isOpened())
	{
		ROS_ERROR("Test is failed. video is empty! you should check video path (constructor argument is correct)");
		return false;
	}

	while (1) 
    {
		if (!cap.read(frame)) break;
        double angle = LaneDetecting();
        cv::waitKey(15);
	}
}