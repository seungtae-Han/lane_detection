#include <iostream>
#include "lane_detection/lane_detection_node.h"
#include "opencv2/opencv.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "seungtae_detection");

	// using usb_cam
	//LaneDetectorNode lane_detector_node;
	//ros::spin();


	// using mp4 file
	LaneDetectorNode lane_detector_node("/home/seungtae/catkin_ws/src/lane_detection/src/project_video.mp4");
	lane_detector_node.run_test();

	return 0;
}
