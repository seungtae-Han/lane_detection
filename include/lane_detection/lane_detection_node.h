#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "lane_detection/lane_detection.h"


#ifndef LANEDETECTORNODE_H
#define LANEDETECTORNODE_H

class LaneDetectorNode {
    public:
        LaneDetectorNode();
        LaneDetectorNode(cv::String path);

        cv::String test_video_path;

        bool run_test();

        void imageCallback(const sensor_msgs::ImageConstPtr& image);

    protected:
        double LaneDetecting();
        ros::NodeHandle n;
        ros::Publisher steer_pub;
        ros::Subscriber image_sub;

        LaneDetector lanedetector;
        cv::Mat frame;
        cv::Mat resize_frame;
        cv::Mat img_denoise;
        cv::Mat img_edges;
        cv::Mat img_mask;
        cv::Mat img_lines;
        std::vector<cv::Vec4i> lines;
        std::vector<std::vector<cv::Vec4i> > left_right_lines;
        std::vector<cv::Point> lane;
        std::string turn;
        int flag_plot;
        double angle;

        void parseRawimg(const sensor_msgs::ImageConstPtr& ros_img, cv::Mat& cv_img);
};

#endif
