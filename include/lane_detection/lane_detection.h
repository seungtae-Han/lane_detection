#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>
#include <vector>

#ifndef LANE_DETECTION_H
#define LANE_DETECTION_H

class LaneDetector {
    private :
        double img_size;
        double img_center;
        bool left_flag = false;
        bool right_flag = false;
        cv::Point right_b;
        double right_m;
        cv::Point left_b;
        double left_m;
        cv::Mat ROI;
        cv::Mat Final;

    public :
        cv::Mat deNoise(cv::Mat inputImage);
        cv::Mat edgeDetector(cv::Mat img_noise);
        cv::Mat mask(cv::Mat img_edges);
        std::vector<cv::Vec4i> houghLines(cv::Mat img_mask);
        std::vector<std::vector<cv::Vec4i> > lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_mask);
        std::vector<double> angleCal(std::vector<cv::Vec4i> lines, cv::Mat img_edges);
        std::vector<cv::Point> regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage);
        void plotLane(cv::Mat inputImage, std::vector<cv::Point> lane);
        double angleCal(cv::Mat inputImage, std::vector<cv::Point> lane);
};

#endif
