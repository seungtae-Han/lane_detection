#include "lane_detection/lane_detection.h"

// IMAGE BLURRING
cv::Mat LaneDetector::deNoise(cv::Mat inputImage)
{
    cv::Mat resized_inputImage;
    cv::resize(inputImage, resized_inputImage, cv::Size(inputImage.cols*0.6, inputImage.rows*0.6), 0, 0, CV_INTER_LINEAR);
    cv::imshow("inputImage", resized_inputImage);

    // gaussian blurring
    cv::Mat blur;
    cv::GaussianBlur(inputImage, blur, cv::Size(3, 3), 0, 0);
    
    //cv::imshow("blur", blur);

    return blur;
}

// EDGE DETECTION
cv::Mat LaneDetector::edgeDetector(cv::Mat img_noise)
{
    cv::Mat gray, thre_w, filter2D, kernel, hls, thre_y, thre;
    cv::Point anchor;

    // gray converstion
    cv::cvtColor(img_noise, gray, cv::COLOR_RGB2GRAY);

    // thresholding for white lane
    cv::threshold(gray, thre_w, 210, 255, cv::THRESH_BINARY);

    // hls converstion
    cv::cvtColor(img_noise, hls, cv::COLOR_BGR2HSV);

    // thresholding for yellow lane
    cv::Scalar low(20, 150, 100);
    cv::Scalar upp(30, 255, 255);
    cv::inRange(hls, low, upp, thre_y);

    // merge white lane and yellow lane
    cv::bitwise_or(thre_w, thre_y, thre);

    // create the kernel [-1 0 1]
    anchor = cv::Point(-1, -1);
    kernel = cv::Mat(1, 3, CV_32F);
    kernel.at<float>(0, 0) = -1;
    kernel.at<float>(0, 1) = 0;
    kernel.at<float>(0, 2) = 1;

    // filter the binary image to obtain the edges
    cv::filter2D(thre, filter2D, -1, kernel, anchor, 0, cv::BORDER_DEFAULT);

    //cv::imshow("thre_w", thre_w);
    //cv::imshow("thre_y", thre_y);
    //cv::imshow("thre", thre);
    //cv::imshow("filter2D", filter2D);

    return filter2D;
}

// MASK THE EDGE IMAGE
cv::Mat LaneDetector::mask(cv::Mat img_edges)
{
    // create a rectangle to mask the input image
    cv::Point pts[4] = {
        cv::Point(0, img_edges.rows),
        cv::Point(0, img_edges.rows*1/2),
        cv::Point(img_edges.cols, img_edges.rows*1/2),
        cv::Point(img_edges.cols, img_edges.rows) 
    };
    cv::Mat mask = cv::Mat::zeros(img_edges.size(), img_edges.type());
    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255, 255, 255));
    cv::bitwise_and(img_edges, mask, ROI);

    //cv::imshow("ROI", ROI);
  
    return ROI;
}

// HOUGH TRANSFORM
std::vector<cv::Vec4i> LaneDetector::houghLines(cv::Mat img_mask)
{
    std::vector<cv::Vec4i> line;
    cv::Point ini, fini;
    HoughLinesP(img_mask, line, 1, CV_PI/180, 65, 20, 30);

    /////////////////// plot hough line //////////////////////
    // cv::Mat houghLines = cv::Mat::zeros(img_mask.size(), img_mask.type());
    
    // cv::Point pts[4] = {
    //     cv::Point(0, 0),    //left bottom
    //     cv::Point(0, houghLines.rows),    //left top
    //     cv::Point(houghLines.cols, houghLines.rows),    //right top
    //     cv::Point(houghLines.cols, 0)     //right bottom
    // };
    
    // cv::fillConvexPoly(houghLines, pts, 4, cv::Scalar(0, 0, 0));
    
    // for( size_t i = 0; i < line.size(); i++ )
    // {
    //     ini = cv::Point(line[i][0], line[i][1]);
    //     fini = cv::Point(line[i][2], line[i][3]);
    //     cv::line(houghLines, ini, fini, cv::Scalar(255, 255, 255), 1, CV_AA);
    // }
    // cv::resize(houghLines, houghLines, cv::Size(img_mask.cols*0.6, img_mask.rows*0.6), 0, 0, CV_INTER_LINEAR);
    // cv::imshow("houghLines", houghLines);
    /////////////////////////////////////////////////////////
    
    return line;
}

// SORT RIGHT AND LEFT LINES
std::vector<std::vector<cv::Vec4i> > LaneDetector::lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_mask) {
    std::vector<std::vector<cv::Vec4i> > left_right_lines(2);
    size_t j = 0;
    cv::Point ini, fini;
    double slope_thresh = 0.3;
    std::vector<double> slopes;
    std::vector<cv::Vec4i> selected_lines, right_lines, left_lines;

    // calculate the slope of all the detected lines
    for (auto k : lines)
    {
        ini = cv::Point(k[0], k[1]);
        fini = cv::Point(k[2], k[3]);

        // slope = (y1 - y0)/(x1 - x0)
        double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y))/(static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);
        
        // slope is too horizontal -> discard
        // if not -> save
        if (std::abs(slope) > slope_thresh) 
        {
            slopes.push_back(slope);
            selected_lines.push_back(k);
        }
    }

    // split the lines into right and left lines
    img_center = static_cast<double>((img_mask.cols / 2));
    while (j < selected_lines.size()) 
    {
        ini = cv::Point(selected_lines[j][0], selected_lines[j][1]);
        fini = cv::Point(selected_lines[j][2], selected_lines[j][3]);

        if (slopes[j] > 0 && fini.x > img_center && ini.x > img_center)
        {
            right_lines.push_back(selected_lines[j]);
            right_flag = true;
        }

        else if (slopes[j] < 0 && fini.x < img_center && ini.x < img_center)
        {
            left_lines.push_back(selected_lines[j]);
            left_flag = true;
        }

        j++;
    }

    left_right_lines[0] = right_lines;
    left_right_lines[1] = left_lines;

    ///////////////// plot right and left lanes ///////////////////
    cv::Mat left_right = cv::Mat::zeros(img_mask.size(), img_mask.type());
    
    cv::Point pts[4] = {
        cv::Point(0, 0),    //left bottom
        cv::Point(0, left_right.rows),    //left top
        cv::Point(left_right.cols, left_right.rows),    //right top
        cv::Point(left_right.cols, 0)     //right bottom
    };

    cv::fillConvexPoly(left_right, pts, 4, cv::Scalar(0, 0, 0));

    for( size_t i = 0; i < right_lines.size(); i++ )
    {
        ini = cv::Point(right_lines[i][0], right_lines[i][1]);
        fini = cv::Point(right_lines[i][2], right_lines[i][3]);
        cv::line(left_right, ini, fini, cv::Scalar(255, 255, 255), 1, CV_AA);
    }

    for( size_t i = 0; i < left_lines.size(); i++ )
    {
        ini = cv::Point(left_lines[i][0], left_lines[i][1]);
        fini = cv::Point(left_lines[i][2], left_lines[i][3]);
        cv::line(left_right, ini, fini, cv::Scalar(255, 255, 255), 1, CV_AA);
    }
    
    cv::resize(left_right, left_right, cv::Size(left_right.cols*0.6, left_right.rows*0.6), 0, 0, CV_INTER_LINEAR);
    cv::imshow("left_right_lines", left_right);
    ////////////////////////////////////////////////////////////

    return left_right_lines;
}


// REGRESSION FOR LEFT AND RIGHT LINES
std::vector<cv::Point> LaneDetector::regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage)
{
    std::vector<cv::Point> lane_point(4);
    cv::Point ini, fini;
    cv::Point ini2, fini2;
    cv::Vec4d right_line, left_line;
    std::vector<cv::Point> right_pts, left_pts;

    // fit a right line using all the init and final points
    if (right_flag == true) 
    {
        for (auto i : left_right_lines[0]) 
        {
            ini = cv::Point(i[0], i[1]);
            fini = cv::Point(i[2], i[3]);

            right_pts.push_back(ini);
            right_pts.push_back(fini);
        }

        if (right_pts.size() > 0) 
        {
            cv::fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01, 0.01);
            right_m = right_line[1] / right_line[0];
            right_b = cv::Point(right_line[2], right_line[3]);
        }
    }
    
    // fit a left line using all the init and final points
    if (left_flag == true) 
    {
        for (auto j : left_right_lines[1]) 
        {
            ini2 = cv::Point(j[0], j[1]);
            fini2 = cv::Point(j[2], j[3]);

            left_pts.push_back(ini2);
            left_pts.push_back(fini2);
        }

        if (left_pts.size() > 0) 
        {
            cv::fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01, 0.01);
            left_m = left_line[1] / left_line[0];
            left_b = cv::Point(left_line[2], left_line[3]);            
        }
    }

    // apply the line equation to obtain the line points
    int ini_y = inputImage.rows;
    int fin_y = inputImage.rows * 60 / 100;

    double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
    double right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;

    double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
    double left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;
    
    lane_point[0] = cv::Point(right_ini_x, ini_y);  // right bottom
    lane_point[1] = cv::Point(right_fin_x, fin_y);  // right top
    lane_point[2] = cv::Point(left_ini_x, ini_y);  // left bottom
    lane_point[3] = cv::Point(left_fin_x, fin_y);  // left top

    return lane_point;
}

// PLOT RESULTS
void LaneDetector::plotLane(cv::Mat inputImage, std::vector<cv::Point> lane)
{
    // create polygon
    std::vector<cv::Point> poly_points;
    cv::Mat output;
    inputImage.copyTo(output);
    poly_points.push_back(lane[2]);  // left bottom
    poly_points.push_back(lane[0]);  // right bottom
    poly_points.push_back(lane[1]);  // right top
    poly_points.push_back(lane[3]);  // left top
    cv::fillConvexPoly(output, poly_points, cv::Scalar(0, 0, 255), CV_AA, 0); //다각형 채우기
    cv::addWeighted(output, 0.3, inputImage, 1.0 - 0.3, 0, inputImage);

    // plot lanes
    cv::line(inputImage, lane[0], lane[1], cv::Scalar(0, 255, 255), 5, CV_AA);
    cv::line(inputImage, lane[2], lane[3], cv::Scalar(0, 255, 255), 5, CV_AA);

    // Show the final output image
    cv::Mat final_image;
    cv::resize(inputImage, final_image, cv::Size(inputImage.cols*0.6, inputImage.rows*0.6), 0, 0, CV_INTER_LINEAR);
    cv::imshow("Lane", final_image);
}


double LaneDetector::angleCal(cv::Mat inputImage, std::vector<cv::Point> lane)
{
    double fin_y = inputImage.rows * 60 / 100;
    
    double center_x = inputImage.cols / 2 ;

    //theta = atan((x1-x2)/y)
    double lane_center_x = (lane[0].x + lane[2].x) / 2 ;

    double Y = lane_center_x - center_x;

    double X = inputImage.rows - fin_y;

    double angle = atan2(Y, X) * 180 / 3.141592654;

    return angle;
}
