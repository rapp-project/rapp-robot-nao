#include <vector>
#include <string>

#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>

namespace FindObjects {

int findObjects(std::vector<std::string> names, std::vector<std::string> files, int limit, 
                std::vector<std::string> & found_names, std::vector<geometry_msgs::Point> & found_centers, std::vector<double> & found_scores) {
  ROS_INFO("Finding objects from %d models", names.size());
  
  // TODO: Implement here
  
  cv::Mat img;
  
  return 0;
}

};
