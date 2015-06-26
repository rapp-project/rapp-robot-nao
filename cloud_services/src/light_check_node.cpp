#include "ros/ros.h"
#include "cloud_services/LightCheck.h"

#include "cloud_services/light_check.hpp"

bool service_LightCheck(cloud_services::LightCheck::Request  &req,
                        cloud_services::LightCheck::Response &res)
{
  res.result = LightCheck::lightCheck(req.fname);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_objects_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("light_check", service_LightCheck);
  ROS_INFO("Ready for light checking.");
  ros::spin();

  return 0;
}
