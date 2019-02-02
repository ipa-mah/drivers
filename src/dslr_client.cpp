#include <ros/ros.h>
#include <hardware_driver/DSLRCameraService.h>
#include <hardware_driver/KapturaService.h>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <stdio.h>
#include <iostream>
using namespace  std;
int main(int argc, char **argv)
{

  ros::init(argc, argv, "test_client");
  ros::NodeHandle n;

  /*
  ros::ServiceClient client = n.serviceClient<hardware_driver::DSLRCameraService>("dslr_capture");
  hardware_driver::DSLRCameraService srv;
  for(int i=0;i<10;i++)
  {
    std::ostringstream ss;
    ss<<i;
    srv.request.image_name = "/home/manhha/catkin_ws/"+ss.str()+".png";
    if (client.call(srv))
    {
      ROS_INFO("result: %ld", (long int)srv.response.success);
      ROS_INFO("image path :%s\n", srv.response.image_path.c_str());
    }
    else
    {
      ROS_ERROR("  Could not capture image from DSLR, exiting ...\n");
      abort();
    }
  }
*/
  ros::ServiceClient client = n.serviceClient<hardware_driver::KapturaService>("kaptura_service");
  hardware_driver::KapturaService srv;
  srv.request.degree = 360;
  srv.request.speed = 10;
  srv.request.station_version = "201712";
  if (client.call(srv))
  {
    ROS_INFO("result: %ld", (long int)srv.response.success);
  }
  else
    ROS_ERROR("  Could not run kaptura frame\n");


  return 0;
}
