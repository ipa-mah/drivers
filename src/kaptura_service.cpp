#include <curl/curl.h>
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <istream>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <hardware_driver/KapturaService.h>

struct webserver_signal {
  std::string scale_value;
  float motor_speed;
  float motor_position;
};
CURL *curl_;
CURLcode res_;
std::string read_buffer_;

////
/// \brief WriteCallback : callback function to get response from webserver using cURL
///
static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
  ((std::string*)userp)->append((char*)contents, size * nmemb);
  return size * nmemb;
}

bool KapturaCallback(hardware_driver::KapturaService::Request& req,hardware_driver::KapturaService::Response& res)
{
  ROS_INFO("Running kaptura frame with the speed = %f in number of degrees = %f", req.speed, req.degree);
  if(req.station_version.compare("201810"))
  {
    ROS_INFO("Running kaptura version 201810\n");
    float step=req.degree*165;
    float spd=req.speed*5.5;
    std::ostringstream os1,os2;
    os1 << step; os2<<spd;
    std::string command = "http://192.168.1.177/?setMotorSpeed="+os2.str()+"&setMotorRelSteps="+os1.str();
    const char* command_char = command.c_str();
    curl_easy_setopt(curl_, CURLOPT_URL,command_char);
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION,WriteCallback);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &read_buffer_);
    res_ = curl_easy_perform(curl_);
    read_buffer_.clear();
  }
  if(req.station_version.compare("201712"))
  {
    ROS_INFO("Running kaptura version 201712\n");
    std::ostringstream os1,os2;
    os1 << req.speed;
    os2 <<req.degree;
    std::string command = "http://raspberrypi:8080/?AngularSpeed=" + os1.str();
    const char* command_char_speed = command.c_str();
    curl_easy_setopt(curl_, CURLOPT_URL,command_char_speed);
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION,WriteCallback);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &read_buffer_);
    res_ = curl_easy_perform(curl_);
    read_buffer_.clear();

    command = "http://raspberrypi:8080/?RotateDegRel=" + os2.str();
    const char* command_char_rotate = command.c_str();
    curl_easy_setopt(curl_, CURLOPT_URL,command_char_rotate);
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION,WriteCallback);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &read_buffer_);
    res_ = curl_easy_perform(curl_);
    read_buffer_.clear();
  }
  else
  {
    ROS_INFO("Could not detect kaptura frame station\n");
  }
  ROS_INFO("Kaptura frame is running");
  res.success = true;


}
//void emergencyCallback();



int main(int argc, char **argv)
{
  curl_ = curl_easy_init();
  ros::init(argc, argv,"kaptura");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("kaptura_service",KapturaCallback);

  ROS_INFO("Ready to run kaptura frame. \n");
  ros::spin();

  curl_easy_cleanup(curl_);


  return 0;

}
