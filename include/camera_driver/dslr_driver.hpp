//#ifndef DSLR_DRIVER_HPP
//#define DSLR_DRIVER_HPP
//#include <stdlib.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>
//#include <stdio.h>
//#include <stdarg.h>
//#include <string.h>
//#include <gphoto2/gphoto2-camera.h>
//#include <gphoto2/gphoto2.h>
//#include <gphoto2/gphoto2-port-log.h>
//#include <opencv2/opencv.hpp>
//#include <ros/ros.h>
//#include <std_msgs/UInt8MultiArray.h>
//#include <camera_driver/DSLRCameraService.h>
//static void errordumper(GPLogLevel level, const char *domain, const char *str,
//                        void *data) {
//  fprintf(stdout, "%s\n", str);
//}
//static void ctx_error_func (GPContext *context, const char *str, void *data)
//{
//  ROS_INFO("gphoto error: %s", str);
//}


//static void ctx_status_func (GPContext *context, const char *str, void *data)
//{
//  ROS_INFO("gphoto info: %s", str);
//}
//class DSLRCamera
//{
//public:

//  DSLRCamera(const ros::NodeHandle& nh;const cv::Mat& camera_matrix,const cv::Mat& dist_coeffs);
//  bool initCamera();
//  void closeCamera();
//  void saveImageToDisk(const std::string& image_name);
//  void saveImageCallback(camera_driver::DSLRCameraService::Request& req,
//                         camera_driver::DSLRCameraService::Response& res);
//private:
//  ros::NodeHandle nh_;
//  static int capture_to_memory(Camera *camera, GPContext *context, const char **ptr, unsigned long int *size);
//  Camera *camera_;
//  int retval_;
//  GPContext *context_;
//  cv::Mat camera_matrix_;
//  cv::Mat dist_coeffs_;
//};
//#endif // DSLR_DRIVER_HPP
