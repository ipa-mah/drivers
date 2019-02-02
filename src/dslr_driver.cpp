#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstdlib>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <gphoto2/gphoto2-camera.h>
#include <gphoto2/gphoto2.h>
#include <gphoto2/gphoto2-port-log.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <hardware_driver/DSLRCameraService.h>
Camera  *camera_;
GPContext *context_;
cv::Mat cameraMatrix, distCoeffs;

static void ctx_error_func (GPContext *context, const char *str, void *data)
{
  ROS_INFO("gphoto error: %s", str);
}


static void ctx_status_func (GPContext *context, const char *str, void *data)
{
  ROS_INFO("gphoto info: %s", str);
}
size_t RunUnixCommandStoreOutput(char *param, char *result, int resultdim)
{
  //	runs an Unix command/program and returns the output into the result variable
  FILE *fp;
  int status;
  fp = popen(param, "r");
  size_t byte_count = fread(result, 1, resultdim-1, fp);
  if(byte_count>0)
    *(result+byte_count-1)=0;
  else
    *(result)=0;
  return byte_count;
}
void killGvfsdProcess()
{
  char buffer[50], command[50];
  int result=RunUnixCommandStoreOutput("ps -A | grep gvfsd-gphoto2", buffer, 50);
  if(result > 0)
  {
    int spacesTrimAtBeginning=strcspn(buffer,"0123456789");
    for(int j=spacesTrimAtBeginning; j<result; j++)
    {
      buffer[j-spacesTrimAtBeginning]=buffer[j];
      if(buffer[j]==32)
      {
        buffer[j-spacesTrimAtBeginning]=0;
        j=result;
      }
    }
    strcpy(command, "kill -9 ");
    strcat(command,buffer);
    RunUnixCommandStoreOutput(command, buffer, 50);
  }
}
int capture_to_memory(Camera *camera, GPContext *context, const char **ptr, unsigned long int *size)
{

  int retval;
  CameraFile *file;
  CameraFilePath camera_file_path;

  printf("Capturing.\n");

  /* NOP: This gets overridden in the library to /capt0000.jpg */
  strcpy(camera_file_path.folder, "/");
  strcpy(camera_file_path.name, "foo.jpg");

  retval = gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);
  printf("  Retval: %d\n", retval);

  printf("Pathname on the camera: %s/%s\n", camera_file_path.folder, camera_file_path.name);

  retval = gp_file_new(&file);
  printf("  Retval: %d\n", retval);
  retval = gp_camera_file_get(camera, camera_file_path.folder, camera_file_path.name,
                              GP_FILE_TYPE_NORMAL, file, context);
  printf("  Retval: %d\n", retval);

  gp_file_get_data_and_size (file, ptr, size);

  printf("Deleting.\n");
  retval = gp_camera_file_delete(camera, camera_file_path.folder, camera_file_path.name,
                                 context);
  printf("  Retval: %d\n", retval);

  return 1;
}

void saveImageToDisk(const std::string& image_name)
{
  char *data;
  unsigned long size;
  std_msgs::UInt8MultiArray image_UInt8MultiArray;
  cv::Mat image;
  if(capture_to_memory(camera_, context_, (const char**)&data, &size) != 1)
  {
    ROS_ERROR("Error capturing image to memory!");
    return ;
  }
  std::vector<char> buff(data, data+size);
  image_UInt8MultiArray.data.clear();
  for(int i=0; i<buff.size(); i++)
  {
    image_UInt8MultiArray.data.push_back(buff[i]);
  }
  buff.clear();
  if(image_UInt8MultiArray.data.size()>0)
  {
    image = cv::imdecode(cv::Mat(image_UInt8MultiArray.data), 1);
  }
  else
  {
    ROS_INFO("Buffer empty, image cannot be decoded, exit dslr service");
    abort();
  }

  cv::Mat view, rview, map1, map2;
  cv::Mat new_cameraMatrix=cameraMatrix;
  cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), new_cameraMatrix,
                              image.size(), CV_16SC2, map1, map2);

  view = image.clone();
  cv::remap(view, rview, map1, map2, cv::INTER_LINEAR);
  cv::imwrite(image_name,rview);
}

bool saveImageCallback(hardware_driver::DSLRCameraService::Request &req,
                       hardware_driver::DSLRCameraService::Response &res)
{
  ROS_INFO("save_DSLRImage_callback");
  saveImageToDisk(req.image_name);
  ROS_INFO("DSLRImage captured");
  res.success = true;
  res.image_path = "image saved in "+ req.image_name;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dslr_service");
  ros::NodeHandle n;

  std::string cam_file = "src/hardware_driver/files/out_camera_data.xml";
  cv::FileStorage fs(cam_file,cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    ROS_WARN("not found intrinsic file");
    abort();
  }
  fs["camera_matrix"]>>cameraMatrix;
  fs["distortion_coefficients"]>>distCoeffs;
  killGvfsdProcess();
  context_ = gp_context_new();
  gp_context_set_error_func (context_, ctx_error_func, NULL);
  gp_context_set_status_func (context_, ctx_status_func, NULL);
  gp_camera_new (&camera_);
  printf("DSLR Camera init.  Takes about 3 seconds.\n");
  int retval = gp_camera_init(camera_, context_);
  if (retval != GP_OK) {
    ROS_ERROR("  Retval of capture_to_file: %d\n", retval);
    exit(1);
  }
  ros::ServiceServer service = n.advertiseService("dslr_capture",saveImageCallback);
  ROS_INFO("Ready capture image.");
  ros::spin();
  gp_camera_exit(camera_, context_);
  return 0;
}
