#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// define a global client that can request services
ros::ServiceClient client;

// this function calls the command_robot service to drive robot in the right direction
void drive_robot(float lin_x, float ang_z)
{
  // todo: request a service and pass velociities to it to drive the robot
  ROS_INFO_STREAM("Moving robot towards ball.");

  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;
  
  if (client.call(srv)) {
    ROS_INFO_STREAM("DriveToTarget implemented successfully.");
  }
  else {
    ROS_INFO_STREAM("DriveToTarget service call failed.");
  }
}


// this callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
  int white_pixel = 255;
  bool is_pixel_white = false;
  int col;

  for(int i = 0; i < img.height*img.step; i++) {
    if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel) {
      is_pixel_white = true;
      col = int(i/3) % img.width;
      break;
    }
  }

  // depending on ball position, call drive_bot function and pass velocities to it
  if (is_pixel_white == true) {
    float linx;
    float angz;
    int pane_size = img.width/3;

    if (col < pane_size){
      linx = 0.0;
      angz = 0.5;
      ROS_INFO(" robot moves to the left.");
    }
    else if (col > pane_size && col < 2*pane_size){
      linx = 0.5;
      angz = 0.0;
      ROS_INFO(" robot moves forward.");
    }
    else if (col > 2*pane_size) {
      linx = 0.0;
      angz = -0.5;
      ROS_INFO(" robot moves to the right.");
    }
    drive_robot(linx, angz);
  }
  else {
    drive_robot(0.0, 0.0);
  }
}


int main(int argc, char** argv)
{
  // initialize
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // define a client service capable of requesting services from command_robot
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // subscribe to camera topic to read data image inside
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  ros::spin();

  return 0;
}
