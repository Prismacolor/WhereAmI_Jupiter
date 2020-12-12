#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

//ROS:: Publisher motor commands
ros::Publisher motor_command_publisher;

//create handle drive request callback function that executes whenever drivebot service is requested
// function should publish requested linear and x angular velocities to robot wheel joints
// after publishing velocities, meassage feedback is returned with requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
  ROS_INFO("DriveToTarget request received = linear_vel:%1.2f, angular_vel:%1.2f", (float)req.linear_x, (float)req.angular_z);

  // create a motor_command object of type geometry_msgs::Twist
  geometry_msgs::Twist motor_command;
  // set wheel velocities forward [0.5, 0.0]
  motor_command.linear.x = req.linear_x;
  motor_command.angular.z = req.angular_z;
  // publish angles to drive the robot
  motor_command_publisher.publish(motor_command);

  // return the response
  res.msg_feedback= "linear x: " + std::to_string(motor_command.linear.x) + ", angular z: " + std::to_string(motor_command.angular.z);

  return true;
}


int main(int argc, char** argv)
{
  // initialize ROS node
  ros::init(argc, argv, "drive_bot");

  // create a handler
  ros::NodeHandle n;

  // inform ros master that we're publishing a message of geometry type to robot actuation topic with a message queue of ten. 
  motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // define a drive /ball chaser/ command robot service with handle_drive_request callback function
  ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
  ROS_INFO("Ready to send velocities");


  // handle ros communication events
  ros::spin();

  return 0;

}


