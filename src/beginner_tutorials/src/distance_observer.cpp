// This program subscribes to turtle1/pose and shows its
// messages on the screen.
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <iomanip> // for std::setprecision and std::fixed
#include "beginner_tutorials/Ej2_dis.h"
 
// A callback function.  Executed each time a new pose
// message arrives.
 
float x_1;
float y_1;
float x_2;
float y_2;
 
 
void turtlePose1(const turtlesim::Pose& msg) {

    x_1 = msg.x;
    y_1 = msg.y;
}
 
void turtlePose2(const turtlesim::Pose& msg) {

    x_2 = msg.x;
    y_2 = msg.y;
}
 
 
int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "Ej2_dis");
  ros::NodeHandle nh;
  // Create a subscriber object.
  ros::Subscriber sub1 = nh.subscribe("turtle1/pose", 1000,
    &turtlePose1);
  ros::Subscriber sub2 = nh.subscribe("turtle2/pose", 1000,
    &turtlePose2);
 
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::Ej2_dis>("Ej2_dis");
 
 
  beginner_tutorials::Ej2_dis srv;
 
 
  ros::Rate rate(0.2);
 
  while (ros::ok()) {
 
    ros::spinOnce();
 
    srv.request.x1 = x_1;
    srv.request.y1 = y_1;
    srv.request.x2 = x_2;
    srv.request.y2 = y_2;
 
    if (client.call(srv))
    {
      ROS_INFO("Ej2_dis: %f", srv.response.Ej2_dis);
    }
    else
    {
      ROS_ERROR("Error -_-)/");
      return 1;
    }
 
 
    rate.sleep();
  }
 
  return 0;
}
 