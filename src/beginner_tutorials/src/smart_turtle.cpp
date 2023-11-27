#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  // For geometry_msgs::Twist
#include <stdlib.h> // For rand() and RAND_MAX
#include <turtlesim/Pose.h>
 
bool wall_detect = true;
int wall_detect_condition = 0;
float theta = 0.0;
void turtlePose_wall(const turtlesim::Pose& msg) {
  ROS_INFO_STREAM(std::setprecision(2) << std::fixed
    << "position=(" <<  msg.x << "," << msg.y << ")"
    << " direction=" << msg.theta);
    theta = msg.theta;
    if(msg.x <= 1 || msg.x >= 10 || msg.y <= 1 || msg.y >= 10){
      wall_detect = true;
      if (msg.y >= 10){
        wall_detect_condition = 1;
      }
      if(msg.x >= 10){
        wall_detect_condition = 2;
      }
    }
    else{
      wall_detect = false;
    }
}
 
int main(int argc, char **argv) {
  // Initialize the ROS system and become a node.
  ros::init(argc, argv, "smart_turtle");
  ros::NodeHandle nh;
 
  // Create a publisher object.
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
    "turtle1/cmd_vel", 1000);
  // Create a subscriber object.
  ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000,
    &turtlePose_wall);
 
  // Seed the random number generator.
  srand(time(0));
  // Loop at 2Hz until the node is shut down.
  ros::Rate rate(2);
  while(ros::ok()) {
    geometry_msgs::Twist msg;
    if(wall_detect){
       msg.angular.z = 2;
       msg.linear.x = 2;
    }
    else{
        msg.linear.x = double(rand())/double(RAND_MAX);
        msg.angular.z = 2*double(rand())/double(RAND_MAX) - 1;
    }
    // Publish the message.
    pub.publish(msg);
    ROS_INFO_STREAM("angulo"
      << " theta=" << theta
      << " condition=" << wall_detect_condition);
 
    rate.sleep();
    ros::spinOnce();
  }
}