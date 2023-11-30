#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

ros::Publisher second_turtle_vel_pub;

void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  // Imprimir las velocidades recibidas
  ROS_INFO("Received linear velocity: %f, angular velocity: %f", msg->linear.x, msg->angular.z);

  // Publicar la velocidad opuesta en el topic "second_turtle/cmd_vel"
  geometry_msgs::Twist opposite_twist;
  opposite_twist.linear.x = -msg->linear.x;
  opposite_twist.angular.z = -msg->angular.z;

  // Imprimir las velocidades opuestas que se publican
  ROS_INFO("Publishing opposite linear velocity: %f, angular velocity: %f", opposite_twist.linear.x, opposite_twist.angular.z);

  second_turtle_vel_pub.publish(opposite_twist);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_second_turtle");
  ros::NodeHandle nh;

  // Spawning a new turtle
  ros::service::waitForService("/spawn");
  ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("/spawn");
  turtlesim::Spawn spawnSrv;
  spawnSrv.request.x = 5.0;  // Ajusta la posición inicial según tus necesidades
  spawnSrv.request.y = 5.0;
  spawnSrv.request.name = "second_turtle";
  spawnClient.call(spawnSrv);

  // Publisher de las velocidades opuestas para la segunda tortuga
  second_turtle_vel_pub = nh.advertise<geometry_msgs::Twist>("second_turtle/cmd_vel", 10);

  // Subscriber para el topic "turtle1/cmd_vel"
  ros::Subscriber teleop_sub = nh.subscribe("turtle1/cmd_vel", 10, teleopCallback);

  ros::spin();
  return 0;
}