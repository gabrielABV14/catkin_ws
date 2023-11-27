//this program toggles between rotation and translation
//commands,based on calls to a service.
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <message_tests/Changerate.h>
#include <message_tests/Changespeed.h>
 
 
bool forward = true;
double newfrequency;
bool ratechanged = false;
bool runStop = false;
double newSpeed = 1.0;
bool speedchanged = false;
 
 
bool toggleForward(
    std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &resp){
    forward = !forward;
    ROS_INFO_STREAM("Now sending "<<(forward?
                    "forward":"rotate")<< " commands.");
    return true;
}
 
bool changeRate(
    message_tests::Changerate::Request &req,
    message_tests::Changerate::Response &resp){
 
    ROS_INFO_STREAM("Changing rate to "<<req.newrate);
 
    newfrequency = req.newrate;    
    ratechanged = true;
    return true;
}
 
 
bool toggleStopRun(
    std_srvs::Empty::Request &req,
    std_srvs::Empty::Response &resp){
    runStop = !runStop;
    ROS_INFO_STREAM("Now sending "<<(runStop?
                    "run":"stop"));
    return true;
}
 
bool changeSpeed(
    message_tests::Changespeed::Request &req,
    message_tests::Changespeed::Response &resp){
 
    ROS_INFO_STREAM("Changing speed to "<<req.speed);
 
    newSpeed = req.speed;    
    speedchanged = true;
    return true;
}
 
int main(int argc, char **argv){
    ros::init(argc,argv,"pubvel_toggle_rate");
    ros::NodeHandle nh;
       
    ros::ServiceServer server = nh.advertiseService("toggle_forward",&toggleForward);
               
    ros::ServiceServer server0 = nh.advertiseService("change_rate",&changeRate);
 
    ros::ServiceServer server1 = nh.advertiseService("toogle_stop_run",&toggleStopRun);
 
    ros::ServiceServer server2 = nh.advertiseService("change_speed",&changeSpeed);
 
               
    ros::Publisher pub=nh.advertise<geometry_msgs::Twist>(  "turtle1/cmd_vel",1000);
   
    ros::Rate rate(2);
    while(ros::ok()){
                if(runStop){
                        geometry_msgs::Twist msg;
                        msg.linear.x = forward?newSpeed:0.0;
                        msg.angular.z=forward?0.0:1.0;
                pub.publish(msg);
                ros::spinOnce();
                        if(ratechanged) {
                            rate = ros::Rate(newfrequency);
                            ratechanged = false;
                        }
                        if(speedchanged){
                            speedchanged = false;
                        }
                        rate.sleep();
                }
                else{
                        geometry_msgs::Twist msg;
                        msg.linear.x = 0;
                        msg.angular.z= 0;
                        pub.publish(msg);
                        ros::spinOnce();
                        rate.sleep();
                }
    }
}