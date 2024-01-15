#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mybot_node");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  geometry_msgs::Twist msg;
  msg.linear.x = 1.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok()) {
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
