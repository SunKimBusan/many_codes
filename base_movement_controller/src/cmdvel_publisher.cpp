#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmdvel_publisher");
  ros::NodeHandle n;

  ros::Publisher cmdvel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);

  geometry_msgs::Twist msg;
  msg.linear.x = 0.1;
  msg.angular.z = 0.2;
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    cmdvel_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
