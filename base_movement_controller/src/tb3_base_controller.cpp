#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <iostream>

#define ROBOT_WIDTH 0.16 // m
#define WHEEL_RADIUS 0.033 // m

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    jointcmd_pub = n.advertise<trajectory_msgs::JointTrajectory>("joint_cmd",1000);
    cmdvel_sub = n.subscribe("cmd_vel",1000,&SubscribeAndPublish::cmdvelCallback, this);
  }

  void cmdvelCallback(const geometry_msgs::Twist& msg)
  {
    float left_rpm = (msg.linear.x - msg.angular.z*ROBOT_WIDTH/2)/(2*M_PI*WHEEL_RADIUS);
    float right_rpm = (msg.linear.x + msg.angular.z*ROBOT_WIDTH/2)/(2*M_PI*WHEEL_RADIUS);


    trajectory_msgs::JointTrajectory traj;
    traj.header.stamp = ros::Time::now();
    traj.joint_names.resize(2);
    traj.joint_names[0] ="wheel1";
    traj.joint_names[1] ="wheel2";

    trajectory_msgs::JointTrajectoryPoint points_n;
    points_n.positions.push_back(0);
    points_n.positions.push_back(0);
    points_n.velocities.push_back(left_rpm);
    points_n.velocities.push_back(right_rpm);
    traj.points.push_back(points_n);
    traj.points[0].time_from_start = ros::Duration(1.0,0.0);

    jointcmd_pub.publish(traj);
  }

private:
  ros::NodeHandle n;
  ros::Publisher jointcmd_pub;
  ros::Subscriber cmdvel_sub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tb3_base_controller");
  SubscribeAndPublish SAPObject;
  ros::spin();

  return 0;

}
