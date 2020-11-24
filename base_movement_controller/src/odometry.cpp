#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_broadcaster.h"
#include "ros/ros.h"
#include "math.h"
#include <iostream>

double delta_x = 0;
double delta_y = 0;
double delta_a1 = 0;
double delta_a2 = 0;
double fi = 0;
double current_fi = 0;
double current_a1, current_a2;
double last_a1 = 0;
double last_a2 = 0;


void OdomCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // Getting the positions of right and left back wheels from /joint_states topic
  current_a1 = -msg->position[0]; //left wheel, this value is minus, so change it to plus by add -
  current_a2 = msg->position[1]; //right wheel
  //std::cout << "current_a1 : "<< current_a1 << '\n';
  //std::cout << "current_a1 : "<< current_a1 << '\n';

}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "odom_publisher");

    ros::NodeHandle n;
    ros::Publisher odom_pub;
    ros::Subscriber odom_sub;

    odom_sub = n.subscribe("/joint_states",1000,OdomCallback);

    odom_pub = n.advertise<nav_msgs::Odometry>("odom",100);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Rate r(10);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    double R = 0.033; //wheel radius
    double d = 0.16;  //distance between two wheels
    double x = 0;     //initial values
    double y = 0;
    double th = 0;
    double vx, vth;
    double w_r, w_l;

    int count = 0;
    while(n.ok())
    {
      ros::spinOnce();               // check for incoming messages
      current_time = ros::Time::now();

      delta_a1 = current_a1 - last_a1;
      delta_a2 = current_a2 - last_a2;

      last_a1 = current_a1;
      last_a2 = current_a2;

      //ROS_INFO("delta_a %f", delta_a);
      double dt = (current_time - last_time).toSec();

      w_r = delta_a1/dt;
      w_l = delta_a2/dt;
      //std::cout << "w_r : "<< w_r << '\n';
      //std::cout << "w_l : "<< w_l << '\n';
      vx = R*(w_r + w_l)/2.0;

      current_fi = R*(delta_a2 - delta_a1)/d;
      fi +=current_fi;
      ROS_INFO("yaw %f", fi);

      delta_x = vx*cos(fi)*dt;
      delta_y = vx*sin(fi)*dt;

      std::cout << "vx : "<< vx << '\n';
      std::cout << "fi : "<< fi << '\n';

      x += delta_x;
      y += delta_y;
      th = fi;
      vth = current_fi/dt;

       //first, we'll publish the transform over tf
       geometry_msgs::TransformStamped odom_trans;
       odom_trans.header.stamp = current_time;
       odom_trans.header.frame_id = "odom";
       odom_trans.child_frame_id = "base_link";

       odom_trans.transform.translation.x = 0.0;
       odom_trans.transform.translation.y = 0.0;
       odom_trans.transform.translation.z = 0.0;
       odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.0);
       //send the transform
       odom_broadcaster.sendTransform(odom_trans);

       //next, we'll publish the odometry message over ROS
       nav_msgs::Odometry odom;
       odom.header.stamp = current_time;
       odom.header.frame_id = "odom";
       odom.child_frame_id = "base_link";

       //set the position
       odom.pose.pose.position.x = x;
       odom.pose.pose.position.y = y;
       odom.pose.pose.position.z = 0.0;
       odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);

       //set the velocity
       odom.twist.twist.linear.x = vx;
       odom.twist.twist.linear.y = 0.0;
       odom.twist.twist.linear.z = 0.0;
       odom.twist.twist.angular.z = vth;

       //publish the message
       odom_pub.publish(odom);

       last_time = current_time;
       r.sleep();
       count++;
    }

 return 0;
}
