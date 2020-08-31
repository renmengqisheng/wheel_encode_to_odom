
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <vector>

#define wheelRadius 0.0305  // meter 
#define wheelBase 0.24  // meter

using namespace std;

float elv = 0.0;  // rad/s
float erv = 0.0;  // rad/s

float convertValue2Velocity(const int32_t &value)
{
    float velocity = 0;
    const float RPM2RADPERSEC = 0.104719755f;
    const float RPM = 0.732f;
  
    if (value == 32768 || value == 0) velocity = 0.0f;
    else if (value > 0 && value < 32768) velocity = value * RPM * RPM2RADPERSEC;
    else if (value > 32768 && value < 65536) velocity = (value - 32768) * RPM * RPM2RADPERSEC  * (-1.0f);

    return velocity;  //rad/s
}


void DynamixelStateCallback(const dynamixel_workbench_msgs::DynamixelStateListConstPtr& msg)
{
  elv = convertValue2Velocity(msg->dynamixel_state[0].present_velocity);  // rad/s
  erv = convertValue2Velocity(msg->dynamixel_state[1].present_velocity) * (-1.0f);  // rad/s
}


int main(int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle node;
    ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Subscriber dynamixel_state_sub = node.subscribe("dynamixel_workbench/dynamixel_state", 1, DynamixelStateCallback);

    float x = 0.0;
    float y = 0.0;
    float th = 0.0;

    float w_l = 0.0;
    float w_r = 0.0;
    float v_l = 0.0;
    float v_r = 0.0;
    float v_k = 0.0;
    float w_k = 0.0;
    float delta_s = 0.0;
    float delta_th = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(100.0);
    while(node.ok()){
        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        // rotation velocity of each wheel
        float dt = (current_time - last_time).toSec();
        w_l = elv;
        w_r = erv;

        // linear velocity of each wheel
        v_l = w_l * wheelRadius;
        v_r = w_r * wheelRadius;

        // linear and angular velocity of the car
        v_k = (v_l + v_r) / 2; // m/s
        w_k = (v_r - v_l) / wheelBase; // rad/s

        // update new pose
        delta_s = v_k * dt;
        delta_th = w_k * dt;


        x += delta_s * cos(th + delta_th / 2);
        y += delta_s * sin(th + delta_th / 2);
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = v_k;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = w_k;

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
  }

  return 0;
}

