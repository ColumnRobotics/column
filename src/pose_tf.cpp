/*
     Code to publish transforms for vizualization of the /mavros/local_position/local topic
*/

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::PoseStamped curr_pose;
geometry_msgs::PoseStamped last_pose;
tf::Transform fcu;
bool flag = true;
nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_trans;
ros::Publisher odom_pub;

//ros::Time current_time, last_time;
double vx, vy, vz;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    curr_pose = *pose;
    //current_time = ros::Time::now(); // Get current time
    //double dt = (current_time - last_time).toSec(); // Get change in time
    //vx = (curr_pose.pose.position.x - last_pose.pose.position.x) / dt;
    //vy = (curr_pose.pose.position.y - last_pose.pose.position.y) / dt;
    //vz = (curr_pose.pose.position.z - last_pose.pose.position.z) / dt;
    if(flag) { // Should only run once
        static tf::TransformBroadcaster local_origin_br;
        tf::Transform local_origin;
        local_origin.setOrigin( tf::Vector3(curr_pose.pose.position.x, curr_pose.pose.position.y, curr_pose.pose.position.z) );
        tf::Quaternion q = tf::Quaternion(curr_pose.pose.orientation.x, curr_pose.pose.orientation.y,curr_pose.pose.orientation.z,curr_pose.pose.orientation.w);
        local_origin.setRotation(q);
        local_origin_br.sendTransform(tf::StampedTransform(local_origin, ros::Time::now(), "map", "local_origin"));
        flag = false;
    }

    static tf::TransformBroadcaster fcu_br;
    fcu.setOrigin( tf::Vector3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z) );
    tf::Quaternion q = tf::Quaternion(pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w);
    fcu.setRotation(q);
    fcu_br.sendTransform(tf::StampedTransform(fcu, ros::Time::now(), "local_origin", "fcu"));

    odom.pose.pose.position.x = pose->pose.position.x;
    odom.pose.pose.position.y = pose->pose.position.y;
    odom.pose.pose.position.z = pose->pose.position.z;
    odom.pose.pose.orientation = pose->pose.orientation;
    odom.child_frame_id = "fcu";
    odom.header.frame_id = "fcu";
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;
    odom_pub.publish(odom);


    //last_time = current_time;
    last_pose = curr_pose;
}

bool flag1 = true;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "copter_visualization");
  //  while(flag1){
    //    current_time = ros::Time::now();
      //  last_time = ros::Time::now();
       // flag1 = false;
   // }
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/local", 10, position_cb);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    std::cout << "Subscribed to mavros/local_position/local" << std::endl;

    ros::spin();
    return 0;
}

