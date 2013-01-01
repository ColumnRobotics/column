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
tf::Transform fcu;
bool flag = true;
nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_trans;
ros::Publisher odom_pub;
geometry_msgs::Pose first_pose;
double old_yaw, old_pitch, old_roll;
double new_yaw, new_pitch, new_roll;
double d_yaw, d_pitch, d_roll;
tf::Matrix3x3 R;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    curr_pose = *pose;
    if(flag) { // Should only run once
        static tf::TransformBroadcaster local_origin_br;
        tf::Transform local_origin;
	    first_pose = curr_pose.pose;
        local_origin.setOrigin( tf::Vector3(curr_pose.pose.position.x, curr_pose.pose.position.y, 0) ); // Z should be zero
        tf::Quaternion q = tf::Quaternion(curr_pose.pose.orientation.x, curr_pose.pose.orientation.y,curr_pose.pose.orientation.z,curr_pose.pose.orientation.w);
        local_origin.setRotation(q);
        local_origin_br.sendTransform(tf::StampedTransform(local_origin, ros::Time::now(), "map", "first_pose"));
	    R = tf::Matrix3x3(q); // Get the rotation matrix
	    R.getEulerYPR(old_yaw, old_pitch, old_roll); 
        flag = false;
    }
    tf::Quaternion q_new = tf::Quaternion(curr_pose.pose.orientation.x, curr_pose.pose.orientation.y,curr_pose.pose.orientation.z,curr_pose.pose.orientation.w);

    R = tf::Matrix3x3(q_new); // Get the rotation matrix
    R.getEulerYPR(new_yaw, new_pitch, new_roll); // Get the Euler angles YPR
    d_yaw = new_yaw - old_yaw;
    d_pitch = new_pitch - old_pitch;
    d_roll = new_roll - old_roll;
    // Get the new quaternion

    static tf::TransformBroadcaster fcu_br;
    fcu.setOrigin( tf::Vector3(pose->pose.position.x - first_pose.position.x, pose->pose.position.y - first_pose.position.y, 0) ); // Z should be zero
    tf::Quaternion q = tf::Quaternion();
    q.setEuler(d_yaw, d_pitch, d_roll); // YPR 
    fcu.setRotation(q);
    fcu_br.sendTransform(tf::StampedTransform(fcu, ros::Time::now(), "map", "base_footprint"));

    odom.pose.pose.position.x = pose->pose.position.x;
    odom.pose.pose.position.y = pose->pose.position.y;
    odom.pose.pose.position.z = 0; // Z should be zero
    odom.pose.pose.orientation = pose->pose.orientation;
    odom.child_frame_id = "base_footprint";
    odom.header.frame_id = "map";
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;
    odom_pub.publish(odom);
}

bool flag1 = true;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "copter_visualization");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/local", 10, position_cb);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom22", 50);

    std::cout << "Subscribed to mavros/local_position/local" << std::endl;

    ros::spin();
    return 0;
}

