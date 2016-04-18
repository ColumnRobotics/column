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


ros::Subscriber odom_sub;
ros::Publisher pose_estimate_pub;
geometry_msgs::PoseStamped pose_estimate;
void position_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
    pose_estimate.header = odom->header;
    pose_estimate.pose = odom->pose.pose;
    pose_estimate_pub.publish(pose_estimate);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_send");
  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  odom_sub = nh.subscribe<nav_msgs/Odometry>("odom", 10, position_cb); // Publisher of rectified pose

  pose_estimate_pub = nh.advertise<geometry_msgs/PoseStamped>("mavros/vision_pose/pose", 10); // Publisher of rectified pose
  
  ros::spin();

  return 0;
}
