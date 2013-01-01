/** 
* Code to get rotationaly iode to get rotationaly invariant april tag information
*/  
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>

ros::Publisher rectified_pose_pub;
geometry_msgs::PoseStamped tag_position_in_camera_frame;
geometry_msgs::PoseStamped camera_position_in_tag_frame;
void tag_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    double roll, pitch, yaw;
    // Get the tag position in the camera frame
    tag_position_in_camera_frame = *pose;
    // Get the camera position in the camera frame
    //tf::Quaternion Q = tf::Quaternion(tag_position_in_camera_frame.orientation.x,
    //	tag_position_in_camera_frame.orientation.y, tag_position_in_camera_frame.orientation.z,
    //	tag_position_in_camera_frame.orientation.w);
    tf::Matrix3x3 R = tf::Matrix3x3(); // Get the rotation matrix
    yaw = tag_position_in_camera_frame.pose.orientation.x;
    pitch = tag_position_in_camera_frame.pose.orientation.y;
    roll = tag_position_in_camera_frame.pose.orientation.z;
    //R.getRPY(roll,pitch,yaw);
    R.setEulerYPR(yaw, pitch, roll);
    R = R.inverse();
    //R.getRPY(roll,pitch,yaw);
    //ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f",
    //	roll*180/3.1415926, pitch*180/3.1415926, yaw*180/3.1415926);

    //FIX X coordinate to be right = positive with NEGATIVE
    camera_position_in_tag_frame.pose.position.x = -(R[0][0]*tag_position_in_camera_frame.pose.position.x +
        R[0][1]*tag_position_in_camera_frame.pose.position.y +
        R[0][2]*tag_position_in_camera_frame.pose.position.z) * 0.0254;

    camera_position_in_tag_frame.pose.position.y = (R[1][0]*tag_position_in_camera_frame.pose.position.x + 
        R[1][1]*tag_position_in_camera_frame.pose.position.y +
        R[1][2]*tag_position_in_camera_frame.pose.position.z) * 0.0254;
    camera_position_in_tag_frame.pose.position.z = (R[2][0]*tag_position_in_camera_frame.pose.position.x +
        R[2][1]*tag_position_in_camera_frame.pose.position.y +
        R[2][2]*tag_position_in_camera_frame.pose.position.z) * 0.0254;
    camera_position_in_tag_frame.pose.orientation.x = 0;
    camera_position_in_tag_frame.pose.orientation.y = 0;
    camera_position_in_tag_frame.pose.orientation.z = 0;
    camera_position_in_tag_frame.pose.orientation.w = 0;
    camera_position_in_tag_frame.header.stamp = tag_position_in_camera_frame.header.stamp;
    
    
    rectified_pose_pub.publish(camera_position_in_tag_frame);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rectified_april_pose");
  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  rectified_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("rectified_pose", 10); // Publisher of rectified pose
  
  ros::Subscriber tag_sub = nh.subscribe<geometry_msgs::PoseStamped>("april_pose", 10, tag_cb);   //changed to april_pose_drop,updates every 1Hz
  
  while (ros::ok()){
    ros::spinOnce();
  }

  return 0;
}
