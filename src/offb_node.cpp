/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

bool flag = false;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_position;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    current_position = *pose;
}

geometry_msgs::Pose tag_position;
void tag_cb(const geometry_msgs::Pose::ConstPtr& pose){
    tag_position = *pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber tag_sub = nh.subscribe<geometry_msgs::Pose>
            ("april_pose", 10, tag_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Publisher set_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/local", 10, position_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);
    int publish_skip = 5*20; //Publish only every 2 seconds
    int publish_idx = 0;
    
    float avg_april_pose_x = 0.0; 
    float avg_april_pose_y = 0.0;

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        
        ros::spinOnce();
        rate.sleep();
    }
    
    geometry_msgs::TwistStamped twist_zero;
    twist_zero.header.stamp = ros::Time::now();
    twist_zero.header.frame_id = "fcu";
    twist_zero.twist.linear.x = 0.0;
    twist_zero.twist.linear.y = 0.0;
    twist_zero.twist.linear.z = 0.0;
    twist_zero.twist.angular.x = 0.0;
    twist_zero.twist.angular.y = 0.0;
    twist_zero.twist.angular.z = 0.0;

    geometry_msgs::TwistStamped twist_x = twist_zero;
    twist_x.twist.linear.x = 0.5;
    
    geometry_msgs::TwistStamped twist_y = twist_zero;
    twist_y.twist.linear.y = 0.5;

    geometry_msgs::TwistStamped twist_pub;

    //send a few setpoints before starting
    while(current_state.mode != "OFFBOARD" && ros::ok()){
        set_vel_pub.publish(twist_zero);
        ros::spinOnce();
        rate.sleep();
    }
    float kp = 1.0;
    geometry_msgs::PoseStamped des_position = current_position;
//    des_position.pose.position.x += 0.5;
    ros::Time time_begin = ros::Time::now();
    while(ros::ok()){
      float time = (ros::Time::now()-time_begin).toSec();
      
      if(current_state.mode == "OFFBOARD"){
	if(time < 0){
	  twist_pub = twist_zero;
        }
	else if(time < 1){
	  twist_pub = twist_x; //was x
	}
	//else if(time < 2){
	//  twist_pub = twist_zero;
	//}
	else if(time < 3){
	  twist_pub = twist_x;
	}
      }
      twist_pub = twist_zero;
      twist_pub.twist.linear.x = kp*(des_position.pose.position.x - current_position.pose.position.x);
      twist_pub.twist.linear.y = kp*(des_position.pose.position.y - current_position.pose.position.y);
      twist_pub.twist.linear.z = kp*(des_position.pose.position.z - current_position.pose.position.z);

      set_vel_pub.publish(twist_pub);
      ROS_INFO("secs: %f vx:%f vy:%f vz:%f", time, twist_pub.twist.linear.x, twist_pub.twist.linear.y, twist_pub.twist.linear.z);
      ros::spinOnce();
      rate.sleep();
    }

    return 0;
}
