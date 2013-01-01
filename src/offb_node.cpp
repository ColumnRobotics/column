/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265

float yy;
float pp;
float rr;

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

geometry_msgs::Pose tf_OA;
geometry_msgs::Pose tf_1;
geometry_msgs::Pose t_pose;
geometry_msgs::Pose b_pose;
void transformer(const geometry_msgs::Pose tag_pose, const geometry_msgs::Pose body_pose)
{
  t_pose = tag_pose;//*
  b_pose = body_pose;
  
  tf_1.position.x = (t_pose.position.x+1)*0.0254;
  tf_1.position.y = (1-t_pose.position.y)*0.0254;
  tf_1.position.z = (-2-t_pose.position.z)*0.0254;

  float t_x = tf_1.position.x;
  float t_y = tf_1.position.y;
  float t_z = tf_1.position.z;

  yy = b_pose.orientation.x;  //yaw
  pp = b_pose.orientation.y;  //pitch
  rr = b_pose.orientation.z;  //roll

  tf_OA.position.x = (cos(yy)*cos(pp))*t_x + (cos(yy)*sin(pp)*sin(rr)-sin(yy)*cos(rr))*t_y;
  tf_OA.position.x = tf_OA.position.x + (cos(yy)*sin(pp)*cos(rr)+sin(yy)*sin(rr))*t_z + b_pose.position.x;
  tf_OA.position.y = (sin(yy)*cos(pp))*t_x + (sin(yy)*sin(pp)*sin(rr)+cos(yy)*cos(rr))*t_y;
  tf_OA.position.y = tf_OA.position.y + (sin(yy)*sin(pp)*cos(rr)-cos(yy)*sin(rr))*t_z + b_pose.position.y;
  tf_OA.position.z = (-sin(pp))*t_x + (cos(pp)*sin(rr))*t_y + (cos(pp)*cos(rr))*t_z + b_pose.position.z;
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
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/local", 10, position_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped zero_pose;
    zero_pose.pose.position.x = current_position.pose.position.x;
    zero_pose.pose.position.y = current_position.pose.position.y;
    zero_pose.pose.position.z = current_position.pose.position.z;

    geometry_msgs::PoseStamped ref_pose;
    ROS_INFO("X:%f, Y:%f, Z:%f", current_position.pose.position.x,
    	current_position.pose.position.y,
	current_position.pose.position.z);
    ref_pose.pose.position.x = current_position.pose.position.x + tag_position.position.x/100; //CHECK UNITs
    ref_pose.pose.position.y = current_position.pose.position.y + tag_position.position.y/100;
    ref_pose.pose.position.z = current_position.pose.position.z + tag_position.position.z/100;

    geometry_msgs::PoseStamped tf_f;
    transformer(tag_position, current_position.pose);
    tf_f.pose.position.x = tf_OA.position.x;
    tf_f.pose.position.y = tf_OA.position.y;
    tf_f.pose.position.z = tf_OA.position.z;
    ROS_INFO("X:%f, Y:%f, Z:%f", tf_f.pose.position.x,
	     tf_f.pose.position.y,
	     tf_f.pose.position.z);

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(ref_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
/*
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
		//ref_pose.pose.position.x = current_position.pose.position.x;
        	//ref_pose.pose.position.y = current_position.pose.position.y;
        	//ref_pose.pose.position.z = current_position.pose.position.z;
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
	if(!flag){
		ref_pose.pose.position.x = - 100;
    		ref_pose.pose.position.y = current_position.pose.position.y;
    		ref_pose.pose.position.z = current_position.pose.position.z;
		flag = true;
	}
*/
	 //zero_pose.pose.position.x = current_position.pose.position.x;
         //zero_pose.pose.position.y = current_position.pose.position.y;
         //zero_pose.pose.position.z = current_position.pose.position.z;

        ref_pose.pose.position.x = 100 + current_position.pose.position.x + tag_position.position.x/100; //CHECK UNITs
        ref_pose.pose.position.y = current_position.pose.position.y + tag_position.position.y/100;
        ref_pose.pose.position.z = current_position.pose.position.z + tag_position.position.z/100;
        local_pos_pub.publish(ref_pose);
/*	ROS_INFO("Ref-X:%f Ref-Y:%f Ref-Z:%f", ref_pose.pose.position.x, 
           ref_pose.pose.position.y, 
           ref_pose.pose.position.z);*/
	ROS_INFO("Tag-X:%f Tag-Y:%f Tag-Z:%f", tag_position.position.x,
	   tag_position.position.y,
	   tag_position.position.z);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
