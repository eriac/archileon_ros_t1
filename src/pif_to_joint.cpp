#include "ros/ros.h"

#include "std_msgs/Float32.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_broadcaster.h>

#include "math.h"
#include <sstream>
#include <string>

#define PS3_Button_Max 17
#define PS3_Select 0
#define PS3_L3     1
#define PS3_R3     2
#define PS3_Start  3
#define PS3_Up     4
#define PS3_Right  5
#define PS3_Down   6
#define PS3_Left   7
#define PS3_L2     8
#define PS3_R2     9
#define PS3_L1    10
#define PS3_R1    11
#define PS3_triangle 12
#define PS3_circle   13
#define PS3_cross    14
#define PS3_square   15
#define PS3_PS    16

float target_width[2]={0.12,0.12};
float WHEEL_BASE =0.26;
float target_steer[4]={0,0,0,0};
float target_motor[4]={0,0,0,0};
void width0_callback(const std_msgs::Float32& float_msg){
	target_width[0]=float_msg.data;
}
void width1_callback(const std_msgs::Float32& float_msg){
	target_width[1]=float_msg.data;
}
void steer0_callback(const std_msgs::Float32& float_msg){
	target_steer[0]=float_msg.data;
}
void steer1_callback(const std_msgs::Float32& float_msg){
	target_steer[1]=float_msg.data;
}
void steer2_callback(const std_msgs::Float32& float_msg){
	target_steer[2]=float_msg.data;
}
void steer3_callback(const std_msgs::Float32& float_msg){
	target_steer[3]=float_msg.data;
}
void motor0_callback(const std_msgs::Float32& float_msg){
	target_motor[0]=float_msg.data;
}
void motor1_callback(const std_msgs::Float32& float_msg){
	target_motor[1]=float_msg.data;
}
void motor2_callback(const std_msgs::Float32& float_msg){
	target_motor[2]=float_msg.data;
}
void motor3_callback(const std_msgs::Float32& float_msg){
	target_motor[3]=float_msg.data;
}

ros::Publisher joint_pub;
void jp(){
	sensor_msgs::JointState js0;
	js0.header.stamp = ros::Time::now();
	js0.name.resize(14);
	js0.name[ 0]="slider0_joint";
	js0.name[ 1]="slider1_joint";
	js0.name[ 2]="wheelset0_holder1_joint";
	js0.name[ 3]="wheelset0_holder2_joint";
	js0.name[ 4]="wheelset0_wheel_joint";
	js0.name[ 5]="wheelset1_holder1_joint";
	js0.name[ 6]="wheelset1_holder2_joint";
	js0.name[ 7]="wheelset1_wheel_joint";
	js0.name[ 8]="wheelset2_holder1_joint";
	js0.name[ 9]="wheelset2_holder2_joint";
	js0.name[10]="wheelset2_wheel_joint";
	js0.name[11]="wheelset3_holder1_joint";
	js0.name[12]="wheelset3_holder2_joint";
	js0.name[13]="wheelset3_wheel_joint";
	
	js0.position.resize(14);
	js0.position[ 0]= WHEEL_BASE/2;
	js0.position[ 1]=-WHEEL_BASE/2;
	js0.position[ 2]= target_width[0]/2;
	js0.position[ 3]= target_steer[0];
	js0.position[ 4]= 0;
	js0.position[ 5]=-target_width[0]/2;
	js0.position[ 6]= target_steer[1];
	js0.position[ 7]= 0;
	js0.position[ 8]=-target_width[1]/2;
	js0.position[ 9]= target_steer[2];
	js0.position[10]= 0;
	js0.position[11]= target_width[1]/2;
	js0.position[12]= target_steer[3];
	js0.position[13]= 0;
	
	joint_pub.publish(js0);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rough_sim");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	pn.getParam("WHEEL_BASE", WHEEL_BASE);
	
	//publish
	joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);

	//subscribe
	ros::Subscriber width0_sub   = n.subscribe("width0", 10, width0_callback); 
	ros::Subscriber width1_sub   = n.subscribe("width1", 10, width1_callback); 
	ros::Subscriber steer0_sub   = n.subscribe("steer0", 10, steer0_callback); 
	ros::Subscriber steer1_sub   = n.subscribe("steer1", 10, steer1_callback); 
	ros::Subscriber steer2_sub   = n.subscribe("steer2", 10, steer2_callback); 
	ros::Subscriber steer3_sub   = n.subscribe("steer3", 10, steer3_callback); 
	ros::Subscriber motor0_sub   = n.subscribe("motor0", 10, motor0_callback); 
	ros::Subscriber motor1_sub   = n.subscribe("motor1", 10, motor1_callback); 
	ros::Subscriber motor2_sub   = n.subscribe("motor2", 10, motor2_callback); 
	ros::Subscriber motor3_sub   = n.subscribe("motor3", 10, motor3_callback); 
			
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		jp();
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

