#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

#include <string>


ros::Publisher servo_pub[4];
ros::Publisher motor_pub[4];

//temporal fixed
float ws_pos[4][2]={
	{ 0.13, 0.06},//FL
	{ 0.13,-0.06},//FR
	{-0.13, 0.06},//BL
	{-0.13,-0.06} //BR
};
float min_radius=0.3;
float max_radius=10.0;

void joy_callback(const sensor_msgs::Joy& joy_msg){
	if(-0.05<joy_msg.axes[0] && joy_msg.axes[0]<0.05){//straight
		std_msgs::Float32 ss;
		ss.data=0;
		std_msgs::Float32 ms;
		ms.data=0.5*joy_msg.axes[1];
		for(int i=0;i<4;i++){
			servo_pub[i].publish(ss);
			motor_pub[i].publish(ms);
		}
	}
	else if(joy_msg.axes[0]>0){//curve
		float center_y=min_radius/joy_msg.axes[0];
		for(int i=0;i<4;i++){
			std_msgs::Float32 sv;
			std_msgs::Float32 mv;
			sv.data=atan2(ws_pos[i][0],center_y-ws_pos[i][1]);
			mv.data=0.5*joy_msg.axes[1]*(center_y-ws_pos[i][1])/center_y;
			servo_pub[i].publish(sv);
			motor_pub[i].publish(mv);
		}	
	}
	else{
		float center_y=min_radius/joy_msg.axes[0];
		for(int i=0;i<4;i++){
			std_msgs::Float32 sv;
			std_msgs::Float32 mv;
			sv.data=-atan2(ws_pos[i][0],ws_pos[i][1]-center_y);
			mv.data=-0.5*joy_msg.axes[1]*(ws_pos[i][1]-center_y)/center_y;
			servo_pub[i].publish(sv);
			motor_pub[i].publish(mv);
		}
	}
/*
	std_msgs::Float32 s1;
	std_msgs::Float32 m1;
	s1.data=joy_msg.axes[0];
	m1.data=joy_msg.axes[1];
	servo0_pub.publish(s1);
	servo1_pub.publish(s1);
	servo2_pub.publish(s1);
	servo3_pub.publish(s1);
	motor0_pub.publish(m1);
	motor1_pub.publish(m1);
	motor2_pub.publish(m1);
	motor3_pub.publish(m1);
*/
}

int main(int argc, char **argv){
	ros::init(argc, argv, "joy_control");
	ros::NodeHandle n;

	//publish
	servo_pub[0] = n.advertise<std_msgs::Float32>("servo0", 1000);
	servo_pub[1] = n.advertise<std_msgs::Float32>("servo1", 1000);
	servo_pub[2] = n.advertise<std_msgs::Float32>("servo2", 1000);
	servo_pub[3] = n.advertise<std_msgs::Float32>("servo3", 1000);
	motor_pub[0] = n.advertise<std_msgs::Float32>("motor0", 1000);
	motor_pub[1] = n.advertise<std_msgs::Float32>("motor1", 1000);
	motor_pub[2] = n.advertise<std_msgs::Float32>("motor2", 1000);
	motor_pub[3] = n.advertise<std_msgs::Float32>("motor3", 1000);

	//subscriibe
	ros::Subscriber joy_sub   = n.subscribe("joy", 10, joy_callback); 

	ros::Rate loop_rate(20); 
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}


