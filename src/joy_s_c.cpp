#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Joy.h"

float curve_value=0.0;
float speed_value=0.0;
void joy_callback(const sensor_msgs::Joy& joy_msg){
		curve_value=joy_msg.axes[0];
		speed_value=joy_msg.axes[1];
}

int main(int argc, char **argv){
	ros::init(argc, argv, "joy_control");
	ros::NodeHandle n;

	//publish
	ros::Publisher curve_pub = n.advertise<std_msgs::Float32>("move_curve", 1000);
	ros::Publisher speed_pub = n.advertise<std_msgs::Float32>("move_speed", 1000);

	//subscriibe
	ros::Subscriber joy_sub   = n.subscribe("joy", 10, joy_callback); 

	ros::Rate loop_rate(10); 
	while (ros::ok()){
        std_msgs::Float32 curve0;
        curve0.data=curve_value;
        curve_pub.publish(curve0);
        std_msgs::Float32 speed0;
        speed0.data=speed_value;
        speed_pub.publish(speed0);
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}


