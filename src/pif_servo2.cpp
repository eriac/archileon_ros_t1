#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include <string>

ros::Publisher  arduino_pub;
float REDUCTION =1.0;
std::string ID ="NONE";

void servo_callback(const std_msgs::Float32& float_msg){
	if(-1.3<float_msg.data && float_msg.data <1.3){
		int target=90+REDUCTION*float_msg.data/(2*3.14)*360;
		std_msgs::String pub_data;
		std::string f1="#";
		std::string f2="T";
		std::string f4=";";
		char temp[6]={0};
		sprintf(temp,"%04X",target);
		std::string f3=temp;
		pub_data.data=f1+ID+f2+f3+f4;
		arduino_pub.publish(pub_data);
	}
}


int main(int argc, char **argv){
	ros::init(argc, argv, "pif_servo2");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	pn.getParam("ID", ID);
	pn.getParam("REDUCTION", REDUCTION);	

	//publish
	arduino_pub = n.advertise<std_msgs::String>("arduino_out", 1000);

	//subscriibe
	ros::Subscriber canin_sub = n.subscribe("servo", 10, servo_callback); 
	
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}


