#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include <string>

ros::Publisher  arduino_pub;
std::string ID ="NONE";
float ACC=1.0;

void mortor_callback(const std_msgs::Float32& float_msg){
	if(-1.0<float_msg.data*ACC && float_msg.data*ACC <1.0){
		int target=0x0100+float_msg.data*ACC*0x0100;
		std_msgs::String pub_data;
		std::string f1="#";
		std::string f2="M";
		std::string f4=";";
		char temp[6]={0};
		sprintf(temp,"%04X",target);
		std::string f3=temp;
		pub_data.data=f1+ID+f2+f3+f4;
		arduino_pub.publish(pub_data);
	}
}


int main(int argc, char **argv){
	ros::init(argc, argv, "pif_motor");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	pn.getParam("ID", ID);
	pn.getParam("ACC", ACC);
	
	//publish
	arduino_pub = n.advertise<std_msgs::String>("arduino_out", 1000);

	//subscriibe
	ros::Subscriber canin_sub = n.subscribe("motor", 10, mortor_callback); 
	
	ros::Rate loop_rate(20); 
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}


