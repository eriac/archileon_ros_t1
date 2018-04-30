#include "ros/ros.h"

#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>

#include "math.h"
#include <sstream>
#include <string>
#include <vector>

bool array_enable=false;
std::vector<float> array_x;
std::vector<float> array_y;
int array_pos=0;
int array_size=0;
void map_callback(const std_msgs::Float32MultiArray& float_array){
    array_enable=true;
    array_x.clear();
    array_y.clear();
    array_pos=0;
    array_size=(int)float_array.data.size()/2;
    for(int i=0;i<array_size;i++){
        array_x.push_back(float_array.data[i*2+0]);
        array_y.push_back(float_array.data[i*2+1]);
    }
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_driver");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");	
	//publish
	//ros::Publisher marker_pub = n.advertise<std_msgs::Marker>("map_vis", 10);
    //subscribe
    ros::Subscriber canin_sub = n.subscribe("map_data", 10, map_callback);

	ros::Rate loop_rate(10);
	while (ros::ok()){
        if(array_enable){

        }
		ros::spinOnce();
		loop_rate.sleep();
	}
 	return 0;
}
