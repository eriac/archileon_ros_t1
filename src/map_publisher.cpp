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

#define MAP_SIZE 15
float map_points[MAP_SIZE][2]={
	{0.0,0.0},
	{0.5,0.0},
	{1.0,0.0},
	{1.5,0.0},
	{2.0,0.0},

	{2.5,0.1},
	{2.9,0.5},
	{3.0,1.0},
	{2.9,1.5},
	{2.5,1.9},
	
	{2.0,2.0},
	{1.5,2.0},
	{1.0,2.0},
	{0.5,2.0},
	{0.0,2.0},
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_publisher");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	//publish
	ros::Publisher map_pub = n.advertise<std_msgs::Float32MultiArray>("map_data", 10,true);
	
	std_msgs::Float32MultiArray map_data;
	
	for(int i=0;i<MAP_SIZE;i++){
		map_data.data.push_back(map_points[i][0]);
		map_data.data.push_back(map_points[i][1]);
	}

	map_pub.publish(map_data);

	ros::Rate loop_rate(10); 
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}

