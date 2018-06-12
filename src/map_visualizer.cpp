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

std_msgs::Float32MultiArray map_data;
void map_callback(const std_msgs::Float32MultiArray& float_array){
    std_msgs::Float32MultiArray tmp(float_array);
    map_data=tmp;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_visualizer");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	//publish
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("map_vis", 10);
  //subscribe
  ros::Subscriber canin_sub = n.subscribe("map_data", 10, map_callback);


	ros::Rate loop_rate(100);
	while (ros::ok()){
        //線の設定
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id="world";
        line_strip.header.stamp=ros::Time::now();
        line_strip.ns="maps";
        line_strip.action=visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w=1.0;
        line_strip.id=1;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.01;
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;
        //ここpにデータを突っ込むと、Rviz上にその点がマップされる使用
        for(int i=0;i<(int)map_data.data.size()/2;i++){
            geometry_msgs::Point p;
            p.x=map_data.data[i*2+0];
            p.y=map_data.data[i*2+1];
            p.z=0.0;
            line_strip.points.push_back(p);
        }
        marker_pub.publish(line_strip);

		ros::spinOnce();
		loop_rate.sleep();
	}
 	return 0;
}
