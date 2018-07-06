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
#include <cmath>
 
std_msgs::Float32MultiArray map_data;
void map_callback(const std_msgs::Float32MultiArray& float_array){
    std_msgs::Float32MultiArray tmp(float_array);
    map_data=tmp;    
}
 


int main(int argc, char **argv){
  ros::init(argc, argv, "test_map");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
 
  float f = 0.0;
  //publish
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  //subscribe
//   ros::Subscriber sub_map_rob = n.subscribe("map_rob", 10, map_callback);
 
  // printf("%s", "EEEEEE");
  ros::Rate loop_rate(1);
    while (ros::ok()){
        //線の設定
        visualization_msgs::Marker points;
        points.header.frame_id="world";
        points.header.stamp=ros::Time::now();
        points.ns="test_map";
        points.action=visualization_msgs::Marker::ADD;
        points.pose.orientation.w=1.0;
        points.id=0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.1;
        points.scale.y = 0.1;
        points.color.r = 1.0;
        points.color.a = 1.0;

  
        for (uint32_t i = 0; i < 100; ++i){
            float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
            float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

            geometry_msgs::Point p;
            p.x = (int32_t)i - 50;
            p.y = y;
            p.z = z;

            points.points.push_back(p);

        }

        marker_pub.publish(points);
 
        ros::spinOnce();
        loop_rate.sleep();
  }
   return 0;
}