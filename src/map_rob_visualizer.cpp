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
  ros::init(argc, argv, "map_rob_visualizer");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
 
  //publish
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("map_rob_vis", 10);
    //subscribe
    ros::Subscriber sub_map_rob = n.subscribe("map_rob", 10, map_callback);
 
    // printf("%s", "EEEEEE");
  ros::Rate loop_rate(1);
    while (ros::ok()){
        //線の設定
        visualization_msgs::Marker points;
        points.header.frame_id="world";
        points.header.stamp=ros::Time::now();
        points.ns="maps";
        points.action=visualization_msgs::Marker::ADD;
        points.pose.orientation.w=1.0;
        points.id=2;
        points.type = visualization_msgs::Marker::LINE_STRIP;
        points.scale.x = 1;
        points.scale.y = 1;
        points.color.r = 1.0;
        points.color.a = 1.0;
 
        float map_points[2][2]={
            {0.1, 0.1},
            {0.2, 0.2},
        };
 
        //ここpにデータを突っ込むと、Rviz上にその点がマップされる使用
        // for(int i=0;i<(int)map_data.data.size()/2;i++){
        //     geometry_msgs::Point p;
        //     p.x=map_data.data[i*2+0];
        //     p.y=map_data.data[i*2+1];
        //     p.z=0.0;
        //     points.points.push_back(p);
        // }
 
        for(int i=0;i<2;i++){
            geometry_msgs::Point p;
            p.x=map_points[i][0];
            p.y=map_points[i][1];
            p.z=0.0;
            points.points.push_back(p);
        }

        marker_pub.publish(points);
 
        ros::spinOnce();
        loop_rate.sleep();
  }
   return 0;
}