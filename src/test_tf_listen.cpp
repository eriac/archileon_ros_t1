#include "ros/ros.h"
  
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"

void get_pose(float &pos_x, float &pos_y, float &rot_z){
    static tf::TransformListener tflistener;

    geometry_msgs::PoseStamped source_pose;
	source_pose.header.frame_id="base_link";
	source_pose.pose.orientation.w=1.0;
	geometry_msgs::PoseStamped target_pose;

    tflistener.waitForTransform("world", "base_link", ros::Time(0), ros::Duration(1.0));
    tflistener.transformPose("world",ros::Time(0),source_pose,"base_link",target_pose);


    double roll,pitch,yaw;
	tf::Quaternion btq(
        target_pose.pose.orientation.x,
        target_pose.pose.orientation.y,
        target_pose.pose.orientation.z,
        target_pose.pose.orientation.w
        );
	tf::Matrix3x3(btq).getRPY(roll, pitch, yaw);

    printf("P X:%f, Y:%f, Z:%f\n",target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z);
    printf("O R:%f, P:%f, Y:%f\n",roll,pitch,yaw);

    pos_x=target_pose.pose.position.x;
    pos_y=target_pose.pose.position.y;
    rot_z=yaw;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_tf_listen");
	ros::NodeHandle n;
	
    
	ros::Rate loop_rate(5); 
	while (ros::ok()){
		try{//sometime tf cause exeption (especially initial time)
            float v0,v1,v2;
            get_pose(v0,v1,v2);
		}
		catch(...){
			printf("error\n");
		}
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}
