#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include <math.h>
#include <string>

//regulator
float min_radius = 0.3;
float max_radius = 10.0;
float curve_factor = 0.6; //really 1?
float max_speed = 0.5;    //really 0.05

float curve_value = 0.0;
float speed_value = 0.0;

float bl_servo_angle;
float br_servo_angle;

void curve_callback(const std_msgs::Float32 &float_msg)
{
  if (float_msg.data < -(1.0 / min_radius))
    curve_value = -(1.0 / min_radius);
  else if (float_msg.data > +(1.0 / min_radius))
    curve_value = +(1.0 / min_radius);
  else
    curve_value = float_msg.data;
}
void speed_callback(const std_msgs::Float32 &float_msg)
{
  if (float_msg.data < -max_speed)
    speed_value = -max_speed;
  else if (float_msg.data > +max_speed)
    speed_value = +max_speed;
  else
    speed_value = float_msg.data;
}

void bl_nozzle_callback(const std_msgs::Float32 &float_msg)
{
  bl_servo_angle = float_msg.data;
}

void br_nozzle_callback(const std_msgs::Float32 &float_msg)
{
  br_servo_angle = float_msg.data;
}

//temporal fixed
float ws_pos[6][2] = {
    {0.13, 0.0750},   //FL
    {0.13, -0.0750},  //FR
    {-0.13, 0.0750},  //BL
    {-0.13, -0.0750}, //BR
    {-0.235, 0.0750}, //BL_nozuru
    {-0.235, -0.0750} //BR_nozuru
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_driver");
  ros::NodeHandle n;

  //publish
  ros::Publisher servo_pub[6];
  ros::Publisher motor_pub[4];

  servo_pub[0] = n.advertise<std_msgs::Float32>("servo0", 1000);
  servo_pub[1] = n.advertise<std_msgs::Float32>("servo1", 1000);
  servo_pub[2] = n.advertise<std_msgs::Float32>("servo2", 1000);
  servo_pub[3] = n.advertise<std_msgs::Float32>("servo3", 1000);
  servo_pub[4] = n.advertise<std_msgs::Float32>("servo4", 1000);
  servo_pub[5] = n.advertise<std_msgs::Float32>("servo5", 1000);
  motor_pub[0] = n.advertise<std_msgs::Float32>("motor0", 1000);
  motor_pub[1] = n.advertise<std_msgs::Float32>("motor1", 1000);
  motor_pub[2] = n.advertise<std_msgs::Float32>("motor2", 1000);
  motor_pub[3] = n.advertise<std_msgs::Float32>("motor3", 1000);

  //subscriibe
  ros::Subscriber speed_sub = n.subscribe("move_speed", 10, speed_callback);
  ros::Subscriber curve_sub = n.subscribe("move_curve", 10, curve_callback);
  ros::Subscriber bl_nozzle_angle_sub = n.subscribe("bl_rot_tube_angle", 10, bl_nozzle_callback);
  ros::Subscriber br_nozzle_angle_sub = n.subscribe("br_rot_tube_angle", 10, br_nozzle_callback);

  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    float f_val0 = curve_value / curve_factor;
    float f_val1 = speed_value;
    float center_y = 1 / f_val0;
    float hypotense = sqrt(center_y * center_y + ws_pos[2][0] * ws_pos[2][0]);

    std_msgs::Float32 sv1;
    std_msgs::Float32 sv2;

    sv1.data = bl_servo_angle;
    sv2.data = br_servo_angle;
    servo_pub[4].publish(sv1);
    servo_pub[5].publish(sv2);

    if (-0.05 < f_val0 && f_val0 < 0.05)
    { //straight
      std_msgs::Float32 ss;
      ss.data = 0;
      std_msgs::Float32 ms;
      ms.data = f_val1 * 1 / 2;
      for (int i = 0; i < 4; i++)
      {
        servo_pub[i].publish(ss);
        motor_pub[i].publish(ms);
      }
    }
    //左回り
    else if (f_val0 > 0)
    { //curve
      for (int i = 0; i < 4; i++)
      {
        std_msgs::Float32 sv;
        std_msgs::Float32 mv;
        if (i % 2 == 0)
        {
          sv.data = atan2(ws_pos[i][0], center_y - ws_pos[i][1]);
          mv.data = f_val1 * (hypotense - ws_pos[i][1]) / hypotense * 1 / 2;
        }
        else
        {
          sv.data = atan2(ws_pos[i][0], center_y - ws_pos[i][1]);
          mv.data = -f_val1 * (ws_pos[i][1] - hypotense) / hypotense * 1 / 2;
        }
        servo_pub[i].publish(sv);
        motor_pub[i].publish(mv);
      }
    }
    //右回り
    else
    {
      for (int i = 0; i < 4; i++)
      {
        std_msgs::Float32 sv;
        std_msgs::Float32 mv;
        if (i % 2 == 0)
        {
          sv.data = -atan2(ws_pos[i][0], ws_pos[i][1] - center_y);
          mv.data = f_val1 * (hypotense - ws_pos[i][1]) / hypotense * 1 / 2;
        }
        else
        {
          sv.data = -atan2(ws_pos[i][0], ws_pos[i][1] - center_y);
          mv.data = -f_val1 * (ws_pos[i][1] - hypotense) / hypotense * 1 / 2;
        }
        servo_pub[i].publish(sv);
        motor_pub[i].publish(mv);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
