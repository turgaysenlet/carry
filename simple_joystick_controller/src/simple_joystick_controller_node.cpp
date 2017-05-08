#include <errno.h>  // Error number definitions
#include <fcntl.h>  // File control definitions
#include <geometry_msgs/Vector3.h>
#include <motor_controller/ignition_control.h>
#include <motor_controller/speed.h>
#include <motor_controller/speed_steering.h>
#include <motor_controller/steering.h>
#include <robot_state/robot_state.h>
#include <robot_state/robot_state_constants.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <speech_engine/speech_request.h>
#include <std_msgs/Bool.h>
#include <stdio.h>    // standard input / output functions
#include <string.h>   // string function definitions
#include <termios.h>  // POSIX terminal control definitions
#include <time.h>     // time calls
#include <unistd.h>   // UNIX standard function definitions
#include <iostream>

using namespace std;

const int JoyLinearAxisPositive = 1;  // X
const int JoyLinearAxisNegative = 3;  // Throttle
const int JoyRotationAxis = 0;        // Y
const float MaximumSteeringAngle = 80.0f;
/// <summary>
/// Maximum forward travel speed where 255 is max motor speed ~4m/s.
/// </summary>
const float MaximumSpeed = 120.0f;
/// <summary>
/// Turning speed where 255 is max motor speed ~4m/s.
/// </summary>
const float TurningSpeed = 60.0f;
class SimpleJoystickControllerCls {
 public:
  SimpleJoystickControllerCls();

 private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void Stop();
  ros::NodeHandle nh_;

  ros::Publisher motors_pub_;
  ros::Subscriber joy_sub_;
  ros::Publisher speech_pub_;
};

void SimpleJoystickControllerCls::Stop() {
  geometry_msgs::Vector3 motors_message;

  motors_message.x = 0;
  motors_message.y = 0;
  motors_message.z = 0;

  motors_pub_.publish(motors_message);
  ROS_INFO("Left: %d, Right %d", 0, 0);
}

SimpleJoystickControllerCls::SimpleJoystickControllerCls() {
  speech_pub_ = nh_.advertise<speech_engine::speech_request>(
      "speech_engine/speech_request", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(
      "joy", 1, &SimpleJoystickControllerCls::joyCallback, this);
  motors_pub_ = nh_.advertise<geometry_msgs::Vector3>("motors", 1);
}

float DegreeToRadian(float degree) { return degree / 57.295779524f; }

void SimpleJoystickControllerCls::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  float left = -joy->axes[13];  
  float right = -joy->axes[12];
  ROS_INFO("left: %f, right: %f", left, right);
  int left_speed = MaximumSpeed * left;
  int right_speed = MaximumSpeed * right;
  ROS_INFO("left_speed: %d, right_speed: %d", left_speed, right_speed);

  geometry_msgs::Vector3 motors_message;

  motors_message.x = left_speed;
  motors_message.y = right_speed;
  motors_message.z = 0;

  motors_pub_.publish(motors_message);
  ROS_INFO("Left: %d, Right %d", (int)motors_message.x, (int)motors_message.y);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "SimpleJoystickController");
  SimpleJoystickControllerCls simple_joystick_controller;
  ros::Rate rate(4.);
  while(ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
  }  
//ros::spin();
}
