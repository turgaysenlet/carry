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
#include <sensor_msgs/Imu.h>
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
const float MaximumSpeed = 100.0f;
/// <summary>
/// Turning speed where 255 is max motor speed ~4m/s.
/// </summary>
const float TurningSpeed = 60.0f;
class AndroidControllerCls {
 public:
  AndroidControllerCls();

 private:
  void joyCallback(const sensor_msgs::Imu::ConstPtr& joy);
  void Stop();
  ros::NodeHandle nh_;

  ros::Publisher motors_pub_;
  ros::Subscriber joy_sub_;
  ros::Publisher speech_pub_;
};

void AndroidControllerCls::Stop() {
  geometry_msgs::Vector3 motors_message;

  motors_message.x = 0;
  motors_message.y = 0;
  motors_message.z = 0;

  motors_pub_.publish(motors_message);
  ROS_INFO("Left: %d, Right %d", 0, 0);
}

AndroidControllerCls::AndroidControllerCls() {
  speech_pub_ = nh_.advertise<speech_engine::speech_request>(
      "speech_engine/speech_request", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Imu>(
      "imu", 1, &AndroidControllerCls::joyCallback, this);
  motors_pub_ = nh_.advertise<geometry_msgs::Vector3>("motors", 1);
}

float DegreeToRadian(float degree) { return degree / 57.295779524f; }

void AndroidControllerCls::joyCallback(const sensor_msgs::Imu::ConstPtr& joy) {
  float steering = joy->linear_acceleration.x / 5.0f;
  ROS_INFO("steering: %f", steering);
  float steering_degree = steering * MaximumSteeringAngle * 0;

  float positive_speed = (joy->linear_acceleration.y - 7.0f) / 2.0f;
  // Map from [1,-1] to [0,1]
  int speed = MaximumSpeed * positive_speed;
  ROS_INFO("positive_speed: %f, speed: %d", positive_speed, speed);

  geometry_msgs::Vector3 motors_message;

  float steering_radian = DegreeToRadian(steering_degree);
  motors_message.x = -sin(steering_radian) * TurningSpeed + speed;
  motors_message.y = sin(steering_radian) * TurningSpeed + speed;
  motors_message.z = 0;

  motors_pub_.publish(motors_message);
  ROS_INFO("Left: %d, Right %d", (int)motors_message.x, (int)motors_message.y);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "AndroidController");
  AndroidControllerCls android_controller;
  ros::spin();
}
