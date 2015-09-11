#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <motor_controller/speed.h>
#include <motor_controller/steering.h>
#include <motor_controller/speed_steering.h>
#include <motor_controller/ignition_control.h>
#include <robot_state/robot_state_constants.h>
#include <robot_state/robot_state.h>
#include <speech_engine/speech_request.h>
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <time.h>   // time calls
#include <ros/console.h>
#include <iostream>

using namespace std;

//const int JoyLinearAxisPositive = 9;
//const int JoyLinearAxisNegative = 8;
const int JoyLinearAxisPositive = 3;
const int JoyLinearAxisNegative = 1;
const int JoyRotationAxis = 0;
const int JoyHeadRotationAxis = 2;
const int JoyIgnitionButton = 1;
const int JoyIgnitionResetButton = 6;
const float MaximumHeadAngle = 90.0f;
const float MaximumSteeringAngle = 30.0f;
/// <summary>
/// Maximum forward travel speed in meters per second.
/// </summary>
const float MaximumForwardSpeed = 4.1f;
/// <summary>
/// Maximum reverse travel speed in meters per second.
/// </summary>
const float MaximumReverseSpeed = -4.1f;

class RemoteControllerCls
{
public:
	RemoteControllerCls();

private:
	void robotStateCallback(const robot_state::robot_state::ConstPtr& state);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void heartBeatStopCallback(const std_msgs::Bool::ConstPtr& stop);
	void joyDiagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag);
	void ChangeState(int new_state);
	ros::NodeHandle nh_;

	ros::Publisher ignition_control_pub_;
	ros::Publisher speed_steering_pub_;
	ros::Subscriber heart_beat_stop_sub_;
	ros::Subscriber joy_sub_;
	ros::Subscriber joy_diag_sub_;
	ros::Subscriber robot_state_sub_;
	ros::Publisher robot_state_change_request_pub_;
	ros::Publisher speech_pub_;
	bool force_next_ignition_;
	int robot_state;
	bool joy_ok;
};

RemoteControllerCls::RemoteControllerCls()
{
	joy_ok = false;
	force_next_ignition_ = false;
	//nh_.param("axis_linear", linear_, linear_);
	//nh_.param("axis_angular", angular_, angular_);
	//nh_.param("scale_angular", a_scale_, a_scale_);
	//nh_.param("scale_linear", l_scale_, l_scale_);
	speech_pub_ = nh_.advertise <speech_engine::speech_request> ("speech_engine/speech_request", 1);
	heart_beat_stop_sub_ = nh_.subscribe <std_msgs::Bool> ("heart_beat_stop", 3, &RemoteControllerCls::heartBeatStopCallback, this);
	joy_sub_ = nh_.subscribe < sensor_msgs::Joy> ("joy", 3, &RemoteControllerCls::joyCallback, this);
	joy_diag_sub_ = nh_.subscribe < diagnostic_msgs::DiagnosticArray> ("diagnostics", 3, &RemoteControllerCls::joyDiagCallback, this);
	speed_steering_pub_ = nh_.advertise < motor_controller::speed_steering
			> ("motor_controller/speed_steering_control", 1);
	ignition_control_pub_ = nh_.advertise < motor_controller::ignition_control
			> ("motor_controller/ignition_control", 1);
	robot_state_sub_ = nh_.subscribe < robot_state::robot_state > ("robot_state/robot_state", 10, &RemoteControllerCls::robotStateCallback, this);
	robot_state_change_request_pub_ = nh_.advertise < robot_state::robot_state> ("robot_state/robot_state_change_request", 1, false);
}
void RemoteControllerCls::heartBeatStopCallback(const std_msgs::Bool::ConstPtr& stop)
{
	if (robot_state != robot_state::robot_state_constants::RobotState_Stop)
	{
		if (stop->data == true)
		{
			ROS_WARN("Heart beat not received.");
			robot_state::robot_state stop_message;
			stop_message.state = robot_state::robot_state_constants::RobotState_Stop;
			robot_state_change_request_pub_.publish(stop_message);

			sleep(1);
			// Speak after state transition.
			speech_engine::speech_request speech_message;
			string speech_text = "Connection lost!";
			speech_message.speech_request = speech_text;
			speech_pub_.publish(speech_message);
		}
		else
		{
			ROS_WARN("No heart beat message received but value is false???.");
		}
	}
}
void RemoteControllerCls::robotStateCallback(const robot_state::robot_state::ConstPtr& state)
{
	ChangeState((int)((int)state->state));
}


void RemoteControllerCls::ChangeState(int new_state)
{
	robot_state = new_state;
}

void RemoteControllerCls::joyDiagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag)
{
	if (diag->status.size() > 0)
	{
		if (joy_ok && (diag->status[0].level != 0))
		{
			ROS_WARN("Joystick disconnected");
			robot_state::robot_state stop_message;
			stop_message.state = robot_state::robot_state_constants::RobotState_Stop;
			robot_state_change_request_pub_.publish(stop_message);
		}
		joy_ok = (diag->status[0].level == 0);
	}
	//ROS_INFO("Joy OK");
}

void RemoteControllerCls::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	//ROS_INFO("Joy callback OK: %d, Buttons: %d,%d,%d,%d", joy_ok, joy->buttons[0],
	//						joy->buttons[1], joy->buttons[2], joy->buttons[3], joy->buttons[4]);
	if (!joy_ok)
	{

	}
	else
	{
		if (robot_state == robot_state::robot_state_constants::RobotState_Remote)
		{
			if (joy->buttons[JoyIgnitionResetButton])
			{
				force_next_ignition_ = true;
			}
			else
			{
				if (joy->buttons[JoyIgnitionButton])
				{
					ROS_INFO("Ignition button");
					if (force_next_ignition_)
					{
						force_next_ignition_ = false;
						motor_controller::ignition_control ignition_message;
						ignition_message.initiate_ignition = 1;
						ignition_message.force = 1;
						ignition_control_pub_.publish(ignition_message);
					}
					else
					{
						motor_controller::ignition_control ignition_message;
						ignition_message.initiate_ignition = 1;
						ignition_message.force = 0;
						ignition_control_pub_.publish(ignition_message);
					}
				}
				else
				{
					if (joy_ok)
					{
						float steering = joy->axes[JoyRotationAxis];
						float steering_degree = steering * MaximumSteeringAngle;
						float head = joy->axes[JoyHeadRotationAxis];						
						float head_degree = steering * MaximumHeadAngle; // head * MaximumHeadAngle;

						float positive_speed = joy->axes[JoyLinearAxisPositive];
						// Map from [1,-1] to [0,1]
						positive_speed = (1 - positive_speed) / 2.0f;
						float negative_speed = joy->axes[JoyLinearAxisNegative];
						// Map from [1,-1] to [0,1]
						negative_speed = (1 - negative_speed) / 2.0f;
						float speed = positive_speed - negative_speed;
						float speed_mps = speed * MaximumForwardSpeed;

						motor_controller::speed_steering speed_steering_message;

						speed_steering_message.speed_mps.speed_mps = speed_mps;
						speed_steering_message.steering_degree.degree = steering_degree;
						speed_steering_message.head_degree.degree = head_degree;

						speed_steering_pub_.publish(speed_steering_message);
						ROS_INFO("Joy OK: %d, Speed: %f, Steering %f, Head %f", joy_ok, speed_mps, steering_degree, head_degree);
					}
				}
			}
		}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "RemoteController");
	RemoteControllerCls remote_controller;
	ros::spin();
}

