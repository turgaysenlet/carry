#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Vector3.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <motor_controller/speed_steering.h>
#include <motor_controller/steering.h>
#include <motor_controller/speed.h>
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
const float rate = 5;
const float stop_counter_seconds = 15.0f;
int counter = 0;

class AutonomousControllerCls
{
public:
	AutonomousControllerCls();
	void Run();
	
private:
	void robotStateCallback(const robot_state::robot_state::ConstPtr& state);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void heartBeatStopCallback(const std_msgs::Bool::ConstPtr& stop);
	void joyDiagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag);
	void ChangeState(int new_state);
	void topViewImageCallback(const sensor_msgs::Image::ConstPtr& image_msg);
    void neurobotCallback(const geometry_msgs::Vector3::ConstPtr& vector_msg);
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
	ros::Subscriber image_sub_;
	int robot_state;
    ros::Subscriber neurobot_sub_;
	bool joy_ok;
};

AutonomousControllerCls::AutonomousControllerCls()
{
	joy_ok = false;
	force_next_ignition_ = false;
	//nh_.param("axis_linear", linear_, linear_);
	//nh_.param("axis_angular", angular_, angular_);
	//nh_.param("scale_angular", a_scale_, a_scale_);
	//nh_.param("scale_linear", l_scale_, l_scale_);
	speech_pub_ = nh_.advertise <speech_engine::speech_request> ("speech_engine/speech_request", 1);
	heart_beat_stop_sub_ = nh_.subscribe <std_msgs::Bool> ("heart_beat_stop", 3, &AutonomousControllerCls::heartBeatStopCallback, this);
	joy_sub_ = nh_.subscribe < sensor_msgs::Joy> ("joy", 3, &AutonomousControllerCls::joyCallback, this);
	joy_diag_sub_ = nh_.subscribe < diagnostic_msgs::DiagnosticArray> ("diagnostics", 3, &AutonomousControllerCls::joyDiagCallback, this);
	speed_steering_pub_ = nh_.advertise < motor_controller::speed_steering
			> ("motor_controller/speed_steering_control", 1);
	robot_state_sub_ = nh_.subscribe < robot_state::robot_state > ("robot_state/robot_state", 10, &AutonomousControllerCls::robotStateCallback, this);
	robot_state_change_request_pub_ = nh_.advertise < robot_state::robot_state> ("robot_state/robot_state_change_request", 1, false);
	image_sub_ = nh_.subscribe("/stereo/top_view", 1, &AutonomousControllerCls::topViewImageCallback, this);
	neurobot_sub_ = nh_.subscribe <geometry_msgs::Vector3> ("neurobot", 3, &AutonomousControllerCls::neurobotCallback, this);
}


float steering = 0.5f;
float head = 0.5f;						
float speed = 0.3f;

void AutonomousControllerCls::neurobotCallback(
	const geometry_msgs::Vector3::ConstPtr& vector_msg)
{
  steering = vector_msg->x;
  head = vector_msg->y;
  speed = vector_msg->z;
}

void AutonomousControllerCls::topViewImageCallback(
	const sensor_msgs::Image::ConstPtr& image_msg)
{
	ROS_INFO("Image received.");		
	//cv_bridge::CvImageConstPtr cv_ptr_disp = cv_bridge::toCvShare(image_msg->image, image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
	cv_bridge::CvImageConstPtr cv_ptr_top = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
	const cv::Mat top_view_image = cv_ptr_top->image;	

	/*for (int j = 0; j < top_view_image.rows; j++)
	{		
		float jj = (j - top_view_image.rows / 2.0f) / (top_view_image.rows / 2.0f);
		for (int i = 0; i < top_view_image.cols; i++)
		{
			if (disp_image.at<float>(j, i) / jj > PLANE_RATIO)
			{
				float d = 1.0f / disp_image.at<float>(j, i);
				depth_image.at<float>(j, i) = d;
				int y = (int)(d*dk);
				if (y < 0) y = 0;
				if (y > top_view_image.cols - 1) y = top_view_image.cols - 1;
				int x = (int)((depth_image.cols / 2 - i) / xk + top_view_image.cols / 2);
				if (x < 0) x = 0;
				if (x > top_view_image.cols - 1) x = top_view_image.cols - 1;
				
				top_view_image.at<float>(y, x) += 0.2f;
			} 
			else 
			{
				depth_image.at<float>(j, i) = 0;
			}
		}
	}*/
}

void AutonomousControllerCls::heartBeatStopCallback(const std_msgs::Bool::ConstPtr& stop)
{
	/*if (robot_state != robot_state::robot_state_constants::RobotState_Stop)
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
	}*/
}
void AutonomousControllerCls::robotStateCallback(const robot_state::robot_state::ConstPtr& state)
{
	ChangeState((int)((int)state->state));
}


void AutonomousControllerCls::ChangeState(int new_state)
{
	robot_state = new_state;
	if (robot_state == robot_state::robot_state_constants::RobotState_Autonomous)
	{
		counter = 0;
	}
}

void AutonomousControllerCls::joyDiagCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& diag)
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

void AutonomousControllerCls::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	
}

void AutonomousControllerCls::Run() 
{
	ros::Rate loop_rate(rate);
	while (ros::ok())
	{			
		//ROS_INFO("State: %d", robot_state);
		if (robot_state == robot_state::robot_state_constants::RobotState_Autonomous)
		{
			counter++;
			if (counter > stop_counter_seconds * rate)
			{
				robot_state::robot_state stop_message;
				stop_message.state = robot_state::robot_state_constants::RobotState_Stop;
				robot_state_change_request_pub_.publish(stop_message);
			}
//			float steering = 0.5f;
			float steering_degree = steering * MaximumSteeringAngle;
//			float head = 0.5f;						
			float head_degree = head * MaximumHeadAngle;

//			float speed = 0.3f;
			float speed_mps = speed * MaximumForwardSpeed;

			motor_controller::speed_steering speed_steering_message;

			speed_steering_message.speed_mps.speed_mps = speed_mps;
			speed_steering_message.steering_degree.degree = steering_degree;
			speed_steering_message.head_degree.degree = head_degree;

			speed_steering_pub_.publish(speed_steering_message);
			ROS_INFO("Joy OK: %d, Speed: %f, Steering %f, Head %f", joy_ok, speed_mps, steering_degree, head_degree);
		}		
		loop_rate.sleep();
		ros::spinOnce();
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "AutonomousController");
	ROS_INFO("Autonomous Controller");
	AutonomousControllerCls autonomous_controller;
	autonomous_controller.Run();			
}

