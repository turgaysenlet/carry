#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
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

using namespace std;

//PS3 controller
const int JoyStopStateButtonNo = 15; // Square Button
const int JoyAutonomousStateButtonNo = 12; // Triangle Button
const int JoyRemoteStateButtonNo = 14; // X Button
const int JoyFollowSidewalkStateButtonNo = 13; // O Button

//const int JoyStopStateButtonNo = 2; // Button 3
//const int JoyAutonomousStateButtonNo = 3; // Button 4
//const int JoyRemoteStateButtonNo = 4; // Button 5
//const int JoyFollowSidewalkStateButtonNo = 5; // Button 6

string StateName(int state)
{
	string state_name = "";
	if (state == robot_state::robot_state_constants::RobotState_Stop)
		state_name = "Stop";
	else if (state == robot_state::robot_state_constants::RobotState_Remote)
		state_name = "Remote";
	else if (state == robot_state::robot_state_constants::RobotState_Autonomous)
		state_name = "Autonomous";
	else if (state == robot_state::robot_state_constants::RobotState_FollowSidewalk)
		state_name = "Follow Sidewalk";
	else
		state_name = "Unknown";
	return state_name;
}

class RobotStateCls
{
public:
	RobotStateCls();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void robotStateChangeRequestCallback(const robot_state::robot_state::ConstPtr& state_request);
	void ChangeState(int requested_state);

	ros::NodeHandle nh_;
	ros::Publisher robot_state_pub_;
	ros::Publisher speech_pub_;
	ros::Subscriber robot_state_change_request_sub_;
	ros::Subscriber joy_sub_;

public:
	int robot_state;
};

RobotStateCls::RobotStateCls()
{
	robot_state = robot_state::robot_state_constants::RobotState_Stop;
	//nh_.param("axis_linear", linear_, linear_);
	//nh_.param("axis_angular", angular_, angular_);
	//nh_.param("scale_angular", a_scale_, a_scale_);
	//nh_.param("scale_linear", l_scale_, l_scale_);
	speech_pub_ = nh_.advertise <speech_engine::speech_request> ("speech_engine/speech_request", 1);
	// Latch robot state
	robot_state_pub_ = nh_.advertise < robot_state::robot_state> ("robot_state/robot_state", 1, true);
	robot_state_change_request_sub_ = nh_.subscribe < robot_state::robot_state> ("robot_state/robot_state_change_request", 10, &RobotStateCls::robotStateChangeRequestCallback, this);
	joy_sub_ = nh_.subscribe <sensor_msgs::Joy> ("joy", 10, &RobotStateCls::joyCallback, this);
}
void RobotStateCls::ChangeState(int requested_state)
{
	if (requested_state != robot_state)
	{
		speech_engine::speech_request speech_message;

		string speech_text = StateName(requested_state);

		string info_text = "State changed from " + StateName(robot_state) +
						", to " + StateName(requested_state);

		// Change the state
		robot_state = requested_state;

		speech_message.speech_request = speech_text;
		speech_pub_.publish(speech_message);

		robot_state::robot_state state_message;
		state_message.state = robot_state;
		robot_state_pub_.publish(state_message);
		ROS_INFO(info_text.c_str());
	}
}
void RobotStateCls::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	if (joy->buttons[JoyStopStateButtonNo])
	{
		ChangeState(robot_state::robot_state_constants::RobotState_Stop);
	} else if (joy->buttons[JoyRemoteStateButtonNo])
	{
		ChangeState(robot_state::robot_state_constants::RobotState_Remote);
	} else if (joy->buttons[JoyAutonomousStateButtonNo])
	{
		ChangeState(robot_state::robot_state_constants::RobotState_Autonomous);
	} else if (joy->buttons[JoyFollowSidewalkStateButtonNo])
	{
		ChangeState(robot_state::robot_state_constants::RobotState_FollowSidewalk);
	}

}
void RobotStateCls::robotStateChangeRequestCallback(const robot_state::robot_state::ConstPtr& state_request)
{
	int requested_state = (int)((int)(state_request->state));

	ChangeState(requested_state);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "RobotStateCls");
	RobotStateCls robot_state;
	ros::spin();
}

