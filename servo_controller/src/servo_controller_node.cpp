#include <ros/ros.h>
#include <servo_controller/servo_control.h>
#include <servo_controller/servo_status.h>
#include <servo_controller/servo_array_status.h>
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <time.h>   // time calls
#include <ros/console.h>

using namespace std;
using namespace ros::console::levels;

class ServoControllerCls
{
public:
	ServoControllerCls();
	void SetServoPosition(int channel_no, int target_position);
	void InitializeSerialPort();
	void InitializePubSub();
	void InitializeServoProperties();
	servo_controller::servo_array_status servo_array_status;
private:
	void servoControlCallback(const servo_controller::servo_control::ConstPtr& control_command);

	ros::NodeHandle nh_;

	ros::Publisher servo_array_status_pub_;
	ros::Subscriber servo_0_control_sub_;
	ros::Subscriber servo_1_control_sub_;
	ros::Subscriber servo_2_control_sub_;
	ros::Subscriber servo_3_control_sub_;
	ros::Subscriber servo_4_control_sub_;
	ros::Subscriber servo_5_control_sub_;

	int serial_port;
};

void ServoControllerCls::InitializeSerialPort() {
	// Make sure to set the servo controller to Dual USB mode
	serial_port = open("/dev/ttyACM1", O_RDWR | O_NOCTTY | O_NDELAY);
	if (serial_port < 0)
	{
		ROS_WARN("Cannot open servo controller serial port on /dev/ttyACM1. Make sure you have correct permissions. Port no: %d", serial_port);
	}
	else
	{
		ROS_LOG(Info, ROSCONSOLE_DEFAULT_NAME, "Opened servo controller serial port on /dev/ttyACM1. Port no: %d", serial_port);
		//printf("Opened ACM1: %d\r\n", (int) (serial_port));
	}
}

void ServoControllerCls::InitializePubSub() {

	//nh_.param("axis_linear", linear_, linear_);
	//nh_.param("axis_angular", angular_, angular_);
	//nh_.param("scale_angular", a_scale_, a_scale_);
	//nh_.param("scale_linear", l_scale_, l_scale_);


	// Publish status for all servos
	// Set latch to true, so the last message published on
    // this topic will be saved and sent to new subscribers when they
    // connect
	servo_array_status_pub_ = nh_.advertise < servo_controller::servo_array_status
			> ("servo_controller/servo_array_status", 1, true);

	// Separate messages for independent control of each servo
	servo_0_control_sub_ =
			nh_.subscribe < servo_controller::servo_control
					> ("servo_controller/servo_0_control", 4, &ServoControllerCls::servoControlCallback, this);
	servo_1_control_sub_ =
			nh_.subscribe < servo_controller::servo_control
					> ("servo_controller/servo_1_control", 4, &ServoControllerCls::servoControlCallback, this);
	servo_2_control_sub_ =
			nh_.subscribe < servo_controller::servo_control
					> ("servo_controller/servo_2_control", 4, &ServoControllerCls::servoControlCallback, this);
	servo_3_control_sub_ =
			nh_.subscribe < servo_controller::servo_control
					> ("servo_controller/servo_3_control", 4, &ServoControllerCls::servoControlCallback, this);
	servo_4_control_sub_ =
			nh_.subscribe < servo_controller::servo_control
					> ("servo_controller/servo_4_control", 4, &ServoControllerCls::servoControlCallback, this);
	servo_5_control_sub_ =
			nh_.subscribe < servo_controller::servo_control
					> ("servo_controller/servo_5_control", 4, &ServoControllerCls::servoControlCallback, this);

	// Publish initial, make sure values are initialized
	servo_array_status_pub_.publish(servo_array_status);
}

void ServoControllerCls::InitializeServoProperties() {
	const uint center = 6000;
	const uint max_value = 8000;
	const uint min_value = 4000;
	const uint velocity_limit = 100;
	const uint acceleration_limit = 20;

	servo_array_status.servo[0].enabled = false;
	servo_array_status.servo[1].enabled = false;
	servo_array_status.servo[2].enabled = false;
	servo_array_status.servo[3].enabled = false;
	servo_array_status.servo[4].enabled = false;
	servo_array_status.servo[5].enabled = false;

	servo_array_status.servo[0].servo_no = 0;
	servo_array_status.servo[1].servo_no = 1;
	servo_array_status.servo[2].servo_no = 2;
	servo_array_status.servo[3].servo_no = 3;
	servo_array_status.servo[4].servo_no = 4;
	servo_array_status.servo[5].servo_no = 5;

	servo_array_status.servo[0].name = "";
	servo_array_status.servo[1].name = "--Right Wheel Encoder";
	servo_array_status.servo[2].name = "Head Servo";
	servo_array_status.servo[3].name = "Right Steering Servo";
	servo_array_status.servo[4].name = "Left Steering Servo";
	servo_array_status.servo[5].name = "Main Motor";

	servo_array_status.servo[0].mode = 0; // Not defined
	servo_array_status.servo[1].mode = 2; // Digital input - wheel encoder
	servo_array_status.servo[2].mode = 1; // Servo control signal output - servo
	servo_array_status.servo[3].mode = 1; // Servo control signal output - servo
	servo_array_status.servo[4].mode = 1; // Servo control signal output - servo
	servo_array_status.servo[5].mode = 1; // Servo control signal output - motor

	servo_array_status.servo[2].target = center;
	servo_array_status.servo[3].target = center;
	servo_array_status.servo[4].target = center;
	servo_array_status.servo[5].target = center;

	servo_array_status.servo[2].actual = center;
	servo_array_status.servo[3].actual = center;
	servo_array_status.servo[4].actual = center;
	servo_array_status.servo[5].actual = center;

	servo_array_status.servo[2].center = center;
	servo_array_status.servo[3].center = center;
	servo_array_status.servo[4].center = center;
	servo_array_status.servo[5].center = center;

	servo_array_status.servo[2].max = max_value;
	servo_array_status.servo[3].max = max_value;
	servo_array_status.servo[4].max = max_value;
	servo_array_status.servo[5].max = max_value;

	servo_array_status.servo[2].min = min_value;
	servo_array_status.servo[3].min = min_value;
	servo_array_status.servo[4].min = min_value;
	servo_array_status.servo[5].min = min_value;

	servo_array_status.servo[2].velocity_limit = velocity_limit;
	servo_array_status.servo[3].velocity_limit = velocity_limit;
	servo_array_status.servo[4].velocity_limit = velocity_limit;
	servo_array_status.servo[5].velocity_limit = velocity_limit;

	servo_array_status.servo[2].acceleration_limit = acceleration_limit;
	servo_array_status.servo[3].acceleration_limit = acceleration_limit;
	servo_array_status.servo[4].acceleration_limit = acceleration_limit;
	servo_array_status.servo[5].acceleration_limit = acceleration_limit;

	servo_array_status.servo[0].enabled = false;
	servo_array_status.servo[1].enabled = false;
	servo_array_status.servo[2].enabled = true;
	servo_array_status.servo[3].enabled = true;
	servo_array_status.servo[4].enabled = true;
	servo_array_status.servo[5].enabled = true;
}

ServoControllerCls::ServoControllerCls()
{
	InitializeSerialPort();
	InitializeServoProperties();
	InitializePubSub();
}

void ServoControllerCls::SetServoPosition(int channel_no, int target_position) {
	char command_bytes[4] = { 0x84, channel_no, (char) ((target_position & 0x7F)),
			(char) ((target_position >> 7 & 0x7F)) };
	int n = write(serial_port, command_bytes, 4);
	if (n < 0)
	{
		ROS_WARN("Could not write bytes to servo controller serial port for servo no: %d, target: %d", channel_no, target_position);
	}
	else
	{
		ROS_LOG(Info, ROSCONSOLE_DEFAULT_NAME, "Set servo no: %d, target: %d", channel_no, target_position);
	}
}

void ServoControllerCls::servoControlCallback(const servo_controller::servo_control::ConstPtr& control_command)
{
	int servo_no = control_command->servo_no;
	if (servo_no < 0 || servo_no >= 6)
	{
		ROS_WARN("Wrong servo number passed: %d", servo_no);
		return;
	}
	if (!servo_array_status.servo[servo_no].enabled)
	{
		ROS_WARN("Trying to control a not enabled servo: %d", servo_no);
		return;
	}
	if (servo_array_status.servo[servo_no].mode != 1)
	{
		ROS_WARN("Trying to control servo: %d, in mode: %d. Mode has to be 1 for servos", servo_no, servo_array_status.servo[servo_no].mode);
		return;
	}

	uint target = control_command->target;
	const uint max_value = servo_array_status.servo[servo_no].max;
	const uint min_value = servo_array_status.servo[servo_no].min;
	if (target > max_value)
	{
		ROS_WARN("Trying to set servo: %d to target: %d more than maximum value: %d. Setting to maximum value", servo_no, target, max_value);
		target = max_value;
	}

	if (target < min_value)
	{
		ROS_WARN("Trying to set servo: %d to target: %d less than minimum value: %d. Setting to maximum value", servo_no, target, min_value);
		target = min_value;
	}
	SetServoPosition(servo_no, target);

	// Publish update
	servo_array_status.servo[servo_no].target = target;
	servo_array_status_pub_.publish(servo_array_status);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ServoController");
	ServoControllerCls servo_controller;
	ros::spin();
}
