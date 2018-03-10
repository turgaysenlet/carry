#include <ros/ros.h>
#include <servo_controller/servo_array_status.h>
#include <servo_controller/servo_status.h>
#include <servo_controller/servo_control.h>
#include <motor_controller/speed.h>
#include <motor_controller/steering.h>
#include <motor_controller/speed_steering.h>
#include <motor_controller/ignition_control.h>
#include <speech_engine/speech_request.h>
#include <robot_state/robot_state_constants.h>
#include <robot_state/robot_state.h>
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

const int HeadServoNo = 2;
const int SteeringServo1No = 3;
const int SteeringServo2No = 4;
const int MotorServoNo = 5;

const float SpeedCenterConstant = 5910.0f;
const float SpeedConstant = 542.0f;
const float ForwardSpeedStartConstant = 120.0f;
const float BackwardSpeedStartConstant = 0.0f;

/// <summary>
/// Maximum head servo angle in degrees.
/// </summary>
const float MaximumHeadAngle = 90.0f;
/// <summary>
/// Maximum steering angle in degrees.
/// </summary>
const float MaximumSteeringAngle = 30.0f;
/// <summary>
/// Maximum forward travel speed in meters per second.
/// </summary>
const float MaximumForwardSpeed = 1.0f;//4.1f;
/// <summary>
/// Maximum reverse travel speed in meters per second.
/// </summary>
const float MaximumReverseSpeed = -0.8f;//-4.1f;
/// <summary>
/// Minimum forward travel speed in meters per second.
/// Below this value motor does not work properly.
/// </summary>
const float MinimumSpeed = 0.2f;//0.2f;
/// <summary>
/// Normal travel speed in meters per second.
/// </summary>
const float NormalSpeed = 1.0f;//1.0f;
/// <summary>
/// Slow travel speed in meters per second.
/// </summary>
const float SlowSpeed = 0.45f;//0.45f;
/// <summary>
/// Extra slow travel speed in meters per second.
/// </summary>
const float ExtraSlowSpeed = 0.3f;//0.28f;
/// <summary>
/// Fast travel speed in meters per second.
/// </summary>
const float FastSpeed = 1.0f;//2.0f;
/// <summary>
/// Extra fast travel speed in meters per second. Not suitable for indoors.
/// </summary>
const float ExtraFastSpeed = 1.0f;//4.0f;
/// <summary>
/// Stop speed.
/// </summary>
const float StopSpeed = 0.0f;

/// <summary>
/// Ignition step 1 speed in meters per second.
/// Can be too high, do not use it for running the robot.
/// Use it only for less than 1 seconds.
/// </summary>
const float IgnitionStep1Speed = 0.0f;
/// <summary>
/// Ignition step 2 speed in meters per second.
/// Can be too high, do not use it for running the robot.
/// Use it only for less than 1 seconds.
/// </summary>
const float IgnitionStep2Speed = -4;
/// <summary>
/// Ignition step 3 speed in meters per second.
/// Can be too high, do not use it for running the robot.
/// Use it only for less than 1 seconds.
/// </summary>
const float IgnitionStep3Speed = 0.0f;

const float IgnitionStep0SleepSeconds = 1;
const float IgnitionStep1SleepSeconds = 2;
const float IgnitionStep2SleepSeconds = 2;
const float IgnitionStep3SleepSeconds = 2;

float desired_speed = 0;
float desired_speed_final = 0;

class MotorControllerCls
{
public:
	MotorControllerCls();
	void SetServoPosition(int channelNo, int targetPosition);
	void SetSteering(float steering_degree);
	void SetHead(float head_degree);
	void SetSpeed(float speed_mps);
	double CalculateMotorServoPositionFromSpeed(float speed);
	void PerformIgnition();
	void Stop();
	void ChangeState(int new_state);
private:
	void robotStateCallback(const robot_state::robot_state::ConstPtr& state);
	void speedCallback(const motor_controller::speed::ConstPtr& speed);
	void steeringCallback(const motor_controller::steering::ConstPtr& steering);
	void speedSteeringCallback(const motor_controller::speed_steering::ConstPtr& speed_steering);
	void servoStatusCallback(const servo_controller::servo_array_status::ConstPtr& servo_array_status);
	void ignitionCallback(const motor_controller::ignition_control::ConstPtr& ignition_control);

	servo_controller::servo_array_status servo_array_status_;
	bool ignition_performed_;
	int robot_state;
	bool first_time_trying_ignition;

	ros::NodeHandle nh_;

	ros::Publisher servo_0_control_pub_;
	ros::Publisher servo_1_control_pub_;
	ros::Publisher servo_2_control_pub_;
	ros::Publisher servo_3_control_pub_;
	ros::Publisher servo_4_control_pub_;
	ros::Publisher servo_5_control_pub_;
	ros::Publisher speech_pub_;
	ros::Subscriber speed_sub_;
	ros::Subscriber steering_sub_;
	ros::Subscriber speed_steering_sub_;
	ros::Subscriber servo_array_status_sub_;
	ros::Subscriber ignition_callback_sub_;
	ros::Subscriber robot_state_sub_;
};

MotorControllerCls::MotorControllerCls()
{
	robot_state = robot_state::robot_state_constants::RobotState_Stop;
	ignition_performed_ = false;
	first_time_trying_ignition = true;
	// Empty
	servo_0_control_pub_ = nh_.advertise < servo_controller::servo_control > ("servo_controller/servo_0_control", 3);
	// Empty
	servo_1_control_pub_ = nh_.advertise < servo_controller::servo_control > ("servo_controller/servo_1_control", 3);
	// Head servo
	servo_2_control_pub_ = nh_.advertise < servo_controller::servo_control > ("servo_controller/servo_2_control", 3);
	// Right steering servo
	servo_3_control_pub_ = nh_.advertise < servo_controller::servo_control > ("servo_controller/servo_3_control", 3);
	// Left steering servo
	servo_4_control_pub_ = nh_.advertise < servo_controller::servo_control > ("servo_controller/servo_4_control", 3);
	// Main motor servo
	servo_5_control_pub_ = nh_.advertise < servo_controller::servo_control > ("servo_controller/servo_5_control", 4);

	speech_pub_ = nh_.advertise <speech_engine::speech_request> ("speech_engine/speech_request", 1);
	robot_state_sub_ = nh_.subscribe < robot_state::robot_state > ("robot_state/robot_state", 10, &MotorControllerCls::robotStateCallback, this);

	// Subscribe to separate speed and steering messages to be able to control one
	// without changing the other. Also subscribe to combined speed steering control
	// messages for combined control of both
	speed_sub_ = nh_.subscribe < motor_controller::speed > ("motor_controller/speed_control", 1, &MotorControllerCls::speedCallback, this);
	steering_sub_ = nh_.subscribe < motor_controller::steering > ("motor_controller/steering_control", 1, &MotorControllerCls::steeringCallback, this);
	speed_steering_sub_ = nh_.subscribe < motor_controller::speed_steering > ("motor_controller/speed_steering_control", 1, &MotorControllerCls::speedSteeringCallback, this);
	// Subscribe to servo status, to be able to see wheel encoder status
	// This information will be used to understand if the robot is stuck
	// in cases, where motor command is non-zero but wheels are not moving
	servo_array_status_sub_ = nh_.subscribe < servo_controller::servo_array_status > ("servo_controller/servo_array_status", 1, &MotorControllerCls::servoStatusCallback, this);
	ignition_callback_sub_ = nh_.subscribe < motor_controller::ignition_control > ("motor_controller/ignition_control", 1, &MotorControllerCls::ignitionCallback, this);
}

void MotorControllerCls::SetServoPosition(int channel_no, int target)
{
	ROS_INFO("Speed sent: %0.2f, speed: %0.2f, target: %d, servono: %d", desired_speed, desired_speed_final, target, channel_no);
	servo_controller::servo_control message;
	message.servo_no = channel_no;
	message.target = target;
	if (channel_no == 0) {
		servo_0_control_pub_.publish(message);
	} else if (channel_no == 1) {
		servo_1_control_pub_.publish(message);
	} else if (channel_no == 2) {
		servo_2_control_pub_.publish(message);
	} else if (channel_no == 3) {
		servo_3_control_pub_.publish(message);
	} else if (channel_no == 4) {
		servo_4_control_pub_.publish(message);
	} else if (channel_no == 5) {
		servo_5_control_pub_.publish(message);
	}
}
void MotorControllerCls::SetSpeed(float speed_mps)
{
	desired_speed = speed_mps;
	printf("speed_mps: %0.2f\n", speed_mps);
	if (robot_state == robot_state::robot_state_constants::RobotState_Stop)
	{
		speed_mps = 0;
	}
	uint target = 6000;
	uint center = SpeedCenterConstant;

	if (speed_mps > MaximumForwardSpeed)
	{
		ROS_WARN("Speed command sent: %0.2f is more than maximum forward speed: %0.2f. Setting to maximum", speed_mps, MaximumForwardSpeed);
		speed_mps = MaximumForwardSpeed;
	}
	else if (speed_mps < MaximumReverseSpeed)
	{
		ROS_WARN("Speed command sent: %0.2f is less than maximum reverse speed: %0.2f. Setting to minimum", speed_mps, MaximumReverseSpeed);
		speed_mps = MaximumReverseSpeed;
	}

	if (speed_mps == 0)
	{
		target = center;
	}
	else if (speed_mps > 0)
	{
		target = (uint)(speed_mps * SpeedConstant + center);
	}
	else if (speed_mps < 0)
	{
		target = (uint)(speed_mps * SpeedConstant + center);
	}

	desired_speed_final = speed_mps;
	SetServoPosition(MotorServoNo, target);
}
void MotorControllerCls::SetHead(float  head_degree)
{
	// Invert direction
	head_degree = -head_degree;
	if (robot_state == robot_state::robot_state_constants::RobotState_Stop)
	{
		head_degree = 0;
	}
	uint target2 = 6000;
	uint center2 = servo_array_status_.servo[2].center;
	uint max_value2 = servo_array_status_.servo[2].max;
	uint min_value2 = servo_array_status_.servo[2].min;

	if (head_degree > MaximumHeadAngle)
	{
		ROS_WARN("Head command sent: %0.2f is more than maximum head angle: %0.2f. Setting to maximum", head_degree, MaximumHeadAngle);
		head_degree = MaximumHeadAngle;
	}
	else if (head_degree < -MaximumHeadAngle)
	{
		ROS_WARN("Head command sent: %0.2f is less than minimum head angle: %0.2f. Setting to minimum", head_degree, -MaximumHeadAngle);
		head_degree = -MaximumHeadAngle;
	}
	if (center2 != 0 && max_value2 != 0)
	{
		if (head_degree == 0)
		{
			target2 = center2;
		}
		else if (head_degree > 0)
		{
			target2 = (uint)(head_degree * (max_value2 - center2) / MaximumHeadAngle + center2);
		}
		else if (head_degree < 0)
		{
			target2 = (uint)(head_degree * (center2 - min_value2) / MaximumHeadAngle + center2);
		}
	}
	else
	{
		target2 += (uint)(head_degree * 1500.0 / MaximumHeadAngle);
	}
	SetServoPosition(HeadServoNo, target2);
}
void MotorControllerCls::SetSteering(float steering_degree)
{
	if (robot_state == robot_state::robot_state_constants::RobotState_Stop)
	{
		steering_degree = 0;
	}
	uint target3 = 6000;
	uint center3 = servo_array_status_.servo[3].center;
	uint max_value3 = servo_array_status_.servo[3].max;
	uint min_value3 = servo_array_status_.servo[3].min;

	uint target4 = 6000;
	uint center4 = servo_array_status_.servo[4].center;
	uint max_value4 = servo_array_status_.servo[4].max;
	uint min_value4 = servo_array_status_.servo[4].min;

	if (steering_degree > MaximumSteeringAngle)
	{
		ROS_WARN("Steering command sent: %0.2f is more than maximum steering angle: %0.2f. Setting to maximum", steering_degree, MaximumSteeringAngle);
		steering_degree = MaximumSteeringAngle;
	}
	else if (steering_degree < -MaximumSteeringAngle)
	{
		ROS_WARN("Steering command sent: %0.2f is less than minimum steering angle: %0.2f. Setting to minimum", steering_degree, -MaximumSteeringAngle);
		steering_degree = -MaximumSteeringAngle;
	}
	if (center3 != 0 && max_value3 != 0)
	{
		if (steering_degree == 0)
		{
			target3 = center3;
		}
		else if (steering_degree > 0)
		{
			target3 = (uint)(steering_degree * (max_value3 - center3) / MaximumSteeringAngle + center3);
		}
		else if (steering_degree < 0)
		{
			target3 = (uint)(steering_degree * (center3 - min_value3) / MaximumSteeringAngle + center3);
		}
	}
	else
	{
		target3 += (uint)(steering_degree * 1500.0 / MaximumSteeringAngle);
	}
	if (center4 != 0 && max_value4 != 0)
	{
		if (steering_degree == 0)
		{
			target4 = center4;
		}
		else if (steering_degree > 0)
		{
			target4 = (uint)(steering_degree * (max_value4 - center4) / MaximumSteeringAngle + center4);
		}
		else if (steering_degree < 0)
		{
			target4 = (uint)(steering_degree * (center4 - min_value4) / MaximumSteeringAngle + center4);
		}
	}
	else
	{
		target4 += (uint)(steering_degree * 1500.0 / MaximumSteeringAngle);
	}
	SetServoPosition(SteeringServo1No, target3);
	SetServoPosition(SteeringServo2No, target4);
}

double MotorControllerCls::CalculateMotorServoPositionFromSpeed(float speed)
{
	if (speed == IgnitionStep1Speed || speed == IgnitionStep2Speed || speed == IgnitionStep3Speed)
	{
		if (speed > 0.1)
		{
			return 5910.0f + speed * SpeedConstant + ForwardSpeedStartConstant;
		}
		else if (speed < -0.1)
		{
			return SpeedCenterConstant + speed * SpeedConstant - BackwardSpeedStartConstant;
		}
	}
	else if (abs(speed) < 10)
	{
		if (speed > 0.1)
		{
			return SpeedCenterConstant + speed * SpeedConstant + ForwardSpeedStartConstant;
		}
		else if (speed < -0.1)
		{
			return SpeedCenterConstant + speed * SpeedConstant - BackwardSpeedStartConstant;
		}
	}
	return SpeedCenterConstant;
}

void MotorControllerCls::PerformIgnition()
{
	if (robot_state != robot_state::robot_state_constants::RobotState_Remote)
	{
		if (first_time_trying_ignition)
		{
			speech_engine::speech_request speech_message;
			speech_message.speech_request = "Error: Motor ingnition can only be done in remote state.";
			speech_pub_.publish(speech_message);
			first_time_trying_ignition = false;
		}
	}
	else
	{
		speech_engine::speech_request speech_message;
		speech_message.speech_request = "Starting motor ingnition sequence...";
		speech_pub_.publish(speech_message);

		ROS_INFO("Performing ignition");
		int channel0 = MotorServoNo;
		usleep((int)(IgnitionStep0SleepSeconds * 1000000.0f));
		ROS_INFO("Performing ignition - Step 1/3");
		SetServoPosition(channel0, CalculateMotorServoPositionFromSpeed(IgnitionStep1Speed));
		usleep((int)(IgnitionStep1SleepSeconds * 1000000.0f));
		ROS_INFO("Performing ignition - Step 2/3");
		SetServoPosition(channel0, CalculateMotorServoPositionFromSpeed(IgnitionStep2Speed));
		usleep((int)(IgnitionStep2SleepSeconds * 1000000.0f));
		ROS_INFO("Performing ignition - Step 3/3");
		SetServoPosition(channel0, CalculateMotorServoPositionFromSpeed(IgnitionStep3Speed));
		usleep((int)(IgnitionStep3SleepSeconds * 1000000.0f));
		ROS_INFO("Performing ignition - Ignition completed");
		ignition_performed_ = true;

		speech_message.speech_request = "Motor ingnition complete.";
		speech_pub_.publish(speech_message);
	}
}

void MotorControllerCls::Stop()
{
	SetSpeed(0);
	SetSteering(0);
	SetHead(0);
}

void MotorControllerCls::ChangeState(int new_state)
{
	if (new_state == robot_state::robot_state_constants::RobotState_Stop)
	{
		Stop();
		first_time_trying_ignition = true;
	}
	robot_state = new_state;
}

void MotorControllerCls::robotStateCallback(const robot_state::robot_state::ConstPtr& state)
{
	ChangeState((int)((int)state->state));
}

void MotorControllerCls::speedCallback(const motor_controller::speed::ConstPtr& speed)
{
	SetSpeed(speed->speed_mps);
}

void MotorControllerCls::steeringCallback(const motor_controller::steering::ConstPtr& steering)
{
	SetSteering(steering->degree);
}

void MotorControllerCls::speedSteeringCallback(const motor_controller::speed_steering::ConstPtr& speed_steering)
{
	SetSpeed(speed_steering->speed_mps.speed_mps);
	SetSteering(speed_steering->steering_degree.degree);
	SetHead(speed_steering->head_degree.degree);
}

void MotorControllerCls::servoStatusCallback(const servo_controller::servo_array_status::ConstPtr& servo_array_status)
{
	servo_array_status_ = *servo_array_status;
}

void MotorControllerCls::ignitionCallback(const motor_controller::ignition_control::ConstPtr& ignition_control)
{
	if (ignition_control->initiate_ignition)
	{
		if (ignition_performed_)
		{
			if (ignition_control->force)
			{
				ROS_WARN("Received ignition request with 'force' option and ignition has already been performed. Performing ignition");
				PerformIgnition();
			}
			else
			{
				ROS_WARN("Received ignition request with no 'force' option and ignition has already been performed. Not performing ignition");
			}
		}
		else
		{
			PerformIgnition();
		}
	}
	else
	{
		ROS_WARN("Received ignition message with no ignition request");
	}
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "MotorController");
	MotorControllerCls motor_controller;
	ros::spin();
}
