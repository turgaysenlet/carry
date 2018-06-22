#include <ros/ros.h>
#include <servo_controller/servo_control.h>
#include <servo_controller/servo_status.h>
#include <servo_controller/servo_array_status.h>
#include <std_msgs/Int32.h>
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


// Gets the position of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
int maestroGetPosition(int fd, unsigned char channel)
{
	unsigned char command[] = {0x90, channel};
	int n = write(fd, command, sizeof(command));
	if (n < 0) {
		perror("error reading");
		return -1;
	}

	unsigned char response[2];
	n = read(fd, response,2);
	if (n != 2) {
		perror("error reading");
		return -1;
	}
	return response[0] + 256*response[1];
}

class ServoControllerCls
{
public:
	ServoControllerCls();
	~ServoControllerCls();
	void SetServoPosition(int channel_no, int target_position);
	int GetServoPosition(int channel_no);
	void ReadServos();
	void InitializeSerialPort();
	void InitializePubSub();
	void InitializeServoProperties();
	servo_controller::servo_array_status servo_array_status;
private:
	void servoControlCallback(const servo_controller::servo_control::ConstPtr& control_command);

	ros::NodeHandle nh_;

	ros::Publisher servo_array_status_pub_;
	ros::Publisher left_encoder_pub_;
	ros::Publisher right_encoder_pub_;
	ros::Subscriber servo_0_control_sub_;
	ros::Subscriber servo_1_control_sub_;
	ros::Subscriber servo_2_control_sub_;
	ros::Subscriber servo_3_control_sub_;
	ros::Subscriber servo_4_control_sub_;
	ros::Subscriber servo_5_control_sub_;

	int serial_port;
	int left_encoder_count;
	int right_encoder_count;
	bool left_encoder_prev;
	bool right_encoder_prev;
};

ServoControllerCls::~ServoControllerCls() {
	close(serial_port);
}

void ServoControllerCls::InitializeSerialPort() {
	// Make sure to set the servo controller to Dual USB mode
	// Open the Maestro's virtual COM port.
	//const char * device = "\\\\.\\USBSER000"; // Windows, "\\\\.\\COM6" also works
	const char * device = "/dev/ttyACM0"; // Linux
	//const char * device = "/dev/cu.usbmodem00034567"; // Mac OS X
	serial_port = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
	if (serial_port == -1)
	{
		perror(device);
		return;
	}
	#ifdef _WIN32
		_setmode(serial_port, _O_BINARY);
	#else
		struct termios options;
		tcgetattr(serial_port, &options);
		options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
		options.c_oflag &= ~(ONLCR | OCRNL);
		options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		tcsetattr(serial_port, TCSANOW, &options);
	#endif

	if (serial_port < 0)
	{
		ROS_WARN("Cannot open servo controller serial port on /dev/ttyACM0. Make sure you have correct permissions. Port no: %d", serial_port);
	}
	else
	{
		ROS_LOG(Info, ROSCONSOLE_DEFAULT_NAME, "Opened servo controller serial port on /dev/ttyACM0. Port no: %d", serial_port);
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

	left_encoder_pub_ = nh_.advertise < std_msgs::Int32> ("servo_controller/left_encoder_count", 1, true);
	right_encoder_pub_ = nh_.advertise < std_msgs::Int32> ("servo_controller/right_encoder_count", 1, true);


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
	const uint min_value = 3000;
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

	servo_array_status.servo[0].name = "Left Wheel Encoder";
	servo_array_status.servo[1].name = "Right Wheel Encoder";
	servo_array_status.servo[2].name = "Head Servo";
	servo_array_status.servo[3].name = "Right Steering Servo";
	servo_array_status.servo[4].name = "Left Steering Servo";
	servo_array_status.servo[5].name = "Main Motor";

	servo_array_status.servo[0].mode = 2; // Digital input - wheel encoder
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

	servo_array_status.servo[0].enabled = true;
	servo_array_status.servo[1].enabled = true;
	servo_array_status.servo[2].enabled = true;
	servo_array_status.servo[3].enabled = true;
	servo_array_status.servo[4].enabled = true;
	servo_array_status.servo[5].enabled = true;
}

ServoControllerCls::ServoControllerCls()
{
	left_encoder_count = 0;
	right_encoder_count = 0;
	left_encoder_prev = false;
	right_encoder_prev = false;
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
		ROS_LOG(Info, ROSCONSOLE_DEFAULT_NAME, "Servo %d: %d", channel_no, target_position);
	}
}

int ServoControllerCls::GetServoPosition(int channel_no) {
	int pos = maestroGetPosition(serial_port, (unsigned char) channel_no);
	if (channel_no == 0) {
		if (left_encoder_prev != (pos>127)) {
			left_encoder_prev = (pos>127);
			left_encoder_count++;
		 	// Fake right the same as left count.
			right_encoder_count++;
		}
	}
	/*if (channel_no == 1) {
		if (right_encoder_prev != pos) {
			right_encoder_prev = pos;
			right_encoder_count++;
		}
	}*/
	return pos;
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
	if (servo_array_status.servo[servo_no].mode == 1) {
	   servo_array_status.servo[servo_no].target = target;
    } else {
		ROS_ERROR("Servo: %d is read only.", servo_no);
	}
	servo_array_status_pub_.publish(servo_array_status);
}

void ServoControllerCls::ReadServos() {
	for (int servo_no = 0; servo_no < 6; servo_no++) {
		if (servo_array_status.servo[servo_no].mode == 2) {
			ROS_INFO("Reading servo %d", servo_no);
			servo_array_status.servo[servo_no].actual = GetServoPosition(servo_no);
			ROS_INFO("Servo %d position %d", servo_no, servo_array_status.servo[servo_no].actual);
		}
	}
	servo_array_status_pub_.publish(servo_array_status);	
	std_msgs::Int32 left;
	left.data = left_encoder_count;
	left_encoder_pub_.publish(left);
	std_msgs::Int32 right;
	right.data = right_encoder_count;
	right_encoder_pub_.publish(right);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ServoController");
	ServoControllerCls servo_controller;
	ros::Rate loop_rate(500);
	while (ros::ok())
	{	
		// Check messages
		ros::spinOnce();
		
		servo_controller.ReadServos();
		// This will adjust as needed per iteration
		loop_rate.sleep();
	}
}

/*

// Uses POSIX functions to send and receive data from a Maestro.
// NOTE: The Maestro's serial mode must be set to "USB Dual Port".
// NOTE: You must change the 'const char * device' line below.
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif
// Gets the position of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
int maestroGetPosition1(int fd, unsigned char channel)
{
	unsigned char command[] = {0x90, channel};
	if(write(fd, command, sizeof(command)) == -1)
	{
		perror("error writing");
		return -1;
	}
	unsigned char response[2];
	if(read(fd,response,2) != 2)
	{
		perror("error reading");
		return -1;
	}
	return response[0] + 256*response[1];
}

// Sets the target of a Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// The units of 'target' are quarter-microseconds.
int maestroSetTarget(int fd, unsigned char channel, unsigned short target)
{
	unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
	if (write(fd, command, sizeof(command)) == -1)
	{
		perror("error writing");
		return -1;
	}
	return 0;
}
int main()
{
	// Open the Maestro's virtual COM port.
	//const char * device = "\\\\.\\USBSER000"; // Windows, "\\\\.\\COM6" also works
	const char * device = "/dev/ttyACM0"; // Linux
	//const char * device = "/dev/cu.usbmodem00034567"; // Mac OS X
	int fd = open(device, O_RDWR | O_NOCTTY| O_NDELAY);
	if (fd == -1)
	{
		perror(device);
		return 1;
	}
	#ifdef _WIN32
		_setmode(fd, _O_BINARY);
	#else
		struct termios options;
		tcgetattr(fd, &options);
		options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
		options.c_oflag &= ~(ONLCR | OCRNL);
		options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		tcsetattr(fd, TCSANOW, &options);
	#endif

	int position = maestroGetPosition(fd, 0);
	printf("Current position is %d.\n", position);
	int target = (position < 6000) ? 7000 : 5000;
	printf("Setting target to %d (%d us).\n", target, target/4);
	maestroSetTarget(fd, 2, target);
	close(fd);
	return 0;
}*/
