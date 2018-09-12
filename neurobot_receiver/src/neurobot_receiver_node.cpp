#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <motor_controller/speed.h>
#include <motor_controller/steering.h>
#include <motor_controller/speed_steering.h>
#include <cv_bridge/cv_bridge.h>
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <time.h>   // time calls
#include <ros/console.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

// udpserver
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

using namespace std;
using namespace ros::console::levels;
namespace enc = sensor_msgs::image_encodings;

float X = 0;
bool ProcessStereo = false;
const int LEFT_RECEIVE_PORT = 2000;

const char* COMMAND_SEND_IP = "10.0.0.108";
//const char* COMMAND_SEND_IP = "192.168.1.70";
const int COMMAND_SEND_PORT = 3000;

class NeurobotReceiverCls
{
public:
	NeurobotReceiverCls();
	void ReceiveData();
private:
	ros::NodeHandle nh_;
	//image_transport::ImageTransport it_;
	//ros::Subscriber speed_sub_;
	//ros::Subscriber steering_sub_;
	//ros::Subscriber speed_steering_sub_;
	//image_transport::Publisher left_raw_pub_;
	//image_transport::Publisher right_raw_pub_;
	ros::Publisher neurobot_pub_;
	
	void speedCallback(const motor_controller::speed::ConstPtr& speed);
	void steeringCallback(const motor_controller::steering::ConstPtr& steering);
	void speedSteeringCallback(const motor_controller::speed_steering::ConstPtr& speed_steering);
	
	void SetSteering(float steering_degree);
	void SetSpeed(float speed_mps);
	double CalculateMotorServoPositionFromSpeed(float speed);
	void PerformIgnition();
	void Stop();

	float desired_speed;
	float desired_steering;

	int Width;
	int Height;
};

NeurobotReceiverCls::NeurobotReceiverCls()
{
	desired_speed = 0;
	desired_steering = 0;


	Width = 0;
	Height = 0;
    neurobot_pub_ = nh_.advertise <geometry_msgs::Vector3> ("neurobot", 1);
}

void NeurobotReceiverCls::speedCallback(const motor_controller::speed::ConstPtr& speed)
{
	SetSpeed(speed->speed_mps);
}

void NeurobotReceiverCls::steeringCallback(const motor_controller::steering::ConstPtr& steering)
{
	SetSteering(steering->degree);
}

void NeurobotReceiverCls::speedSteeringCallback(const motor_controller::speed_steering::ConstPtr& speed_steering)
{
	ROS_WARN("Received request: %f, %f", speed_steering->speed_mps.speed_mps, speed_steering->steering_degree.degree);
	//SetSpeed(speed_steering->speed_mps.speed_mps);
	//SetSteering(speed_steering->steering_degree.degree);
}

void NeurobotReceiverCls::SetSpeed(float speed_mps)
{
	desired_speed = speed_mps;
}

void NeurobotReceiverCls::SetSteering(float steering_degree)
{
	desired_steering = steering_degree;
}

void NeurobotReceiverCls::Stop()
{
	SetSpeed(0);
	SetSteering(0);
}

void NeurobotReceiverCls::ReceiveData()
{
	int sock_left;	
	int sock_command_sender;
	unsigned int addr_len;
	int bytes_read_left;
	unsigned char recv_data_left[65536] = {0};
	sockaddr_in server_addr_left;
	sockaddr_in client_addr;
	sockaddr_in client_addr_command;

	// Left UDP receiver
	if ((sock_left = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		ROS_ERROR("Socket creation problem");
		exit(1);
	}

	// Non-blocking receives	
	fcntl(sock_left, F_SETFL, O_NONBLOCK); 

	server_addr_left.sin_family = AF_INET;
	server_addr_left.sin_port = htons(LEFT_RECEIVE_PORT);
	server_addr_left.sin_addr.s_addr = INADDR_ANY;
	bzero(&(server_addr_left.sin_zero),8);

	if (bind(sock_left,(sockaddr *)&server_addr_left,
			sizeof(sockaddr)) == -1)
	{
		ROS_ERROR("Socket binding problem");
		exit(1);
	}
	
/*	hostent *host;
	host = (hostent *) gethostbyname((char *)COMMAND_SEND_IP);
	client_addr_command.sin_family = AF_INET;
	client_addr_command.sin_port = htons(COMMAND_SEND_PORT);
	client_addr_command.sin_addr = *((in_addr *)host->h_addr);
	bzero(&(client_addr_command.sin_zero),8);*/

	///////////////
	addr_len = sizeof(sockaddr);

	ROS_INFO("UDP Server waiting for client on port %d", htons(server_addr_left.sin_port));


	ros::Rate loop_rate(30);
	// message declarations
	geometry_msgs::TransformStamped odom_trans;	
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "axis";
	int frame_counter = 0;
	bool keep_old_left = false;
	cv::Mat image_left;
	while (ros::ok())
	{
		bytes_read_left = recvfrom(sock_left, recv_data_left, 2, 0, (sockaddr *)&client_addr, &addr_len);

		int frame_counter_left = (int)recv_data_left[0];

		ROS_INFO("Left count: %d", frame_counter_left);
		ROS_INFO("Left bytes: %d", bytes_read_left);
		ROS_INFO("Left byte[0]: %d", (int)recv_data_left[0]);
		if (bytes_read_left <= 0) 
		{
			continue;
		}
		geometry_msgs::Vector3 vector_message;
		float x = -5.0f*(((float)((int)recv_data_left[0]) / 255.0f)-0.5f);
		X = X * 0.95f + x * 0.05f;
        vector_message.x = X;
        vector_message.y = vector_message.x;
        vector_message.z = 0;
		neurobot_pub_.publish(vector_message);
		continue;

		float send_data[2] = {desired_speed, desired_steering};
		ROS_INFO("Send speed: %f, sent steer: %f", send_data[0], send_data[1]);		
		sendto(sock_command_sender, (void*)send_data, sizeof(send_data), 0, (sockaddr *)&client_addr_command, sizeof(sockaddr));

		// update transform
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.transform.translation.x = 0;
		odom_trans.transform.translation.y = 0;
		odom_trans.transform.translation.z = 0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

		// Check messages
		ros::spinOnce();

		// This will adjust as needed per iteration
		loop_rate.sleep();
	}
	close (sock_left);

	close (sock_command_sender);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "SimulatorImageReceiver");
	NeurobotReceiverCls neurobot_receiver;
	//ros::spin();
	ros::spinOnce();
	neurobot_receiver.ReceiveData();
}

