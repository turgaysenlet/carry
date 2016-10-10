#include <ros/ros.h>
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

bool ProcessStereo = false;
const int LEFT_RECEIVE_PORT = 2000;
const int RIGHT_RECEIVE_PORT = 2001;
const char* COMMAND_SEND_IP = "192.168.1.70";
const int COMMAND_SEND_PORT = 3000;

class SimulatorImageReceiverCls
{
public:
	SimulatorImageReceiverCls();
	void ReceiveImages();
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	ros::Subscriber speed_sub_;
	ros::Subscriber steering_sub_;
	ros::Subscriber speed_steering_sub_;
	image_transport::Publisher left_raw_pub_;
	image_transport::Publisher right_raw_pub_;
	image_transport::Publisher disparity_pub_;
	ros::Publisher left_info_pub_;
	ros::Publisher right_info_pub_;
	ros::ServiceServer service_left_;
	ros::ServiceServer service_right_;
	ros::Publisher joint_pub_;
	tf::TransformBroadcaster broadcaster_;

	void speedCallback(const motor_controller::speed::ConstPtr& speed);
	void steeringCallback(const motor_controller::steering::ConstPtr& steering);
	void speedSteeringCallback(const motor_controller::speed_steering::ConstPtr& speed_steering);
	bool ServeCameraRequestLeft(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res);
	bool ServeCameraRequestRight(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res);

	void SetSteering(float steering_degree);
	void SetSpeed(float speed_mps);
	double CalculateMotorServoPositionFromSpeed(float speed);
	void PerformIgnition();
	void Stop();

	float desired_speed;
	float desired_steering;

	int Width;
	int Height;
	camera_info_manager::CameraInfoManager camera_info_manager_left_;
	camera_info_manager::CameraInfoManager camera_info_manager_right_;
};

SimulatorImageReceiverCls::SimulatorImageReceiverCls() : it_(nh_), camera_info_manager_left_(nh_, "left_simulation", ""), camera_info_manager_right_(nh_, "right_simulation", "")
{
	desired_speed = 0;
	desired_steering = 0;

	camera_info_manager_left_.loadCameraInfo("package://simulator_image_receiver/camera_info/left.yaml");	
	camera_info_manager_right_.loadCameraInfo("package://simulator_image_receiver/camera_info/right.yaml");

	//ROS_INFO("Camera Info Load Result: %d", (int)result);
	Width = 0;
	Height = 0;

	speed_sub_ = nh_.subscribe < motor_controller::speed > ("motor_controller/speed_control", 1, &SimulatorImageReceiverCls::speedCallback, this);
	steering_sub_ = nh_.subscribe < motor_controller::steering > ("motor_controller/steering_control", 1, &SimulatorImageReceiverCls::steeringCallback, this);
	speed_steering_sub_ = nh_.subscribe < motor_controller::speed_steering > ("motor_controller/speed_steering_control", 1, &SimulatorImageReceiverCls::speedSteeringCallback, this);
	left_raw_pub_ = it_.advertise("/stereo/left/image_raw", 1, false);
	right_raw_pub_ = it_.advertise("/stereo/right/image_raw", 1, false);
	joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

	if (ProcessStereo)
	{
		disparity_pub_ = it_.advertise("/stereo/disparity", 1, false);
	}

	left_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 1, false);
	right_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 1, false);
	service_left_ = nh_.advertiseService("/stereo/left/set_camera_info", &SimulatorImageReceiverCls::ServeCameraRequestLeft, this);
	service_right_ = nh_.advertiseService("/stereo/right/set_camera_info", &SimulatorImageReceiverCls::ServeCameraRequestRight, this);
}

bool SimulatorImageReceiverCls::ServeCameraRequestLeft(sensor_msgs::SetCameraInfo::Request &req,
		sensor_msgs::SetCameraInfo::Response &res)
{
	res.success = 1;
	return true;
}

bool SimulatorImageReceiverCls::ServeCameraRequestRight(sensor_msgs::SetCameraInfo::Request &req,
		sensor_msgs::SetCameraInfo::Response &res)
{
	res.success = 1;
	return true;
}

void SimulatorImageReceiverCls::speedCallback(const motor_controller::speed::ConstPtr& speed)
{
	SetSpeed(speed->speed_mps);
}

void SimulatorImageReceiverCls::steeringCallback(const motor_controller::steering::ConstPtr& steering)
{
	SetSteering(steering->degree);
}

void SimulatorImageReceiverCls::speedSteeringCallback(const motor_controller::speed_steering::ConstPtr& speed_steering)
{
	ROS_WARN("Received request: %f, %f", speed_steering->speed_mps.speed_mps, speed_steering->steering_degree.degree);
	SetSpeed(speed_steering->speed_mps.speed_mps);
	SetSteering(speed_steering->steering_degree.degree);
}

void SimulatorImageReceiverCls::SetSpeed(float speed_mps)
{
	desired_speed = speed_mps;
}

void SimulatorImageReceiverCls::SetSteering(float steering_degree)
{
	desired_steering = steering_degree;
}

void SimulatorImageReceiverCls::PerformIgnition()
{
	// Nothing
}

void SimulatorImageReceiverCls::Stop()
{
	SetSpeed(0);
	SetSteering(0);
}

void SimulatorImageReceiverCls::ReceiveImages()
{
	int sock_left;
	int sock_right;
	int sock_command_sender;
	unsigned int addr_len;
	int bytes_read_left, bytes_read_right;
	unsigned char recv_data_left[65536] = {0};
	unsigned char recv_data_right[65536] = {0};
	sockaddr_in server_addr_left;
	sockaddr_in server_addr_right;
	sockaddr_in client_addr;
	sockaddr_in client_addr_command;

	// Left UDP receiver
	if ((sock_left = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		ROS_ERROR("Socket creation problem");
		exit(1);
	}
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
	////////////////////
	// Right UDP receiver
	if ((sock_right = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		ROS_ERROR("Socket creation problem");
		exit(1);
	}
	fcntl(sock_right, F_SETFL, O_NONBLOCK); 
	
	server_addr_right.sin_family = AF_INET;
	server_addr_right.sin_port = htons(RIGHT_RECEIVE_PORT);
	server_addr_right.sin_addr.s_addr = INADDR_ANY;
	bzero(&(server_addr_right.sin_zero),8);

	if (bind(sock_right,(sockaddr *)&server_addr_right,
			sizeof(sockaddr)) == -1)
	{
		ROS_ERROR("Socket binding problem");
		exit(1);
	}
	////////////////////
	// UDP command sender
	if ((sock_command_sender = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		ROS_ERROR("Socket creation problem");
		exit(1);
	}
	hostent *host;
	host = (hostent *) gethostbyname((char *)COMMAND_SEND_IP);
	client_addr_command.sin_family = AF_INET;
	client_addr_command.sin_port = htons(COMMAND_SEND_PORT);
	client_addr_command.sin_addr = *((in_addr *)host->h_addr);
	bzero(&(client_addr_command.sin_zero),8);

	///////////////
	addr_len = sizeof(sockaddr);

	ROS_INFO("UDP Server waiting for client on port %d", htons(server_addr_left.sin_port));
	ROS_INFO("UDP Server waiting for client on port %d", htons(server_addr_right.sin_port));

	ros::Rate loop_rate(15);
	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	sensor_msgs::JointState joint_state;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "axis";
	int frame_counter = 0;
	bool keep_old_left = false;
	bool keep_old_right = false;
	cv::Mat image_left;
	cv::Mat image_right;
	while (ros::ok())
	{
		if (keep_old_left == false)
		{
			bytes_read_left = recvfrom(sock_left, recv_data_left, 65535, 0, (sockaddr *)&client_addr, &addr_len);
		}
		if (keep_old_right == false)
		{
			bytes_read_right = recvfrom(sock_right, recv_data_right, 65535, 0, (sockaddr *)&client_addr, &addr_len);
		}

		int frame_counter_left = (int)recv_data_left[0];
		int frame_counter_right = (int)recv_data_right[0];

		ROS_INFO("Left: %d, Right: %d", frame_counter_left, frame_counter_right);
		ROS_INFO("Left bytes: %d, Right bytes: %d", bytes_read_left, bytes_read_right);
		if (bytes_read_left > 0 && bytes_read_right > 0)
		{
			FILE* fl = fopen("/tmp/left.jpg","wb");
			fwrite(recv_data_left, sizeof(char), bytes_read_left, fl);
			fclose(fl);

			FILE* fr = fopen("/tmp/right.jpg","wb");
			fwrite(recv_data_right, sizeof(char), bytes_read_right, fr);
			fclose(fr);

			image_left = cv::imread("/tmp/left.jpg", 1);
			image_right = cv::imread("/tmp/right.jpg", 1);
			/*
			std::vector<char> data_left(recv_data_left, recv_data_left + bytes_read_left);
			std::vector<char> data_right(recv_data_right, recv_data_right + bytes_read_right);
			image_left = cv::imdecode(data_left, -1);
			image_right = cv::imdecode(data_right, -1);*/
			//cv::imwrite("/tmp/left.jpg", image_left);
			ROS_INFO("Image resolution: %dx%d", image_left.cols, image_left.rows);
			frame_counter++;
			if (frame_counter_left != frame_counter_right)
			{
				if (frame_counter_left > frame_counter_right)
				{
					ROS_WARN("Frame counters not match, keeping left. Global: %d, Left: %d, Right: %d", frame_counter, frame_counter_left, frame_counter_right);
					keep_old_right = false;
					keep_old_left = true;
				}
				else
				{
					ROS_WARN("Frame counters not match, keeping right. Global: %d, Left: %d, Right: %d", frame_counter, frame_counter_left, frame_counter_right);
					keep_old_right = true;
					keep_old_left = false;
				}
				continue;
			}
			else
			{
				keep_old_right = false;
				keep_old_left = false;
			}

			if (image_left.cols != image_right.cols || image_left.rows != image_right.rows)
			{
				ROS_WARN("Image sizes not match. Left: %dx%d, Right: %dx%d", image_left.cols, image_left.rows, image_right.cols, image_right.rows);
				continue;
			}

			if (Width != image_left.cols || Height != image_left.rows)
			{
				Width = image_left.cols;
				Height = image_left.rows;
				ROS_INFO("New image resolution: %dx%d", Width, Height);
			}

			//ROS_INFO("Left");
			std_msgs::Header header;
			header.seq = frame_counter;
			sensor_msgs::Image image_message_left;
			cv_bridge::CvImage cvimage_left(header, enc::BGR8, image_left);
			cvimage_left.toImageMsg(image_message_left);
			//ROS_INFO("Right");
			sensor_msgs::Image image_message_right;
			cv_bridge::CvImage cvimage_right(header, enc::BGR8, image_right);
			cvimage_right.toImageMsg(image_message_right);

			sensor_msgs::Image image_message_disp;
			if (ProcessStereo)
			{
				/*cv::StereoSGBM stereo(0, 64, 5);
				cv::Mat disp;
				stereo(image_left, image_right, disp);

				cv::Mat disp8;
				disp.convertTo(disp8, CV_8U, 0.25, 0);
				cv_bridge::CvImage cvimage_disp(header, enc::MONO8, disp8);
				cvimage_disp.toImageMsg(image_message_disp);*/
			}

			left_raw_pub_.publish(image_message_left);
			left_info_pub_.publish(camera_info_manager_left_.getCameraInfo());

			right_raw_pub_.publish(image_message_right);
			right_info_pub_.publish(camera_info_manager_right_.getCameraInfo());

			if (ProcessStereo)
			{
				disparity_pub_.publish(image_message_disp);
			}

			// Release images using deallocate (not release) otherwise memory leakge happens after imread
			image_left.deallocate();
			image_right.deallocate();
		}
		else
		{
			ROS_WARN("Image bytes don't match skipping. Left: %d, Right: %d", bytes_read_left, bytes_read_right);
		}
		
		float send_data[2] = {desired_speed, desired_steering};
		ROS_WARN("Send speed: %f, sent steer: %f", send_data[0], send_data[1]);		
		sendto(sock_command_sender, (void*)send_data, sizeof(send_data), 0, (sockaddr *)&client_addr_command, sizeof(sockaddr));

		//update joint_state
		//update joint_state
		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(3);
		joint_state.position.resize(3);
		joint_state.name[0] ="swivel";
		joint_state.position[0] = 0;
		joint_state.name[1] ="tilt";
		joint_state.position[1] = 0;
		joint_state.name[2] ="periscope";
		joint_state.position[2] = 0;

		// update transform
		odom_trans.header.stamp = ros::Time::now();
		odom_trans.transform.translation.x = 0;
		odom_trans.transform.translation.y = 0;
		odom_trans.transform.translation.z = 0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

		//send the joint state and transform
		joint_pub_.publish(joint_state);
		broadcaster_.sendTransform(odom_trans);

		// Check messages
		ros::spinOnce();

		// This will adjust as needed per iteration
		loop_rate.sleep();
	}
	close (sock_left);
	close (sock_right);
	close (sock_command_sender);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "SimulatorImageReceiver");
	SimulatorImageReceiverCls simulator_image_receiver;
	//ros::spin();
	ros::spinOnce();
	simulator_image_receiver.ReceiveImages();
}

