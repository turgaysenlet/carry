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

// TCP server

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
const char* COMMAND_SEND_IP = "10.0.0.108";
//const char* COMMAND_SEND_IP = "192.168.1.70";
const int COMMAND_SEND_PORT = 3000;
const int MAX_BYTES = 131072;

int socket_left;
int socket_right;

class SimulatorImageReceiverTcpCls
{
public:
	SimulatorImageReceiverTcpCls();
	~SimulatorImageReceiverTcpCls();
	void ReceiveImages();
	int ListenOnSocket(int &sock, int receive_port, sockaddr_in &server_addr);
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

	int width;
	int height;

	camera_info_manager::CameraInfoManager camera_info_manager_left_;
	camera_info_manager::CameraInfoManager camera_info_manager_right_;
};

SimulatorImageReceiverTcpCls::~SimulatorImageReceiverTcpCls() {
	ROS_INFO("SimulatorImageReceiverTcpCls cleaning up");
	if (socket_left != 0) {
		ROS_INFO("SimulatorImageReceiverTcpCls closing left socket...");		
		shutdown(socket_left, 2);
		close(socket_left);
		ROS_INFO("SimulatorImageReceiverTcpCls closing left socket done.");
	}
	if (socket_right != 0) {
		ROS_INFO("SimulatorImageReceiverTcpCls closing right socket...");
		shutdown(socket_right, 2);
		close(socket_right);
		ROS_INFO("SimulatorImageReceiverTcpCls closing right socket done.");
	}	
}

SimulatorImageReceiverTcpCls::SimulatorImageReceiverTcpCls() : it_(nh_), camera_info_manager_left_(nh_, "left_simulation", ""), camera_info_manager_right_(nh_, "right_simulation", "")
{
	desired_speed = 0;
	desired_steering = 0;

	//camera_info_manager_left_.loadCameraInfo("package://simulator_image_receiver/camera_info/left.yaml");	
	//camera_info_manager_right_.loadCameraInfo("package://simulator_image_receiver/camera_info/right.yaml");

	camera_info_manager_left_.loadCameraInfo("package://simulator_image_receiver/camera_info/left400.yaml");	
	camera_info_manager_right_.loadCameraInfo("package://simulator_image_receiver/camera_info/right400.yaml");

	//ROS_INFO("Camera Info Load Result: %d", (int)result);
	width = 0;
	height = 0;

	speed_sub_ = nh_.subscribe < motor_controller::speed > ("motor_controller/speed_control", 1, &SimulatorImageReceiverTcpCls::speedCallback, this);
	steering_sub_ = nh_.subscribe < motor_controller::steering > ("motor_controller/steering_control", 1, &SimulatorImageReceiverTcpCls::steeringCallback, this);
	speed_steering_sub_ = nh_.subscribe < motor_controller::speed_steering > ("motor_controller/speed_steering_control", 1, &SimulatorImageReceiverTcpCls::speedSteeringCallback, this);
	left_raw_pub_ = it_.advertise("/stereo/left/image_raw", 1, false);
	right_raw_pub_ = it_.advertise("/stereo/right/image_raw", 1, false);
	joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

	if (ProcessStereo)
	{
		disparity_pub_ = it_.advertise("/stereo/disparity", 1, false);
	}

	left_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 1, false);
	right_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 1, false);
	service_left_ = nh_.advertiseService("/stereo/left/set_camera_info", &SimulatorImageReceiverTcpCls::ServeCameraRequestLeft, this);
	service_right_ = nh_.advertiseService("/stereo/right/set_camera_info", &SimulatorImageReceiverTcpCls::ServeCameraRequestRight, this);	
}

int SimulatorImageReceiverTcpCls::ListenOnSocket(int &sock, int receive_port, sockaddr_in &server_addr) {
	ROS_INFO("SimulatorImageReceiverTcpCls::ListenOnSocket for port %d...", receive_port);
	sock = socket(AF_INET, SOCK_STREAM, 0);

	// Check to see if we have a valid socket
	if(sock < 0) {
   		int iSocketError = 0;//WSAGetLastError();
   		ROS_ERROR("SimulatorImageReceiverTcpCls::ListenOnSocket create socket failed with %d", iSocketError);
   		exit(1);
	}
	fcntl(sock, F_SETFL, (~O_NONBLOCK)); 

	memset(&server_addr, 0, sizeof(sockaddr_in));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(receive_port);
	server_addr.sin_addr.s_addr = INADDR_ANY;
	bzero(&(server_addr.sin_zero), 8);

	// Bind socket
	if (bind(sock,(sockaddr *)&server_addr, sizeof(sockaddr)) < -1)
	{
   		int iSocketError = 0;//WSAGetLastError();
		ROS_ERROR("SimulatorImageReceiverTcpCls::ListenOnSocket bind socket failed with %d", iSocketError);
		exit(1);
	}

	// Listen for incoming connections
	if(listen(sock, 1) < 0) {
	   int iSocketError = 0;//WSAGetLastError();
   		ROS_ERROR("SimulatorImageReceiverTcpCls::ListenOnSocket listen socket failed with %d", iSocketError);
		exit(1);
	}

	// Accept incoming connection request
	sockaddr_in sIncomingAddr;
	memset(&sIncomingAddr, 0, sizeof(sockaddr_in));
	unsigned int iAddrLen = sizeof(sockaddr_in);
	int sIncomingSocket = accept(sock, (sockaddr *) &sIncomingAddr, &iAddrLen);
	if(sIncomingSocket < 0) {
   		int iSocketError = 0;//WSAGetLastError();
   		ROS_WARN("SimulatorImageReceiverTcpCls::ListenOnSocket accept socket failed with %d", iSocketError);
   		// Return false, but we can continue accepting in a polling loop until a connection request comes.
   		return -1;
	} else {
		ROS_INFO("SimulatorImageReceiverTcpCls::ListenOnSocket accept socket connection with socket %d from %s port %d",
            sIncomingSocket, inet_ntoa(sIncomingAddr.sin_addr), ntohs(sIncomingAddr.sin_port));
	}

	ROS_INFO("SimulatorImageReceiverTcpCls::ListenOnSocket for port %d done.", receive_port);
	return sIncomingSocket;
}


bool SimulatorImageReceiverTcpCls::ServeCameraRequestLeft(sensor_msgs::SetCameraInfo::Request &req,
		sensor_msgs::SetCameraInfo::Response &res)
{
	res.success = 1;
	return true;
}

bool SimulatorImageReceiverTcpCls::ServeCameraRequestRight(sensor_msgs::SetCameraInfo::Request &req,
		sensor_msgs::SetCameraInfo::Response &res)
{
	res.success = 1;
	return true;
}

void SimulatorImageReceiverTcpCls::speedCallback(const motor_controller::speed::ConstPtr& speed)
{
	SetSpeed(speed->speed_mps);
}

void SimulatorImageReceiverTcpCls::steeringCallback(const motor_controller::steering::ConstPtr& steering)
{
	SetSteering(steering->degree);
}

void SimulatorImageReceiverTcpCls::speedSteeringCallback(const motor_controller::speed_steering::ConstPtr& speed_steering)
{
	ROS_WARN("Received request: %f, %f", speed_steering->speed_mps.speed_mps, speed_steering->steering_degree.degree);
	SetSpeed(speed_steering->speed_mps.speed_mps);
	SetSteering(speed_steering->steering_degree.degree);
}

void SimulatorImageReceiverTcpCls::SetSpeed(float speed_mps)
{
	desired_speed = speed_mps;
}

void SimulatorImageReceiverTcpCls::SetSteering(float steering_degree)
{
	desired_steering = steering_degree;
}

void SimulatorImageReceiverTcpCls::PerformIgnition()
{
	// Nothing
}

void SimulatorImageReceiverTcpCls::Stop()
{
	SetSpeed(0);
	SetSteering(0);
}

void SimulatorImageReceiverTcpCls::ReceiveImages()
{	
	int socket_command_sender;
	unsigned int addr_len;
	int bytes_read_left, bytes_read_right;
	unsigned char recv_data_left[MAX_BYTES+1] = {0};
	unsigned char recv_data_right[MAX_BYTES+1] = {0};
	sockaddr_in client_addr;
	sockaddr_in client_addr_command;

	///////////////
	addr_len = sizeof(sockaddr);

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
		//if (keep_old_left == false)
		{
			//bytes_read_left = recvfrom(socket_left, recv_data_left, MAX_BYTES, 0, (sockaddr *)&client_addr, &addr_len);
			bytes_read_left = recv(socket_left, recv_data_left, MAX_BYTES, 0);
		}
		//if (keep_old_right == false)
		{
			bytes_read_right = recv(socket_right, recv_data_right, MAX_BYTES, 0);
		}

		int frame_counter_left = (int)recv_data_left[0];
		int frame_counter_right = (int)recv_data_right[0];

		ROS_INFO("SimulatorImageReceiverTcpCls::ReceiveImages Left count: %d, Right count: %d, Left bytes: %d, Right bytes: %d", frame_counter_left, frame_counter_right, bytes_read_left, bytes_read_right);
		
		if (bytes_read_left > 0 && bytes_read_right > 0)
		{
			/*
			FILE* fl = fopen("/tmp/left.jpg","wb");
			fwrite(recv_data_left, sizeof(char), bytes_read_left, fl);
			fclose(fl);

			FILE* fr = fopen("/tmp/right.jpg","wb");
			fwrite(recv_data_right, sizeof(char), bytes_read_right, fr);
			fclose(fr);

			image_left = cv::imread("/tmp/left.jpg", 1);
			image_right = cv::imread("/tmp/right.jpg", 1);
			*/

			std::vector<char> data_left(recv_data_left, recv_data_left + bytes_read_left);
			std::vector<char> data_right(recv_data_right, recv_data_right + bytes_read_right);
			image_left = cv::imdecode(data_left, -1);
			image_right = cv::imdecode(data_right, -1);
			//cv::imwrite("/tmp/left.jpg", image_left);
			ROS_INFO("Image resolution: %dx%d", image_left.cols, image_left.rows);
			frame_counter++;
			if (frame_counter_left != frame_counter_right)
			{
				if (frame_counter_left > frame_counter_right)
				{
					ROS_WARN("SimulatorImageReceiverTcpCls::ReceiveImages Frame counters not match, keeping left. Global: %d, Left: %d, Right: %d", frame_counter, frame_counter_left, frame_counter_right);
					keep_old_right = false;
					keep_old_left = true;
				}
				else
				{
					ROS_WARN("SimulatorImageReceiverTcpCls::ReceiveImagesFrame counters not match, keeping right. Global: %d, Left: %d, Right: %d", frame_counter, frame_counter_left, frame_counter_right);
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
				ROS_WARN("SimulatorImageReceiverTcpCls::ReceiveImages Image sizes not match. Left: %dx%d, Right: %dx%d", image_left.cols, image_left.rows, image_right.cols, image_right.rows);
				continue;
			}

			if (width != image_left.cols || height != image_left.rows)
			{
				width = image_left.cols;
				height = image_left.rows;
				ROS_INFO("SimulatorImageReceiverTcpCls::ReceiveImages New image resolution: %dx%d", width, height);
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
			// Only use it when needed. Trying to do it when not needed casuses memory leakage too.
			//image_left.deallocate();
			//image_right.deallocate();
		}
		else
		{
			ROS_WARN("SimulatorImageReceiverTcpCls::ReceiveImages Image bytes don't match skipping. Left: %d, Right: %d", bytes_read_left, bytes_read_right);
		}
		
		float send_data[2] = {desired_speed, desired_steering};
		ROS_INFO("SimulatorImageReceiverTcpCls::ReceiveImages Send speed: %f, sent steer: %f", send_data[0], send_data[1]);		
		sendto(socket_command_sender, (void*)send_data, sizeof(send_data), 0, (sockaddr *)&client_addr_command, sizeof(sockaddr));

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
	close (socket_left);
	close (socket_right);
	close (socket_command_sender);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "SimulatorImageReceiverTcp");
	SimulatorImageReceiverTcpCls simulator_image_receiver_tcp;
	//ros::spin();
	ros::spinOnce();
	int socket_left_server;
	int socket_right_server;
	sockaddr_in receive_left;
	sockaddr_in receive_right;
	memset(&receive_left, sizeof(sockaddr_in), 0);
	memset(&receive_right, sizeof(sockaddr_in), 0);
	socket_left = simulator_image_receiver_tcp.ListenOnSocket(socket_left_server, LEFT_RECEIVE_PORT, receive_left);
	socket_right = simulator_image_receiver_tcp.ListenOnSocket(socket_right_server, RIGHT_RECEIVE_PORT, receive_right);
	ROS_INFO("socket_left: %d, socket_right: %d", socket_left, socket_right);

	// unsigned char recv_data_left[MAX_BYTES+1] = {0};
	// while (ros::ok()) {
	// 	bytes_read_left = read(socket_left2, recv_data_left, 100);
	// 	int x = recv(socket_left2, recv_data_left, 100, 0);
	// 	if (bytes_read_left > 0) {
	// 		ROS_INFO("bytes_read_left: %d %d", bytes_read_left, x);
	// 	}
	// }
	simulator_image_receiver_tcp.ReceiveImages();
	exit(0);
}

