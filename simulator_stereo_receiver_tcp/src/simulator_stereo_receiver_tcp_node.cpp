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

const char* COMMAND_SEND_IP = "10.0.0.108";
const int CAMERA_RECEIVE_PORT = 2000;
const int ODOM_RECEIVE_PORT = 2500;
const int COMMAND_SEND_PORT = 3000;
const int MAX_BYTES = 262143;

struct OdomMessage {
	int OdomSequence;
	int Length;
	float OdomDLeft;
	float OdomDRight;
	float OdomDT;
	// Odometry postion x in meters.
	float OdomPosX;
	// Odometry postion y in meters.
	float OdomPosY;
	// Odometry orientation in degrees.
	float OdomOrientation;
	float Time;
};

class SimulatorImageReceiverTcpCls
{
public:
	SimulatorImageReceiverTcpCls();
	~SimulatorImageReceiverTcpCls();
	void ReceiveImage();
	void ReceiveOdom();
	void Loop();
	void ListenOnOdomUdpSocket();
	void ListenOnCameraTcpSocket();
private:
	ros::Time now;
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	ros::Subscriber speed_sub_;
	ros::Subscriber steering_sub_;
	ros::Subscriber speed_steering_sub_;
	image_transport::Publisher image_raw_pub_;
	image_transport::Publisher depth_pub_;
	image_transport::Publisher disparity_pub_;
	ros::Publisher image_info_pub_;
	ros::Publisher depth_info_pub_;
	ros::ServiceServer service_image_;
	ros::Publisher joint_pub_;
	tf::TransformBroadcaster broadcaster_;

	void speedCallback(const motor_controller::speed::ConstPtr& speed);
	void steeringCallback(const motor_controller::steering::ConstPtr& steering);
	void speedSteeringCallback(const motor_controller::speed_steering::ConstPtr& speed_steering);
	bool ServeCameraRequestLeft(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &res);

	void SetSteering(float steering_degree);
	void SetSpeed(float speed_mps);
	double CalculateMotorServoPositionFromSpeed(float speed);
	void PerformIgnition();
	void Stop();

	float desired_speed;
	float desired_steering;

	int width;
	int height;

	camera_info_manager::CameraInfoManager camera_info_manager_image_;
	cv::Mat image;
	bool first_image;
	int camera_socket;
	int odom_socket;
	unsigned char* recv_data_image;
	int bytes_read_image;
	int bytes_read_odom;
	int frame_counter;
	OdomMessage odom_message;
};

SimulatorImageReceiverTcpCls::~SimulatorImageReceiverTcpCls() {
	ROS_INFO("cleaning up");
	if (recv_data_image != NULL) 
	{
		delete[] recv_data_image;
	}
	if (camera_socket != 0) {
		ROS_INFO("closing image socket...");		
		shutdown(camera_socket, 2);
		close(camera_socket);
		ROS_INFO("closing image socket done.");
	}
	if (odom_socket != 0) {
		ROS_INFO("closing odom socket...");		
		shutdown(odom_socket, 2);
		close(odom_socket);
		ROS_INFO("closing odom socket done.");
	}
}

SimulatorImageReceiverTcpCls::SimulatorImageReceiverTcpCls() : it_(nh_), camera_info_manager_image_(nh_, "image_simulation", "")
{	
	recv_data_image = new unsigned char[MAX_BYTES+1];
	desired_speed = 0;
	desired_steering = 0;
	first_image = true;

	camera_info_manager_image_.loadCameraInfo("package://simulator_stereo_receiver_tcp/camera_info/simulator_left_640_400.yaml");	

	//ROS_INFO("Camera Info Load Result: %d", (int)result);
	width = 0;
	height = 0;

	// We need an initial RGB image in case first receive does not succeed.
	image = cv::Mat(400, 1280, CV_8UC3);

	speed_sub_ = nh_.subscribe < motor_controller::speed > ("motor_controller/speed_control", 1, &SimulatorImageReceiverTcpCls::speedCallback, this);
	steering_sub_ = nh_.subscribe < motor_controller::steering > ("motor_controller/steering_control", 1, &SimulatorImageReceiverTcpCls::steeringCallback, this);
	speed_steering_sub_ = nh_.subscribe < motor_controller::speed_steering > ("motor_controller/speed_steering_control", 1, &SimulatorImageReceiverTcpCls::speedSteeringCallback, this);
	image_raw_pub_ = it_.advertise("/stereo/image/image_raw", 1, false);
	depth_pub_ = it_.advertise("/stereo/depth/image_raw", 1, false);
	joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

	image_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/image/camera_info", 1, false);
	depth_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/depth/camera_info", 1, false);

	service_image_ = nh_.advertiseService("/stereo/image/set_camera_info", &SimulatorImageReceiverTcpCls::ServeCameraRequestLeft, this);
	now = ros::Time::now();
}

void SimulatorImageReceiverTcpCls::ListenOnOdomUdpSocket() {
	unsigned int addr_len;
	sockaddr_in server_addr_odom;
	sockaddr_in client_addr;

	// Left UDP receiver
	if ((odom_socket = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		ROS_ERROR("Socket creation problem");
		exit(1);
	}
	fcntl(odom_socket, F_SETFL, O_NONBLOCK); 

	server_addr_odom.sin_family = AF_INET;
	server_addr_odom.sin_port = htons(ODOM_RECEIVE_PORT);
	server_addr_odom.sin_addr.s_addr = INADDR_ANY;
	bzero(&(server_addr_odom.sin_zero),8);

	if (bind(odom_socket,(sockaddr *)&server_addr_odom,
			sizeof(sockaddr)) == -1)
	{
		ROS_ERROR("UDP socket binding problem");
		exit(1);
	}

}
void SimulatorImageReceiverTcpCls::ListenOnCameraTcpSocket() {
	int receive_port = CAMERA_RECEIVE_PORT;
	int sock;
	sockaddr_in server_addr;
	memset(&server_addr, sizeof(sockaddr_in), 0);

	ROS_INFO("ListenOnCameraTcpSocket for port %d...", receive_port);
	sock = socket(AF_INET, SOCK_STREAM, 0);

	// Check to see if we have a valid socket
	if(sock < 0) {
   		int iSocketError = 0;//WSAGetLastError();
   		ROS_ERROR("ListenOnCameraTcpSocket create socket failed with %d", iSocketError);
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
		ROS_ERROR("ListenOnCameraTcpSocket bind socket failed with %d", iSocketError);
		exit(1);
	}

	// Listen for incoming connections
	if(listen(sock, 1) < 0) {
	   int iSocketError = 0;//WSAGetLastError();
   		ROS_ERROR("ListenOnCameraTcpSocket listen socket failed with %d", iSocketError);
		exit(1);
	}

	// Accept incoming connection request
	sockaddr_in sIncomingAddr;
	memset(&sIncomingAddr, 0, sizeof(sockaddr_in));
	unsigned int iAddrLen = sizeof(sockaddr_in);
	int sIncomingSocket = accept(sock, (sockaddr *) &sIncomingAddr, &iAddrLen);
	if(sIncomingSocket < 0) {
   		int iSocketError = 0;//WSAGetLastError();
   		ROS_WARN("ListenOnCameraTcpSocket accept socket failed with %d", iSocketError);
   		// Return false, but we can continue accepting in a polling loop until a connection request comes.
   		return;
	} else {
		ROS_INFO("ListenOnCameraTcpSocket accept socket connection with socket %d from %s port %d",
            sIncomingSocket, inet_ntoa(sIncomingAddr.sin_addr), ntohs(sIncomingAddr.sin_port));
	}

	ROS_INFO("ListenOnCameraTcpSocket for port %d done.", receive_port);
	camera_socket = sIncomingSocket;
	ROS_INFO("camera_socket: %d", camera_socket);	
}


bool SimulatorImageReceiverTcpCls::ServeCameraRequestLeft(sensor_msgs::SetCameraInfo::Request &req,
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

void SimulatorImageReceiverTcpCls::ReceiveImage() 
{
	// Set contents to zero
	memset(recv_data_image, 0, MAX_BYTES+1);

	bytes_read_image = recv(camera_socket, recv_data_image, MAX_BYTES, 0);
	
	frame_counter = *((int*)recv_data_image);
	int size_image = *((int*)(recv_data_image+4));	

	if (bytes_read_image != size_image || size_image == 0) {
		ROS_WARN("Leftcount: %d, Leftbytes: %d/%d", (int)frame_counter, bytes_read_image, size_image);
		return;
	}
	ROS_INFO("Leftcount: %d, Leftbytes: %d/%d", (int)frame_counter, bytes_read_image, size_image);

	/*
	FILE* fl = fopen("/tmp/image.jpg","wb");
	fwrite(recv_data_image, sizeof(char), bytes_read_image, fl);
	fclose(fl);

	image = cv::imread("/tmp/image.jpg", 1);
	*/

	if (first_image || (size_image > 0 && size_image <= bytes_read_image))
	{
		std::vector<char> data_image(recv_data_image+8, recv_data_image + size_image);
		image = cv::imdecode(data_image, -1);
		first_image = false;
		ROS_INFO("Image: %dx%d - %d", image.cols, image.rows, size_image);
	}
	
	//cv::imwrite("/tmp/image.jpg", image);
	ROS_INFO("Image resolution: %dx%d", image.cols, image.rows);
	
	if (width != image.cols || height != image.rows)
	{
		width = image.cols;
		height = image.rows;
		ROS_INFO("New image resolution: %dx%d", width, height);
	}
}

void SimulatorImageReceiverTcpCls::ReceiveOdom() 
{
	OdomMessage temp_odom_message;
	// Set contents to zero
	memset(&temp_odom_message, 0, sizeof(OdomMessage));

	bytes_read_odom = recv(odom_socket, &temp_odom_message, sizeof(OdomMessage), 0);
	
	if (bytes_read_odom != temp_odom_message.Length || bytes_read_odom != (int)sizeof(OdomMessage)) {
		ROS_WARN("Odomseq: %d, Odombytes: %d/%d, x: %0.2f, y: %0.2f", (int)temp_odom_message.OdomSequence, (int)bytes_read_odom, (int)sizeof(OdomMessage), temp_odom_message.OdomPosX, temp_odom_message.OdomPosY);
		return;
	}
	ROS_INFO("Odomseq: %d, Odombytes: %d/%d, x: %0.2f, y: %0.2f", (int)temp_odom_message.OdomSequence, (int)bytes_read_odom, (int)sizeof(OdomMessage), temp_odom_message.OdomPosX, temp_odom_message.OdomPosY);
	odom_message = temp_odom_message;
}

void SimulatorImageReceiverTcpCls::Loop()
{	
	ros::Rate loop_rate(50);
	int socket_command_sender;
	unsigned int addr_len;
	sockaddr_in client_addr;
	sockaddr_in client_addr_command;

	///////////////
	addr_len = sizeof(sockaddr);

	// message declarations
	geometry_msgs::TransformStamped odom_trans;
	sensor_msgs::JointState joint_state;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";
	odom_trans.header.stamp = now;
	
	while (ros::ok())
	{
		now = ros::Time::now();
		ReceiveImage();
		ReceiveOdom();

		std_msgs::Header header;
		header.seq = frame_counter;
		header.frame_id = "camera";
		header.stamp = now;

		sensor_msgs::Image image_message;
		cv_bridge::CvImage cvimage(header, enc::BGR8, image);
		cvimage.toImageMsg(image_message);

		sensor_msgs::CameraInfo camera_info = camera_info_manager_image_.getCameraInfo();
		camera_info.header.frame_id = "camera";
		camera_info.header.stamp = now;

		image_raw_pub_.publish(image_message);
		image_info_pub_.publish(camera_info);

		sensor_msgs::Image image_message_depth;

		cv::Mat image_depth;
		cvtColor(image, image_depth, cv::COLOR_RGB2GRAY);
		image_depth.convertTo(image_depth, CV_16U, 1, 0);

		cv_bridge::CvImage cvimage_depth(header, enc::TYPE_16UC1, 100*image_depth);
		cvimage_depth.toImageMsg(image_message_depth);

		depth_pub_.publish(image_message_depth);
		depth_info_pub_.publish(camera_info);
		
		// Release images using deallocate (not release) otherwise memory leakge happens after imread
		// Only use it when needed. Trying to do it when not needed casuses memory leakage too.
		//image.deallocate();
		
		float send_data[2] = {desired_speed, desired_steering};
		//ROS_INFO("Send speed: %f, sent steer: %f", send_data[0], send_data[1]);		
		sendto(socket_command_sender, (void*)send_data, sizeof(send_data), 0, (sockaddr *)&client_addr_command, sizeof(sockaddr));

		//update joint_state
		joint_state.header.stamp = now;
		joint_state.name.resize(3);
		joint_state.position.resize(3);
		joint_state.name[0] ="swivel";
		joint_state.position[0] = 0;
		joint_state.name[1] ="tilt";
		joint_state.position[1] = 0;
		joint_state.name[2] ="periscope";
		joint_state.position[2] = 0;

		// update transform
		odom_trans.header.stamp = now;
		odom_trans.transform.translation.x = odom_message.OdomPosX;
		odom_trans.transform.translation.y = odom_message.OdomPosY;
		odom_trans.transform.translation.z = 0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(odom_message.OdomOrientation/180.0f*3.14159266f);

		//send the joint state and transform
		joint_pub_.publish(joint_state);
		broadcaster_.sendTransform(odom_trans);

		// Check messages
		ros::spinOnce();

		// This will adjust as needed per iteration
		loop_rate.sleep();
	}
	close (camera_socket);
	close (socket_command_sender);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "SimulatorImageReceiverTcp");
	SimulatorImageReceiverTcpCls simulator_stereo_receiver_tcp;
	ros::spinOnce();
	simulator_stereo_receiver_tcp.ListenOnOdomUdpSocket();
	simulator_stereo_receiver_tcp.ListenOnCameraTcpSocket();
	simulator_stereo_receiver_tcp.Loop();
	exit(0);
}

