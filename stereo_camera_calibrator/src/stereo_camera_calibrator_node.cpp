#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
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
class StereoCameraCalibratorCls
{
public:
	StereoCameraCalibratorCls();
	void ReceiveImages();
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	ros::Subscriber left_image_sub_;
	ros::Subscriber right_image_sub_;	
	image_transport::Publisher image_pub_;	
	int Width;
	int Height;
	void imageCallbackLeft(const sensor_msgs::Image::ConstPtr& left_image_msg);
	void imageCallbackRight(const sensor_msgs::Image::ConstPtr& right_image_msg);
    cv::Mat right_image;
};

bool right_image_received = false;
void StereoCameraCalibratorCls::imageCallbackLeft(const sensor_msgs::Image::ConstPtr& left_image_msg)
{
	ROS_INFO("Image received.");
	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(left_image_msg, sensor_msgs::image_encodings::TYPE_8UC3);
	const cv::Mat left_image = cv_ptr->image;
	if (right_image_received) {
		std_msgs::Header header;
		header.stamp = ros::Time::now();
		cv::Mat image = right_image/2 + left_image/2;

		for (int j = 0; j < image.rows; j++)
		{
			if ((j/3) % 2)
			{
				for (int i = 0; i < image.cols; i++)
				{
					int kk = 2;
					cv::Point3_<uchar>* p = image.ptr<cv::Point3_<uchar> >(j,i);
					p->x = p->x/kk;
					p->y = p->y/kk;
					p->z = p->z/kk;
				}
			}
		}
		cv_bridge::CvImage cv_image_ptr(header, enc::RGB8, image);
		image_pub_.publish(cv_image_ptr.toImageMsg());
	}
}

void StereoCameraCalibratorCls::imageCallbackRight(const sensor_msgs::Image::ConstPtr& right_image_msg)
{
	cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(right_image_msg, sensor_msgs::image_encodings::TYPE_8UC3);
	right_image = cv_ptr->image;
	right_image_received = true;
}

StereoCameraCalibratorCls::StereoCameraCalibratorCls() : it_(nh_)
{
	Width = 0;
	Height = 0;
	image_pub_ = it_.advertise("/stereo/image_combined", 1);
	left_image_sub_ = nh_.subscribe("/stereo/left/image_raw", 1, &StereoCameraCalibratorCls::imageCallbackLeft, this);
	right_image_sub_ = nh_.subscribe("/stereo/right/image_raw", 1, &StereoCameraCalibratorCls::imageCallbackRight, this);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "StereoCameraCalibrator");
	StereoCameraCalibratorCls stereo_camera_calibrator;
	ros::spin();
}

/*#include <ros/ros.h>
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

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;

class StereoCameraCalibratorCls
{
public:
	StereoCameraCalibratorCls();
	void ReceiveImages();
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::SubscriberFilter left_image_sub_;
    image_transport::SubscriberFilter right_image_sub_;
	message_filters::Synchronizer< SyncPolicy > sync_;
	void imageCallback(
        const sensor_msgs::ImageConstPtr& left_image_msg,
        const sensor_msgs::ImageConstPtr& right_image_msg,
        );

	image_transport::Subscriber< stereo_msgs::Image> left_image_sub_;
	image_transport::Subscriber< stereo_msgs::Image> right_image_sub_;		

	int Width;
	int Height;
};

StereoCameraCalibratorCls::StereoCameraCalibratorCls() : it_(nh_)
{
	sync_.registerCallback( boost::bind( &StereoCameraCalibratorCls::imageCallback, this, _1, _2 ) );
	desired_speed = 0;
	desired_steering = 0;

	camera_info_manager_left_.loadCameraInfo("package://stereo_camera_calibrator/camera_info/left.yaml");	
	camera_info_manager_right_.loadCameraInfo("package://stereo_camera_calibrator/camera_info/right.yaml");

	//ROS_INFO("Camera Info Load Result: %d", (int)result);
	Width = 0;
	Height = 0;

	speed_sub_ = nh_.subscribe < motor_controller::speed > ("motor_controller/speed_control", 1, &StereoCameraCalibratorCls::speedCallback, this);
	steering_sub_ = nh_.subscribe < motor_controller::steering > ("motor_controller/steering_control", 1, &StereoCameraCalibratorCls::steeringCallback, this);
	speed_steering_sub_ = nh_.subscribe < motor_controller::speed_steering > ("motor_controller/speed_steering_control", 1, &StereoCameraCalibratorCls::speedSteeringCallback, this);
	left_raw_pub_ = it_.advertise("/stereo/left/image_raw", 1, false);
	right_raw_pub_ = it_.advertise("/stereo/right/image_raw", 1, false);
	joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

	if (ProcessStereo)
	{
		disparity_pub_ = it_.advertise("/stereo/disparity", 1, false);
	}

	left_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info", 1, false);
	right_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info", 1, false);
	service_left_ = nh_.advertiseService("/stereo/left/set_camera_info", &StereoCameraCalibratorCls::ServeCameraRequestLeft, this);
	service_right_ = nh_.advertiseService("/stereo/right/set_camera_info", &StereoCameraCalibratorCls::ServeCameraRequestRight, this);
}

bool StereoCameraCalibratorCls::ServeCameraRequestLeft(sensor_msgs::SetCameraInfo::Request &req,
		sensor_msgs::SetCameraInfo::Response &res)
{
	res.success = 1;
	return true;
}

bool StereoCameraCalibratorCls::ServeCameraRequestRight(sensor_msgs::SetCameraInfo::Request &req,
		sensor_msgs::SetCameraInfo::Response &res)
{
	res.success = 1;
	return true;
}

void StereoCameraCalibratorCls::speedCallback(const motor_controller::speed::ConstPtr& speed)
{
	SetSpeed(speed->speed_mps);
}

void StereoCameraCalibratorCls::steeringCallback(const motor_controller::steering::ConstPtr& steering)
{
	SetSteering(steering->degree);
}

void StereoCameraCalibratorCls::speedSteeringCallback(const motor_controller::speed_steering::ConstPtr& speed_steering)
{
	ROS_WARN("Received request: %f, %f", speed_steering->speed_mps.speed_mps, speed_steering->steering_degree.degree);
	SetSpeed(speed_steering->speed_mps.speed_mps);
	SetSteering(speed_steering->steering_degree.degree);
}

void StereoCameraCalibratorCls::SetSpeed(float speed_mps)
{
	desired_speed = speed_mps;
}

void StereoCameraCalibratorCls::SetSteering(float steering_degree)
{
	desired_steering = steering_degree;
}

void StereoCameraCalibratorCls::PerformIgnition()
{
	// Nothing
}

void StereoCameraCalibratorCls::Stop()
{
	SetSpeed(0);
	SetSteering(0);
}

void StereoCameraCalibratorCls::ReceiveImages()
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
			// FILE* fl = fopen("/tmp/left.jpg","wb");
			// fwrite(recv_data_left, sizeof(char), bytes_read_left, fl);
			// fclose(fl);

			// FILE* fr = fopen("/tmp/right.jpg","wb");
			// fwrite(recv_data_right, sizeof(char), bytes_read_right, fr);
			// fclose(fr);

			// image_left = cv::imread("/tmp/left.jpg", 1);
			// image_right = cv::imread("/tmp/right.jpg", 1);
			
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
				// cv::StereoSGBM stereo(0, 64, 5);
				// cv::Mat disp;
				// stereo(image_left, image_right, disp);

				// cv::Mat disp8;
				// disp.convertTo(disp8, CV_8U, 0.25, 0);
				// cv_bridge::CvImage cvimage_disp(header, enc::MONO8, disp8);
				// cvimage_disp.toImageMsg(image_message_disp);
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
	ros::init(argc, argv, "StereoCameraCalibrator");
	StereoCameraCalibratorCls stereo_camera_calibrator;
	//ros::spin();
	ros::spinOnce();
	stereo_camera_calibrator.ReceiveImages();
}
*/
