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

const char* HOST_IP = "192.168.1.70";
const int HOST_PORT = 4100;
int sock_image_sender;
sockaddr_in client_addr_image;
class ImageSenderCls
{
public:
	ImageSenderCls();	
private:
	ros::NodeHandle nh_;
	//image_transport::ImageTransport it_;
	//image_transport::Subscriber< stereo_msgs::DisparityImage> image_sub_;	
	ros::Subscriber image_sub_;
	void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg);
	//void imageCallback(const stereo_msgs::DisparityImage& image_msg);
	int Width;
	int Height;
};

ImageSenderCls::ImageSenderCls()// : it_(nh_)
{
	image_sub_ = nh_.subscribe("/stereo/left/image_raw", 1, &ImageSenderCls::imageCallback, this);	
	////////////////////
	// UDP command sender
	if ((sock_image_sender = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		ROS_ERROR("Socket creation problem!");
		exit(1);
	}
	hostent *host;
	host = (hostent *) gethostbyname((char *)HOST_IP);
	client_addr_image.sin_family = AF_INET;
	client_addr_image.sin_port = htons(HOST_PORT);
	client_addr_image.sin_addr = *((in_addr *)host->h_addr);
	bzero(&(client_addr_image.sin_zero),8);	
	ROS_INFO("Image sender socket created.");	
}

void ImageSenderCls::imageCallback(
	//const stereo_msgs::DisparityImage& image_msg)
	const sensor_msgs::Image::ConstPtr& image_msg)
{
	ROS_INFO("Image received.");		
	//cv_bridge::CvImageConstPtr cv_ptr_disp = cv_bridge::toCvShare(image_msg->image, image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
	cv_bridge::CvImageConstPtr cv_ptr_disp = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_8UC3);
	const cv::Mat image = cv_ptr_disp->image;	
	ROS_INFO("Image converted.");	
	ROS_INFO("Image size %dx%d", image.cols, image.rows);
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(75);
	//cv::Mat rgb_image;
	//cvtColor(image, rgb_image, CV_GRAY2BGR );
	//ROS_INFO("Disparity image color converted.");	
	//cv::imwrite("/tmp/disp.jpg", rgb_image);
	//cv::imwrite("/tmp/disp.jpg", image);
	vector<uchar> buff;	
	cv::imencode(".jpg", image, buff, compression_params);
	ROS_INFO("Image encoded.");		
	sendto(sock_image_sender, (void*)buff.data(), buff.size(), 0, (sockaddr *)&client_addr_image, sizeof(sockaddr));
	ROS_INFO("Image sent.");
	// Release images using deallocate (not release) otherwise memory leakge happens after imread
	//image.deallocate();
	ROS_INFO("Image deallocated.");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ImageSender");
	ImageSenderCls image_sender;
	ros::spin();
	close (sock_image_sender);
}

