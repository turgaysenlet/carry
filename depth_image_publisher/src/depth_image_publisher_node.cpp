#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stereo_msgs/DisparityImage.h>
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
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

using namespace std;
using namespace ros::console::levels;
namespace enc = sensor_msgs::image_encodings;

class DepthImagePublisherCls
{
public:
	DepthImagePublisherCls();	
private:
	ros::NodeHandle nh_;
	ros::Subscriber disp_image_sub_;
	image_transport::ImageTransport it_;	
	image_transport::Publisher image_pub_;
	void disparityCallback(const stereo_msgs::DisparityImage::ConstPtr& disp_image_msg);
	int Width;
	int Height;
};

DepthImagePublisherCls::DepthImagePublisherCls() : it_(nh_)
{
	image_pub_ = it_.advertise("/stereo/disparity2", 1);
	//image_pub_ = it_.advertise("/stereo/left/image_raw", 1);
	disp_image_sub_ = nh_.subscribe("/stereo/disparity", 1, &DepthImagePublisherCls::disparityCallback, this);	
}

void DepthImagePublisherCls::disparityCallback(
	const stereo_msgs::DisparityImage::ConstPtr& disp_image_msg)
{
	ROS_INFO("Disparity image received.");		
	cv_bridge::CvImageConstPtr cv_ptr_disp = cv_bridge::toCvShare(disp_image_msg->image, disp_image_msg, enc::TYPE_32FC1);
	//cv_bridge::CvImageConstPtr cv_ptr_disp = cv_bridge::toCvCopy(disp_image_msg->image, enc::TYPE_32FC1);
	const cv::Mat disp_image = cv_ptr_disp->image;	
	ROS_INFO("Image size %dx%d", disp_image.cols, disp_image.rows);
	
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	cv::Mat image;
	//cv_bridge::CvImage cv_image_ptr(header, enc::TYPE_32FC1, disp_image);
  cv::Mat rgb_image;
	//cvtColor(disp_image, rgb_image, CV_GRAY2BGR);
	//cv_bridge::CvImage cv_image_ptr(header, enc::BGR8, rgb_image);
	disp_image.convertTo(rgb_image,CV_8U);
	cv_bridge::CvImage cv_image_ptr(header, enc::MONO8, rgb_image);
	image_pub_.publish(cv_image_ptr.toImageMsg());	
	ROS_INFO("Disparity published.");
	//disp_image.deallocate();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "DepthImagePublisher");
	DepthImagePublisherCls depth_image_publisher;
	ros::spin();
}

