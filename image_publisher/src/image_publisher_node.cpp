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

using namespace std;
using namespace ros::console::levels;
namespace enc = sensor_msgs::image_encodings;

const float rate = 5.0f;

class ImagePublisherCls
{
public:
	ImagePublisherCls();	
	void PublishImage();
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;	
	image_transport::Publisher image_pub_;
};

ImagePublisherCls::ImagePublisherCls() : it_(nh_)
{
	image_pub_ = it_.advertise("/usb_cam/image_raw", 1);
}

void ImagePublisherCls::PublishImage()
{	
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	cv::Mat image = cv::imread("/tmp/image.png", 1);
	ROS_INFO("Image read: %dx%d.", image.cols, image.rows);
	cv_bridge::CvImage cv_image_ptr(header, enc::BGR8, image);
	image_pub_.publish(cv_image_ptr.toImageMsg());	
	//cv_bridge::CvImage cv_depth_ptr(header, enc::TYPE_32FC1, depth_image);	
	ROS_INFO("Image published.");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ImagePublisher");
	ImagePublisherCls image_publisher;
	ros::Rate loop_rate(rate);
	while (ros::ok())
	{			
		image_publisher.PublishImage();
		loop_rate.sleep();
		ros::spinOnce();
	}
}
