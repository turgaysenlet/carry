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

class ObstableDetectorCls
{
public:
	ObstableDetectorCls();	
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;	
	image_transport::Publisher top_view_pub_;
	image_transport::Publisher depth_pub_;
	ros::Subscriber disp_image_sub_;
	void disparityCallback(const stereo_msgs::DisparityImage::ConstPtr& disp_image_msg);
	
	int Width;
	int Height;
};

ObstableDetectorCls::ObstableDetectorCls() : it_(nh_)
{
	disp_image_sub_ = nh_.subscribe("/stereo/disparity", 1, &ObstableDetectorCls::disparityCallback, this);	
	top_view_pub_ = it_.advertise("stereo/top_view", 1);
	depth_pub_ = it_.advertise("stereo/depth", 1);
}

void ObstableDetectorCls::disparityCallback(
	//const stereo_msgs::DisparityImage& disp_image_msg)
	const stereo_msgs::DisparityImage::ConstPtr& disp_image_msg)
{
	ROS_INFO("Disparity image received.");		
	//cv_bridge::CvImageConstPtr cv_ptr_disp = cv_bridge::toCvShare(disp_image_msg->image, disp_image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
	cv_bridge::CvImageConstPtr cv_ptr_disp = cv_bridge::toCvCopy(disp_image_msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
	const cv::Mat disp_image = cv_ptr_disp->image;	
	ROS_INFO("Disparity image converted.");	
	ROS_INFO("Image size %dx%d", disp_image.cols, disp_image.rows);
	// Top view grayscale image
	cv::Mat depth_image(384, 512, CV_32FC1, cv::Scalar(0));	
	cv::Mat top_view_image(64, 64, CV_32FC1, cv::Scalar(0));	
	
	//disp_image.convertTo(depth_image, CV_32FC1, 1.0, 0);
	
	for (int j = 0; j < depth_image.rows; j++)
	{		
		float jj = (j - depth_image.rows / 2.0f) / (depth_image.rows / 2.0f);
		for (int i = 0; i < depth_image.cols; i++)
		{
			if (j < depth_image.rows / 1.9f)
			{
				depth_image.at<float>(j, i) = 0;
			}
			else 
			{								
				if (disp_image.at<float>(j, i) / jj > 35)
				{
					float d = 1.0f / disp_image.at<float>(j, i);
					depth_image.at<float>(j, i) = d;
					int y = (int)(d*128.0f);
					if (y < 0) y = 0;
					if (y > 63) y = 63;
					int x = i / 8;
					if (x < 0) x = 0;
					if (x > 63) x = 63;
					
					top_view_image.at<float>(y, x) += 0.2f;
				} 
				else 
				{
					depth_image.at<float>(j, i) = 0;
				}
			}
		}
	}
	

	//cvtColor(disp_image, rgb_image, CV_GRAY2BGR );
	//ROS_INFO("Disparity image color converted.");		
	//cv::imwrite("/tmp/disp.jpg", disp_image);
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	cv_bridge::CvImage cv_top_view_ptr(header, enc::TYPE_32FC1, top_view_image);	
	top_view_pub_.publish(cv_top_view_ptr.toImageMsg());	
	cv_bridge::CvImage cv_depth_ptr(header, enc::TYPE_32FC1, depth_image);	
	depth_pub_.publish(cv_depth_ptr.toImageMsg());
	//disp_image.deallocate();
	//ROS_INFO("Depth image published.");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ObstableDetector");
	ObstableDetectorCls obstable_detector;
	ros::spin();
}

