#include <costmap_2d/costmap_2d_ros.h>
#include <cv_bridge/cv_bridge.h>
#include <errno.h> // Error number definitions
#include <fcntl.h> // File control definitions
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h> // standard input / output functions
#include <stereo_msgs/DisparityImage.h>
#include <string.h> // string function definitions
#include <termios.h> // POSIX terminal control definitions
#include <tf/transform_listener.h>
#include <time.h>   // time calls
#include <unistd.h> // UNIX standard function definitions

using namespace std;
using namespace ros::console::levels;
using namespace sensor_msgs;
using namespace message_filters;
using namespace stereo_msgs;
namespace enc = sensor_msgs::image_encodings;

const float HEIGHT_CENTER = 0.5f;
const float PLANE_RATIO = 200.0f;

// Forward decleration
class ObstableDetectorCls; 
ObstableDetectorCls* obstable_detector = NULL;

void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
	//Process images
	if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols) {
		mono8_img = cv::Mat(float_img.size(), CV_8UC1);
	}
	cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
	//The following doesn't work due to NaNs
	//double minVal, maxVal; 
	//minMaxLoc(float_img, &minVal, &maxVal);
	//ROS_DEBUG("Minimum/Maximum Depth in current image: %f/%f", minVal, maxVal);
	//mono8_img = cv::Scalar(0);
	//cv::line( mono8_img, cv::Point2i(10,10),cv::Point2i(200,100), cv::Scalar(255), 3, 8);
}

class ObstableDetectorCls
{
public:
	ObstableDetectorCls();
	void disparityCallback(const stereo_msgs::DisparityImage::ConstPtr& disp, 
		const sensor_msgs::CameraInfo::ConstPtr& info);
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	ros::Publisher camera_info_pub_;
	image_transport::Publisher top_view_pub_;
	image_transport::Publisher top_view_rotated_pub_;
	image_transport::Publisher depth_pub_;
	image_transport::Publisher depth_pub2_;
	std::string frame;
	//ros::Subscriber disp_image_sub_;
	//ros::Subscriber camera_info_sub_;
	
	//void disparityCallback(const stereo_msgs::DisparityImage::ConstPtr& disp_image_msg);		
	int Width;
	int Height;
};

// Global function, just to get around error 'ISO C++ forbids taking the address of 
// an unqualified or parenthesized non-static member function to form a pointer to 
// member function. Say ‘&ObstableDetectorCls::disparityCallback’ [-fpermissive]'
void disparityCallbackBase(const stereo_msgs::DisparityImage::ConstPtr& disp, 
	const sensor_msgs::CameraInfo::ConstPtr& info)
{
	ROS_INFO("Hello, again.");
	if (obstable_detector != NULL) 
	{
		obstable_detector->disparityCallback(disp, info);
	}
}

ObstableDetectorCls::ObstableDetectorCls() : it_(nh_)
{
	ROS_INFO("Constructor start...");
	frame = "camera";
	message_filters::Subscriber<DisparityImage> disp_image_sub(nh_, "/stereo/disparity", 10);
	disp_image_sub.unsubscribe();

	message_filters::Subscriber<CameraInfo> camera_info_sub(nh_, "/stereo/left/camera_info", 10);
	camera_info_sub.unsubscribe();

	typedef sync_policies::ApproximateTime<DisparityImage, CameraInfo> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), disp_image_sub, camera_info_sub);
   	sync.setAgePenalty(1000);
    sync.registerCallback(boost::bind(&disparityCallbackBase, _1, _2));
  	
  	camera_info_pub_ = nh_.advertise<CameraInfo>("stereo/camera_info", 1);

	top_view_pub_ = it_.advertise("stereo/top_view", 1);
	top_view_rotated_pub_ = it_.advertise("stereo/top_view_rotated", 1);
	depth_pub_ = it_.advertise("stereo/depth", 1);
	depth_pub2_ = it_.advertise("stereo/depth2", 1);
	ROS_INFO("Constructor done.");
}

	//void ObstableDetectorCls::disparityCallback(
		//const stereo_msgs::DisparityImage& disp_image_msg)
		//const stereo_msgs::DisparityImage::ConstPtr& disp_image_msg)
void ObstableDetectorCls::disparityCallback(
	const stereo_msgs::DisparityImage::ConstPtr& disp_image_msg, 
	const sensor_msgs::CameraInfo::ConstPtr& info)
{
	ROS_INFO("Disparity image received.");
	//cv_bridge::CvImageConstPtr cv_ptr_disp = cv_bridge::toCvShare(disp_image_msg->image, disp_image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
	cv_bridge::CvImageConstPtr cv_ptr_disp = cv_bridge::toCvCopy(disp_image_msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
	const cv::Mat disp_image = cv_ptr_disp->image;	
	ROS_INFO("Disparity image converted.");	
	ROS_INFO("Image size %dx%d", disp_image.cols, disp_image.rows);
	// Top view grayscale image
	cv::Mat depth_image(disp_image.rows, disp_image.cols, CV_32FC1, cv::Scalar(0));
	// Top view grayscale image, only part that will be converted to top view (a middle slice)
	cv::Mat depth_image2(disp_image.rows, disp_image.cols, CV_32FC1, cv::Scalar(0));
	cv::Mat top_view_image(64, 64, CV_32FC1, cv::Scalar(0));
	float xk = (float)disp_image.cols / (float)64;
	float yk = (float)disp_image.rows / (float)64;
	float dk = 256.0f;
	
	//disp_image.convertTo(depth_image, CV_32FC1, 1.0, 0);
	
	for (int j = 0; j < depth_image.rows; j++)
	{		
		float jj = (j - depth_image.rows / 2.0f) / (depth_image.rows / 2.0f);
		for (int i = 0; i < depth_image.cols; i++)
		{
			float d = 1.0f / disp_image.at<float>(j, i);
			depth_image.at<float>(j, i) = d;
			if (j < depth_image.rows * HEIGHT_CENTER)
			{
				depth_image2.at<float>(j, i) = 0;
			}
			else 
			{								
				if (disp_image.at<float>(j, i) / jj > PLANE_RATIO)
				{
					depth_image2.at<float>(j, i) = d;
					int y = (int)(d*dk);
					if (y < 0) y = 0;
					if (y > top_view_image.cols - 1) y = top_view_image.cols - 1;
					int x = (int)((depth_image2.cols / 2 - i) / xk + top_view_image.cols / 2);
					if (x < 0) x = 0;
					if (x > top_view_image.cols - 1) x = top_view_image.cols - 1;
					
					top_view_image.at<float>(y, x) += 0.2f;
				} 
				else 
				{
					depth_image2.at<float>(j, i) = 0;
				}
			}
		}
	}
	
	//cvtColor(disp_image, rgb_image, CV_GRAY2BGR );
	
	//ROS_INFO("Disparity image color converted.");		
	//cv::imwrite("/tmp/disp.jpg", disp_image);
	std_msgs::Header header;
	header.stamp = ros::Time::now();
	header.frame_id = frame;
	//Top view image
	cv::Mat top_mono_image;
	cv::Mat depth_mono_image;
	cv::Mat depth_mono_image2;
	//cvtColor(top_view_image, top_rgb_image, CV_GRAY2BGR );
	depthToCV8UC1(top_view_image, top_mono_image);
	cv_bridge::CvImage cv_top_view_ptr(header, enc::MONO8, top_mono_image);
	//top_view_pub_.publish(cv_top_view_ptr.toImageMsg());
	top_view_pub_.publish(cv_top_view_ptr.toImageMsg());	

	//Rotated top view image
	cv::Mat top_view_rotated_mono_image;	
	cv::flip(top_mono_image, top_view_rotated_mono_image, 1);
	cv_bridge::CvImage cv_top_view_rotated_ptr(header, enc::MONO8, top_view_rotated_mono_image);
	top_view_rotated_pub_.publish(cv_top_view_rotated_ptr.toImageMsg());	

	//cv_bridge::CvImage cv_depth_ptr(header, enc::TYPE_32FC1, depth_image);	
	depthToCV8UC1(depth_image, depth_mono_image);
	cv_bridge::CvImage cv_depth_ptr(header, enc::MONO8, depth_mono_image);	
	depth_pub_.publish(cv_depth_ptr.toImageMsg());
	
	depthToCV8UC1(depth_image2, depth_mono_image2);
	cv_bridge::CvImage cv_depth_ptr2(header, enc::MONO8, depth_mono_image2);	
	depth_pub2_.publish(cv_depth_ptr2.toImageMsg());
	
	//CameraInfo c1 = *info;
	//camera_info_pub_.publish(c1);
	// CameraInfoPtr info_right(new CameraInfo(right_info_mgr.getCameraInfo()));

 //  info_left->header.stamp = time;
 

 //  left_info_pub.publish(info_left);
 //  right_info_pub.publish(info_right);
	//disp_image.deallocate();
	//ROS_INFO("Depth image published.");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ObstableDetector");
	ROS_INFO("Hello.");
	obstable_detector = new ObstableDetectorCls();
	ros::spin();
	delete obstable_detector;
}

