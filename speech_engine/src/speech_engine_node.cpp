#include <ros/ros.h>
#include <speech_engine/speech_request.h>
#include <std_msgs/String.h>
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <time.h>   // time calls
#include <ros/console.h>
#include <festival.h>

using namespace std;

class SpeechEngineCls
{
public:
	SpeechEngineCls();

private:
	void speechRequestCallback(const speech_engine::speech_request::ConstPtr& speech_request);

	ros::NodeHandle nh_;

	ros::Subscriber speech_sub_;
	ros::Publisher speech_string_pub_;
};

SpeechEngineCls::SpeechEngineCls()
{
	int options = 0;
	void* user_data;
	char *path=NULL;
	printf("Initializing speech...");
	speech_sub_ = nh_.subscribe < speech_engine::speech_request> ("speech_engine/speech_request", 10, &SpeechEngineCls::speechRequestCallback, this);

	speech_string_pub_ = nh_.advertise<std_msgs::String>("/speech_engine/speech_request_string", 1);

	ros::Duration(3).sleep();

    festival_initialize(1,210000);
    festival_say_text("I am alive!");
	printf("Initializing speech done.");
}

void SpeechEngineCls::speechRequestCallback(const speech_engine::speech_request::ConstPtr& speech_request)
{
    std_msgs::String msg;
   
    std::stringstream ss;
  	ss << speech_request->speech_request.c_str();
    msg.data = ss.str();
  
	speech_string_pub_.publish(msg);

    festival_say_text(speech_request->speech_request.c_str());
	ROS_INFO("Speaking: \"%s\"", speech_request->speech_request.c_str());	

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "SpeechEngine");
	SpeechEngineCls speech_engine;
	ros::spin();
}

