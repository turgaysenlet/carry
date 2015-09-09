#include <ros/ros.h>
#include <SpeechEngine/SpeechRequest.h>
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <time.h>   // time calls
#include <ros/console.h>
#include "speak_lib.h"

using namespace std;

class SpeechEngineCls
{
public:
	SpeechEngineCls();

private:
	void speechRequestCallback(const SpeechEngine::SpeechRequest::ConstPtr& speech_request);

	ros::NodeHandle nh_;

	ros::Subscriber speech_sub_;
};

SpeechEngineCls::SpeechEngineCls()
{
	int options = 0;
	int result = espeak_Initialize(AUDIO_OUTPUT_SYNCH_PLAYBACK, 10, NULL, options);
	printf("Speech initialization result: %d\n", result);
	unsigned int uid = 0;
	espeak_Synth("Ready", 5, 0, POS_SENTENCE, 0, espeakCHARS_8BIT, &uid, NULL);
	printf("Speech UID: %d\n", (int)uid);
	//nh_.param("axis_linear", linear_, linear_);
	//nh_.param("axis_angular", angular_, angular_);
	//nh_.param("scale_angular", a_scale_, a_scale_);
	//nh_.param("scale_linear", l_scale_, l_scale_);
	speech_sub_ = nh_.subscribe < SpeechEngine::SpeechRequest> ("SpeechEngine/speech_request", 10, &SpeechEngineCls::speechRequestCallback, this);
}

void SpeechEngineCls::speechRequestCallback(const SpeechEngine::SpeechRequest::ConstPtr& speech_request)
{
	unsigned int uid = 0;
	espeak_Synth(speech_request->speech_request.c_str(), speech_request->speech_request.size(), 0, POS_SENTENCE, 0, espeakCHARS_8BIT, &uid, NULL);
	ROS_INFO("Speaking: \"%s\"", speech_request->speech_request.c_str());
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "SpeechEngine");
	SpeechEngineCls speech_engine;
	ros::spin();
}

