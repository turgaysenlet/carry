#include <ArduinoHardware.h>

#include <MicroView.h>

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>

int SCREEN_WIDTH = uView.getLCDWidth();
int SCREEN_HEIGHT = uView.getLCDHeight();
int SHAPE_SIZE = 600;
int ROTATION_SPEED = 0; // ms delay between cube draws
int a = 0;

ros::NodeHandle  nh;
MicroViewWidget *widget1;
bool speaking = false;

void speech_cb( const std_msgs::String& cmd_msg) {
  speaking = true;
  uView.clear(PAGE);
  uView.setCursor(0,8);
  uView.print("speech\n");
  uView.print(cmd_msg.data);
  uView.display();
  delay(1500);
  speaking = false;
}

void scan_cb( const std_msgs::String& cmd_msg) {
  if (speaking) {return;}
  // a++;
  // a %= 100;
  uView.clear(PAGE);
  uView.setCursor(0,8);
  uView.print("laser");
  //widget1->setValue(a++);
  //uView.setCursor(0,50);
  //uView.print(cmd_msg.data);
  float p = 3.0f;
  float t = 45.0f;
  float x = (float)SCREEN_WIDTH/20.0f;
  uView.line(SCREEN_WIDTH/2,SCREEN_HEIGHT,SCREEN_WIDTH/2-3,SCREEN_HEIGHT-3);
  uView.line(SCREEN_WIDTH/2,SCREEN_HEIGHT,SCREEN_WIDTH/2+3,SCREEN_HEIGHT-3);
  uView.line(0,SCREEN_HEIGHT-3,SCREEN_WIDTH,SCREEN_HEIGHT-3);
  for (int i = 2; i < 40; i+=2) {
    int ii = i/2;
    float y0 = (float)(cmd_msg.data[i-2]- 48) * 10.0 + (float)(cmd_msg.data[i-1]- 48);
    float y1 = (float)(cmd_msg.data[i]- 48) * 10.0 + (float)(cmd_msg.data[i+1]- 48);
    uView.line(x*(ii-1),t-p*y0,x*ii,t-p*y1);
  }
  uView.display();
}

ros::Subscriber<std_msgs::String> sub("/scan_string", scan_cb);
ros::Subscriber<std_msgs::String> sub2("/speech_engine/speech_request_string", speech_cb);


void setup() {
  nh.initNode();
  // nh.advertise(chatter);
  // put your setup code here, to run once:
  uView.begin();
  uView.clear(PAGE);      // clear page
  uView.setCursor(0,8);
  uView.print("laser");
  uView.line(SCREEN_WIDTH/2,SCREEN_HEIGHT,SCREEN_WIDTH/2-3,SCREEN_HEIGHT-3);
  uView.line(SCREEN_WIDTH/2,SCREEN_HEIGHT,SCREEN_WIDTH/2+3,SCREEN_HEIGHT-3);
  uView.line(0,SCREEN_HEIGHT-3,SCREEN_WIDTH,SCREEN_HEIGHT-3);  
  uView.display();
  //widget1 = new MicroViewSlider(0,0,0,100);   // declare widget0 as a Slider at x=0, y=0, min=0, max=100
  nh.subscribe(sub);
  nh.subscribe(sub2);
}

void loop() {
  delay(10);
  nh.spinOnce();
}

