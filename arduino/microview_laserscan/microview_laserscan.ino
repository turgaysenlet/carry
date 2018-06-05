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

void scan_cb( const std_msgs::String& cmd_msg) {
  
  a++;
  a %= 100;
  widget1->setValue(a++);
  uView.display();
}

ros::Subscriber<std_msgs::String> sub("/scan_string", scan_cb);

void setup() {
  nh.initNode();
  // nh.advertise(chatter);
  // put your setup code here, to run once:
  uView.begin();
  uView.clear(PAGE);      // clear page
  widget1 = new MicroViewSlider(0,0,0,100);   // declare widget0 as a Slider at x=0, y=0, min=0, max=100
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:  
  //widget1->setValue(a);
  //uView.display();
  a %= 100;
  //uView.line(10,10,100,100);
  delay(10);
  nh.spinOnce();
}

