#include <ArduinoHardware.h>

#include <MicroView.h>

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

int a = 0;

void scan_cb( const std_msgs::String& cmd_msg) {
  uView.clear(PAGE);
  uView.setCursor(0,10);
  uView.print(cmd_msg.data);
  uView.display();
}

ros::Subscriber<std_msgs::String> sub("/test", scan_cb);

void setup() {
  nh.initNode();
  // nh.advertise(chatter);
  // put your setup code here, to run once:
  uView.begin();
  uView.clear(PAGE);      // clear page
  nh.subscribe(sub);
}

void loop() {
  //uView.setCursor(0,10);
  //uView.print("1");
  //uView.display();
  delay(10);
  nh.spinOnce();
}

