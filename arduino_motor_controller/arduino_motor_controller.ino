// Arduino-PID-Library - Version: Latest
// #include <PID_v1.h>
// Copied and modified from:
// https://github.com/sungjik/my_personal_robotic_companion/blob/master/my_personal_robotic_companion/arduino/motor_controller/motor_controller.ino
// Arduino motor controller for differential two motored robot
// Also reads encoders, only does open-loop, does not useencoders for PID closed-loop
// Uses ROS for Android over serial port

#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

// Encoder 0
#define encoder0PinA  0
#define encoder0PinB  4

// Encoder 1
#define encoder1PinA  1
#define encoder1PinB  5

// Motor 0
#define motor0Pwm 9
#define motor0Dir 10

// Motor 1
#define motor1Pwm 11
#define motor1Dir 12
#define LOOPTIME  100   // PID loop time(ms)


double alpha = 0.45;
double alpha_rpm = 0.1;
// Each turn of right wheel is this many counts of encoder pulses in encoder0Pos.
int encoder_pulse_right = 121;
// Each turn of left wheel is this many counts of encoder pulses in encoder1Pos.
int encoder_pulse_left = 121;
// Ratio between encoder counts and actual turns. 1.0 means this is not used and
// end result encode_pulse values
double gear_ratio = 1.0;

volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;

ros::NodeHandle  nh;
geometry_msgs::Vector3Stamped rpm_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg);
ros::Time current_time;
ros::Time last_time;

char base_link[] = "/base_link";
unsigned int counter = 0;
unsigned int global_counter = 0;
unsigned long lastMilli = 0;       // loop timing
unsigned long lastMilliPub = 0;
float motor0 = 0;
float motor1 = 0;
long encoder0PosOld = 0;
long encoder1PosOld = 0;
double rpm_x = 0;
double rpm_y = 0;


void setSpeed(int motorNo, int speed) {
  if (motorNo == 0) {
    int dir0;
    if (speed > 0) {
      dir0 = LOW;
      motor0 = motor0 * (1-alpha) + speed*alpha;
    } else {
      dir0 = HIGH;
      motor0 = motor0 * (1-alpha) + -speed*alpha;
    }
    analogWrite(motor0Pwm, (int)motor0);
    digitalWrite(motor0Dir, dir0);
  } else if (motorNo == 1) {
    int dir1;
    if (speed > 0) {
      dir1 = LOW;
      motor1 = motor1 * (1-alpha) + speed*alpha;
    } else {
      dir1 = HIGH;
      motor1 = motor1 * (1-alpha) + -speed*alpha;
    }
    analogWrite(motor1Pwm, (int)motor1);
    digitalWrite(motor1Dir, dir1);
  }
}

void motors_cb( const geometry_msgs::Vector3& cmd_msg) {
  // nh.loginfo("Motor callback");
  int x = (int)((255.0f) * cmd_msg.x);
  int y = (int)((255.0f) * cmd_msg.y);
  if (x > 255) {
    x = 255;
  } else if (x < -255) {
    x = -255;
  }
  if (y > 255) {
    y = 255;
  } else if (y < -255) {
    y = -255;
  }
  setSpeed(0, x);
  setSpeed(1, y);
  counter = 0;

}

void setup()
{
  pinMode(encoder0PinA, INPUT_PULLUP);
  digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT_PULLUP);
  digitalWrite(encoder0PinB, HIGH);       // turn on pull-up resistor
  //digitalPinToInterrupt
  attachInterrupt(encoder0PinA, doEncoder0, CHANGE);  // encoder pin on interrupt 0 - pin 2

  pinMode(encoder1PinA, INPUT_PULLUP);
  digitalWrite(encoder1PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder1PinB, INPUT_PULLUP);
  digitalWrite(encoder1PinB, HIGH);       // turn on pull-up resistor
  //digitalPinToInterrupt
  attachInterrupt(encoder1PinA, doEncoder1, CHANGE);  // encoder pin on interrupt 0 - pin 2

  pinMode(motor0Pwm, OUTPUT);
  pinMode(motor0Dir, OUTPUT);
  pinMode(motor1Pwm, OUTPUT);
  pinMode(motor1Dir, OUTPUT);

  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.loginfo("Start");
  ros::Subscriber<geometry_msgs::Vector3> subMotor("/motors", motors_cb);
  ros::Subscriber<geometry_msgs::Vector3> subMotor2("/test", motors_cb);

  nh.subscribe(subMotor);
  nh.subscribe(subMotor2);

  nh.advertise(rpm_pub);
}

void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = nh.now();
  rpm_x = rpm_x * (1.0 - alpha_rpm) + double((encoder0Pos-encoder0PosOld)*60*1000)/double(time*encoder_pulse_right*gear_ratio) * alpha;
  rpm_y = rpm_y * (1.0 - alpha_rpm) + double((encoder1Pos-encoder1PosOld)*60*1000)/double(time*encoder_pulse_left*gear_ratio) * alpha_rpm;
  rpm_msg.vector.x = rpm_x;
  rpm_msg.vector.y = rpm_y;
  //rpm_msg.vector.x = double((encoder0Pos-encoder0PosOld)*60*1000)/double(time*encoder_pulse_right*gear_ratio);
  //rpm_msg.vector.y = double((encoder1Pos-encoder1PosOld)*60*1000)/double(time*encoder_pulse_left*gear_ratio);
  //rpm_msg.vector.x = encoder0Pos;
  //rpm_msg.vector.y = encoder1Pos;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  encoder0PosOld = encoder0Pos;
  encoder1PosOld = encoder1Pos;
  nh.spinOnce();
  //nh.loginfo(String(String(encoder0Pos)+ " - " + String(encoder1Pos)).c_str());
}
void loop()
{
  global_counter++;
  // If no new motor command given in 5s, stop motors
  if (counter > 285) {
    setSpeed(0, 0);
    setSpeed(1, 0);
  } else {
    counter++;
  }

  nh.spinOnce();
  unsigned long time = millis();
  if(time-lastMilli>= LOOPTIME)   {      // enter tmed loop

    publishRPM(time-lastMilli);
    lastMilli = time;
  }
  if(time-lastMilliPub >= LOOPTIME) {
    //  publishRPM(time-lastMilliPub);
    lastMilliPub = time;
  }
  // Without a delay motors don't respond ??
  delay(10);
}

/* read a rotary encoder with interrupts
   Encoder hooked up with common to GROUND,
   encoder0PinA to pin 2, encoder0PinB to pin 4 (or pin 3 see below)
   it doesn't matter which encoder pin you use for A or B

   uses Arduino pull-ups on A & B channel outputs
   turning on the pull-ups saves having to hook up resistors
   to the A & B channel outputs

*/

void doEncoder0() {
  /* If pinA and pinB are both high or both low, it is spinning
     forward. If they're different, it's going backward.

     For more information on speeding up this process, see
     [Reference/PortManipulation], specifically the PIND register.
  */
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    //encoder0Pos--;
  }
}

void doEncoder1() {
  /* If pinA and pinB are both high or both low, it is spinning
     forward. If they're different, it's going backward.

     For more information on speeding up this process, see
     [Reference/PortManipulation], specifically the PIND register.
  */
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos++;
  } else {
    //encoder1Pos--;
  }
}
