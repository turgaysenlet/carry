// Arduino-PID-Library - Version: Latest 
// #include <PID_v1.h>

/*
   rosserial Time and TF Example
   Publishes a transform at current time
*/

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>

#include <tf/transform_broadcaster.h>

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

float alpha = 0.5f;

volatile unsigned int encoder0Pos = 0;
volatile unsigned int encoder1Pos = 0;

ros::NodeHandle  nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";
char odom[] = "/odom";
unsigned int counter = 0;
unsigned int global_counter = 0;
float motor0 = 0;
float motor1 = 0;

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
  nh.loginfo("Motor callback");
  t.transform.rotation.w = 1.2;
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

  //Serial.begin (9600);
  //Serial.println("start");                

  nh.initNode();
  broadcaster.init(nh);
  nh.loginfo("Start");
  ros::Subscriber<geometry_msgs::Vector3> subMotor("/motors", motors_cb);
ros::Subscriber<geometry_msgs::Vector3> subMotor2("/test", motors_cb);
  
  nh.subscribe(subMotor);
  nh.subscribe(subMotor2);
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
  
  // if (Output1 < 0)                                       // decide which way to turn the motor
  // {
  //   Output1a = abs(Output1);
  // }
  // analogWrite(motor0Pwm, Output1a);
  
  t.header.frame_id = odom;
  t.child_frame_id = base_link;

  t.transform.translation.x = encoder0Pos;
  t.transform.translation.y = encoder1Pos;
  t.transform.translation.z = counter;
  t.transform.rotation.x = motor0;
  t.transform.rotation.y = motor1;
  t.transform.rotation.z = 0;
  //t.transform.rotation.w = 1.0;
  t.header.stamp = nh.now();
  t.header.seq = global_counter;
  broadcaster.sendTransform(t);
  nh.spinOnce();
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
