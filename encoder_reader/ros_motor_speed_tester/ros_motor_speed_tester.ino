/* 
 * rosserial Time and TF Example
 * Publishes a transform at current time
 */

#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>

// Encoder 0
#define encoder0PinA  2
#define encoder0PinB  4

// Encoder 1
#define encoder1PinA  3
#define encoder1PinB  5

// Motor 0
#define motor0Pwm 9
#define motor0Dir 10

// Motor 1
#define motor1Pwm 11
#define motor1Dir 12

volatile unsigned int encoder0Pos = 0;
volatile unsigned int encoder1Pos = 0;

ros::NodeHandle  nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";
char odom[] = "/odom";
unsigned int counter = 0;
unsigned int global_counter = 0;
int motor0 = 0;
int motor1 = 0;

void motors_cb(int x, int y){
  int dir0;
  int dir1;
  if (x > 0) { 
    dir0 = LOW;
    motor0 = x;  
  } else {
    dir0 = HIGH;
    motor0 = -x;
  }
  if (y > 0) { 
    dir1 = LOW;
    motor1 = y;  
  } else {
    dir1 = HIGH;
    motor1 = -y;
  }  
  analogWrite(motor0Pwm, motor0);
  analogWrite(motor1Pwm, motor1);
  digitalWrite(motor0Dir, dir0);
  digitalWrite(motor1Dir, dir1);
  counter = 0;
}

void setup()
{
  pinMode(encoder0PinA, INPUT); 
  digitalWrite(encoder0PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder0PinB, INPUT); 
  digitalWrite(encoder0PinB, HIGH);       // turn on pull-up resistor
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0, CHANGE);  // encoder pin on interrupt 0 - pin 2

  pinMode(encoder1PinA, INPUT); 
  digitalWrite(encoder1PinA, HIGH);       // turn on pull-up resistor
  pinMode(encoder1PinB, INPUT); 
  digitalWrite(encoder1PinB, HIGH);       // turn on pull-up resistor
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1, CHANGE);  // encoder pin on interrupt 0 - pin 2

  //pinMode(motor0Pwm, OUTPUT);
  pinMode(motor0Dir, OUTPUT);
  //pinMode(motor1Pwm, OUTPUT);
  pinMode(motor1Dir, OUTPUT);
  
  Serial.begin (9600);
  Serial.println("start");                // a personal quirk
}

ros::Time now;
int last0;
int last1;
void loop()
{ 
  int T = 280;
  global_counter++;
  
  if (global_counter == 1) {
    last0 = encoder0Pos;
    last1 = encoder1Pos;
    now = nh.now();
    analogWrite(motor0Pwm, 50);
    analogWrite(motor1Pwm, 0); 
  }
  if (global_counter == T) {
    Serial.println(String(encoder0Pos-last0) + ", " + String(encoder1Pos-last1) + ", " + String(nh.now().toSec()-now.toSec()));
    last1 = encoder1Pos;    
    analogWrite(motor0Pwm, 0);
    analogWrite(motor1Pwm, 0);  
  }

  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = encoder0Pos;
  t.transform.translation.y = encoder1Pos;
  t.transform.translation.z = counter;
  t.transform.rotation.x = motor0;
  t.transform.rotation.y = motor1; 
  t.transform.rotation.z = 0.0; 
  t.transform.rotation.w = 1.0;  
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
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
  if (digitalRead(encoder0PinA) == digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }  
}

void doEncoder1() {
  /* If pinA and pinB are both high or both low, it is spinning
   * forward. If they're different, it's going backward.
   *
   * For more information on speeding up this process, see
   * [Reference/PortManipulation], specifically the PIND register.
   */
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Pos++;
  } else {
    encoder1Pos--;
  }  
}

