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

unsigned int global_counter = 0;
int motor0 = 0;
int motor1 = 0;

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

int last0;
int last1;
int last_counter = 0;
int T = 500;  
void speedTest(int x, int y, int t) {
  if (global_counter == 1+t-T) {    
    last0 = encoder0Pos;
    last1 = encoder1Pos; 
    last_counter = global_counter; 
    analogWrite(motor0Pwm, x);
    analogWrite(motor1Pwm, y);
  } else if (global_counter == t) {
    Serial.println("Speed: " + String(x) + "/" +String(y) + ", Time: " + String(global_counter-last_counter+1) + ", " + String(encoder0Pos-last0) + ", " + String(encoder1Pos-last1));    
    analogWrite(motor0Pwm, 0);
    analogWrite(motor1Pwm, 0);
  }
}
void loop()
{   
  global_counter++;
  speedTest(50, 0, T);
  speedTest(0, 50, 2*T);
  speedTest(55, 0, 3*T);
  speedTest(0, 55, 4*T);
  speedTest(58, 0, 5*T);
  speedTest(0, 58, 6*T);
  speedTest(60, 0, 7*T);
  speedTest(0, 60, 8*T);
  speedTest(70, 0, 9*T);
  speedTest(0, 70, 10*T);
  speedTest(80, 0, 11*T);
  speedTest(0, 80, 12*T);
  speedTest(90, 0, 13*T);
  speedTest(0, 90, 14*T);
  speedTest(100, 0, 15*T);
  speedTest(0, 100, 16*T);
  speedTest(110, 0, 17*T);
  speedTest(0, 110, 18*T);
  speedTest(120, 0, 19*T);
  speedTest(0, 120, 20*T);
  speedTest(130, 0, 21*T);
  speedTest(0, 130, 22*T);
  speedTest(140, 0, 23*T);
  speedTest(0, 140, 24*T);
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

