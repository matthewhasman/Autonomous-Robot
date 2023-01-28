#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <NewPing.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET  -1 // This display does not have a reset pin accessible

#define SERVO_PIN PB_9

#define LEFT_FWD PA_0 
#define LEFT_REV PA_1 

#define RIGHT_REV PA_2 
#define RIGHT_FWD PA_3 

#define IR_L PB1
#define IR_R PB0

#define LEFT_REFLECT PA_5
#define RIGHT_REFLECT PA_4

#define PROPORTIONAL PA_6 // Dial to control proportional gain of PID

#define SELECT_A PA11 // Multiplexer pins to select sonar readings
#define SELECT_B PA10
#define SELECT_C PA9

#define TRIGGER_PIN PB4
#define ECHO_PIN PB3
#define MAX_DISTANCE 200
#define MAX_DISTANCE_1 50

#define COLLISION_SWITCH_1 PB11


Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Define variables: 

double kp; // Proportional Gain
int potent; // Potentiometer output determined kp
int max_kp_val = 700;

int p; // Proportional error value sent to motor
int right_val; //ir reading for right sensor
int left_val; //ir reading for left sensor
int right_speed; // Relative voltage sent to the right motor
int left_speed; // Relative voltage sent to the left motor

int startSpeed = 2400; 
int prevTime = millis();
int state = 1; // State 1 is the robot following tape
int clock_freq = 75;
int initial_time;
int lastPing = 0;

// Variables for treasure collection
int leftSpeedTurn = 2550;
int rightWheelLeftTurn = 2550;
int rightSpeedTurn = 2550;
int leftWheelRightTurn = 2550;

int leftStraight = 1800;
int rightStraight = 1800;

void reverse_movements(boolean, boolean, int,int);
void treasure_retrevial();
boolean verification();

/**
 * The following varibles are relevant when including a differential response for the PID control: 
int prev_right_speed; previous duty cycle for speed of right motor
int prev_left_speed; previous duty cycle for speed of left motor
int d; Derivative error value sent to motor
double kd; this value should be entered using a potentiometer
double last_error = 0; 
 * 
*/

// Two reflectance sensors determine whether the robot is on top of the tape. These booleans report the state of each sensor
boolean leftOn = false;
boolean rightOn = false;

// The previous states are recorded to correct the robot if it fully exits the tape
boolean prev_left = false;
boolean prev_right = false;

boolean turn = false;
boolean arch = false; 

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
    pinMode(LEFT_REFLECT, INPUT_PULLUP);
    pinMode(RIGHT_REFLECT, INPUT_PULLUP);
    pinMode(PROPORTIONAL, INPUT_PULLUP);
    pinMode(IR_L, INPUT_PULLUP);
    pinMode(IR_R, INPUT_PULLUP);
    pinMode(SELECT_A, OUTPUT);
    pinMode(SELECT_B, OUTPUT);
    pinMode(SELECT_C, OUTPUT);
    pinMode(SERVO_PIN, OUTPUT);
    pinMode(COLLISION_SWITCH_1, INPUT_PULLUP);

    
    display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    
    // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
    display_handler.display();
     
    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);

    right_speed = startSpeed;
    left_speed = startSpeed;

    potent = analogRead(PROPORTIONAL);
    
    kp = max_kp_val*((double) potent/1022);
    // kd = 5*((double) potent/1022);

    delay(3000);
    initial_time = millis();
    Serial.begin(9600);

}

void loop() {

pwm_start(SERVO_PIN, 50, 400, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT); //repoen arm
  while(true) {

  display_handler.clearDisplay();
  display_handler.setCursor(0,0);

  // Use sonar sensors to detect treasures:
  if(millis()-lastPing >=50) { // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.

    // RIGHT

    // Set the multiplexers to the correct select values to choose which sonar sensor to read:
    digitalWrite(SELECT_A, HIGH);
    digitalWrite(SELECT_B, LOW);
    digitalWrite(SELECT_C, LOW);
    display_handler.print("Right Distance: ");
    display_handler.display();
    unsigned int right_distance = sonar.ping(MAX_DISTANCE_1) / US_ROUNDTRIP_CM; // Send ping, get ping time in microseconds (uS).

    // can lower/raise threshold of cm
    delay(50);
    lastPing = millis();

    if (right_distance <=20 && right_distance > 5) {
      // Verification function ensures that the sensor properly detects the idol. Sonar sensor is unreliable and needs filtering
      if(verification()){
          treasure_retrevial();
      }
    }
  }

  // The robot passed through a narrow arch. The following corrects the path of the robot if the arch is collided with. 
  // ARCH IS HIT: 
  if (digitalRead(COLLISION_SWITCH_1) && !arch) {

    //stop robot
    pwm_start(LEFT_FWD, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_FWD, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    delay(50);

    //reverse robot
    pwm_start(LEFT_REV, clock_freq, 2800, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_REV, clock_freq, 2800, RESOLUTION_12B_COMPARE_FORMAT);
    delay(500);

    //turn robot
    pwm_start(LEFT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);

    pwm_start(LEFT_REV, clock_freq, 3400, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_FWD, clock_freq, 3400, RESOLUTION_12B_COMPARE_FORMAT);
    delay(200);

    pwm_start(LEFT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(LEFT_FWD, clock_freq, 2500, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_FWD, clock_freq, 2500, RESOLUTION_12B_COMPARE_FORMAT);
    arch = true; 
    break;
  }

  int currState; 

  int currTime = millis();
  display_handler.clearDisplay();
  display_handler.setCursor(0,0);

  int num_high_L = 0;
  int num_high_R = 0;
  

  for (int i = 0; i < 10; i++) {

    if (analogRead(IR_L) > 620) {
      ++num_high_L;
    }
    if (analogRead(IR_R) > 620) {
      ++num_high_R;
    }
  }

  if (num_high_L > 2 || num_high_R > 2) {
    break; 
  }

  double x = 0;

  potent = analogRead(PROPORTIONAL);
  kp = max_kp_val*((double) potent/1022);
  //kd = 5*((double) potent/1022);

  if (leftOn == true || rightOn == true) {
    prev_right = rightOn;
    prev_left = leftOn;
  }

  pwm_start(LEFT_FWD, clock_freq, left_speed, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_FWD, clock_freq, right_speed, RESOLUTION_12B_COMPARE_FORMAT);

  int left_val = analogRead(LEFT_REFLECT);
  int right_val = analogRead(RIGHT_REFLECT);

  leftOn = false;
  rightOn = false;

  if (left_val > 150) {
    leftOn = true;
  }

  if (right_val > 150) {
    rightOn = true;
  }

  // Left on, right off
  if (leftOn == true && rightOn == false) {
    x = 1;
    currState = 2;
  }

  // Right on, left off
  if (leftOn == false && rightOn == true) {
    x = -1;
    currState = 3;
  }

  // Go straight
  if (leftOn == true && rightOn == true) {
    x = 0;
    currState = 1;
  }

  // Both off tape:
  if (leftOn == false && rightOn == false) {
    turn = true; 
    currState = 4;
    
    
    // Left last on
    if (prev_left == true) {
        x = 6;

    }

    // Right last on
    else {
        x = -6;
    }
  }

  if (currState != state) {
    prevTime = millis();
  }

  int dt = millis() - prevTime; 

  p = (int) kp*x;
  //d = (int) kd*((x - last_error)/(dt));

  //int g = p + d;

  left_speed = min(max(startSpeed - p, 0), 4095);
  right_speed = min(max(startSpeed + p, 0), 4095);

  state = currState; 

  int error = left_val - right_val;
  display_handler.display();
  }

  while (true) {

  // This loop manages the IR detection and PID

  // ARCH IS HIT: 
  if (digitalRead(COLLISION_SWITCH_1)) {

    //stop robot
    pwm_start(LEFT_FWD, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_FWD, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    delay(50);

    //reverse robot
    pwm_start(LEFT_REV, clock_freq, 2800, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_REV, clock_freq, 2800, RESOLUTION_12B_COMPARE_FORMAT);
    delay(500);

    //turn robot
    pwm_start(LEFT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);

    pwm_start(LEFT_REV, clock_freq, 3400, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_FWD, clock_freq, 3400, RESOLUTION_12B_COMPARE_FORMAT);
    delay(500);

    pwm_start(LEFT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(LEFT_FWD, clock_freq, 2500, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_FWD, clock_freq, 2500, RESOLUTION_12B_COMPARE_FORMAT);
  }

  startSpeed = 2400;

  int num_high_L = 0;
  int num_high_R = 0;
  

  for (int i = 0; i < 100; i++) {

    if (analogRead(IR_L) > 620) {
      ++num_high_L;
    }
    if (analogRead(IR_R) > 620) {
      ++num_high_R;
    }
  }

  int currState; 
  display_handler.clearDisplay();
  display_handler.setCursor(0,0);

  double x = 0;

  potent = analogRead(PROPORTIONAL);
  kp = 213;

  display_handler.print("IR Following");
  display_handler.display();

  if (leftOn == true || rightOn == true) {
    prev_right = rightOn;
    prev_left = leftOn;
  }

  pwm_start(LEFT_FWD, clock_freq, left_speed, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_FWD, clock_freq, right_speed, RESOLUTION_12B_COMPARE_FORMAT);

  int left_val = num_high_L;
  int right_val = num_high_R;

  leftOn = false;
  rightOn = false;

  if (left_val > 15) {
    leftOn = true;
  }

  if (right_val > 15) {
    rightOn = true;
  }

  // Left on, right off
  if (leftOn == true && rightOn == false) {
    x = 1;
    currState = 2;
  }

  // Right on, left off
  if (leftOn == false && rightOn == true) {
    x = -1;
    currState = 3;
  }

  // Go straight
  if (leftOn == true && rightOn == true) {
    x = 0;
    currState = 1;
  }

  // Both off tape:
  if (leftOn == false && rightOn == false) {
    turn = true; 
    currState = 4;
    
    
    // Left last on
    if (prev_left == true) {
        x = 2.5;

    }

    // Right last on
    else {
        x = -2.5;
    }
  }

  if (currState != state) {
    prevTime = millis();
  }

  int dt = millis() - prevTime; 

  p = (int) kp*x;
  //d = (int) kd*((x - last_error)/(dt));

  left_speed = min(max(startSpeed - p, 0), 4095);
  right_speed = min(max(startSpeed + p, 0), 4095);

  state = currState; 

  int error = left_val - right_val;

  }
}

void treasure_retrevial(){
    int turnTime;
    int straightTime; 
    boolean rightTurn = true;
    boolean leftTurn = false; 
    // trigger slowing down of robot
    pwm_start(SERVO_PIN, 50, 2300, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT); //repoen arm

    rightTurn = true;
    leftTurn = false;

    delay(50);

    // digitalWrite to front pin
    digitalWrite(SELECT_A, HIGH);
    digitalWrite(SELECT_B, HIGH);
    digitalWrite(SELECT_C, LOW);

    unsigned int distance_front;
    int turnStartTime = millis();
    pwm_start(LEFT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_REV, clock_freq, rightWheelLeftTurn, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(LEFT_FWD, clock_freq, leftSpeedTurn, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_FWD, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    delay(1500);
  
    turnTime = millis() - turnStartTime;
    int straightStartTime = millis();
    distance_front = sonar.ping_median(3, MAX_DISTANCE_1)/US_ROUNDTRIP_CM;
    while(distance_front >= 0){
      // when front finally close enough --> starting going in straight
      pwm_start(LEFT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(RIGHT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);

      pwm_start(LEFT_FWD, clock_freq, leftStraight, RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(RIGHT_FWD, clock_freq, rightStraight, RESOLUTION_12B_COMPARE_FORMAT);

      display_handler.clearDisplay();
      display_handler.setCursor(0,0);
      display_handler.println("Backing Up Time");
      // delay(50); //TODO: maybe replace bc its blocking code
      distance_front = sonar.ping_median(5, MAX_DISTANCE_1)/US_ROUNDTRIP_CM;
      display_handler.print("Front Distance: ");
      display_handler.print(distance_front); 
      display_handler.println("cm");
      display_handler.print(digitalRead(COLLISION_SWITCH_1));
      display_handler.display();
      straightTime = millis() - straightStartTime;

      if(digitalRead(COLLISION_SWITCH_1)){
          pwm_start(LEFT_FWD, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(RIGHT_FWD, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);

          pwm_start(LEFT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
          pwm_start(RIGHT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
 
          // trigger the picking up of treasures thru claw by servo
          display_handler.println("COLLISION!");
          display_handler.display();

          // going slow to detect the bomb
          for(int j = 2200; j > 1200; j--) {
              pwm_start(SERVO_PIN, 50, j, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT);
              delay(2);
          }
          delay(200); // detect bomb during this time

          // if(digitalRead(HALL_EFFECT_SENSOR) == LOW){
          //   pwm_start(SERVO_PIN, 50, 2300, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT); //repoen arm
          //   break;
          // }

          pwm_start(SERVO_PIN, 50, 2300, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT); //repoen arm
          delay(1000);
          pwm_start(SERVO_PIN, 50, 400, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT); //close arm completely
          delay(1000);
          pwm_start(SERVO_PIN, 50, 2300, TimerCompareFormat_t::MICROSEC_COMPARE_FORMAT); //reset

          // reverse steps to go back on tape.
          delay(50);
          break;
    }
    
  }
  reverse_movements(rightTurn, leftTurn, turnTime, straightTime);
}

void reverse_movements(boolean rightTurn, boolean leftTurn, int turnTime, int straightTime){
  display_handler.println("REVERSING MOTIONS");
  display_handler.display();
  
  pwm_start(LEFT_REV, clock_freq, leftStraight, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_REV, clock_freq, rightStraight, RESOLUTION_12B_COMPARE_FORMAT);

  pwm_start(LEFT_FWD, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_FWD, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
  delay(straightTime);
  delay(300);

  if(rightTurn){
    // this is for turning left
    pwm_start(LEFT_REV, clock_freq, leftWheelRightTurn, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);

    pwm_start(LEFT_FWD, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_FWD, clock_freq, rightSpeedTurn, RESOLUTION_12B_COMPARE_FORMAT);
    
  }
  else if(leftTurn){
    // this is for turning right
    pwm_start(LEFT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_REV, clock_freq, rightWheelLeftTurn, RESOLUTION_12B_COMPARE_FORMAT);

    pwm_start(LEFT_FWD, clock_freq, leftSpeedTurn, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_FWD, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
  }
  delay(turnTime);
  pwm_start(LEFT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);

  pwm_start(LEFT_FWD, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_FWD, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
}

boolean verification(){

  // Median filter is used to ensure the robot actually detects a treasure before beginning the retrieval function.

    pwm_start(LEFT_FWD, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_FWD, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);

    pwm_start(LEFT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(RIGHT_REV, clock_freq, 0, RESOLUTION_12B_COMPARE_FORMAT);
    digitalWrite(SELECT_A, HIGH);
    digitalWrite(SELECT_B, LOW);
    digitalWrite(SELECT_C, LOW);
    display_handler.print("VERIFICATION");
    display_handler.display();
    unsigned int right_distance = sonar.ping_median(20, MAX_DISTANCE_1) / US_ROUNDTRIP_CM; // Send ping, get ping time in microseconds (uS).
    delay(1050);
    return right_distance <= 18 && right_distance > 5;

}
