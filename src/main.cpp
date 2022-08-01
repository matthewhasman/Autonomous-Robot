#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <pid.hpp>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible

#define LEFT_MOTOR_0 PA_0
#define LEFT_MOTOR_1 PA_1

#define RIGHT_MOTOR_2 PA_2
#define RIGHT_MOTOR_3 PA_3

#define IR_L PB1
#define IR_R PB0

#define LEFT PA_5
#define RIGHT PA_4
#define PROPORTIONAL PA_6
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int potent;
double kp; //this value should be entered using a potentiometer
double kd; //this value should be entered using a potentiometer
double last_error = 0; 
int p; //proportional error value sent to motor
int d; //Derivative error value sent to motor
int right_val; //ir reading for right sensor
int left_val; //ir reading for left sensor
int right_speed; //ir reading for right sensor
int left_speed; //ir reading for left sensor
int prev_right_speed; //previous duty cycle for speed of right motor
int prev_left_speed; //previous duty cycle for speed of left motor
int max_kp_val = 700;
int startSpeed = 1850; 
int turnSpeed = 1100;
int prevTime = millis();
int state = 1; 
int clock_freq = 75;
int initial_time;
boolean leftOn = false;
boolean rightOn = false;

boolean prev_left = false;
boolean prev_right = false;
boolean turn = false;


void setup() {
    pinMode(LEFT, INPUT_PULLUP);
    pinMode(RIGHT_REFLECT, INPUT_PULLUP);
    pinMode(PROPORTIONAL, INPUT_PULLUP);
    pinMode(IR_L, INPUT_PULLUP);
    pinMode(IR_R, INPUT_PULLUP);
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
    kd = 5*((double) potent/1022);

    delay(3000);
    initial_time = millis();
}

void loop() {

  while(true) {
  int currState; 

  int currTime = millis();

  if (currTime - initial_time > 5300) {
    startSpeed = 1750;
  }

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
  pwm_start(RIGHT_MOTOR_2, clock_freq, right_speed, RESOLUTION_12B_COMPARE_FORMAT);

  int left_val = analogRead(LEFT) - 200;
  int right_val = analogRead(RIGHT_REFLECT);

  leftOn = false;
  rightOn = false;

  if (left_val > 450) {
    leftOn = true;
  }

  if (right_val > 450) {
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
        x = 5.5;

    }

    // Right last on
    else {
        x = -5.5;
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

  last_error = x; 

  int error = left_val - right_val;

  display_handler.print("kp: ");
  display_handler.println(kp);
  display_handler.print("Left_val: ");
  display_handler.println(left_val);
  display_handler.print("Right_val: ");
  display_handler.println(right_val);

  display_handler.print("Tape Following");

  display_handler.print("numHighs (L): ");
  display_handler.println(num_high_L);
  display_handler.print("numHighs (R): ");
  display_handler.println(num_high_R); 

  display_handler.print("IR: ");
  display_handler.println(analogRead(IR_L));
  display_handler.print("IR: ");
  display_handler.println(analogRead(IR_R));
  display_handler.display();

  }

  while (true) {

  display_handler.clearDisplay();
  display_handler.setCursor(0,0);

  startSpeed = 1600;

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
  //kd = 5*((double) potent/1022);

   display_handler.print("numHighs (L): ");
  display_handler.println(num_high_L);
  display_handler.print("numHighs (R): ");
  display_handler.println(num_high_R); 

  display_handler.print("IR: ");
  display_handler.println(analogRead(IR_L));
  display_handler.print("IR: ");
  display_handler.println(analogRead(IR_R));
  display_handler.print("kp: ");
  display_handler.println(313);
  display_handler.print("IR Following");
  display_handler.display();

  if (leftOn == true || rightOn == true) {
    prev_right = rightOn;
    prev_left = leftOn;
  }

  pwm_start(LEFT_FWD, clock_freq, left_speed, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_MOTOR_2, clock_freq, right_speed, RESOLUTION_12B_COMPARE_FORMAT);

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

  //int g = p + d;

  left_speed = min(max(startSpeed - p, 0), 4095);
  right_speed = min(max(startSpeed + p, 0), 4095);

  state = currState; 

  last_error = x; 

  int error = left_val - right_val;

  }
}