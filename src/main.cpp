#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <pid.hpp>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible

#define LEFT_REV PA_0 // rev
#define LEFT_FWD PA_1 // fwd

#define RIGHT_FWD PA_2 // fwd
#define RIGHT_REV PA_3 // rev

#define IR_L PB1
#define IR_R PB0

#define LEFT_REFLECT PA_5
#define RIGHT_REFLECT PA_4
#define PROPORTIONAL PA_6

Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Define variables: 
int potent;
double kp; //this value should be entered using a potentiometer
int p; //proportional error value sent to motor
int right_val; //ir reading for right sensor
int left_val; //ir reading for left sensor
int right_speed; //ir reading for right sensor
int left_speed; //ir reading for left sensor
int max_kp_val = 700;
int startSpeed = 2466; 
int prevTime = millis();
int state = 1; 
int clock_freq = 75;
int initial_time;
int mode = 0; // 0 - Tape following, 1 - IR following

// The following varibles are relevant when including a differential response: 
// int prev_right_speed; previous duty cycle for speed of right motor
// int prev_left_speed; previous duty cycle for speed of left motor
// int d; Derivative error value sent to motor
// double kd; this value should be entered using a potentiometer
// double last_error = 0; 

boolean leftOn = false;
boolean rightOn = false;

boolean prev_left = false;
boolean prev_right = false;
boolean turn = false;


void setup() {
    pinMode(LEFT_REFLECT, INPUT_PULLUP);
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
    // kd = 5*((double) potent/1022);

    delay(3000);
    initial_time = millis();
}

void loop() {

  display_handler.clearDisplay();
  display_handler.setCursor(0,0);

  int currState; 

  int currTime = millis();

  if (currTime - initial_time > 5300) {
    startSpeed = 2333;
  }

  int num_high_L = 0;
  int num_high_R = 0;

  int numSamples = 10;

  if (mode == 1) { // If state is in IR mode, check additional samples of digital reads
    numSamples = 100;
  }

  for (int i = 0; i < numSamples; i++) {

    if (analogRead(IR_L) > 620) {
      ++num_high_L;
    }
    if (analogRead(IR_R) > 620) {
      ++num_high_R;
    }
  }

  // Switch mode to IR if sufficient 10kHz IR is detected
  if (num_high_L > 2 || num_high_R > 2) {
    mode = 1; 
  }

  double x = 0; //error variable for PID

  potent = analogRead(PROPORTIONAL);
  kp = max_kp_val*((double) potent/1022);

  // If at least one sensor is on the tape/IR, update previous states 
  if (leftOn == true || rightOn == true) {
    prev_right = rightOn;
    prev_left = leftOn;
  }

  // Set PWM to desired speeds
  pwm_start(LEFT_FWD, clock_freq, left_speed, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(RIGHT_FWD, clock_freq, right_speed, RESOLUTION_12B_COMPARE_FORMAT);

  // Read new reflectance or IR values

  if (mode == 0) {
    int left_val = analogRead(LEFT_REFLECT);
    int right_val = analogRead(RIGHT_REFLECT);
  }

  else if (mode == 1) {
    int left_val = num_high_L;
    int right_val = num_high_R;
  }

  leftOn = false;
  rightOn = false;

  int threshold;
  int error_threshold;

  // Set threshold depending on whether robot is reading reflectance or number of high IR readings
  if (mode == 0) {
    threshold = 450;
    error_threshold = 4.5;
  }

  else if (mode == 1) {
    threshold = 15;
    error_threshold = 2.5;
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
        x = error_threshold;

    }

    // Right last on
    else {
        x = -1*(error_threshold);
    }
  }

  p = (int) kp*x;

  left_speed = min(max(startSpeed - p, 0), 4095);
  right_speed = min(max(startSpeed + p, 0), 4095);

  int error = left_val - right_val;

  display_handler.print("kp: ");
  display_handler.println(kp);
  display_handler.print("Left_val: ");
  display_handler.println(left_val);
  display_handler.print("Right_val: ");
  display_handler.println(right_val);
  display_handler.print("Mode: ");
  display_handler.println(mode);
  display_handler.print("numHighs (L): ");
  display_handler.println(num_high_L);
  display_handler.print("numHighs (R): ");
  display_handler.println(num_high_R);
  display_handler.print("IR-L: ");
  display_handler.println(analogRead(IR_L));
  display_handler.print("IR-R: ");
  display_handler.println(analogRead(IR_R));
  display_handler.display(); 
}