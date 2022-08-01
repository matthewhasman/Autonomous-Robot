#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <pid.hpp>

#define OUTPUT_1 PA_0 //bluepill output controlling left wheel
#define OUTPUT_2 PA_2 //bluepill output controlling right wheel

/*
* Controls speed of left motor
* Input: value should be a value between 0 and 4095
*/
void leftSpeed(int value) {
    int clock_freq = 1000;

    pwm_start(OUTPUT_1, clock_freq, value, RESOLUTION_12B_COMPARE_FORMAT);
}

/*
* Controls speed of right motor
* Input: value should be a value between 0 and 4095
*/
void rightSpeed(int value) {
    int clock_freq = 1000;

    pwm_start(OUTPUT_2, clock_freq, value, RESOLUTION_12B_COMPARE_FORMAT);
}