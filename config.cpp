#include "config.h"

int Config::LANE_WIDTH = 100;

int Config::WIDTH = 320;
int Config::HEIGHT = 240;

float Config::VELOCITY = 50;

int Config::SKY_LINE = 120;

int Config::CANNY_LOW = 25;
int Config::CANNY_HIGH = 50;

int Config::LOW_H = 0;
int Config::LOW_S = 0;
int Config::LOW_V = 0;

int Config::HIGH_H = 180;
int Config::HIGH_S = 255;
int Config::HIGH_V = 100;   // Do sang moi truong

// Button Pin

int Config::BTN1 = 15;

int Config::BTN2 = 7;

int Config::BTN3 = 16;

int Config::BTN4 = 12;

// SoftPWM Pin

int Config::softPWMLeft1 = 21;

int Config::softPWMLeft2 = 30;

int Config::softPWMRight1 = 24;

int Config::softPWMRight2 = 23;

// Servo Pin

int Config::STEER_SERVO = 12;

int Config::PROX_PIN = 1;

float Config::kP = 0.9f;
float Config::kI = 0.1f;
float Config::kD = 0.4f;

