#include "carcontrol.h"

namespace {

void StopServo(int signum) {
    gpioServo(Config::STEER_SERVO, 0);
    gpioTerminate();
}

int Angle2Pulse(float angle) {
    int result = 0.19444 * angle * angle + 23.33334 * angle + 1425;
    if (result > 2300)
    {
        result = 2300;
    }
    if (result < 900)
    {
        result = 900;
    }
    return result;
}

}

CarControl::~CarControl()
{
    StopServo(0);
    SetSpeed(0);
}

void CarControl::Init()
{
    if (gpioInitialise() < 0) {
        return;
    }
    gpioSetSignalFunc(SIGINT, StopServo);

    wiringPiSetup();

    softPwmCreate(Config::softPWMLeft1, 0, 100);
    softPwmCreate(Config::softPWMLeft2, 0, 100);
    softPwmCreate(Config::softPWMRight1, 0, 100);
    softPwmCreate(Config::softPWMRight2, 0, 100);

    pinMode (Config::BTN1, INPUT);
    pullUpDnControl (Config::BTN1, PUD_UP) ;

    pinMode (Config::BTN2, INPUT);
    pullUpDnControl (Config::BTN2, PUD_UP) ;

    pinMode (Config::BTN3, INPUT);
    pullUpDnControl (Config::BTN3, PUD_UP) ;

    pinMode (Config::PROX_PIN, INPUT);
    pullUpDnControl (Config::PROX_PIN, PUD_UP);

}

void CarControl::SetSpeed(int speed, float bias)
{
    if (bias > 1)
    {
        bias = 1;
    }
    if (bias < -1)
    {
        bias = -1;
    }
    if (speed > 0) {
        softPwmWrite(Config::softPWMLeft1, speed - bias * 10);
        softPwmWrite(Config::softPWMLeft2, 0);
        softPwmWrite(Config::softPWMRight1, speed + bias * 10);
        softPwmWrite(Config::softPWMRight2, 0);
    } else {
        speed = -speed;
        softPwmWrite(Config::softPWMLeft1, 0);
        softPwmWrite(Config::softPWMLeft2, speed - bias * 10);
        softPwmWrite(Config::softPWMRight1, 0);
        softPwmWrite(Config::softPWMRight2, speed + bias * 10);
    }
}

void CarControl::Brake()
{
    SetSpeed(1);
    time_sleep(0.1);
    SetSpeed(0);
}

void CarControl::SetSteerAngle(float angle)
{
    int pulseWidth = Angle2Pulse(angle);
    gpioServo(Config::STEER_SERVO, pulseWidth);
}
