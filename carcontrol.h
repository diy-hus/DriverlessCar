#ifndef CARCONTROL_H
#define CARCONTROL_H
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>

#include <softPwm.h>
#include <wiringPi.h>

#include <signal.h>
#include <pigpio.h>

#include "config.h"

class CarControl
{
public:
    ~CarControl();

    void Init();
    void SetSpeed(int speed, float bias = 0);
    void Brake();
    void SetSteerAngle(float angle);

private:
    float m_steerAngle;
    int m_speed;
};

#endif // CARCONTROL_H
