#ifndef APPLICATION_H
#define APPLICATION_H
#include <PCD8544.h>

#include "imagecapture.h"
#include "carcontrol.h"
#include "detectlane.h"
#include "config.h"

class Application
{
public:
    Application();

    void Start();

    float PID(float error);

private:
    ImageCapture m_capture;
    DetectLane m_detector;
    CarControl m_car;

    float m_integral = 0;
    float m_preError;
};

#endif // APPLICATION_H
