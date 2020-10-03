#ifndef VIDEOCAPTURE_H
#define VIDEOCAPTURE_H
#include <raspicam/raspicam_cv.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "config.h"

class ImageCapture
{
public:
    ImageCapture();
    void operator>>(cv::Mat& image);

private:
    raspicam::RaspiCam_Cv m_camera;
};

#endif // VIDEOCAPTURE_H
