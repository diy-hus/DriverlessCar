#include "imagecapture.h"

ImageCapture::ImageCapture()
{
    m_camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    m_camera.setCaptureSize(Config::WIDTH, Config::HEIGHT);

    if (!m_camera.open()) {
        std::cerr<< "Error opening the camera" << std::endl;
    }
}

void ImageCapture::operator>>(cv::Mat& image)
{
//    static cv::VideoCapture capture("video.avi");
//    capture >> image;
//    resize(image, image, cv::Size(320, 240));
    m_camera.grab();
    m_camera.retrieve(image);
}
