#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>
#include <algorithm>

#include "config.h"

using namespace std;
using namespace cv;

class DetectLane
{
public:
    DetectLane();
    ~DetectLane();

    Mat update(const Mat &src);
    float getErrorAngle();

    static Vec4i UndefinedLine;

    bool HasLane();

    bool IsStop();

private:
    Vec4i m_leftLane;
    Vec4i m_rightLane;
    Vec4i m_stopLane;

    Mat preProcess(const Mat &src);
    Mat binaryImage(const Mat &src);

    void fitLane2Line(const Mat &src, vector<Vec4i>& laneLines, vector<Vec4i>& stopLines);
    void findLane(const vector<Vec4i> &lines);
    void findStopLane(const vector<Vec4i> &lines);

    float errorAngle(const Point &dst);

    Vec4i m_preLane;

    Point m_carPos;
};

#endif
