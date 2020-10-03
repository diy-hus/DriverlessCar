#ifndef LANEDETECT_H
#define LANEDETECT_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "config.h"
#include "types.h"

using namespace cv;

class LaneDetect
{
public:
    void detectWayPoint(const Mat &image);
    WayPoint &GetWayPoints();

private:
    Mat thresholdImage(const Mat &image);
    Mat findLane(const Mat &img);
    void slidingWindow(const Mat &image);
    Mat BirdViewTranform(const Mat &image);

private:
    WayPoint m_wayPoints;
};

#endif // LANEDETECT_H
