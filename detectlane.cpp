#include "detectlane.h"

void swap(int *a, int *b)
{
    int temp = *a;
    *a = *b;
    *b = temp;
}

float lineLength(Vec4i line)
{
    return sqrt((line[0] - line[2]) * (line[0] - line[2]) + (line[1] - line[3]) * (line[1] - line[3]));
}

float lineAngle(Vec4i line)
{
    float X = line[2] - line[0];
    float Y = line[3] - line[1];
    float angle = atan2(Y, X) * 180 / CV_PI;
    if (angle > 90)
    {
        angle -= 180;
    }
    if (angle < -90)
    {
        angle += 180;
    }
    return angle;
}

float distance(float x1, float y1, float x2, float y2)
{
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

Point getPointInLine(Vec4i line, float y)
{
    return Point((y - line[1]) * (line[0] - line[2]) / (line[1] - line[3]) + line[0], y);
}

bool isIntersect(Vec4i a, Vec4i b)
{
    int head1 = getPointInLine(a, Config::HEIGHT / 2).x;
    int head2 = getPointInLine(b, Config::HEIGHT / 2).x;
    int tail1 = getPointInLine(a, Config::HEIGHT).x;
    int tail2 = getPointInLine(b, Config::HEIGHT).x;
    return (head1 > head2 && tail1 < tail2) ||
            (head1 < head2 && tail1 > tail2) ||
            (abs(head1 - head2) < Config::LANE_WIDTH / 2);
}

struct Dist1
{
    bool operator()(Vec4i a, Vec4i b)
    {
        float angle1 = lineAngle(a);
        float angle2 = lineAngle(b);
        float dist = min(distance(a[0], a[1], b[0], b[1]), distance(a[2], a[3], b[2], b[3]));
        dist = min(dist, distance(a[0], a[1], b[2], b[3]));
        dist = min(dist, distance(a[2], a[3], b[0], b[1]));

        return (abs(angle1 - angle2) < 10) && (dist < 20);
    }
};

struct Dist2
{
    bool operator()(Vec4i a, Vec4i b)
    {
        float angle1 = lineAngle(a);
        float angle2 = lineAngle(b);
        float dist = min(distance(a[0], a[1], b[0], b[1]), distance(a[2], a[3], b[2], b[3]));
        dist = min(dist, distance(a[0], a[1], b[2], b[3]));
        dist = min(dist, distance(a[2], a[3], b[0], b[1]));

        return (abs(angle1 - angle2) < 5) && (dist < 50);
    }
};

Vec4i DetectLane::UndefinedLine = Vec4i();

DetectLane::DetectLane()
{
    m_carPos.x = 167;
    m_carPos.y = Config::HEIGHT;

    m_preLane = Vec4i(Config::WIDTH / 2, Config::HEIGHT / 2, Config::WIDTH / 2, Config::HEIGHT);
}

float DetectLane::errorAngle(const Point &dst)
{
    if (dst.x == m_carPos.x)
        return 0;
    if (dst.y == m_carPos.y)
        return (dst.x < m_carPos.x ? -90 : 90);

    double dx = dst.x - m_carPos.x;
    double dy = m_carPos.y - dst.y;

    if (dx < 0)
        return -atan(-dx / dy) * 180 / CV_PI;
    return atan(dx / dy) * 180 / CV_PI;
}

DetectLane::~DetectLane() {}

Mat DetectLane::update(const Mat &src)
{
    Mat output = src.clone();

    m_stopLane = UndefinedLine;
    m_leftLane = UndefinedLine;
    m_rightLane = UndefinedLine;

    resize(output, output, Size(Config::WIDTH, Config::HEIGHT));

    Mat binary = preProcess(output);

    vector<Vec4i> laneLines;
    vector<Vec4i> stopLines;
    fitLane2Line(binary, laneLines, stopLines);
    findLane(laneLines);
    findStopLane(stopLines);

    if (m_leftLane != UndefinedLine)
    {
        Point pt1 = getPointInLine(m_leftLane, Config::HEIGHT);
        Point pt2 = getPointInLine(m_leftLane, Config::HEIGHT / 2);
        line(output, pt1, pt2, Scalar(0, 0, 255), 3);
    }

    if (m_rightLane != UndefinedLine)
    {
        Point pt1 = getPointInLine(m_rightLane, Config::HEIGHT);
        Point pt2 = getPointInLine(m_rightLane, Config::HEIGHT / 2);
        line(output, pt1, pt2, Scalar(0, 255, 0), 3);
    }

    if (m_stopLane != UndefinedLine)
    {
        line(output, Point(m_stopLane[0], m_stopLane[1]), Point(m_stopLane[2], m_stopLane[3]), Scalar(255, 0, 0), 3);
    }

    return output;
}

float DetectLane::getErrorAngle()
{
    Point dst(Config::WIDTH / 2, Config::HEIGHT / 2);
    int p1 = getPointInLine(m_leftLane, Config::HEIGHT / 2).x;
    int p2 = getPointInLine(m_rightLane, Config::HEIGHT / 2).x;

    if (m_leftLane != UndefinedLine && m_rightLane != UndefinedLine)
    {
        dst.x = (p1 + p2) / 2;
    }
    else if (m_rightLane != UndefinedLine)
    {
        dst.x = p2 - Config::LANE_WIDTH / 2;
    }
    else if (m_leftLane != UndefinedLine)
    {
        dst.x = p1 + Config::LANE_WIDTH / 2;
    }

    m_preLane = Vec4i(dst.x, dst.y, m_carPos.x, m_carPos.y);

    return errorAngle(dst);
}

bool DetectLane::HasLane()
{
    return m_leftLane != UndefinedLine || m_rightLane != UndefinedLine;
}

bool DetectLane::IsStop()
{
    return m_stopLane != UndefinedLine && (m_stopLane[1] + m_stopLane[3]) > Config::HEIGHT / 3 * 2;
}

Mat DetectLane::preProcess(const Mat &src)
{
    Mat binary;

    binary = binaryImage(src);

    return binary;
}

void DetectLane::findLane(const vector<Vec4i> &lines)
{
    if (lines.size() == 0)
        return;

    vector<int> labels;

    int cnt = partition(lines, labels, Dist1());

    int countLine[cnt];
    Vec4i mean[cnt];

    for (int i = 0; i < cnt; i++)
    {
        countLine[i] = 0;
        mean[i] = Vec4i(0, 0, 0, 0);
    }

    for (int i = 0; i < labels.size(); i++)
    {
        countLine[labels[i]]++;
    }

    for (int i = 0; i < lines.size(); i++)
    {
        mean[labels[i]] += lines[i];
    }

    for (int i = 0; i < cnt; i++)
    {
        mean[i] /= countLine[i];
    }

    for (int i = 0; i < cnt - 1; i++)
    {
        for (int j = i + 1; j < cnt; j++)
        {
            if (countLine[i] < countLine[j])
            {
                swap(countLine[i], countLine[j]);
                swap(mean[i], mean[j]);
            }
        }
    }

    m_leftLane = UndefinedLine;
    m_rightLane = UndefinedLine;

    if (cnt >= 2)
    {
        m_leftLane = mean[0];

        for (int i = 1; i < cnt; i++)
        {
            if (!isIntersect(m_leftLane, mean[i]))
            {
                int x1 = getPointInLine(m_leftLane, Config::HEIGHT).x;
                int x2 = getPointInLine(mean[i], Config::HEIGHT).x;

                if ((Config::WIDTH / 2 >= x1 && Config::WIDTH / 2 <= x2) || ((Config::WIDTH / 2 <= x1 && Config::WIDTH / 2 >= x2)))
                {
                    m_rightLane = mean[i];
                    break;
                }
            }
        }
        if (m_rightLane != UndefinedLine && getPointInLine(m_leftLane, Config::HEIGHT).x > getPointInLine(m_rightLane, Config::HEIGHT).x)
        {
            std::swap(m_leftLane, m_rightLane);
        } else if (getPointInLine(m_leftLane, Config::HEIGHT).x > Config::WIDTH / 2)
        {
            m_rightLane = m_leftLane;
            m_leftLane = UndefinedLine;
        }
    }
    else if (cnt > 0)
    {
        if (getPointInLine(mean[0], Config::HEIGHT).x < Config::WIDTH / 2)
            m_leftLane = mean[0];
        else
            m_rightLane = mean[0];
    }
}

void DetectLane::findStopLane(const vector<Vec4i> &lines)
{
    if (lines.size() == 0)
        return;

    vector<int> labels;

    int cnt = partition(lines, labels, Dist2());

    int countLine[cnt];
    Vec4i mean[cnt];

    for (int i = 0; i < cnt; i++)
    {
        countLine[i] = 0;
        mean[i] = Vec4i(Config::WIDTH, Config::HEIGHT, 0, 0);
    }

    for (int i = 0; i < labels.size(); i++)
    {
        countLine[labels[i]]++;
    }

    for (int i = 0; i < lines.size(); i++)
    {
        if (mean[labels[i]][0] > lines[i][0])
        {
            mean[labels[i]][0] = lines[i][0];
            mean[labels[i]][1] = lines[i][1];
        }
        if (mean[labels[i]][0] > lines[i][2])
        {
            mean[labels[i]][0] = lines[i][2];
            mean[labels[i]][1] = lines[i][3];
        }
        if (mean[labels[i]][2] < lines[i][0])
        {
            mean[labels[i]][2] = lines[i][0];
            mean[labels[i]][3] = lines[i][1];
        }
        if (mean[labels[i]][2] < lines[i][2])
        {
            mean[labels[i]][2] = lines[i][2];
            mean[labels[i]][3] = lines[i][3];
        }
    }

    for (int i = 0; i < cnt - 1; i++)
    {
        for (int j = i + 1; j < cnt; j++)
        {
            if (countLine[i] < countLine[j])
            {
                swap(countLine[i], countLine[j]);
                swap(mean[i], mean[j]);
            }
        }
    }

    m_stopLane = UndefinedLine;

    if (cnt > 0)
    {
        int pLeft = 0;
        int pRight = Config::WIDTH;

        if (m_leftLane != UndefinedLine)
        {
            pLeft = getPointInLine(m_leftLane, mean[0][1]).x;
            pRight = getPointInLine(m_rightLane, mean[0][3]).x;
        }

        if (abs(mean[0][0] - pLeft) < 10 && abs(mean[0][2]- pRight) < 10)
        {
            m_stopLane = mean[0];
        }
    }
}

void DetectLane::fitLane2Line(const Mat &src, vector<Vec4i>& laneLines, vector<Vec4i>& stopLines)
{
    vector<Vec4i> lines;
    HoughLinesP(src, lines, 1, CV_PI / 180, 20, 10, 7);

    for (int i = 0; i < lines.size(); i++)
    {
        if (lines[i][1] < Config::SKY_LINE)
        {
            if (lines[i][3] < Config::HEIGHT / 3 * 2) continue;
        }
        if (lines[i][3] < Config::SKY_LINE)
        {
            if (lines[i][1] < Config::HEIGHT / 3 * 2) continue;
        }

        float angle = lineAngle(lines[i]);

        if (abs(angle) < 15)
        {
            int weight = max(lineLength(lines[i]) / 10, 1.0f);
            for (int j = 0; j < weight; j++)
            {
                stopLines.push_back(lines[i]);
            }
            continue;
        }

        if (lines[i][1] < lines[i][3])
        {
            std::swap(lines[i][0], lines[i][2]);
            std::swap(lines[i][1], lines[i][3]);
        }

        int weight = max(lineLength(lines[i]) / 10, 1.0f);
        for (int j = 0; j < weight; j++)
        {
            laneLines.push_back(lines[i]);
        }
    }
}

Mat DetectLane::binaryImage(const Mat &src)
{
    Mat img, imgThresholded, imgHSV, cannyImg, gray;

    img = src.clone();

    GaussianBlur(img, img, Size(3, 3), 0);

    cvtColor(img, imgHSV, COLOR_BGR2HSV);
    cvtColor(img, gray, COLOR_BGR2GRAY);

    inRange(imgHSV, Scalar(Config::LOW_H, Config::LOW_S, Config::LOW_V),
            Scalar(Config::HIGH_H, Config::HIGH_S, Config::HIGH_V),
            imgThresholded);
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5), Point(2, 2));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element, Point(2, 2));
    dilate(imgThresholded, imgThresholded, element, Point(2, 2));

    rectangle(imgThresholded, Point(70, 220), Point(270, 240), 0, CV_FILLED);

    Canny(gray, cannyImg, Config::CANNY_LOW,Config::CANNY_HIGH);

    Mat lane = Mat::zeros(img.size(), CV_8UC1);

    cannyImg.copyTo(lane, imgThresholded);

    imshow("Binary", imgThresholded);

    return lane;
}
