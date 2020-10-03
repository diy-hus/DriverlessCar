#include "application.h"

float middle(float value, float min, float max)
{
    if( value > max )
        value = max;
    else if( value < min )
        value = min;
    return value;
}

Application::Application()
{
    startWindowThread();
    namedWindow( "Camera", CV_WINDOW_AUTOSIZE );
}

void Application::Start()
{
    m_car.Init();
    Mat image;

    m_car.SetSteerAngle(30);
    time_sleep(1);
    m_car.SetSteerAngle(0);

    bool start = false;

    //    VideoWriter video("video.avi",CV_FOURCC('M','J','P','G'),10, Size(320,240));

    while (true) {
        m_capture >> image;
        if (waitKey(1) == 'q')
        {
            break;
        }

        if(digitalRead (Config::BTN3) == LOW)
        {
            time_sleep(1);
            if(digitalRead (Config::BTN3) == LOW)
            {
                start = false;
                break;
            }
        }

        if (digitalRead (Config::PROX_PIN) == LOW)
        {
            m_car.SetSpeed(0);
            m_car.SetSteerAngle(0);
            continue;
        }

        if (!start)
        {
            imshow("Camera", image);
            if(digitalRead (Config::BTN1) == LOW)
            {
                start = true;
                time_sleep(0.25);
            }
            continue;
        }

        if(digitalRead (Config::BTN2) == LOW)
        {
            start = false;
            m_car.Brake();
            time_sleep(0.25);
            continue;
        }

        //        video.write(image);
        image = m_detector.update(image);
        if (m_detector.IsStop())
        {
            m_car.SetSteerAngle(0);
            m_car.Brake();
            time_sleep(5);
            m_car.SetSpeed(Config::VELOCITY);
            time_sleep(0.5);
        }
        else if (m_detector.HasLane())
        {
            float errorAngle = middle(m_detector.getErrorAngle(), -60, 60);
            float pidAngle = PID(errorAngle);

            if (abs(errorAngle) > 10)
            {
                m_car.SetSpeed(max(Config::VELOCITY - 10, Config::VELOCITY - (abs(errorAngle) - 10)), pidAngle / 50);
            }
            else
            {
                m_car.SetSpeed(Config::VELOCITY, pidAngle / 50);
            }
            m_car.SetSteerAngle(pidAngle);
        } else {
            m_car.SetSpeed(Config::VELOCITY / 3.0 * 2.0);
            m_car.SetSteerAngle(PID(m_preError));
        }
        imshow("Camera", image);
    }
}

float Application::PID(float error)
{
    float Pout = Config::kP * error;

    m_integral += error;

    if (m_integral > 1) m_integral = 1;
    else if (m_integral < -1) m_integral = -1;
    float Iout = Config::kI * m_integral;

    float Dout = Config::kD * (error - m_preError);

    float output = Pout + Dout;

    output = middle(output, -60, 60);

    m_preError = error;

    return output;
}
