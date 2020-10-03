#include "application.h"

int main(int argc, char *argv[])
{
    Application app;

    system("clear");

    if (argc >= 2)
    {
        int speed = atoi(argv[1]);
        std::cout << "Car Speed: " << speed << std::endl;
        Config::VELOCITY = speed;
    }

    app.Start();

    return 0;
}
