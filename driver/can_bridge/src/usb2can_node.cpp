#include "usb2can_core.h"
#include <iostream>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "can_module");
    USB2CAN::CAN_app app;
    app.run();
    // signal(SIGINT, USB2CAN::ExitHandler);
    return 0;
}
