#include "mainwindow.h"

#include <QApplication>

// class APP
// {
// public:
//     APP(int argc, char *argv[]) : a(argc, argv){};

//     int start()
//     {
//         // ros::init(argc, argv, "trajectory_generator"); // we must call ros::init() before nodehandle, otherwise will get Couldn't find an AF_INET address for []' Error
//         MainWindow w;
//         w.show();
//     }

//     ~APP()
//     {
//     }

// private:
//     QApplication a;
//     MainWindow w;

// public:
// };

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "trajectory_generator"); // we must call ros::init() before nodehandle, otherwise will get Couldn't find an AF_INET address for []' Error
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
