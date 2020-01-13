#include "mainwindow.h"

#include <QApplication>
using ::MainWindow;
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    // MainWindow w;
    // w.show();
    MainWindow::instance()->show();  // 使用单例模式
    return a.exec();
}
