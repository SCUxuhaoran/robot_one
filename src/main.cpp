#include <QtGui>
#include <QApplication>
#include "../include/robot_one/main_window.hpp"

int main(int argc, char **argv)
{

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    robot_one::MainWindow w(argc, argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

    return result;
}
