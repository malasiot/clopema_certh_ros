#include "Dashboard.h"

#include <ros/ros.h>
#include <QtGui>
#include <boost/algorithm/string.hpp>

using namespace std ;
using namespace viz_helpers ;

int main(int argc, char *argv[])
{
    QApplication app(argc, argv) ;

    ros::init(argc, argv, "dashboard") ;
    ros::NodeHandle nh ;

    QMainWindow *mwin = new QMainWindow() ;

    RobotDashboard *widget = new RobotDashboard(nh, mwin) ;
    mwin->setCentralWidget(widget);

    mwin->show() ;

    ros::AsyncSpinner spinner(1) ;
    spinner.start() ;

    app.exec() ;


}
