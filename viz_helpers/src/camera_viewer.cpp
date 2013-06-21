#include <ros/ros.h>
#include <QtGui>


#include "CameraView.h"
#include <viz_helpers/CameraViewServer.h>

using namespace std ;
using namespace viz_helpers ;

void printMouse(int x, int y)
{
    cout << x << ' ' << y << endl ;
}

int main(int argc, char *argv[])
{
	QApplication app(argc, argv) ;

    ros::init(argc, argv, "camera_viewer") ;
    ros::NodeHandle nh ;


    QMainWindow *mwin = new QMainWindow() ;

    viz_helpers::QCameraView *cam = new viz_helpers::QCameraView(mwin, nh) ;
    mwin->setCentralWidget(cam);

    mwin->show() ;
   
    viz_helpers::CameraViewServer srv ;

    srv.setMouseClickCallback(printMouse);
	
    ros::AsyncSpinner spinner(1) ;
    spinner.start() ;

	app.exec() ;
	

}
