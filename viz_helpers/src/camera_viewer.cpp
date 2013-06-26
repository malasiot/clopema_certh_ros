#include <ros/ros.h>
#include <QtGui>
#include <boost/algorithm/string.hpp>

#include "CameraView.h"

using namespace std ;
using namespace viz_helpers ;

int main(int argc, char *argv[])
{
	QApplication app(argc, argv) ;

    ros::init(argc, argv, "camera_viewer") ;
    ros::NodeHandle nh ;

    QMainWindow *mwin = new QMainWindow() ;

    string topicStr ;

    if ( !ros::param::get("/camera_viewer/image", topicStr) )
    {
        ROS_ERROR("No input topic specified") ;
        exit(1) ;
    }

    std::vector<string> topics ;

    boost::algorithm::split( topics, topicStr, boost::is_any_of(";:"), boost::algorithm::token_compress_on );

    viz_helpers::QCameraView *cam = new viz_helpers::QCameraView(mwin, nh, topics) ;
    mwin->setCentralWidget(cam);

    mwin->show() ;

    ros::AsyncSpinner spinner(1) ;
    spinner.start() ;

	app.exec() ;
	

}
