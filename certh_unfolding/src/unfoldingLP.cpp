#include "RFLibrary/rf.h"
#include "RFLibrary/rf.cpp"
#include "HFLibrary/hf.h"
#include "HFLibrary/hf.cpp"
#include "Unfold.h"
#include <sstream>
#include <time.h>
#include <camera_helpers/OpenNIServiceClient.h>

using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "graspTwoTimesLP");
    ros::NodeHandle nh;

    ros::Publisher marker_pub;
    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker_", 0);

    sleep(3);
    Unfold uf("r1" );
    camera_helpers::openni::connect(uf.camera) ;
    ros::Duration(1).sleep() ;
    while(!uf.graspLowestPoint(false, true));
    while(!uf.graspLowestPoint(true, true));

    camera_helpers::openni::disconnect(uf.camera) ;


    setServoPowerOff() ;
    return 0;
}
