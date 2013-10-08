#include  <ros/ros.h>
#include <certh_pickup/PickUp.h>
#include <certh_unfolding/unfold.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unfolding_demo") ;
    ros::NodeHandle nh ;


    // pick cloth from table

//    certh_pickup::PickUp pu;
//    pu.request.time_out = 0.0;
//    ros::service::waitForService("/certh_pickup/pickUp") ;
//    ros::service::call("/certh_pickup/pickUp", pu);

    // unfold hanging cloth
    certh_unfolding::unfold uf;
    uf.request.time_out = 0.0 ;
    ros::service::waitForService("/unfolding_Service/unfold");
    ros::service::call("/unfolding_Service/unfold", uf);


    ros::shutdown() ;

    return 1;

}
