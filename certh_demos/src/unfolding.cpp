#include  <ros/ros.h>
#include <certh_pickup/PickUp.h>
#include <certh_unfolding/unfold.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "unfolding_demo") ;
    ros::NodeHandle nh ;


    // pick cloth from table

    certh_pickup::PickUp pu;
    pu.request.time_out = 0.0;
     if (!ros::service::waitForService("/pickup_Service/pickup") )
         ROS_ERROR("Can't call service pickup_Service/pickup");
    ros::service::call("/pickup_Service/pickup", pu);

    // unfold hanging cloth
    certh_unfolding::unfold uf;
    uf.request.time_out = 0.0 ;
    if(!ros::service::waitForService("/unfolding_Service/unfold"))
        ROS_ERROR("Can't call service unfolding_Service/unfold");
    ros::service::call("/unfolding_Service/unfold", uf);


    ros::shutdown() ;

    return 1;

}
