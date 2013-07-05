#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_head_virtual");

    ros::NodeHandle nh ;

    ros::Publisher pub_joint_state_ = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

    sensor_msgs::JointState rh_state_ ;
    rh_state_.name.push_back("left_to_pan");
    rh_state_.name.push_back("right_to_pan");
    rh_state_.name.push_back("left_to_tilt");
    rh_state_.name.push_back("right_to_tilt");

    int nJoints = rh_state_.name.size() ;

    rh_state_.position.resize(nJoints);
    rh_state_.velocity.resize(nJoints);
    rh_state_.effort.resize(nJoints);

    for(int i=0 ; i<nJoints ; i++)
    {
        rh_state_.position[i] = 0.0 ;
        rh_state_.velocity[i] = 0.0 ;
        rh_state_.effort[i] = 0.0 ;
    }

    while ( ros::ok() )
    {

        rh_state_.header.stamp = ros::Time::now() ;

        pub_joint_state_.publish(rh_state_) ;
        ros::Duration(1.0).sleep() ;

        ros::spinOnce() ;
    }


    return 0 ;

}
