#include <LinearMath/btMatrix3x3.h>
#include <clopema_motoros/ClopemaMove.h>
#include <clopema_arm_navigation/ClopemaLinearInterpolation.h>
#include <tf/transform_listener.h>
#include <robot_helpers/Utils.h>
#include <robot_helpers/Geometry.h>

using namespace std ;
using namespace robot_helpers ;

int main(int argc, char **argv) {

    ros::init(argc, argv, "move_pose");
	ros::NodeHandle nh;

    std::string armName;
    cout << "Select arm " << endl;
    cin >> armName ;
    armName = "r2";

    geometry_msgs::Pose pose ;
    Eigen::Matrix3d orient ;
    orient << 0, -1, 0,-1, 0, 0, 0, 0, -1; //vertical
    //orient << ; //horizontal face down
    cout << "Gimme coords \n";
    cin >> pose.position.x >> pose.position.y >> pose.position.z ;

    pose.orientation = rotationMatrix3ToQuaternion(orient ) ;
    cout << "Quaternion = ( " << pose.orientation.x << ", "<< pose.orientation.y << ", "<< pose.orientation.z << ", "<< pose.orientation.w << " )" << endl ;

    moveArm(pose, armName) ;

    setServoPowerOff() ;
    ros::shutdown() ;

	return 1;
}
