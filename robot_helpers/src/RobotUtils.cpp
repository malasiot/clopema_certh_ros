#include "robot_helpers/Utils.h"

#include <geometric_shapes/shape_operations.h>
#include <planning_environment/util/construct_object.h>
#include <geometry_msgs/Pose.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/GetPlanningScene.h>

#include <visualization_msgs/MarkerArray.h>
#include <planning_environment/models/collision_models.h>


using namespace std ;

namespace robot_helpers {

bool moveHome(MoveRobot &cmove)
{

    clopema_arm_navigation::ClopemaMotionPlan mp;
    mp.request.motion_plan_req.group_name = "arms";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    if (!getRobotState(mp.request.motion_plan_req.start_state)) return false ;

    vector<string> joint_names ;

    joint_names.push_back("r1_joint_s");
    joint_names.push_back("r1_joint_l");
    joint_names.push_back("r1_joint_u");
    joint_names.push_back("r1_joint_r");
    joint_names.push_back("r1_joint_b");
    joint_names.push_back("r1_joint_t");

    joint_names.push_back("r2_joint_s");
    joint_names.push_back("r2_joint_l");
    joint_names.push_back("r2_joint_u");
    joint_names.push_back("r2_joint_r");
    joint_names.push_back("r2_joint_b");
    joint_names.push_back("r2_joint_t");

    mp.request.motion_plan_req.goal_constraints.joint_constraints.resize(joint_names.size());

    for (unsigned int i = 0; i < mp.request.motion_plan_req.goal_constraints.joint_constraints.size(); ++i) {
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].joint_name = joint_names[i];
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position = 0.0;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
    }

    if (!plan(mp)) return false ;

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = mp.response.joint_trajectory;
    cmove.doGoal(goal);

    mp.request.motion_plan_req.group_name = "ext_axis";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);
    if (!getRobotState(mp.request.motion_plan_req.start_state)) return false ;

    joint_names.clear() ;
    joint_names.push_back("ext_axis") ;

    mp.request.motion_plan_req.goal_constraints.joint_constraints.resize(joint_names.size());
    for (unsigned int i = 0; i < mp.request.motion_plan_req.goal_constraints.joint_constraints.size(); ++i) {
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].joint_name = joint_names[i];
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position = 0.0;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
    }

    if ( !plan(mp) ) return false ;


    goal.trajectory = mp.response.joint_trajectory;
    cmove.doGoal(goal);

    return true ;
}

bool moveGripper(MoveRobot &cmove, const std::string &armName, const Eigen::Vector3d &pos, const Eigen::Quaterniond &q)
{
    clopema_arm_navigation::ClopemaMotionPlan mp;
    mp.request.motion_plan_req.group_name = armName + "_arm" ;
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    if ( !getRobotState(mp.request.motion_plan_req.start_state) ) return false ;

    arm_navigation_msgs::SimplePoseConstraint desired_pose;

    desired_pose.header.frame_id = "base_link";
    desired_pose.header.stamp = ros::Time::now();
    desired_pose.link_name = armName + "_ee";
    desired_pose.pose.position.x = pos.x() ;
    desired_pose.pose.position.y = pos.y() ;
    desired_pose.pose.position.z = pos.z() ;

    desired_pose.pose.orientation.x = q.x() ;
    desired_pose.pose.orientation.y = q.y() ;
    desired_pose.pose.orientation.z = q.z() ;
    desired_pose.pose.orientation.w = q.w() ;

    desired_pose.absolute_position_tolerance.x = desired_pose.absolute_position_tolerance.y = desired_pose.absolute_position_tolerance.z = 0.02;
    desired_pose.absolute_roll_tolerance = desired_pose.absolute_pitch_tolerance = desired_pose.absolute_yaw_tolerance = 0.04;

    poseToClopemaMotionPlan(mp, desired_pose);

    ROS_INFO("Planning to the start position");

    if (!plan(mp)) return false;

    trajectory_msgs::JointTrajectory wholeTraj = mp.response.joint_trajectory;
    mp.response.joint_trajectory = wholeTraj;

    control_msgs::FollowJointTrajectoryGoal goal;

    if ( mp.response.joint_trajectory.points.size() > 2 )
    {
        goal.trajectory = mp.response.joint_trajectory;
        cmove.doGoal(goal) ;

    }

    return true ;


}

bool moveGripperPointingDown(MoveRobot &cmove, const std::string &armName, double X, double Y, double Z)
{
    Eigen::Quaterniond q ;
    q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) ;
    return moveGripper(cmove, armName, Eigen::Vector3d(X, Y, Z), q) ;
}



bool rotateGripper(MoveRobot &cmove, const std::string &armName, double theta)
{
    clopema_arm_navigation::ClopemaMotionPlan mp;

    mp.request.motion_plan_req.group_name = armName + "_arm";
    mp.request.motion_plan_req.allowed_planning_time = ros::Duration(5.0);

    if ( !getRobotState(mp.request.motion_plan_req.start_state) ) return false ;

    std::vector<std::string> names;
    names.push_back(armName + "_joint_s");
    names.push_back(armName + "_joint_l");
    names.push_back(armName + "_joint_u");
    names.push_back(armName + "_joint_r");
    names.push_back(armName + "_joint_b");
    names.push_back(armName + "_joint_t");

    string gripperJointName = armName + "_joint_t" ;

    mp.request.motion_plan_req.goal_constraints.joint_constraints.resize(names.size());

    for( int i=0 ; i<names.size() ; i++ )
    {
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].joint_name = names[i];
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
        mp.request.motion_plan_req.goal_constraints.joint_constraints[i].tolerance_above = 0.1;

        for(int j=0 ; j< mp.request.motion_plan_req.start_state.joint_state.name.size() ; j++ )
        {
            if ( mp.request.motion_plan_req.start_state.joint_state.name[j] == names[i] )
            {
                if ( names[i] == gripperJointName )
                    mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position =
                            mp.request.motion_plan_req.start_state.joint_state.position[j] + theta ;
                else
                    mp.request.motion_plan_req.goal_constraints.joint_constraints[i].position =
                            mp.request.motion_plan_req.start_state.joint_state.position[j]  ;
            }
        }

    }

    if (!plan(mp)) return false ;

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = mp.response.joint_trajectory;
    cmove.doGoal(goal);

    return true ;
}




////////////////////////////////////////////////////////////////////////////////////////////////////////

// hacked from freeglut

static void fghCircleTable(double **sint,double **cost,const int n)
{
    int i;

    /* Table size, the sign of n flips the circle direction */

    const int size = abs(n);

    /* Determine the angle between samples */

    const double angle = 2*M_PI/(double)( ( n == 0 ) ? 1 : n );

    /* Allocate memory for n samples, plus duplicate of first entry at the end */

    *sint = (double *) calloc(sizeof(double), size+1);
    *cost = (double *) calloc(sizeof(double), size+1);

    /* Bail out if memory allocation fails, fgError never returns */

    if (!(*sint) || !(*cost))
    {
        free(*sint);
        free(*cost);

    }

    /* Compute cos and sin around the circle */

    (*sint)[0] = 0.0;
    (*cost)[0] = 1.0;

    for (i=1; i<size; i++)
    {
        (*sint)[i] = sin(angle*i);
        (*cost)[i] = cos(angle*i);
    }

    /* Last sample is duplicate of the first */

    (*sint)[size] = (*sint)[0];
    (*cost)[size] = (*cost)[0];
}


void makeSolidCone( shapes::Mesh  &mesh, double base, double height, int slices, int stacks )
{
    int i,j;

    /* Step in z and radius as stacks are drawn. */

    double z0, z1;
    double r0, r1;

    const double zStep = height / ( ( stacks > 0 ) ? stacks : 1 );
    const double rStep = base / ( ( stacks > 0 ) ? stacks : 1 );

    /* Scaling factors for vertex normals */

    const double cosn = ( height / sqrt ( height * height + base * base ));
    const double sinn = ( base   / sqrt ( height * height + base * base ));

    /* Pre-computed circle */

    double *sint,*cost;


    fghCircleTable(&sint,&cost,-slices);

    /* Cover the circular base with a triangle fan... */

    z0 = -height ;

    r0 = base;
    r1 = r0 - rStep;

    int maxVert = 6*(slices * stacks + 10) ;

    mesh.vertexCount = 0 ;
    mesh.vertices = new double [maxVert] ;
    mesh.triangles = new unsigned int [maxVert] ;
    mesh.normals =  new double [maxVert] ;

    int vc = 0, nc = 0, tc = 0 ;

    // make bottom faces

    mesh.vertices[vc++] = 0.0 ;
    mesh.vertices[vc++] = 0.0 ;
    mesh.vertices[vc++] = z0 ;


    mesh.vertexCount ++ ;

    for (j=0; j<slices; j++ )
    {
        mesh.vertices[vc++] = cost[j]*r0 ;
        mesh.vertices[vc++] = sint[j]*r0 ;
        mesh.vertices[vc++] = z0 ;

        mesh.vertexCount ++ ;
    }

#define CYCLE(a) (((a)==slices+1) ? 1 : (a))

    for (j=0; j<slices; j++ )
    {

        mesh.triangles[tc++] = j+1 ;
        mesh.triangles[tc++] = 0 ;
        mesh.triangles[tc++] = CYCLE(j+2) ;


        mesh.normals[nc++] = 0.0 ;
        mesh.normals[nc++] = 0.0 ;
        mesh.normals[nc++] = 1 ;

        mesh.triangleCount ++ ;
    }


    /* Cover each stack with a quad strip, except the top stack */

    r1 = r0 ;
    z1 = z0 ;

    for( i=1; i<stacks-1; i++ )
    {
        r1 -= rStep ;
        z1 += zStep ;

        for(j=0; j<slices; j++)
        {
            mesh.vertices[vc++] = cost[j]*r1 ;
            mesh.vertices[vc++] = sint[j]*r1 ;
            mesh.vertices[vc++] = z1 ;

            mesh.vertexCount ++ ;
        }



        for(j=0; j<slices; j++)
        {

            mesh.triangles[tc++] = (i -1)*(slices) + j + 1 ;
            mesh.triangles[tc++] = (i -1)*(slices) + CYCLE(j+2);
            mesh.triangles[tc++] = i * (slices)  + CYCLE(j+2) ;
            mesh.triangleCount ++ ;

            mesh.normals[nc++] = cost[j]*cosn ;
            mesh.normals[nc++] = sint[j]*cosn ;
            mesh.normals[nc++] = sinn ;


            mesh.triangles[tc++] = (i -1)*(slices) + j + 1 ;
            mesh.triangles[tc++] = i * (slices) + CYCLE(j+2) ;
            mesh.triangles[tc++] = i * (slices)  + j + 1 ;
            mesh.triangleCount ++ ;

            mesh.normals[nc++] = cost[j]*cosn ;
            mesh.normals[nc++] = sint[j]*cosn ;
            mesh.normals[nc++] = sinn ;

        }
     }

    mesh.vertices[vc++] = 0 ;
    mesh.vertices[vc++] = 0 ;
    mesh.vertices[vc++] = 0 ;

    mesh.vertexCount ++ ;

    for(j=0; j<slices; j++)
    {

        mesh.triangles[tc++] = mesh.vertexCount -1 - slices + j + 1 ;
        mesh.triangles[tc++] = mesh.vertexCount -1 - slices + CYCLE(j+2) ;
        mesh.triangles[tc++] = mesh.vertexCount -1 ;

        mesh.normals[nc++] = cost[j]*cosn ;
        mesh.normals[nc++] = sint[j]*cosn ;
        mesh.normals[nc++] = sinn ;

        mesh.triangleCount ++ ;
    }

    /* Release sin and cos tables */

    free(sint);
    free(cost);
}

// We add a collision object as attached object to the gripper. the parent frame is set to the baselink since otherwise the
// cone will rotate as the gripper moves. Thus this function may be used for one arm static or by calling resetCollisionModel, moving the arm,
// and then setting again the collision object. This is a peculiarity of non-rigid attachement.

bool addConeToCollisionModel(const std::string &armName, double length, double radius)
{
    ros::NodeHandle nh("~") ;

    ros::service::waitForService("/environment_server/set_planning_scene_diff");
    ros::ServiceClient get_planning_scene_client =
      nh.serviceClient<arm_navigation_msgs::GetPlanningScene>("/environment_server/set_planning_scene_diff");

    arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
    arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

    arm_navigation_msgs::AttachedCollisionObject att_object;
    att_object.link_name = armName + "_gripper";

    att_object.object.id = "attached_cone";
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

    att_object.object.header.frame_id = "base_link";
    att_object.object.header.stamp = ros::Time::now();

    Eigen::Vector3d p = robot_helpers::getPose(armName).inverse().translation() ;


    arm_navigation_msgs::Shape object;

    shapes::Mesh mesh ;
    makeSolidCone(mesh, radius, length, 10, 20) ;

    if(!planning_environment::constructObjectMsg(&mesh, object)) {
      ROS_WARN_STREAM("Object construction fails");
    }

    geometry_msgs::Pose pose;
    pose.position.x = p.x();
    pose.position.y = p.y();
    pose.position.z = p.z() ;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    att_object.object.shapes.push_back(object);
    att_object.object.poses.push_back(pose);

    planning_scene_req.planning_scene_diff.attached_collision_objects.push_back(att_object);

    if(!get_planning_scene_client.call(planning_scene_req, planning_scene_res)) return false;


    return true ;
}


bool addSphereToCollisionModel(const std::string &armName, double radius)
{
    std::string arm2Name;
    ros::NodeHandle nh("~") ;

    ros::service::waitForService("/environment_server/set_planning_scene_diff");
    ros::ServiceClient get_planning_scene_client =
      nh.serviceClient<arm_navigation_msgs::GetPlanningScene>("/environment_server/set_planning_scene_diff");

    arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
    arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

    arm_navigation_msgs::AttachedCollisionObject att_object;

    att_object.link_name = armName + "_gripper";

    att_object.touch_links.push_back(armName +"_link_1");
    att_object.touch_links.push_back(armName +"_link_2");
    att_object.touch_links.push_back(armName +"_link_3");
    att_object.touch_links.push_back(armName +"_link_4");
    att_object.touch_links.push_back(armName +"_link_5"); 
    att_object.touch_links.push_back(armName +"_link_6");
    att_object.touch_links.push_back(armName +"_tip_link");
    att_object.touch_links.push_back(armName +"_ee");
    att_object.touch_links.push_back(armName +"_xtion");
    att_object.touch_links.push_back(armName +"_cable_1");
    att_object.touch_links.push_back("r750_base");
    att_object.touch_links.push_back("base_link");
    
    if (armName=="r1")
       arm2Name="r2";
    else
       arm2Name="r1";

    att_object.touch_links.push_back(arm2Name + "_link_1");
    att_object.touch_links.push_back(arm2Name + "_link_2");
    att_object.touch_links.push_back(arm2Name + "_link_3");
    att_object.touch_links.push_back(arm2Name + "_link_4");
    att_object.touch_links.push_back(arm2Name + "_link_5"); 
    att_object.touch_links.push_back(arm2Name + "_link_6");
    att_object.touch_links.push_back(arm2Name + "_tip_link");
    att_object.touch_links.push_back(arm2Name + "_xtion");
    att_object.touch_links.push_back(arm2Name + "_cable_1");

    att_object.object.id = "/attached_sphere";
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

    att_object.object.header.frame_id = "base_link";
    att_object.object.header.stamp = ros::Time::now();


	
    Eigen::Vector3d p = robot_helpers::getPose(armName).inverse().translation() ;


    arm_navigation_msgs::Shape object;

    object.type = arm_navigation_msgs::Shape::SPHERE;
    object.dimensions.resize(1);
    object.dimensions[0] = radius;

    geometry_msgs::Pose pose;
    pose.position.x = p.x();
    pose.position.y = p.y();
    pose.position.z = p.z();

    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    att_object.object.shapes.push_back(object);
    att_object.object.poses.push_back(pose);

    planning_scene_req.planning_scene_diff.attached_collision_objects.push_back(att_object);

    if(!get_planning_scene_client.call(planning_scene_req, planning_scene_res)) return false;


    return true ;
}


bool resetCollisionModel()
{
    ros::NodeHandle nh("~") ;

    ros::service::waitForService("/environment_server/set_planning_scene_diff");
    ros::ServiceClient set_planning_scene_client =
      nh.serviceClient<arm_navigation_msgs::GetPlanningScene>("/environment_server/set_planning_scene_diff");

    arm_navigation_msgs::GetPlanningScene::Request planning_scene_req;
    arm_navigation_msgs::GetPlanningScene::Response planning_scene_res;

    arm_navigation_msgs::AttachedCollisionObject att_object;

    att_object.object.id = "all";
    att_object.object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;

    att_object.object.header.frame_id = "base_link";
    att_object.object.header.stamp = ros::Time::now();

    planning_scene_req.planning_scene_diff.attached_collision_objects.push_back(att_object);

    if(!set_planning_scene_client.call(planning_scene_req, planning_scene_res))
       return false;


}



















}
