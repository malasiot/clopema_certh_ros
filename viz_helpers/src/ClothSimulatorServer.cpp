#include <ros/ros.h>

#include <boost/algorithm/string.hpp>

#include "ClothSimulator.h"
#include <planning_environment/monitors/kinematic_model_state_monitor.h>
#include <viz_helpers/ClothSimulatorService.h>
#include <robot_helpers/Utils.h>

using namespace std ;



class Server {
public:
    Server(ros::NodeHandle nh)
    {
        marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

        cm.reset(new planning_environment::CollisionModels("robot_description")) ;

        monitor.reset( new planning_environment::KinematicModelStateMonitor(cm.get(), &listener) ) ;

        monitor->startStateMonitor() ;
        monitor->waitForState() ;

        state.reset(new planning_models::KinematicState(monitor->getKinematicModel())) ;

        monitor->setStateValuesFromCurrentValues(*state);

        physics.reset(new Physics(*cm.get(), *state)) ;

        start() ;

        server = nh.advertiseService<viz_helpers::ClothSimulatorService::Request, viz_helpers::ClothSimulatorService::Response>("cloth_simulator_service",
           boost::bind(&Server::callback, this, _1, _2));


    }

    void loop() {



        ros::Time prevTime = ros::Time::now(), curTime ;

        while ( ros::ok() )
        {
            {
                boost::unique_lock<boost::mutex> lock ;
                if ( stopLoop ) break ;
            }

            // get current state

            monitor->waitForState() ;
            monitor->setStateValuesFromCurrentValues(*state);

            {
                // update colision model

                boost::lock_guard<boost::mutex> guard(physics_lock) ;

                physics->updateCollisions(*state) ;

                // update simulation

                curTime = ros::Time::now() ;
                physics->updatePhysicsSimulation((curTime - prevTime).toSec());
                prevTime = curTime ;

                // get marker and publish it

                if ( marker_pub.getNumSubscribers() > 0 ) {

                    map<string, boost::shared_ptr<SoftBody> >::const_iterator it = sb.begin() ;

                    while ( it != sb.end() )
                    {
                        visualization_msgs::Marker marker ;

                        physics->getMeshMarker((*it).first, marker);

                        marker_pub.publish(marker) ;

                        ++it ;
                    }


                }

            }

            prevTime = curTime ;

            ros::spinOnce() ;
        }



    }

    void start() {

        stopLoop = false ;
        thread.reset(new boost::thread(boost::bind(&Server::loop, this))) ;

    }

    ~Server()
    {
        {
            boost::lock_guard<boost::mutex> guard(stop_mutex) ;
            stopLoop = true ;
        }

        thread->join() ;

    }

    bool callback(viz_helpers::ClothSimulatorService::Request& request, viz_helpers::ClothSimulatorService::Response& response) ;


    boost::shared_ptr<Physics> physics ;
    ros::ServiceServer server ;
    tf::TransformListener listener ;
    ros::Publisher marker_pub ;
    boost::shared_ptr<planning_environment::CollisionModels> cm ;
    boost::shared_ptr<planning_environment::KinematicModelStateMonitor> monitor ;
    boost::shared_ptr<planning_models::KinematicState> state ;
    boost::mutex physics_lock, stop_mutex ;
    boost::shared_ptr<boost::thread> thread ;
    std::map<string, boost::shared_ptr<SoftBody> > sb ;
    bool stopLoop ;

};


bool Server::callback(viz_helpers::ClothSimulatorService::Request &request, viz_helpers::ClothSimulatorService::Response &response)
{
 //   boost::lock_guard<boost::mutex> guard(stop_mutex) ;


    if ( request.action == viz_helpers::ClothSimulatorService::Request::ADD )
    {
        if ( request.name.empty() ) {
            response.return_code = -1 ;
            return true ;
        }

        boost::lock_guard<boost::mutex> guard(physics_lock) ;

        if ( sb.count(request.name) != 0 )
            physics->removeSoftBody(request.name) ;

        SoftBody *pBody ;

        if ( request.width <= 0.0 || request.height <= 0.0 || request.nodes == 0 )
        {
            response.return_code = -2 ;
            return true ;

        }

        unsigned int nvu = sqrt(request.nodes) + 0.5 ;

        switch ( request.type )
        {
            case viz_helpers::ClothSimulatorService::Request::TOWEL:
                pBody = new Cloth(request.pose, request.width, request.height, nvu, nvu) ;
                break ;
            default:
                response.return_code = -3 ;
                return true ;
        }

        sb[request.name].reset(pBody) ;

        physics->addSoftBody(request.name, *pBody) ;

        for(int i=0 ; i<request.anchors.size() ; i++ )
        {
            if ( i >= request.links.size() )
            {
                response.return_code = -4 ;
                return true ;
            }

            physics->attachSoftBodyToLink(request.name, request.anchors[i], request.links[i], *state);

        }

        response.return_code = 0 ;
        return true ;

    }
    else if ( request.action == viz_helpers::ClothSimulatorService::Request::REMOVE )
    {
        if ( request.name.empty() ) {
            response.return_code = -1 ;
            return true ;
        }

        boost::lock_guard<boost::mutex> guard(physics_lock) ;

        map<string, boost::shared_ptr<SoftBody> >::iterator it = sb.find(request.name) ;

        if ( it != sb.end() )
        {
            physics->removeSoftBody(request.name) ;
            sb.erase(it) ;

            visualization_msgs::Marker marker ;

            marker.ns = request.name ;
            marker.id = 0;
            marker.header.frame_id = "/base_link";
            marker.header.stamp = ros::Time::now();

            marker.type = visualization_msgs::Marker::POINTS ;

            marker_pub.publish(marker) ;

            response.return_code = 0 ;
        }
        else
            response.return_code = -2 ;

        return true ;
    }


    return false ;
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "cloth_simulation_server") ;
    ros::NodeHandle nh ;


    Server server(nh) ;


    ros::spin() ;

}
