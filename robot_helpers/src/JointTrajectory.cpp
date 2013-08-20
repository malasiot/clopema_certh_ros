#include <robot_helpers/JointTrajectory.h>

using namespace std ;

namespace robot_helpers {


double JointTrajectory::lerp(const std::string &joint, double t) const
{
    t = std::max(0.0, std::min(1.0, t)) ;

    assert ( times.size() > 1 ) ;

    int idx = 0 ;
    for( ; idx < times.size() ; idx++ )
    {
        if ( times[idx+1] >= t ) break ;
    }

    vector<string>::const_iterator it = std::find(names.begin(), names.end(), joint) ;

    assert( it != names.end() ) ;

    int joint_idx = std::distance(it, names.begin()) ;

    double state1 = positions[joint_idx][idx] ;
    double state2 = positions[joint_idx][idx+1] ;

    return  (1 -t) * state1 + t * state2 ;

}

void JointTrajectory::addPoint(double t, const JointState &state)
{
    if ( names.empty() )
    {
        map<string, double>::const_iterator it = state.joint_values.begin() ;

        for( ; it != state.joint_values.end() ; ++it )
            names.push_back((*it).first) ;
    }

    assert(t >= 0.0 && t <= 1.0) ;

    positions.resize(positions.size()+1) ;
    times.push_back(t) ;

    for(int i=0 ; i<names.size() ; i++ )
    {
        map<string, double>::const_iterator it = state.joint_values.find(names[i]) ;

        assert( it != state.joint_values.end() ) ;

        double val = (*it).second ;
        positions.back().push_back(val) ;
    }

}

trajectory_msgs::JointTrajectory JointTrajectory::toMsg(double time_scale) const
{
    trajectory_msgs::JointTrajectory msg ;

    assert( !names.empty() && !positions.empty() ) ;

    msg.joint_names = names ;

    for(int i=0 ; i<positions.size() ; i++ )
    {
        trajectory_msgs::JointTrajectoryPoint pt ;

        pt.time_from_start =ros::Duration(times[i] * time_scale) ;

        for( int j=0 ; j<names.size() ; j++ )
            pt.positions.push_back(positions[i][j]) ;

        msg.points.push_back(pt) ;
    }

    return msg ;
}

void JointTrajectory::completeTrajectory(const JointState &rs)
{
    map<string, double>::const_iterator it = rs.joint_values.begin() ;

    for( ; it != rs.joint_values.end() ; ++it )
    {
        string name_ = (*it).first ;
        double val_ = (*it).second ;

        vector<string>::const_iterator jt = find(names.begin(), names.end(), name_) ;

        if ( jt == names.end() )
        {
            names.push_back(name_) ;

            for(int i=0 ; i<positions.size() ; i++ )
                positions[i].push_back(val_) ;
        }
    }
}

}
