#ifndef __CLOTH_SIMULATOR_H__
#define __CLOTH_SIMULATOR_H__

#include <btBulletCollisionCommon.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>

#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>

#include <map>
#include <boost/shared_ptr.hpp>

#include <planning_environment/models/collision_models.h>

#include <boost/signals.hpp>
#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <visualization_msgs/Marker.h>


struct RigidBodyData {
    std::string name ;
    btTransform lt ;
    btRigidBody *obj ;
};

struct SoftBodyData {

    std::map<std::string, int> idxs ;
    std::map<std::string, std::string> anchors ;

    btSoftBody *obj ;
};


class SoftBody {
public:

    SoftBody() {}

    virtual void constructMesh(std::vector<btVector3> &vtx, std::vector<int> &triangles, std::map<std::string, int> &anchorMap) const = 0 ;

};

class Cloth: public SoftBody {

public:
    Cloth(double u, double v, int nu, int nv) ;

    virtual void constructMesh(std::vector<btVector3> &vtx, std::vector<int> &triangles, std::map<std::string, int> &anchorMap) const  ;

private:

    double u, v ;
    unsigned int nu, nv ;
};


class Physics {

public:

    Physics(const planning_environment::CollisionModels &cm, const planning_models::KinematicState &state, double padding = 0.0001) ;
    ~Physics() ;

    void addSoftBody(const std::string &name, const SoftBody &sb) ;

    void updateCollisions(const planning_models::KinematicState &state) ;
    void updatePhysicsSimulation(double step) ;
    void attachSoftBodyToLink(const std::string &bname, const std::string &body_pos,
                              const std::string &link_name, const planning_models::KinematicState &state) ;
    void clearSoftBodyAttachments(const std::string &bname) ;

    void getMeshMarker(const std::string &bname, visualization_msgs::Marker &marker) ;

private:

    void create(const planning_environment::CollisionModels &cm, const planning_models::KinematicState &state, double padding) ;



    void updateAnchor(btSoftBody *body, int nodeIdx, const btVector3 &pos ) ;



    struct PhysicsPriv *impl_ ;

    btCollisionDispatcher *dispatcher ;
    btSoftBodyRigidBodyCollisionConfiguration *config ;
    btBroadphaseInterface *broadphase ;
    btSoftBodyWorldInfo sb_world_info ;
    btSoftRigidDynamicsWorld * world ;
    btSequentialImpulseConstraintSolver *solver ;
    btSoftBody *softBody ;

    std::map<std::string, boost::shared_ptr<RigidBodyData> > colData ;
    std::map<std::string, boost::shared_ptr<SoftBodyData> > sbData ;

};







#endif
