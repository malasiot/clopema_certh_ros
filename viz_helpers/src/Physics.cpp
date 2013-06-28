#include "ClothSimulator.h"

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>

#include <boost/algorithm/string.hpp>
#include <tf/LinearMath/Vector3.h>
#include <set>

using namespace std ;


std::ostream &operator << ( std::ostream &strm, const btVector3 &vec )
{

    strm << vec.x() <<  ' ' << vec.y() << ' ' << vec.z() ;

    return strm ;

}

Physics::Physics(const planning_environment::CollisionModels &cm, const planning_models::KinematicState &state, double padding)
{
    btVector3 worldAabbMin( -10000, -10000, -10000 );
    btVector3 worldAabbMax( 10000, 10000, 10000 );
    //broadphase = new btAxisSweep3( worldAabbMin, worldAabbMax, 1000 );
    broadphase = new btDbvtBroadphase();
    sb_world_info.m_broadphase = broadphase;

    config = new btSoftBodyRigidBodyCollisionConfiguration();
    dispatcher = new  btCollisionDispatcher(config);

    sb_world_info.m_dispatcher = dispatcher;

    solver = new btSequentialImpulseConstraintSolver();


    world = new btSoftRigidDynamicsWorld( dispatcher,
                                            broadphase,
                                            solver,
                                            config,
                                            NULL);

    //            m_dynamicsWorld->setInternalTickCallback(pickingPreTickCallback,this,true);


    btVector3 gravity( 0, 0, -10 );
    world->setGravity(gravity) ;

   // world->getDispatchInfo().m_enableSPU = true;

    sb_world_info.m_gravity = gravity;
//    sb_world_info.air_density = btScalar( 1.0 );
 //   sb_world_info.water_density = 0;
 //   sb_world_info.water_offset = 0;
 //   sb_world_info.water_normal = btVector3( 0, 0, 0 );

    sb_world_info.m_sparsesdf.Initialize();

    create(cm, state, padding) ;
}

Physics::~Physics() {
    delete world ;
    delete dispatcher ;
    delete config ;
    delete broadphase ;
    delete solver ;

}

static btCollisionObject* createCollisionBody(const shapes::Shape *shape, double scale, double padding)
{
    btCollisionShape *btshape = NULL;

    switch (shape->type)
    {
        case shapes::SPHERE:
        {
            btshape = dynamic_cast<btCollisionShape*>(new btSphereShape(static_cast<const shapes::Sphere*>(shape)->radius * scale + padding));
        }
        break;
        case shapes::BOX:
        {
            const double *size = static_cast<const shapes::Box*>(shape)->size;
            btshape = dynamic_cast<btCollisionShape*>(new btBoxShape(btVector3(size[0] * scale / 2.0 + padding, size[1] * scale / 2.0 + padding, size[2] * scale / 2.0 + padding)));
        }
        break;
        case shapes::CYLINDER:
        {
            double r2 = static_cast<const shapes::Cylinder*>(shape)->radius * scale + padding;
            btshape = dynamic_cast<btCollisionShape*>(new btCylinderShapeZ(btVector3(r2, r2, static_cast<const shapes::Cylinder*>(shape)->length * scale / 2.0 + padding)));
        }
        break;
        case shapes::MESH:
        {
            const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);
            btConvexHullShape *btmesh = new btConvexHullShape();

            for (unsigned int i = 0 ; i < mesh->vertexCount ; ++i)
                btmesh->addPoint(btVector3(mesh->vertices[3*i], mesh->vertices[3*i + 1], mesh->vertices[3*i + 2]));

            btmesh->setLocalScaling(btVector3(scale, scale, scale));
            btmesh->setMargin(padding + 0.0001); // we need this to be positive
            btshape = dynamic_cast<btCollisionShape*>(btmesh);
        }

        default:
        break;
    }

    if ( btshape )
    {
        btCollisionObject *object = new btCollisionObject();
        object->setCollisionShape(btshape);
        return object;
    }
    else
        return NULL;
}



void Physics::create(const planning_environment::CollisionModels &desc, const planning_models::KinematicState &state, double padding)
{

    const collision_space::EnvironmentModel *cs = desc.getCollisionSpace() ;

    const planning_models::KinematicModel *robot = cs->getRobotModel() ;

    const std::vector<planning_models::KinematicModel::LinkModel *> links = robot->getLinkModels() ;

    for(int i=0 ; i<links.size() ; i++)
    {
        const planning_models::KinematicModel::LinkModel *link_ = links[i] ;

        string name = link_->getName() ;

        const planning_models::KinematicState::LinkState* link_state_ = state.getLinkState(name);

        btTransform gt, lt ;

        btCollisionObject *collisionObj = 0 ;

        const shapes::Shape* shape = link_->getLinkShape();

        if ( !shape )
        {
            btCollisionShape *btShape = dynamic_cast<btCollisionShape*>(new btSphereShape(0.0001));
            collisionObj = new btCollisionObject();
            collisionObj->setCollisionShape(btShape);

            gt = link_state_->getGlobalLinkTransform().asBt() ;

        }
        else
        {
            collisionObj = createCollisionBody(shape, 1.0, padding) ;

            gt = link_state_->getGlobalLinkTransform().asBt() ;

        }

        boost::shared_ptr<RigidBodyData> data(new RigidBodyData) ;

        btVector3 localInertia(0,0,0);

        btTransform c = gt  ;

        btDefaultMotionState* myMotionState = new btDefaultMotionState(c);

        btRigidBody::btRigidBodyConstructionInfo cInfo(0.0, myMotionState, collisionObj->getCollisionShape(),
                                                          localInertia);
        btRigidBody *body = new btRigidBody(cInfo);

        body->setFriction(1.0) ;

        data->obj = body ;
        data->lt = lt ;
        data->name = name ;

        colData[name] = data ;

        body->setUserPointer(data.get()) ;
        world->addRigidBody(body);

    }


}


void Physics::updateCollisions(const planning_models::KinematicState &state)
{
    const btCollisionObjectArray &colObjs = world->getCollisionObjectArray() ;

    for( int i=0 ; i<colObjs.size() ; i++ )
    {
        btCollisionObject *obj = colObjs[i] ;

        btSoftBody *sb = dynamic_cast<btSoftBody *>(obj) ;
        btRigidBody *rb = dynamic_cast<btRigidBody *>(obj) ;

        if ( rb )
        {

            RigidBodyData *data = static_cast<RigidBodyData *>(obj->getUserPointer()) ;

            btTransform tr = state.getLinkState(data->name)->getGlobalLinkTransform().asBt() ;

            btTransform c = tr  ;

            obj->setWorldTransform( c );
            continue ;
        }

        if ( sb )
        {

            SoftBodyData *sb_data = static_cast<SoftBodyData *>(obj->getUserPointer()) ;

            map<string, string>::const_iterator it = sb_data->anchors.begin() ;

            for( ; it != sb_data->anchors.end() ; ++it )
            {

                btTransform tr = state.getLinkState((*it).second)->getGlobalLinkTransform().asBt() ;

                updateAnchor((btSoftBody *)obj, sb_data->idxs[(*it).first], tr.getOrigin()) ;

            }


        }

    }


}




/////////////////////////////////////////////////////////////////////////////////////

void Physics::updateAnchor(btSoftBody *body, int nodeIdx, const btVector3 &pos )
{
    btSoftBody::tNodeArray& nodes = body->m_nodes;

    //nodes[nodeIdx].m_x = pos ;


  //  body->setMass(nodeIdx, btScalar(0.0)) ;
 }



void Physics::addSoftBody(const std::string &name, const SoftBody &sb)
{

    vector<int> triangles ;
    vector<btVector3> vertices ;

    map<string, int> anchorMap ;

    sb.constructMesh(vertices, triangles, anchorMap) ;

    int *triangles_ = new int [triangles.size()] ;
    std::copy(triangles.begin(), triangles.end(), triangles_) ;

    double *vertices_ = new double [vertices.size() * 3 ] ;
    for(int i=0, k=0 ; i<vertices.size() ; i++)
    {
        vertices_[k++] = vertices[i].x() ;
        vertices_[k++] = vertices[i].y() ;
        vertices_[k++] = vertices[i].z() ;
    }

    btSoftBody* softBody =  btSoftBodyHelpers::CreateFromTriMesh(sb_world_info, vertices_, triangles_, triangles.size()/3) ;

    boost::shared_ptr<SoftBodyData> pData(new SoftBodyData) ;

    pData->obj = softBody ;

    map<string, int>::const_iterator it = anchorMap.begin() ;

    pData->idxs = anchorMap ;

    sbData[name] = pData  ;

    softBody->setUserPointer(pData.get()) ;

    softBody->getCollisionShape()->setMargin( 0.0001 );

    softBody->m_materials[ 0 ]->m_kLST = 0.1;   // Linear stiffness coefficient [0,1]
    softBody->m_materials[ 0 ]->m_kAST = 0.1 ;  // Area/Angular stiffness coefficient [0,1]
    softBody->m_materials[ 0 ]->m_kVST = 0.1 ;  // Volume stiffness coefficient [0,1]

    softBody->generateBendingConstraints( 2, softBody->m_materials[ 0 ] );
    softBody->setTotalMass(btScalar(1) );
    softBody->generateClusters(512);

    softBody->m_cfg.collisions	=	0;
    softBody->m_cfg.collisions += btSoftBody::fCollision::SDF_RS; ///SDF based rigid vs soft
      //psb->m_cfg.collisions += btSoftBody::fCollision::CL_RS; ///Cluster vs convex rigid vs soft
      //psb->m_cfg.collisions += btSoftBody::fCollision::VF_SS; ///Vertex vs face soft vs soft handling
    softBody->m_cfg.collisions += btSoftBody::fCollision::CL_SS; ///Cluster vs cluster soft vs soft handling
    softBody->m_cfg.collisions	+= btSoftBody::fCollision::CL_SELF; ///Cluster soft body self collision

    softBody->m_cfg.piterations = 50; // for more fine meshes more iterations will be need. this value is good for 1000 vertices
    softBody->m_cfg.citerations = 50;
    softBody->m_cfg.diterations = 50;
    softBody->m_cfg.viterations = 50;

    softBody->m_cfg.kDF = 0.9 ;

    softBody->m_cfg.kSRHR_CL =	0.1f;
    softBody->m_cfg.kSKHR_CL =	1.f;
    softBody->m_cfg.kSSHR_CL =	0.5f;
    softBody->m_cfg.kSR_SPLT_CL = 0.5f;

    softBody->m_cfg.kSK_SPLT_CL = 0.9f;
    softBody->m_cfg.kSS_SPLT_CL = 0.9f;
    softBody->m_cfg.kVCF =	1;
    softBody->m_cfg.kDP = 0.05;   //?

    softBody->m_cfg.kDG = 0.0;
    softBody->m_cfg.kLF = 0;
    softBody->m_cfg.kPR = 0;
    softBody->m_cfg.kVC = 0;

    softBody->m_cfg.kDF =	0.2f;
    softBody->m_cfg.kMT =	0.05;
    softBody->m_cfg.kCHR =	1.0f;
    softBody->m_cfg.kKHR =	0.9f;

    softBody->m_cfg.kSHR =	1.0f;
    softBody->m_cfg.kAHR =	1.0f;

  //   softBody->m_cfg.collisions|=btSoftBody::fCollision::CL_SELF;

    softBody->randomizeConstraints();

    world->addSoftBody(softBody);
}


void Physics::updatePhysicsSimulation(double step)
{
    world->stepSimulation( step );
    sb_world_info.m_sparsesdf.GarbageCollect();

}

void Physics::attachSoftBodyToLink(const std::string &bname, const std::string &body_pos,
                          const std::string &link_name, const planning_models::KinematicState &state)
{
    map<string, boost::shared_ptr<SoftBodyData> >::const_iterator sb =
            sbData.find(bname) ;

    assert( sb != sbData.end() ) ;

    map<string, boost::shared_ptr<RigidBodyData> >::const_iterator rb =
            colData.find(link_name) ;

    assert( rb != colData.end() ) ;

    SoftBodyData *sb_data = (*sb).second.get() ;
    RigidBodyData *rb_data = (*rb).second.get() ;

    int idx = -1 ;

    map<string, int>::const_iterator it  = sb_data->idxs.begin() ;

    for ( ; it != sb_data->idxs.end() ; ++it )
    {
        if ( (*it).first == body_pos ) {
            idx = (*it).second ;
            sb_data->anchors[(*it).first] = link_name ;
            break ;
        }
    }


    if ( idx < 0 ) return ;

    const planning_models::KinematicState::LinkState* link_state_ = state.getLinkState(link_name);

    if ( !link_state_) return ;

    btTransform gt = link_state_->getGlobalCollisionBodyTransform().asBt() ;


    btVector3 pivot = gt.getOrigin() ;


    sb_data->obj->appendAnchor(idx, rb_data->obj, btVector3(0, 0, -0.05), false, btScalar(1) ) ;

    ////   sb_data->obj->setMass(sb_data->idxs[idx], 0) ;
    sb_world_info.m_sparsesdf.Reset();


}

void Physics::clearSoftBodyAttachments(const std::string &bname)
{
    map<string, boost::shared_ptr<SoftBodyData> >::const_iterator sb = sbData.find(bname) ;

    assert( sb != sbData.end() ) ;

    SoftBodyData *sb_data = (*sb).second.get() ;

    sb_data->obj->m_anchors.clear() ;
    sb_data->anchors.clear() ;

 }

void Physics::getMeshMarker(const std::string &bname, visualization_msgs::Marker &marker)
{
    map<string, boost::shared_ptr<SoftBodyData> >::const_iterator sb = sbData.find(bname) ;

    assert( sb != sbData.end() ) ;

    SoftBodyData *sb_data = (*sb).second.get() ;

    const btSoftBody::tNodeArray& nodes = sb_data->obj->m_nodes;
    const btSoftBody::tFaceArray& triangles = sb_data->obj->m_faces ;

    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();

    marker.ns = bname;
    marker.id = 0;

    marker.type = visualization_msgs::Marker::TRIANGLE_LIST ;

    for( int i=0 ; i<triangles.size() ; i++ )
    {
        const btSoftBody::Face &face = triangles[i] ;

        geometry_msgs::Point p ;

        p.x = face.m_n[0]->m_x.x() ;
        p.y = face.m_n[0]->m_x.y() ;
        p.z = face.m_n[0]->m_x.z() ;

        marker.points.push_back(p) ;

        p.x = face.m_n[1]->m_x.x() ;
        p.y = face.m_n[1]->m_x.y() ;
        p.z = face.m_n[1]->m_x.z() ;

        marker.points.push_back(p) ;

        p.x = face.m_n[2]->m_x.x() ;
        p.y = face.m_n[2]->m_x.y() ;
        p.z = face.m_n[2]->m_x.z() ;

        marker.points.push_back(p) ;

        btVector3 normal0 = face.m_n[0]->m_n ;
        btVector3 normal1 = face.m_n[1]->m_n ;
        btVector3 normal2 = face.m_n[2]->m_n ;

        cout << normal0.z() << endl ;
        std_msgs::ColorRGBA clr ;
        clr.r = fabs(normal0.z()) ;
        clr.g = fabs(normal0.z()) ;
        clr.b = fabs(normal0.z());
        clr.a = 1.0 ;

        marker.colors.push_back(clr) ;

        clr.r = fabs(normal1.z()) ;
        clr.g = fabs(normal1.z()) ;
        clr.b = fabs(normal1.z());
        clr.a = 1.0 ;

        marker.colors.push_back(clr) ;

        clr.r = fabs(normal2.z()) ;
        clr.g = fabs(normal2.z()) ;
        clr.b = fabs(normal2.z());
        clr.a = 1 ;

        marker.colors.push_back(clr) ;


    }

    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.7f;
    marker.color.a = 1;

    marker.pose.orientation.x = 0 ;
    marker.pose.orientation.y = 0 ;
    marker.pose.orientation.z = 0 ;
    marker.pose.orientation.w = 1 ;


    marker.scale.x = 1.0 ;
    marker.scale.y = 1.0 ;
    marker.scale.z = 1.0 ;

    marker.lifetime = ros::Duration();




}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void buildPlaneData( const btVector3 &corner,
              const btVector3& u, unsigned short uSteps,
              const btVector3& v, unsigned short vSteps,
                     std::vector<btVector3> &vtx, std::vector<int> &triangles, std::map<string, int> &anchorMap
              )
 {
     int nVertices = (uSteps + 1)*(vSteps + 1) ;
     int nFaces = uSteps * vSteps * 2 ;

     anchorMap["corner0"] = 0 ;
     anchorMap["corner1"] = uSteps ;
     anchorMap["corner2"] = uSteps * vSteps ;
     anchorMap["corner3"] = uSteps * (vSteps - 1) ;

     unsigned short uIdx, vIdx;
     int k = 0 ;
     int nc = 0 ;

     for( vIdx=0; vIdx<=vSteps; vIdx++ )
     {
         const float vPct( (float)vIdx / (float)vSteps );
         const btVector3 vVec( v * vPct );

         for( uIdx=0; uIdx<=uSteps; uIdx++ )
         {
             const float uPct( (float)uIdx / (float)uSteps );

             btVector3 vertex( corner + vVec + (u * uPct) );

             vtx.push_back(vertex) ;
         }
     }

     k = 0 ;
     for( vIdx=0; vIdx<vSteps; vIdx++ )
     {

         for( uIdx=0; uIdx<uSteps; uIdx++ )
         {
             triangles.push_back(vIdx * (uSteps+1) + uIdx) ;
             triangles.push_back(vIdx * (uSteps+1) + uIdx + 1);
             triangles.push_back((vIdx + 1) * (uSteps+1) + uIdx) ;

             triangles.push_back(vIdx * (uSteps+1) + uIdx + 1) ;
             triangles.push_back((vIdx + 1) * (uSteps+1) + uIdx) ;
             triangles.push_back((vIdx + 1) * (uSteps+1) + uIdx + 1);

         }

     }


 }

 Cloth::Cloth(double su, double sv, int steps_u, int steps_v): u(su), v(sv), nu(steps_u), nv(steps_v)
 {

 }


 void Cloth::constructMesh(std::vector<btVector3> &vtx, std::vector<int> &triangles, std::map<string, int> &anchorMap) const {

     const btVector3 llCorner( 0, -1.0, 1.5 );

     const btVector3 uVec( u, 0., 0 );
     const btVector3 vVec( 0., v, 0 );

     buildPlaneData(llCorner, uVec, nu, vVec,  nv, vtx, triangles, anchorMap) ;

 }
