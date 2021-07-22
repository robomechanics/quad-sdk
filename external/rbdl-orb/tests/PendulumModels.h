#include "rbdl/rbdl.h"

struct DoublePerpendicularPendulumJointCoordinates {
  DoublePerpendicularPendulumJointCoordinates()
    : model()
    , q()
    , qd()
    , qdd()
    , tau()
    , l1(1.)
    , l2(1.)
    , m1(1.)
    , m2(1.)
    , idB1(0)
    , idB2(0){
    using namespace RigidBodyDynamics;
    using namespace RigidBodyDynamics::Math;

    model.gravity = Vector3d(0.,-9.81,0.);
    /*
      The perpendicular pendulum pictured with joint angles of 0,0.
      The first joint rotates about the x axis, while the second
      joint rotates about the local y axis of link 1

         y
         |
         |___ x
      z / |
        | |
      / | | link1
        | |
    /   | |
axis1:z0| |__________
       (_____________) link 2
        | |
         |

         |

         | axis2:y1
    */

    Body link1 = Body(m1, Vector3d(          0., -l1*0.5,          0.),
                      Matrix3d( m1*l1*l1/3.,           0.,           0.,
                                         0., m1*l1*l1/30.,           0.,
                                         0.,           0.,  m1*l1*l1/3.));


    Body link2 = Body(m2, Vector3d( l2*0.5,          0.,          0.),
                          Matrix3d( m2*l2*l2/30.,          0.,           0.,
                                              0., m2*l2*l2/3.,           0.,
                                              0.,          0.,  m2*l2*l2/3.));


    Joint joint_rev_z = Joint(SpatialVector(0.,0.,1.,0.,0.,0.));
    Joint joint_rev_y = Joint(SpatialVector(0.,1.,0.,0.,0.,0.));

    idB1 = model.AddBody(   0, Xtrans(Vector3d(0., 0, 0. )),
                            joint_rev_z, link1, "body1");
    idB2 = model.AddBody(idB1, Xtrans(Vector3d(0.,-l1, 0.)),
                         joint_rev_y, link2, "body2");

    q   = VectorNd::Zero(model.dof_count);
    qd  = VectorNd::Zero(model.dof_count);
    qdd = VectorNd::Zero(model.dof_count);
    tau = VectorNd::Zero(model.dof_count);
  }

  RigidBodyDynamics::Model model;

  RigidBodyDynamics::Math::VectorNd q;
  RigidBodyDynamics::Math::VectorNd qd;
  RigidBodyDynamics::Math::VectorNd qdd;
  RigidBodyDynamics::Math::VectorNd tau;


  double l1;
  double l2;
  double m1;
  double m2;

  unsigned int idB1;
  unsigned int idB2;
};

struct DoublePerpendicularPendulumAbsoluteCoordinates {
  DoublePerpendicularPendulumAbsoluteCoordinates()
    : model()
    , cs()
    , q()
    , qd()
    , qdd()
    , tau()
    , l1(1.)
    , l2(1.)
    , m1(1.)
    , m2(1.)
    , idB1(0)
    , idB2(0)
    , X_p1(RigidBodyDynamics::Math::Xtrans(
            RigidBodyDynamics::Math::Vector3d(0., 0., 0.)))
    , X_s1(RigidBodyDynamics::Math::Xtrans(
            RigidBodyDynamics::Math::Vector3d(0., 0., 0.)))
    , X_p2(RigidBodyDynamics::Math::Xtrans(
            RigidBodyDynamics::Math::Vector3d(0.,-l1, 0.)))
    , X_s2(RigidBodyDynamics::Math::Xtrans(
            RigidBodyDynamics::Math::Vector3d(0., 0., 0.))){

    using namespace RigidBodyDynamics;
    using namespace RigidBodyDynamics::Math;

    model.gravity = Vector3d(0.,-9.81,0.);
    //Planar pendulum is at 0 when it is hanging down.
    //  x: points to the right
    //  y: points up
    //  z: out of the page
    Body link1 = Body(m1, Vector3d(          0., -l1*0.5,          0.),
                      Matrix3d( m1*l1*l1/3.,          0. ,           0.,
                                         0., m1*l1*l1/30.,           0.,
                                         0.,          0. ,  m1*l1*l1/3.));

    r2C2 = Vector3d( l2*0.5,          0.,          0.);
    Body link2 = Body(m2, r2C2,
                          Matrix3d( m2*l2*l2/30.,          0.,           0.,
                                              0., m2*l2*l2/3.,           0.,
                                              0.,          0.,  m2*l2*l2/3.));


    Body nullbody = Body(0, Vector3d( 0.,0.,0.),
                            Vector3d( 0.,0.,0.));
    Joint jointT123 (JointTypeTranslationXYZ);
    Joint jointA123 (JointTypeEulerZYX);

    model.AddBody(0, SpatialTransform(), jointT123, nullbody);
    idB1 = model.AppendBody(SpatialTransform(), jointA123, link1);

    model.AddBody(0, SpatialTransform(), jointT123, nullbody);
    idB2 = model.AppendBody(SpatialTransform(), jointA123, link2);

    //Make the revolute joints about the y axis using 5 constraints
    //between the end points
    cs.AddLoopConstraint(0, idB1, X_p1, X_s1, SpatialVector(0,0,0,1,0,0),
                        false,0.1,"RzBase");
    cs.AddLoopConstraint(0, idB1, X_p1, X_s1, SpatialVector(0,0,0,0,1,0));
    cs.AddLoopConstraint(0, idB1, X_p1, X_s1, SpatialVector(0,0,0,0,0,1));
    cs.AddLoopConstraint(0, idB1, X_p1, X_s1, SpatialVector(1,0,0,0,0,0));
    cs.AddLoopConstraint(0, idB1, X_p1, X_s1, SpatialVector(0,1,0,0,0,0));


    cs.AddLoopConstraint(idB1, idB2, X_p2, X_s2, SpatialVector(0,0,0,1,0,0),
                         false,0.1, "RyLink1");
    cs.AddLoopConstraint(idB1, idB2, X_p2, X_s2, SpatialVector(0,0,0,0,1,0));
    cs.AddLoopConstraint(idB1, idB2, X_p2, X_s2, SpatialVector(0,0,0,0,0,1));
    cs.AddLoopConstraint(idB1, idB2, X_p2, X_s2, SpatialVector(1,0,0,0,0,0));
    cs.AddLoopConstraint(idB1, idB2, X_p2, X_s2, SpatialVector(0,0,1,0,0,0));

    cs.Bind(model);

    q   = VectorNd::Zero(model.dof_count);
    qd  = VectorNd::Zero(model.dof_count);
    qdd = VectorNd::Zero(model.dof_count);
    tau = VectorNd::Zero(model.dof_count);
  }


  RigidBodyDynamics::Model model;
  RigidBodyDynamics::ConstraintSet cs;

  RigidBodyDynamics::Math::Vector3d r2C2;
  RigidBodyDynamics::Math::VectorNd q;
  RigidBodyDynamics::Math::VectorNd qd;
  RigidBodyDynamics::Math::VectorNd qdd;
  RigidBodyDynamics::Math::VectorNd tau;

  double l1;
  double l2;
  double m1;
  double m2;

  unsigned int idB1;
  unsigned int idB2;

  RigidBodyDynamics::Math::SpatialTransform X_p1;
  RigidBodyDynamics::Math::SpatialTransform X_s1;
  RigidBodyDynamics::Math::SpatialTransform X_p2;
  RigidBodyDynamics::Math::SpatialTransform X_s2;
};

//==============================================================================
struct SinglePendulumJointCoordinates {
  SinglePendulumJointCoordinates()
    : model()
    , q()
    , qd()
    , qdd()
    , tau()
    , l1(1.)
    , m1(1.)
    , idB1(0){
    using namespace RigidBodyDynamics;
    using namespace RigidBodyDynamics::Math;

    model.gravity = Vector3d(0.,-9.81,0.);
    /*
      The perpendicular pendulum pictured with joint angles of 0,0.
      The first joint rotates about the x axis, while the second
      joint rotates about the local y axis of link 1

         y
         |
         |___ x
      z / |
        | |
      / | | link1
        | |
    /   | |
        | |
        +-+
    */

    Body link1 = Body(m1, Vector3d(          0., -l1*0.5,          0.),
                      Matrix3d( m1*l1*l1/3.,           0.,           0.,
                                         0., m1*l1*l1/30.,           0.,
                                         0.,           0.,  m1*l1*l1/3.));



    Joint joint_rev_z = Joint(SpatialVector(0.,0.,1.,0.,0.,0.));

    idB1 = model.AddBody(   0, Xtrans(Vector3d(0., 0, 0. )),
                            joint_rev_z, link1, "body1");

    q   = VectorNd::Zero(model.dof_count);
    qd  = VectorNd::Zero(model.dof_count);
    qdd = VectorNd::Zero(model.dof_count);
    tau = VectorNd::Zero(model.dof_count);
  }

  RigidBodyDynamics::Model model;

  RigidBodyDynamics::Math::VectorNd q;
  RigidBodyDynamics::Math::VectorNd qd;
  RigidBodyDynamics::Math::VectorNd qdd;
  RigidBodyDynamics::Math::VectorNd tau;

  double l1;
  double m1;

  unsigned int idB1;
};

struct SinglePendulumAbsoluteCoordinates {
  SinglePendulumAbsoluteCoordinates()
    : model()
    , cs()
    , q()
    , qd()
    , qdd()
    , tau()
    , l1(1.)
    , m1(1.)
    , idB1(0)
    , X_p1(RigidBodyDynamics::Math::Xtrans(
            RigidBodyDynamics::Math::Vector3d(0., 0., 0.)))
    , X_s1(RigidBodyDynamics::Math::Xtrans(
            RigidBodyDynamics::Math::Vector3d(0., 0., 0.))){

    using namespace RigidBodyDynamics;
    using namespace RigidBodyDynamics::Math;

    model.gravity = Vector3d(0.,-9.81,0.);
    //Planar pendulum is at 0 when it is hanging down.
    //  x: points to the right
    //  y: points up
    //  z: out of the page
    Body link1 = Body(m1, Vector3d(          0., -l1*0.5,          0.),
                      Matrix3d( m1*l1*l1/3.,          0. ,           0.,
                                         0., m1*l1*l1/30.,           0.,
                                         0.,          0. ,  m1*l1*l1/3.));

    Joint jointXYRz (SpatialVector(0.,0.,0., 1.,0.,0.),
                     SpatialVector(0.,0.,0., 0.,1.,0.),
                     SpatialVector(0.,0.,1., 0.,0.,0.));

    idB1 = model.AddBody(0, SpatialTransform(), jointXYRz, link1);

    //Make the revolute joints about the y axis using 5 constraints
    //between the end points
    cpX = cs.AddContactConstraint(idB1,Vector3dZero,Vector3d(1.,0.,0.),"cx");
    cpY = cs.AddContactConstraint(idB1,Vector3dZero,Vector3d(0.,1.,0.),"cy");

    cs.Bind(model);

    q   = VectorNd::Zero(model.dof_count);
    qd  = VectorNd::Zero(model.dof_count);
    qdd = VectorNd::Zero(model.dof_count);
    tau = VectorNd::Zero(model.dof_count);
  }


  RigidBodyDynamics::Model model;
  RigidBodyDynamics::ConstraintSet cs;

  RigidBodyDynamics::Math::VectorNd q;
  RigidBodyDynamics::Math::VectorNd qd;
  RigidBodyDynamics::Math::VectorNd qdd;
  RigidBodyDynamics::Math::VectorNd tau;

  double l1;
  double m1;

  unsigned int idB1;
  unsigned int cpX,cpY;

  RigidBodyDynamics::Math::SpatialTransform X_p1;
  RigidBodyDynamics::Math::SpatialTransform X_s1;
};
