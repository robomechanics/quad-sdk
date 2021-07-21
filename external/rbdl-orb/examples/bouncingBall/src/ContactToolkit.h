/*====================================================================
   ContactToolkit.h
   Copyright (c) 2019 Matthew Millard <matthew.millard@iwr.uni-heidelberg.de>
   Licensed under the zlib license. See LICENSE for more details.
 *///=================================================================

#ifndef CONTACTTOOLKIT_H_
#define CONTACTTOOLKIT_H_

#include <rbdl/rbdl.h>
#include <rbdl/addons/geometry/geometry.h>


struct HuntCrossleyContactInfo{

  double force;
  double springForce;
  double dampingForce;
};

class ContactToolkit
{
  public:

  /**
    \brief A function to compute the contact point location between a sphere
           and a plane contact pair.

  @param r0S0 : center of the sphere in the root frame
  @param r    : sphere radius
  @param eN0  : normal vector of the plane
  @param r0K0 : contact point location in the root frame
  */
  static void calcSpherePlaneContactPointPosition(
                  const RigidBodyDynamics::Math::Vector3d &r0S0, 
                  double r,
                  const RigidBodyDynamics::Math::Vector3d &eN0,
                  RigidBodyDynamics::Math::Vector3d &r0K0)
  {
   r0K0 = r0S0 - r*eN0;
  }

  /**
    \brief Low-level function which evaluates Hunt-Crossley's
           numerically well-behaved contact model in
           which damping forces scale with elastic forces.

\f[
                \tau_{M} = \pm \tau_{ISO}( \mathbf{a} \, \mathbf{t}_A(\theta) \mathbf{t}_V(\dot{\theta}/\dot{\theta}_{MAX})
                                 +\mathbf{t}_P(1- \beta (\dot{\theta}/\dot{\theta}_{MAX})) \, )
                \f]

    f*  = k*pow(-x,p)*( 1.0 - beta*dx)  

            f*, f* > 0 & x < 0 (e.g. no suction) 
    f   =
            0, otherwise

    [1] K Hunt and F Crossley. Coefficient of restitution interpreted as damping
    in vibroimpact. Transactions of the ASME Journal of Applied Mechanics,
    42(E):440445, 1975.

    @param z  : penetration depth (-ve sign means more compression)
    @param dz : penetration speed (-ve sign means more compression faster)
    @param k  : stiffness
    @param p  : the exponent applied to the penetration depth
    @param beta: a damping coefficient which has been normalized by the
                 spring force
    @param hcInfo: a structure to store the force, spring force, and damping
                   force returned by this model
  */
  static void calcHuntCrossleyContactForce(
                double z,
                double dz,
                double k,
                double p,
                double beta,
                HuntCrossleyContactInfo &hcInfo)
  {
    if(z < 0.0){
      hcInfo.springForce = k*std::pow(-z,p);
      hcInfo.dampingForce= hcInfo.springForce*(-beta*dz);
      hcInfo.force = hcInfo.springForce + hcInfo.dampingForce;

      //Damper is generating suction forces
      if(hcInfo.force < 0.){
        hcInfo.force        = 0.;
        hcInfo.springForce  = 0.;
        hcInfo.dampingForce = 0.;        
      }
    }else{
        hcInfo.force        = 0.;
        hcInfo.springForce  = 0.;
        hcInfo.dampingForce = 0.; 
    }
  }


  /**
    \brief Creates a C2 continuous Bezier spline that represents the
           coefficent of friction as a function of velocity.

    \par This implements a regularized friction model in which the coefficient
         of friction is described as a function of the tangental velocity
         between two points. The curve is similar to the one that appears in
         Brown et al. The reason why Brown et al.'s curve has not been used
         directly is that it is more convenient to implement this curve as
         a Bezier curve: the function values and derivatives are available
         once the curve has been constructed. As Bezier curves are very fast
         to evaluate, it is not clear to me that Brown et al. (or anyone else
         making friction models) have not chosen to use Bezier curves.

    \par This formulation means that the coefficient
         of friction must (sharply) go to zero at zero velocity: otherwise
         the resulting equations of motion will be very stiff. The physical
         consequence of this model is that, if any tangential force is applied,
         the two surfaces will be moving. If you need a model that can generate
         static stiction forces please see Gonthier et al.

    @param staticFrictionSpeed (m/s)
            The non-zero speed at which static friction reaches its peak value
            (staticFrictionSpeed > 0)
    @param staticFrictionCoefficient (N/N)
            The peak coefficient of friction. (staticFrictionCoefficient >= 0)
    @param dynamicFrictionSpeed (m/s),
            The speed at which the friction coefficient transitions from
            static friction to dynamic friction
            (dynamicFrictionsSpeed > staticFrictionSpeed)
    @param dynamicFrictionCoefficient (N/N)
            The minimum friction coefficient once the two surfaces are moving.
            (dynamicFrictionCoefficient <= staticFrictionCoefficient
    @param viscousFrictionSlope (N/N)/(m/s)
            The rate change increase in friction coefficient for tangental
            surface speeds greater than the dynamicFrictionSpeed.
            (viscousFrictionSlope >= 0)

    [1] Brown P, McPhee J. A continuous velocity-based friction model for
    dynamics and control with physically meaningful parameters. Journal of
    Computational and Nonlinear Dynamics. 2016 Sep 1;11(5):054502.

    [2] Gonthier Y, McPhee J, Lange C, Piedboeuf JC. A regularized contact
    model with asymmetric damping and dwell-time dependent friction. Multibody
    System Dynamics. 2004 Apr 1;11(3):209-33.
  */
  static void createRegularizedFrictionCoefficientCurve(
        double staticFrictionSpeed,
        double staticFrictionCoefficient,
        double dynamicFrictionSpeed,
        double dynamicFrictionCoefficient,
        double viscousFrictionSlope,
        const std::string& curveName,
        RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction&
          smoothSegmentedFunctionToUpdate )
  {


    if( staticFrictionSpeed <= 0 ){
      std::cerr  << "Required: staticFrictionSpeed > 0, but given "
                 << staticFrictionSpeed << std::endl;
      assert(0);
      abort();
    }
    if( staticFrictionCoefficient < 0 ){
      std::cerr  << "Required: staticFrictionCoefficient >= 0, but given "
                 << staticFrictionCoefficient << std::endl;
      assert(0);
      abort();
    }

    if( dynamicFrictionSpeed < staticFrictionSpeed){
      std::cerr  << "Required: dynamicFrictionSpeed > staticFrictionSpeed,"
                 << "but given " << dynamicFrictionSpeed
                 << " !> " << staticFrictionSpeed
                 << std::endl;
      assert(0);
      abort();
    }
    if( dynamicFrictionCoefficient > staticFrictionCoefficient){
      std::cerr  << "Required: dynamicFrictionCoefficient <= staticFrictionCoeffient,"
                 << "but given " << dynamicFrictionCoefficient
                 << " !<= " << staticFrictionCoefficient
                 << std::endl;
      assert(0);
      abort();
    }
    if( viscousFrictionSlope < 0){
      std::cerr  << "Required: viscousFrictionSlope > 0, but given "
                 << viscousFrictionSlope << std::endl;
      assert(0);
      abort();
    }

    RigidBodyDynamics::Math::MatrixNd  xM(6,3);
    RigidBodyDynamics::Math::MatrixNd  yM(6,3);
    RigidBodyDynamics::Math::MatrixNd pts(6,2);

    double curviness = 0.5;
    double c = RigidBodyDynamics::Addons::Geometry::
                SegmentedQuinticBezierToolkit::scaleCurviness(curviness);

    double x0,x1,x2,x3,y0,y1,y2,y3,dydx0,dydx1,dydx2,dydx3;
    x0 = 0;
    x1 = staticFrictionSpeed;
    x2 = dynamicFrictionSpeed;
    x3 = x2 + (x2-x1);

    y0 = 0;
    y1 = staticFrictionCoefficient;
    y2 = dynamicFrictionCoefficient;
    y3 = y2 + 0.5*(x3-x2)*viscousFrictionSlope;

    dydx0 = 2.*(y1-y0)/(x1-x0);
    dydx1 = 0.;
    dydx2 = 0.;
    dydx3 = viscousFrictionSlope;


    pts = RigidBodyDynamics::Addons::Geometry::SegmentedQuinticBezierToolkit::
            calcQuinticBezierCornerControlPoints(
              x0,  y0,  dydx0,
              x1,  y1,  dydx1 , c);
    xM.col(0) = pts.col(0);
    yM.col(0) = pts.col(1);

    pts = RigidBodyDynamics::Addons::Geometry::SegmentedQuinticBezierToolkit::
            calcQuinticBezierCornerControlPoints(
              x1,  y1,  dydx1,
              x2,  y2,  dydx2 , c);
    xM.col(1) = pts.col(0);
    yM.col(1) = pts.col(1);

    pts = RigidBodyDynamics::Addons::Geometry::SegmentedQuinticBezierToolkit::
            calcQuinticBezierCornerControlPoints(
              x2,  y2,  dydx2,
              x3,  y3,  dydx3 , c);
    xM.col(2) = pts.col(0);
    yM.col(2) = pts.col(1);


    smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
          xM,yM,
          x0,x3,
          y0,y3,
          dydx0,dydx3,
          curveName);
  }


  /**
    \brief Computes the unit direction vector of the tangental velocity
           component using a numerically stable method described in
           Eqn 20 of Gonthier et al.

    \note This method is not C2 continuous, and as such will pose problems
          when applied to optimal control problems. This method can be made
          C2 continuous by replacing the hard switching statment with a
          soft one by making use of a soft step function. Such a soft-step
          function can be easily created using the SmoothSegmentedFunction
          memeber funrction updSmoothSegmentedFunction;

    [1] Gonthier Y, McPhee J, Lange C, Piedboeuf JC. A regularized contact
    model with asymmetric damping and dwell-time dependent friction. Multibody
    System Dynamics. 2004 Apr 1;11(3):209-33.

    @param vt : the tangential velocity
    @param veps: a small velocity at which the relaxed method is used to
                 compute the direction vector.
    @param eT0: the direction vector of vt. Note this goes to zero as |vt| goes
                to zero.
  */
  static void calcTangentialVelocityDirection(
              const RigidBodyDynamics::Math::Vector3d &vt,
              double veps,
              RigidBodyDynamics::Math::Vector3d &eT0)
  {
    double vtn = vt.norm();

    if(vtn >= veps){
      eT0 = vt/vtn;
    }else{
      //RigidBodyDynamics::Math::Vector3d vte = vt*(1./veps);
      //I'm going to avoid creating a new Vector3d by using eT0
      eT0 = vt*(1./veps);
      double ste = vtn/veps;
      eT0 = eT0*( (3./2.)*(ste)-(1./2.)*(ste*ste*ste) );
    }
  }

};


#endif //CONTACTTOOLKIT_H_
