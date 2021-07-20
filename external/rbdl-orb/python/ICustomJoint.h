
#ifndef ICUSTOMJOINT_H_
#define ICUSTOMJOINT_H_

// Created by Cython when providing 'public api' keywords
#include "rbdl-python_api.h"

#include <rbdl/rbdl_math.h>
#include <rbdl/Joint.h>
#include <rbdl/Model.h>

namespace RigidBodyDynamics {
    

  class ICustomJoint : public CustomJoint {
  public:
      PyObject *m_obj;

      ICustomJoint(PyObject *obj);
      virtual ~ICustomJoint();
      
      virtual void jcalc (Model &model,
        unsigned int joint_id,
        const Math::VectorNd &q,
        const Math::VectorNd &qdot
        );
        
      virtual void jcalc_X_lambda_S (Model &model,
        unsigned int joint_id,
        const Math::VectorNd &q
        );
        
  };
  
  
  


}
#endif 
