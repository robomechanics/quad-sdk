
#include "ICustomJoint.h"


namespace RigidBodyDynamics{
  
  
ICustomJoint::ICustomJoint(PyObject *obj): m_obj(obj) {
    // Provided by "elps_api.h"
    std::cout << "import " << import_rbdl() << std::endl;
    if (import_rbdl()) {
    } else {
        Py_XINCREF(this->m_obj);
    }
}

ICustomJoint::~ICustomJoint() {
    Py_XDECREF(this->m_obj);
}

void ICustomJoint::jcalc (Model &model,
      unsigned int joint_id,
      const Math::VectorNd &q,
      const Math::VectorNd &qdot
      )
{
    if (this->m_obj) {
        int error;
        // Call a virtual overload, if it exists
     /*   std::cout << joint_id << std::endl;
        std::cout << q.transpose() << std::endl;
        std::cout << qdot.transpose() << std::endl;
        std::cout << "qindex " << model.mJoints[joint_id].q_index << std::endl;*/

        //test_model = &model;
        //std::cout << "test " << test_model->X_J[joint_id] << std::endl;
        std::cout << "c" << model.X_J[joint_id] << std::endl;
        std::cout << "lambda before cy[" << 0 << "] :" << model.lambda[0] << std::endl;
        
        cy_call_jcalc(this->m_obj, &model, joint_id, q, qdot, &error);

        std::cout << "lambda after cy[" << 0 << "] :" << model.lambda[0] << std::endl;

        if (error){
            std::cerr << "no jcalc function implemented";
        }
    }else{
      // Throw error ?
      std::cerr << "sth went wrong";
    }
}  

  

  
void ICustomJoint::jcalc_X_lambda_S (Model &model,
      unsigned int joint_id,
      const Math::VectorNd &q
      )
{
    if (this->m_obj) {
        int error;
        // Call a virtual overload, if it exists
        cy_call_jcalc_X_lambda_S(this->m_obj, &model, joint_id, q, &error);
        if (error){
            std::cerr << "no jcalc_X_lambda_S function implemented";
        }
    }else{
    // Throw error ?
    std::cerr << "sth went wrong";
    }
}   


  
  
}
