#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#include "urdfreader.h"

#include <iostream>
#include <iomanip>

using namespace std;

bool verbose = false;
bool floatbase = false;
string filename = "";

using namespace std;

using namespace RigidBodyDynamics::Math;

void usage (const char* argv_0) {
  cerr << "Usage: " << argv_0 << "[-v] [-m] [-d] <robot.urdf>" << endl;
  cerr << "  -v | --verbose            "
       << "enable additional output" << endl;
  cerr << "  -d | --dof-overview       "
       << "print an overview of the degress of freedom" << endl;
  cerr << "  -f | --floatbase          "
       <<"set the first mobile body as floating base" << endl;
  cerr << "  -m | --model-hierarchy    "
       << "print the hierarchy of the model" << endl;
  cerr << "  -o | --body-origins       "
       << "print the origins of all bodies that have names" << endl;
  cerr << "  -c | --center_of_mass     "
       << "print center of mass for bodies and full model" << endl;
  cerr << "  -h | --help               print this help" << endl;
  exit (1);
}

int main (int argc, char *argv[]) {
  if (argc < 2) {
    cerr << "Error: not enough arguments!" << endl;
    usage(argv[0]);
  }

  bool verbose = false;
  bool dof_overview = false;
  bool model_hierarchy = false;
  bool body_origins = false;
  bool center_of_mass = false;

  string filename = argv[1];

  for (int i = 1; i < argc; i++) {
    if (string(argv[i]) == "-v" || string (argv[i]) == "--verbose")
      verbose = true;
    else if (string(argv[i]) == "-d" || string (argv[i]) == "--dof-overview")
      dof_overview = true;
    else if (string(argv[i]) == "-m" || string (argv[i]) == "--model-hierarchy")
      model_hierarchy = true;
    else if (string(argv[i]) == "-f" || string (argv[i]) == "--floatbase")
      floatbase = true;
    else if (string(argv[i]) == "-o" || string (argv[i]) == "--body-origins")
      body_origins = true;
    else if (string(argv[i]) == "-c" || string (argv[i]) == "--center-of-mass")
      center_of_mass = true;
    else if (string(argv[i]) == "-h" || string (argv[i]) == "--help")
      usage(argv[0]);
    else
      filename = argv[i];
  }

  RigidBodyDynamics::Model model;

  if (!RigidBodyDynamics::Addons::URDFReadFromFile(filename.c_str(), &model,
                                                   floatbase, verbose)) {
    cerr << "Loading of urdf model failed!" << endl;
    return -1;
  }

  cout << "Model loading successful!" << endl;

  if (dof_overview) {
    cout << "Degree of freedom overview:" << endl;
    cout << RigidBodyDynamics::Utils::GetModelDOFOverview(model);
  }

  if (model_hierarchy) {
    cout << "Model Hierarchy:" << endl;
    cout << RigidBodyDynamics::Utils::GetModelHierarchy (model);
  }

  if (body_origins) {
    cout << "Body Origins:" << endl;
    cout << RigidBodyDynamics::Utils::GetNamedBodyOriginsOverview(model);
  }

  if (center_of_mass) {
    VectorNd q_zero (VectorNd::Zero (model.q_size));
    VectorNd qdot_zero (VectorNd::Zero (model.qdot_size));
    RigidBodyDynamics::UpdateKinematics (model, q_zero, qdot_zero, qdot_zero);

    for (unsigned int i = 1; i < model.mBodies.size(); i++) {
      if (model.mBodies[i].mIsVirtual)
        continue;

      SpatialRigidBodyInertia rbi_base = model.X_base[i].apply(model.I[i]);
      Vector3d body_com = rbi_base.h / rbi_base.m;
      cout << setw(12) << model.GetBodyName (i) << ": " << setw(10)
           <<  body_com.transpose() << endl;
    }

    Vector3d model_com;
    double mass;
    RigidBodyDynamics::Utils::CalcCenterOfMass (model, q_zero, qdot_zero,
                                                NULL, mass, model_com);
    cout << setw(14) << "Model COM: " << setw(10)
         <<  model_com.transpose() << endl;
    cout << setw(14) << "Model mass: " << mass << endl;
  }

  return 0;
}
