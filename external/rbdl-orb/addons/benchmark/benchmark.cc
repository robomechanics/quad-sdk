#include <iostream>

#include <algorithm>
#include <string>
#include <vector>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <fstream>

#include "rbdl/rbdl.h"
#include "model_generator.h"
#include "Human36Model.h"
#include "SampleData.h"
#include "Timer.h"

#ifdef RBDL_BUILD_ADDON_LUAMODEL
#include "../addons/luamodel/luamodel.h"
bool have_luamodel = true;
#else
bool have_luamodel = false;
#endif

#ifdef RBDL_BUILD_ADDON_URDFREADER
#include "../addons/urdfreader/urdfreader.h"
bool have_urdfreader = true;
bool urdf_floating_base = false;
#else
bool have_urdfreader = false;
#endif

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int benchmark_sample_count = 1000;
int benchmark_model_max_depth = 5;

bool benchmark_run_fd_aba = true;
bool benchmark_run_fd_lagrangian = true;
bool benchmark_run_id_rnea = true;
bool benchmark_run_crba = true;
bool benchmark_run_nle = true;
bool benchmark_run_calc_minv_times_tau = true;
bool benchmark_run_contacts = true;
bool benchmark_run_ik = true;

bool json_output = false;

string model_name;

enum ContactsMethod {
    ConstraintsMethodDirect = 0,
    ConstraintsMethodRangeSpaceSparse,
    ConstraintsMethodNullSpace,
    ConstraintsMethodKokkevis
};

struct BenchmarkRun {
    string model_name;
    int model_dof;
    string benchmark;
    int sample_count;

    double duration;
    double avg;
    double min;
    double max;
};

vector<BenchmarkRun> benchmark_runs;

void report_section (const char* section_name) {
  if (!json_output) {
    cout << "= " << section_name << " =" << endl;
  }
}

void register_run(const Model &model, const SampleData &data, const char *run_name) {
  BenchmarkRun run;
  run.benchmark = run_name;
  run.model_name = model_name;
  run.model_dof = model.dof_count;
  run.sample_count = data.count;

  run.duration = data.durations.sum();
  run.avg = data.durations.mean();
  run.min = data.durations.minCoeff();
  run.max = data.durations.maxCoeff();

  benchmark_runs.push_back(run);
}

void report_run(const Model &model, const SampleData &data,
        const char *run_name) {
  register_run(model, data, run_name);

  if (!json_output) {
    cout << "#DOF: " << setw(3) << model.dof_count
         << " #samples: " << data.count
         << " duration = " << setw(10) << data.durations.sum() << "(s)"
         << " (~" << setw(10) << data.durations.mean() << "(s) per call)" << endl;
  }
}

void report_constraints_run(const Model &model, const SampleData &data,
        const char *run_name) {
  register_run(model, data, run_name);

  if (!json_output) {
    cout << model_name << ": "
         << " duration = " << setw(10) << data.durations.sum() << "(s)"
         << " (~" << setw(10) << data.durations.mean() << "(s) per call)" << endl;
  }
}

/** Parses /proc/cpuinfo for the CPU model name. */
string get_cpu_model_name () {
  ifstream proc_cpu ("/proc/cpuinfo", ios_base::in);
  if (!proc_cpu) {
    cerr << "Cannot determine cpu model: could not open /proc/cpuinfo for reading." << endl;
    abort();
  }
  ostringstream content;
  content << proc_cpu.rdbuf();
  proc_cpu.close();

  string content_str = content.str();
  std::size_t model_name_pos = content_str.find("model name");
  if (model_name_pos != string::npos) {
    std::size_t start = content_str.find(':', model_name_pos + strlen("model name")) + 2;
    std::size_t end = content_str.find('\n', model_name_pos + strlen("model name"));

    return content_str.substr(start, end - start);
  }
  return "unknown";
}

string get_utc_time_string () {
  time_t  current_time;
  struct tm* timeinfo;
  time(&current_time);
  timeinfo = gmtime(&current_time);
  char time_buf[80];
  std::size_t time_len = strftime(&time_buf[0], 80, "%a %b %d %T %Y", timeinfo);
  return time_buf;
}

double run_forward_dynamics_ABA_benchmark (Model *model, int sample_count) {
  SampleData sample_data;
  sample_data.fillRandom(model->dof_count, sample_count);

  TimerInfo tinfo;

  for (int i = 0; i < sample_count; i++) {
    timer_start (&tinfo);
    ForwardDynamics (*model,
        sample_data.q[i],
        sample_data.qdot[i],
        sample_data.tau[i],
        sample_data.qddot[i]);
    sample_data.durations[i] = timer_stop (&tinfo);
  }

  report_run(*model, sample_data, "ForwardDynamics");

  return sample_data.durations.sum();
}

double run_forward_dynamics_lagrangian_benchmark (Model *model, int sample_count) {
  SampleData sample_data;
  sample_data.fillRandom(model->dof_count, sample_count);

  TimerInfo tinfo;

  MatrixNd H (MatrixNd::Zero(model->dof_count, model->dof_count));
  VectorNd C (VectorNd::Zero(model->dof_count));

  for (int i = 0; i < sample_count; i++) {
    timer_start (&tinfo);
    ForwardDynamicsLagrangian (*model,
        sample_data.q[i],
        sample_data.qdot[i],
        sample_data.tau[i],
        sample_data.qddot[i],
        Math::LinearSolverPartialPivLU,
        NULL,
        &H,
        &C
        );
    sample_data.durations[i] = timer_stop (&tinfo);
  }

  report_run(*model, sample_data, "ForwardDynamicsLagrangian_PivLU");

  return sample_data.durations.sum();
}

double run_inverse_dynamics_RNEA_benchmark (Model *model, int sample_count) {
  SampleData sample_data;
  sample_data.fillRandom(model->dof_count, sample_count);

  TimerInfo tinfo;
  timer_start (&tinfo);

  for (int i = 0; i < sample_count; i++) {
    timer_start (&tinfo);
    InverseDynamics (*model,
        sample_data.q[i],
        sample_data.qdot[i],
        sample_data.qddot[i],
        sample_data.tau[i]
        );
    sample_data.durations[i] = timer_stop (&tinfo);
  }

  report_run(*model, sample_data, "InverseDynamics");

  return sample_data.durations.sum();
}

double run_CRBA_benchmark (Model *model, int sample_count) {
  SampleData sample_data;
  sample_data.fillRandom(model->dof_count, sample_count);

  Math::MatrixNd H = Math::MatrixNd::Zero(model->dof_count, model->dof_count);
  Math::MatrixNd identity = Math::MatrixNd::Identity(model->dof_count, model->dof_count);
  Math::MatrixNd Hinv = Math::MatrixNd::Zero(model->dof_count, model->dof_count);

  TimerInfo tinfo;

  for (int i = 0; i < sample_count; i++) {
    timer_start (&tinfo);
    CompositeRigidBodyAlgorithm (*model, sample_data.q[i], H, true);
    sample_data.durations[i] = timer_stop (&tinfo);
  }

  report_run(*model, sample_data, "CompositeRigidBodyAlgorithm");

  return sample_data.durations.sum();
}

double run_nle_benchmark (Model *model, int sample_count) {
  SampleData sample_data;
  sample_data.fillRandom(model->dof_count, sample_count);

  TimerInfo tinfo;

  for (int i = 0; i < sample_count; i++) {
    timer_start (&tinfo);
    NonlinearEffects (*model,
        sample_data.q[i],
        sample_data.qdot[i],
        sample_data.tau[i]
        );
    sample_data.durations[i] = timer_stop (&tinfo);
  }

  report_run(*model, sample_data, "NonlinearEffects");

  return sample_data.durations.sum();
}

double run_calc_minv_times_tau_benchmark (Model *model, int sample_count) {
  SampleData sample_data;
  sample_data.fillRandom(model->dof_count, sample_count);

  CalcMInvTimesTau (*model, sample_data.q[0], sample_data.tau[0], sample_data.qddot[0]);

  TimerInfo tinfo;

  for (int i = 0; i < sample_count; i++) {
    timer_start (&tinfo);
    CalcMInvTimesTau (*model, sample_data.q[i], sample_data.tau[i], sample_data.qddot[i]);
    sample_data.durations[i] = timer_stop (&tinfo);
  }

  report_run(*model, sample_data, "NonlinearEffects");

  return sample_data.durations.sum();
}

double run_inverse_dynamics_constraints_benchmark (Model *model, ConstraintSet *constraint_set, std::vector<bool> &dofActuated, int sample_count) {
  SampleData sample_data;
  sample_data.fillRandom(model->dof_count, sample_count);
  VectorNd qddot = VectorNd::Zero(model->dof_count);
  TimerInfo tinfo;
  timer_start (&tinfo);

  constraint_set->SetActuationMap(*model, dofActuated);

  for (int i = 0; i < sample_count; i++) {
    InverseDynamicsConstraintsRelaxed (*model, sample_data.q[i], sample_data.qdot[i], sample_data.qddot[i], *constraint_set, qddot, sample_data.tau[i]); 
  }

  double duration = timer_stop (&tinfo);

  return duration;
}

double run_contacts_lagrangian_benchmark (Model *model, ConstraintSet *constraint_set, int sample_count) {
  SampleData sample_data;
  sample_data.fillRandom(model->dof_count, sample_count);

  TimerInfo tinfo;

  for (int i = 0; i < sample_count; i++) {
    timer_start (&tinfo);
    ForwardDynamicsConstraintsDirect (*model, sample_data.q[i], sample_data.qdot[i], sample_data.tau[i], *constraint_set, sample_data.qddot[i]);
    sample_data.durations[i] = timer_stop (&tinfo);
  }

  report_constraints_run(*model, sample_data, "ForwardDynamicsConstraintsDirect");

  return sample_data.durations.sum();
}

double run_contacts_lagrangian_sparse_benchmark (Model *model, ConstraintSet *constraint_set, int sample_count) {
  SampleData sample_data;
  sample_data.fillRandom(model->dof_count, sample_count);

  TimerInfo tinfo;

  for (int i = 0; i < sample_count; i++) {
    timer_start (&tinfo);
    ForwardDynamicsConstraintsRangeSpaceSparse (*model, sample_data.q[i], sample_data.qdot[i], sample_data.tau[i], *constraint_set, sample_data.qddot[i]);
    sample_data.durations[i] = timer_stop (&tinfo);
  }

  report_constraints_run(*model, sample_data, "ForwardDynamicsConstraintsRangeSpaceSparse");

  return sample_data.durations.sum();
}

double run_contacts_null_space (Model *model, ConstraintSet *constraint_set, int sample_count) {
  SampleData sample_data;
  sample_data.fillRandom(model->dof_count, sample_count);

  TimerInfo tinfo;

  for (int i = 0; i < sample_count; i++) {
    timer_start (&tinfo);
    ForwardDynamicsConstraintsNullSpace (*model, sample_data.q[i], sample_data.qdot[i], sample_data.tau[i], *constraint_set, sample_data.qddot[i]);
    sample_data.durations[i] = timer_stop (&tinfo);
  }

  report_constraints_run(*model, sample_data, "ForwardDynamicsConstraintsNullSpace");

  return sample_data.durations.sum();
}

double run_contacts_kokkevis_benchmark (Model *model, ConstraintSet *constraint_set, int sample_count) {
  SampleData sample_data;
  sample_data.fillRandom(model->dof_count, sample_count);

  TimerInfo tinfo;

  for (int i = 0; i < sample_count; i++) {
    timer_start (&tinfo);
    ForwardDynamicsContactsKokkevis(*model, sample_data.q[i], sample_data.qdot[i], sample_data.tau[i], *constraint_set, sample_data.qddot[i]);
    sample_data.durations[i] = timer_stop (&tinfo);
  }

  report_constraints_run(*model, sample_data, "ForwardDynamicsContactsKokkevis");

  return sample_data.durations.sum();
}

void contacts_benchmark (int sample_count, ContactsMethod contacts_method) {
  // initialize the human model
  Model *model = new Model();
  generate_human36model(model);

  // initialize the constraint sets
  unsigned int foot_r = model->GetBodyId ("foot_r");
  unsigned int foot_l = model->GetBodyId ("foot_l");
  unsigned int hand_r = model->GetBodyId ("hand_r");
  unsigned int hand_l = model->GetBodyId ("hand_l");

  std::vector< bool > actuatedDof;
  actuatedDof.resize(model->qdot_size);

  for(unsigned int i=0; i<actuatedDof.size();++i){
    if(i < 6){
      actuatedDof[i] = false;
    }else{
      actuatedDof[i] = true;
    }
  }

  ConstraintSet one_body_one_constraint;
  ConstraintSet two_bodies_one_constraint;
  ConstraintSet four_bodies_one_constraint;

  ConstraintSet one_body_four_constraints;
  ConstraintSet two_bodies_four_constraints;
  ConstraintSet four_bodies_four_constraints;

  LinearSolver linear_solver = LinearSolverPartialPivLU;

  one_body_one_constraint.linear_solver = linear_solver;
  two_bodies_one_constraint.linear_solver = linear_solver;
  four_bodies_one_constraint.linear_solver = linear_solver;
  one_body_four_constraints.linear_solver = linear_solver;
  two_bodies_four_constraints.linear_solver = linear_solver;
  four_bodies_four_constraints.linear_solver = linear_solver;

  // one_body_one
  one_body_one_constraint.AddContactConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
  one_body_one_constraint.Bind (*model);

  // two_bodies_one
  two_bodies_one_constraint.AddContactConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
  two_bodies_one_constraint.AddContactConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
  two_bodies_one_constraint.Bind (*model);

  // four_bodies_one
  four_bodies_one_constraint.AddContactConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
  four_bodies_one_constraint.AddContactConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
  four_bodies_one_constraint.AddContactConstraint (hand_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
  four_bodies_one_constraint.AddContactConstraint (hand_l, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
  four_bodies_one_constraint.Bind (*model);

  // one_body_four
  one_body_four_constraints.AddContactConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
  one_body_four_constraints.AddContactConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
  one_body_four_constraints.AddContactConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
  one_body_four_constraints.AddContactConstraint (foot_r, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));
  one_body_four_constraints.Bind (*model);	

  // two_bodies_four
  two_bodies_four_constraints.AddContactConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
  two_bodies_four_constraints.AddContactConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
  two_bodies_four_constraints.AddContactConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
  two_bodies_four_constraints.AddContactConstraint (foot_r, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));

  two_bodies_four_constraints.AddContactConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
  two_bodies_four_constraints.AddContactConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
  two_bodies_four_constraints.AddContactConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
  two_bodies_four_constraints.AddContactConstraint (foot_l, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));

  two_bodies_four_constraints.Bind (*model);

  // four_bodies_four
  four_bodies_four_constraints.AddContactConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
  four_bodies_four_constraints.AddContactConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
  four_bodies_four_constraints.AddContactConstraint (foot_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
  four_bodies_four_constraints.AddContactConstraint (foot_r, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));

  four_bodies_four_constraints.AddContactConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
  four_bodies_four_constraints.AddContactConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
  four_bodies_four_constraints.AddContactConstraint (foot_l, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
  four_bodies_four_constraints.AddContactConstraint (foot_l, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));

  four_bodies_four_constraints.AddContactConstraint (hand_r, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
  four_bodies_four_constraints.AddContactConstraint (hand_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
  four_bodies_four_constraints.AddContactConstraint (hand_r, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
  four_bodies_four_constraints.AddContactConstraint (hand_r, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));

  four_bodies_four_constraints.AddContactConstraint (hand_l, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
  four_bodies_four_constraints.AddContactConstraint (hand_l, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
  four_bodies_four_constraints.AddContactConstraint (hand_l, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
  four_bodies_four_constraints.AddContactConstraint (hand_l, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));

  four_bodies_four_constraints.Bind (*model);

  model_name = "Human36";
  if (!json_output) {
    cout << "= #DOF: " << setw(3) << model->dof_count << endl;
    cout << "= #samples: " << sample_count << endl;
    cout << "= No constraints (Articulated Body Algorithm):" << endl;
    run_forward_dynamics_ABA_benchmark(model, sample_count);
  }

  // one body one
  model_name = "Human36_1Bodies1Constraints";
  if (contacts_method == ConstraintsMethodDirect) {
    run_contacts_lagrangian_benchmark (model, &one_body_one_constraint, sample_count);
  } else if (contacts_method == ConstraintsMethodRangeSpaceSparse) {
    run_contacts_lagrangian_sparse_benchmark (model, &one_body_one_constraint, sample_count);
  } else if (contacts_method == ConstraintsMethodNullSpace) {
    run_contacts_null_space (model, &one_body_one_constraint, sample_count);
  } else {
    run_contacts_kokkevis_benchmark (model, &one_body_one_constraint, sample_count);
  }

  // two_bodies_one
  model_name = "Human36_2Bodies1Constraints";
  if (contacts_method == ConstraintsMethodDirect) {
    run_contacts_lagrangian_benchmark (model, &two_bodies_one_constraint, sample_count);
  } else if (contacts_method == ConstraintsMethodRangeSpaceSparse) {
    run_contacts_lagrangian_sparse_benchmark (model, &two_bodies_one_constraint, sample_count);
  } else if (contacts_method == ConstraintsMethodNullSpace) {
    run_contacts_null_space (model, &two_bodies_one_constraint, sample_count);
  } else {
    run_contacts_kokkevis_benchmark (model, &two_bodies_one_constraint, sample_count);
  }

  // four_bodies_one
  model_name = "Human36_4Bodies1Constraints";
  if (contacts_method == ConstraintsMethodDirect) {
    run_contacts_lagrangian_benchmark (model, &four_bodies_one_constraint, sample_count);
  } else if (contacts_method == ConstraintsMethodRangeSpaceSparse) {
    run_contacts_lagrangian_sparse_benchmark (model, &four_bodies_one_constraint, sample_count);
  } else if (contacts_method == ConstraintsMethodNullSpace) {
    run_contacts_null_space (model, &four_bodies_one_constraint, sample_count);
  } else {
    run_contacts_kokkevis_benchmark (model, &four_bodies_one_constraint, sample_count);
  }

  // one_body_four
  model_name = "Human36_1Bodies4Constraints";
  if (contacts_method == ConstraintsMethodDirect) {
    run_contacts_lagrangian_benchmark (model, &one_body_four_constraints, sample_count);
  } else if (contacts_method == ConstraintsMethodRangeSpaceSparse) {
    run_contacts_lagrangian_sparse_benchmark (model, &one_body_four_constraints, sample_count);
  } else if (contacts_method == ConstraintsMethodNullSpace) {
    run_contacts_null_space (model, &one_body_four_constraints, sample_count);
  } else {
    run_contacts_kokkevis_benchmark (model, &one_body_four_constraints, sample_count);
  }

  // two_bodies_four
  model_name = "Human36_2Bodies4Constraints";
  if (contacts_method == ConstraintsMethodDirect) {
    run_contacts_lagrangian_benchmark (model, &two_bodies_four_constraints, sample_count);
  } else if (contacts_method == ConstraintsMethodRangeSpaceSparse) {
    run_contacts_lagrangian_sparse_benchmark (model, &two_bodies_four_constraints, sample_count);
  } else if (contacts_method == ConstraintsMethodNullSpace) {
    run_contacts_null_space (model, &two_bodies_four_constraints, sample_count);
  } else {
    run_contacts_kokkevis_benchmark (model, &two_bodies_four_constraints, sample_count);
  }

  // four_bodies_four
  model_name = "Human36_4Bodies4Constraints";
  if (contacts_method == ConstraintsMethodDirect) {
    run_contacts_lagrangian_benchmark (model, &four_bodies_four_constraints, sample_count);
  } else if (contacts_method == ConstraintsMethodRangeSpaceSparse) {
    run_contacts_lagrangian_sparse_benchmark (model, &four_bodies_four_constraints, sample_count);
  } else if (contacts_method == ConstraintsMethodNullSpace) {
    run_contacts_null_space (model, &four_bodies_four_constraints, sample_count);
  } else {
    run_contacts_kokkevis_benchmark (model, &four_bodies_four_constraints, sample_count);
  }

  delete model;
}

double run_single_inverse_kinematics_benchmark(Model *model, std::vector<InverseKinematicsConstraintSet> &CS, int sample_count){
  TimerInfo tinfo;
  timer_start (&tinfo);
  VectorNd qinit = VectorNd::Zero(model->dof_count);
  VectorNd qres = VectorNd::Zero(model->dof_count);
  VectorNd failures = VectorNd::Zero(model->dof_count);

  for (int i = 0; i < sample_count; i++) {
    if (!InverseKinematics(*model, qinit, CS[i], qres)){
      failures[i] = 1;
    }
  }
  double duration = timer_stop (&tinfo);
  std::cout << "Success Rate: " << (1-failures.mean())*100 << "%  for: ";
  return duration;
  
}

double run_all_inverse_kinematics_benchmark (int sample_count){
  
  //initialize the human model
  Model *model = new Model();
  generate_human36model(model);
  
  unsigned int foot_r = model->GetBodyId ("foot_r");
  unsigned int foot_l = model->GetBodyId ("foot_l");
  unsigned int hand_r = model->GetBodyId ("hand_r");
  unsigned int hand_l = model->GetBodyId ("hand_l");
  unsigned int head   = model->GetBodyId ("head");
  
  Vector3d foot_r_point (1., 0., 0.);
  Vector3d foot_l_point (-1., 0., 0.);
  Vector3d hand_r_point (0., 1., 0.);
  Vector3d hand_l_point (1., 0., 1.);
  Vector3d head_point (0.,0.,-1.);
  
  SampleData sample_data;
  sample_data.fillRandom(model->dof_count, sample_count);
  
  
  //create constraint sets
  std::vector<InverseKinematicsConstraintSet> cs_one_point;
  std::vector<InverseKinematicsConstraintSet> cs_two_point_one_orientation;
  std::vector<InverseKinematicsConstraintSet> cs_two_full_one_point;
  std::vector<InverseKinematicsConstraintSet> cs_two_full_two_point_one_orientation;
  std::vector<InverseKinematicsConstraintSet> cs_five_full;
  
  for (unsigned int i = 0; i < sample_count; i++){
    Vector3d foot_r_position = CalcBodyToBaseCoordinates (*model, sample_data.q[i], foot_r, foot_r_point);
    Vector3d foot_l_position = CalcBodyToBaseCoordinates (*model, sample_data.q[i], foot_l, foot_l_point);
    Vector3d hand_r_position = CalcBodyToBaseCoordinates (*model, sample_data.q[i], hand_r, hand_r_point);
    Vector3d hand_l_position = CalcBodyToBaseCoordinates (*model, sample_data.q[i], hand_l, hand_l_point);
    Vector3d head_position   = CalcBodyToBaseCoordinates (*model, sample_data.q[i], head  ,   head_point);

    Matrix3d foot_r_orientation = CalcBodyWorldOrientation (*model, sample_data.q[i], foot_r, false);
    Matrix3d foot_l_orientation = CalcBodyWorldOrientation (*model, sample_data.q[i], foot_l, false);
    Matrix3d hand_r_orientation = CalcBodyWorldOrientation (*model, sample_data.q[i], hand_r, false);
    Matrix3d hand_l_orientation = CalcBodyWorldOrientation (*model, sample_data.q[i], hand_l, false);
    Matrix3d head_orientation   = CalcBodyWorldOrientation (*model, sample_data.q[i], head  , false);

    //single point
    InverseKinematicsConstraintSet one_point;
    one_point.AddPointConstraint(foot_r, foot_r_point, foot_r_position);
    one_point.step_tol = 1e-12;
    cs_one_point.push_back(one_point);

    //two point and one orientation
    InverseKinematicsConstraintSet two_point_one_orientation;
    two_point_one_orientation.AddPointConstraint(foot_l,foot_l_point, foot_l_position);
    two_point_one_orientation.AddPointConstraint(foot_r, foot_r_point, foot_r_position);
    two_point_one_orientation.AddOrientationConstraint(head, head_orientation);
    two_point_one_orientation.step_tol = 1e-12;
    cs_two_point_one_orientation.push_back(two_point_one_orientation);

    //two full and one point
    InverseKinematicsConstraintSet two_full_one_point;
    two_full_one_point.AddFullConstraint(hand_r, hand_r_point, hand_r_position, hand_r_orientation);
    two_full_one_point.AddFullConstraint(hand_l, hand_l_point, hand_l_position, hand_l_orientation);
    two_full_one_point.AddPointConstraint(head, head_point, head_position);
    two_full_one_point.step_tol = 1e-12;
    cs_two_full_one_point.push_back(two_full_one_point);
    
    //two full, two points and one orienation
    InverseKinematicsConstraintSet two_full_two_point_one_orientation;
    two_full_two_point_one_orientation.AddPointConstraint(foot_r, foot_r_point, foot_r_position);
    two_full_two_point_one_orientation.AddPointConstraint(foot_l, foot_l_point, foot_l_position);
    two_full_two_point_one_orientation.AddFullConstraint(hand_r, hand_r_point, hand_r_position, hand_r_orientation);
    two_full_two_point_one_orientation.AddFullConstraint(hand_l, hand_l_point, hand_l_position, hand_l_orientation);
    two_full_two_point_one_orientation.AddOrientationConstraint(head, head_orientation);
    two_full_two_point_one_orientation.step_tol = 1e-12;
    cs_two_full_two_point_one_orientation.push_back(two_full_two_point_one_orientation);
    
    //five points 5 orientations
    InverseKinematicsConstraintSet five_full;
    five_full.AddFullConstraint(foot_r, foot_r_point, foot_r_position, foot_r_orientation);
    five_full.AddFullConstraint(foot_l, foot_l_point, foot_l_position, foot_l_orientation);
    five_full.AddFullConstraint(hand_r, hand_r_point, hand_r_position, hand_r_orientation);
    five_full.AddFullConstraint(hand_l, hand_l_point, hand_l_position, hand_l_orientation);
    five_full.AddFullConstraint(head, head_point, head_position, head_orientation);
    five_full.step_tol = 1e-12;
    cs_five_full.push_back(five_full);
  }
  
  cout << "= #DOF: " << setw(3) << model->dof_count << endl;
  cout << "= #samples: " << sample_count << endl;
  double duration;
  
  duration = run_single_inverse_kinematics_benchmark(model, cs_one_point, sample_count);
  cout << "Constraints: 1 Body:   1 Point                      : "
  << " duration = " << setw(10) << duration << "(s)"
  << " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;
  
  duration = run_single_inverse_kinematics_benchmark(model, cs_two_point_one_orientation, sample_count);
  cout << "Constraints: 3 Bodies: 2 Points 1 Orienation        : "
  << " duration = " << setw(10) << duration << "(s)"
  << " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;
  
  duration = run_single_inverse_kinematics_benchmark(model, cs_two_full_one_point, sample_count);
  cout << "Constraints: 3 Bodies: 2 Full 1 Point               : "
  << " duration = " << setw(10) << duration << "(s)"
  << " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;
  
  duration = run_single_inverse_kinematics_benchmark(model, cs_two_full_two_point_one_orientation, sample_count);  
  cout << "Constraints: 5 Bodies: 2 Full 2 Points 1 Orienation : "
  << " duration = " << setw(10) << duration << "(s)"
  << " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;
  
  duration = run_single_inverse_kinematics_benchmark(model, cs_five_full, sample_count);  
  cout << "Constraints: 5 Bodies: 5 Full                       : "
  << " duration = " << setw(10) << duration << "(s)"
  << " (~" << setw(10) << duration / sample_count << "(s) per call)" << endl;
  return duration;
}

void print_usage () {
#if defined (RBDL_BUILD_ADDON_LUAMODEL) || defined (RBDL_BUILD_ADDON_URDFREADER)
  cout << "Usage: benchmark [--count|-c <sample_count>] [--depth|-d <depth>] <model.lua>" << endl;
#else
  cout << "Usage: benchmark [--count|-c <sample_count>] [--depth|-d <depth>]" << endl;
#endif
  cout << "Simple benchmark tool for the Rigid Body Dynamics Library." << endl;
  cout << "  --count | -c <sample_count> : sets the number of sample states that should" << endl;
  cout << "                be calculated (default: 1000)" << endl;
  cout << "  --depth | -d <depth>        : sets maximum depth for the branched test model" << endl;
  cout << "                which is created increased from 1 to <depth> (default: 5)." << endl;
#if defined RBDL_BUILD_ADDON_URDFREADER
  cout << "  --floating-base | -f        : the specified URDF model is a floating base model." << endl;
#endif
  cout << "  --json                      : prints output in json format." << endl;
  cout << "  --no-fd                     : disables benchmarking of forward dynamics." << endl;
  cout << "  --no-fd-aba                 : disables benchmark for forwards dynamics using" << endl;
  cout << "                                the Articulated Body Algorithm" << endl;
  cout << "  --no-fd-lagrangian          : disables benchmark for forward dynamics via" << endl;
  cout << "                                solving the lagrangian equation." << endl;
  cout << "  --no-id-rnea                : disables benchmark for inverse dynamics using" << endl;
  cout << "                                the recursive newton euler algorithm." << endl;
  cout << "  --no-crba                   : disables benchmark for joint space inertia" << endl;
  cout << "                                matrix computation using the composite rigid" << endl;
  cout << "                                body algorithm." << endl;
  cout << "  --no-nle                    : disables benchmark for the nonlinear effects." << endl;
  cout << "  --no-calc-minv              : disables benchmark M^-1 * tau benchmark." << endl;
  cout << "  --only-contacts | -C        : only runs contact model benchmarks." << endl;
  cout << "  --only-ik                   : only runs inverse kinematics benchmarks." << endl;
  cout << "  --help | -h                 : prints this help." << endl;
}

void disable_all_benchmarks () {
  benchmark_run_fd_aba = false;
  benchmark_run_fd_lagrangian = false;
  benchmark_run_id_rnea = false;
  benchmark_run_crba = false;
  benchmark_run_nle = false;
  benchmark_run_calc_minv_times_tau = false;
  benchmark_run_contacts = false;
}

void parse_args (int argc, char* argv[]) {
  int argi = 1;

  while (argi < argc) {
    string arg = argv[argi];

    if (arg == "--help" || arg == "-h") {
      print_usage();
      exit (1);
    } else if (arg == "--count" || arg == "-c" ) {
      if (argi == argc - 1) {
        print_usage();

        cerr << "Error: missing number of samples!" << endl;
        exit (1);
      }

      argi++;
      stringstream count_stream (argv[argi]);

      count_stream >> benchmark_sample_count;
    } else if (arg == "--depth" || arg == "-d" ) {
      if (argi == argc - 1) {
        print_usage();

        cerr << "Error: missing number for model depth!" << endl;
        exit (1);
      }

      argi++;
      stringstream depth_stream (argv[argi]);

      depth_stream >> benchmark_model_max_depth;
#ifdef RBDL_BUILD_ADDON_URDFREADER
    } else if (arg == "--floating-base" || arg == "-f") {
      urdf_floating_base = true;
#endif
    } else if (arg == "--json") {
      json_output = true;
    } else if (arg == "--no-fd" ) {
      benchmark_run_fd_aba = false;
      benchmark_run_fd_lagrangian = false;
    } else if (arg == "--no-fd-aba" ) {
      benchmark_run_fd_aba = false;
    } else if (arg == "--no-fd-lagrangian" ) {
      benchmark_run_fd_lagrangian = false;
    } else if (arg == "--no-id-rnea" ) {
      benchmark_run_id_rnea = false;
    } else if (arg == "--no-crba" ) {
      benchmark_run_crba = false;
    } else if (arg == "--no-nle" ) {
      benchmark_run_nle = false;
    } else if (arg == "--no-calc-minv" ) {
      benchmark_run_calc_minv_times_tau = false;
    } else if (arg == "--only-contacts" || arg == "-C") {
      disable_all_benchmarks();
      benchmark_run_contacts = true;
    } else if (arg == "--only-ik") {
      disable_all_benchmarks();
      benchmark_run_ik = true;
#if defined (RBDL_BUILD_ADDON_LUAMODEL) || defined (RBDL_BUILD_ADDON_URDFREADER)
    } else if (model_name == "") {
      model_name = arg;
#endif
    } else {
      print_usage();
      cerr << "Invalid argument '" << arg << "'." << endl;
      exit(1);
    }
    argi++;
  }
}

int main (int argc, char *argv[]) {
  parse_args (argc, argv);

  Model *model = NULL;

  model = new Model();

  if (model_name != "") {
    if (model_name.substr (model_name.size() - 4, 4) == ".lua") {
#ifdef RBDL_BUILD_ADDON_LUAMODEL
      RigidBodyDynamics::Addons::LuaModelReadFromFile (model_name.c_str(), model);
#else
      cerr << "Could not load Lua model: LuaModel addon not enabled!" << endl;
      abort();
#endif
    }
    if (model_name.substr (model_name.size() - 5, 5) == ".urdf") {
#ifdef RBDL_BUILD_ADDON_URDFREADER
      RigidBodyDynamics::Addons::URDFReadFromFile(model_name.c_str(), model, urdf_floating_base);
#else
      cerr << "Could not load URDF model: urdfreader addon not enabled!" << endl;
      abort();
#endif
    }

    if (benchmark_run_fd_aba) {
      report_section("Forward Dynamics: ABA");
      run_forward_dynamics_ABA_benchmark (model, benchmark_sample_count);
    }

    if (benchmark_run_fd_lagrangian) {
      report_section("Forward Dynamics: Lagrangian (Piv. LU decomposition)");
      run_forward_dynamics_lagrangian_benchmark (model, benchmark_sample_count);
    }

    if (benchmark_run_id_rnea) {
      report_section("Inverse Dynamics: RNEA");
      run_inverse_dynamics_RNEA_benchmark (model, benchmark_sample_count);
    }

    if (benchmark_run_crba) {
      report_section("Joint Space Inertia Matrix: CRBA");
      run_CRBA_benchmark (model, benchmark_sample_count);
    }

    if (benchmark_run_nle) {
      report_section("Nonlinear Effects");
      run_nle_benchmark (model, benchmark_sample_count);
    }

    delete model;

    return 0;
  }

  if (!json_output) {
    rbdl_print_version();
    cout << endl;
  }

  if (benchmark_run_fd_aba) {
    report_section("Forward Dynamics: ABA");
    for (int depth = 1; depth <= benchmark_model_max_depth; depth++) {
      ostringstream model_name_stream;
      model_name_stream << "planar_model_depth_" << depth;
      model_name = model_name_stream.str();

      model = new Model();
      model->gravity = Vector3d (0., -9.81, 0.);

      generate_planar_tree (model, depth);

      run_forward_dynamics_ABA_benchmark (model, benchmark_sample_count);

      delete model;
    }
  }

  if (benchmark_run_fd_lagrangian) {
    report_section("Forward Dynamics: Lagrangian (Piv. LU decomposition)");
    for (int depth = 1; depth <= benchmark_model_max_depth; depth++) {
      model = new Model();
      model->gravity = Vector3d (0., -9.81, 0.);

      generate_planar_tree (model, depth);

      run_forward_dynamics_lagrangian_benchmark (model, benchmark_sample_count);

      delete model;
    }
  }

  if (benchmark_run_id_rnea) {
    report_section("Inverse Dynamics: RNEA");
    for (int depth = 1; depth <= benchmark_model_max_depth; depth++) {
      model = new Model();
      model->gravity = Vector3d (0., -9.81, 0.);

      generate_planar_tree (model, depth);

      run_inverse_dynamics_RNEA_benchmark (model, benchmark_sample_count);

      delete model;
    }
  }

  if (benchmark_run_crba) {
    report_section("Joint Space Inertia Matrix: CRBA");
    for (int depth = 1; depth <= benchmark_model_max_depth; depth++) {
      model = new Model();
      model->gravity = Vector3d (0., -9.81, 0.);

      generate_planar_tree (model, depth);

      run_CRBA_benchmark (model, benchmark_sample_count);

      delete model;
    }
  }

  if (benchmark_run_nle) {
    report_section("Nonlinear Effects");
    for (int depth = 1; depth <= benchmark_model_max_depth; depth++) {
      model = new Model();
      model->gravity = Vector3d (0., -9.81, 0.);

      generate_planar_tree (model, depth);

      run_nle_benchmark (model, benchmark_sample_count);

      delete model;
    }
  }

  if (benchmark_run_calc_minv_times_tau) {
    report_section("CalcMInvTimesTau");
    for (int depth = 1; depth <= benchmark_model_max_depth; depth++) {
      model = new Model();
      model->gravity = Vector3d (0., -9.81, 0.);

      generate_planar_tree (model, depth);

      run_calc_minv_times_tau_benchmark (model, benchmark_sample_count);

      delete model;
    }
  }

  if (benchmark_run_contacts) {
    report_section("Contacts: ForwardDynamicsConstraintsDirect");
    contacts_benchmark (benchmark_sample_count, ConstraintsMethodDirect);

    report_section("Contacts: ForwardDynamicsConstraintsRangeSpaceSparse");
    contacts_benchmark (benchmark_sample_count, ConstraintsMethodRangeSpaceSparse);

    report_section("Contacts: ForwardDynamicsConstraintsNullSpace");
    contacts_benchmark (benchmark_sample_count, ConstraintsMethodNullSpace);

    report_section("Contacts: ForwardDynamicsContactsKokkevis");
    contacts_benchmark (benchmark_sample_count, ConstraintsMethodKokkevis);
  }

  if (benchmark_run_ik) {
    report_section("Inverse Kinematics");
    run_all_inverse_kinematics_benchmark(benchmark_sample_count);
  }

  if (json_output) {
    cout.precision(15);
    cout << "{" << endl;

    cout << "    \"rbdl_info\" : {" << endl;
    int compile_version = rbdl_get_api_version();
    int compile_major = (compile_version & 0xff0000) >> 16;
    int compile_minor = (compile_version & 0x00ff00) >> 8;
    int compile_patch = (compile_version & 0x0000ff);

    std::ostringstream compile_version_string("");
    compile_version_string << compile_major << "." << compile_minor << "." << compile_patch;

    cout << "        \"version_str\" : \"" << compile_version_string.str() << "\"," << endl;
    cout << "        \"major\" : " << compile_major << "," << endl;
    cout << "        \"minor\" : " << compile_minor << "," << endl;
    cout << "        \"patch\" : " << compile_patch << "," << endl;
    cout << "        \"build_type\" : \"" << RBDL_BUILD_TYPE << "\"," << endl;
    cout << "        \"commit\" : \"" << RBDL_BUILD_COMMIT << "\"," << endl;
    cout << "        \"branch\" : \"" << RBDL_BUILD_BRANCH << "\"," << endl;
    cout << "        \"compiler_id\" : \"" << RBDL_BUILD_COMPILER_ID << "\"," << endl;
    cout << "        \"compiler_version\" : \"" << RBDL_BUILD_COMPILER_VERSION << "\"" << endl;

    cout << "    }," << endl;

    cout << "    \"host_info\" : {" << endl;
    cout << "        \"cpu_model_name\" : \"" << get_cpu_model_name() << "\"," << endl;
    cout << "        \"time_utc\" : " << "\"" << get_utc_time_string() << "\"" << endl;
    cout << "    }," << endl;

    cout << "    \"runs\" : ";
    cout << "[" << endl;

    for (int i; i < benchmark_runs.size(); i++) {
      const BenchmarkRun& run = benchmark_runs[i];

      const char* indent = "            ";

      cout << "        " << "{" << endl;
      cout << indent << "\"model\" : \"" << run.model_name << "\"," << endl;
      cout << indent << "\"dof\" : " << run.model_dof << "," << endl;
      cout << indent << "\"benchmark\" : \"" << run.benchmark << "\"," << endl;
      cout << indent << "\"duration\" : " << run.duration << "," << endl;
      cout << indent << "\"sample_count\" : " << run.sample_count << "," << endl;
      cout << indent << "\"avg\" : " << run.avg << "," << endl;
      cout << indent << "\"min\" : " << run.min << "," << endl;
      cout << indent << "\"max\" : " << run.max << endl;
      cout << "        " << "}";

      if (i != benchmark_runs.size() - 1) {
        cout << ",";
      }
      cout << endl;
    }

    cout << "    ]" << endl;

    cout << "}" << endl;
  }

  return 0;
}
