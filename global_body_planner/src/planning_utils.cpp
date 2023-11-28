#include "global_body_planner/planning_utils.h"
<<<<<<< HEAD
=======
#include "quad_utils/matplotlibcpp.h"

>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
namespace planning_utils {

State fullStateToState(const FullState &full_state) {
  State state;
  state.pos = full_state.pos;
  state.vel = full_state.vel;

  return state;
}

FullState stateToFullState(const State &state, double roll, double pitch,
                           double yaw, double roll_rate, double pitch_rate,
                           double yaw_rate) {
  FullState full_state;

  full_state.pos = state.pos;
  full_state.vel = state.vel;
  full_state.ang << roll, pitch, yaw;

  // Convert euler rates to body angular velocity
  Eigen::Vector3d d_rpy, ang_vel;
  d_rpy << roll_rate, pitch_rate, yaw_rate;

  Eigen::Matrix3d d_rpy_to_ang_vel;
  d_rpy_to_ang_vel << cos(pitch) * cos(yaw), -sin(yaw), 0,
      cos(pitch) * sin(yaw), cos(yaw), 0, -sin(pitch), 0, 1;

  ang_vel = d_rpy_to_ang_vel * d_rpy;
  full_state.ang_vel = ang_vel;

  return full_state;
}

void eigenToFullState(const Eigen::VectorXd &s_eig, FullState &s) {
  if (s_eig.size() != 12) {
    ROS_ERROR("Eigen::VectorXd is incorrect size");
  }
  s.pos = s_eig.segment(0, 3);
  s.ang = s_eig.segment(3, 3);
  s.vel = s_eig.segment(6, 3);
  s.ang_vel = s_eig.segment(9, 3);
}

Eigen::VectorXd fullStateToEigen(const FullState &s) {
  Eigen::VectorXd s_eig(12);
  s_eig.segment(0, 3) = s.pos;
  s_eig.segment(3, 3) = s.ang;
  s_eig.segment(6, 3) = s.vel;
  s_eig.segment(9, 3) = s.ang_vel;
  return s_eig;
}

void vectorToFullState(const std::vector<double> &v, FullState &s) {
  if (v.size() != 12) {
    ROS_ERROR("std::vector<double> is incorrect size");
  }
  s.pos[0] = v[0];
  s.pos[1] = v[1];
  s.pos[2] = v[2];
  s.ang[0] = v[3];
  s.ang[1] = v[4];
  s.ang[2] = v[5];
  s.vel[0] = v[6];
  s.vel[1] = v[7];
  s.vel[2] = v[8];
  s.ang_vel[0] = v[9];
  s.ang_vel[1] = v[10];
  s.ang_vel[2] = v[11];
}

void flipDirection(State &state) { state.vel = -state.vel; }

void flipDirection(Action &action) {
  // Reverse ground reaction forces (works if GRF is symmetric)
  GRF temp_grf = action.grf_0;
  action.grf_0 = action.grf_f;
  action.grf_f = temp_grf;

  // Reverse initial and final vertical velocities
  double temp_dz = action.dz_0;
  action.dz_0 = -action.dz_f;
  action.dz_f = -temp_dz;

  // Reverse stance times for landing and leaping if this is an actual leap
  if (action.t_f > 0) {
    double temp_t_s = action.t_s_leap;
    action.t_s_leap = action.t_s_land;
    action.t_s_land = temp_t_s;
  }
}

void printState(const State &s) {
  std::cout << "STATE: pos = " << s.pos.transpose()
            << ", vel = " << s.vel.transpose() << std::endl;
}

void printFullState(const FullState &s) {
  std::cout << "STATE pos = " << s.pos.transpose()
            << ", vel = " << s.vel.transpose() << std::endl;
  std::cout << "ang = " << s.ang.transpose()
            << ", ang_vel = " << s.ang_vel.transpose() << std::endl;
}

void printAction(const Action &a) {
  std::cout << "ACTION: grf_0 = " << a.grf_0.transpose()
            << ", grf_f = " << a.grf_f.transpose() << std::endl;
  std::cout << "Phase durations = " << a.t_s_leap << ", " << a.t_f << ", "
            << a.t_s_land << std::endl;
  std::cout << "Terminal velocities: " << a.dz_0 << ", " << a.dz_f << std::endl;
}

void printStateNewline(const State &s) {
  printState(s);
  std::cout << std::endl;
}

void printActionNewline(const Action &a) {
  printAction(a);
  std::cout << std::endl;
}

void printStateSequence(const std::vector<State> &state_sequence) {
  for (State s : state_sequence) printState(s);
}

void printInterpStateSequence(const std::vector<State> &state_sequence,
                              std::vector<double> &interp_t) {
  for (int i = 0; i < state_sequence.size(); i++) {
    std::cout << interp_t[i] << "\t";
    printStateNewline(state_sequence[i]);
  }
}

void printActionSequence(const std::vector<Action> &action_sequence) {
  for (Action a : action_sequence) printActionNewline(a);
}

double poseDistance(const State &s1, const State &s2) {
  return (s1.pos - s2.pos).norm();
}

double poseDistance(const FullState &s1, const FullState &s2) {
  return (s1.pos - s2.pos).norm();
}

double poseDistance(const std::vector<double> &v1,
                    const std::vector<double> &v2) {
  double sum = 0;

  if (v1.size() < 3 || v2.size() < 3) {
    throw std::runtime_error("Not enough data in vectors to compute distance");
  }

  for (int i = 0; i < 3; i++) {
    sum = sum + (v2[i] - v1[i]) * (v2[i] - v1[i]);
  }

  double dist = sqrt(sum);
  return dist;
}

double stateDistance(const State &q1, const State &q2) {
  double w_pos = 1;
  double w_vel = 1;

  return ((q2.pos - q1.pos).norm() * w_pos + (q2.vel - q1.vel).norm() * w_vel);
}

void addFullStates(const FullState &start_state,
                   std::vector<State> interp_reduced_plan, double dt,
                   std::vector<FullState> &interp_full_plan,
                   const PlannerConfig &planner_config) {
  int num_states = interp_reduced_plan.size();

  // Set roll and roll rate to zero
  double roll = 0;
  double roll_rate = 0;

  // Declare variables for yaw
  std::vector<double> z(num_states);
  std::vector<double> filtered_z(num_states);
  std::vector<double> z_rate(num_states);
  std::vector<double> filtered_z_rate(num_states);
  std::vector<double> pitch(num_states);
  std::vector<double> pitch_rate(num_states);
  std::vector<double> filtered_pitch_rate(num_states);
  std::vector<double> wrapped_yaw(num_states);
  std::vector<double> unwrapped_yaw(num_states);
  std::vector<double> filtered_yaw(num_states);
  std::vector<double> yaw_rate(num_states);
  std::vector<double> filtered_yaw_rate(num_states);

  // Enforce that yaw and pitch match current state, let the filter smooth
  // things out
  z[0] = start_state.pos[2];
  pitch[0] = start_state.ang[1];
  wrapped_yaw[0] = start_state.ang[2];

  // Compute wrapped yaw first to align with heading
  for (int i = 1; i < num_states; i++) {
    State body_state = interp_reduced_plan[i];
    wrapped_yaw[i] = atan2(body_state.vel[1], body_state.vel[0]);
  }

  // Unwrap yaw for filtering
  unwrapped_yaw = math_utils::unwrap(wrapped_yaw);

  // Compute pitch and height to align with the terrain, add first order filter
  // on init state
  double gamma = 0.98;
  for (int i = 1; i < num_states; i++) {
    double weight = pow(gamma, i);
    State body_state = interp_reduced_plan[i];
    z[i] = weight * z[0] + (1 - weight) * body_state.pos[2];
    pitch[i] = weight * pitch[0] +
               (1 - weight) * getPitchFromState(body_state, planner_config);
    unwrapped_yaw[i] =
        weight * unwrapped_yaw[0] + (1 - weight) * unwrapped_yaw[i];
  }

  // Hold penultimate yaw value to avoid singularity issues at terminal state
  // (often zero velocity)
  unwrapped_yaw[num_states - 1] = unwrapped_yaw[num_states - 2];

  // Filter yaw and compute its derivative via central difference method
  int window_size = 25;
  filtered_yaw = math_utils::movingAverageFilter(unwrapped_yaw, window_size);
  yaw_rate = math_utils::centralDiff(filtered_yaw, dt);
  filtered_yaw_rate = math_utils::movingAverageFilter(yaw_rate, window_size);
  pitch_rate = math_utils::centralDiff(pitch, dt);
  filtered_pitch_rate =
      math_utils::movingAverageFilter(pitch_rate, window_size);

  // Filter z with a much tighter window
  int z_window_size = 5;
  filtered_z = math_utils::movingAverageFilter(z, z_window_size);
  z_rate = math_utils::centralDiff(filtered_z, dt);
  filtered_z_rate = math_utils::movingAverageFilter(z_rate, z_window_size);

  std::vector<double> interp_t(num_states);
  for (int i = 0; i < num_states; i++) {
    interp_t[i] = i * dt;
  }

  // Wrap yaw again
  wrapped_yaw = math_utils::wrapToPi(unwrapped_yaw);
  filtered_yaw = math_utils::wrapToPi(filtered_yaw);

  // Add full state data into the array
  std::vector<double> x_vec, y_vec, z_vec;
  for (int i = 0; i < num_states; i++) {
    State body_state = interp_reduced_plan[i];
    // body_state[2] = filtered_z[i];
    body_state.vel[2] = filtered_z_rate[i];
    FullState body_full_state =
        stateToFullState(body_state, roll, pitch[i], filtered_yaw[i], roll_rate,
                         filtered_pitch_rate[i], filtered_yaw_rate[i]);

    interp_full_plan.push_back(body_full_state);
    x_vec.push_back(body_state.pos[0]);
    y_vec.push_back(body_state.pos[1]);
    z_vec.push_back(body_state.pos[2]);
  }
}

GRF getGRF(const Action &a, double t, int phase,
           const PlannerConfig &planner_config) {
  double m = planner_config.mass;
  double g = planner_config.g;

  GRF grf;

  if (phase == LEAP_STANCE) {
    grf = m * g * a.grf_0 * t * 1.0 / (a.t_s_leap * a.t_s_leap) *
          (t - a.t_s_leap) * -4.0;
  } else if (phase == FLIGHT) {
    grf.setZero();
  } else if (phase == LAND_STANCE) {
    grf = m * g * a.grf_f * t * 1.0 / (a.t_s_land * a.t_s_land) *
          (t - a.t_s_land) * -4.0;
  } else if (phase == CONNECT) {
    grf = m * (a.grf_0 + (a.grf_f - a.grf_0) * t / a.t_s_leap -
               planner_config.g_vec);
    grf[2] = m * g;
  } else {
    throw std::runtime_error("Invalid stance phase in getGRF");
  }

  return grf;
}

Eigen::Vector3d getAcceleration(const Action &a, double t, int phase,
                                const PlannerConfig &planner_config) {
  return (getGRF(a, t, phase, planner_config) / planner_config.mass +
          planner_config.g_vec);
}

double getPitchFromState(const State &s, const PlannerConfig &planner_config) {
  Eigen::Vector3d surf_norm = getSurfaceNormalFiltered(s, planner_config);

  // Get magnitude of lateral velociy
  double vel = s.vel.head<2>().norm();

  // If velocity is zero or surf norm undefined, assume v is in +x direction
  double v_proj = (vel == 0 || surf_norm[2] <= 0)
                      ? surf_norm[0]
                      : s.vel.head<2>().dot(surf_norm.head<2>()) / vel;

  // set pitch to angle that aligns v_proj with surface normal
  return atan2(v_proj, surf_norm[2]);
}

double getDzFromState(const State &s, const PlannerConfig &planner_config) {
  Eigen::Vector3d surf_norm = getSurfaceNormalFiltered(s, planner_config);

  return ((surf_norm[2] <= 0)
              ? std::numeric_limits<double>::max()
              : -(s.vel.head<2>().dot(surf_norm.head<2>()) / surf_norm[2]));
}

void setDz(State &s, const PlannerConfig &planner_config) {
  Eigen::Vector3d surf_norm = getSurfaceNormalFiltered(s, planner_config);

  s.vel[2] = (surf_norm[2] <= 0)
                 ? std::numeric_limits<double>::max()
                 : -(s.vel.head<2>().dot(surf_norm.head<2>()) / surf_norm[2]);
}

void setDz(State &s, const Eigen::Vector3d &surf_norm) {
  s.vel[2] = (surf_norm[2] <= 0)
                 ? std::numeric_limits<double>::max()
                 : (s.vel.head<2>().dot(surf_norm.head<2>()) / surf_norm[2]);
}

State interpStateActionPair(const State &s_in, const Action &a, double t0,
                            double dt, std::vector<State> &interp_reduced_plan,
                            std::vector<GRF> &interp_GRF,
                            std::vector<double> &interp_t,
                            std::vector<int> &interp_primitive_id,
                            std::vector<double> &interp_length,
                            const PlannerConfig &planner_config) {
  double t_s = a.t_s_leap;
  double t_f = a.t_f;
  double t_s_land = a.t_s_land;

  State s = s_in;

  int phase = (t_f == 0) ? CONNECT : LEAP_STANCE;
  if (phase == LEAP_STANCE) {
    s.vel[2] = a.dz_0;
  }

  // Add points during stance phase
  for (double t = 0; t < t_s; t += dt) {
    interp_t.push_back(t0 + t);
    State s_next = applyStance(s, a, t, phase, planner_config);
    if (!interp_reduced_plan.empty()) {
      interp_length.push_back(interp_length.back() +
                              poseDistance(s_next, interp_reduced_plan.back()));
    }
    interp_reduced_plan.push_back(s_next);
    interp_GRF.push_back(getGRF(a, t, phase, planner_config));

    interp_primitive_id.push_back(phase);
  }

  State s_takeoff = applyStance(s, a, phase, planner_config);

  // Include the remainder of the flight phase (double count takeoff state to
  // get discontinuous dropoff in grf when interpolating)
  for (double t = 0; t < t_f; t += dt) {
    interp_t.push_back(t0 + t_s + t);
    State s_next = applyFlight(s_takeoff, t, planner_config);
    interp_length.push_back(interp_length.back() +
                            poseDistance(s_next, interp_reduced_plan.back()));
    interp_reduced_plan.push_back(s_next);
    GRF grf = {0, 0, 0};
    interp_GRF.push_back(grf);
    interp_primitive_id.push_back(FLIGHT);
  }

  State s_land = applyFlight(s_takeoff, t_f, planner_config);
  State s_final = s_land;

  if (phase != CONNECT) {
    phase = LAND_STANCE;

    // Add points during stance phase
    for (double t = 0; t < t_s_land; t += dt) {
      interp_t.push_back(t0 + t_s + t_f + t);
      State s_next = applyStance(s_land, a, t, phase, planner_config);
      if (!interp_reduced_plan.empty()) {
        interp_length.push_back(
            interp_length.back() +
            poseDistance(s_next, interp_reduced_plan.back()));
      }
      interp_reduced_plan.push_back(s_next);
      interp_GRF.push_back(getGRF(a, t, phase, planner_config));

      interp_primitive_id.push_back(phase);
    }
    s_final = applyStance(s_land, a, t_s_land, phase, planner_config);
  }
  return s_final;
}

void getInterpPlan(FullState start_state,
                   const std::vector<State> &state_sequence,
                   const std::vector<Action> &action_sequence, double dt,
                   double t0, std::vector<FullState> &interp_full_plan,
                   std::vector<GRF> &interp_GRF, std::vector<double> &interp_t,
                   std::vector<int> &interp_primitive_id,
                   std::vector<double> &interp_length,
                   const PlannerConfig &planner_config) {
  std::vector<State> interp_reduced_plan;

  // Loop through state action pairs, interp each and add to the path
  for (int i = 0; i < action_sequence.size(); i++) {
    interpStateActionPair(state_sequence[i], action_sequence[i], t0, dt,
                          interp_reduced_plan, interp_GRF, interp_t,
                          interp_primitive_id, interp_length, planner_config);
    t0 += (action_sequence[i].t_s_leap + action_sequence[i].t_f +
           action_sequence[i].t_s_land);
  }

  // Add the final state in case it was missed by interp (GRF is undefined
  // here so just copy the last element)
  interp_t.push_back(t0);
  interp_length.push_back(
      interp_length.back() +
      poseDistance(state_sequence.back(), interp_reduced_plan.back()));
  interp_reduced_plan.push_back(state_sequence.back());
  interp_GRF.push_back(interp_GRF.back());
  interp_primitive_id.push_back(LEAP_STANCE);

  // Lift from reduced into full body plan
  addFullStates(start_state, interp_reduced_plan, dt, interp_full_plan,
                planner_config);
}

Eigen::Vector3d rotateGRF(const Eigen::Vector3d &surface_norm_eig,
                          const Eigen::Vector3d &grf_eig) {
  // Declare unit Z vector
  Eigen::Vector3d surface_norm_unit_eig, Zs;
  Zs << 0, 0, 1;

  // Normalize surface normal
  surface_norm_unit_eig = surface_norm_eig.normalized();

  // Compute priors
  Eigen::Vector3d v = Zs.cross(surface_norm_eig);
  double s = v.norm();
  double c = Zs.dot(surface_norm_eig);

  Eigen::Matrix3d vskew;
  vskew << 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0;

  Eigen::Matrix3d
      R;  // Rotation matrix to rotate from contact frame to spatial frame
  double eps = 1e-6;
  if (s < eps) {
    R = Eigen::Matrix3d::Identity();
  } else {
    R = Eigen::Matrix3d::Identity() + vskew + vskew * vskew * (1 - c) / (s * s);
  }

  Eigen::Vector3d grf_spatial_eig = R * grf_eig;

  return grf_spatial_eig;
}

State applyStance(const State &s, const Action &a, double t, int phase,
                  const PlannerConfig &planner_config) {
  double g = planner_config.g;
  State s_new = s;

  if (phase == CONNECT) {
    Eigen::Vector3d acc_0 = a.grf_0 * g + planner_config.g_vec;
    Eigen::Vector3d acc_f = a.grf_f * g + planner_config.g_vec;

    // Note: in connect phase, a.grf represents acceleration
    s_new.pos = s.pos + s.vel * t + 0.5 * (acc_0)*t * t +
                (acc_f - acc_0) * (t * t * t) / (6.0 * a.t_s_leap);
    s_new.vel =
        s.vel + acc_0 * t + (acc_f - acc_0) * t * t / (2.0 * a.t_s_leap);

    // Set height based on terrain
    // (interp from state 1 clearance to state 2 clearance, add terrain height)
    s_new.pos[2] = a.grf_0[2] + (a.grf_f[2] - a.grf_0[2]) * (t / a.t_s_leap) +
                   getTerrainZFilteredFromState(s_new, planner_config);
    setDz(s_new, planner_config);

  } else {
    GRF peak_grf = (phase == LEAP_STANCE) ? a.grf_0 : a.grf_f;
    double t_s = (phase == LEAP_STANCE) ? a.t_s_leap : a.t_s_land;
    s_new.vel[2] = (phase == LEAP_STANCE) ? a.dz_0 : s_new.vel[2];

    s_new.pos = s_new.pos -
                1.0 / (t_s * t_s) *
                    ((g * peak_grf * t * t * t * t) / 3.0 -
                     g * peak_grf * t * t * t * t_s * (2.0 / 3.0)) +
                s_new.vel * t + planner_config.g_vec * t * t * 0.5;

    s_new.vel = s_new.vel -
                g * peak_grf * t * t * 1.0 / (t_s * t_s) *
                    (t * 2.0 - t_s * 3.0) * (2.0 / 3.0) +
                planner_config.g_vec * t;
  }

  return s_new;
}

State applyStance(const State &s, const Action &a, int phase,
                  const PlannerConfig &planner_config) {
  double t_s =
      (phase == CONNECT || phase == LEAP_STANCE) ? a.t_s_leap : a.t_s_land;
  return applyStance(s, a, t_s, phase, planner_config);
}

State applyFlight(const State &s, double t_f,
                  const PlannerConfig &planner_config) {
  State s_new;
  s_new.pos = s.pos + s.vel * t_f + 0.5 * planner_config.g_vec * t_f * t_f;
  s_new.vel = s.vel + planner_config.g_vec * t_f;

  return s_new;
}

State applyAction(const State &s, const Action &a,
                  const PlannerConfig &planner_config) {
  State s_new;

  if (a.t_f == 0) {
    s_new = applyStance(s, a, CONNECT, planner_config);
  } else {
    State s_to = applyStance(s, a, LEAP_STANCE, planner_config);
    State s_land = applyFlight(s_to, a.t_f, planner_config);
    s_new = applyStance(s_land, a, LAND_STANCE, planner_config);
  }

  return s_new;
}

bool getRandomLeapAction(const State &s, const Eigen::Vector3d &surf_norm,
                         Action &a, const PlannerConfig &planner_config) {
  // Declare variables;
  const double g = planner_config.g;
  const double m = planner_config.mass;

  // Sample stance time and initial vertical velocity
  double dz_impulse = (planner_config.dz0_max - planner_config.dz0_min) *
                          (double)rand() / RAND_MAX +
                      planner_config.dz0_min;
  a.dz_0 = getDzFromState(s, planner_config) - dz_impulse;
  a.dz_f = dz_impulse;
  a.t_s_leap = (planner_config.t_s_max - planner_config.t_s_min) *
                   (double)rand() / RAND_MAX +
               planner_config.t_s_min;
  a.t_f = 1e-6;
  a.t_s_land = a.t_s_leap;
  a.grf_0.setZero();
  a.grf_f.setZero();

  bool is_valid = refineAction(s, a, planner_config);

  return is_valid;
}

bool refineAction(const State &s, Action &a,
                  const PlannerConfig &planner_config) {
  if (!refineStance(s, LEAP_STANCE, a, planner_config)) return false;

  State s_leap = applyStance(s, a, LEAP_STANCE, planner_config);

  if (!refineFlight(s_leap, a.t_f, planner_config)) {
    return false;
  }

  State s_land = applyFlight(s_leap, a.t_f, planner_config);

  if (!refineStance(s_land, LAND_STANCE, a, planner_config)) return false;

  State s_final = applyStance(s_land, a, LAND_STANCE, planner_config);
  a.dz_f = s_final.vel[2];

  return true;
}

bool refineStance(const State &s, int phase, Action &a,
                  const PlannerConfig &planner_config) {
#ifdef DEBUG_REFINE_STATE
  std::cout << "Starting stance refine, phase = " << phase << std::endl;
#endif

  // Load parameters (use references for action params that may change)
  double &t_s = (phase == LEAP_STANCE) ? a.t_s_leap : a.t_s_land;
  GRF &grf_stance = (phase == LEAP_STANCE) ? a.grf_0 : a.grf_f;
  double dz_0 = (phase == LEAP_STANCE) ? a.dz_0 : s.vel[2];
  double g = planner_config.g;
#ifdef DEBUG_REFINE_STATE
  std::cout << "dz_0 = " << dz_0 << std::endl;
#endif

  // Declare booleans for validity checking
  bool grf_valid, friction_cone_valid, final_state_valid, midstance_state_valid,
      grf_increased;
  grf_valid = friction_cone_valid = final_state_valid = midstance_state_valid =
      grf_increased = false;
  State s_final, s_midstance;

  // Compute nominal z force (corresponds to zero net height displacement over
  // stance)
  double f_z_nominal =
      ((dz_0 * 6.0 - g * t_s * 3.0) * (-1.0 / 2.0)) / (g * t_s);

  // // Sample lateral forces, respecting friction about the nominal peak grf
  double ang_az = 2 * M_PI * (double)rand() / RAND_MAX;
  double f_lateral_mag = (double)rand() / RAND_MAX * planner_config.mu;
  Eigen::Vector3d pos_f;
  grf_stance[0] = f_lateral_mag * f_z_nominal * cos(ang_az);
  grf_stance[1] = f_lateral_mag * f_z_nominal * sin(ang_az);
  grf_stance[2] = f_z_nominal;

  while (!(grf_valid && friction_cone_valid && final_state_valid &&
           midstance_state_valid)) {
    s_final = applyStance(s, a, t_s, phase, planner_config);
    if (!isInMap(s_final, planner_config)) {
      return false;
    }
    pos_f = s_final.pos;

    double buffer = 3e-2;
    if (phase == LEAP_STANCE) {
      isValidState(s_final, planner_config, phase, pos_f[2]);
      pos_f[2] -= buffer;
    } else {
      pos_f[2] = planner_config.h_nom + getTerrainZ(pos_f, planner_config);
    }

    // Compute final GRF
    grf_stance[2] =
        (1.0 / (t_s * t_s) *
         (s.pos[2] - pos_f[2] + (t_s * (dz_0 * 6.0 - g * t_s * 3.0)) / 6.0) *
         -3.0) /
        g;
    s_final = applyStance(s, a, t_s, phase, planner_config);
    s_midstance = applyStance(s, a, 0.5 * t_s, phase, planner_config);
    s_midstance.pos[2] -=
        0.01;  // Make sure validity is robust to discretization

    // Compute validity checks
    // TODO(jcnorby): sample surface normal from ground projection
    // (non-filtered)
    grf_valid =
        (grf_stance.norm() <= planner_config.grf_max) && (grf_stance[2] >= 1);
    friction_cone_valid =
        (grf_stance.head<2>().norm() <= abs(grf_stance[2] * planner_config.mu));

    final_state_valid = isValidState(s_final, planner_config, phase);
    midstance_state_valid = isValidState(s_midstance, planner_config, phase);

    // Exit if final state is invalid
    if (!final_state_valid) {
#ifdef DEBUG_REFINE_STATE
      std::cout << "Final state invalid, exiting" << std::endl;
      printState(s_final);
#endif
      return false;
    }

    // Enforce friction cone
    if (!friction_cone_valid) {
      // std::cout << "grf_stance before = \n" << grf_stance << std::endl;
      grf_stance.head<2>() = 0.9 * grf_stance.head<2>() * grf_stance[2] *
                             planner_config.mu / grf_stance.head<2>().norm();

#ifdef DEBUG_REFINE_STATE
      std::cout << "Friction cone violated, trying again" << std::endl;
#endif
      continue;
    }

    // Increase t_s or dz0 (decrease GRF) if GRF exceeds limits (exiting if
    // midstance was already infeasible)
    if (!grf_valid) {
      if (!midstance_state_valid || grf_increased) {
#ifdef DEBUG_REFINE_STATE
        std::cout
            << "GRF exceeds limits and midstance is or was invalid, exiting"
            << std::endl;
#endif
        return false;
      }

      if (phase == LEAP_STANCE) {
        dz_0 += 0.2;
#ifdef DEBUG_REFINE_STATE
        std::cout << "GRF exceeds limits, reducing initial downwards velocity"
                  << std::endl;
#endif
      } else {
#ifdef DEBUG_REFINE_STATE
        std::cout << "GRF exceeds limits, increasing stance time" << std::endl;
#endif
        t_s = t_s * 1.2;
      }
      continue;
    }

    // Decrease t_s or dz0 (increase GRF) if midstance is infeasible
    if (!midstance_state_valid) {
#ifdef DEBUG_REFINE_STATE
      std::cout << "Midstance state invalid, reducing stance time from " << t_s
                << " to " << t_s / 1.2 << std::endl;
#endif
      // t_s = t_s / 1.1;
      dz_0 -= 0.2;

      grf_increased = true;
      continue;
    }
  }
  if (phase == LEAP_STANCE) {
    a.dz_0 = dz_0;
  } else {
    a.dz_f = s_final.vel[2];
  }

#ifdef DEBUG_REFINE_STATE
  std::cout << "stance refine success" << std::endl;
#endif
  return true;
}

bool refineFlight(const State &s, double &t_f,
                  const PlannerConfig &planner_config) {
#ifdef DEBUG_REFINE_STATE
  std::cout << "starting flight refine" << std::endl;
#endif
  bool is_valid = false;
  double t = 0;
  double z_f = planner_config.h_nom + 0.05;
  State s_check = s;
  double h = getZRelToTerrain(s_check, planner_config);

  double h_last = h;
  while (h >= planner_config.h_min) {
    if ((h <= z_f) && (h_last > z_f)) {
      t_f = t;
      is_valid = true;
    }
    t += planner_config.dt;
    s_check = applyFlight(s, t, planner_config);
    if (!isInMap(s_check, planner_config)) {
      break;
    }
    h_last = h;
    h = getZRelToTerrain(s_check, planner_config);
  }
  if (!is_valid) {
    State s_test = applyFlight(s, planner_config.dt, planner_config);
#ifdef DEBUG_REFINE_STATE
    std::cout << "No valid flight detected" << std::endl;
    std::cout << "t = " << t << std::endl;
    printState(s);
    printState(s_test);
#endif
  }
  return is_valid;
}

bool isValidAction(const Action &a, const PlannerConfig &planner_config) {
  if ((a.t_s_leap <= 0) || (a.t_f < 0)) return false;

  double m = planner_config.mass;
  double g = planner_config.g;
  double mu = planner_config.mu;

  // Get peak forces
  GRF grf_0, grf_f;
  grf_0 = a.grf_0;
  grf_f = a.grf_f;

  if (a.t_f == 0) {
    grf_0[2] = 1.0;
    grf_f[2] = 1.0;
  }

  // Check force limits
  if ((grf_0.norm() >= planner_config.grf_max) ||
      (grf_f.norm() >= planner_config.grf_max) || (grf_0[2] < 0) ||
      (grf_f[2] < 0)) {
    return false;
  }

  // Check friction cone
  if ((grf_0.head<2>().norm() >= mu * grf_0[2]) ||
      (grf_f.head<2>().norm() >= mu * grf_f[2])) {
    return false;
  }

  return true;
}

bool isValidState(const State &s, const PlannerConfig &planner_config,
                  int phase) {
  double dummy_max_valid_z = 0;
  return isValidState(s, planner_config, phase, dummy_max_valid_z);
}

bool isValidState(const State &s, const PlannerConfig &planner_config,
                  int phase, double &max_valid_z) {
  // Check elevation independent constraints first (out of range, velocity)
  if (!isInMap(s, planner_config)) {
#ifdef DEBUG_INVALID_STATE
    printf("Out of terrain range, phase = %d\n", phase);
    printStateNewline(s);
#endif
    return false;
  }

  // Ensure body is over traversable terrain unless in flight or leaping
  // disabled
  if (!isTraversable(s.pos, planner_config) && phase != FLIGHT &&
      planner_config.enable_leaping) {
#ifdef DEBUG_INVALID_STATE
    printf("!isContactTraversable, phase = %d\n", phase);
    printStateNewline(s);
#endif
    return false;
  }

  if (s.vel.head<2>().norm() >
      planner_config.v_max) {  // Ignore limit on vertical velocity since this
                               // is accurately bounded by gravity
#ifdef DEBUG_INVALID_STATE
    printf("v_max exceeded, phase = %d\n", phase);
    printStateNewline(s);
#endif
    return false;
  }

  // Find yaw, pitch, and their sines and cosines
  // TODO(jcnorby): Add getRollFromState
  double yaw = atan2(s.vel[1], s.vel[0]);
  double cy = cos(yaw);
  double sy = sin(yaw);
  double pitch = getPitchFromState(s, planner_config);
  double cp = cos(pitch);
  double sp = sin(pitch);

  // Compute each element of the rotation matrix
  Eigen::Matrix3d R_mat;
  R_mat << cy * cp, -sy, cy * sp, sy * cp, cy, sy * sp, -sp, 0, cp;

  // Compute the collision points in the world frame
  Eigen::Matrix<double, 3, planner_config.num_collision_points>
      collision_points_world =
          R_mat * planner_config.collision_points_body +
          s.pos.replicate(1, planner_config.num_collision_points);

  // Check each of the four corners of the robot
  for (int i = 0; i < planner_config.num_collision_points; i++) {
    Eigen::Vector3d collision_point = collision_points_world.col(i);
    if (!isInMap(collision_point, planner_config)) {
#ifdef DEBUG_INVALID_STATE
      printf("collision_point not in map, phase = %d\n", phase);
      printStateNewline(s);
#endif
      return false;
    }

    double collision_clearance =
        getZRelToTerrain(collision_point, planner_config);

    // Check for collision
    if (collision_clearance < planner_config.h_min) {
#ifdef DEBUG_INVALID_STATE
      printf("h_min exceeded for leg, phase = %d\n", phase);
      printStateNewline(s);
#endif
      return false;
    }
  }

  // Initialize max_valid_z to infinity (ensures that update will overwrite it)
  max_valid_z = std::numeric_limits<double>::max();

  // Compute the reachability points in the world frame
  Eigen::Matrix<double, 3, planner_config.num_reachability_points>
      reachability_points_world =
          R_mat * planner_config.reachability_points_body +
          s.pos.replicate(1, planner_config.num_reachability_points);

  // TODO(jcnorby): allow alternative gaits, check region around foot location
  for (int i = 0; i < planner_config.num_reachability_points; i++) {
    Eigen::Vector3d reachability_point = reachability_points_world.col(i);
    if (!isInMap(reachability_point, planner_config)) {
#ifdef DEBUG_INVALID_STATE
      printf("reachability_point not in map, phase = %d\n", phase);
#endif
      max_valid_z = s.pos[2] + planner_config.h_max -
                    getZRelToTerrain(reachability_point, planner_config);
      return false;
    }

    // Make sure legs are over a valid region of the terrain
    if (!isTraversable(reachability_point, planner_config) && phase != FLIGHT) {
      max_valid_z = s.pos[2] + planner_config.h_max -
                    getZRelToTerrain(reachability_point, planner_config);
      return false;
    }

    double reachability_clearance =
        getZRelToTerrain(reachability_point, planner_config);

    bool is_rear_leg = (i % 2 == 1);

    // Check for reachability
    if (phase == CONNECT) {
      max_valid_z = std::min(max_valid_z, s.pos[2] + planner_config.h_max -
                                              reachability_clearance);
      if (reachability_clearance > planner_config.h_max) {
#ifdef DEBUG_INVALID_STATE
        printf("h_max exceeded, phase = %d\n", phase);
#endif
        return false;
      }
    } else if (phase == LEAP_STANCE) {
      if (is_rear_leg) {
        max_valid_z = std::min(max_valid_z, s.pos[2] + planner_config.h_max -
                                                reachability_clearance);
        if (reachability_clearance > planner_config.h_max) {
#ifdef DEBUG_INVALID_STATE
          printf("h_max exceeded, phase = %d\n", phase);
          printStateNewline(s);
#endif
          return false;
        }
      }
    } else if (phase == LAND_STANCE) {
      // if (!is_rear_leg) {
      max_valid_z = std::min(max_valid_z, s.pos[2] + planner_config.h_max -
                                              reachability_clearance);
      if (reachability_clearance > planner_config.h_max) {
#ifdef DEBUG_INVALID_STATE
        printf("h_max exceeded, phase = %d\n", phase);
        printStateNewline(s);
#endif
        return false;
      }
    }
  }

  return true;
}

bool isValidStateActionPair(const State &s_in, const Action &a,
                            StateActionResult &result,
                            const PlannerConfig &planner_config) {
  // Declare stance and flight times
  double t_s = a.t_s_leap;
  double t_f = a.t_f;
  double t_s_land = a.t_s_land;

  // Declare states, actions, and lengths for upcoming validity and distance
  // checks
  State s = s_in;
  State s_prev = s;
  State s_next;
  result.length = 0;
  result.s_new = s;
  result.a_new = a;

  // Initialize phase
  int phase = (t_f == 0) ? CONNECT : LEAP_STANCE;

  // LEAP (OR CONNECT) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  double t = 0;

  while (t <= t_s) {
    // Compute state to check
    State s_next = applyStance(s, a, t, phase, planner_config);

    if (!isValidState(s_next, planner_config, phase)) {
      result.t_new = (1.0 - planner_config.backup_ratio) * t;
      result.s_new = applyStance(s, a, result.t_new, phase, planner_config);
      result.a_new.t_s_leap = result.t_new;
      result.a_new.t_s_land = std::min(0.001, result.a_new.t_s_land);
#ifdef DEBUG_INVALID_STATE
      printf("Invalid leaping stance config\n");
#endif

      return false;
    } else {
      result.length += poseDistance(s_next, s_prev);
      s_prev = s_next;
    }

    t += planner_config.dt;
  }

  State s_takeoff = applyStance(s, a, phase, planner_config);

  if (!isValidState(s_takeoff, planner_config, phase)) {
    result.t_new = (1.0 - planner_config.backup_ratio) * t_s;
    result.s_new = applyStance(s, a, result.t_new, planner_config);
    result.a_new.t_f = std::min(0.001, result.a_new.t_f);
    result.a_new.t_s_land = std::min(0.001, result.a_new.t_s_land);
#ifdef DEBUG_INVALID_STATE
    printf("Invalid takeoff config\n");
#endif
    return false;
  } else {
    result.s_new = s_takeoff;
    result.t_new = t_s;
    result.length += poseDistance(s_takeoff, s_prev);
  }

  // FLIGHT ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  t = planner_config.dt;
  while (t < t_f) {
    State s_next = applyFlight(s_takeoff, t, planner_config);

    // Check collision in flight
    if (!isValidState(s_next, planner_config, FLIGHT)) {
#ifdef DEBUG_INVALID_STATE
      printf("Flight collision, exiting\n");
      printState(s_next);
      std::cout << "t = " << t << std::endl;
      std::cout << "t_f = " << t_f << std::endl;
#endif
      result.s_new = s_next;
      return false;
    }
    result.length += poseDistance(s_next, s_prev);
    s_prev = s_next;

    t += planner_config.dt;
  }

  // LANDING ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  if (phase != CONNECT) {
    State s_land = applyFlight(s_takeoff, t_f, planner_config);

    t = 0;
    while (t < t_s_land) {
      // Compute state to check
      State s_next =
          applyStance(s_land, result.a_new, t, LAND_STANCE, planner_config);

      if (!isValidState(s_next, planner_config, LAND_STANCE)) {
#ifdef DEBUG_INVALID_STATE
        printf("Invalid landing stance config\n");
#endif
        result.s_new = s_next;
        return false;
      } else {
        result.length += poseDistance(s_next, s_prev);
        s_prev = s_next;
      }
      t += planner_config.dt;
    }

    State s_final =
        applyStance(s_land, result.a_new, LAND_STANCE, planner_config);

    if (!isValidState(s_final, planner_config, LAND_STANCE)) {
#ifdef DEBUG_INVALID_STATE
      printf("Invalid s_final config");
#endif
      return false;
    }
    result.s_new = s_final;
  }

  return true;
}

void publishStateActionPair(const State &s, const Action &a,
                            const State &s_goal,
                            const PlannerConfig &planner_config,
                            visualization_msgs::MarkerArray &tree_viz_msg,
                            ros::Publisher &tree_pub) {
  // Confirm ros is ok
  if (!ros::ok()) return;

  // Create the marker object for the new state action pair
  visualization_msgs::Marker state_action_line;
  state_action_line.header.stamp = ros::Time::now();
  state_action_line.header.frame_id = "map";
  state_action_line.action = visualization_msgs::Marker::ADD;
  state_action_line.pose.orientation.w = 1;
  state_action_line.id = tree_viz_msg.markers.size();
  state_action_line.type = visualization_msgs::Marker::LINE_STRIP;
  state_action_line.scale.x = 0.06;

  // Interpolate the state action pair
  double t0 = 0;
  double dt = planner_config.dt;
  std::vector<State> interp_state_action;
  std::vector<GRF> interp_grf;
  std::vector<double> interp_t;
  std::vector<int> interp_primitive_id;
  std::vector<double> interp_length;
  interp_length.push_back(0.0);

  interpStateActionPair(s, a, t0, dt, interp_state_action, interp_grf, interp_t,
                        interp_primitive_id, interp_length, planner_config);

  // Add the exact last state so the visualization is continuous
  interp_state_action.push_back(applyAction(s, a, planner_config));
  interp_primitive_id.push_back(interp_primitive_id.back());

  // Loop through the interpolated state action pair and add points to the line
  // strip
  int length = interp_state_action.size();
  for (int i = 0; i < length; i++) {
    // Load in the pose data from the interpolated path
    geometry_msgs::Point point;
    point.x = interp_state_action.at(i).pos.x();
    point.y = interp_state_action.at(i).pos.y();
    point.z = interp_state_action.at(i).pos.z();

    // Set the color of the line strip according to the motion primitive
    std_msgs::ColorRGBA color;
    color.a = 1;
    if (interp_primitive_id[i] == FLIGHT) {
      color.r = 0 / 255.0;
      color.g = 45.0 / 255.0;
      color.b = 144.0 / 255.0;
    } else if (interp_primitive_id[i] == LEAP_STANCE) {
      color.r = 0 / 255.0;
      color.g = 132.0 / 255.0;
      color.b = 61.0 / 255.0;
    } else if (interp_primitive_id[i] == LAND_STANCE) {
      color.r = 0 / 255.0;
      color.g = 200.0 / 255.0;
      color.b = 100.0 / 255.0;
    } else if (interp_primitive_id[i] == CONNECT) {
      color.r = 166.0 / 255.0;
      color.g = 25.0 / 255.0;
      color.b = 46.0 / 255.0;
    } else {
      ROS_WARN_THROTTLE(1, "Invalid primitive ID received in planning utils");
    }

    // Add the point and the color
    state_action_line.points.push_back(point);
    state_action_line.colors.push_back(color);
  }

  // Add the marker to the array
  tree_viz_msg.markers.push_back(state_action_line);

  // Create the marker object for the goal marker of this particular action
  visualization_msgs::Marker goal_marker;
  goal_marker.header.stamp = ros::Time::now();
  goal_marker.header.frame_id = "map";
  goal_marker.action = visualization_msgs::Marker::ADD;
  goal_marker.id = 0;
  goal_marker.type = visualization_msgs::Marker::ARROW;

  // Define the point for the arrow base
  geometry_msgs::Point goal_marker_base;
  goal_marker_base.x = s_goal.pos[0];
  goal_marker_base.y = s_goal.pos[1];
  goal_marker_base.z = s_goal.pos[2];

  // Define the point for the arrow tip and set the scaling accordingly
  geometry_msgs::Point goal_marker_tip;
  double scale = 0.5;
  double vel_magnitude = getSpeed(s_goal);
  goal_marker.scale.x = 0.05 * scale * vel_magnitude;
  goal_marker.scale.y = 0.1 * scale * vel_magnitude;
  goal_marker_tip.x = s_goal.pos[0] + scale * s_goal.vel[0];
  goal_marker_tip.y = s_goal.pos[1] + scale * s_goal.vel[1];
  goal_marker_tip.z = s_goal.pos[2] + scale * s_goal.vel[2];

  // Add the points to the goal marker and set the color
  goal_marker.points.push_back(goal_marker_base);
  goal_marker.points.push_back(goal_marker_tip);
  goal_marker.color.a = 1.0;
  goal_marker.color.g = 1.0;

  // Add the goal marker to the front of the marker array
  tree_viz_msg.markers.front() = goal_marker;

  // Publish the tree and wait so that RViz has time to process it
  tree_pub.publish(tree_viz_msg);
  double freq = 5.0;  // Hz
  usleep(1000000.0 / freq);
}

}  // namespace planning_utils
