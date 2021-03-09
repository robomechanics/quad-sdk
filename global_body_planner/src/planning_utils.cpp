#include "global_body_planner/planning_utils.h"
#include "spirit_utils/matplotlibcpp.h"

namespace planning_utils {

namespace plt = matplotlibcpp;

State fullStateToState(FullState full_state)
{
  State state;
  double x = full_state[0];
  double y = full_state[1];
  double z = full_state[2];
  // double roll = full_state[3];
  // double pitch = full_state[4];
  // double yaw = full_state[5];
  double dx = full_state[6];
  double dy = full_state[7];
  double dz = full_state[8];
  // double roll_rate = full_state[9];
  // double pitch_rate = full_state[10];
  // double yaw_rate = full_state[11];

  state[0] = x;
  state[1] = y;
  state[2] = z;
  state[3] = dx;
  state[4] = dy;
  state[5] = dz;

  return state;
}

FullState stateToFullState(State state, double roll, double pitch, double yaw, 
  double roll_rate, double pitch_rate, double yaw_rate)
{
  FullState full_state;
  full_state.resize(FULLSTATEDIM);

  full_state[0] = state[0];
  full_state[1] = state[1];
  full_state[2] = state[2];
  full_state[3] = roll;
  full_state[4] = pitch;
  full_state[5] = yaw;
  full_state[6] = state[3];
  full_state[7] = state[4];
  full_state[8] = state[5];
  full_state[9] = roll_rate;
  full_state[10] = pitch_rate;
  full_state[11] = yaw_rate;

  return full_state;
}

void vectorToArray(State vec, double * new_array)
{
  for (int i = 0; i < vec.size(); i++)
    new_array[i] = vec.at(i);
}
void printState(State vec)
{
  printf("{");
  for (int i=0; i < vec.size(); i++)
    printf("%4.3f, ",vec[i]);
  printf("\b\b} ");
}
void printAction(Action a)
{
  std::cout << "{";
  for (int i= 0; i < a.size(); i++)
    std::cout << a[i] << ", ";
  std::cout << "\b\b}"; 
}
void printVector(std::vector<double> vec)
{
  std::cout << "{";
  for (double i= 0; i < vec.size(); i++)
    std::cout << vec[i] << ", ";
  std::cout << "\b\b}"; 
}
void printVectorInt(std::vector<int> vec)
{
  std::cout << "{";
  for (int i= 0; i < vec.size(); i++)
    std::cout << vec[i] << ", ";
  std::cout << "\b\b}"; 
}
void printStateNewline(State vec)
{
  printState(vec); std::cout << std::endl;
}
void printActionNewline(Action a)
{
  printAction(a); std::cout << std::endl;
}
void printVectorNewline(std::vector<double> vec)
{
  printVector(vec); std::cout << std::endl;
}
void printStateSequence(std::vector<State> state_sequence)
{
  for (State s : state_sequence)
    printStateNewline(s);
}
void printInterpStateSequence(std::vector<State> state_sequence, 
  std::vector<double> interp_t)
{
  for (int i=0;i<state_sequence.size();i++)
  {
    std::cout << interp_t[i] << "\t";
    printStateNewline(state_sequence[i]);
  }
}
void printActionSequence(std::vector<Action> action_sequence)
{
  for (Action a : action_sequence)
    printActionNewline(a);
}
void printVectorIntNewline(std::vector<int> vec)
{
  printVectorInt(vec); std::cout << std::endl;
}
void plotYaw(std::vector<double> interp_t,
  std::vector<FullState> interp_full_plan) {

  std::vector<double> yaw;
  std::vector<double> yaw_rate;

  for (FullState full_state : interp_full_plan) {
    yaw.push_back(full_state[5]);
    yaw_rate.push_back(full_state[11]);
  }

  // plt::clf();
  plt::ion();
  plt::named_plot("yaw", interp_t, yaw);
  plt::named_plot("yaw rate", interp_t, yaw_rate);
  plt::xlabel("t");
  plt::ylabel("yaw");
  plt::legend();
  plt::show();
  plt::pause(0.001);

}

double poseDistance(State q1, State q2)
{
  double sum = 0;
  for (int i = 0; i < POSEDIM; i++)
  {
    sum = sum + (q2[i] - q1[i])*(q2[i] - q1[i]);
  }

  double dist = sqrt(sum);
  return dist;
}

double poseDistance(std::vector<double> v1, std::vector<double> v2)
{
  double sum = 0;

  if (v1.size() < 3 || v2.size() < 3) {
    throw std::runtime_error("Not enough data in vectors to compute distance");
  }

  for (int i = 0; i < 3; i++)
  {
    sum = sum + (v2[i] - v1[i])*(v2[i] - v1[i]);
  }

  double dist = sqrt(sum);
  return dist;
}

double stateDistance(State q1,State q2)
{
  double sum = 0;
  State state_weight = {1,1,1,1,1,1};

  for (int i = 0; i < q1.size(); i++)
  {
    sum = sum + state_weight[i]*(q2[i] - q1[i])*(q2[i] - q1[i]);
  }

  double dist = sqrt(sum);
  return dist;
}

bool isWithinBounds(State s1, State s2)
{
  return (stateDistance(s1, s2) <= GOAL_BOUNDS);
}

void addFullStates(FullState start_state, std::vector<State> interp_reduced_plan, double dt, 
  std::vector<FullState> &interp_full_plan, FastTerrainMap& terrain) {

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
  std::vector<double> yaw(num_states);
  std::vector<double> filtered_yaw(num_states);
  std::vector<double> yaw_rate(num_states);
  std::vector<double> filtered_yaw_rate(num_states);

  // Enforce that yaw and pitch match current state, let the filter smooth things out
  z[0] = start_state[2];
  pitch[0] = start_state[4];
  yaw[0] = start_state[5];
  double gamma = 0.9;

  // Compute yaw to align with heading, pitch and height to align with the terrain
  for (int i = 1; i < num_states; i++) {
    double weight = pow(gamma,i);
    State body_state = interp_reduced_plan[i];
    z[i] = weight*z[0] + (1-weight)*getHeightFromState(body_state,terrain);
    pitch[i] = weight*pitch[0] + (1-weight)*getPitchFromState(body_state,terrain);
    yaw[i] = weight*yaw[0] + (1-weight)*atan2(body_state[4],body_state[3]);
  }

  // Filter yaw and compute its derivative via central difference method
  int window_size = 15;
  filtered_yaw = math_utils::movingAverageFilter(yaw, window_size);
  yaw_rate = math_utils::centralDiff(filtered_yaw, dt);
  filtered_yaw_rate = math_utils::movingAverageFilter(yaw_rate, window_size);
  pitch_rate = math_utils::centralDiff(pitch, dt);
  filtered_pitch_rate = math_utils::movingAverageFilter(pitch_rate,window_size);

  // Filter z with a much tighter window
  int z_window_size = 5;
  filtered_z = math_utils::movingAverageFilter(z, z_window_size);
  z_rate = math_utils::centralDiff(filtered_z, dt);
  filtered_z_rate = math_utils::movingAverageFilter(z_rate,z_window_size);

  std::vector<double> interp_t(num_states);
  for (int i = 0; i < num_states; i++) {
    interp_t[i] = i*dt;
  }

  // plt::clf();
  // plt::ion();
  // plt::named_plot("yaw", interp_t, yaw);
  // plt::named_plot("filtered yaw", interp_t, filtered_yaw);
  // plt::named_plot("yaw rate", interp_t, yaw_rate);
  // plt::named_plot("filtered yaw rate", interp_t, filtered_yaw_rate);
  // plt::xlabel("t");
  // plt::ylabel("yaw");
  // plt::legend();
  // plt::show();
  // plt::pause(0.001);

  // Add full state data into the array
  for (int i = 0; i < num_states; i++) {
    State body_state = interp_reduced_plan[i];
    body_state[2] = filtered_z[i];
    body_state[8] = filtered_z_rate[i];
    FullState body_full_state = stateToFullState(body_state, roll, pitch[i],
      filtered_yaw[i], roll_rate, filtered_pitch_rate[i], filtered_yaw_rate[i]);

    interp_full_plan.push_back(body_full_state);
  }
}

GRF getGRF(Action a,double t) {

  double m = M_CONST;
  double g = G_CONST;
  double t_s = a[6];

  GRF grf;

  // If in stance fill grf with appropriate data, otherwise leave as zeros
  if (t<=t_s) {
    // Get corresponding forces
    grf[0] = m*(a[0] + (a[3]-a[0])*t/t_s);
    grf[1] = m*(a[1] + (a[4]-a[1])*t/t_s);
    grf[2] = m*((a[2] + (a[5]-a[2])*t/t_s) + g);
  } else {
    grf = {0,0,0};
  }
  
  return grf;

}

double getPitchFromState(State s, FastTerrainMap& terrain) {

  std::array<double, 3> surf_norm = terrain.getSurfaceNormal(s[0], s[1]);

  double denom = s[3]*s[3] + s[4]*s[4];

  if (denom <= 0 || surf_norm[2] <=0) {
    double default_pitch = 0;
    return default_pitch;
  } else {

    double v_proj = (s[3]*surf_norm[0] + s[4]*surf_norm[1])/
      sqrt(denom);

    double pitch = atan2(v_proj,surf_norm[2]);

    return pitch;
  }
}

double getHeightFromState(State s, FastTerrainMap& terrain) {

  return (0.3+terrain.getGroundHeight(s[0], s[1]));

}

void interpStateActionPair(State s, Action a,double t0,double dt, 
  std::vector<State> &interp_reduced_plan, std::vector<GRF> &interp_GRF,
  std::vector<double> &interp_t, std::vector<int> &interp_primitive_id,
  FastTerrainMap& terrain)
{
  double t_s = a[6];
  double t_f = a[7];

  // Add points during stance phase
  for (double t = 0; t < t_s; t += dt)
  {
    interp_t.push_back(t0+t);
    interp_reduced_plan.push_back(applyStance(s,a,t,terrain));
    interp_GRF.push_back(getGRF(a,t));

    if (t_f==0)
      interp_primitive_id.push_back(CONNECT_STANCE);
    else
      interp_primitive_id.push_back(STANCE);
  }

  // Include the exact moment of liftoff if flight phase exists
  State s_takeoff = applyStance(s, a, terrain);
  if (t_f>0) {
    interp_t.push_back(t0+t_s);
    interp_reduced_plan.push_back(s_takeoff);
    interp_GRF.push_back(getGRF(a,t_s));
    interp_primitive_id.push_back(STANCE);
  }

  // Include the remainder of the flight phase (double count takeoff state to
  // get discontinuous dropoff in grf when interpolating)
  for (double t = 0; t < t_f; t += dt)
  {
    interp_t.push_back(t0+t_s+t);
    interp_reduced_plan.push_back(applyFlight(s_takeoff, t));
    GRF grf = {0,0,0};
    interp_GRF.push_back(grf);
    interp_primitive_id.push_back(FLIGHT);
  }

  // // Include the exact landing state if there is a flight phase
  // if (t_f>0)
  // {
  //   interp_t.push_back(t_s+t_f+t0);
  //   interp_path.push_back(applyFlight(s_takeoff, t_f));
  //   interp_GRF.push_back(getGRF(a,t_s+t_f));
  //   interp_primitive_id.push_back(STANCE);
  // }
}

void getInterpPlan(FullState start_state, std::vector<State> state_sequence,
  std::vector<Action> action_sequence,double dt, double t0,
  std::vector<FullState> &interp_full_plan, std::vector<GRF> &interp_GRF, 
  std::vector<double> &interp_t, std::vector<int> &interp_primitive_id,
  FastTerrainMap& terrain)
{
  std::vector<State> interp_reduced_plan;

  // Loop through state action pairs, interp each and add to the path
  for (int i=0; i < action_sequence.size();i++)
  {
    interpStateActionPair(state_sequence[i], action_sequence[i], t0, dt, 
      interp_reduced_plan, interp_GRF, interp_t, interp_primitive_id, terrain);
    t0 += (action_sequence[i][6] + action_sequence[i][7]);
  }

  // Add the final state in case it was missed by interp (GRF is undefined 
  // here so just copy the last element)
  interp_t.push_back(t0);
  interp_reduced_plan.push_back(state_sequence.back());
  interp_GRF.push_back(interp_GRF.back());
  interp_primitive_id.push_back(STANCE);

  // Lift from reduced into full body plan
  addFullStates(start_state, interp_reduced_plan, dt, interp_full_plan, terrain);

}

std::array<double,3> rotateGRF(std::array<double,3> surface_norm,
  std::array<double,3> grf)
{
  // Receive data and convert to Eigen
  Eigen::Vector3d Zs;
  Zs << 0,0,1;

  Eigen::Vector3d surface_norm_eig;
  surface_norm_eig << surface_norm[0],surface_norm[1],surface_norm[2];

  // Normalize surface normal
  // surface_norm_eig.normalize();

  Eigen::Vector3d grf_eig;
  grf_eig << grf[0],grf[1],grf[2];

  // Compute priors
  Eigen::Vector3d v = surface_norm_eig.cross(Zs);
  double s = v.norm();
  double c = surface_norm_eig.dot(Zs);

  Eigen::Matrix3d vskew;
  vskew << 0, -v[2], v[1],
       v[2], 0, -v[0],
       -v[1], v[0], 0;

  Eigen::Matrix3d R; // Rotation matrix to rotate from contact frame to spatial frame
  double eps = 1e-6;
  if (s < eps)
  {
    R = Eigen::Matrix3d::Identity();
  }
  else
  {
    R = Eigen::Matrix3d::Identity() + vskew + vskew*vskew * (1-c)/(s*s);
  }

  Eigen::Vector3d grf_spatial_eig = R*grf_eig;
  std::array<double,3> grf_spatial = {grf_spatial_eig[0],grf_spatial_eig[1],grf_spatial_eig[2]};

  return grf_spatial;
}

State applyStance(State s, Action a, double t, FastTerrainMap& terrain)
{
  double a_x_td = a[0]; // Acceleration in x at touchdown
  double a_y_td = a[1];
  double a_z_td = a[2];
  double a_x_to = a[3]; // Acceleration in x at takeoff
  double a_y_to = a[4];
  double a_z_to = a[5];
  double t_s = a[6];

  double x_td = s[0];
  double y_td = s[1];
  double z_td = s[2];
  double dx_td = s[3];
  double dy_td = s[4];
  double dz_td = s[5];

  State s_new;

  s_new[0] = x_td + dx_td*t + 0.5*a_x_td*t*t + (a_x_to - a_x_td)*(t*t*t)/(6.0*t_s);
  s_new[1] = y_td + dy_td*t + 0.5*a_y_td*t*t + (a_y_to - a_y_td)*(t*t*t)/(6.0*t_s);
  s_new[2] = z_td + dz_td*t + 0.5*a_z_td*t*t + (a_z_to - a_z_td)*(t*t*t)/(6.0*t_s);
  s_new[3] = dx_td + a_x_td*t + (a_x_to - a_x_td)*t*t/(2.0*t_s);
  s_new[4] = dy_td + a_y_td*t + (a_y_to - a_y_td)*t*t/(2.0*t_s);
  s_new[5] = dz_td + a_z_td*t + (a_z_to - a_z_td)*t*t/(2.0*t_s);

  if (a[7] > 0) {
    s_new[2] = z_td + dz_td*t + 0.5*a_z_td*t*t + (a_z_to - a_z_td)*(t*t*t)/(6.0*t_s);
    s_new[5] = dz_td + a_z_td*t + (a_z_to - a_z_td)*t*t/(2.0*t_s);
  } else {
    s_new[2] = getHeightFromState(s_new, terrain);
    s_new[5] = 0;//sqrt(s_new[3]*s_new[3] + s_new[4]*s_new[4])*sin();
  }

  return s_new;
}

State applyStance(State s, Action a, FastTerrainMap& terrain)
{
  return applyStance(s, a, a[6], terrain);
}

State applyFlight(State s, double t_f)
{
  double g = 9.81;
  double x_to = s[0];
  double y_to = s[1];
  double z_to = s[2];
  double dx_to = s[3];
  double dy_to = s[4];
  double dz_to = s[5];

  State s_new;
  s_new[0] = x_to + dx_to*t_f;
  s_new[1] = y_to + dy_to*t_f;
  s_new[2] = z_to + dz_to*t_f - 0.5*g*t_f*t_f;
  s_new[3] = dx_to;
  s_new[4] = dy_to;
  s_new[5] = dz_to - g*t_f;

  return s_new;
}

State applyAction(State s, Action a, FastTerrainMap& terrain)
{
  double t_s;
  double t_f;

  t_s = a[6];
  t_f = a[7];

  State s_to = applyStance(s, a, terrain);
  State s_new = applyFlight(s_to, t_f);
  return s_new;
}

State applyStanceReverse(State s, Action a, double t, FastTerrainMap& terrain)
{
  double a_x_td = a[0];
  double a_y_td = a[1];
  double a_z_td = a[2];
  double a_x_to = a[3];
  double a_y_to = a[4];
  double a_z_to = a[5];
  double t_s = a[6];

  double x_to = s[0];
  double y_to = s[1];
  double z_to = s[2];
  double dx_to = s[3];
  double dy_to = s[4];
  double dz_to = s[5];

  State s_new;

  double cx = dx_to - a_x_td*t_s - 0.5*(a_x_to - a_x_td)*t_s;
  double cy = dy_to - a_y_td*t_s - 0.5*(a_y_to - a_y_td)*t_s;
  double cz = dz_to - a_z_td*t_s - 0.5*(a_z_to - a_z_td)*t_s;

  s_new[3] = dx_to - a_x_td*(t_s - t) - (a_x_to - a_x_td)*(t_s*t_s - t*t)/(2.0*t_s);
  s_new[4] = dy_to - a_y_td*(t_s - t) - (a_y_to - a_y_td)*(t_s*t_s - t*t)/(2.0*t_s);
  s_new[5] = dz_to - a_z_td*(t_s - t) - (a_z_to - a_z_td)*(t_s*t_s - t*t)/(2.0*t_s);
  s_new[0] = x_to - cx*(t_s - t) - 0.5*a_x_td*(t_s*t_s - t*t) - (a_x_to - a_x_td)*(t_s*t_s*t_s - t*t*t)/(6.0*t_s);
  s_new[1] = y_to - cy*(t_s - t) - 0.5*a_y_td*(t_s*t_s - t*t) - (a_y_to - a_y_td)*(t_s*t_s*t_s - t*t*t)/(6.0*t_s);
  s_new[2] = z_to - cz*(t_s - t) - 0.5*a_z_td*(t_s*t_s - t*t) - (a_z_to - a_z_td)*(t_s*t_s*t_s - t*t*t)/(6.0*t_s);

  if (a[7] > 0) {
    s_new[2] = z_to - cz*(t_s - t) - 0.5*a_z_td*(t_s*t_s - t*t) - (a_z_to - a_z_td)*(t_s*t_s*t_s - t*t*t)/(6.0*t_s);
    s_new[5] = dz_to - a_z_td*(t_s - t) - (a_z_to - a_z_td)*(t_s*t_s - t*t)/(2.0*t_s);
  } else {
    s_new[2] = getHeightFromState(s_new, terrain);
    s_new[5] = 0;//sqrt(s_new[3]*s_new[3] + s_new[4]*s_new[4])*sin();
  }

  return s_new;
}

State applyStanceReverse(State s, Action a, FastTerrainMap& terrain)
{
  return applyStanceReverse(s, a, 0, terrain);
}

Action getRandomAction(std::array<double, 3> surf_norm)
{ 
  Action a;

  // Random normal forces between 0 and F_MAX
  double f_z_td = F_MAX*(double)rand()/RAND_MAX;
  double f_z_to = F_MAX*(double)rand()/RAND_MAX;

  // Random tangential forces between -mu*f_z and mu*f_z
  double f_x_td = 2*MU*f_z_td*(double)rand()/RAND_MAX - MU*f_z_td;
  double f_x_to = 2*MU*f_z_to*(double)rand()/RAND_MAX - MU*f_z_to;
  double f_y_td = 2*MU*f_z_td*(double)rand()/RAND_MAX - MU*f_z_td;
  double f_y_to = 2*MU*f_z_to*(double)rand()/RAND_MAX - MU*f_z_to;

  std::array<double, 3> f_td = {f_x_td, f_y_td, f_z_td};
  std::array<double, 3> f_to = {f_x_to, f_y_to, f_z_to};

  f_td = rotateGRF(surf_norm, f_td);
  f_to = rotateGRF(surf_norm, f_to);

  // Random stance and flight times between 0 and T_MAX
  // double t_s = (T_S_MAX - T_S_MIN)*(double)rand()/RAND_MAX + T_S_MIN;
  double t_s = 0.3;
  double t_f = (T_F_MAX - T_F_MIN)*(double)rand()/RAND_MAX + T_F_MIN;

  a[0] = f_td[0]/M_CONST;
  a[1] = f_td[1]/M_CONST;
  a[2] = f_td[2]/M_CONST - G_CONST;
  a[3] = f_to[0]/M_CONST;
  a[4] = f_to[1]/M_CONST;
  a[5] = f_to[2]/M_CONST - G_CONST;
  a[6] = t_s;
  a[7] = t_f;

  return a;
}

bool isValidAction(Action a)
{
   
  if ((a[6] <= 0) || (a[7] < 0))
    return false;

  double m = M_CONST;
  double g = G_CONST;
  double mu = MU;

  // Get corresponding forces
  double f_x_td = m*a[0];
  double f_y_td = m*a[1];
  double f_z_td = m*(a[2] + g);
  double f_x_to = m*a[3];
  double f_y_to = m*a[4];
  double f_z_to = m*(a[5] + g);

  // Check force limits
  if ((sqrt(f_x_td*f_x_td + f_y_td*f_y_td + f_z_td*f_z_td) >= F_MAX) || 
    (sqrt(f_x_to*f_x_to + f_y_to*f_y_to + f_z_to*f_z_to) >= F_MAX) ||
    (f_z_td < 0) || (f_z_to < 0) )
  {
    // std::cout << "Force limits violated" << std::endl;
    return false;
  }

  // Check friction cone
  if ((sqrt(f_x_td*f_x_td + f_y_td*f_y_td) >= mu*f_z_td) || 
    (sqrt(f_x_to*f_x_to + f_y_to*f_y_to) >= mu*f_z_to))
  {
    // std::cout << "Friction cone violated" << std::endl;
    return false;
  }

  return true;
}

bool isValidState(State s, FastTerrainMap& terrain, int phase)
{
  // if ((s[0] < terrain.getXData().front()) || (s[0] > terrain.getXData().back()) || (s[1] < terrain.getYData().front()) || (s[1] > terrain.getYData().back()) || (abs(s[6]) >= P_MAX) )
  //     return false;
  if (terrain.isInRange(s[0], s[1]) == false)
    return false;

  if (sqrt(s[3]*s[3] + s[4]*s[4]) > V_MAX) // Ignore limit on vertical velocity since this is accurately bounded by gravity
    return false;

  // Find yaw, pitch, and their sines and cosines
  double yaw = atan2(s[4],s[3]); double cy = cos(yaw); double sy = sin(yaw);
  
  double pitch = getPitchFromState(s,terrain);
  // double pitch = 0;
  double cp = cos(pitch); double sp = sin(pitch);  

  // Compute each element of the rotation matrix
  double R_11 = cy*cp; double R_12 = -sy; double R_13 = cy*sp;
  double R_21 = sy*cp; double R_22 = cy;  double R_23 = sy*sp;
  double R_31 = -sp;   double R_32 = 0;   double R_33 = cp; 

  std::array<double, 2> test_x = {-0.5*ROBOT_L, 0.5*ROBOT_L};
  std::array<double, 2> test_y = {-0.5*ROBOT_W, 0.5*ROBOT_W};
  double z_body = -ROBOT_H;

  // Check each of the four corners of the robot
  for (double x_body : test_x)
  {
    for (double y_body : test_y)
    {
      double x_leg = s[0] + R_11*x_body + R_12*y_body;
      double y_leg = s[1] + R_21*x_body + R_22*y_body;
      double z_leg = s[2] + R_31*x_body + R_32*y_body;

      double x_corner = x_leg + R_13*z_body;
      double y_corner = y_leg + R_23*z_body;
      double z_corner = z_leg + R_33*z_body;

      double leg_height = z_leg - terrain.getGroundHeight(x_leg,y_leg);
      double corner_height = z_corner - 
        terrain.getGroundHeight(x_corner,y_corner);
      // std::cout << "x,y,z = (" << x_spatial << ", " << y_spatial << ", " << z_spatial << "), height = " <<
      //     z_spatial << " - " << terrain.getGroundHeight(x_spatial,y_spatial) << " = " << height << std::endl;

      // Always check for collision, only check reachability in stance
      if ((corner_height < H_MIN) || ((phase == STANCE) && 
        (leg_height > H_MAX))) {
        return false;
      }
    }
  }

  // Check the center of the underside of the robot
  double height = (s[2] + R_33*z_body) - terrain.getGroundHeight(s[0] + 
    R_13*z_body,s[1] + R_23*z_body);
  if (height < H_MIN)
    return false;


  return true;
}

bool isValidStateActionPair(State s, Action a, FastTerrainMap& terrain, 
  State &s_new, double& t_new)
{
  double t_s;
  double t_f;

  t_s = a[6];
  t_f = a[7];

    for (double t = 0; t <= t_s; t += KINEMATICS_RES)
  {
    State s_check = applyStance(s,a,t,terrain);

    if (isValidState(s_check, terrain, STANCE) == false)
    {
      s_new = applyStance(s,a,(1.0-BACKUP_RATIO)*t,terrain);
      // s_new = applyStance(s,a,(t - BACKUP_TIME));
      return false;
    } else {
      s_new = s_check;
      t_new = t;
    }
  }

  State s_takeoff = applyStance(s, a,terrain);

  for (double t = 0; t < t_f; t += KINEMATICS_RES)
  {
    State s_check = applyFlight(s_takeoff, t);

    if (isValidState(s_check, terrain, FLIGHT) == false)
      return false;
  }

  // Check the exact landing state
  State s_land = applyFlight(s_takeoff, t_f);
  if (isValidState(s_land, terrain, STANCE) == false)
  {
    return false;
  } else {
    s_new = s_land;
    t_new = t_s+t_f;
  }

  // If both stance and flight trajectories are entirely valid, return true
  return true;
}


bool isValidStateActionPair(State s, Action a, FastTerrainMap& terrain)
{
  State dummy_state;
  double dummy_time;
  return isValidStateActionPair(s, a, terrain, dummy_state, dummy_time);
}

bool isValidStateActionPairReverse(State s, Action a, FastTerrainMap& terrain, 
  State &s_new, double& t_new)
{
  double t_s;
  double t_f;

  t_s = a[6];
  t_f = a[7];

  for (double t = 0; t < t_f; t += KINEMATICS_RES)
  {
    State s_check = applyFlight(s, -t);

    if (isValidState(s_check, terrain, FLIGHT) == false)
      return false;
  }

  State s_takeoff = applyFlight(s, -t_f);

  for (double t = t_s; t >= 0; t -= KINEMATICS_RES)
  {
    State s_check = applyStanceReverse(s_takeoff,a,t,terrain);

    if (isValidState(s_check, terrain, STANCE) == false)
    {
      s_new = applyStance(s,a,t + BACKUP_RATIO*(t_s - t), terrain);
      return false;
    } else {
      s_new = s_check;
      t_new = t_s - t;
    }
  }

  // Check the exact starting state
  State s_start = applyStanceReverse(s_takeoff,a, terrain);
  if (isValidState(s_start, terrain, STANCE) == false)
  {
    return false;
  } else {
    s_new = s_start;
    t_new = t_s;
  }
  // If both stance and flight trajectories are entirely valid, return true
  return true;
}


bool isValidStateActionPairReverse(State s, Action a, FastTerrainMap& terrain)
{
  State dummy_state;
  double dummy_time;
  return isValidStateActionPairReverse(s, a, terrain, dummy_state, dummy_time);
}

}