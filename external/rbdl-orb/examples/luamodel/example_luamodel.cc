/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#ifndef RBDL_BUILD_ADDON_LUAMODEL
#error "Error: RBDL addon LuaModel not enabled."
#endif

#include <rbdl/addons/luamodel/luamodel.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main (int argc, char* argv[]) {
  rbdl_check_api_version (RBDL_API_VERSION);

  Model* model = NULL;

  model = new Model();

  if (argc != 2) {
    std::cerr << "Error: Invalid number of arguments." << std::endl;
    std::cerr << "usage: " << argv[0] << " <model.lua>" << std::endl;
    exit(-1);
  }

  if (!Addons::LuaModelReadFromFile (argv[1], model, false)) {
    std::cerr << "Error loading model " << argv[1] << std::endl;
    abort();
  }

  std::cout << "Degree of freedom overview:" << std::endl;
  std::cout << Utils::GetModelDOFOverview(*model);

  std::cout << "Model Hierarchy:" << std::endl;
  std::cout << Utils::GetModelHierarchy(*model);

  VectorNd Q = VectorNd::Zero (model->dof_count);
  VectorNd QDot = VectorNd::Zero (model->dof_count);
  VectorNd Tau = VectorNd::Zero (model->dof_count);
  VectorNd QDDot = VectorNd::Zero (model->dof_count);

  std::cout << "Forward Dynamics with q, qdot, tau set to zero:" << std::endl;
  ForwardDynamics (*model, Q, QDot, Tau, QDDot);

  std::cout << QDDot.transpose() << std::endl;

  delete model;

  return 0;
}

