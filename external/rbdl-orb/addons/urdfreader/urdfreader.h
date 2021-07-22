#ifndef RBDL_URDFREADER_H
#define RBDL_URDFREADER_H

#include <rbdl/rbdl_config.h>
#include <rbdl/rbdl_math.h>

namespace RigidBodyDynamics {

struct Model;

namespace Addons {
  /**
   * This function will load a URDF model from a file.
   *
   * @param filename: path to the URDF file
   * @param model: reference to the (loaded) multibody model
   * @param floating_pase: does the model use a floating base
   * @param verbose: information will be printed to the command window if this
   *                 is set to true
   */
  RBDL_ADDON_DLLAPI bool URDFReadFromFile (const char* filename, Model* model,
                                     bool floating_base, bool verbose = false);

  /**
   * This function will load a URDF model from a c string.
   *
   * @param model_xml_string: URDF data in form of a string
   * @param model: reference to the (loaded) multibody model
   * @param floating_pase: does the model use a floating base
   * @param verbose: information will be printed to the command window if this
   *                 is set to true
   */
  RBDL_ADDON_DLLAPI bool URDFReadFromString (const char* model_xml_string,
                                       Model* model, bool floating_base,
                                       bool verbose = false);
}

}

/* _RBDL_URDFREADER_H */
#endif
