#include <rbdl/rbdl.h>

#ifdef RBDL_BUILD_ADDON_LUAMODEL
#include <addons/luamodel/luamodel.h>
#endif

#ifdef RBDL_BUILD_ADDON_URDFREADER
#include <addons/urdfreader/urdfreader.h>
#endif

#include <string>
#include <ctype.h>

using namespace RigidBodyDynamics;
using namespace std;

bool rbdl_loadmodel (const char* filename, Model* model, bool floating_base=false, bool verbose=false) {
  string fname (filename);

  for (size_t i = 0; i < fname.size(); i++) {
    fname[i] = tolower(fname[i]);
  }

  bool result = false;
  if (fname.substr (fname.size() - 4, 4) == ".lua") {
#ifdef RBDL_BUILD_ADDON_LUAMODEL
    result = Addons::LuaModelReadFromFile (filename, model, verbose);  
#else
    cerr << "Error: RBDL Addon LuaModel not enabled!" << endl;
#endif
  } else if (fname.substr (fname.size() - 5, 5) == ".urdf") {
#ifdef RBDL_BUILD_ADDON_URDFREADER
    result = Addons::URDFReadFromFile (filename, model, floating_base, verbose);  
#else
    cerr << "Error: RBDL Addon URDFReader not enabled!" << endl;
#endif
  } else {
    cerr << "Error: Cannot identify model type from filename '" << filename << "'!" << endl;
  }

  return result;
}
