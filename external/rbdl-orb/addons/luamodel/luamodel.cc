#include "rbdl/rbdl.h"
#include "rbdl/rbdl_errors.h"
#include "luamodel.h"

#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <vector>
#include <stdlib.h>

#include <iostream>
#include <iomanip>
#include <map>

#include "luatables.h"
#include "luatypes.h"

extern "C"
{
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
}

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

#ifdef RBDL_BUILD_ADDON_MUSCLE
#include "../muscle/Millard2016TorqueMuscle.h"
using namespace RigidBodyDynamics::Addons::Muscle;
#endif


namespace RigidBodyDynamics
{

namespace Addons
{

//==============================================================================
void appendEnumToFileStream(
    std::ofstream &updFileStream,
    const std::string &enumName,
    const std::vector<std::string> &enumEntries,
    unsigned int startingIndex,
    const std::string &entryPrefix)
{

  updFileStream << "\n";
  updFileStream << "enum " << enumName << "\n{\n";
  unsigned int maxLineLength=0;
  unsigned int numChars;
  for(unsigned int i=0; i<enumEntries.size();++i){
    if(maxLineLength < unsigned(enumEntries[i].size())){
      maxLineLength = unsigned(enumEntries[i].size());
    }
  }
  maxLineLength += unsigned(2+entryPrefix.size());
  unsigned int startingIndexLength =
      unsigned(int(std::to_string(startingIndex).size()));


  for(unsigned int i=0; i<enumEntries.size();++i){
    updFileStream << "  " << entryPrefix << enumEntries[i];
    numChars = maxLineLength;
    if(i==0){
      numChars -= unsigned( startingIndexLength + 1);
    }
    for(unsigned int j=unsigned(int(entryPrefix.size()+enumEntries[i].size()));
        j<numChars;++j){
      updFileStream << " ";
    }
    if(i==0){
      updFileStream << "=" << startingIndex;
    }
    if(i < (enumEntries.size()-1)){
      updFileStream << ",";
    }
    updFileStream << "\n";
  }
  updFileStream << "};\n";
}
//==============================================================================
void appendEnumNameIndexStructToFileStream(
    std::ofstream &updFileStream,
    const std::string &mapName,
    const std::string &enumName,
    const std::vector<std::string> &enumEntries,
    const std::string &entryPrefix,
    const std::vector< unsigned int> &indexEntries)
{

  updFileStream << "\n";
  updFileStream << "struct " << mapName << "Entry {\n";
  updFileStream << "  " << enumName << " id;\n";
  updFileStream << "  const char* name;\n";
  updFileStream << "  unsigned int index;\n";
  updFileStream << "};\n\n";
  updFileStream << "static const " << mapName << "Entry " << mapName << "[] = {\n";

  unsigned int longLine=0;
  unsigned int indexPadding =1;
  unsigned int indexPaddingTemp=1;
  for(unsigned int i=0;i<enumEntries.size();++i){
    if(longLine < unsigned(int(enumEntries[i].size()))){
      longLine = unsigned(int(enumEntries[i].size()));
    }
    if(indexEntries[i] > 9){
      indexPaddingTemp = 1+
          unsigned(int(std::floor( log10(double(indexEntries[i])))));
      if(indexPaddingTemp > indexPadding){
        indexPadding = indexPaddingTemp;
      }
    }

  }
  longLine += unsigned(entryPrefix.size()+2);
  unsigned int numChars=0;

  for(unsigned int i=0; i<enumEntries.size();++i){
    updFileStream << "  { " << entryPrefix << enumEntries[i];
    numChars=longLine;
    for(unsigned int j=unsigned(int(enumEntries[i].size())); j<numChars;++j){
      updFileStream << " ";
    }
    updFileStream << ", " << "\"" << entryPrefix << enumEntries[i] << "\"";

    numChars= longLine;
    for(unsigned int j=unsigned(int(enumEntries[i].size())); j<numChars;++j){
      updFileStream << " ";
    }
    updFileStream << ",";

    numChars = indexPadding;
    if(indexEntries[i] > 9){
      numChars-=unsigned(int(std::floor( log10(double(indexEntries[i])))));
    }
    numChars -= 1;
    for(unsigned int j=0; j<numChars;++j){
      updFileStream << " ";
    }

    updFileStream  << indexEntries[i];
    updFileStream << " }";
    if(i < (enumEntries.size()-1)){
      updFileStream << ",";
    }
    updFileStream << "\n";
  }
  updFileStream << "};\n";

}

//==============================================================================

void appendEnumStructToFileStream(
    std::ofstream &updFileStream,
    const std::string &mapName,
    const std::vector < std::string > &enumTypeAndFieldNames,
    const std::vector< std::vector<std::string> > &enumEntries,
    const std::vector< std::string > &entryPrefix,
    const std::vector< std::string > &indexTypeAndFieldName,
    const std::vector< std::vector< unsigned int > > &indexEntries)
{

  updFileStream << "\n";
  updFileStream << "struct " << mapName << "Entry {\n";
  for(unsigned int i=0; i<enumTypeAndFieldNames.size(); ++i){
    updFileStream << "  " << enumTypeAndFieldNames[i] << ";\n";
  }
  for(unsigned int i=0; i<indexTypeAndFieldName.size();++i){
    updFileStream << "  " << indexTypeAndFieldName[i] <<";\n";
  }
  updFileStream << "};\n\n";
  updFileStream << "static const " << mapName << "Entry " << mapName << "[] = {\n";

  std::vector < unsigned int > enumNamePadding;
  enumNamePadding.resize(enumTypeAndFieldNames.size());
  unsigned int enumNamePaddingTemp =1;
  for(unsigned int i=0; i<enumTypeAndFieldNames.size();++i){
    enumNamePadding[i]=enumNamePaddingTemp;
  }

  std::vector < unsigned int > indexPadding;
  indexPadding.resize( indexTypeAndFieldName.size() );
  unsigned int indexPaddingTemp=1;
  for(unsigned int j=0; j<indexTypeAndFieldName.size();++j){
    indexPadding[j]=indexPaddingTemp;
  }

  for(unsigned int i=0;i<enumEntries.size();++i){
    for(unsigned int j=0; j<enumEntries[0].size();++j){
      if(enumNamePadding[j] < unsigned(int(enumEntries[i][j].size()))){
        enumNamePadding[j] = unsigned(int(enumEntries[i][j].size()));
      }
    }
    if(indexEntries.size()>0){
      for(unsigned int j=0; j< indexEntries[0].size(); ++j){
        if(indexEntries[i][j]>9){
          indexPaddingTemp = 1+
              unsigned(int(std::floor( log10(double(indexEntries[i][j])))));
          if(indexPaddingTemp > indexPadding[j]){
            indexPadding[j] = indexPaddingTemp;
          }
        }
      }
    }
  }

  for(unsigned int i=0; i<enumEntries[0].size();++i){
    enumNamePadding[i]+=2;
  }

  unsigned int numChars=0;

  for(unsigned int i=0; i<enumEntries.size();++i){

    for(unsigned int j=0; j<enumEntries[i].size();++j){
      if(j==0){
        updFileStream << "  { " ;
      }
      updFileStream << entryPrefix[j] << enumEntries[i][j];
      numChars=enumNamePadding[j];
      for(unsigned int z=unsigned(int(enumEntries[i][j].size()));z<numChars;++z)
      {
        updFileStream << " ";
      }
      if(j < enumEntries[i].size()-1){
        updFileStream << ",";
      }
    }

    if(indexEntries.size()>0){
      updFileStream << ",";
      for(unsigned int j=0; j<indexEntries[i].size();++j){
        numChars = indexPadding[j];
        if(indexEntries[i][j] > 9){
          numChars-=unsigned(int(std::floor( log10(double(indexEntries[i][j])))));
        }
        numChars -= 1;
        for(unsigned int z=0; z<numChars;++z){
          updFileStream << " ";
        }

        updFileStream  << indexEntries[i][j];
        if(j < indexEntries[i].size()-1){
          updFileStream   << ",";
        }
      }
    }
    updFileStream << " }";
    if(i < (enumEntries.size()-1)){
      updFileStream << ",";
    }
    updFileStream << "\n";
  }
  updFileStream << "};\n";
}

//==============================================================================
bool LuaModelReadFromTable (LuaTable &model_table, Model *model, bool verbose);

//==============================================================================
bool LuaModelReadConstraintsFromTable (
  LuaTable &model_table,
  const Model *model,
  std::vector<ConstraintSet>& constraint_sets,
  const std::vector<std::string>& constraint_set_names,
  bool verbose
);

typedef map<string, unsigned int> StringIntMap;
StringIntMap body_table_id_map;

//==============================================================================
RBDL_ADDON_DLLAPI
bool LuaModelReadFromLuaState (lua_State* L, Model* model, bool verbose)
{
  assert (model);

  LuaTable model_table = LuaTable::fromLuaState (L);

  return LuaModelReadFromTable (model_table, model, verbose);
}

//==============================================================================
bool LuaModelReadLocalFrames (
      LuaTable &model_table,
      const RigidBodyDynamics::Model *model,
      std::vector<LocalFrame>& upd_local_frame_set,
      bool verbose);

RBDL_ADDON_DLLAPI
bool LuaModelReadLocalFrames (
      const char* filename,
      const RigidBodyDynamics::Model *model,
      std::vector<LocalFrame>& upd_local_frame_set,
      bool verbose)
{
  LuaTable model_table       = LuaTable::fromFile (filename);
  return LuaModelReadLocalFrames(model_table,model,upd_local_frame_set,verbose);
}

//==============================================================================
bool LuaModelReadPoints (
      LuaTable &model_table,
      const RigidBodyDynamics::Model *model,
      std::vector<Point>& upd_point_set,
      bool verbose);

RBDL_ADDON_DLLAPI
bool LuaModelReadPoints (
      const char* filename,
      const RigidBodyDynamics::Model *model,
      std::vector<Point>& upd_point_set,
      bool verbose)
{
  LuaTable model_table       = LuaTable::fromFile (filename);
  return LuaModelReadPoints(model_table,model,upd_point_set,verbose);
}

//==============================================================================
RBDL_ADDON_DLLAPI
bool LuaModelReadFromFile (const char* filename, Model* upd_model, bool verbose)
{
  if(!upd_model) {
    throw Errors::RBDLError("Model not provided.");
  }

  LuaTable model_table = LuaTable::fromFile (filename);
  return LuaModelReadFromTable (model_table, upd_model, verbose);
}

//==============================================================================
RBDL_ADDON_DLLAPI
std::vector<std::string> LuaModelGetConstraintSetNames(const char* filename)
{
  std::vector<std::string> result;

  LuaTable model_table = LuaTable::fromFile (filename);

  std::vector<LuaKey> constraint_keys;
  if (model_table["constraint_sets"].exists()) {
    constraint_keys = model_table["constraint_sets"].keys();
  }

  if (constraint_keys.size() == 0) {
    return result;
  }

  for (size_t ci = 0; ci < constraint_keys.size(); ++ci) {
    if (constraint_keys[ci].type != LuaKey::String) {
      throw Errors::RBDLFileParseError(
            "Invalid constraint found in model.constraint_sets: "
            "no constraint set name was specified!");
    }

    result.push_back(constraint_keys[ci].string_value);
  }

  return result;
}

//==============================================================================
RBDL_ADDON_DLLAPI
bool LuaModelGetConstraintSetPhases(const char* filename,
    const std::vector<std::string> &constraint_set_names,
    std::vector< unsigned int > &constraint_set_phases)
{
  LuaTable     luaTable = LuaTable::fromFile (filename);
  unsigned int phases = luaTable["constraint_set_phases"].length();
  constraint_set_phases.resize(phases);
  bool found = false;
  std::string phaseName;

  for(unsigned int i=1; i<phases; ++i){
    phaseName = luaTable["constraint_set_phases"][i].get<std::string>();
    found = false;
    for(unsigned int j=0; j<constraint_set_names.size();++j){
      if(constraint_set_names[j] == phaseName){
        constraint_set_phases[i-1] = j;
        found=true;
      }
    }
    if(found==false){
      throw Errors::RBDLFileParseError(
            "constraint_phases lists the constraint_set name " + phaseName
            + " but this does not appear in the list of constraint_sets. ");
    }
  }
  return true;
}

//==============================================================================
RBDL_ADDON_DLLAPI
bool LuaModelReadFromFileWithConstraints (
  const char* filename,
  Model* model,
  std::vector<ConstraintSet>& constraint_sets,
  const std::vector<std::string>& constraint_set_names,
  bool verbose
)
{
  if(!model) {
    throw Errors::RBDLError("Model not provided.");
  }
  if(constraint_sets.size() != constraint_set_names.size()) {
    throw Errors::RBDLFileParseError(
          "Number of constraint sets different from"
          " the number of constraint set names.");
  }

  LuaTable model_table = LuaTable::fromFile (filename);
  bool modelLoaded = LuaModelReadFromTable (model_table, model, verbose);
  bool constraintsLoaded = LuaModelReadConstraintsFromTable (model_table, model
                           , constraint_sets, constraint_set_names, verbose);
  for(size_t i = 0; i < constraint_sets.size(); ++i) {
    constraint_sets[i].Bind(*model);
  }

  return modelLoaded && constraintsLoaded;
}

//==============================================================================
bool LuaModelReadFromTable (LuaTable &model_table, Model* model, bool verbose)
{
  if (model_table["gravity"].exists()) {
    model->gravity = model_table["gravity"].get<Vector3d>();

    if (verbose) {
      cout << "gravity = " << model->gravity.transpose() << endl;
    }
  }

  int frame_count = model_table["frames"].length();

  body_table_id_map["ROOT"] = 0;

  for (int i = 1; i <= frame_count; i++) {
    if (!model_table["frames"][i]["parent"].exists()) {
      throw Errors::RBDLError("Parent not defined for frame ");
    }

    string body_name = model_table["frames"][i]["name"].getDefault<string>("");
    string parent_name = model_table["frames"][i]["parent"].get<string>();
    unsigned int parent_id = body_table_id_map[parent_name];

    SpatialTransform joint_frame
      = model_table["frames"][i]["joint_frame"].getDefault(SpatialTransform());
    Joint joint
      = model_table["frames"][i]["joint"].getDefault(Joint(JointTypeFixed));
    Body body = model_table["frames"][i]["body"].getDefault<Body>(Body());

    unsigned int body_id
      = model->AddBody (parent_id, joint_frame, joint, body, body_name);
    body_table_id_map[body_name] = body_id;

    if (verbose) {
      cout << "==== Added Body ====" << endl;
      cout << "  body_name  : " << body_name << endl;
      cout << "  body id	: " << body_id << endl;
      cout << "  parent_id  : " << parent_id << endl;
      cout << "  joint dofs : " << joint.mDoFCount << endl;
      for (unsigned int j = 0; j < joint.mDoFCount; j++) {
        cout << "	" << j << ": " << joint.mJointAxes[j].transpose() << endl;
      }
      cout << "  joint_frame: " << joint_frame << endl;
    }
  }

  return true;
}

//==============================================================================
bool LuaModelReadConstraintsFromTable (
  LuaTable &model_table,
  const Model *model,
  std::vector<ConstraintSet>& constraint_sets,
  const std::vector<std::string>& constraint_set_names,
  bool verbose
)
{
  std::string conName;

  std::vector< Vector3d > normalSets;
  MatrixNd normalSetsMatrix;
  Vector3d normal;

  MatrixNd axisSetsMatrix;
  SpatialVector axis;
  std::vector< SpatialVector > axisSets;

  std::vector< Point > pointSet;
  bool pointSetLoaded = LuaModelReadPoints(model_table,model,pointSet,verbose);

  std::vector< LocalFrame > localFrameSet;
  bool localFramesLoaded =
      LuaModelReadLocalFrames(model_table,model,localFrameSet,verbose);

  for(size_t i = 0; i < constraint_set_names.size(); ++i) {
    conName = constraint_set_names[i];
    if (verbose) {
      std::cout << "==== Constraint Set: " << conName << std::endl;
    }

    if(!model_table["constraint_sets"][conName.c_str()]
        .exists()) {
      ostringstream errormsg;
      errormsg << "Constraint set not existing: " << conName << "." << endl;
      throw Errors::RBDLFileParseError(errormsg.str());
    }

    size_t num_constraints = model_table["constraint_sets"]
                             [conName.c_str()]
                             .length();

    for(size_t ci = 0; ci < num_constraints; ++ci) {
      if (verbose) {
        std::cout << "== Constraint " << ci << "/" << num_constraints
                  << " ==" << std::endl;
      }

      if(!model_table["constraint_sets"]
          [conName.c_str()][ci + 1]["constraint_type"].exists()) {
        throw Errors::RBDLFileParseError("constraint_type not specified.\n");
      }

      string constraintType = model_table["constraint_sets"]
                              [conName.c_str()][ci + 1]["constraint_type"]
                              .getDefault<string>("");
      std::string constraint_name =
        model_table["constraint_sets"][conName.c_str()]
        [ci + 1]["name"].getDefault<string>("");

      bool enable_stabilization =
        model_table["constraint_sets"][conName.c_str()][ci + 1]
        ["enable_stabilization"].getDefault<bool>(false);
      double stabilization_parameter = 0.1;

      if (enable_stabilization) {
        stabilization_parameter =
          model_table["constraint_sets"][conName.c_str()][ci + 1]
          ["stabilization_parameter"].getDefault<double>(0.1);
        if (stabilization_parameter <= 0.0) {
          std::stringstream errormsg;
          errormsg  << "Invalid stabilization parameter: "
                    << stabilization_parameter
                    << " must be > 0.0" << std::endl;
          throw Errors::RBDLFileParseError(errormsg.str());
        }
      }


      //========================================================================
      //Contact
      //========================================================================
      if(constraintType == "contact") {

        unsigned int constraint_user_id =
            std::numeric_limits<unsigned int>::max();

        if(model_table["constraint_sets"][conName.c_str()][ci + 1]
                      ["id"].exists()) {
          constraint_user_id = unsigned(int(
            model_table["constraint_sets"][conName.c_str()][ci + 1]
                       ["id"].getDefault<double>(0.)));
        }

        //Go get the body id and the local coordinates of the point:
        //    If a named point is given, go through the point set
        //    Otherwise look for the explicit fields
        unsigned int bodyId;
        Vector3d bodyPoint;

        if(model_table["constraint_sets"][conName.c_str()][ci+1]
           ["point_name"].exists()){
          std::string pointName = model_table["constraint_sets"]
              [conName.c_str()][ci+1]["point_name"].getDefault<string>("");
          bool pointFound = false;
          unsigned int pi=0;
          while(pi < pointSet.size() && pointFound == false){
            if(std::strcmp(pointSet[pi].name.c_str(),pointName.c_str())==0){
              pointFound = true;
              bodyId = pointSet[pi].body_id;
              bodyPoint = pointSet[pi].point_local;
            }
            ++pi;
          }
          if(pointFound == false){
            ostringstream errormsg;
            errormsg << "Could not find a point with the name: " << pointName
                     << " which is used in constraint set " << constraint_name
                     << endl;
            throw Errors::RBDLFileParseError(errormsg.str());
          }
        }else{
          if(!model_table["constraint_sets"][conName.c_str()]
              [ci + 1]["body"].exists()) {
            throw Errors::RBDLFileParseError("body not specified.\n");
          }
          bodyId = model->GetBodyId(model_table["constraint_sets"]
                                               [conName.c_str()][ci + 1]["body"]
                                               .getDefault<string>("").c_str());

          bodyPoint = model_table["constraint_sets"]
                               [conName.c_str()][ci + 1]
                               ["point"].getDefault<Vector3d>(Vector3d::Zero());
        }

        normalSets.resize(0);
        normalSetsMatrix.resize(1,1);

        if(model_table["constraint_sets"][conName.c_str()][ci + 1]
            ["normal_sets"].exists()) {

          normalSetsMatrix =
            model_table["constraint_sets"][conName.c_str()]
            [ci + 1]["normal_sets"].getDefault< MatrixNd >(MatrixNd::Zero(1,1));

          if(normalSetsMatrix.cols() != 3 ) {
            std::ostringstream errormsg;
            errormsg << "The normal_sets field must be m x 3, the one read for "
                     << conName.c_str() << " has an normal_sets of size "
                     << normalSetsMatrix.rows() << " x "
                     << normalSetsMatrix.cols()
                     << ". In addition the normal_sets field should resemble:"
                     << endl;
            errormsg << "  normal_sets = {{1.,0.,0.,}, " << endl;
            errormsg << "                 {0.,1.,0.,},}, " << endl;
            throw Errors::RBDLFileParseError(errormsg.str());
          }


          for(unsigned int r=0; r<normalSetsMatrix.rows(); ++r) {
            for(unsigned int c=0; c<normalSetsMatrix.cols(); ++c) {
              normal[c] = normalSetsMatrix(r,c);
            }
            normalSets.push_back(normal);
          }

        } else if(model_table["constraint_sets"][conName.c_str()]
                  [ci + 1]["normal"].exists()) {

          normal = model_table["constraint_sets"]
                   [conName.c_str()][ci + 1]
                   ["normal"].getDefault<Vector3d>(Vector3d::Zero());
          normalSets.push_back(normal);

        } else {
          std::ostringstream errormsg;
          errormsg << "The ContactConstraint must have either normal_sets field "
                   "(which is a m x 3 matrix) or an normal field. Neither of "
                   "these fields was found in "
                   << conName.c_str() << endl;
          throw Errors::RBDLFileParseError(errormsg.str());
        }

        std::string contactName = model_table["constraint_sets"]
                                  [conName.c_str()][ci + 1]
                                  ["name"].getDefault<string>("").c_str();

        for(unsigned int c=0; c<normalSets.size(); ++c) {
          constraint_sets[i].AddContactConstraint(bodyId,
                                                  bodyPoint,
                                                  normalSets[c],
                                                  contactName.c_str(),
                                                  constraint_user_id);
        }

        if(verbose) {
          cout  << "  type = contact" << endl;
          cout  << "  name = " << constraint_name << std::endl;
          cout  << "  body = "
                << model->GetBodyName(bodyId) << endl;
          cout  << "  body point = "
                << bodyPoint.transpose()
                << endl;
          cout  << "  world normal = " << endl;
          for(unsigned int c=0; c<normalSets.size(); ++c) {
            cout << normalSets[c].transpose() << endl;
          }
          cout << "  normal acceleration = DEPRECATED::IGNORED" << endl;
        }

        //========================================================================
        //Loop
        //========================================================================
      } else if(constraintType == "loop") {

        unsigned int constraint_user_id=std::numeric_limits<unsigned int>::max();
        if(model_table["constraint_sets"][conName.c_str()][ci + 1]
            ["id"].exists()) {
          constraint_user_id =
              unsigned(int(model_table["constraint_sets"][conName.c_str()]
                           [ci + 1]["id"].getDefault<double>(0.)));
        }

        //Get the local frames that this constraint will be applied to
        // If named local frames have been given, use them
        // Otherwise look for the individual fields
        unsigned int idPredecessor;
        unsigned int idSuccessor;
        SpatialTransform Xp;
        SpatialTransform Xs;

        if(model_table["constraint_sets"]
           [conName.c_str()][ci + 1]["predecessor_local_frame"].exists()){

          std::string localFrameName = model_table["constraint_sets"]
              [conName.c_str()][ci + 1]["predecessor_local_frame"]
              .getDefault<string>("");
          bool frameFound = false;
          unsigned int fi=0;
          while(fi < localFrameSet.size() && frameFound == false){
            if(std::strcmp(localFrameSet[fi].name.c_str(),
                           localFrameName.c_str())==0){
              frameFound = true;
              idPredecessor = localFrameSet[fi].body_id;
              Xp.r = localFrameSet[fi].r;
              Xp.E = localFrameSet[fi].E;
            }
            ++fi;
          }
          if(frameFound == false){
            ostringstream errormsg;
            errormsg << "Could not find a local frame with the name: "
                     << localFrameName
                     << " which is used in constraint set " << constraint_name
                     << endl;
            throw Errors::RBDLFileParseError(errormsg.str());
          }
        }else{
          if(!model_table["constraint_sets"][conName.c_str()]
              [ci + 1]["predecessor_body"].exists()) {
            throw Errors::RBDLFileParseError(
                  "predecessor_body not specified.\n");
          }

          idPredecessor =
            model->GetBodyId(model_table["constraint_sets"]
                             [conName.c_str()][ci + 1]["predecessor_body"]
                             .getDefault<string>("").c_str());
          Xp = model_table["constraint_sets"][conName.c_str()]
                [ci + 1]["predecessor_transform"]
                .getDefault<SpatialTransform>(SpatialTransform());
        }

        if(model_table["constraint_sets"]
           [conName.c_str()][ci + 1]["successor_local_frame"].exists()){

          std::string localFrameName = model_table["constraint_sets"]
              [conName.c_str()][ci + 1]["successor_local_frame"]
              .getDefault<string>("");
          bool frameFound = false;
          unsigned int fi=0;
          while(fi < localFrameSet.size() && frameFound == false){
            if(std::strcmp(localFrameSet[fi].name.c_str(),
                           localFrameName.c_str())==0){
              frameFound = true;
              idSuccessor = localFrameSet[fi].body_id;
              Xs.r = localFrameSet[fi].r;
              Xs.E = localFrameSet[fi].E;
            }
            ++fi;
          }
          if(frameFound == false){
            ostringstream errormsg;
            errormsg << "Could not find a local frame with the name: "
                     << localFrameName
                     << " which is used in constraint set " << constraint_name
                     << endl;
            throw Errors::RBDLFileParseError(errormsg.str());
          }

        }else{

          if(!model_table["constraint_sets"][conName.c_str()]
              [ci + 1]["successor_body"].exists()) {
            throw Errors::RBDLFileParseError("successor_body not specified.\n");
          }

          idSuccessor =
            model->GetBodyId(model_table["constraint_sets"]
                             [conName.c_str()][ci + 1]["successor_body"]
                             .getDefault<string>("").c_str());


          Xs =  model_table["constraint_sets"][conName.c_str()]
                  [ci + 1]["successor_transform"]
                  .getDefault<SpatialTransform>(SpatialTransform());
        }



        // Add the loop constraint as a non-stablized constraint and compute
        // and set the actual stabilization cofficients for the Baumgarte
        // stabilization afterwards if enabled.

        axisSetsMatrix.resize(1,1);
        axisSets.resize(0);
        if(model_table["constraint_sets"][conName.c_str()][ci + 1]
            ["axis_sets"].exists()) {
          axisSetsMatrix =
            model_table["constraint_sets"][conName.c_str()][ci + 1]
            ["axis_sets"].getDefault< MatrixNd >( MatrixNd::Zero(1,1));

          if(axisSetsMatrix.cols() != 6 ) {
            std::stringstream errormsg;
            errormsg  << "The axis_sets field must be m x 6, the one read for "
                      << conName.c_str() << " has an axis_sets of size "
                      << axisSetsMatrix.rows() << " x " << axisSetsMatrix.cols()
                      << ". In addition the axis_sets field should resemble:"
                      << endl;
            errormsg  << "  axis_sets = {{0.,0.,0.,1.,0.,0.,}, " <<endl;
            errormsg  << "               {0.,0.,0.,0.,1.,0.,},}, " <<endl;
            throw Errors::RBDLFileParseError(errormsg.str());
          }

          for(unsigned int r=0; r<axisSetsMatrix.rows(); ++r) {
            for(unsigned int c=0; c<axisSetsMatrix.cols(); ++c) {
              axis[c] = axisSetsMatrix(r,c);
            }
            axisSets.push_back(axis);
          }

        } else if(model_table["constraint_sets"][conName.c_str()][ci + 1]
                  ["axis"].exists()) {
          axis = model_table["constraint_sets"][conName.c_str()][ci + 1]
                 ["axis"].getDefault< SpatialVector >( SpatialVector::Zero());

          axisSets.push_back(axis);

        } else {
          std::stringstream errormsg;
          errormsg  << "The LoopConstraint must have either axis_sets field "
                    "(which is a m x 6 matrix) or an axis field. Neither of "
                    "these fields was found in "
                    << conName.c_str() << endl;
          throw Errors::RBDLFileParseError(errormsg.str());
        }

        unsigned int constraint_id;
        for(unsigned int r=0; r<axisSets.size(); ++r) {
          constraint_id = constraint_sets[i].AddLoopConstraint(
                            idPredecessor
                            , idSuccessor
                            , Xp
                            , Xs
                            , axisSets[r]
                            , enable_stabilization
                            , stabilization_parameter
                            , constraint_name.c_str()
                            , constraint_user_id);
        }

        if(verbose) {
          cout << "  type = loop" << endl;
          cout << "  name = " << constraint_name << std::endl;
          cout << "  predecessor body = "
               << model->GetBodyName(idPredecessor)<< endl;
          cout << "  successor body = "
               << model->GetBodyName(idSuccessor) << endl;
          cout << "  predecessor body transform = " << endl
               << Xp << endl;
          cout << "  successor body transform = " << endl
               << Xs << endl;
          cout << "  constraint axis (in predecessor frame) = " << endl;
          for(unsigned int c=0; c<axisSets.size(); ++c) {
            cout << axisSets[c].transpose() << endl;
          }
          cout << "  enable_stabilization = " << enable_stabilization
               << endl;
          if (enable_stabilization) {
            cout << "  stabilization_parameter = " << stabilization_parameter
                 << endl;
          }
          cout << "  constraint name = "
               << constraint_name.c_str() << endl;
        }
      } else {
        ostringstream errormsg;
        errormsg << "Invalid constraint type: " << constraintType << endl;
        throw Errors::RBDLFileParseError(errormsg.str());
      }
    }
  }

  return true;
}

//==============================================================================

bool LuaModelReadMotionCaptureMarkers (
      const char* filename,
      const RigidBodyDynamics::Model *model,
      std::vector<MotionCaptureMarker>& upd_marker_set,
      bool verbose)
{

  LuaTable luaTable       = LuaTable::fromFile (filename);
  upd_marker_set.clear();

  if(luaTable["frames"].exists()){
    unsigned int frameCount = luaTable["frames"].length();
    std::vector<LuaKey> marker_keys;
    MotionCaptureMarker marker;
    std::string body_name;
    unsigned int body_id;
    for(unsigned int i=1; i<frameCount; ++i){
      if(luaTable["frames"][i]["markers"].exists()){

        body_name = luaTable["frames"][i]["name"].getDefault<string>("");
        body_id = model->GetBodyId(body_name.c_str());
        marker_keys = luaTable["frames"][i]["markers"].keys();

        for(unsigned int j=0; j < marker_keys.size(); ++j){
          if (marker_keys[j].type != LuaKey::String) {
            throw Errors::RBDLFileParseError(
                    "Invalid marker found: missing name!");
          }
          marker.name      = marker_keys[j].string_value;
          marker.body_name = body_name;
          marker.body_id   = body_id;
          marker.point_local = luaTable["frames"][i]["markers"][marker.name.c_str()]
                              .getDefault<Vector3d>(Vector3d::Zero());
          upd_marker_set.push_back(marker);
        }
      }
    }
  }

  return true;
}
//==============================================================================
bool LuaModelReadLocalFrames (
      LuaTable &model_table,
      const RigidBodyDynamics::Model *model,
      std::vector<LocalFrame>& upd_local_frame_set,
      bool verbose)
{
  //LuaTable luaTable       = LuaTable::fromFile (filename);
  upd_local_frame_set.clear();
  unsigned int localFrameCount =
      unsigned(int(model_table["local_frames"].length()));

  if(localFrameCount > 0){
    upd_local_frame_set.resize(localFrameCount);
    LocalFrame localFrame;

    for (unsigned int i = 1; i <= localFrameCount; ++i) {

      localFrame = model_table["local_frames"][signed(i)];

      localFrame.body_id     = model->GetBodyId (localFrame.body_name.c_str());
      upd_local_frame_set[i-1] = localFrame;

      if (verbose) {
        cout  << "LocalFrame '" << upd_local_frame_set[i-1].name
              << "' (name = "   << upd_local_frame_set[i-1].name << ")" << endl;
        cout  << "  body        = " << upd_local_frame_set[i-1].body_name
              << " (id = " << upd_local_frame_set[i-1].body_id << ")" << endl;
        cout  << "  r  = '" << upd_local_frame_set[i-1].r.transpose() << endl;
        cout  << "  E  = '" << upd_local_frame_set[i-1].E << endl;
      }
    }
  }
  return true;
}

//==============================================================================
bool LuaModelReadPoints (
      LuaTable &model_table,
      const RigidBodyDynamics::Model *model,
      std::vector<Point>& upd_point_set,
      bool verbose)
{

  upd_point_set.clear();
  unsigned int pointCount = unsigned(int(model_table["points"].length()));

  if(pointCount > 0){
    upd_point_set.resize(pointCount);
    Point point;

    for (unsigned int i = 1; i <= pointCount; ++i) {

      point = model_table["points"][i];

      point.body_id   = model->GetBodyId (point.body_name.c_str());
      upd_point_set[i-1]   = point;

      if (verbose) {
        cout  << "Point '"           << upd_point_set[i-1].name
              << "' (PointName = "   << upd_point_set[i-1].name << ")"    << endl;
        cout  << "  body        = "  << upd_point_set[i-1].body_name
              << " (id = "           << upd_point_set[i-1].body_id << ")" << endl;
        cout  << "  point_local  = '"       << upd_point_set[i-1].point_local.transpose()
                                     << endl;
      }
    }
  }
  return true;
}

//==============================================================================
bool LuaModelReadHumanMetaData(
  const char* filename,
  HumanMetaData &human_meta_data,
  bool verbose)
{

  LuaTable luaTable = LuaTable::fromFile (filename);
  unsigned int subjectCount = luaTable["human_meta_data"].length();

  if(subjectCount != 1){
    ostringstream errormsg;
    errormsg << "human_meta_data should contain data "
             << "for only 1 subject but has data for "
             << subjectCount << endl;

    throw Errors::RBDLError(errormsg.str());
  }

  human_meta_data = luaTable["human_meta_data"][1];  

  return true;
}

//==============================================================================
#ifdef RBDL_BUILD_ADDON_MUSCLE
std::vector<unsigned int> getQIndex(
                  const RigidBodyDynamics::Model *model,
                  const char *childBodyName)
{

  unsigned int idChild = model->GetBodyId(childBodyName);
  unsigned int idJoint=idChild;
  while(idJoint > 1 && model->mBodies[idJoint-1].mIsVirtual){
    --idJoint;
  }

  std::vector<unsigned int> qIndex;

  unsigned int ccid;

  unsigned int id=idJoint;
  unsigned int dof = 0;

  while(id <= idChild){
    if(model->mJoints[id].mJointType == JointTypeCustom){
      ccid = model->mJoints[id].custom_joint_index;
      for(unsigned int i=0; i<model->mCustomJoints[ccid]->mDoFCount;++i){
        qIndex.push_back(model->mJoints[id].q_index+i);
      }
      dof = model->mCustomJoints[ccid]->mDoFCount;
    }else{
      for(unsigned int i=0; i<model->mJoints[id].mDoFCount;++i){
        qIndex.push_back(model->mJoints[id].q_index+i);
      }
      dof = model->mJoints[id].mDoFCount;
    }
    id += dof;
  }
  //To get the extra index for the quaternion joints
  for(id=idJoint;id<=idChild;++id){
    if(model->multdof3_w_index[id] > 0){
      qIndex.push_back(model->multdof3_w_index[id]);
    }
  }
  return qIndex;
}

//==============================================================================

unsigned int getMillard2016TorqueMuscleTypeId(std::string name)
{


  unsigned int i = 0;
  while (i != JointTorqueSet::Last) {
    if (name.find( std::string(JointTorqueSet.names[i]) )
        != std::string::npos ) {
      break;
    }
    ++i;
  }

  if (i == JointTorqueSet::Last) {
    cerr <<"Error: " << name << " does not contain the name of any registered"
         <<" muscle torque generator. For the full list look for the definition"
         <<" of JointTorqueSet::names[] in  "
         <<"addons/muscle/Millard2016TorqueMuscle.cc" << endl;
    assert(0);
    abort();
  }
  
  return i;

}

//==============================================================================
bool LuaModelReadMillard2016TorqueMuscleSets(
    const char* filename,
    const RigidBodyDynamics::Model *model,
    const HumanMetaData &human_meta_data,
    std::vector <Millard2016TorqueMuscle> &updMtgSet,
    std::vector <Millard2016TorqueMuscleConfig> &updMtgSetInfo,
    bool verbose)
{


  LuaTable     luaTable  = LuaTable::fromFile (filename);
  unsigned int mtgCount  = luaTable["millard2016_torque_muscles"].length();


  updMtgSet.resize(mtgCount);
  updMtgSetInfo.resize(mtgCount);
  Millard2016TorqueMuscleConfig mtgInfo;
  Millard2016TorqueMuscleConfig mtgInfoDefault;
  unsigned int id;

  for(unsigned int i = 1; i <= mtgCount; ++i){
    mtgInfo = luaTable["millard2016_torque_muscles"][i];
    id = i-unsigned(int(1));

    updMtgSetInfo[id] = mtgInfo;

    if (verbose) {
      if(i == 1){
        std::cout << "Millard2016TorqueMuscle" << std::endl;
        std::cout << std::setw(20) << "Name"
                  << std::setw(10)  << "Angle-Sign"
                  << std::setw(11)  << "Torque-Sign"
                  << std::endl;
      }
        std::cout   << std::setw(24) << mtgInfo.name
                    << std::setw(3)  << mtgInfo.angle_sign
                    << std::setw(3)  << mtgInfo.torque_sign
                    << std::endl;
    }

  }

  //Populate the subject information structure
  SubjectInformation subjectInfo;
  if (human_meta_data.gender == "male") {
    subjectInfo.gender = GenderSet::Male;
  }else if (human_meta_data.gender == "female") {
    subjectInfo.gender = GenderSet::Female;
  }else {
    cerr << "Unknown gender in subject metadata, " << 
      human_meta_data.gender << endl;
    abort();
  }
  
  if (human_meta_data.age_group == "Young18To25") {
    subjectInfo.ageGroup =
            AgeGroupSet::Young18To25;
  }else if (human_meta_data.age_group == "Middle55to65") {
    subjectInfo.ageGroup =
            AgeGroupSet::Middle55To65;
  }else if (human_meta_data.age_group == "SeniorOver65") {
    subjectInfo.ageGroup =
            AgeGroupSet::SeniorOver65;
  }else {
    cerr << "Unknown age group in subject metadata, " << 
      human_meta_data.age_group << endl;
    abort();
  }

  subjectInfo.heightInMeters  = human_meta_data.height;
  subjectInfo.massInKg        = human_meta_data.mass;


  DataSet::item mtgDataSet;
  SubjectInformation mtgSubjectInfo;

  for(unsigned int i = 0; i < mtgCount; ++i){
    mtgSubjectInfo = subjectInfo;

    //Get the data set for this MTG
    if(updMtgSetInfo[i].data_set == "Gymnast"){
      mtgDataSet = DataSet::Gymnast;
    }else if(updMtgSetInfo[i].data_set == "Anderson2007"){
      mtgDataSet = DataSet::Anderson2007;
    }else{
      cerr << "Error: the data_set entry for "
           << updMtgSetInfo[i].name << " is "
           << updMtgSetInfo[i].data_set
           << "which neither Gymnast nor Anderson2007.";
      assert(0);
      abort();
    }

    //Get the age group for this MTG
    if(updMtgSetInfo[i].age_group == "Young18To25"){
      mtgSubjectInfo.ageGroup = AgeGroupSet::Young18To25;
    }else if(updMtgSetInfo[i].age_group == "Middle55To65"){
      mtgSubjectInfo.ageGroup = AgeGroupSet::Middle55To65;
    }else if(updMtgSetInfo[i].age_group == "SeniorOver65"){
      mtgSubjectInfo.ageGroup = AgeGroupSet::SeniorOver65;
    }else{
      mtgSubjectInfo.ageGroup = subjectInfo.ageGroup;
    }


    //Get the gender for this MTG
    if (updMtgSetInfo[i].gender == "male") {
      mtgSubjectInfo.gender = GenderSet::Male;
    }else if (updMtgSetInfo[i].gender == "female") {
      mtgSubjectInfo.gender = GenderSet::Female;
    }else {
      mtgSubjectInfo.gender = subjectInfo.gender;
    }

    if(updMtgSetInfo[i].body != mtgInfoDefault.body){
      unsigned int joint_offset = 0;
      if(updMtgSetInfo[i].joint_index != mtgInfoDefault.joint_index){
        joint_offset =updMtgSetInfo[i].joint_index;
      }

      //Go get the index of the first joint between the child body (given)
      //and its parent.
      std::vector<unsigned int> qIndices =
              getQIndex(model, updMtgSetInfo[i].body.c_str());

      updMtgSetInfo[i].q_index = qIndices[0] + joint_offset;
      updMtgSetInfo[i].qdot_index  = updMtgSetInfo[i].q_index;
      updMtgSetInfo[i].force_index = updMtgSetInfo[i].q_index;

    }
    if(updMtgSetInfo[i].activation_index == mtgInfoDefault.activation_index){
      updMtgSetInfo[i].activation_index = i;
    }

    updMtgSet[i] = Millard2016TorqueMuscle(
                          mtgDataSet,
                          mtgSubjectInfo,
                      int(getMillard2016TorqueMuscleTypeId(updMtgSetInfo[i].name)),
                          updMtgSetInfo[i].joint_angle_offset,
                          updMtgSetInfo[i].angle_sign,
                          updMtgSetInfo[i].torque_sign,
                          updMtgSetInfo[i].name);

    //Parameters for manual adjustment
    if (!std::isnan(updMtgSetInfo[i].max_isometric_torque_scale)) {
        double fiso = updMtgSet[i].getMaximumActiveIsometricTorque();
        double updFiso = fiso*updMtgSetInfo[i].max_isometric_torque_scale;
        updMtgSet[i].setMaximumActiveIsometricTorque(updFiso);
    }

    if (!std::isnan(updMtgSetInfo[i].max_angular_velocity_scale)) {
        double omegaMax =
            updMtgSet[i].getMaximumConcentricJointAngularVelocity();
        double updOmegaMax =
            omegaMax*updMtgSetInfo[i].max_angular_velocity_scale;
        updMtgSet[i].setMaximumConcentricJointAngularVelocity(updOmegaMax);
    }


    if (!std::isnan(updMtgSetInfo[i].passive_element_damping_coeff)) {
        updMtgSet[i].setNormalizedDampingCoefficient(
                    updMtgSetInfo[i].passive_element_damping_coeff);
    }

    if (!std::isnan(updMtgSetInfo[i].passive_element_torque_scale)) {
        updMtgSet[i].setPassiveTorqueScale(
                    updMtgSetInfo[i].passive_element_torque_scale);
    }

    if (!std::isnan(updMtgSetInfo[i].passive_element_angle_offset)) {
        updMtgSet[i].setPassiveCurveAngleOffset(
                    updMtgSetInfo[i].passive_element_angle_offset);
    }

    //Basic fitting of the passive curves
    if (!std::isnan(updMtgSetInfo[i].fit_passive_torque_scale[0]) &&
        !std::isnan(updMtgSetInfo[i].fit_passive_torque_scale[1])) {
        updMtgSet[i].fitPassiveTorqueScale(
                    updMtgSetInfo[i].fit_passive_torque_scale[0],
                    updMtgSetInfo[i].fit_passive_torque_scale[1]);
    }
    if (!std::isnan(updMtgSetInfo[i].fit_passive_torque_offset[0]) &&
        !std::isnan(updMtgSetInfo[i].fit_passive_torque_offset[1])) {

        updMtgSet[i].fitPassiveCurveAngleOffset(
                    updMtgSetInfo[i].fit_passive_torque_offset[0],
                    updMtgSetInfo[i].fit_passive_torque_offset[1]);
    }

    //Fitting parameters from the fitting method

    if(!std::isnan(updMtgSetInfo[i].max_isometric_torque)){
        updMtgSet[i].setMaximumActiveIsometricTorque(
                    updMtgSetInfo[i].max_isometric_torque);
    }

    if(!std::isnan(updMtgSetInfo[i].max_angular_velocity)){
        updMtgSet[i].setMaximumConcentricJointAngularVelocity(
                    updMtgSetInfo[i].max_angular_velocity);
    }
    if(!std::isnan(updMtgSetInfo[i].active_torque_angle_blending)) {
        updMtgSet[i].setActiveTorqueAngleCurveBlendingVariable(
                    updMtgSetInfo[i].active_torque_angle_blending);
    }
    if(!std::isnan(updMtgSetInfo[i].passive_torque_angle_blending)) {
        updMtgSet[i].setPassiveTorqueAngleCurveBlendingVariable(
                    updMtgSetInfo[i].passive_torque_angle_blending);
    }
    if(!std::isnan(updMtgSetInfo[i].torque_velocity_blending)) {
        updMtgSet[i].setTorqueAngularVelocityCurveBlendingVariable(
                    updMtgSetInfo[i].torque_velocity_blending);
    }
    if(!std::isnan(updMtgSetInfo[i].active_torque_angle_scale)) {
        updMtgSet[i].setActiveTorqueAngleCurveAngleScaling(
                    updMtgSetInfo[i].active_torque_angle_scale);
    }


    
  }

  return true;
}
#endif
//==============================================================================
bool LuaModelAddHeaderGuards(const char* filename){

  //Extract the file name and capitalize it to make a custom headerguard
  std::string name;
  std::string headerFileName(filename);
  unsigned int idx0 = unsigned(int(headerFileName.find_last_of("/")));
  if(idx0 == headerFileName.size()){
    idx0 = unsigned(int(headerFileName.find_last_of("\\")));
  }
  if(idx0==headerFileName.size()){
    idx0=0;
  }else{
    ++idx0;
  }
  unsigned int idx1 = unsigned(int(headerFileName.find_last_of(".")));
  if(idx1 == headerFileName.size()){
    idx1 = unsigned(int(headerFileName.size()));
  }
  name = headerFileName.substr(idx0, (idx1-idx0));

  std::transform(name.begin(), name.end(), name.begin(), ::toupper);

  //Read in the entire header file
  ifstream headerFileInput(headerFileName.c_str()); //taking file as inputstream
  string headerFileText;
  ostringstream ss;
  if(headerFileInput) {
    ss << headerFileInput.rdbuf(); // reading data
    headerFileText = ss.str();
  }
  headerFileInput.close();

  ofstream headerFileOut(headerFileName.c_str(),std::ofstream::out);
  headerFileOut << "#ifndef " << name << "_MAP\n";
  headerFileOut << "#define " << name << "_MAP\n";
  headerFileOut << headerFileText << "//" << name << "_MAP\n" << "#endif\n";

  headerFileOut.flush();
  headerFileOut.close();

  return true;

}
//==============================================================================
void LuaModelGetCoordinateNames(
      const Model* model,
      std::vector< std::string >& qNames,
      std::vector< std::string >& qDotNames,
      std::vector< std::string >& tauNames)
{

  qNames.resize(model->q_size);
  qDotNames.resize(model->qdot_size);
  tauNames.resize(model->qdot_size);

  unsigned int q_index;
  unsigned int idChild;
  unsigned int dof;

  std::ostringstream ss;

  std::string jointType;
  std::string axisType;
  unsigned int axisIndex;
  unsigned int ccid;
  std::string childName;
  bool newBody=false;

  //Populate vectors that q, qdot, and tau
  for(unsigned int idJoint=1;idJoint < model->mJoints.size();++idJoint){

    //Get the first index in q associated with this joint
    q_index   = model->mJoints[idJoint].q_index;

    //Get the index of the child body to this joint.
    idChild=idJoint;
    while(idChild < model->mBodies.size()
          && model->mBodies[idChild].mIsVirtual){
      ++idChild;
    }


    //Get the name of the child body associated with this index
    for(map<string,unsigned int>::const_iterator it=model->mBodyNameMap.begin();
        it != model->mBodyNameMap.end(); ++it){
      if(it->second == idChild){
        childName = it->first;
      }
    }

    //Get the name of the joint type
    jointType = JointMap[ model->mJoints[idJoint].mJointType].abbr;
    if(jointType.length()>0){
      jointType.append("_");
    }
    childName.append("_");
    //If this is a canonical joint, go get the index of the spatial axis
    if( model->mJoints[idJoint].mJointType==JointTypeCustom){
      ccid = model->mJoints[idJoint].custom_joint_index;
      dof = model->mCustomJoints[ccid]->mDoFCount;
      for(unsigned int i=0; i<model->mCustomJoints[ccid]->mDoFCount;++i){
        ss.str("");
        ss  << childName << jointType <<   i ;
        qNames[q_index+i] = std::string("Q_").append( ss.str());
        qDotNames[q_index+i] = std::string("QDot_").append( ss.str());
        tauNames[q_index+i] = std::string("Tau_").append( ss.str());
      }
    }else{
      dof = model->mJoints[idJoint].mDoFCount;
      axisIndex=6;
      for(unsigned int j=0; j<6;++j){
        if( fabs( fabs(model->mJoints[idJoint].mJointAxes[0][j])-1.)
            < std::numeric_limits<double>::epsilon() ){
          axisIndex=j;
        }
      }
      if(axisIndex == 6){
        axisType= std::to_string(axisIndex);
      }else{
        axisType = AxisMap[ axisIndex ].name;
        if(model->mJoints[idJoint].mJointAxes[0][axisIndex] < 0.){
          axisType.append("N");
        }
      }
      for(unsigned int i=0; i<model->mJoints[idJoint].mDoFCount;++i){

        if(model->mJoints[idJoint].mDoFCount > 1){
          axisType = std::to_string(i);
        }

        ss.str("");
        ss << childName << jointType <<  axisType ;
        qNames[q_index+i] = std::string("Q_").append( ss.str());
        qDotNames[q_index+i] = std::string("QDot_").append( ss.str());
        tauNames[q_index+i] = std::string("Tau_").append( ss.str());
      }
      //If there is a quaternion joint the 4th component (w) is stored
      //in a different location
      if(model->multdof3_w_index[idJoint] > 0.){

        ss.str("");
        ss <<   childName << jointType <<  "w";
        qNames[ model->multdof3_w_index[idJoint] ]
            = std::string("Q_").append( ss.str());
      }

    }


  }

}

//==============================================================================
bool LuaModelWriteModelHeaderEntries(const char* filename,
                                     const RigidBodyDynamics::Model &model,
                                     bool append){



  std::vector< std::string > bodyNames;
  std::vector< unsigned int> bodyIndex;
  std::vector< std::string > qNames;
  std::vector< std::string > qDotNames;
  std::vector< std::string > qDDotNames;
  std::vector< std::string > tauNames;
  std::string tempStr;
  LuaModelGetCoordinateNames(&model, qNames,qDotNames, tauNames);

  for(unsigned int i=0; i<qDotNames.size();++i){
    tempStr = qDotNames[i];
    tempStr.replace(0,1,"QD");
    qDDotNames.push_back(tempStr);
  }

  std::string childName;
  unsigned int q_index;
  unsigned int idChild;

/*
  qNames.resize(model.q_size);
  qDotNames.resize(model.qdot_size);



  unsigned int dof;

  std::ostringstream ss;

  std::string jointType;
  std::string axisType;
  unsigned int axisIndex;
  unsigned int ccid;

*/
  bool newBody=false;

  //Populate vectors that q, qdot, and tau
  for(unsigned int idJoint=1;idJoint < model.mJoints.size();++idJoint){

    //Get the first index in q associated with this joint
    q_index   = model.mJoints[idJoint].q_index;

    //Get the index of the child body to this joint.
    idChild=idJoint;
    while(idChild < model.mBodies.size() && model.mBodies[idChild].mIsVirtual){
      ++idChild;
    }

    //Get the name of the child body associated with this index
    for(map<string,unsigned int>::const_iterator it=model.mBodyNameMap.begin();
        it != model.mBodyNameMap.end(); ++it){
      if(it->second == idChild){
        childName = it->first;
      }
    }
    newBody=true;
    for(unsigned int i=0; i<bodyIndex.size();++i){
      if(bodyIndex[i]==idChild){
        newBody=false;
      }
    }
    if(newBody){
      bodyNames.push_back(childName);
      bodyIndex.push_back(idChild);
    }

  }

  //Write the map file

  std::vector<unsigned int> qIndex;
  std::vector<unsigned int> qDotIndex;

  qIndex.resize(qNames.size());
  qDotIndex.resize(qDotNames.size());
  for(unsigned int i=0; i<qNames.size();++i){
    qIndex[i] = i;
  }
  for(unsigned int i=0; i<qDotNames.size();++i){
    qDotIndex[i] = i;
  }

  std::ofstream headerFile;
  if(append){
    headerFile.open(filename, std::ofstream::app);
  }else{
    headerFile.open(filename, std::ofstream::out);
  }

  bodyNames.push_back("Last");
  qNames.push_back("Q_Last");
  qDotNames.push_back("QDot_Last");
  qDDotNames.push_back("QDDot_Last");
  tauNames.push_back("Tau_Last");

  appendEnumToFileStream(headerFile,"BodyId",        bodyNames,0, "BodyId_");
  appendEnumToFileStream(headerFile,"PositionId",    qNames,   0, ""     );
  appendEnumToFileStream(headerFile,"VelocityId",    qDotNames,0, ""  );
  appendEnumToFileStream(headerFile,"AccelerationId",qDDotNames,0, "" );
  appendEnumToFileStream(headerFile,"ForceId",       tauNames,0, ""   );

  bodyIndex.push_back(std::numeric_limits<unsigned int>::max());
  qIndex.push_back(std::numeric_limits<unsigned int>::max());
  qDotIndex.push_back(std::numeric_limits<unsigned int>::max());

  appendEnumNameIndexStructToFileStream(headerFile,"BodyMap","BodyId",
                                        bodyNames,"BodyId_",bodyIndex);
  appendEnumNameIndexStructToFileStream(headerFile,"PositionMap","PositionId",
                                        qNames,"",qIndex);
  appendEnumNameIndexStructToFileStream(headerFile,"VelocityMap","VelocityId",
                                        qDotNames,"",qDotIndex);
  appendEnumNameIndexStructToFileStream(headerFile,"AccelerationMap",
                             "AccelerationId", qDDotNames,"",qDotIndex);
  appendEnumNameIndexStructToFileStream(headerFile,"ForceMap","ForceId",
                                        tauNames,"",qDotIndex);

  headerFile.flush();
  headerFile.close();

  return true;
}
//==============================================================================
RBDL_ADDON_DLLAPI
bool LuaModelWritePointsHeaderEntries(const char* header_file_name,
                                      const std::vector<Point> &point_set,
                                      bool append)
{

  std::ofstream headerFile;
  if(append){
    headerFile.open(header_file_name,std::ofstream::app);
  }else{
    headerFile.open(header_file_name,std::ofstream::out);
  }

  std::vector< std::string > names;
  std::vector< unsigned int > indices;

  names.resize(point_set.size());
  indices.resize(point_set.size());

  for(unsigned int i=0;i<point_set.size();++i){
    names[i]  = point_set[i].name;
    indices[i]= i;
  }

  if(point_set.size()>0){
    names.push_back("Last");
    appendEnumToFileStream(headerFile,"PointId", names, 0, "Point_");
    indices.push_back(std::numeric_limits<unsigned int>::max());
    appendEnumNameIndexStructToFileStream(headerFile,"PointMap","PointId",
                                            names,"Point_",indices);
  }
  headerFile.flush();
  headerFile.close();

  return true;
}
//==============================================================================
RBDL_ADDON_DLLAPI
bool LuaModelWriteMotionCaptureMarkerHeaderEntries(
        const char* header_file_name,
        const std::vector< MotionCaptureMarker > &marker_set,
        bool append)
{
  std::ofstream headerFile;
  if(append){
    headerFile.open(header_file_name,std::ofstream::app);
  }else{
    headerFile.open(header_file_name,std::ofstream::out);
  }

  std::vector< std::string > names;
  std::vector< unsigned int > indices;

  names.resize(marker_set.size());
  indices.resize(marker_set.size());

  for(unsigned int i=0;i<marker_set.size();++i){
    names[i]  = marker_set[i].name;
    indices[i]= i;
  }

  if(marker_set.size()>0){
    names.push_back("Last");
    appendEnumToFileStream(headerFile,"MarkerId", names, 0, "Marker_");
    indices.push_back(std::numeric_limits<unsigned int>::max());
    appendEnumNameIndexStructToFileStream(headerFile,"MotionCaptureMarkerMap",
                                          "MarkerId",names,"Marker_",indices);
  }
  headerFile.flush();
  headerFile.close();

  return true;

}

//==============================================================================
RBDL_ADDON_DLLAPI
bool LuaModelWriteLocalFrameHeaderEntries(const char* header_file_name,
                         const std::vector<LocalFrame> &local_frame_set,
                         bool append)
{
  std::ofstream headerFile;
  if(append){
    headerFile.open(header_file_name,std::ofstream::app);
  }else{
    headerFile.open(header_file_name,std::ofstream::out);
  }

  std::vector< std::string > names;
  std::vector< unsigned int > indices;

  names.resize(local_frame_set.size());
  indices.resize(local_frame_set.size());

  for(unsigned int i=0;i<local_frame_set.size();++i){
    names[i]  = local_frame_set[i].name;
    indices[i]= i;
  }

  if(local_frame_set.size()>0){
    names.push_back("Last");
    appendEnumToFileStream(headerFile,"LocalFrameId", names, 0, "LocalFrame_");
    indices.push_back(std::numeric_limits<unsigned int>::max());
    appendEnumNameIndexStructToFileStream(headerFile,"LocalFrameMap",
                                          "LocalFrameId",names,"LocalFrame_",
                                          indices);
  }
  headerFile.flush();
  headerFile.close();
  return true;
}

//==============================================================================
RBDL_ADDON_DLLAPI
bool LuaModelWriteConstraintSetPhaseHeaderEntries(const char* header_file_name,
                         const std::vector< std::string > &constraint_set_names,
                         const std::vector< unsigned int > &constraint_phases,
                         bool append)
{
  std::ofstream headerFile;
  if(append){
    headerFile.open(header_file_name,std::ofstream::app);
  }else{
    headerFile.open(header_file_name,std::ofstream::out);
  }

  std::vector< std::string > phaseNames;
  phaseNames.resize(constraint_phases.size()+1);

  std::vector< std::string > enumTypeAndFieldNames;
  std::vector< std::vector< std::string > > enumEntries;
  std::vector< std::string > entryPrefix;

  std::vector< std::string > indexTypeAndFieldNames;
  std::vector< std::vector< unsigned int > > indexMatrix;

  indexTypeAndFieldNames.resize(0);
  indexMatrix.resize(0);

  enumTypeAndFieldNames.resize(3);
  enumEntries.resize(constraint_phases.size()+1);
  enumTypeAndFieldNames[0] = "ConstraintSetPhaseId phase_id";
  enumTypeAndFieldNames[1] = "const char* name";
  enumTypeAndFieldNames[2] = "ConstraintSetId constraint_set_id";

  entryPrefix.resize(3);
  entryPrefix[0] = "";
  entryPrefix[1] = "";
  entryPrefix[2] = "CS_";
  for(unsigned int i=0;i<constraint_phases.size();++i){
    enumEntries[i].resize(3);

    enumEntries[i][0] = "Phase_";
    enumEntries[i][0].append(std::to_string(i));//conSetInfo[phases[i]].name;
    enumEntries[i][0].append("_");
    enumEntries[i][0].append(constraint_set_names[constraint_phases[i]]);
    enumEntries[i][1] = "\"";
    enumEntries[i][1].append( enumEntries[i][0] );
    enumEntries[i][1].append("\"");
    enumEntries[i][2] = constraint_set_names[constraint_phases[i]];

    phaseNames[i] = enumEntries[i][0];
  }
  enumEntries[constraint_phases.size()].resize(3);
  enumEntries[constraint_phases.size()][0]="Phase_Last";
  enumEntries[constraint_phases.size()][1]="\"Phase_Last\"";
  enumEntries[constraint_phases.size()][2]="Last";
  phaseNames[ constraint_phases.size()]="Phase_Last";


  if(constraint_phases.size()>0){
    appendEnumToFileStream(headerFile,"ConstraintSetPhaseId", phaseNames, 0, "");

    appendEnumStructToFileStream(  headerFile,
                                   "ConstraintSetPhaseMap",
                                   enumTypeAndFieldNames,
                                   enumEntries,
                                   entryPrefix,
                                   indexTypeAndFieldNames,
                                   indexMatrix);

  }
  headerFile.flush();
  headerFile.close();
  return true;
}
//==============================================================================
RBDL_ADDON_DLLAPI
bool LuaModelWriteConstraintSetHeaderEntries(const char* header_file_name,
       const std::vector< std::string > &constraint_set_names,
       const std::vector<RigidBodyDynamics::ConstraintSet> &constraint_sets,
       bool append)
{

  if(constraint_set_names.size()!=constraint_sets.size()){
    ostringstream errormsg;
    errormsg << "constraint_set_names (size: "
             << constraint_set_names.size()
             << ") and "
             << "constraint_sets "
             << "(size: "
             << constraint_sets.size()
             << ")"
             <<" must have the same size but do not."
             <<  endl;
    throw Errors::RBDLError(errormsg.str());
  }

  std::ofstream headerFile;
  if(append){
    headerFile.open(header_file_name, std::ofstream::app);
  }else{
    headerFile.open(header_file_name, std::ofstream::out);
  }

  std::vector< std::string > conSetNames;
  std::vector< unsigned int > conSetIndex;

  std::vector< std::string > groupNames;

  std::vector< std::string > conNames;
  std::vector< std::string > conEnumPrefix;
  std::vector< std::vector< std::string > > conEnumMatrix;
  std::vector< std::string > conEnumVector;
  std::vector< std::string > conEnumTypeAndFieldNames;

  conEnumPrefix.push_back("C_");
  conEnumPrefix.push_back("CS_");
  conEnumPrefix.push_back("CG_");
  conEnumVector.resize(3);
  conEnumTypeAndFieldNames.push_back("ConstraintId constraint_id");
  conEnumTypeAndFieldNames.push_back("ConstraintSetId constraint_set_id");
  conEnumTypeAndFieldNames.push_back("ConstraintGroupId constraint_group_id");

  std::vector< std::vector< unsigned int > > conIndexMatrix;
  std::vector< unsigned int > conIndexVector;
  std::vector< std::string > conIndexTypeAndFieldNames;
  conIndexVector.resize(1);

  conIndexTypeAndFieldNames.push_back("unsigned int constraint_set_item_id");

  std::string conTypeName;
  std::stringstream ss;



  for(unsigned int i=0;i<constraint_set_names.size();++i){
    conSetNames.push_back(constraint_set_names[i]);
    conSetIndex.push_back(i);



    for(unsigned int j = 0; j < constraint_sets[i].constraints.size();++j){
      groupNames.push_back(
            std::string(constraint_sets[i].constraints[j]->getName()) );
    }


    for(unsigned int j=0; j<constraint_sets[i].constraints.size();++j){
      for(unsigned int k=0; k<constraint_sets[i].constraints[j]->getConstraintSize();++k){

        ss.str("");
        ss << constraint_set_names[i];
        ss << "_" << std::string(constraint_sets[i].constraints[j]->getName());
        ss << "_" << k;
        conNames.push_back(ss.str());
        conEnumVector[0] = ss.str();
        conEnumVector[1] = constraint_set_names[i];
        conEnumVector[2] =
            std::string(constraint_sets[i].constraints[j]->getName());
        conEnumMatrix.push_back(conEnumVector);
        conIndexVector[0] = k;
        conIndexMatrix.push_back(conIndexVector);
      }

    }
  }

  if(constraint_sets.size()>0){
    conSetNames.push_back("Last");
    appendEnumToFileStream(headerFile,"ConstraintSetId", conSetNames, 0, "CS_");

    conSetIndex.push_back(std::numeric_limits<unsigned int>::max());
    appendEnumNameIndexStructToFileStream(headerFile,"ConstraintSetMap",
                                          "ConstraintSetId",
                                          conSetNames,"CS_",conSetIndex);
    groupNames.push_back("Last");
    appendEnumToFileStream(headerFile,"ConstraintGroupId",groupNames,0,"CG_");

    for(unsigned int k=0; k < conEnumVector.size();++k){
      conEnumVector[k] = "Last";
    }
    conEnumMatrix.push_back(conEnumVector);
    for(unsigned int k=0; k < conIndexVector.size();++k){
      conIndexVector[k] = std::numeric_limits<unsigned int>::max();
    }
    conIndexMatrix.push_back(conIndexVector);

    //Unique set of enums for every combination of constraint x set
    conNames.push_back("Last");
    appendEnumToFileStream(headerFile,"ConstraintId", conNames, 0, "C_");

    //Table that has for every unique constraint, its set, and set index
    //This table makes it possible for the human using this model to
    //access a specific constraint using a human readable name.
    appendEnumStructToFileStream(headerFile,"ConstraintMap",
                                   conEnumTypeAndFieldNames,
                                   conEnumMatrix, conEnumPrefix,
                                   conIndexTypeAndFieldNames, conIndexMatrix);

  }
  headerFile.flush();
  headerFile.close();

  return true;
}
//==============================================================================
#ifdef RBDL_BUILD_ADDON_MUSCLE
RBDL_ADDON_DLLAPI
bool LuaModelWriteMillard2016TorqueMuscleHeaderEntries(
    const char* header_file_name,
    const std::vector<RigidBodyDynamics::Addons::Muscle
                      ::Millard2016TorqueMuscle> &mtg_set,
    const std::vector<Millard2016TorqueMuscleConfig > &mtg_set_info,
    bool append)
{

  if(mtg_set.size()!=mtg_set_info.size()){
    ostringstream errormsg;
    errormsg << "mtg_set (size: "
             << mtg_set.size()
             << ") and "
             << "mtg_set_info "
             << "(size: "
             << mtg_set_info.size()
             << ")"
             <<" must have the same size but do not."
             <<  endl;
    throw Errors::RBDLError(errormsg.str());
  }

  std::ofstream headerFile;
  if(append){
    headerFile.open(header_file_name,std::ofstream::app);
  }else{
    headerFile.open(header_file_name,std::ofstream::out);
  }
  std::vector< std::string > names;
  std::vector< unsigned int > indices;

  names.resize(mtg_set_info.size());
  indices.resize(mtg_set_info.size());

  for(unsigned int i=0;i<mtg_set_info.size();++i){
    names[i] = mtg_set_info[i].name;
    indices[i]= i;
  }

  if(mtg_set_info.size()>0){
    names.push_back("Last");
    appendEnumToFileStream(headerFile,"Millard2016TorqueMuscleId",
                           names, 0, "MTG_");
    indices.push_back(std::numeric_limits<unsigned int>::max());
    appendEnumNameIndexStructToFileStream(
          headerFile,"Millard2016TorqueMuscleMap",
          "Millard2016TorqueMuscleId", names,"MTG_",indices);
  }
  headerFile.flush();
  headerFile.close();

  return true;
}

#endif

//==============================================================================

}
}
