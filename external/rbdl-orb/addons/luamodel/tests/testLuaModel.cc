/*
 * RBDL - Rigid Body Dynamics Library: Addon : muscle
 * Copyright (c) 2016 Matthew Millard <millard.matthew@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

//==============================================================================
// INCLUDES
//==============================================================================
#define CATCH_CONFIG_MAIN
#include "luamodel.h"
#include "luatables.h"
#include <rbdl/rbdl.h>
#include <string>
#include <vector>
#include <cstring>

#include "rbdl_tests.h"

#ifdef RBDL_BUILD_ADDON_MUSCLE
#include "../muscle/Millard2016TorqueMuscle.h"
#endif

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons;

using namespace std;

const double TEST_PREC = 1.0e-11;

std::string rbdlSourcePath = RBDL_LUAMODEL_SOURCE_DIR;
   
TEST_CASE(__FILE__"_LoadLuaModel", "")
{
  Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/samplemodel.lua");
  bool modelLoaded = LuaModelReadFromFile(modelFile.c_str(), &model, false);
  CHECK(modelLoaded);
}
TEST_CASE(__FILE__"_LoadMotionCaptureMarkers", "")
{
  Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/samplemodel.lua");
  bool modelLoaded = LuaModelReadFromFile(modelFile.c_str(), &model, false);
  std::vector< MotionCaptureMarker > updMarkerSet;
  bool markersLoaded = LuaModelReadMotionCaptureMarkers(modelFile.c_str(),
                                               &model, updMarkerSet,false);
  CHECK(updMarkerSet.size()==6);
  //The markers come out of order which makes testing a bit tricky.
  for(unsigned int i=0; i<updMarkerSet.size(); ++i){
    bool flag_found = false;
    if(std::strcmp(updMarkerSet[i].name.c_str(),"LASI")==0){
      CHECK( updMarkerSet[i].body_id == model.GetBodyId("pelvis"));
      CHECK_THAT(updMarkerSet[i].point_local[0],
                 IsClose(0.047794,TEST_PREC, TEST_PREC));
      CHECK_THAT(updMarkerSet[i].point_local[1],
                 IsClose(0.200000,TEST_PREC, TEST_PREC));
      CHECK_THAT(updMarkerSet[i].point_local[2],
                 IsClose(0.070908,TEST_PREC, TEST_PREC));
      flag_found = true;
    }
    if(std::strcmp(updMarkerSet[i].name.c_str(),"RASI")==0){
      CHECK( updMarkerSet[i].body_id == model.GetBodyId("pelvis"));
      CHECK_THAT(updMarkerSet[i].point_local[0],
                 IsClose(0.047794,TEST_PREC, TEST_PREC)
      );
      CHECK_THAT(updMarkerSet[i].point_local[1],
                 IsClose(-0.200000,TEST_PREC, TEST_PREC)
      );
      CHECK_THAT(updMarkerSet[i].point_local[2],
                 IsClose(0.070908,TEST_PREC, TEST_PREC)
      );
      flag_found = true;
    }
    if(std::strcmp(updMarkerSet[i].name.c_str(),"LPSI")==0){
      CHECK( updMarkerSet[i].body_id == model.GetBodyId("pelvis"));
      CHECK_THAT(updMarkerSet[i].point_local[0],
                 IsClose(-0.106106,TEST_PREC, TEST_PREC)
      );
      CHECK_THAT(updMarkerSet[i].point_local[1],
                 IsClose(0.200000,TEST_PREC, TEST_PREC)
      );
      CHECK_THAT(updMarkerSet[i].point_local[2],
                 IsClose(0.070908,TEST_PREC, TEST_PREC)
      );
      flag_found = true;
    }
    if(std::strcmp(updMarkerSet[i].name.c_str(),"RPSI")==0){
      CHECK( updMarkerSet[i].body_id == model.GetBodyId("pelvis"));
      CHECK_THAT(updMarkerSet[i].point_local[0],
                 IsClose(-0.106106,TEST_PREC, TEST_PREC)
      );
      CHECK_THAT(updMarkerSet[i].point_local[1],
                 IsClose(-0.200000,TEST_PREC, TEST_PREC)
      );
      CHECK_THAT(updMarkerSet[i].point_local[2],
                 IsClose(0.070908,TEST_PREC, TEST_PREC)
      );
      flag_found = true;
    }
    if(std::strcmp(updMarkerSet[i].name.c_str(),"RTHI")==0){
      CHECK( updMarkerSet[i].body_id == model.GetBodyId("thigh_right"));
      CHECK_THAT(updMarkerSet[i].point_local[0],
                 IsClose(-0.007376,TEST_PREC, TEST_PREC)
      );
      CHECK_THAT(updMarkerSet[i].point_local[1],
                 IsClose(0.000000,TEST_PREC, TEST_PREC)
      );
      CHECK_THAT(updMarkerSet[i].point_local[2],
                 IsClose(-0.243721,TEST_PREC, TEST_PREC)
      );
      flag_found = true;
    }
    if(std::strcmp(updMarkerSet[i].name.c_str(),"RKNE")==0){
      CHECK( updMarkerSet[i].body_id == model.GetBodyId("thigh_right"));
      CHECK_THAT(updMarkerSet[i].point_local[0],
                 IsClose(-0.011611,TEST_PREC, TEST_PREC)
      );
      CHECK_THAT(updMarkerSet[i].point_local[1],
                 IsClose(0.000000,TEST_PREC, TEST_PREC)
      );
      CHECK_THAT(updMarkerSet[i].point_local[2],
                 IsClose(-0.454494,TEST_PREC, TEST_PREC)
      );
      flag_found = true;
    }
    CHECK(flag_found);
  }


}
TEST_CASE(__FILE__"_LoadLocalFrames", "")
{
  Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/samplemodel.lua");
  bool modelLoaded = LuaModelReadFromFile(modelFile.c_str(), &model, false);
  std::vector< LocalFrame > updLocalFrameSet;
  bool localFramesLoaded = LuaModelReadLocalFrames(modelFile.c_str(),&model,
                                                    updLocalFrameSet,false);

  CHECK(updLocalFrameSet.size()==2);

  unsigned int thighLeftId = model.GetBodyId("thigh_left");
  unsigned int thighRightId = model.GetBodyId("thigh_right");

  CHECK(std::strcmp("Pocket_L",updLocalFrameSet[0].name.c_str())==0);
  CHECK(updLocalFrameSet[0].body_id == thighLeftId);

  CHECK_THAT(updLocalFrameSet[0].r[0],
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[0].r[1],
             IsClose(0.2, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[0].r[2],
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );

  CHECK_THAT(updLocalFrameSet[0].E(0,0),
             IsClose(1.0, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[0].E(0,1),
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[0].E(0,2),
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );

  CHECK_THAT(updLocalFrameSet[0].E(1,0),
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[0].E(1,1),
             IsClose(1.0, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[0].E(1,2),
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );

  CHECK_THAT(updLocalFrameSet[0].E(2,0),
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[0].E(2,1),
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[0].E(2,2),
             IsClose(1.0, TEST_PREC, TEST_PREC)
  );


  CHECK(std::strcmp("Pocket_R",updLocalFrameSet[1].name.c_str())==0);
  CHECK(updLocalFrameSet[1].body_id == thighRightId);
  CHECK_THAT(updLocalFrameSet[1].r[0],
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[1].r[1],
             IsClose(-0.2, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[1].r[2],
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );

  CHECK_THAT(updLocalFrameSet[1].E(0,0),
             IsClose(1.0, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[1].E(0,1),
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[1].E(0,2),
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );

  CHECK_THAT(updLocalFrameSet[1].E(1,0),
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[1].E(1,1),
             IsClose(1.0, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[1].E(1,2),
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );

  CHECK_THAT(updLocalFrameSet[1].E(2,0),
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[1].E(2,1),
             IsClose(0.0, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updLocalFrameSet[1].E(2,2),
             IsClose(1.0, TEST_PREC, TEST_PREC)
  );


}
TEST_CASE(__FILE__"_LoadPoints", "")
{
  Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/samplemodel.lua");
  bool modelLoaded = LuaModelReadFromFile(modelFile.c_str(), &model, false);
  std::vector< Point > updPointSet;
  bool pointsLoaded = LuaModelReadPoints(modelFile.c_str(),&model,
                                         updPointSet,false);
  CHECK(updPointSet.size()==4);
  
  unsigned int bodyId = model.GetBodyId("foot_right");

  CHECK( strcmp( updPointSet[0].name.c_str(),"Heel_Medial_L")      == 0);
  CHECK( strcmp( updPointSet[1].name.c_str(),"Heel_Lateral_L")     == 0);
  CHECK( strcmp( updPointSet[2].name.c_str(),"ForeFoot_Medial_L")  == 0);
  CHECK( strcmp( updPointSet[3].name.c_str(),"ForeFoot_Lateral_L") == 0);

  CHECK( updPointSet[0].body_id == bodyId );
  CHECK( updPointSet[1].body_id == bodyId );
  CHECK( updPointSet[2].body_id == bodyId );
  CHECK( updPointSet[3].body_id == bodyId );

  CHECK_THAT(updPointSet[0].point_local[0],
             IsClose(-0.080, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updPointSet[0].point_local[1],
             IsClose(-0.042, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updPointSet[0].point_local[2],
             IsClose(-0.091, TEST_PREC, TEST_PREC)
  );

  CHECK_THAT(updPointSet[1].point_local[0],
             IsClose(-0.080, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updPointSet[1].point_local[1],
             IsClose(0.042, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updPointSet[1].point_local[2],
             IsClose(-0.091, TEST_PREC, TEST_PREC)
  );
  
  CHECK_THAT(updPointSet[2].point_local[0],
             IsClose(0.181788, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updPointSet[2].point_local[1],
             IsClose(-0.054000, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updPointSet[2].point_local[2],
             IsClose(-0.091000, TEST_PREC, TEST_PREC)
  );

  CHECK_THAT(updPointSet[3].point_local[0],
             IsClose(0.181788, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updPointSet[3].point_local[1],
             IsClose(0.054000, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(updPointSet[3].point_local[2],
             IsClose(-0.091000, TEST_PREC, TEST_PREC)
  );
}

TEST_CASE(__FILE__"_LoadConstrainedLuaModel", "")
{
  RigidBodyDynamics::Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/sampleconstrainedmodel.lua");

  std::vector<std::string> constraintSetNames = 
    LuaModelGetConstraintSetNames(modelFile.c_str());
  std::vector<RigidBodyDynamics::ConstraintSet> constraintSets;

  constraintSets.resize(constraintSetNames.size());
  for(unsigned int i=0; i<constraintSetNames.size();++i){
    constraintSets[i] = ConstraintSet();
  }


  bool modelLoaded = LuaModelReadFromFileWithConstraints( modelFile.c_str(),
                                                          &model,
                                                          constraintSets,
                                                          constraintSetNames,
                                                          false);

  CHECK(modelLoaded);

  unsigned int baseId = model.GetBodyId("base");
  unsigned int rootId = model.GetBodyId("ROOT");
  unsigned int l12Id = model.GetBodyId("l12");
  unsigned int l22Id = model.GetBodyId("l22");

  unsigned int groupIndex = 0;


  // Contact Constraint X
  groupIndex = constraintSets[0].getGroupIndexByName("contactBaseX");
  CHECK(constraintSets[0].getGroupSize(groupIndex) == 1);
  CHECK(constraintSets[0].getGroupType(groupIndex) == ConstraintTypeContact);
  unsigned int userDefinedId = constraintSets[0].getGroupId(groupIndex);
  CHECK(userDefinedId == 2);

  std::vector<unsigned int> bodyIds =
      constraintSets[0].contactConstraints[0]->getBodyIds();
  CHECK(bodyIds.size() == 2);
  CHECK(bodyIds[0] == baseId);
  CHECK(bodyIds[1] == rootId);

  std::vector< Vector3d > normalVectors =
      constraintSets[0].contactConstraints[0]->getConstraintNormalVectors();

  // (all contact constraints between the same pair of bodies are grouped)
  CHECK(normalVectors.size()==1);
  CHECK_THAT(normalVectors[0][0],
             IsClose(1., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(normalVectors[0][1],
             IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(normalVectors[0][2],
             IsClose(0., TEST_PREC, TEST_PREC)
  );

  //MM 17/5/2020
  //Contract constraints currently do not have the Baumgarte stabilization
  //parameter exposed: these kinds of constraints are so well numerically
  //behaved that this kind of constraint stabilization is normally not required.
  CHECK(constraintSets[0].isBaumgarteStabilizationEnabled(groupIndex)==false);

  // Contact Constraint YZ
  groupIndex = constraintSets[0].getGroupIndexByName("contactBaseYZ");
  CHECK(constraintSets[0].getGroupSize(groupIndex) == 2);
  CHECK(constraintSets[0].getGroupType(groupIndex) == ConstraintTypeContact);
  userDefinedId = constraintSets[0].getGroupId(groupIndex);
  CHECK(userDefinedId == 3);

  normalVectors =
        constraintSets[0].contactConstraints[1]->getConstraintNormalVectors();
  CHECK(normalVectors.size()==2);

  CHECK_THAT(normalVectors[0][0],
             IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(normalVectors[0][1],
             IsClose(1., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(normalVectors[0][2],
             IsClose(0., TEST_PREC, TEST_PREC)
  );

  CHECK_THAT(normalVectors[1][0],
             IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(normalVectors[1][1],
             IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(normalVectors[1][2],
             IsClose(1., TEST_PREC, TEST_PREC)
  );

  //MM 17/5/2020
  //Contract constraints currently do not have the Baumgarte stabilization
  //parameter exposed: these kinds of constraints are so well numerically
  //behaved that this kind of constraint stabilization is normally not required.
  CHECK(constraintSets[0].isBaumgarteStabilizationEnabled(groupIndex) == false);

  // Loop Constraint X
  groupIndex = constraintSets[0].getGroupIndexByName("loopL12L22Tx");

  CHECK(constraintSets[0].getGroupSize(groupIndex) == 1);
  CHECK(constraintSets[0].getGroupType(groupIndex) == ConstraintTypeLoop);
  userDefinedId = constraintSets[0].getGroupId(groupIndex);
  CHECK(userDefinedId == 1);

  bodyIds = constraintSets[0].loopConstraints[0]->getBodyIds();
  CHECK(bodyIds.size()==2);
  CHECK(bodyIds[0] == l12Id);
  CHECK(bodyIds[1] == l22Id);

  //Loop constraints often require stabilization so the Baumgarte
  //stabilization parameters are exposed
  CHECK(constraintSets[0].isBaumgarteStabilizationEnabled(groupIndex) == false);

  std::vector< SpatialVector > axis =
    constraintSets[0].loopConstraints[0]->getConstraintAxes();
  CHECK(axis.size()==1);
  CHECK_THAT( axis[0][0],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[0][1],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[0][2],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[0][3],
              IsClose(1., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[0][4],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[0][5],
              IsClose(0., TEST_PREC, TEST_PREC)
  );

  // Loop Constraint Y
  groupIndex = constraintSets[0].getGroupIndexByName("loopL12L22Ty");
  CHECK(constraintSets[0].getGroupSize(groupIndex) == 1);
  CHECK(constraintSets[0].getGroupType(groupIndex) == ConstraintTypeLoop);
  userDefinedId = constraintSets[0].getGroupId(groupIndex);
  CHECK(userDefinedId == 2);

  axis =constraintSets[0].loopConstraints[1]->getConstraintAxes();
  CHECK(axis.size()==1);

  CHECK_THAT( axis[0][0],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[0][1],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[0][2],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[0][3],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[0][4],
              IsClose(1., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[0][5],
              IsClose(0., TEST_PREC, TEST_PREC)
  );

  //Loop constraints often require stabilization so the Baumgarte
  //stabilization parameters are exposed
  CHECK(constraintSets[0].isBaumgarteStabilizationEnabled(groupIndex) == true);


  // Contact Constraint XYZ
  groupIndex = constraintSets[1].getGroupIndexByName("contactBaseXYZ");
  CHECK(constraintSets[1].getGroupSize(groupIndex) == 3);
  CHECK(constraintSets[1].getGroupType(groupIndex) == ConstraintTypeContact);
  CHECK(constraintSets[1].getGroupId(groupIndex) == 2);

  bodyIds = constraintSets[1].contactConstraints[0]->getBodyIds();
  CHECK(bodyIds.size()==2);
  CHECK(bodyIds[0] == baseId);
  CHECK(bodyIds[1] == rootId);

  normalVectors =
      constraintSets[1].contactConstraints[0]->getConstraintNormalVectors();
  CHECK(normalVectors.size()==3);
  CHECK_THAT(normalVectors[0][0],
             IsClose(1., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(normalVectors[0][1],
             IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(normalVectors[0][2],
             IsClose(0., TEST_PREC, TEST_PREC)
  );

  CHECK_THAT(normalVectors[1][0],
             IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(normalVectors[1][1],
             IsClose(1., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(normalVectors[1][2],
             IsClose(0., TEST_PREC, TEST_PREC)
  );

  CHECK_THAT(normalVectors[2][0],
             IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(normalVectors[2][1],
             IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(normalVectors[2][2],
             IsClose(1., TEST_PREC, TEST_PREC)
  );

  CHECK(constraintSets[1].isBaumgarteStabilizationEnabled(groupIndex) == false);

  // Loop Constraint Tx Ty
  groupIndex = constraintSets[1].getGroupIndexByName("loopL12L22TxTy");
  CHECK(constraintSets[1].getGroupSize(groupIndex) == 2);
  CHECK(constraintSets[1].getGroupType(groupIndex) == ConstraintTypeLoop);
  CHECK(constraintSets[1].getGroupId(groupIndex) == 1);

  bodyIds = constraintSets[1].loopConstraints[0]->getBodyIds();
  CHECK(bodyIds.size()==2);
  CHECK(bodyIds[0] == l12Id);
  CHECK(bodyIds[1] == l22Id);

  axis =
    constraintSets[1].loopConstraints[0]->getConstraintAxes();
  CHECK(axis.size()==2);
  CHECK_THAT( axis[0][0],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[0][1],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[0][2],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[0][3],
              IsClose(1., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[0][4],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[0][5],
              IsClose(0., TEST_PREC, TEST_PREC)
  );

  CHECK_THAT( axis[1][0],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[1][1],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[1][2],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[1][3],
              IsClose(0., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[1][4],
              IsClose(1., TEST_PREC, TEST_PREC)
  );
  CHECK_THAT( axis[1][5],
              IsClose(0., TEST_PREC, TEST_PREC)
  );

  CHECK(constraintSets[1].isBaumgarteStabilizationEnabled(groupIndex) == false);

  std::vector<unsigned int> phasing;
  bool constraintSetPhasingLoaded =
      LuaModelGetConstraintSetPhases(modelFile.c_str(),constraintSetNames,
                                     phasing);
  CHECK(constraintSetPhasingLoaded);

  CHECK(phasing[0]==0);
  CHECK(phasing[1]==1);
  CHECK(phasing[2]==1);
  CHECK(phasing[3]==0);

}

#ifdef RBDL_BUILD_ADDON_MUSCLE
TEST_CASE(__FILE__"_LoadMuscleTorqueGenerators", "")
{
  RigidBodyDynamics::Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/samplemodelwithtorquemuscles.lua");



  bool modelLoaded = LuaModelReadFromFile( modelFile.c_str(),
                                           &model,
                                           false);

  CHECK(modelLoaded);

  HumanMetaData humanData;
  bool humanDataLoaded =
      LuaModelReadHumanMetaData(modelFile.c_str(),humanData,false);
  CHECK(humanDataLoaded);

  CHECK(std::fabs(humanData.age - 35.0) < TEST_PREC);
  CHECK(std::fabs(humanData.height - 1.73) < TEST_PREC);
  CHECK(std::fabs(humanData.height - 1.73) < TEST_PREC);
  CHECK(std::strcmp(humanData.age_group.c_str(),"Young18To25")==0);
  CHECK(std::strcmp(humanData.gender.c_str(),"male")==0);


  std::vector < Muscle::Millard2016TorqueMuscle > mtgSet;
  std::vector< Millard2016TorqueMuscleConfig > mtgInfoSet;

  bool torqueMusclesLoaded = LuaModelReadMillard2016TorqueMuscleSets(
        modelFile.c_str(),&model,humanData,mtgSet,mtgInfoSet,false);

  CHECK(torqueMusclesLoaded);
  CHECK(mtgSet.size() == 12);
  CHECK(mtgInfoSet.size() == 12);

  unsigned int i=0;

  //Check that the data is being loaded as it is written in the file for the
  //full right leg
  i=0;
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),"HipExtension_R")==0);
  CHECK(std::fabs(mtgInfoSet[i].angle_sign  - (-1)) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].torque_sign - ( 1)) < TEST_PREC);
  CHECK(std::strcmp(mtgInfoSet[i].body.c_str(),"thigh_right")== 0);
  CHECK(mtgInfoSet[i].joint_index - 1 == 0);
  CHECK(std::fabs(mtgInfoSet[i].activation_time_constant   - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].deactivation_time_constant - 0.05) < TEST_PREC);
  i=1;
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),"HipFlexion_R")==0);
  CHECK(std::fabs(mtgInfoSet[i].angle_sign  - (-1)) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].torque_sign - (-1)) < TEST_PREC);
  CHECK(std::strcmp(mtgInfoSet[i].body.c_str(),"thigh_right")== 0);
  CHECK(mtgInfoSet[i].joint_index - 1 == 0);
  CHECK(std::fabs(mtgInfoSet[i].activation_time_constant   - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].deactivation_time_constant - 0.05) < TEST_PREC);
  i=2;
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),"KneeExtension_R")==0);
  CHECK(std::fabs(mtgInfoSet[i].angle_sign  - ( 1)) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].torque_sign - (-1)) < TEST_PREC);
  CHECK(std::strcmp(mtgInfoSet[i].body.c_str(),"shank_right")== 0);
  CHECK(std::fabs(mtgInfoSet[i].activation_time_constant   - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].deactivation_time_constant - 0.05) < TEST_PREC);
  i=3;
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),"KneeFlexion_R")==0);
  CHECK(std::fabs(mtgInfoSet[i].angle_sign  - ( 1)) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].torque_sign - ( 1)) < TEST_PREC);
  CHECK(std::strcmp(mtgInfoSet[i].body.c_str(),"shank_right")== 0);
  CHECK(std::fabs(mtgInfoSet[i].activation_time_constant   - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].deactivation_time_constant - 0.05) < TEST_PREC);
  i=4;
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),"AnkleExtension_R")==0);
  CHECK(std::fabs(mtgInfoSet[i].angle_sign  - (-1)) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].torque_sign - ( 1)) < TEST_PREC);
  CHECK(std::strcmp(mtgInfoSet[i].body.c_str(),"foot_right")== 0);
  CHECK(std::fabs(mtgInfoSet[i].activation_time_constant   - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].deactivation_time_constant - 0.05) < TEST_PREC);
  i=5;
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),"AnkleFlexion_R")==0);
  CHECK(std::fabs(mtgInfoSet[i].angle_sign  - (-1)) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].torque_sign - (-1)) < TEST_PREC);
  CHECK(std::strcmp(mtgInfoSet[i].body.c_str(),"foot_right")== 0);
  CHECK(std::fabs(mtgInfoSet[i].activation_time_constant   - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].deactivation_time_constant - 0.05) < TEST_PREC);
  i=6;
  //Check that the passive_element_torque_scale is working
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),
                    "AnkleFlexion_R_FpeHalfScale")==0);
  unsigned int iRef = 5;
  CHECK_THAT(mtgSet[i].getPassiveTorqueScale(),
             IsClose(0.5, TEST_PREC, TEST_PREC)
  );
  Muscle::TorqueMuscleInfo tmi, tmiRef;
  mtgSet[i].calcTorqueMuscleInfo(1,0,0,tmi);
  mtgSet[iRef].calcTorqueMuscleInfo(1,0,0,tmiRef);
  CHECK(tmiRef.fiberPassiveTorqueAngleMultiplier > 0.);
  CHECK_THAT(tmiRef.fiberPassiveTorqueAngleMultiplier,
             IsClose(tmi.fiberPassiveTorqueAngleMultiplier*2.0,
                     TEST_PREC, TEST_PREC)
  );
  i=7;
  //Check that the max_isometric_torque_scale is working
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),
                    "AnkleFlexion_R_FisoHalfScale")==0);
  CHECK_THAT(mtgSet[iRef].getMaximumActiveIsometricTorque(),
             IsClose(mtgSet[i].getMaximumActiveIsometricTorque()*2.0,
                     TEST_PREC, TEST_PREC)
  );
  i=8;
  //Check that max_angular_velocity_scale is working
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),
                    "AnkleFlexion_R_OmegaHalfScale")==0);
  CHECK_THAT(mtgSet[iRef].getMaximumConcentricJointAngularVelocity(),
             IsClose(mtgSet[i].getMaximumConcentricJointAngularVelocity()*2.0,
                     TEST_PREC, TEST_PREC)
  );

  i=9;
  //UnitExtensor
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),
                    "UnitExtensor_R")==0);
  mtgSet[i].calcTorqueMuscleInfo(0,0,1,tmi);
  double angleSign = mtgInfoSet[i].angle_sign;
  CHECK_THAT(tmi.fiberActiveTorqueAngleMultiplier,
             IsClose(1.0, TEST_PREC, TEST_PREC)
  );

  //This extensor gets a Gaussian shaped active force length curve
  //with a standard deviation of 1 radian.
  double angle = 1;
  double width = 1;
  double faRef = exp(-angle*angle/(2*width*width));
  mtgSet[i].calcTorqueMuscleInfo(1*angleSign,0,1,tmi);
  CHECK_THAT(tmi.fiberActiveTorqueAngleMultiplier,
             IsClose(faRef, TEST_PREC, TEST_PREC)
  );
  //The UnitExtensors passive curve reaches a unit torque at 1 radian of flexion
  CHECK_THAT(tmi.fiberPassiveTorqueAngleMultiplier,
             IsClose(1.0, TEST_PREC, TEST_PREC)
  );

  mtgSet[i].calcTorqueMuscleInfo(-1*angleSign,0,1,tmi);
  CHECK_THAT(tmi.fiberActiveTorqueAngleMultiplier,
             IsClose(faRef, TEST_PREC, TEST_PREC)
  );
  //The UnitExtensor has a maximum isometric torque of 1 Nm
  CHECK_THAT(mtgSet[i].getMaximumActiveIsometricTorque(),
             IsClose(1., TEST_PREC, TEST_PREC)
  );
  //The UnitExtensor has a maximum angular velocity of 1 rad/sec
  CHECK_THAT(mtgSet[i].getMaximumConcentricJointAngularVelocity(),
             IsClose(1., TEST_PREC, TEST_PREC)
  );

  i=10;
  //UnitFlexor
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),
                    "UnitFlexor_R")==0);

  mtgSet[i].calcTorqueMuscleInfo(0,0,1,tmi);
  CHECK_THAT(tmi.fiberActiveTorqueAngleMultiplier,
             IsClose(1.0, TEST_PREC, TEST_PREC)
  );

  //This flexor gets a Gaussian shaped active force length curve
  //with a standard deviation of 1 radian.
  mtgSet[i].calcTorqueMuscleInfo(1*angleSign,0,1,tmi);
  CHECK_THAT(tmi.fiberActiveTorqueAngleMultiplier,
             IsClose(faRef, TEST_PREC, TEST_PREC)
  );

  mtgSet[i].calcTorqueMuscleInfo(-1*angleSign,0,1,tmi);
  CHECK_THAT(tmi.fiberActiveTorqueAngleMultiplier,
             IsClose(faRef, TEST_PREC, TEST_PREC)
  );
  //The UnitFlexor's passive curve reaches a unit torque at 1 radian of extension
  CHECK_THAT(tmi.fiberPassiveTorqueAngleMultiplier,
             IsClose(1.0, TEST_PREC, TEST_PREC)
  );

  //The UnitExtensor has a maximum isometric torque of 1 Nm
  CHECK_THAT(mtgSet[i].getMaximumActiveIsometricTorque(),
             IsClose(1., TEST_PREC, TEST_PREC)
  );
  //The UnitExtensor has a maximum angular velocity of 1 rad/sec
  CHECK_THAT(mtgSet[i].getMaximumConcentricJointAngularVelocity(),
             IsClose(1.*angleSign, TEST_PREC, TEST_PREC)
  );

  i=11;
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),
                    "KneeExtension_R_Anderson2007")==0);
  CHECK(std::strcmp( mtgInfoSet[i].data_set.c_str(),
                     "Anderson2007") == 0);
  CHECK(std::strcmp( mtgInfoSet[i].age_group.c_str(),
                     "SeniorOver65") == 0);
  CHECK(std::strcmp( mtgInfoSet[i].gender.c_str(),
                     "female") == 0);

  CHECK(mtgSet[i].getDataSet() == Muscle::DataSet::Anderson2007);
  CHECK(mtgSet[i].getAgeGroup() == Muscle::AgeGroupSet::SeniorOver65);
  CHECK(mtgSet[i].getGender() == Muscle::GenderSet::Female);
  CHECK_THAT(mtgSet[i].getSubjectMass(),
             IsClose(81.68, TEST_PREC, TEST_PREC)
  );
  CHECK_THAT(mtgSet[i].getSubjectHeight(),
             IsClose(1.73, TEST_PREC, TEST_PREC)
  );

  CHECK_THAT(mtgSet[i].getActiveTorqueAngleCurveAngleScaling(),
             IsClose(2.0,TEST_PREC, TEST_PREC)
  );

  mtgSet[i].calcTorqueMuscleInfo(0.,0.,0.,tmi);
  mtgSet[i].setActiveTorqueAngleCurveAngleScaling(0.1);
  mtgSet[i].calcTorqueMuscleInfo(0.,0.,0.,tmiRef);

  CHECK(std::fabs( tmi.fiberActiveTorqueAngleMultiplier
                  -tmiRef.fiberActiveTorqueAngleMultiplier) > TEST_PREC );

}

#endif

//At the present time this is not much of a test: all of the code is run and
//it is checked that each function returns true. The header has been manually
//inspected but is otherwise not checked in this test for correctness. It could
//be compared to a saved prototype header. This is a weak check, but better than
//nothing I suppose.
TEST_CASE(__FILE__"_ModelHeaderGeneration", "")
{
  RigidBodyDynamics::Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/complexmodel.lua");

  //Get the constraint set names
  std::vector<std::string> constraintSetNames =
    LuaModelGetConstraintSetNames(modelFile.c_str());
  std::vector<RigidBodyDynamics::ConstraintSet> constraintSets;

  //Get the constrained model
  std::vector< ConstraintSet > conSet;
  conSet.resize(constraintSetNames.size());
  for(unsigned int i=0; i<conSet.size();++i){
    conSet[i] = ConstraintSet();
  }
  bool constrainedModelLoaded = LuaModelReadFromFileWithConstraints(
        modelFile.c_str(),&model, conSet,constraintSetNames,false);
  CHECK(constrainedModelLoaded);

  //Get the constraint set phase ordering
  std::vector<unsigned int> phasing;
  bool constraintSetPhasingLoaded =
      LuaModelGetConstraintSetPhases(modelFile.c_str(),constraintSetNames,
                                     phasing);
  CHECK(constraintSetPhasingLoaded);

  //Get the local points
  std::vector< Point > pointSet;
  bool pointsLoaded =
      LuaModelReadPoints(modelFile.c_str(),&model,pointSet,false);
  CHECK(pointsLoaded);

  //Get the local motion capture markers
  std::vector< MotionCaptureMarker > markerSet;
  bool markersLoaded =
    LuaModelReadMotionCaptureMarkers(modelFile.c_str(),&model,markerSet,false);
  CHECK(markersLoaded);

  //Get the local frames
  std::vector< LocalFrame > localFrames;
  bool localFramesLoaded =
    LuaModelReadLocalFrames(modelFile.c_str(),&model,localFrames,false);
  CHECK(localFramesLoaded);

  //--------------------------------------------
  #ifdef RBDL_BUILD_ADDON_MUSCLE

  HumanMetaData participant_data;
  bool participantDataLoaded = LuaModelReadHumanMetaData(modelFile.c_str(),
                                participant_data,false);
  CHECK(participantDataLoaded);

  std::vector< Addons::Muscle::Millard2016TorqueMuscle > mtgSet;
  std::vector< Millard2016TorqueMuscleConfig > mtgSetInfo;
  bool mtgSetLoaded = LuaModelReadMillard2016TorqueMuscleSets(
        modelFile.c_str(), &model, participant_data, mtgSet, mtgSetInfo, false);
  CHECK(mtgSetLoaded);



  #endif
  //--------------------------------------------
  //std::string headerFile = rbdlSourcePath;
  //headerFile.append("/complexmodel.h");
  std::string headerFile("complexmodel.h");

  bool modelHeaderGenerated=
      LuaModelWriteModelHeaderEntries(headerFile.c_str(),model,false);
  CHECK(modelHeaderGenerated);

  bool pointsHeaderGenerated=
      LuaModelWritePointsHeaderEntries(headerFile.c_str(),pointSet,true);
  CHECK(pointsHeaderGenerated);

  bool markerHeaderGenerated= LuaModelWriteMotionCaptureMarkerHeaderEntries(
        headerFile.c_str(),markerSet,true);
  CHECK(markerHeaderGenerated);

  bool localFrameHeaderGenerated = LuaModelWriteLocalFrameHeaderEntries(
        headerFile.c_str(),localFrames,true);
  CHECK(localFrameHeaderGenerated);

  bool constraintSetHeaderGenerated = LuaModelWriteConstraintSetHeaderEntries(
        headerFile.c_str(),constraintSetNames,conSet,true);

  bool phasingHeaderGenerated = LuaModelWriteConstraintSetPhaseHeaderEntries(
        headerFile.c_str(), constraintSetNames, phasing,true);
  CHECK(phasingHeaderGenerated);

  //--------------------------------------------
  #ifdef RBDL_BUILD_ADDON_MUSCLE

  bool mtgHeaderGenerated = LuaModelWriteMillard2016TorqueMuscleHeaderEntries(
        headerFile.c_str(),mtgSet,mtgSetInfo,true);


  #endif
  //--------------------------------------------

  bool headerGuardsAdded = LuaModelAddHeaderGuards(headerFile.c_str());
  CHECK(headerGuardsAdded);

}
