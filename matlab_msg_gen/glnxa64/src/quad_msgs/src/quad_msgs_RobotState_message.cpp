// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for quad_msgs/RobotState
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4100)
#pragma warning(disable : 4265)
#pragma warning(disable : 4456)
#pragma warning(disable : 4458)
#pragma warning(disable : 4946)
#pragma warning(disable : 4244)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif //_MSC_VER
#include "rclcpp/rclcpp.hpp"
#include "quad_msgs/msg/robot_state.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class QUAD_MSGS_EXPORT ros2_quad_msgs_msg_RobotState_common : public MATLABROS2MsgInterface<quad_msgs::msg::RobotState> {
  public:
    virtual ~ros2_quad_msgs_msg_RobotState_common(){}
    virtual void copy_from_struct(quad_msgs::msg::RobotState* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const quad_msgs::msg::RobotState* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_quad_msgs_msg_RobotState_common::copy_from_struct(quad_msgs::msg::RobotState* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //header
        const matlab::data::StructArray header_arr = arr["header"];
        auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
        msgClassPtr_header->copy_from_struct(&msg->header,header_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'header' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'header' is wrong type; expected a struct.");
    }
    try {
        //traj_index
        const matlab::data::TypedArray<uint32_t> traj_index_arr = arr["traj_index"];
        msg->traj_index = traj_index_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'traj_index' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'traj_index' is wrong type; expected a uint32.");
    }
    try {
        //body
        const matlab::data::StructArray body_arr = arr["body"];
        auto msgClassPtr_body = getCommonObject<quad_msgs::msg::BodyState>("ros2_quad_msgs_msg_BodyState_common",loader);
        msgClassPtr_body->copy_from_struct(&msg->body,body_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'body' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'body' is wrong type; expected a struct.");
    }
    try {
        //joints
        const matlab::data::StructArray joints_arr = arr["joints"];
        auto msgClassPtr_joints = getCommonObject<sensor_msgs::msg::JointState>("ros2_sensor_msgs_msg_JointState_common",loader);
        msgClassPtr_joints->copy_from_struct(&msg->joints,joints_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'joints' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'joints' is wrong type; expected a struct.");
    }
    try {
        //feet
        const matlab::data::StructArray feet_arr = arr["feet"];
        auto msgClassPtr_feet = getCommonObject<quad_msgs::msg::MultiFootState>("ros2_quad_msgs_msg_MultiFootState_common",loader);
        msgClassPtr_feet->copy_from_struct(&msg->feet,feet_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'feet' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'feet' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_quad_msgs_msg_RobotState_common::get_arr(MDFactory_T& factory, const quad_msgs::msg::RobotState* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","traj_index","body","joints","feet"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("quad_msgs/RobotState");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // traj_index
    auto currentElement_traj_index = (msg + ctr)->traj_index;
    outArray[ctr]["traj_index"] = factory.createScalar(currentElement_traj_index);
    // body
    auto currentElement_body = (msg + ctr)->body;
    auto msgClassPtr_body = getCommonObject<quad_msgs::msg::BodyState>("ros2_quad_msgs_msg_BodyState_common",loader);
    outArray[ctr]["body"] = msgClassPtr_body->get_arr(factory, &currentElement_body, loader);
    // joints
    auto currentElement_joints = (msg + ctr)->joints;
    auto msgClassPtr_joints = getCommonObject<sensor_msgs::msg::JointState>("ros2_sensor_msgs_msg_JointState_common",loader);
    outArray[ctr]["joints"] = msgClassPtr_joints->get_arr(factory, &currentElement_joints, loader);
    // feet
    auto currentElement_feet = (msg + ctr)->feet;
    auto msgClassPtr_feet = getCommonObject<quad_msgs::msg::MultiFootState>("ros2_quad_msgs_msg_MultiFootState_common",loader);
    outArray[ctr]["feet"] = msgClassPtr_feet->get_arr(factory, &currentElement_feet, loader);
    }
    return std::move(outArray);
  } 
class QUAD_MSGS_EXPORT ros2_quad_msgs_RobotState_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_quad_msgs_RobotState_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_quad_msgs_RobotState_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<quad_msgs::msg::RobotState,ros2_quad_msgs_msg_RobotState_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_quad_msgs_RobotState_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<quad_msgs::msg::RobotState,ros2_quad_msgs_msg_RobotState_common>>();
  }
  std::shared_ptr<void> ros2_quad_msgs_RobotState_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<quad_msgs::msg::RobotState>();
    ros2_quad_msgs_msg_RobotState_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_quad_msgs_RobotState_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_quad_msgs_msg_RobotState_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (quad_msgs::msg::RobotState*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_msg_RobotState_common, MATLABROS2MsgInterface<quad_msgs::msg::RobotState>)
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_RobotState_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER