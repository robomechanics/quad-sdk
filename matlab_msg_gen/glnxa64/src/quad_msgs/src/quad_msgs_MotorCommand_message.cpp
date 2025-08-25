// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for quad_msgs/MotorCommand
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
#include "quad_msgs/msg/motor_command.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class QUAD_MSGS_EXPORT ros2_quad_msgs_msg_MotorCommand_common : public MATLABROS2MsgInterface<quad_msgs::msg::MotorCommand> {
  public:
    virtual ~ros2_quad_msgs_msg_MotorCommand_common(){}
    virtual void copy_from_struct(quad_msgs::msg::MotorCommand* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const quad_msgs::msg::MotorCommand* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_quad_msgs_msg_MotorCommand_common::copy_from_struct(quad_msgs::msg::MotorCommand* msg, const matlab::data::Struct& arr,
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
        //pos_setpoint
        const matlab::data::TypedArray<double> pos_setpoint_arr = arr["pos_setpoint"];
        msg->pos_setpoint = pos_setpoint_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'pos_setpoint' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'pos_setpoint' is wrong type; expected a double.");
    }
    try {
        //vel_setpoint
        const matlab::data::TypedArray<double> vel_setpoint_arr = arr["vel_setpoint"];
        msg->vel_setpoint = vel_setpoint_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'vel_setpoint' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'vel_setpoint' is wrong type; expected a double.");
    }
    try {
        //kp
        const matlab::data::TypedArray<float> kp_arr = arr["kp"];
        msg->kp = kp_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'kp' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'kp' is wrong type; expected a single.");
    }
    try {
        //kd
        const matlab::data::TypedArray<float> kd_arr = arr["kd"];
        msg->kd = kd_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'kd' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'kd' is wrong type; expected a single.");
    }
    try {
        //torque_ff
        const matlab::data::TypedArray<double> torque_ff_arr = arr["torque_ff"];
        msg->torque_ff = torque_ff_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'torque_ff' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'torque_ff' is wrong type; expected a double.");
    }
    try {
        //pos_component
        const matlab::data::TypedArray<double> pos_component_arr = arr["pos_component"];
        msg->pos_component = pos_component_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'pos_component' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'pos_component' is wrong type; expected a double.");
    }
    try {
        //vel_component
        const matlab::data::TypedArray<double> vel_component_arr = arr["vel_component"];
        msg->vel_component = vel_component_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'vel_component' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'vel_component' is wrong type; expected a double.");
    }
    try {
        //fb_component
        const matlab::data::TypedArray<double> fb_component_arr = arr["fb_component"];
        msg->fb_component = fb_component_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'fb_component' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'fb_component' is wrong type; expected a double.");
    }
    try {
        //effort
        const matlab::data::TypedArray<double> effort_arr = arr["effort"];
        msg->effort = effort_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'effort' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'effort' is wrong type; expected a double.");
    }
    try {
        //fb_ratio
        const matlab::data::TypedArray<double> fb_ratio_arr = arr["fb_ratio"];
        msg->fb_ratio = fb_ratio_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'fb_ratio' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'fb_ratio' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_quad_msgs_msg_MotorCommand_common::get_arr(MDFactory_T& factory, const quad_msgs::msg::MotorCommand* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","pos_setpoint","vel_setpoint","kp","kd","torque_ff","pos_component","vel_component","fb_component","effort","fb_ratio"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("quad_msgs/MotorCommand");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // pos_setpoint
    auto currentElement_pos_setpoint = (msg + ctr)->pos_setpoint;
    outArray[ctr]["pos_setpoint"] = factory.createScalar(currentElement_pos_setpoint);
    // vel_setpoint
    auto currentElement_vel_setpoint = (msg + ctr)->vel_setpoint;
    outArray[ctr]["vel_setpoint"] = factory.createScalar(currentElement_vel_setpoint);
    // kp
    auto currentElement_kp = (msg + ctr)->kp;
    outArray[ctr]["kp"] = factory.createScalar(currentElement_kp);
    // kd
    auto currentElement_kd = (msg + ctr)->kd;
    outArray[ctr]["kd"] = factory.createScalar(currentElement_kd);
    // torque_ff
    auto currentElement_torque_ff = (msg + ctr)->torque_ff;
    outArray[ctr]["torque_ff"] = factory.createScalar(currentElement_torque_ff);
    // pos_component
    auto currentElement_pos_component = (msg + ctr)->pos_component;
    outArray[ctr]["pos_component"] = factory.createScalar(currentElement_pos_component);
    // vel_component
    auto currentElement_vel_component = (msg + ctr)->vel_component;
    outArray[ctr]["vel_component"] = factory.createScalar(currentElement_vel_component);
    // fb_component
    auto currentElement_fb_component = (msg + ctr)->fb_component;
    outArray[ctr]["fb_component"] = factory.createScalar(currentElement_fb_component);
    // effort
    auto currentElement_effort = (msg + ctr)->effort;
    outArray[ctr]["effort"] = factory.createScalar(currentElement_effort);
    // fb_ratio
    auto currentElement_fb_ratio = (msg + ctr)->fb_ratio;
    outArray[ctr]["fb_ratio"] = factory.createScalar(currentElement_fb_ratio);
    }
    return std::move(outArray);
  } 
class QUAD_MSGS_EXPORT ros2_quad_msgs_MotorCommand_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_quad_msgs_MotorCommand_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_quad_msgs_MotorCommand_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<quad_msgs::msg::MotorCommand,ros2_quad_msgs_msg_MotorCommand_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_quad_msgs_MotorCommand_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<quad_msgs::msg::MotorCommand,ros2_quad_msgs_msg_MotorCommand_common>>();
  }
  std::shared_ptr<void> ros2_quad_msgs_MotorCommand_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<quad_msgs::msg::MotorCommand>();
    ros2_quad_msgs_msg_MotorCommand_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_quad_msgs_MotorCommand_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_quad_msgs_msg_MotorCommand_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (quad_msgs::msg::MotorCommand*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_msg_MotorCommand_common, MATLABROS2MsgInterface<quad_msgs::msg::MotorCommand>)
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_MotorCommand_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER