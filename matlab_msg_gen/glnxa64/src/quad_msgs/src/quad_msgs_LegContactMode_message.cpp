// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for quad_msgs/LegContactMode
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
#include "quad_msgs/msg/leg_contact_mode.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class QUAD_MSGS_EXPORT ros2_quad_msgs_msg_LegContactMode_common : public MATLABROS2MsgInterface<quad_msgs::msg::LegContactMode> {
  public:
    virtual ~ros2_quad_msgs_msg_LegContactMode_common(){}
    virtual void copy_from_struct(quad_msgs::msg::LegContactMode* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const quad_msgs::msg::LegContactMode* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_quad_msgs_msg_LegContactMode_common::copy_from_struct(quad_msgs::msg::LegContactMode* msg, const matlab::data::Struct& arr,
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
        //contact_prob
        const matlab::data::TypedArray<float> contact_prob_arr = arr["contact_prob"];
        msg->contact_prob = contact_prob_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'contact_prob' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'contact_prob' is wrong type; expected a single.");
    }
    try {
        //contact_state
        const matlab::data::TypedArray<bool> contact_state_arr = arr["contact_state"];
        msg->contact_state = contact_state_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'contact_state' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'contact_state' is wrong type; expected a logical.");
    }
    try {
        //contact_forces
        const matlab::data::StructArray contact_forces_arr = arr["contact_forces"];
        auto msgClassPtr_contact_forces = getCommonObject<geometry_msgs::msg::Vector3>("ros2_geometry_msgs_msg_Vector3_common",loader);
        msgClassPtr_contact_forces->copy_from_struct(&msg->contact_forces,contact_forces_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'contact_forces' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'contact_forces' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_quad_msgs_msg_LegContactMode_common::get_arr(MDFactory_T& factory, const quad_msgs::msg::LegContactMode* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","contact_prob","contact_state","contact_forces"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("quad_msgs/LegContactMode");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // contact_prob
    auto currentElement_contact_prob = (msg + ctr)->contact_prob;
    outArray[ctr]["contact_prob"] = factory.createScalar(currentElement_contact_prob);
    // contact_state
    auto currentElement_contact_state = (msg + ctr)->contact_state;
    outArray[ctr]["contact_state"] = factory.createScalar(currentElement_contact_state);
    // contact_forces
    auto currentElement_contact_forces = (msg + ctr)->contact_forces;
    auto msgClassPtr_contact_forces = getCommonObject<geometry_msgs::msg::Vector3>("ros2_geometry_msgs_msg_Vector3_common",loader);
    outArray[ctr]["contact_forces"] = msgClassPtr_contact_forces->get_arr(factory, &currentElement_contact_forces, loader);
    }
    return std::move(outArray);
  } 
class QUAD_MSGS_EXPORT ros2_quad_msgs_LegContactMode_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_quad_msgs_LegContactMode_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_quad_msgs_LegContactMode_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<quad_msgs::msg::LegContactMode,ros2_quad_msgs_msg_LegContactMode_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_quad_msgs_LegContactMode_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<quad_msgs::msg::LegContactMode,ros2_quad_msgs_msg_LegContactMode_common>>();
  }
  std::shared_ptr<void> ros2_quad_msgs_LegContactMode_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<quad_msgs::msg::LegContactMode>();
    ros2_quad_msgs_msg_LegContactMode_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_quad_msgs_LegContactMode_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_quad_msgs_msg_LegContactMode_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (quad_msgs::msg::LegContactMode*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_msg_LegContactMode_common, MATLABROS2MsgInterface<quad_msgs::msg::LegContactMode>)
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_LegContactMode_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER