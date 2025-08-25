// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for quad_msgs/BodyForceEstimate
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
#include "quad_msgs/msg/body_force_estimate.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class QUAD_MSGS_EXPORT ros2_quad_msgs_msg_BodyForceEstimate_common : public MATLABROS2MsgInterface<quad_msgs::msg::BodyForceEstimate> {
  public:
    virtual ~ros2_quad_msgs_msg_BodyForceEstimate_common(){}
    virtual void copy_from_struct(quad_msgs::msg::BodyForceEstimate* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const quad_msgs::msg::BodyForceEstimate* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_quad_msgs_msg_BodyForceEstimate_common::copy_from_struct(quad_msgs::msg::BodyForceEstimate* msg, const matlab::data::Struct& arr,
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
        //joint_torques
        const matlab::data::TypedArray<double> joint_torques_arr = arr["joint_torques"];
        size_t nelem = joint_torques_arr.getNumberOfElements();
        	msg->joint_torques.resize(nelem);
        	std::copy(joint_torques_arr.begin(), joint_torques_arr.begin()+nelem, msg->joint_torques.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'joint_torques' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'joint_torques' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_quad_msgs_msg_BodyForceEstimate_common::get_arr(MDFactory_T& factory, const quad_msgs::msg::BodyForceEstimate* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","joint_torques"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("quad_msgs/BodyForceEstimate");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // joint_torques
    auto currentElement_joint_torques = (msg + ctr)->joint_torques;
    outArray[ctr]["joint_torques"] = factory.createArray<quad_msgs::msg::BodyForceEstimate::_joint_torques_type::const_iterator, double>({currentElement_joint_torques.size(), 1}, currentElement_joint_torques.begin(), currentElement_joint_torques.end());
    }
    return std::move(outArray);
  } 
class QUAD_MSGS_EXPORT ros2_quad_msgs_BodyForceEstimate_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_quad_msgs_BodyForceEstimate_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_quad_msgs_BodyForceEstimate_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<quad_msgs::msg::BodyForceEstimate,ros2_quad_msgs_msg_BodyForceEstimate_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_quad_msgs_BodyForceEstimate_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<quad_msgs::msg::BodyForceEstimate,ros2_quad_msgs_msg_BodyForceEstimate_common>>();
  }
  std::shared_ptr<void> ros2_quad_msgs_BodyForceEstimate_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<quad_msgs::msg::BodyForceEstimate>();
    ros2_quad_msgs_msg_BodyForceEstimate_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_quad_msgs_BodyForceEstimate_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_quad_msgs_msg_BodyForceEstimate_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (quad_msgs::msg::BodyForceEstimate*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_msg_BodyForceEstimate_common, MATLABROS2MsgInterface<quad_msgs::msg::BodyForceEstimate>)
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_BodyForceEstimate_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER