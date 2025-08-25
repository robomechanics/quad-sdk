// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for quad_msgs/MultiFootPlanContinuous
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
#include "quad_msgs/msg/multi_foot_plan_continuous.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class QUAD_MSGS_EXPORT ros2_quad_msgs_msg_MultiFootPlanContinuous_common : public MATLABROS2MsgInterface<quad_msgs::msg::MultiFootPlanContinuous> {
  public:
    virtual ~ros2_quad_msgs_msg_MultiFootPlanContinuous_common(){}
    virtual void copy_from_struct(quad_msgs::msg::MultiFootPlanContinuous* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const quad_msgs::msg::MultiFootPlanContinuous* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_quad_msgs_msg_MultiFootPlanContinuous_common::copy_from_struct(quad_msgs::msg::MultiFootPlanContinuous* msg, const matlab::data::Struct& arr,
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
        //states
        const matlab::data::StructArray states_arr = arr["states"];
        for (auto _statesarr : states_arr) {
        	quad_msgs::msg::MultiFootState _val;
        auto msgClassPtr_states = getCommonObject<quad_msgs::msg::MultiFootState>("ros2_quad_msgs_msg_MultiFootState_common",loader);
        msgClassPtr_states->copy_from_struct(&_val,_statesarr,loader);
        	msg->states.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'states' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'states' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_quad_msgs_msg_MultiFootPlanContinuous_common::get_arr(MDFactory_T& factory, const quad_msgs::msg::MultiFootPlanContinuous* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","states"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("quad_msgs/MultiFootPlanContinuous");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // states
    auto currentElement_states = (msg + ctr)->states;
    auto msgClassPtr_states = getCommonObject<quad_msgs::msg::MultiFootState>("ros2_quad_msgs_msg_MultiFootState_common",loader);
    outArray[ctr]["states"] = msgClassPtr_states->get_arr(factory,&currentElement_states[0],loader,currentElement_states.size());
    }
    return std::move(outArray);
  } 
class QUAD_MSGS_EXPORT ros2_quad_msgs_MultiFootPlanContinuous_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_quad_msgs_MultiFootPlanContinuous_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_quad_msgs_MultiFootPlanContinuous_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<quad_msgs::msg::MultiFootPlanContinuous,ros2_quad_msgs_msg_MultiFootPlanContinuous_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_quad_msgs_MultiFootPlanContinuous_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<quad_msgs::msg::MultiFootPlanContinuous,ros2_quad_msgs_msg_MultiFootPlanContinuous_common>>();
  }
  std::shared_ptr<void> ros2_quad_msgs_MultiFootPlanContinuous_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<quad_msgs::msg::MultiFootPlanContinuous>();
    ros2_quad_msgs_msg_MultiFootPlanContinuous_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_quad_msgs_MultiFootPlanContinuous_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_quad_msgs_msg_MultiFootPlanContinuous_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (quad_msgs::msg::MultiFootPlanContinuous*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_msg_MultiFootPlanContinuous_common, MATLABROS2MsgInterface<quad_msgs::msg::MultiFootPlanContinuous>)
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_MultiFootPlanContinuous_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER