// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for quad_msgs/FootPlanDiscrete
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
#include "quad_msgs/msg/foot_plan_discrete.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class QUAD_MSGS_EXPORT ros2_quad_msgs_msg_FootPlanDiscrete_common : public MATLABROS2MsgInterface<quad_msgs::msg::FootPlanDiscrete> {
  public:
    virtual ~ros2_quad_msgs_msg_FootPlanDiscrete_common(){}
    virtual void copy_from_struct(quad_msgs::msg::FootPlanDiscrete* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const quad_msgs::msg::FootPlanDiscrete* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_quad_msgs_msg_FootPlanDiscrete_common::copy_from_struct(quad_msgs::msg::FootPlanDiscrete* msg, const matlab::data::Struct& arr,
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
        //footholds
        const matlab::data::StructArray footholds_arr = arr["footholds"];
        for (auto _footholdsarr : footholds_arr) {
        	quad_msgs::msg::FootState _val;
        auto msgClassPtr_footholds = getCommonObject<quad_msgs::msg::FootState>("ros2_quad_msgs_msg_FootState_common",loader);
        msgClassPtr_footholds->copy_from_struct(&_val,_footholdsarr,loader);
        	msg->footholds.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'footholds' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'footholds' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_quad_msgs_msg_FootPlanDiscrete_common::get_arr(MDFactory_T& factory, const quad_msgs::msg::FootPlanDiscrete* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","footholds"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("quad_msgs/FootPlanDiscrete");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // footholds
    auto currentElement_footholds = (msg + ctr)->footholds;
    auto msgClassPtr_footholds = getCommonObject<quad_msgs::msg::FootState>("ros2_quad_msgs_msg_FootState_common",loader);
    outArray[ctr]["footholds"] = msgClassPtr_footholds->get_arr(factory,&currentElement_footholds[0],loader,currentElement_footholds.size());
    }
    return std::move(outArray);
  } 
class QUAD_MSGS_EXPORT ros2_quad_msgs_FootPlanDiscrete_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_quad_msgs_FootPlanDiscrete_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_quad_msgs_FootPlanDiscrete_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<quad_msgs::msg::FootPlanDiscrete,ros2_quad_msgs_msg_FootPlanDiscrete_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_quad_msgs_FootPlanDiscrete_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<quad_msgs::msg::FootPlanDiscrete,ros2_quad_msgs_msg_FootPlanDiscrete_common>>();
  }
  std::shared_ptr<void> ros2_quad_msgs_FootPlanDiscrete_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<quad_msgs::msg::FootPlanDiscrete>();
    ros2_quad_msgs_msg_FootPlanDiscrete_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_quad_msgs_FootPlanDiscrete_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_quad_msgs_msg_FootPlanDiscrete_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (quad_msgs::msg::FootPlanDiscrete*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_msg_FootPlanDiscrete_common, MATLABROS2MsgInterface<quad_msgs::msg::FootPlanDiscrete>)
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_FootPlanDiscrete_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER