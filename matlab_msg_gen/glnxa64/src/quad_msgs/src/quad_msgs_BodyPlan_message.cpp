// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for quad_msgs/BodyPlan
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
#include "quad_msgs/msg/body_plan.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class QUAD_MSGS_EXPORT ros2_quad_msgs_msg_BodyPlan_common : public MATLABROS2MsgInterface<quad_msgs::msg::BodyPlan> {
  public:
    virtual ~ros2_quad_msgs_msg_BodyPlan_common(){}
    virtual void copy_from_struct(quad_msgs::msg::BodyPlan* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const quad_msgs::msg::BodyPlan* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_quad_msgs_msg_BodyPlan_common::copy_from_struct(quad_msgs::msg::BodyPlan* msg, const matlab::data::Struct& arr,
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
        //plan_indices
        const matlab::data::TypedArray<uint32_t> plan_indices_arr = arr["plan_indices"];
        size_t nelem = plan_indices_arr.getNumberOfElements();
        	msg->plan_indices.resize(nelem);
        	std::copy(plan_indices_arr.begin(), plan_indices_arr.begin()+nelem, msg->plan_indices.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'plan_indices' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'plan_indices' is wrong type; expected a uint32.");
    }
    try {
        //primitive_ids
        const matlab::data::TypedArray<uint32_t> primitive_ids_arr = arr["primitive_ids"];
        size_t nelem = primitive_ids_arr.getNumberOfElements();
        	msg->primitive_ids.resize(nelem);
        	std::copy(primitive_ids_arr.begin(), primitive_ids_arr.begin()+nelem, msg->primitive_ids.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'primitive_ids' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'primitive_ids' is wrong type; expected a uint32.");
    }
    try {
        //states
        const matlab::data::StructArray states_arr = arr["states"];
        for (auto _statesarr : states_arr) {
        	nav_msgs::msg::Odometry _val;
        auto msgClassPtr_states = getCommonObject<nav_msgs::msg::Odometry>("ros2_nav_msgs_msg_Odometry_common",loader);
        msgClassPtr_states->copy_from_struct(&_val,_statesarr,loader);
        	msg->states.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'states' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'states' is wrong type; expected a struct.");
    }
    try {
        //grfs
        const matlab::data::StructArray grfs_arr = arr["grfs"];
        for (auto _grfsarr : grfs_arr) {
        	quad_msgs::msg::GRFArray _val;
        auto msgClassPtr_grfs = getCommonObject<quad_msgs::msg::GRFArray>("ros2_quad_msgs_msg_GRFArray_common",loader);
        msgClassPtr_grfs->copy_from_struct(&_val,_grfsarr,loader);
        	msg->grfs.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'grfs' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'grfs' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_quad_msgs_msg_BodyPlan_common::get_arr(MDFactory_T& factory, const quad_msgs::msg::BodyPlan* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","plan_indices","primitive_ids","states","grfs"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("quad_msgs/BodyPlan");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // plan_indices
    auto currentElement_plan_indices = (msg + ctr)->plan_indices;
    outArray[ctr]["plan_indices"] = factory.createArray<quad_msgs::msg::BodyPlan::_plan_indices_type::const_iterator, uint32_t>({currentElement_plan_indices.size(), 1}, currentElement_plan_indices.begin(), currentElement_plan_indices.end());
    // primitive_ids
    auto currentElement_primitive_ids = (msg + ctr)->primitive_ids;
    outArray[ctr]["primitive_ids"] = factory.createArray<quad_msgs::msg::BodyPlan::_primitive_ids_type::const_iterator, uint32_t>({currentElement_primitive_ids.size(), 1}, currentElement_primitive_ids.begin(), currentElement_primitive_ids.end());
    // states
    auto currentElement_states = (msg + ctr)->states;
    auto msgClassPtr_states = getCommonObject<nav_msgs::msg::Odometry>("ros2_nav_msgs_msg_Odometry_common",loader);
    outArray[ctr]["states"] = msgClassPtr_states->get_arr(factory,&currentElement_states[0],loader,currentElement_states.size());
    // grfs
    auto currentElement_grfs = (msg + ctr)->grfs;
    auto msgClassPtr_grfs = getCommonObject<quad_msgs::msg::GRFArray>("ros2_quad_msgs_msg_GRFArray_common",loader);
    outArray[ctr]["grfs"] = msgClassPtr_grfs->get_arr(factory,&currentElement_grfs[0],loader,currentElement_grfs.size());
    }
    return std::move(outArray);
  } 
class QUAD_MSGS_EXPORT ros2_quad_msgs_BodyPlan_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_quad_msgs_BodyPlan_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_quad_msgs_BodyPlan_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<quad_msgs::msg::BodyPlan,ros2_quad_msgs_msg_BodyPlan_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_quad_msgs_BodyPlan_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<quad_msgs::msg::BodyPlan,ros2_quad_msgs_msg_BodyPlan_common>>();
  }
  std::shared_ptr<void> ros2_quad_msgs_BodyPlan_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<quad_msgs::msg::BodyPlan>();
    ros2_quad_msgs_msg_BodyPlan_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_quad_msgs_BodyPlan_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_quad_msgs_msg_BodyPlan_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (quad_msgs::msg::BodyPlan*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_msg_BodyPlan_common, MATLABROS2MsgInterface<quad_msgs::msg::BodyPlan>)
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_BodyPlan_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER