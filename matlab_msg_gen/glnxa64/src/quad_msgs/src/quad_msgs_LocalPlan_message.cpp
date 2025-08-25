// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for quad_msgs/LocalPlan
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
#include "quad_msgs/msg/local_plan.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class QUAD_MSGS_EXPORT ros2_quad_msgs_msg_LocalPlan_common : public MATLABROS2MsgInterface<quad_msgs::msg::LocalPlan> {
  public:
    virtual ~ros2_quad_msgs_msg_LocalPlan_common(){}
    virtual void copy_from_struct(quad_msgs::msg::LocalPlan* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const quad_msgs::msg::LocalPlan* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_quad_msgs_msg_LocalPlan_common::copy_from_struct(quad_msgs::msg::LocalPlan* msg, const matlab::data::Struct& arr,
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
        	quad_msgs::msg::RobotState _val;
        auto msgClassPtr_states = getCommonObject<quad_msgs::msg::RobotState>("ros2_quad_msgs_msg_RobotState_common",loader);
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
        //global_plan_timestamp
        const matlab::data::StructArray global_plan_timestamp_arr = arr["global_plan_timestamp"];
        auto msgClassPtr_global_plan_timestamp = getCommonObject<builtin_interfaces::msg::Time>("ros2_builtin_interfaces_msg_Time_common",loader);
        msgClassPtr_global_plan_timestamp->copy_from_struct(&msg->global_plan_timestamp,global_plan_timestamp_arr[0],loader);
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'global_plan_timestamp' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'global_plan_timestamp' is wrong type; expected a struct.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_quad_msgs_msg_LocalPlan_common::get_arr(MDFactory_T& factory, const quad_msgs::msg::LocalPlan* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","states","grfs","plan_indices","global_plan_timestamp"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("quad_msgs/LocalPlan");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // states
    auto currentElement_states = (msg + ctr)->states;
    auto msgClassPtr_states = getCommonObject<quad_msgs::msg::RobotState>("ros2_quad_msgs_msg_RobotState_common",loader);
    outArray[ctr]["states"] = msgClassPtr_states->get_arr(factory,&currentElement_states[0],loader,currentElement_states.size());
    // grfs
    auto currentElement_grfs = (msg + ctr)->grfs;
    auto msgClassPtr_grfs = getCommonObject<quad_msgs::msg::GRFArray>("ros2_quad_msgs_msg_GRFArray_common",loader);
    outArray[ctr]["grfs"] = msgClassPtr_grfs->get_arr(factory,&currentElement_grfs[0],loader,currentElement_grfs.size());
    // plan_indices
    auto currentElement_plan_indices = (msg + ctr)->plan_indices;
    outArray[ctr]["plan_indices"] = factory.createArray<quad_msgs::msg::LocalPlan::_plan_indices_type::const_iterator, uint32_t>({currentElement_plan_indices.size(), 1}, currentElement_plan_indices.begin(), currentElement_plan_indices.end());
    // global_plan_timestamp
    auto currentElement_global_plan_timestamp = (msg + ctr)->global_plan_timestamp;
    auto msgClassPtr_global_plan_timestamp = getCommonObject<builtin_interfaces::msg::Time>("ros2_builtin_interfaces_msg_Time_common",loader);
    outArray[ctr]["global_plan_timestamp"] = msgClassPtr_global_plan_timestamp->get_arr(factory, &currentElement_global_plan_timestamp, loader);
    }
    return std::move(outArray);
  } 
class QUAD_MSGS_EXPORT ros2_quad_msgs_LocalPlan_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_quad_msgs_LocalPlan_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_quad_msgs_LocalPlan_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<quad_msgs::msg::LocalPlan,ros2_quad_msgs_msg_LocalPlan_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_quad_msgs_LocalPlan_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<quad_msgs::msg::LocalPlan,ros2_quad_msgs_msg_LocalPlan_common>>();
  }
  std::shared_ptr<void> ros2_quad_msgs_LocalPlan_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<quad_msgs::msg::LocalPlan>();
    ros2_quad_msgs_msg_LocalPlan_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_quad_msgs_LocalPlan_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_quad_msgs_msg_LocalPlan_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (quad_msgs::msg::LocalPlan*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_msg_LocalPlan_common, MATLABROS2MsgInterface<quad_msgs::msg::LocalPlan>)
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_LocalPlan_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER