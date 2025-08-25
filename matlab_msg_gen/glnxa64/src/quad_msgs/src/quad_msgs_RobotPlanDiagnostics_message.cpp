// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for quad_msgs/RobotPlanDiagnostics
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
#include "quad_msgs/msg/robot_plan_diagnostics.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class QUAD_MSGS_EXPORT ros2_quad_msgs_msg_RobotPlanDiagnostics_common : public MATLABROS2MsgInterface<quad_msgs::msg::RobotPlanDiagnostics> {
  public:
    virtual ~ros2_quad_msgs_msg_RobotPlanDiagnostics_common(){}
    virtual void copy_from_struct(quad_msgs::msg::RobotPlanDiagnostics* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const quad_msgs::msg::RobotPlanDiagnostics* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_quad_msgs_msg_RobotPlanDiagnostics_common::copy_from_struct(quad_msgs::msg::RobotPlanDiagnostics* msg, const matlab::data::Struct& arr,
               MultiLibLoader loader) {
    try {
        //compute_time
        const matlab::data::TypedArray<double> compute_time_arr = arr["compute_time"];
        msg->compute_time = compute_time_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'compute_time' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'compute_time' is wrong type; expected a double.");
    }
    try {
        //cost
        const matlab::data::TypedArray<double> cost_arr = arr["cost"];
        msg->cost = cost_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'cost' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'cost' is wrong type; expected a double.");
    }
    try {
        //iterations
        const matlab::data::TypedArray<uint32_t> iterations_arr = arr["iterations"];
        msg->iterations = iterations_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'iterations' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'iterations' is wrong type; expected a uint32.");
    }
    try {
        //horizon_length
        const matlab::data::TypedArray<uint32_t> horizon_length_arr = arr["horizon_length"];
        msg->horizon_length = horizon_length_arr[0];
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'horizon_length' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'horizon_length' is wrong type; expected a uint32.");
    }
    try {
        //complexity_schedule
        const matlab::data::TypedArray<uint32_t> complexity_schedule_arr = arr["complexity_schedule"];
        size_t nelem = complexity_schedule_arr.getNumberOfElements();
        	msg->complexity_schedule.resize(nelem);
        	std::copy(complexity_schedule_arr.begin(), complexity_schedule_arr.begin()+nelem, msg->complexity_schedule.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'complexity_schedule' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'complexity_schedule' is wrong type; expected a uint32.");
    }
    try {
        //element_times
        const matlab::data::TypedArray<double> element_times_arr = arr["element_times"];
        size_t nelem = element_times_arr.getNumberOfElements();
        	msg->element_times.resize(nelem);
        	std::copy(element_times_arr.begin(), element_times_arr.begin()+nelem, msg->element_times.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'element_times' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'element_times' is wrong type; expected a double.");
    }
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_quad_msgs_msg_RobotPlanDiagnostics_common::get_arr(MDFactory_T& factory, const quad_msgs::msg::RobotPlanDiagnostics* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","compute_time","cost","iterations","horizon_length","complexity_schedule","element_times"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("quad_msgs/RobotPlanDiagnostics");
    // compute_time
    auto currentElement_compute_time = (msg + ctr)->compute_time;
    outArray[ctr]["compute_time"] = factory.createScalar(currentElement_compute_time);
    // cost
    auto currentElement_cost = (msg + ctr)->cost;
    outArray[ctr]["cost"] = factory.createScalar(currentElement_cost);
    // iterations
    auto currentElement_iterations = (msg + ctr)->iterations;
    outArray[ctr]["iterations"] = factory.createScalar(currentElement_iterations);
    // horizon_length
    auto currentElement_horizon_length = (msg + ctr)->horizon_length;
    outArray[ctr]["horizon_length"] = factory.createScalar(currentElement_horizon_length);
    // complexity_schedule
    auto currentElement_complexity_schedule = (msg + ctr)->complexity_schedule;
    outArray[ctr]["complexity_schedule"] = factory.createArray<quad_msgs::msg::RobotPlanDiagnostics::_complexity_schedule_type::const_iterator, uint32_t>({currentElement_complexity_schedule.size(), 1}, currentElement_complexity_schedule.begin(), currentElement_complexity_schedule.end());
    // element_times
    auto currentElement_element_times = (msg + ctr)->element_times;
    outArray[ctr]["element_times"] = factory.createArray<quad_msgs::msg::RobotPlanDiagnostics::_element_times_type::const_iterator, double>({currentElement_element_times.size(), 1}, currentElement_element_times.begin(), currentElement_element_times.end());
    }
    return std::move(outArray);
  } 
class QUAD_MSGS_EXPORT ros2_quad_msgs_RobotPlanDiagnostics_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_quad_msgs_RobotPlanDiagnostics_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_quad_msgs_RobotPlanDiagnostics_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<quad_msgs::msg::RobotPlanDiagnostics,ros2_quad_msgs_msg_RobotPlanDiagnostics_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_quad_msgs_RobotPlanDiagnostics_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<quad_msgs::msg::RobotPlanDiagnostics,ros2_quad_msgs_msg_RobotPlanDiagnostics_common>>();
  }
  std::shared_ptr<void> ros2_quad_msgs_RobotPlanDiagnostics_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<quad_msgs::msg::RobotPlanDiagnostics>();
    ros2_quad_msgs_msg_RobotPlanDiagnostics_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_quad_msgs_RobotPlanDiagnostics_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_quad_msgs_msg_RobotPlanDiagnostics_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (quad_msgs::msg::RobotPlanDiagnostics*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_msg_RobotPlanDiagnostics_common, MATLABROS2MsgInterface<quad_msgs::msg::RobotPlanDiagnostics>)
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_RobotPlanDiagnostics_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER