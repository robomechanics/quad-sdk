// Copyright 2020-2022 The MathWorks, Inc.
// Common copy functions for quad_msgs/GRFArray
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
#include "quad_msgs/msg/grf_array.hpp"
#include "visibility_control.h"
#include "class_loader/multi_library_class_loader.hpp"
#include "ROS2PubSubTemplates.hpp"
class QUAD_MSGS_EXPORT ros2_quad_msgs_msg_GRFArray_common : public MATLABROS2MsgInterface<quad_msgs::msg::GRFArray> {
  public:
    virtual ~ros2_quad_msgs_msg_GRFArray_common(){}
    virtual void copy_from_struct(quad_msgs::msg::GRFArray* msg, const matlab::data::Struct& arr, MultiLibLoader loader); 
    //----------------------------------------------------------------------------
    virtual MDArray_T get_arr(MDFactory_T& factory, const quad_msgs::msg::GRFArray* msg, MultiLibLoader loader, size_t size = 1);
};
  void ros2_quad_msgs_msg_GRFArray_common::copy_from_struct(quad_msgs::msg::GRFArray* msg, const matlab::data::Struct& arr,
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
        //vectors
        const matlab::data::StructArray vectors_arr = arr["vectors"];
        for (auto _vectorsarr : vectors_arr) {
        	geometry_msgs::msg::Vector3 _val;
        auto msgClassPtr_vectors = getCommonObject<geometry_msgs::msg::Vector3>("ros2_geometry_msgs_msg_Vector3_common",loader);
        msgClassPtr_vectors->copy_from_struct(&_val,_vectorsarr,loader);
        	msg->vectors.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'vectors' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'vectors' is wrong type; expected a struct.");
    }
    try {
        //points
        const matlab::data::StructArray points_arr = arr["points"];
        for (auto _pointsarr : points_arr) {
        	geometry_msgs::msg::Point _val;
        auto msgClassPtr_points = getCommonObject<geometry_msgs::msg::Point>("ros2_geometry_msgs_msg_Point_common",loader);
        msgClassPtr_points->copy_from_struct(&_val,_pointsarr,loader);
        	msg->points.push_back(_val);
        }
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'points' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'points' is wrong type; expected a struct.");
    }
    try {
        //contact_states
        const matlab::data::TypedArray<bool> contact_states_arr = arr["contact_states"];
        size_t nelem = contact_states_arr.getNumberOfElements();
        	msg->contact_states.resize(nelem);
        	std::copy(contact_states_arr.begin(), contact_states_arr.begin()+nelem, msg->contact_states.begin());
    } catch (matlab::data::InvalidFieldNameException&) {
        throw std::invalid_argument("Field 'contact_states' is missing.");
    } catch (matlab::Exception&) {
        throw std::invalid_argument("Field 'contact_states' is wrong type; expected a logical.");
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
  }
  //----------------------------------------------------------------------------
  MDArray_T ros2_quad_msgs_msg_GRFArray_common::get_arr(MDFactory_T& factory, const quad_msgs::msg::GRFArray* msg,
       MultiLibLoader loader, size_t size) {
    auto outArray = factory.createStructArray({size,1},{"MessageType","header","vectors","points","contact_states","traj_index"});
    for(size_t ctr = 0; ctr < size; ctr++){
    outArray[ctr]["MessageType"] = factory.createCharArray("quad_msgs/GRFArray");
    // header
    auto currentElement_header = (msg + ctr)->header;
    auto msgClassPtr_header = getCommonObject<std_msgs::msg::Header>("ros2_std_msgs_msg_Header_common",loader);
    outArray[ctr]["header"] = msgClassPtr_header->get_arr(factory, &currentElement_header, loader);
    // vectors
    auto currentElement_vectors = (msg + ctr)->vectors;
    auto msgClassPtr_vectors = getCommonObject<geometry_msgs::msg::Vector3>("ros2_geometry_msgs_msg_Vector3_common",loader);
    outArray[ctr]["vectors"] = msgClassPtr_vectors->get_arr(factory,&currentElement_vectors[0],loader,currentElement_vectors.size());
    // points
    auto currentElement_points = (msg + ctr)->points;
    auto msgClassPtr_points = getCommonObject<geometry_msgs::msg::Point>("ros2_geometry_msgs_msg_Point_common",loader);
    outArray[ctr]["points"] = msgClassPtr_points->get_arr(factory,&currentElement_points[0],loader,currentElement_points.size());
    // contact_states
    auto currentElement_contact_states = (msg + ctr)->contact_states;
    outArray[ctr]["contact_states"] = factory.createArray<quad_msgs::msg::GRFArray::_contact_states_type::const_iterator, bool>({currentElement_contact_states.size(), 1}, currentElement_contact_states.begin(), currentElement_contact_states.end());
    // traj_index
    auto currentElement_traj_index = (msg + ctr)->traj_index;
    outArray[ctr]["traj_index"] = factory.createScalar(currentElement_traj_index);
    }
    return std::move(outArray);
  } 
class QUAD_MSGS_EXPORT ros2_quad_msgs_GRFArray_message : public ROS2MsgElementInterfaceFactory {
  public:
    virtual ~ros2_quad_msgs_GRFArray_message(){}
    virtual std::shared_ptr<MATLABPublisherInterface> generatePublisherInterface(ElementType /*type*/);
    virtual std::shared_ptr<MATLABSubscriberInterface> generateSubscriberInterface(ElementType /*type*/);
    virtual std::shared_ptr<void> generateCppMessage(ElementType /*type*/, const matlab::data::StructArray& /* arr */, MultiLibLoader /* loader */, std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
    virtual matlab::data::StructArray generateMLMessage(ElementType  /*type*/ ,void*  /* msg */, MultiLibLoader /* loader */ , std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* /*commonObjMap*/);
};  
  std::shared_ptr<MATLABPublisherInterface> 
          ros2_quad_msgs_GRFArray_message::generatePublisherInterface(ElementType /*type*/){
    return std::make_shared<ROS2PublisherImpl<quad_msgs::msg::GRFArray,ros2_quad_msgs_msg_GRFArray_common>>();
  }
  std::shared_ptr<MATLABSubscriberInterface> 
         ros2_quad_msgs_GRFArray_message::generateSubscriberInterface(ElementType /*type*/){
    return std::make_shared<ROS2SubscriberImpl<quad_msgs::msg::GRFArray,ros2_quad_msgs_msg_GRFArray_common>>();
  }
  std::shared_ptr<void> ros2_quad_msgs_GRFArray_message::generateCppMessage(ElementType /*type*/, 
                                           const matlab::data::StructArray& arr,
                                           MultiLibLoader loader,
                                           std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>* commonObjMap){
    auto msg = std::make_shared<quad_msgs::msg::GRFArray>();
    ros2_quad_msgs_msg_GRFArray_common commonObj;
    commonObj.mCommonObjMap = commonObjMap;
    commonObj.copy_from_struct(msg.get(), arr[0], loader);
    return msg;
  }
  matlab::data::StructArray ros2_quad_msgs_GRFArray_message::generateMLMessage(ElementType  /*type*/ ,
                                                    void*  msg ,
                                                    MultiLibLoader  loader ,
                                                    std::map<std::string,std::shared_ptr<MATLABROS2MsgInterfaceBase>>*  commonObjMap ){
    ros2_quad_msgs_msg_GRFArray_common commonObj;	
    commonObj.mCommonObjMap = commonObjMap;	
    MDFactory_T factory;
    return commonObj.get_arr(factory, (quad_msgs::msg::GRFArray*)msg, loader);			
 }
#include "class_loader/register_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_msg_GRFArray_common, MATLABROS2MsgInterface<quad_msgs::msg::GRFArray>)
CLASS_LOADER_REGISTER_CLASS(ros2_quad_msgs_GRFArray_message, ROS2MsgElementInterfaceFactory)
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER