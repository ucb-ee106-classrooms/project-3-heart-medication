// Generated by gencpp from file stdr_msgs/DeleteRobotFeedback.msg
// DO NOT EDIT!


#ifndef STDR_MSGS_MESSAGE_DELETEROBOTFEEDBACK_H
#define STDR_MSGS_MESSAGE_DELETEROBOTFEEDBACK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace stdr_msgs
{
template <class ContainerAllocator>
struct DeleteRobotFeedback_
{
  typedef DeleteRobotFeedback_<ContainerAllocator> Type;

  DeleteRobotFeedback_()
    {
    }
  DeleteRobotFeedback_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct DeleteRobotFeedback_

typedef ::stdr_msgs::DeleteRobotFeedback_<std::allocator<void> > DeleteRobotFeedback;

typedef boost::shared_ptr< ::stdr_msgs::DeleteRobotFeedback > DeleteRobotFeedbackPtr;
typedef boost::shared_ptr< ::stdr_msgs::DeleteRobotFeedback const> DeleteRobotFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace stdr_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "stdr_msgs/DeleteRobotFeedback";
  }

  static const char* value(const ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"#feedback\n"
"\n"
;
  }

  static const char* value(const ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DeleteRobotFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::stdr_msgs::DeleteRobotFeedback_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // STDR_MSGS_MESSAGE_DELETEROBOTFEEDBACK_H
