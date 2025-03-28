// Generated by gencpp from file stdr_msgs/RfidTag.msg
// DO NOT EDIT!


#ifndef STDR_MSGS_MESSAGE_RFIDTAG_H
#define STDR_MSGS_MESSAGE_RFIDTAG_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose2D.h>

namespace stdr_msgs
{
template <class ContainerAllocator>
struct RfidTag_
{
  typedef RfidTag_<ContainerAllocator> Type;

  RfidTag_()
    : tag_id()
    , message()
    , pose()  {
    }
  RfidTag_(const ContainerAllocator& _alloc)
    : tag_id(_alloc)
    , message(_alloc)
    , pose(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _tag_id_type;
  _tag_id_type tag_id;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _message_type;
  _message_type message;

   typedef  ::geometry_msgs::Pose2D_<ContainerAllocator>  _pose_type;
  _pose_type pose;





  typedef boost::shared_ptr< ::stdr_msgs::RfidTag_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::stdr_msgs::RfidTag_<ContainerAllocator> const> ConstPtr;

}; // struct RfidTag_

typedef ::stdr_msgs::RfidTag_<std::allocator<void> > RfidTag;

typedef boost::shared_ptr< ::stdr_msgs::RfidTag > RfidTagPtr;
typedef boost::shared_ptr< ::stdr_msgs::RfidTag const> RfidTagConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::stdr_msgs::RfidTag_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::stdr_msgs::RfidTag_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::stdr_msgs::RfidTag_<ContainerAllocator1> & lhs, const ::stdr_msgs::RfidTag_<ContainerAllocator2> & rhs)
{
  return lhs.tag_id == rhs.tag_id &&
    lhs.message == rhs.message &&
    lhs.pose == rhs.pose;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::stdr_msgs::RfidTag_<ContainerAllocator1> & lhs, const ::stdr_msgs::RfidTag_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace stdr_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::stdr_msgs::RfidTag_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::stdr_msgs::RfidTag_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::stdr_msgs::RfidTag_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::stdr_msgs::RfidTag_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stdr_msgs::RfidTag_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stdr_msgs::RfidTag_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::stdr_msgs::RfidTag_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e37433c890cfe140ccbef22419fae16c";
  }

  static const char* value(const ::stdr_msgs::RfidTag_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe37433c890cfe140ULL;
  static const uint64_t static_value2 = 0xccbef22419fae16cULL;
};

template<class ContainerAllocator>
struct DataType< ::stdr_msgs::RfidTag_<ContainerAllocator> >
{
  static const char* value()
  {
    return "stdr_msgs/RfidTag";
  }

  static const char* value(const ::stdr_msgs::RfidTag_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::stdr_msgs::RfidTag_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Rfid tag description\n"
"\n"
"string tag_id\n"
"string message\n"
"geometry_msgs/Pose2D pose # sensor pose, relative to the map origin\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose2D\n"
"# Deprecated\n"
"# Please use the full 3D pose.\n"
"\n"
"# In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.\n"
"\n"
"# If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.\n"
"\n"
"\n"
"# This expresses a position and orientation on a 2D manifold.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 theta\n"
;
  }

  static const char* value(const ::stdr_msgs::RfidTag_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::stdr_msgs::RfidTag_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.tag_id);
      stream.next(m.message);
      stream.next(m.pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RfidTag_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::stdr_msgs::RfidTag_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::stdr_msgs::RfidTag_<ContainerAllocator>& v)
  {
    s << indent << "tag_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.tag_id);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.message);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose2D_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // STDR_MSGS_MESSAGE_RFIDTAG_H
