// Generated by gencpp from file stdr_msgs/SoundSourceVector.msg
// DO NOT EDIT!


#ifndef STDR_MSGS_MESSAGE_SOUNDSOURCEVECTOR_H
#define STDR_MSGS_MESSAGE_SOUNDSOURCEVECTOR_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <stdr_msgs/SoundSource.h>

namespace stdr_msgs
{
template <class ContainerAllocator>
struct SoundSourceVector_
{
  typedef SoundSourceVector_<ContainerAllocator> Type;

  SoundSourceVector_()
    : sound_sources()  {
    }
  SoundSourceVector_(const ContainerAllocator& _alloc)
    : sound_sources(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::stdr_msgs::SoundSource_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::stdr_msgs::SoundSource_<ContainerAllocator> >> _sound_sources_type;
  _sound_sources_type sound_sources;





  typedef boost::shared_ptr< ::stdr_msgs::SoundSourceVector_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::stdr_msgs::SoundSourceVector_<ContainerAllocator> const> ConstPtr;

}; // struct SoundSourceVector_

typedef ::stdr_msgs::SoundSourceVector_<std::allocator<void> > SoundSourceVector;

typedef boost::shared_ptr< ::stdr_msgs::SoundSourceVector > SoundSourceVectorPtr;
typedef boost::shared_ptr< ::stdr_msgs::SoundSourceVector const> SoundSourceVectorConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::stdr_msgs::SoundSourceVector_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::stdr_msgs::SoundSourceVector_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::stdr_msgs::SoundSourceVector_<ContainerAllocator1> & lhs, const ::stdr_msgs::SoundSourceVector_<ContainerAllocator2> & rhs)
{
  return lhs.sound_sources == rhs.sound_sources;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::stdr_msgs::SoundSourceVector_<ContainerAllocator1> & lhs, const ::stdr_msgs::SoundSourceVector_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace stdr_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::stdr_msgs::SoundSourceVector_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::stdr_msgs::SoundSourceVector_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::stdr_msgs::SoundSourceVector_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::stdr_msgs::SoundSourceVector_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stdr_msgs::SoundSourceVector_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::stdr_msgs::SoundSourceVector_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::stdr_msgs::SoundSourceVector_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b0990ed9e7eeb58876a06c8b484951b4";
  }

  static const char* value(const ::stdr_msgs::SoundSourceVector_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb0990ed9e7eeb588ULL;
  static const uint64_t static_value2 = 0x76a06c8b484951b4ULL;
};

template<class ContainerAllocator>
struct DataType< ::stdr_msgs::SoundSourceVector_<ContainerAllocator> >
{
  static const char* value()
  {
    return "stdr_msgs/SoundSourceVector";
  }

  static const char* value(const ::stdr_msgs::SoundSourceVector_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::stdr_msgs::SoundSourceVector_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Sound sources list\n"
"stdr_msgs/SoundSource[] sound_sources\n"
"\n"
"================================================================================\n"
"MSG: stdr_msgs/SoundSource\n"
"# Source description\n"
"\n"
"string id\n"
"float32 dbs\n"
"\n"
"# sensor pose, relative to the map origin\n"
"geometry_msgs/Pose2D pose \n"
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

  static const char* value(const ::stdr_msgs::SoundSourceVector_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::stdr_msgs::SoundSourceVector_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.sound_sources);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SoundSourceVector_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::stdr_msgs::SoundSourceVector_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::stdr_msgs::SoundSourceVector_<ContainerAllocator>& v)
  {
    s << indent << "sound_sources[]" << std::endl;
    for (size_t i = 0; i < v.sound_sources.size(); ++i)
    {
      s << indent << "  sound_sources[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::stdr_msgs::SoundSource_<ContainerAllocator> >::stream(s, indent + "    ", v.sound_sources[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // STDR_MSGS_MESSAGE_SOUNDSOURCEVECTOR_H
