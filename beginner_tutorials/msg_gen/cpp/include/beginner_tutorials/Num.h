/* Auto-generated by genmsg_cpp for file /home/base/ros/beginner_tutorials/msg/Num.msg */
#ifndef BEGINNER_TUTORIALS_MESSAGE_NUM_H
#define BEGINNER_TUTORIALS_MESSAGE_NUM_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"


namespace beginner_tutorials
{
template <class ContainerAllocator>
struct Num_ : public ros::Message
{
  typedef Num_<ContainerAllocator> Type;

  Num_()
  : num(0)
  {
  }

  Num_(const ContainerAllocator& _alloc)
  : num(0)
  {
  }

  typedef int64_t _num_type;
  int64_t num;


private:
  static const char* __s_getDataType_() { return "beginner_tutorials/Num"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "57d3c40ec3ac3754af76a83e6e73127a"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int64 num\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, num);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, num);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(num);
    return size;
  }

  typedef boost::shared_ptr< ::beginner_tutorials::Num_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::beginner_tutorials::Num_<ContainerAllocator>  const> ConstPtr;
}; // struct Num
typedef  ::beginner_tutorials::Num_<std::allocator<void> > Num;

typedef boost::shared_ptr< ::beginner_tutorials::Num> NumPtr;
typedef boost::shared_ptr< ::beginner_tutorials::Num const> NumConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::beginner_tutorials::Num_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::beginner_tutorials::Num_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace beginner_tutorials

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::beginner_tutorials::Num_<ContainerAllocator> > {
  static const char* value() 
  {
    return "57d3c40ec3ac3754af76a83e6e73127a";
  }

  static const char* value(const  ::beginner_tutorials::Num_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x57d3c40ec3ac3754ULL;
  static const uint64_t static_value2 = 0xaf76a83e6e73127aULL;
};

template<class ContainerAllocator>
struct DataType< ::beginner_tutorials::Num_<ContainerAllocator> > {
  static const char* value() 
  {
    return "beginner_tutorials/Num";
  }

  static const char* value(const  ::beginner_tutorials::Num_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::beginner_tutorials::Num_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int64 num\n\
\n\
";
  }

  static const char* value(const  ::beginner_tutorials::Num_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::beginner_tutorials::Num_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::beginner_tutorials::Num_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.num);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Num_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::beginner_tutorials::Num_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::beginner_tutorials::Num_<ContainerAllocator> & v) 
  {
    s << indent << "num: ";
    Printer<int64_t>::stream(s, indent + "  ", v.num);
  }
};


} // namespace message_operations
} // namespace ros

#endif // BEGINNER_TUTORIALS_MESSAGE_NUM_H
