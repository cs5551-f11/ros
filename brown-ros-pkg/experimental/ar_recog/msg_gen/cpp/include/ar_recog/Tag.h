/* Auto-generated by genmsg_cpp for file /home/aghos7/ros/brown-ros-pkg/experimental/ar_recog/msg/Tag.msg */
#ifndef AR_RECOG_MESSAGE_TAG_H
#define AR_RECOG_MESSAGE_TAG_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"


namespace ar_recog
{
template <class ContainerAllocator>
struct Tag_ : public ros::Message
{
  typedef Tag_<ContainerAllocator> Type;

  Tag_()
  : id(0)
  , cf(0.0)
  , x(0)
  , y(0)
  , diameter(0.0)
  , distance(0.0)
  , xRot(0.0)
  , yRot(0.0)
  , zRot(0.0)
  , xMetric(0.0)
  , yMetric(0.0)
  , zMetric(0.0)
  , cwCorners()
  {
    cwCorners.assign(0.0);
  }

  Tag_(const ContainerAllocator& _alloc)
  : id(0)
  , cf(0.0)
  , x(0)
  , y(0)
  , diameter(0.0)
  , distance(0.0)
  , xRot(0.0)
  , yRot(0.0)
  , zRot(0.0)
  , xMetric(0.0)
  , yMetric(0.0)
  , zMetric(0.0)
  , cwCorners()
  {
    cwCorners.assign(0.0);
  }

  typedef uint32_t _id_type;
  uint32_t id;

  typedef double _cf_type;
  double cf;

  typedef uint32_t _x_type;
  uint32_t x;

  typedef uint32_t _y_type;
  uint32_t y;

  typedef double _diameter_type;
  double diameter;

  typedef double _distance_type;
  double distance;

  typedef double _xRot_type;
  double xRot;

  typedef double _yRot_type;
  double yRot;

  typedef double _zRot_type;
  double zRot;

  typedef double _xMetric_type;
  double xMetric;

  typedef double _yMetric_type;
  double yMetric;

  typedef double _zMetric_type;
  double zMetric;

  typedef boost::array<double, 8>  _cwCorners_type;
  boost::array<double, 8>  cwCorners;


  ROS_DEPRECATED uint32_t get_cwCorners_size() const { return (uint32_t)cwCorners.size(); }
private:
  static const char* __s_getDataType_() { return "ar_recog/Tag"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "8506b66f10a2975d80e32037f36b9ab4"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "# All screen measurements are in pixels, all spatial measurements are in meters.\n\
# Angles in radians.\n\
uint32 id\n\
# This is a rating of confidence in the tag pattern identification 0 < cf < 1.\n\
float64 cf\n\
uint32 x\n\
uint32 y\n\
# The 'diameter' is the square root of the tag's actual area, as estimated by \n\
# the AR software.  You can use it to check the cf confidence.\n\
float64 diameter\n\
# This is the estimated distance from viewer to the center of the tag.\n\
float64 distance\n\
float64 xRot \n\
float64 yRot \n\
float64 zRot\n\
float64 xMetric\n\
float64 yMetric\n\
float64 zMetric\n\
# Screen coordinates of the four corners.\n\
float64[8] cwCorners\n\
# FOR TESTING ONLY\n\
# Uncommenting this and uncommenting the similarly-marked lines in \n\
# ar_recog.cpp will put the ARToolkit rotation matrix into the Tag message,\n\
# which can be useful for debugging and testing.\n\
# float64[3] c3\n\
# float64[3] t0\n\
# float64[3] t1\n\
# float64[3] t2\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, id);
    ros::serialization::serialize(stream, cf);
    ros::serialization::serialize(stream, x);
    ros::serialization::serialize(stream, y);
    ros::serialization::serialize(stream, diameter);
    ros::serialization::serialize(stream, distance);
    ros::serialization::serialize(stream, xRot);
    ros::serialization::serialize(stream, yRot);
    ros::serialization::serialize(stream, zRot);
    ros::serialization::serialize(stream, xMetric);
    ros::serialization::serialize(stream, yMetric);
    ros::serialization::serialize(stream, zMetric);
    ros::serialization::serialize(stream, cwCorners);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, id);
    ros::serialization::deserialize(stream, cf);
    ros::serialization::deserialize(stream, x);
    ros::serialization::deserialize(stream, y);
    ros::serialization::deserialize(stream, diameter);
    ros::serialization::deserialize(stream, distance);
    ros::serialization::deserialize(stream, xRot);
    ros::serialization::deserialize(stream, yRot);
    ros::serialization::deserialize(stream, zRot);
    ros::serialization::deserialize(stream, xMetric);
    ros::serialization::deserialize(stream, yMetric);
    ros::serialization::deserialize(stream, zMetric);
    ros::serialization::deserialize(stream, cwCorners);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(id);
    size += ros::serialization::serializationLength(cf);
    size += ros::serialization::serializationLength(x);
    size += ros::serialization::serializationLength(y);
    size += ros::serialization::serializationLength(diameter);
    size += ros::serialization::serializationLength(distance);
    size += ros::serialization::serializationLength(xRot);
    size += ros::serialization::serializationLength(yRot);
    size += ros::serialization::serializationLength(zRot);
    size += ros::serialization::serializationLength(xMetric);
    size += ros::serialization::serializationLength(yMetric);
    size += ros::serialization::serializationLength(zMetric);
    size += ros::serialization::serializationLength(cwCorners);
    return size;
  }

  typedef boost::shared_ptr< ::ar_recog::Tag_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ar_recog::Tag_<ContainerAllocator>  const> ConstPtr;
}; // struct Tag
typedef  ::ar_recog::Tag_<std::allocator<void> > Tag;

typedef boost::shared_ptr< ::ar_recog::Tag> TagPtr;
typedef boost::shared_ptr< ::ar_recog::Tag const> TagConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::ar_recog::Tag_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::ar_recog::Tag_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace ar_recog

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::ar_recog::Tag_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8506b66f10a2975d80e32037f36b9ab4";
  }

  static const char* value(const  ::ar_recog::Tag_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8506b66f10a2975dULL;
  static const uint64_t static_value2 = 0x80e32037f36b9ab4ULL;
};

template<class ContainerAllocator>
struct DataType< ::ar_recog::Tag_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ar_recog/Tag";
  }

  static const char* value(const  ::ar_recog::Tag_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ar_recog::Tag_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# All screen measurements are in pixels, all spatial measurements are in meters.\n\
# Angles in radians.\n\
uint32 id\n\
# This is a rating of confidence in the tag pattern identification 0 < cf < 1.\n\
float64 cf\n\
uint32 x\n\
uint32 y\n\
# The 'diameter' is the square root of the tag's actual area, as estimated by \n\
# the AR software.  You can use it to check the cf confidence.\n\
float64 diameter\n\
# This is the estimated distance from viewer to the center of the tag.\n\
float64 distance\n\
float64 xRot \n\
float64 yRot \n\
float64 zRot\n\
float64 xMetric\n\
float64 yMetric\n\
float64 zMetric\n\
# Screen coordinates of the four corners.\n\
float64[8] cwCorners\n\
# FOR TESTING ONLY\n\
# Uncommenting this and uncommenting the similarly-marked lines in \n\
# ar_recog.cpp will put the ARToolkit rotation matrix into the Tag message,\n\
# which can be useful for debugging and testing.\n\
# float64[3] c3\n\
# float64[3] t0\n\
# float64[3] t1\n\
# float64[3] t2\n\
\n\
";
  }

  static const char* value(const  ::ar_recog::Tag_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::ar_recog::Tag_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ar_recog::Tag_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.id);
    stream.next(m.cf);
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.diameter);
    stream.next(m.distance);
    stream.next(m.xRot);
    stream.next(m.yRot);
    stream.next(m.zRot);
    stream.next(m.xMetric);
    stream.next(m.yMetric);
    stream.next(m.zMetric);
    stream.next(m.cwCorners);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Tag_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ar_recog::Tag_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::ar_recog::Tag_<ContainerAllocator> & v) 
  {
    s << indent << "id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.id);
    s << indent << "cf: ";
    Printer<double>::stream(s, indent + "  ", v.cf);
    s << indent << "x: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.y);
    s << indent << "diameter: ";
    Printer<double>::stream(s, indent + "  ", v.diameter);
    s << indent << "distance: ";
    Printer<double>::stream(s, indent + "  ", v.distance);
    s << indent << "xRot: ";
    Printer<double>::stream(s, indent + "  ", v.xRot);
    s << indent << "yRot: ";
    Printer<double>::stream(s, indent + "  ", v.yRot);
    s << indent << "zRot: ";
    Printer<double>::stream(s, indent + "  ", v.zRot);
    s << indent << "xMetric: ";
    Printer<double>::stream(s, indent + "  ", v.xMetric);
    s << indent << "yMetric: ";
    Printer<double>::stream(s, indent + "  ", v.yMetric);
    s << indent << "zMetric: ";
    Printer<double>::stream(s, indent + "  ", v.zMetric);
    s << indent << "cwCorners[]" << std::endl;
    for (size_t i = 0; i < v.cwCorners.size(); ++i)
    {
      s << indent << "  cwCorners[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.cwCorners[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // AR_RECOG_MESSAGE_TAG_H

