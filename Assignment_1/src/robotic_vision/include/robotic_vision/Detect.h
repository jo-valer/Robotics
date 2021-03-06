// Generated by gencpp from file robotic_vision/Detect.msg
// DO NOT EDIT!


#ifndef ROBOTIC_VISION_MESSAGE_DETECT_H
#define ROBOTIC_VISION_MESSAGE_DETECT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robotic_vision
{
template <class ContainerAllocator>
struct Detect_
{
  typedef Detect_<ContainerAllocator> Type;

  Detect_()
    : classe(0)
    , center_x(0.0)
    , center_y(0.0)
    , width(0.0)
    , height(0.0)
    , confidence(0.0)  {
    }
  Detect_(const ContainerAllocator& _alloc)
    : classe(0)
    , center_x(0.0)
    , center_y(0.0)
    , width(0.0)
    , height(0.0)
    , confidence(0.0)  {
  (void)_alloc;
    }



   typedef int64_t _classe_type;
  _classe_type classe;

   typedef double _center_x_type;
  _center_x_type center_x;

   typedef double _center_y_type;
  _center_y_type center_y;

   typedef double _width_type;
  _width_type width;

   typedef double _height_type;
  _height_type height;

   typedef double _confidence_type;
  _confidence_type confidence;





  typedef boost::shared_ptr< ::robotic_vision::Detect_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotic_vision::Detect_<ContainerAllocator> const> ConstPtr;

}; // struct Detect_

typedef ::robotic_vision::Detect_<std::allocator<void> > Detect;

typedef boost::shared_ptr< ::robotic_vision::Detect > DetectPtr;
typedef boost::shared_ptr< ::robotic_vision::Detect const> DetectConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotic_vision::Detect_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotic_vision::Detect_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robotic_vision::Detect_<ContainerAllocator1> & lhs, const ::robotic_vision::Detect_<ContainerAllocator2> & rhs)
{
  return lhs.classe == rhs.classe &&
    lhs.center_x == rhs.center_x &&
    lhs.center_y == rhs.center_y &&
    lhs.width == rhs.width &&
    lhs.height == rhs.height &&
    lhs.confidence == rhs.confidence;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robotic_vision::Detect_<ContainerAllocator1> & lhs, const ::robotic_vision::Detect_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robotic_vision

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robotic_vision::Detect_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotic_vision::Detect_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotic_vision::Detect_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotic_vision::Detect_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotic_vision::Detect_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotic_vision::Detect_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotic_vision::Detect_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7686e6995238a4466ac40f8d4fa9c04a";
  }

  static const char* value(const ::robotic_vision::Detect_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7686e6995238a446ULL;
  static const uint64_t static_value2 = 0x6ac40f8d4fa9c04aULL;
};

template<class ContainerAllocator>
struct DataType< ::robotic_vision::Detect_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotic_vision/Detect";
  }

  static const char* value(const ::robotic_vision::Detect_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotic_vision::Detect_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 classe\n"
"float64 center_x\n"
"float64 center_y\n"
"float64 width\n"
"float64 height\n"
"float64 confidence\n"
;
  }

  static const char* value(const ::robotic_vision::Detect_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotic_vision::Detect_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.classe);
      stream.next(m.center_x);
      stream.next(m.center_y);
      stream.next(m.width);
      stream.next(m.height);
      stream.next(m.confidence);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Detect_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotic_vision::Detect_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotic_vision::Detect_<ContainerAllocator>& v)
  {
    s << indent << "classe: ";
    Printer<int64_t>::stream(s, indent + "  ", v.classe);
    s << indent << "center_x: ";
    Printer<double>::stream(s, indent + "  ", v.center_x);
    s << indent << "center_y: ";
    Printer<double>::stream(s, indent + "  ", v.center_y);
    s << indent << "width: ";
    Printer<double>::stream(s, indent + "  ", v.width);
    s << indent << "height: ";
    Printer<double>::stream(s, indent + "  ", v.height);
    s << indent << "confidence: ";
    Printer<double>::stream(s, indent + "  ", v.confidence);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTIC_VISION_MESSAGE_DETECT_H
