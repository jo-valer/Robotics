// Generated by gencpp from file robotic_vision/Localize.msg
// DO NOT EDIT!


#ifndef ROBOTIC_VISION_MESSAGE_LOCALIZE_H
#define ROBOTIC_VISION_MESSAGE_LOCALIZE_H


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
struct Localize_
{
  typedef Localize_<ContainerAllocator> Type;

  Localize_()
    : numLego(0)
    , lego1_imgx(0)
    , lego1_imgy(0)
    , lego1_x(0.0)
    , lego1_y(0.0)
    , lego1_q(0.0)
    , lego1_w(0.0)
    , lego1_h(0.0)
    , lego2_imgx(0)
    , lego2_imgy(0)
    , lego2_x(0.0)
    , lego2_y(0.0)
    , lego2_q(0.0)
    , lego2_w(0.0)
    , lego2_h(0.0)
    , lego3_imgx(0)
    , lego3_imgy(0)
    , lego3_x(0.0)
    , lego3_y(0.0)
    , lego3_q(0.0)
    , lego3_w(0.0)
    , lego3_h(0.0)  {
    }
  Localize_(const ContainerAllocator& _alloc)
    : numLego(0)
    , lego1_imgx(0)
    , lego1_imgy(0)
    , lego1_x(0.0)
    , lego1_y(0.0)
    , lego1_q(0.0)
    , lego1_w(0.0)
    , lego1_h(0.0)
    , lego2_imgx(0)
    , lego2_imgy(0)
    , lego2_x(0.0)
    , lego2_y(0.0)
    , lego2_q(0.0)
    , lego2_w(0.0)
    , lego2_h(0.0)
    , lego3_imgx(0)
    , lego3_imgy(0)
    , lego3_x(0.0)
    , lego3_y(0.0)
    , lego3_q(0.0)
    , lego3_w(0.0)
    , lego3_h(0.0)  {
  (void)_alloc;
    }



   typedef int64_t _numLego_type;
  _numLego_type numLego;

   typedef int64_t _lego1_imgx_type;
  _lego1_imgx_type lego1_imgx;

   typedef int64_t _lego1_imgy_type;
  _lego1_imgy_type lego1_imgy;

   typedef double _lego1_x_type;
  _lego1_x_type lego1_x;

   typedef double _lego1_y_type;
  _lego1_y_type lego1_y;

   typedef double _lego1_q_type;
  _lego1_q_type lego1_q;

   typedef double _lego1_w_type;
  _lego1_w_type lego1_w;

   typedef double _lego1_h_type;
  _lego1_h_type lego1_h;

   typedef int64_t _lego2_imgx_type;
  _lego2_imgx_type lego2_imgx;

   typedef int64_t _lego2_imgy_type;
  _lego2_imgy_type lego2_imgy;

   typedef double _lego2_x_type;
  _lego2_x_type lego2_x;

   typedef double _lego2_y_type;
  _lego2_y_type lego2_y;

   typedef double _lego2_q_type;
  _lego2_q_type lego2_q;

   typedef double _lego2_w_type;
  _lego2_w_type lego2_w;

   typedef double _lego2_h_type;
  _lego2_h_type lego2_h;

   typedef int64_t _lego3_imgx_type;
  _lego3_imgx_type lego3_imgx;

   typedef int64_t _lego3_imgy_type;
  _lego3_imgy_type lego3_imgy;

   typedef double _lego3_x_type;
  _lego3_x_type lego3_x;

   typedef double _lego3_y_type;
  _lego3_y_type lego3_y;

   typedef double _lego3_q_type;
  _lego3_q_type lego3_q;

   typedef double _lego3_w_type;
  _lego3_w_type lego3_w;

   typedef double _lego3_h_type;
  _lego3_h_type lego3_h;





  typedef boost::shared_ptr< ::robotic_vision::Localize_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotic_vision::Localize_<ContainerAllocator> const> ConstPtr;

}; // struct Localize_

typedef ::robotic_vision::Localize_<std::allocator<void> > Localize;

typedef boost::shared_ptr< ::robotic_vision::Localize > LocalizePtr;
typedef boost::shared_ptr< ::robotic_vision::Localize const> LocalizeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotic_vision::Localize_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotic_vision::Localize_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robotic_vision::Localize_<ContainerAllocator1> & lhs, const ::robotic_vision::Localize_<ContainerAllocator2> & rhs)
{
  return lhs.numLego == rhs.numLego &&
    lhs.lego1_imgx == rhs.lego1_imgx &&
    lhs.lego1_imgy == rhs.lego1_imgy &&
    lhs.lego1_x == rhs.lego1_x &&
    lhs.lego1_y == rhs.lego1_y &&
    lhs.lego1_q == rhs.lego1_q &&
    lhs.lego1_w == rhs.lego1_w &&
    lhs.lego1_h == rhs.lego1_h &&
    lhs.lego2_imgx == rhs.lego2_imgx &&
    lhs.lego2_imgy == rhs.lego2_imgy &&
    lhs.lego2_x == rhs.lego2_x &&
    lhs.lego2_y == rhs.lego2_y &&
    lhs.lego2_q == rhs.lego2_q &&
    lhs.lego2_w == rhs.lego2_w &&
    lhs.lego2_h == rhs.lego2_h &&
    lhs.lego3_imgx == rhs.lego3_imgx &&
    lhs.lego3_imgy == rhs.lego3_imgy &&
    lhs.lego3_x == rhs.lego3_x &&
    lhs.lego3_y == rhs.lego3_y &&
    lhs.lego3_q == rhs.lego3_q &&
    lhs.lego3_w == rhs.lego3_w &&
    lhs.lego3_h == rhs.lego3_h;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robotic_vision::Localize_<ContainerAllocator1> & lhs, const ::robotic_vision::Localize_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robotic_vision

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robotic_vision::Localize_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotic_vision::Localize_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotic_vision::Localize_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotic_vision::Localize_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotic_vision::Localize_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotic_vision::Localize_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotic_vision::Localize_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4bac6bf79e8c483390a8bec510c7fab7";
  }

  static const char* value(const ::robotic_vision::Localize_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4bac6bf79e8c4833ULL;
  static const uint64_t static_value2 = 0x90a8bec510c7fab7ULL;
};

template<class ContainerAllocator>
struct DataType< ::robotic_vision::Localize_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotic_vision/Localize";
  }

  static const char* value(const ::robotic_vision::Localize_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotic_vision::Localize_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 numLego\n"
"int64 lego1_imgx\n"
"int64 lego1_imgy\n"
"float64 lego1_x\n"
"float64 lego1_y\n"
"float64 lego1_q\n"
"float64 lego1_w\n"
"float64 lego1_h\n"
"int64 lego2_imgx\n"
"int64 lego2_imgy\n"
"float64 lego2_x\n"
"float64 lego2_y\n"
"float64 lego2_q\n"
"float64 lego2_w\n"
"float64 lego2_h\n"
"int64 lego3_imgx\n"
"int64 lego3_imgy\n"
"float64 lego3_x\n"
"float64 lego3_y\n"
"float64 lego3_q\n"
"float64 lego3_w\n"
"float64 lego3_h\n"
;
  }

  static const char* value(const ::robotic_vision::Localize_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotic_vision::Localize_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.numLego);
      stream.next(m.lego1_imgx);
      stream.next(m.lego1_imgy);
      stream.next(m.lego1_x);
      stream.next(m.lego1_y);
      stream.next(m.lego1_q);
      stream.next(m.lego1_w);
      stream.next(m.lego1_h);
      stream.next(m.lego2_imgx);
      stream.next(m.lego2_imgy);
      stream.next(m.lego2_x);
      stream.next(m.lego2_y);
      stream.next(m.lego2_q);
      stream.next(m.lego2_w);
      stream.next(m.lego2_h);
      stream.next(m.lego3_imgx);
      stream.next(m.lego3_imgy);
      stream.next(m.lego3_x);
      stream.next(m.lego3_y);
      stream.next(m.lego3_q);
      stream.next(m.lego3_w);
      stream.next(m.lego3_h);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Localize_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotic_vision::Localize_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotic_vision::Localize_<ContainerAllocator>& v)
  {
    s << indent << "numLego: ";
    Printer<int64_t>::stream(s, indent + "  ", v.numLego);
    s << indent << "lego1_imgx: ";
    Printer<int64_t>::stream(s, indent + "  ", v.lego1_imgx);
    s << indent << "lego1_imgy: ";
    Printer<int64_t>::stream(s, indent + "  ", v.lego1_imgy);
    s << indent << "lego1_x: ";
    Printer<double>::stream(s, indent + "  ", v.lego1_x);
    s << indent << "lego1_y: ";
    Printer<double>::stream(s, indent + "  ", v.lego1_y);
    s << indent << "lego1_q: ";
    Printer<double>::stream(s, indent + "  ", v.lego1_q);
    s << indent << "lego1_w: ";
    Printer<double>::stream(s, indent + "  ", v.lego1_w);
    s << indent << "lego1_h: ";
    Printer<double>::stream(s, indent + "  ", v.lego1_h);
    s << indent << "lego2_imgx: ";
    Printer<int64_t>::stream(s, indent + "  ", v.lego2_imgx);
    s << indent << "lego2_imgy: ";
    Printer<int64_t>::stream(s, indent + "  ", v.lego2_imgy);
    s << indent << "lego2_x: ";
    Printer<double>::stream(s, indent + "  ", v.lego2_x);
    s << indent << "lego2_y: ";
    Printer<double>::stream(s, indent + "  ", v.lego2_y);
    s << indent << "lego2_q: ";
    Printer<double>::stream(s, indent + "  ", v.lego2_q);
    s << indent << "lego2_w: ";
    Printer<double>::stream(s, indent + "  ", v.lego2_w);
    s << indent << "lego2_h: ";
    Printer<double>::stream(s, indent + "  ", v.lego2_h);
    s << indent << "lego3_imgx: ";
    Printer<int64_t>::stream(s, indent + "  ", v.lego3_imgx);
    s << indent << "lego3_imgy: ";
    Printer<int64_t>::stream(s, indent + "  ", v.lego3_imgy);
    s << indent << "lego3_x: ";
    Printer<double>::stream(s, indent + "  ", v.lego3_x);
    s << indent << "lego3_y: ";
    Printer<double>::stream(s, indent + "  ", v.lego3_y);
    s << indent << "lego3_q: ";
    Printer<double>::stream(s, indent + "  ", v.lego3_q);
    s << indent << "lego3_w: ";
    Printer<double>::stream(s, indent + "  ", v.lego3_w);
    s << indent << "lego3_h: ";
    Printer<double>::stream(s, indent + "  ", v.lego3_h);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTIC_VISION_MESSAGE_LOCALIZE_H
