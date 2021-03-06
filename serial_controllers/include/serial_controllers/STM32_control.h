// Generated by gencpp from file serial_controllers/STM32_control.msg
// DO NOT EDIT!


#ifndef SERIAL_CONTROLLERS_MESSAGE_STM32_CONTROL_H
#define SERIAL_CONTROLLERS_MESSAGE_STM32_CONTROL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace serial_controllers
{
template <class ContainerAllocator>
struct STM32_control_
{
  typedef STM32_control_<ContainerAllocator> Type;

  STM32_control_()
    : Gear(0)
    , Angle(0.0)
    , Angle_vel(0.0)
    , Vel(0.0)
    , Brak(0.0)
    , Park(0)
    , Odom(0)  {
    }
  STM32_control_(const ContainerAllocator& _alloc)
    : Gear(0)
    , Angle(0.0)
    , Angle_vel(0.0)
    , Vel(0.0)
    , Brak(0.0)
    , Park(0)
    , Odom(0)  {
  (void)_alloc;
    }



   typedef int32_t _Gear_type;
  _Gear_type Gear;

   typedef double _Angle_type;
  _Angle_type Angle;

   typedef double _Angle_vel_type;
  _Angle_vel_type Angle_vel;

   typedef double _Vel_type;
  _Vel_type Vel;

   typedef double _Brak_type;
  _Brak_type Brak;

   typedef int32_t _Park_type;
  _Park_type Park;

   typedef int32_t _Odom_type;
  _Odom_type Odom;





  typedef boost::shared_ptr< ::serial_controllers::STM32_control_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serial_controllers::STM32_control_<ContainerAllocator> const> ConstPtr;

}; // struct STM32_control_

typedef ::serial_controllers::STM32_control_<std::allocator<void> > STM32_control;

typedef boost::shared_ptr< ::serial_controllers::STM32_control > STM32_controlPtr;
typedef boost::shared_ptr< ::serial_controllers::STM32_control const> STM32_controlConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::serial_controllers::STM32_control_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::serial_controllers::STM32_control_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::serial_controllers::STM32_control_<ContainerAllocator1> & lhs, const ::serial_controllers::STM32_control_<ContainerAllocator2> & rhs)
{
  return lhs.Gear == rhs.Gear &&
    lhs.Angle == rhs.Angle &&
    lhs.Angle_vel == rhs.Angle_vel &&
    lhs.Vel == rhs.Vel &&
    lhs.Brak == rhs.Brak &&
    lhs.Park == rhs.Park &&
    lhs.Odom == rhs.Odom;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::serial_controllers::STM32_control_<ContainerAllocator1> & lhs, const ::serial_controllers::STM32_control_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace serial_controllers

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::serial_controllers::STM32_control_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::serial_controllers::STM32_control_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::serial_controllers::STM32_control_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::serial_controllers::STM32_control_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_controllers::STM32_control_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_controllers::STM32_control_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::serial_controllers::STM32_control_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a254aabcb3b78b73aa7ce9a2c2c9929b";
  }

  static const char* value(const ::serial_controllers::STM32_control_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa254aabcb3b78b73ULL;
  static const uint64_t static_value2 = 0xaa7ce9a2c2c9929bULL;
};

template<class ContainerAllocator>
struct DataType< ::serial_controllers::STM32_control_<ContainerAllocator> >
{
  static const char* value()
  {
    return "serial_controllers/STM32_control";
  }

  static const char* value(const ::serial_controllers::STM32_control_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::serial_controllers::STM32_control_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 Gear\n"
"float64 Angle\n"
"float64 Angle_vel\n"
"float64 Vel\n"
"float64 Brak\n"
"int32 Park\n"
"int32 Odom\n"
;
  }

  static const char* value(const ::serial_controllers::STM32_control_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::serial_controllers::STM32_control_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Gear);
      stream.next(m.Angle);
      stream.next(m.Angle_vel);
      stream.next(m.Vel);
      stream.next(m.Brak);
      stream.next(m.Park);
      stream.next(m.Odom);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct STM32_control_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::serial_controllers::STM32_control_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::serial_controllers::STM32_control_<ContainerAllocator>& v)
  {
    s << indent << "Gear: ";
    Printer<int32_t>::stream(s, indent + "  ", v.Gear);
    s << indent << "Angle: ";
    Printer<double>::stream(s, indent + "  ", v.Angle);
    s << indent << "Angle_vel: ";
    Printer<double>::stream(s, indent + "  ", v.Angle_vel);
    s << indent << "Vel: ";
    Printer<double>::stream(s, indent + "  ", v.Vel);
    s << indent << "Brak: ";
    Printer<double>::stream(s, indent + "  ", v.Brak);
    s << indent << "Park: ";
    Printer<int32_t>::stream(s, indent + "  ", v.Park);
    s << indent << "Odom: ";
    Printer<int32_t>::stream(s, indent + "  ", v.Odom);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SERIAL_CONTROLLERS_MESSAGE_STM32_CONTROL_H
