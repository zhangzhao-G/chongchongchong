// Generated by gencpp from file serial_controllers/Test.msg
// DO NOT EDIT!


#ifndef SERIAL_CONTROLLERS_MESSAGE_TEST_H
#define SERIAL_CONTROLLERS_MESSAGE_TEST_H


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
struct Test_
{
  typedef Test_<ContainerAllocator> Type;

  Test_()
    : cmd_test(0)
    , vel_test(0)
    , time_test(0)  {
    }
  Test_(const ContainerAllocator& _alloc)
    : cmd_test(0)
    , vel_test(0)
    , time_test(0)  {
  (void)_alloc;
    }



   typedef int32_t _cmd_test_type;
  _cmd_test_type cmd_test;

   typedef int32_t _vel_test_type;
  _vel_test_type vel_test;

   typedef int32_t _time_test_type;
  _time_test_type time_test;





  typedef boost::shared_ptr< ::serial_controllers::Test_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::serial_controllers::Test_<ContainerAllocator> const> ConstPtr;

}; // struct Test_

typedef ::serial_controllers::Test_<std::allocator<void> > Test;

typedef boost::shared_ptr< ::serial_controllers::Test > TestPtr;
typedef boost::shared_ptr< ::serial_controllers::Test const> TestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::serial_controllers::Test_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::serial_controllers::Test_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::serial_controllers::Test_<ContainerAllocator1> & lhs, const ::serial_controllers::Test_<ContainerAllocator2> & rhs)
{
  return lhs.cmd_test == rhs.cmd_test &&
    lhs.vel_test == rhs.vel_test &&
    lhs.time_test == rhs.time_test;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::serial_controllers::Test_<ContainerAllocator1> & lhs, const ::serial_controllers::Test_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace serial_controllers

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::serial_controllers::Test_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::serial_controllers::Test_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::serial_controllers::Test_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::serial_controllers::Test_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_controllers::Test_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::serial_controllers::Test_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::serial_controllers::Test_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dc8b60705ddbc0a36f167dfb9fb7c63a";
  }

  static const char* value(const ::serial_controllers::Test_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdc8b60705ddbc0a3ULL;
  static const uint64_t static_value2 = 0x6f167dfb9fb7c63aULL;
};

template<class ContainerAllocator>
struct DataType< ::serial_controllers::Test_<ContainerAllocator> >
{
  static const char* value()
  {
    return "serial_controllers/Test";
  }

  static const char* value(const ::serial_controllers::Test_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::serial_controllers::Test_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 cmd_test\n"
"int32 vel_test\n"
"int32 time_test\n"
;
  }

  static const char* value(const ::serial_controllers::Test_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::serial_controllers::Test_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.cmd_test);
      stream.next(m.vel_test);
      stream.next(m.time_test);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Test_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::serial_controllers::Test_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::serial_controllers::Test_<ContainerAllocator>& v)
  {
    s << indent << "cmd_test: ";
    Printer<int32_t>::stream(s, indent + "  ", v.cmd_test);
    s << indent << "vel_test: ";
    Printer<int32_t>::stream(s, indent + "  ", v.vel_test);
    s << indent << "time_test: ";
    Printer<int32_t>::stream(s, indent + "  ", v.time_test);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SERIAL_CONTROLLERS_MESSAGE_TEST_H