// Generated by gencpp from file orca_room/test.msg
// DO NOT EDIT!


#ifndef ORCA_ROOM_MESSAGE_TEST_H
#define ORCA_ROOM_MESSAGE_TEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace orca_room
{
template <class ContainerAllocator>
struct test_
{
  typedef test_<ContainerAllocator> Type;

  test_()
    : x(0.0)
    , y(0.0)
    , name()  {
    }
  test_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , name(_alloc)  {
  (void)_alloc;
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _name_type;
  _name_type name;





  typedef boost::shared_ptr< ::orca_room::test_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::orca_room::test_<ContainerAllocator> const> ConstPtr;

}; // struct test_

typedef ::orca_room::test_<std::allocator<void> > test;

typedef boost::shared_ptr< ::orca_room::test > testPtr;
typedef boost::shared_ptr< ::orca_room::test const> testConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::orca_room::test_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::orca_room::test_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::orca_room::test_<ContainerAllocator1> & lhs, const ::orca_room::test_<ContainerAllocator2> & rhs)
{
  return lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.name == rhs.name;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::orca_room::test_<ContainerAllocator1> & lhs, const ::orca_room::test_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace orca_room

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::orca_room::test_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::orca_room::test_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::orca_room::test_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::orca_room::test_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::orca_room::test_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::orca_room::test_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::orca_room::test_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2d9bb10b98c27a155fd867470178e44b";
  }

  static const char* value(const ::orca_room::test_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2d9bb10b98c27a15ULL;
  static const uint64_t static_value2 = 0x5fd867470178e44bULL;
};

template<class ContainerAllocator>
struct DataType< ::orca_room::test_<ContainerAllocator> >
{
  static const char* value()
  {
    return "orca_room/test";
  }

  static const char* value(const ::orca_room::test_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::orca_room::test_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n"
"float64 y\n"
"string name\n"
;
  }

  static const char* value(const ::orca_room::test_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::orca_room::test_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct test_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::orca_room::test_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::orca_room::test_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ORCA_ROOM_MESSAGE_TEST_H
