// Generated by gencpp from file amore/NED_buoy.msg
// DO NOT EDIT!


#ifndef AMORE_MESSAGE_NED_BUOY_H
#define AMORE_MESSAGE_NED_BUOY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace amore
{
template <class ContainerAllocator>
struct NED_buoy_
{
  typedef NED_buoy_<ContainerAllocator> Type;

  NED_buoy_()
    : position()
    , id()  {
    }
  NED_buoy_(const ContainerAllocator& _alloc)
    : position(_alloc)
    , id(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _id_type;
  _id_type id;





  typedef boost::shared_ptr< ::amore::NED_buoy_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::amore::NED_buoy_<ContainerAllocator> const> ConstPtr;

}; // struct NED_buoy_

typedef ::amore::NED_buoy_<std::allocator<void> > NED_buoy;

typedef boost::shared_ptr< ::amore::NED_buoy > NED_buoyPtr;
typedef boost::shared_ptr< ::amore::NED_buoy const> NED_buoyConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::amore::NED_buoy_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::amore::NED_buoy_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::amore::NED_buoy_<ContainerAllocator1> & lhs, const ::amore::NED_buoy_<ContainerAllocator2> & rhs)
{
  return lhs.position == rhs.position &&
    lhs.id == rhs.id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::amore::NED_buoy_<ContainerAllocator1> & lhs, const ::amore::NED_buoy_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace amore

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::amore::NED_buoy_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::amore::NED_buoy_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::amore::NED_buoy_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::amore::NED_buoy_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::amore::NED_buoy_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::amore::NED_buoy_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::amore::NED_buoy_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b5a6f8471e86877e93afa9bcacce1774";
  }

  static const char* value(const ::amore::NED_buoy_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb5a6f8471e86877eULL;
  static const uint64_t static_value2 = 0x93afa9bcacce1774ULL;
};

template<class ContainerAllocator>
struct DataType< ::amore::NED_buoy_<ContainerAllocator> >
{
  static const char* value()
  {
    return "amore/NED_buoy";
  }

  static const char* value(const ::amore::NED_buoy_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::amore::NED_buoy_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point position\n"
"string id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::amore::NED_buoy_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::amore::NED_buoy_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.position);
      stream.next(m.id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NED_buoy_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::amore::NED_buoy_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::amore::NED_buoy_<ContainerAllocator>& v)
  {
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AMORE_MESSAGE_NED_BUOY_H
