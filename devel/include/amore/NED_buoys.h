// Generated by gencpp from file amore/NED_buoys.msg
// DO NOT EDIT!


#ifndef AMORE_MESSAGE_NED_BUOYS_H
#define AMORE_MESSAGE_NED_BUOYS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PointStamped.h>

namespace amore
{
template <class ContainerAllocator>
struct NED_buoys_
{
  typedef NED_buoys_<ContainerAllocator> Type;

  NED_buoys_()
    : buoys()
    , quantity(0)  {
    }
  NED_buoys_(const ContainerAllocator& _alloc)
    : buoys(_alloc)
    , quantity(0)  {
  (void)_alloc;
    }



   typedef std::vector< ::geometry_msgs::PointStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::PointStamped_<ContainerAllocator> >::other >  _buoys_type;
  _buoys_type buoys;

   typedef int32_t _quantity_type;
  _quantity_type quantity;





  typedef boost::shared_ptr< ::amore::NED_buoys_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::amore::NED_buoys_<ContainerAllocator> const> ConstPtr;

}; // struct NED_buoys_

typedef ::amore::NED_buoys_<std::allocator<void> > NED_buoys;

typedef boost::shared_ptr< ::amore::NED_buoys > NED_buoysPtr;
typedef boost::shared_ptr< ::amore::NED_buoys const> NED_buoysConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::amore::NED_buoys_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::amore::NED_buoys_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::amore::NED_buoys_<ContainerAllocator1> & lhs, const ::amore::NED_buoys_<ContainerAllocator2> & rhs)
{
  return lhs.buoys == rhs.buoys &&
    lhs.quantity == rhs.quantity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::amore::NED_buoys_<ContainerAllocator1> & lhs, const ::amore::NED_buoys_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace amore

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::amore::NED_buoys_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::amore::NED_buoys_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::amore::NED_buoys_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::amore::NED_buoys_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::amore::NED_buoys_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::amore::NED_buoys_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::amore::NED_buoys_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8674d58f3fb14856192d33f62b336838";
  }

  static const char* value(const ::amore::NED_buoys_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8674d58f3fb14856ULL;
  static const uint64_t static_value2 = 0x192d33f62b336838ULL;
};

template<class ContainerAllocator>
struct DataType< ::amore::NED_buoys_<ContainerAllocator> >
{
  static const char* value()
  {
    return "amore/NED_buoys";
  }

  static const char* value(const ::amore::NED_buoys_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::amore::NED_buoys_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/PointStamped[] buoys\n"
"int32 quantity\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PointStamped\n"
"# This represents a Point with reference coordinate frame and timestamp\n"
"Header header\n"
"Point point\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::amore::NED_buoys_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::amore::NED_buoys_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.buoys);
      stream.next(m.quantity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NED_buoys_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::amore::NED_buoys_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::amore::NED_buoys_<ContainerAllocator>& v)
  {
    s << indent << "buoys[]" << std::endl;
    for (size_t i = 0; i < v.buoys.size(); ++i)
    {
      s << indent << "  buoys[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::PointStamped_<ContainerAllocator> >::stream(s, indent + "    ", v.buoys[i]);
    }
    s << indent << "quantity: ";
    Printer<int32_t>::stream(s, indent + "  ", v.quantity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AMORE_MESSAGE_NED_BUOYS_H
