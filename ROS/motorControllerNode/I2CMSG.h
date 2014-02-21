/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/viki/AggreGator_ws/src/Remote_Control_Code/msg/I2CMSG.msg
 *
 */


#ifndef REMOTE_CONTROL_CODE_MESSAGE_I2CMSG_H
#define REMOTE_CONTROL_CODE_MESSAGE_I2CMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace Remote_Control_Code
{
template <class ContainerAllocator>
struct I2CMSG_
{
  typedef I2CMSG_<ContainerAllocator> Type;

  I2CMSG_()
    : addr(0)
    , data(0)  {
    }
  I2CMSG_(const ContainerAllocator& _alloc)
    : addr(0)
    , data(0)  {
    }



   typedef int16_t _addr_type;
  _addr_type addr;

   typedef int16_t _data_type;
  _data_type data;




  typedef boost::shared_ptr< ::Remote_Control_Code::I2CMSG_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::Remote_Control_Code::I2CMSG_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct I2CMSG_

typedef ::Remote_Control_Code::I2CMSG_<std::allocator<void> > I2CMSG;

typedef boost::shared_ptr< ::Remote_Control_Code::I2CMSG > I2CMSGPtr;
typedef boost::shared_ptr< ::Remote_Control_Code::I2CMSG const> I2CMSGConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::Remote_Control_Code::I2CMSG_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::Remote_Control_Code::I2CMSG_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace Remote_Control_Code

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'Remote_Control_Code': ['/home/viki/AggreGator_ws/src/Remote_Control_Code/msg'], 'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::Remote_Control_Code::I2CMSG_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::Remote_Control_Code::I2CMSG_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::Remote_Control_Code::I2CMSG_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::Remote_Control_Code::I2CMSG_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::Remote_Control_Code::I2CMSG_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::Remote_Control_Code::I2CMSG_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::Remote_Control_Code::I2CMSG_<ContainerAllocator> >
{
  static const char* value()
  {
    return "605d530a0c28466204596095bd93b9c5";
  }

  static const char* value(const ::Remote_Control_Code::I2CMSG_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x605d530a0c284662ULL;
  static const uint64_t static_value2 = 0x04596095bd93b9c5ULL;
};

template<class ContainerAllocator>
struct DataType< ::Remote_Control_Code::I2CMSG_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Remote_Control_Code/I2CMSG";
  }

  static const char* value(const ::Remote_Control_Code::I2CMSG_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::Remote_Control_Code::I2CMSG_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 addr\n\
int16 data\n\
";
  }

  static const char* value(const ::Remote_Control_Code::I2CMSG_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::Remote_Control_Code::I2CMSG_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.addr);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct I2CMSG_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::Remote_Control_Code::I2CMSG_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::Remote_Control_Code::I2CMSG_<ContainerAllocator>& v)
  {
    s << indent << "addr: ";
    Printer<int16_t>::stream(s, indent + "  ", v.addr);
    s << indent << "data: ";
    Printer<int16_t>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // REMOTE_CONTROL_CODE_MESSAGE_I2CMSG_H
