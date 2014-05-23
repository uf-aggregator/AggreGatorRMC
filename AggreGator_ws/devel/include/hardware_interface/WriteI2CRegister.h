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
 * Auto-generated by genmsg_cpp from file /home/fnivek/GitHub/NASAboticsCode/AggreGator_ws/src/hardware_interface/msg/WriteI2CRegister.msg
 *
 */


#ifndef HARDWARE_INTERFACE_MESSAGE_WRITEI2CREGISTER_H
#define HARDWARE_INTERFACE_MESSAGE_WRITEI2CREGISTER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hardware_interface
{
template <class ContainerAllocator>
struct WriteI2CRegister_
{
  typedef WriteI2CRegister_<ContainerAllocator> Type;

  WriteI2CRegister_()
    : addr(0)
    , reg(0)
    , data()  {
    }
  WriteI2CRegister_(const ContainerAllocator& _alloc)
    : addr(0)
    , reg(0)
    , data(_alloc)  {
    }



   typedef int8_t _addr_type;
  _addr_type addr;

   typedef int8_t _reg_type;
  _reg_type reg;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _data_type;
  _data_type data;




  typedef boost::shared_ptr< ::hardware_interface::WriteI2CRegister_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hardware_interface::WriteI2CRegister_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct WriteI2CRegister_

typedef ::hardware_interface::WriteI2CRegister_<std::allocator<void> > WriteI2CRegister;

typedef boost::shared_ptr< ::hardware_interface::WriteI2CRegister > WriteI2CRegisterPtr;
typedef boost::shared_ptr< ::hardware_interface::WriteI2CRegister const> WriteI2CRegisterConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hardware_interface::WriteI2CRegister_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hardware_interface::WriteI2CRegister_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace hardware_interface

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'hardware_interface': ['/home/fnivek/GitHub/NASAboticsCode/AggreGator_ws/src/hardware_interface/msg'], 'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::hardware_interface::WriteI2CRegister_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hardware_interface::WriteI2CRegister_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hardware_interface::WriteI2CRegister_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hardware_interface::WriteI2CRegister_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hardware_interface::WriteI2CRegister_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hardware_interface::WriteI2CRegister_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hardware_interface::WriteI2CRegister_<ContainerAllocator> >
{
  static const char* value()
  {
    return "73fab66272b043e250c3aa49bf42d5bd";
  }

  static const char* value(const ::hardware_interface::WriteI2CRegister_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x73fab66272b043e2ULL;
  static const uint64_t static_value2 = 0x50c3aa49bf42d5bdULL;
};

template<class ContainerAllocator>
struct DataType< ::hardware_interface::WriteI2CRegister_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hardware_interface/WriteI2CRegister";
  }

  static const char* value(const ::hardware_interface::WriteI2CRegister_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hardware_interface::WriteI2CRegister_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 addr\n\
int8 reg\n\
uint8[] data\n\
";
  }

  static const char* value(const ::hardware_interface::WriteI2CRegister_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hardware_interface::WriteI2CRegister_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.addr);
      stream.next(m.reg);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct WriteI2CRegister_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hardware_interface::WriteI2CRegister_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hardware_interface::WriteI2CRegister_<ContainerAllocator>& v)
  {
    s << indent << "addr: ";
    Printer<int8_t>::stream(s, indent + "  ", v.addr);
    s << indent << "reg: ";
    Printer<int8_t>::stream(s, indent + "  ", v.reg);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // HARDWARE_INTERFACE_MESSAGE_WRITEI2CREGISTER_H
