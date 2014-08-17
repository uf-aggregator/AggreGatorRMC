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
 * Auto-generated by gensrv_cpp from file /home/faytxzen/GitHub/AggreGatorRMC/AggreGator_ws/src/hardware_interface/srv/ReadI2C.srv
 *
 */


#ifndef HARDWARE_INTERFACE_MESSAGE_READI2C_H
#define HARDWARE_INTERFACE_MESSAGE_READI2C_H

#include <ros/service_traits.h>


#include <hardware_interface/ReadI2CRequest.h>
#include <hardware_interface/ReadI2CResponse.h>


namespace hardware_interface
{

struct ReadI2C
{

typedef ReadI2CRequest Request;
typedef ReadI2CResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ReadI2C
} // namespace hardware_interface


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::hardware_interface::ReadI2C > {
  static const char* value()
  {
    return "22e5f5eaf6ee0bc3b5bf519623591445";
  }

  static const char* value(const ::hardware_interface::ReadI2C&) { return value(); }
};

template<>
struct DataType< ::hardware_interface::ReadI2C > {
  static const char* value()
  {
    return "hardware_interface/ReadI2C";
  }

  static const char* value(const ::hardware_interface::ReadI2C&) { return value(); }
};


// service_traits::MD5Sum< ::hardware_interface::ReadI2CRequest> should match 
// service_traits::MD5Sum< ::hardware_interface::ReadI2C > 
template<>
struct MD5Sum< ::hardware_interface::ReadI2CRequest>
{
  static const char* value()
  {
    return MD5Sum< ::hardware_interface::ReadI2C >::value();
  }
  static const char* value(const ::hardware_interface::ReadI2CRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::hardware_interface::ReadI2CRequest> should match 
// service_traits::DataType< ::hardware_interface::ReadI2C > 
template<>
struct DataType< ::hardware_interface::ReadI2CRequest>
{
  static const char* value()
  {
    return DataType< ::hardware_interface::ReadI2C >::value();
  }
  static const char* value(const ::hardware_interface::ReadI2CRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::hardware_interface::ReadI2CResponse> should match 
// service_traits::MD5Sum< ::hardware_interface::ReadI2C > 
template<>
struct MD5Sum< ::hardware_interface::ReadI2CResponse>
{
  static const char* value()
  {
    return MD5Sum< ::hardware_interface::ReadI2C >::value();
  }
  static const char* value(const ::hardware_interface::ReadI2CResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::hardware_interface::ReadI2CResponse> should match 
// service_traits::DataType< ::hardware_interface::ReadI2C > 
template<>
struct DataType< ::hardware_interface::ReadI2CResponse>
{
  static const char* value()
  {
    return DataType< ::hardware_interface::ReadI2C >::value();
  }
  static const char* value(const ::hardware_interface::ReadI2CResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // HARDWARE_INTERFACE_MESSAGE_READI2C_H
