// Copyright (c) 2015-2018, Dataspeed Inc., 2018-2020 New Eagle, All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// * Neither the name of the {copyright_holder} nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef ANALOGX_INTERFACE_CAN__DISPATCH_HPP_
#define ANALOGX_INTERFACE_CAN__DISPATCH_HPP_

namespace analogx_interface_can
{

/** \brief Enumeration of CAN message IDs */
enum ListMessageIDs
{
  ID_ANALOGX1 = 0xe4614,
  ID_BRAKE_TEMP = 0x04c5,
  ID_FRONT_LEFT_EXTERNAL_TIRE_TEMP = 0xa120,
  ID_FRONT_RIGHT_EXTERNAL_TIRE_TEMP = 0xa220,
  ID_REAR_LEFT_EXTERNAL_TIRE_TEMP = 0xa320,
  ID_REAR_RIGHT_EXTERNAL_TIRE_TEMP = 0xa420,
  ID_BRAKE_SENSOR_BODY_TEMP = 0x04c8,
};

}  // namespace analogx_interface_can

#endif  // ANALOGX_INTERFACE_CAN__DISPATCH_HPP_
