/**************************************************************************
Copyright (c) 2021, s.m.s, smart microwave sensors GmbH, Brunswick, Germany

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
**************************************************************************/

#ifndef COM_MASTER_UMRRA4_MSE_V1_0_0_COMOBJECTBASELIST_OBJECT_H
#define COM_MASTER_UMRRA4_MSE_V1_0_0_COMOBJECTBASELIST_OBJECT_H

#include <ExternalTypes.h>
#include <VersionedIface.h>


#include <memory>
#include <vector>

namespace com
{
namespace master
{
namespace umrra4_mse_v1_0_0
{
namespace comobjectbaselist
{

// Forward Declarations



class Object : public com::VersionedIface
{
public:
    Object() {}

    virtual ~Object() {}

    ///
    /// ACCESS FUNCTIONS
    ///

    /*!
     * X-component of the position, depends on selected reference point.
     */
    virtual com::types::float32_t GetXPoint1() const = 0;
    /*!
     * Y-component of the position, depends on selected reference point.
     */
    virtual com::types::float32_t GetYPoint1() const = 0;
    /*!
     * Absolute object speed.
     */
    virtual com::types::float32_t GetSpeedAbs() const = 0;
    /*!
     * Heading of the object.
     */
    virtual com::types::float32_t GetHeadingDeg() const = 0;
    /*!
     * Length of the object.
     */
    virtual com::types::float32_t GetObjectLen() const = 0;
    /*!
     * Quality of the object.
     */
    virtual com::types::float32_t GetQuality() const = 0;
    /*!
     * Object acceleration.
     */
    virtual com::types::float32_t GetAcceleration() const = 0;
    /*!
     * Z-component of the position, depends on selected reference point.
     */
    virtual com::types::float32_t GetZPoint1() const = 0;
    /*!
     * Unique tag number for identification of the object.
     */
    virtual uint16_t GetObjectId() const = 0;

};
}  // comobjectbaselist
}  // umrra4_mse_v1_0_0
}  // namespace master
}  // namespace com

#endif  // COM_MASTER_UMRRA4_MSE_V1_0_0_COMOBJECTBASELIST_OBJECT_H