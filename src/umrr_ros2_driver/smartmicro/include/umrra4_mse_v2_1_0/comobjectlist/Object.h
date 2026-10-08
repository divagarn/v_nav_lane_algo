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

#ifndef COM_MASTER_UMRRA4_MSE_V2_1_0_COMOBJECTLIST_OBJECT_H
#define COM_MASTER_UMRRA4_MSE_V2_1_0_COMOBJECTLIST_OBJECT_H

#include <ExternalTypes.h>
#include <VersionedIface.h>


#include <memory>
#include <vector>

namespace com
{
namespace master
{
namespace umrra4_mse_v2_1_0
{
namespace comobjectlist
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
     * X-component of the position, depends on selected reference point
     */
    virtual com::types::float32_t GetPosX() const = 0;
    /*!
     * Y-component of the position, depends on selected reference point
     */
    virtual com::types::float32_t GetPosY() const = 0;
    /*!
     * Z-component of the position, depends on selected reference point
     */
    virtual com::types::float32_t GetPosZ() const = 0;
    /*!
     * Absolute object speed
     */
    virtual com::types::float32_t GetSpeedAbs() const = 0;
    /*!
     * Heading of the object [-PI, PI]
     */
    virtual com::types::float32_t GetHeading() const = 0;
    /*!
     * Length of the object
     */
    virtual com::types::float32_t GetLength() const = 0;
    /*!
     * Mileage of the object
     */
    virtual com::types::float32_t GetMileage() const = 0;
    /*!
     * Quality of the object
     */
    virtual com::types::float32_t GetQuality() const = 0;
    /*!
     * Object acceleration
     */
    virtual com::types::float32_t GetAcceleration() const = 0;
    /*!
     * Unique tag number for identification of the object
     */
    virtual int16_t GetObjectId() const = 0;
    /*!
     * Cycles since last update
     */
    virtual uint16_t GetIdleCycles() const = 0;
    /*!
     * Index of spline
     */
    virtual uint16_t GetSplineIdx() const = 0;
    /*!
     * Class of the object
     */
    virtual uint8_t GetObjectClass() const = 0;
    /*!
     * Status of the object[bit1:Set to 1 if object is a guardrail object, otherwise set to 0]
     */
    virtual uint8_t GetStatus() const = 0;

};
}  // comobjectlist
}  // umrra4_mse_v2_1_0
}  // namespace master
}  // namespace com

#endif  // COM_MASTER_UMRRA4_MSE_V2_1_0_COMOBJECTLIST_OBJECT_H