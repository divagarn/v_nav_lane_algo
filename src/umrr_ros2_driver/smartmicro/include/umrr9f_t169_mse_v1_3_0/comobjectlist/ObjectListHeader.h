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

#ifndef COM_MASTER_UMRR9F_T169_MSE_V1_3_0_COMOBJECTLIST_OBJECTLISTHEADER_H
#define COM_MASTER_UMRR9F_T169_MSE_V1_3_0_COMOBJECTLIST_OBJECTLISTHEADER_H

#include <ExternalTypes.h>
#include <VersionedIface.h>


#include <memory>
#include <vector>

namespace com
{
namespace master
{
namespace umrr9f_t169_mse_v1_3_0
{
namespace comobjectlist
{

// Forward Declarations



class ObjectListHeader : public com::VersionedIface
{
public:
    ObjectListHeader() {}

    virtual ~ObjectListHeader() {}

    ///
    /// ACCESS FUNCTIONS
    ///

    /*!
     * Cycle time
     */
    virtual com::types::float32_t GetCycleTime() const = 0;
    /*!
     * Number of objects in the current object list
     */
    virtual uint16_t GetNumberOfObjects() const = 0;
    /*!
     * Selected reference point of the objects.
     */
    virtual uint8_t GetSelectedRefPoint() const = 0;
    /*!
     * Start of radar acquisition for object list (NTP coded)
     */
    virtual uint64_t GetTimestampOfMeasurement() const = 0;

};
}  // comobjectlist
}  // umrr9f_t169_mse_v1_3_0
}  // namespace master
}  // namespace com

#endif  // COM_MASTER_UMRR9F_T169_MSE_V1_3_0_COMOBJECTLIST_OBJECTLISTHEADER_H