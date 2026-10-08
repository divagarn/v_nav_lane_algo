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

#ifndef COM_MASTER_UMRR9F_T169_MSE_V1_1_0_COMOBJECTBASELIST_COMOBJECTBASELISTHEADER_H
#define COM_MASTER_UMRR9F_T169_MSE_V1_1_0_COMOBJECTBASELIST_COMOBJECTBASELISTHEADER_H

#include <ExternalTypes.h>
#include <VersionedIface.h>


#include <memory>
#include <vector>

namespace com
{
namespace master
{
namespace umrr9f_t169_mse_v1_1_0
{
namespace comobjectbaselist
{

// Forward Declarations



class ComObjectBaseListHeader : public com::VersionedIface
{
public:
    ComObjectBaseListHeader() {}

    virtual ~ComObjectBaseListHeader() {}

    ///
    /// ACCESS FUNCTIONS
    ///

    /*!
     * Internal reserved field.
     */
    virtual uint64_t GetInternal1() const = 0;
    /*!
     *  Ego Speed 
     *  Source dependant on dynamics source: 0 = internal ego speed estimation 
     *  1,2 or 3 - ego speed coming from an external source via ETH or CAN 
     *  
     */
    virtual com::types::float32_t GetSpeed() const = 0;
    /*!
     *  Value that ranges from 0-1, indicating how well supported the current input of the ego speed is 
     *  Can behave differently, depending on the source 
     *  
     */
    virtual com::types::float32_t GetSpeedQuality() const = 0;
    /*!
     *  Yaw rate of the ego vehicle 
     *  Is 0 when the dynamics source == 0, as there is no ego yaw rate estimation 
     *  Can be != 0 if it is given by an external source 
     *  
     */
    virtual com::types::float32_t GetYawRate() const = 0;
    /*!
     * Value that ranges from 0-1, indicating how well supported the current input of the ego yaw rate is.
     */
    virtual com::types::float32_t GetYawRateQuality() const = 0;
    /*!
     *  Describing the source of the ego dynamics information 
     * [0:Internal ego speed estimation 
     * ][1:External dynamics input via ETH 
     * ][2:External dynamics input via CAN 
     * ][3:Internal ego speed estimation + optional external yaw rate via CAN 
     * ] 
     */
    virtual uint8_t GetDynamicSource() const = 0;
    /*!
     * Internal reserved field.
     */
    virtual uint64_t GetInternal2() const = 0;
    /*!
     * Number of objects in the current object list.
     */
    virtual uint16_t GetNoOfObjects() const = 0;
    /*!
     * Duration of the current radar cycle in seconds.
     */
    virtual com::types::float32_t GetCycleDuration() const = 0;
    /*!
     *  Keeps a count of the total cycle durations that the radar has undergone since reboot starting from zero 
     *  
     */
    virtual uint32_t GetCycleCount() const = 0;

};
}  // comobjectbaselist
}  // umrr9f_t169_mse_v1_1_0
}  // namespace master
}  // namespace com

#endif  // COM_MASTER_UMRR9F_T169_MSE_V1_1_0_COMOBJECTBASELIST_COMOBJECTBASELISTHEADER_H