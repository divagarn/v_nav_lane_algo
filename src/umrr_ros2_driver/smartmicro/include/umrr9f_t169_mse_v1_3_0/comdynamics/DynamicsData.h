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

#ifndef COM_MASTER_UMRR9F_T169_MSE_V1_3_0_COMDYNAMICS_DYNAMICSDATA_H
#define COM_MASTER_UMRR9F_T169_MSE_V1_3_0_COMDYNAMICS_DYNAMICSDATA_H

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
namespace comdynamics
{

// Forward Declarations



class DynamicsData : public com::VersionedIface
{
public:
    DynamicsData() {}

    virtual ~DynamicsData() {}

    ///
    /// ACCESS FUNCTIONS
    ///

    /*!
     *  Source of the dynamics data 
     * [0:Dynamics data is coming from an algorithm 
     * ][1:Dynamics data is coming from CAN 
     * ][2:Dynamics data is coming from CAN (only speed and yawrate) 
     * ][3:Speed from algorithm, Yaw Rate from CAN 
     * ][4:Dynamics data is coming from Ethernet 
     * ] 
     */
    virtual uint8_t GetDynamicsSource() const = 0;
    /*!
     * Vehicle Speed
     */
    virtual com::types::float32_t GetSpeed() const = 0;
    /*!
     * Quality of Vehicle Speed
     */
    virtual com::types::float32_t GetSpeedQuality() const = 0;
    /*!
     * Yaw Rate
     */
    virtual com::types::float32_t GetYawRate() const = 0;
    /*!
     * Quality of Yaw Rate
     */
    virtual com::types::float32_t GetYawRateQuality() const = 0;
    /*!
     * Curve radius coefficient-2 used in road course prediction
     */
    virtual com::types::float32_t GetCurveRadiusCoeff2() const = 0;
    /*!
     * Curve radius maximum length used in road course prediction
     */
    virtual com::types::float32_t GetCurveRadiusMaxLength() const = 0;

};
}  // comdynamics
}  // umrr9f_t169_mse_v1_3_0
}  // namespace master
}  // namespace com

#endif  // COM_MASTER_UMRR9F_T169_MSE_V1_3_0_COMDYNAMICS_DYNAMICSDATA_H