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

#ifndef COM_MASTER_UMRR9F_T169_MSE_V1_3_0_COMDYNAMICS_COMDYNAMICS_H
#define COM_MASTER_UMRR9F_T169_MSE_V1_3_0_COMDYNAMICS_COMDYNAMICS_H

#include <ExternalTypes.h>
#include <VersionedIface.h>

#include <umrr9f_t169_mse_v1_3_0/comdynamics/PortHeader.h>
#include <umrr9f_t169_mse_v1_3_0/comdynamics/DynamicsData.h>

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

class PortHeader;
class DynamicsData;


class ComDynamics : public com::VersionedIface
{
public:
    ComDynamics() {}

    virtual ~ComDynamics() {}

    ///
    /// ACCESS FUNCTIONS
    ///

    /*!
     * smartmicro Generic Port Header
     */
    virtual std::shared_ptr<PortHeader> GetPortHeader() const = 0;
    /*!
     * Static payload
     */
    virtual std::shared_ptr<DynamicsData> GetDynamicsData() const = 0;

};
}  // comdynamics
}  // umrr9f_t169_mse_v1_3_0
}  // namespace master
}  // namespace com

#endif  // COM_MASTER_UMRR9F_T169_MSE_V1_3_0_COMDYNAMICS_COMDYNAMICS_H