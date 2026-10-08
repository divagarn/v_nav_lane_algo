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

#ifndef COM_MASTER_UMRR9F_T169_MSE_V1_3_0_COMOBJECTLIST_COMOBJECTLIST_H
#define COM_MASTER_UMRR9F_T169_MSE_V1_3_0_COMOBJECTLIST_COMOBJECTLIST_H

#include <ExternalTypes.h>
#include <VersionedIface.h>

#include <umrr9f_t169_mse_v1_3_0/comobjectlist/PortHeader.h>
#include <umrr9f_t169_mse_v1_3_0/comobjectlist/ObjectListHeader.h>
#include <umrr9f_t169_mse_v1_3_0/comobjectlist/Object.h>

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

class PortHeader;
class ObjectListHeader;
class Object;


class ComObjectList : public com::VersionedIface
{
public:
    ComObjectList() {}

    virtual ~ComObjectList() {}

    ///
    /// ACCESS FUNCTIONS
    ///

    /*!
     * smartmicro Generic Port Header
     */
    virtual std::shared_ptr<PortHeader> GetPortHeader() const = 0;
    /*!
     * Static header of the object list
     */
    virtual std::shared_ptr<ObjectListHeader> GetObjectListHeader() const = 0;
    /*!
     * Represent a single object
     */
    virtual const std::vector<std::shared_ptr<Object>>& GetObjectList() const = 0;

};
}  // comobjectlist
}  // umrr9f_t169_mse_v1_3_0
}  // namespace master
}  // namespace com

#endif  // COM_MASTER_UMRR9F_T169_MSE_V1_3_0_COMOBJECTLIST_COMOBJECTLIST_H