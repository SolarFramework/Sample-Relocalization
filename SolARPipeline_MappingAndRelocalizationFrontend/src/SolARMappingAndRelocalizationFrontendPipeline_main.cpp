/**
 * @copyright Copyright (c) 2021 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "xpcf/module/ModuleFactory.h"
#include "SolARMappingAndRelocalizationFrontendPipeline.h"

/**
 *  @ingroup xpcfmodule
 */
/**
  * Declare module.
  */
// Declaration of the module embedding the Slam pipeline
XPCF_DECLARE_MODULE("{970415c6-56b2-11ec-bf63-0242ac130002}", "PipelineMappingAndRelocalizationFrontend",
                    "Mapping and Relocalization front end pipeline based on SolAR Framework")

/**
 * This method is the module entry point.
 * XPCF uses this method to create components available in the module.
 *
 * Each component exposed must be declared inside a xpcf::tryCreateComponent<ComponentType>() call.
 */
extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const boost::uuids::uuid& componentUUID, SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
	xpcf::XPCFErrorCode errCode = xpcf::XPCFErrorCode::_FAIL;
    errCode = xpcf::tryCreateComponent<SolAR::PIPELINES::RELOCALIZATION::SolARMappingAndRelocalizationFrontendPipeline>(componentUUID, interfaceRef);

	return errCode;
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::PIPELINES::RELOCALIZATION::SolARMappingAndRelocalizationFrontendPipeline)
XPCF_END_COMPONENTS_DECLARATION
