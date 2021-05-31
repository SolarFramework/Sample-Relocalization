#include "xpcf/module/ModuleFactory.h"
#include "SolARRelocalizationPipeline.h"

/**
 *  @ingroup xpcfmodule
 */
/**
  * Declare module.
  */
// Declaration of the module embedding the Slam pipeline
XPCF_DECLARE_MODULE("{29c5afa7-df22-4704-b5e8-6c0d4ee80036}", "PipelineRelocalization",
                    "Relocalization vision pipeline based on SolAR Framework")

/**
 * This method is the module entry point.
 * XPCF uses this method to create components available in the module.
 *
 * Each component exposed must be declared inside a xpcf::tryCreateComponent<ComponentType>() call.
 */
extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const boost::uuids::uuid& componentUUID, SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
	xpcf::XPCFErrorCode errCode = xpcf::XPCFErrorCode::_FAIL;
    errCode = xpcf::tryCreateComponent<SolAR::PIPELINES::RELOCALIZATION::SolARRelocalizationPipeline>(componentUUID, interfaceRef);

	return errCode;
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::PIPELINES::RELOCALIZATION::SolARRelocalizationPipeline)
XPCF_END_COMPONENTS_DECLARATION
