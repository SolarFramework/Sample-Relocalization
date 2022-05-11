#include "xpcf/module/ModuleFactory.h"
#include "SolARRelocalizationMarkerPipeline.h"

/**
 *  @ingroup xpcfmodule
 */
/**
  * Declare module.
  */
// Declaration of the module embedding the relocalization marker pipeline
XPCF_DECLARE_MODULE("{ee614014-15cf-465b-93de-7f0dddfc7cee}", "PipelineRelocalizationMarker",
                    "Relocalization pipeline based on marker")

/**
 * This method is the module entry point.
 * XPCF uses this method to create components available in the module.
 *
 * Each component exposed must be declared inside a xpcf::tryCreateComponent<ComponentType>() call.
 */
extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const boost::uuids::uuid& componentUUID, SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
	xpcf::XPCFErrorCode errCode = xpcf::XPCFErrorCode::_FAIL;
    errCode = xpcf::tryCreateComponent<SolAR::PIPELINES::RELOCALIZATION::SolARRelocalizationMarkerPipeline>(componentUUID, interfaceRef);

	return errCode;
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::PIPELINES::RELOCALIZATION::SolARRelocalizationMarkerPipeline)
XPCF_END_COMPONENTS_DECLARATION
