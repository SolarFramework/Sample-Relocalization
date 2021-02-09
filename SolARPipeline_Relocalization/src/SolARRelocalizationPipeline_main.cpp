#include "xpcf/module/ModuleFactory.h"
#include "SolARRelocalizationPipeline.h"
#include "core/Log.h"

// Declaration of the module embedding the Slam pipeline
XPCF_DECLARE_MODULE("29c5afa7-df22-4704-b5e8-6c0d4ee80036", "RelocalizationModule", "The module embedding a pipeline to camera relocalization")

extern "C" XPCF_MODULEHOOKS_API xpcf::XPCFErrorCode XPCF_getComponent(const boost::uuids::uuid& componentUUID, SRef<xpcf::IComponentIntrospect>& interfaceRef)
{
	xpcf::XPCFErrorCode errCode = xpcf::XPCFErrorCode::_FAIL;
	errCode = xpcf::tryCreateComponent<SolAR::PIPELINES::SolARRelocalizationPipeline>(componentUUID, interfaceRef);

	return errCode;
}

XPCF_BEGIN_COMPONENTS_DECLARATION
XPCF_ADD_COMPONENT(SolAR::PIPELINES::SolARRelocalizationPipeline)
XPCF_END_COMPONENTS_DECLARATION
