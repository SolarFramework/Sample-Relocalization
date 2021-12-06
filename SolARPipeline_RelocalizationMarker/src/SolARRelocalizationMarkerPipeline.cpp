/**
 * @copyright Copyright (c) 2021 All Right Reserved, B-com http://www.b-com.com/
 *
 * This file is subject to the B<>Com License.
 * All other rights reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 */

#include "xpcf/module/ModuleFactory.h"
#include "SolARRelocalizationMarkerPipeline.h"
#include "core/Log.h"
#include "boost/log/core/core.hpp"

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace PIPELINES {
namespace RELOCALIZATION {

SolARRelocalizationMarkerPipeline::SolARRelocalizationMarkerPipeline():ConfigurableBase(xpcf::toUUID<SolARRelocalizationMarkerPipeline>())
{    
    try {
        declareInterface<api::pipeline::IRelocalizationPipeline>(this);
        LOG_DEBUG("Components injection declaration");        
        declareInjectable<api::input::files::ITrackableLoader>(m_trackableLoader);
        declareInjectable<api::solver::pose::ITrackablePose>(m_poseEstimator);
        LOG_DEBUG("All component injections declared");
    }
    catch (xpcf::Exception & e) {
        LOG_ERROR("The following exception has been caught {}", e.what());
    }
	LOG_DEBUG(" SolARRelocalizationMarkerPipeline constructor");
}

void SolARRelocalizationMarkerPipeline::onInjected() {
    LOG_DEBUG("SolARRelocalizationMarkerPipeline::onInjected");
}

SolARRelocalizationMarkerPipeline::~SolARRelocalizationMarkerPipeline()
{
    LOG_DEBUG("SolARRelocalizationMarkerPipeline destructor")
}

FrameworkReturnCode SolARRelocalizationMarkerPipeline::init()
{
    LOG_DEBUG("SolARRelocalizationMarkerPipeline::init");
	// Load Trackable
	SRef<Trackable> trackable;
	if (m_trackableLoader->loadTrackable(trackable) != FrameworkReturnCode::_SUCCESS)
	{
		LOG_ERROR("cannot load marker");
		return FrameworkReturnCode::_ERROR_;
	}
	else
	{
		if (m_poseEstimator->setTrackable(trackable) != FrameworkReturnCode::_SUCCESS)
		{
			LOG_ERROR("Cannot set trackable to pose estimator");
			return FrameworkReturnCode::_ERROR_;
		}
	}
	m_initOK = true;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationMarkerPipeline::setCameraParameters(const CameraParameters & cameraParams) 
{
	LOG_DEBUG("SolARRelocalizationMarkerPipeline::setCameraParameters");
    m_calibration = cameraParams.intrinsic;
    m_distortion = cameraParams.distortion;
	m_poseEstimator->setCameraParameters(m_calibration, m_distortion);
	m_cameraOK = true;
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationMarkerPipeline::getCameraParameters(CameraParameters & cameraParams) const 
{
    LOG_DEBUG("SolARRelocalizationMarkerPipeline::getCameraParameters");
    if (m_cameraOK) {
        cameraParams.intrinsic = m_calibration;
        cameraParams.distortion = m_distortion;
        return FrameworkReturnCode::_SUCCESS;
    }
    else {
        LOG_ERROR("Camera parameters have not been set");
        return FrameworkReturnCode::_ERROR_;
    }
}

FrameworkReturnCode SolARRelocalizationMarkerPipeline::start() 
{
    LOG_DEBUG("SolARRelocalizationMarkerPipeline::start");

    if (!m_initOK) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }
    else if (!m_cameraOK){
        LOG_ERROR("Camera parameters have not been set");
        return FrameworkReturnCode::_ERROR_;
    }
    else {
        return FrameworkReturnCode::_SUCCESS;
    }
}

FrameworkReturnCode SolARRelocalizationMarkerPipeline::stop()
{
    LOG_DEBUG("SolARRelocalizationMarkerPipeline::stop");
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationMarkerPipeline::relocalizeProcessRequest(const SRef<SolAR::datastructure::Image> image,
                                                                          SolAR::datastructure::Transform3Df& pose, float_t & confidence) 
{
    LOG_DEBUG("SolARRelocalizationMarkerPipeline::relocalizeProcessRequest");
    confidence = 1.f;
    if ((m_initOK) && (m_cameraOK)) {
		return m_poseEstimator->estimate(image, pose);
    }
    else {
        if (!m_initOK) {
            LOG_ERROR("Pipeline has not been initialized");
            return FrameworkReturnCode::_ERROR_;
        }
        else if (!m_cameraOK){
            LOG_ERROR("Camera parameters have not been set");
            return FrameworkReturnCode::_ERROR_;
        }
    }
}

} // namespace RELOCALIZATION
} // namespace PIPELINES
} // namespace SolAR
