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
#include <thread>

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
        declareInjectable<api::input::files::IWorldGraphLoader>(m_worldGraphLoader);
        declareInjectable<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>(m_pnp);
        declareInjectable<SolAR::api::features::I2DTrackablesDetector>(m_QRCodesDetector, "QRcode");
        declareInjectable<SolAR::api::features::I2DTrackablesDetector>(m_fiducialMarkersDetector, "Fiducial");
        declareInjectable<SolAR::api::geom::IProject>(m_projector);
        declareProperty("maxReprojError", m_maxReprojError);
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
    if (m_initOK) {
        LOG_WARNING("SolARRelocalizationMarkerPipeline has already been initialized");
        return FrameworkReturnCode::_SUCCESS;
    }
    // load trackables of the world graph
    std::vector<SRef<Trackable>> trackables;
    if (m_worldGraphLoader->load(trackables) == FrameworkReturnCode::_ERROR_) {
        LOG_ERROR("Error during load the world graph file");
        return FrameworkReturnCode::_ERROR_;
    }
	else
	{        
        int nbFloatingQRCodes(0);
        int nbFloatingFiducials(0);
        // set trackables to detector
        std::vector<SRef<Trackable>> fiducialMarkers,  qrCodes;
        for (const auto& trackable : trackables)
            if (trackable->getType() == FIDUCIAL_MARKER) {
                fiducialMarkers.push_back(trackable);
                SRef<FiducialMarker> fiducial = xpcf::utils::dynamic_pointer_cast<FiducialMarker>(trackable);
                // get 3D pattern points
                std::vector<Point3Df> pts3D;
                if (!fiducial->getTransform3D().matrix().isZero())
                    fiducial->getWorldCorners(pts3D);
                else
                    nbFloatingFiducials++;
                m_fiducials3DPoints.push_back(pts3D);
            }
            else if (trackable->getType() == QRCODE_MARKER) {
                qrCodes.push_back(trackable);
                SRef<QRCode> qrCode = xpcf::utils::dynamic_pointer_cast<QRCode>(trackable);
                // get 3D pattern points
                std::vector<Point3Df> pts3D;
                if (!qrCode->getTransform3D().matrix().isZero())
                    qrCode->getWorldCorners(pts3D);
                else
                    nbFloatingQRCodes++;
                m_QRCodes3DPoints.push_back(pts3D);
            }
        m_nbFiducialMarkers = fiducialMarkers.size();
        m_nbQRCodes = qrCodes.size();
        LOG_INFO("Number of fiducial markers: {}", m_nbFiducialMarkers);
        LOG_INFO("Number of QRcode markers: {}", m_nbQRCodes);
        LOG_DEBUG("Number of floating fiducial markers: {}", nbFloatingFiducials);
        LOG_DEBUG("Number of floating QRcode markers: {}", nbFloatingQRCodes);
        if (m_fiducialMarkersDetector->setTrackables(fiducialMarkers) != FrameworkReturnCode::_SUCCESS) {
            LOG_ERROR("Cannot set fiducial marker to detector");
            return FrameworkReturnCode::_ERROR_;
        }
        if (m_QRCodesDetector->setTrackables(qrCodes) != FrameworkReturnCode::_SUCCESS) {
            LOG_ERROR("Cannot set QR codes to detector");
            return FrameworkReturnCode::_ERROR_;
        }
	}
	m_initOK = true;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationMarkerPipeline::setCameraParameters(const CameraParameters & cameraParams) 
{
	LOG_DEBUG("SolARRelocalizationMarkerPipeline::setCameraParameters");
    m_camParams = cameraParams;
	m_cameraOK = true;
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationMarkerPipeline::getCameraParameters(CameraParameters & cameraParams) const 
{
    LOG_DEBUG("SolARRelocalizationMarkerPipeline::getCameraParameters");
    if (m_cameraOK) {
        cameraParams = m_camParams;
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
                                                                                SolAR::datastructure::Transform3Df& pose,
                                                                                float_t & confidence)
{
    LOG_DEBUG("SolARRelocalizationMarkerPipeline::relocalizeProcessRequest");
    confidence = 1.f;
    if (!m_initOK) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }
    else if (!m_cameraOK){
        LOG_ERROR("Camera parameters have not been set");
        return FrameworkReturnCode::_ERROR_;
    }
    std::vector<Point2Df>	pts2D;
    std::vector<Point3Df>	pts3D;
    std::vector<std::vector<Point2Df>> cornersQRCodes, cornersFiducials;
    int nbFoundFiducials(0);
    int nbFoundQRCodes(0);
    SRef<Image> image2 = image->copy();
    //  detect fiducial markers
    std::thread threadFiducial(&SolARRelocalizationMarkerPipeline::fiducialMarkersDetection, this, image, std::ref(cornersFiducials));
    //  detect qr codes
    std::thread threadQRCode(&SolARRelocalizationMarkerPipeline::qrCodesDetection, this, image2, std::ref(cornersQRCodes));
    threadFiducial.join();
    threadQRCode.join();
    //  extract 2D-3D correspondences from fiducial markers
    if (cornersFiducials.size() != m_fiducials3DPoints.size())
        return FrameworkReturnCode::_ERROR_;

    for (int i = 0; i < m_nbFiducialMarkers; ++i) {
        if ((cornersFiducials[i].size() > 0) && (m_fiducials3DPoints[i].size() > 0)) {
            pts2D.insert(pts2D.end(), cornersFiducials[i].begin(), cornersFiducials[i].end());
            pts3D.insert(pts3D.end(), m_fiducials3DPoints[i].begin(), m_fiducials3DPoints[i].end());
            nbFoundFiducials++;
        }
    }

    //  extract 2D-3D correspondences from QR codes
    if (cornersQRCodes.size() != m_QRCodes3DPoints.size())
        return FrameworkReturnCode::_ERROR_;

    for (int i = 0; i < m_nbQRCodes; ++i) {
        if ((cornersQRCodes[i].size() > 0) && (m_QRCodes3DPoints[i].size() > 0)) {
            pts2D.insert(pts2D.end(), cornersQRCodes[i].begin(), cornersQRCodes[i].end());
            pts3D.insert(pts3D.end(), m_QRCodes3DPoints[i].begin(), m_QRCodes3DPoints[i].end());
            nbFoundQRCodes++;
        }
    }


    LOG_DEBUG("Number of detected fiducial markers: {}", nbFoundFiducials);
    LOG_DEBUG("Number of detected QRcode markers: {}", nbFoundQRCodes);
    if ((nbFoundFiducials + nbFoundQRCodes) == 0)
        return FrameworkReturnCode::_ERROR_;

    // Compute the pose of the camera using a Perspective n Points algorithm using all corners of the detected markers
    if (m_pnp->estimate(pts2D, pts3D, m_camParams, pose) == FrameworkReturnCode::_SUCCESS)
    {
        std::vector<Point2Df> projected2DPts;
        m_projector->project(pts3D, pose, m_camParams, projected2DPts);
        float errorReproj(0.f);
        for (int j = 0; j < projected2DPts.size(); ++j)
            errorReproj += (projected2DPts[j] - pts2D[j]).norm();
        errorReproj /= projected2DPts.size();
        LOG_DEBUG("Mean reprojection error: {}", errorReproj);
        if (errorReproj < m_maxReprojError)
            return FrameworkReturnCode::_SUCCESS;
        pose = Transform3Df::Identity();
    }
    return FrameworkReturnCode::_ERROR_;

}

void SolARRelocalizationMarkerPipeline::fiducialMarkersDetection(const SRef<SolAR::datastructure::Image> image,
                                                                 std::vector<std::vector<SolAR::datastructure::Point2Df>> & corners)
{
    m_fiducialMarkersDetector->detect(image, corners);
}

void SolARRelocalizationMarkerPipeline::qrCodesDetection(const SRef<SolAR::datastructure::Image> image,
                                                         std::vector<std::vector<SolAR::datastructure::Point2Df>> & corners)
{
    m_QRCodesDetector->detect(image, corners);
}

} // namespace RELOCALIZATION
} // namespace PIPELINES
} // namespace SolAR
