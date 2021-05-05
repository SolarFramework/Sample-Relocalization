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
#include "SolARRelocalizationPipeline.h"
#include "core/Log.h"
#include "boost/log/core/core.hpp"
#include <cmath>

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
    using namespace datastructure;
namespace PIPELINES {
namespace RELOCALIZATION {

#define NB_PROCESS_KEYFRAMES 5

// Public methods

SolARRelocalizationPipeline::SolARRelocalizationPipeline():ConfigurableBase(xpcf::toUUID<SolARRelocalizationPipeline>())
{
    #if NDEBUG
        boost::log::core::get()->set_logging_enabled(false);
    #endif

    LOG_ADD_LOG_TO_CONSOLE();

    LOG_DEBUG(" SolARRelocalizationPipeline constructor");

    try {
        declareInterface<api::pipeline::IRelocalizationPipeline>(this);

        LOG_DEBUG("Components injection declaration");

        declareInjectable<api::storage::IPointCloudManager>(m_pointCloudManager);
        declareInjectable<api::storage::IKeyframesManager>(m_keyframesManager);
        declareInjectable<api::storage::ICovisibilityGraph>(m_covisibilityGraph);
        declareInjectable<api::reloc::IKeyframeRetriever>(m_kfRetriever);
        declareInjectable<api::solver::map::IMapper>(m_mapper);
        declareInjectable<api::features::IKeypointDetector>(m_keypointsDetector);
        declareInjectable<api::features::IDescriptorsExtractor>(m_descriptorExtractor);
        declareInjectable<api::display::I2DOverlay>(m_2DOverlay);
        declareInjectable<api::features::IDescriptorMatcher>(m_matcher);
        declareInjectable<api::solver::pose::I2D3DCorrespondencesFinder>(m_corr2D3DFinder);
        declareInjectable<api::solver::pose::I3DTransformSACFinderFrom2D3D>(m_pnpRansac);
        declareInjectable<api::features::IMatchesFilter>(m_matchesFilter);
        declareInjectable<api::image::IImageConvertor>(m_imageConvertorUnity);
        declareInjectable<api::sink::ISinkPoseImage>(m_sink);
        declareInjectable<api::source::ISourceImage>(m_source);

        LOG_DEBUG("All component injections declared");

        LOG_DEBUG("Initialize instance attributes");
        m_minNbInliers = 0;
        m_initOK = false;
    }
    catch (xpcf::Exception & e) {
        LOG_ERROR("The following exception has been caught {}", e.what());
    }

}

void SolARRelocalizationPipeline::onInjected() {

    LOG_DEBUG("SolARRelocalizationPipeline::onInjected");

    // Get properties
    m_minNbInliers = m_pnpRansac->bindTo<xpcf::IConfigurable>()->getProperty("minNbInliers")->getIntegerValue();

    LOG_DEBUG("minNbInliers = {}", m_minNbInliers);
}

SolARRelocalizationPipeline::~SolARRelocalizationPipeline()
{
    LOG_DEBUG("SolARRelocalizationPipeline destructor")
}

FrameworkReturnCode SolARRelocalizationPipeline::init()
{
    LOG_DEBUG("PipelineMappingMultiProcessing init");

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationPipeline::setCameraParameters(const CameraParameters & cameraParams) {

    LOG_DEBUG("SolARRelocalizationPipeline::setCameraParameters");

    m_calibration = cameraParams.intrinsic;
    m_distortion = cameraParams.distortion;

    m_pnpRansac->setCameraParameters(m_calibration, m_distortion);

    LOG_DEBUG("Camera intrinsic / distortion = {} / {}", m_calibration, m_distortion);

    m_initOK = true;

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationPipeline::getCameraParameters(CameraParameters & cameraParams) const {

    LOG_DEBUG("SolARRelocalizationPipeline::getCameraParameters");

    cameraParams.intrinsic = m_calibration;
    cameraParams.distortion = m_distortion;

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationPipeline::start() {

    LOG_DEBUG("SolARRelocalizationPipeline::start");

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationPipeline::stop()
{
    LOG_DEBUG("SolARRelocalizationPipeline::stop");

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationPipeline::relocalizeProcessRequest(const SRef<SolAR::datastructure::Image> image,
                                                                          SolAR::datastructure::Transform3Df& pose, float_t & confidence) {

    LOG_DEBUG("SolARRelocalizationPipeline::relocalizeProcessRequest");

    confidence = 0;

    if (m_initOK) {

        LOG_DEBUG("=> Detection");

        std::vector<Keypoint> keypoints;
        m_keypointsDetector->detect(image, keypoints);
        SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, nullptr, image, Transform3Df::Identity());

        LOG_DEBUG("=> Extraction");

        SRef<DescriptorBuffer>	descriptors;
        m_descriptorExtractor->extract(frame->getView(), frame->getKeypoints(), descriptors);
        frame->setDescriptors(descriptors);

        LOG_DEBUG("=> Localization");

        // keyframes retrieval
        std::vector <uint32_t> retKeyframesId;
        if (m_kfRetriever->retrieve(frame, retKeyframesId) == FrameworkReturnCode::_SUCCESS) {
            LOG_DEBUG("Number of retrieved keyframes: {}", retKeyframesId.size());
            std::vector <uint32_t> processKeyframesId;
            if (retKeyframesId.size() <= NB_PROCESS_KEYFRAMES)
                processKeyframesId.swap(retKeyframesId);
            else
                processKeyframesId.insert(processKeyframesId.begin(), retKeyframesId.begin(), retKeyframesId.begin() + NB_PROCESS_KEYFRAMES);
            std::map<uint32_t, SRef<CloudPoint>> allCorres2D3D;
            for (const auto& it : processKeyframesId) {
                SRef<Keyframe> retKeyframe;
                m_keyframesManager->getKeyframe(it, retKeyframe);
                std::vector < std::pair<uint32_t, SRef<CloudPoint>>> corres2D3D;
                bool isFound = fnFind2D3DCorrespondences(frame, retKeyframe, corres2D3D);
                if (isFound) {
                    for (const auto &corr : corres2D3D) {
                        uint32_t idKp = corr.first;
                        if (allCorres2D3D.find(idKp) == allCorres2D3D.end())
                            allCorres2D3D[idKp] = corr.second;
                    }
                }
            }
            LOG_DEBUG("Number of all 2D-3D correspondences: {}", allCorres2D3D.size());
            std::vector<Point2Df> pts2D;
            std::vector<Point3Df> pts3D;
            const std::vector<Keypoint>& keypoints = frame->getKeypoints();
            for (const auto & corr : allCorres2D3D) {
                pts2D.push_back(Point2Df(keypoints[corr.first].getX(), keypoints[corr.first].getY()));
                pts3D.push_back(Point3Df(corr.second->getX(), corr.second->getY(), corr.second->getZ()));
            }
            // pnp ransac
            std::vector<uint32_t> inliers;
            Transform3Df pose;
            if (m_pnpRansac->estimate(pts2D, pts3D, inliers, pose) == FrameworkReturnCode::_SUCCESS) {
                LOG_DEBUG(" pnp inliers size: {} / {}", inliers.size(), pts3D.size());
                frame->setPose(pose);
                std::vector<Point2Df> pts2DInliers;
                for (const auto& it : inliers)
                    pts2DInliers.push_back(pts2D[it]);
                m_2DOverlay->drawCircles(pts2DInliers, frame->getView());
                m_sink->set(frame->getPose(), frame->getView());
            }
            else
                m_sink->set(frame->getView());
        }

        sink::SinkReturnCode result = m_sink->tryGet(pose);

        if ((result == sink::SinkReturnCode::_NEW_POSE) || (result == sink::SinkReturnCode::_NEW_POSE_AND_IMAGE)) {

            LOG_DEBUG("Got the new pose!");

            return FrameworkReturnCode::_SUCCESS;
        }
        else {
            LOG_DEBUG("Failed to get the new pose");

            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_DEBUG("Camera parameters have not been set!");

        return FrameworkReturnCode::_ERROR_;
    }
}

// Private methods

bool SolARRelocalizationPipeline::fnFind2D3DCorrespondences(const SRef<Frame> &frame, const SRef<Keyframe>& candidateKf, std::vector<std::pair<uint32_t, SRef<CloudPoint>>> &corres2D3D)
{
	// feature matching to reference keyframe			
	std::vector<DescriptorMatch> matches;
	m_matcher->match(candidateKf->getDescriptors(), frame->getDescriptors(), matches);
	m_matchesFilter->filter(matches, matches, candidateKf->getKeypoints(), frame->getKeypoints());
	// find 2D-3D point correspondences
	if (matches.size() < m_minNbInliers)
		return false;
	// find 2D-3D point correspondences
	std::vector<Point2Df> pts2d;
	std::vector<Point3Df> pts3d;
	std::vector<DescriptorMatch> foundMatches;
	std::vector<DescriptorMatch> remainingMatches;
	m_corr2D3DFinder->find(candidateKf, frame, matches, pts3d, pts2d, corres2D3D, foundMatches, remainingMatches);
	LOG_DEBUG("Nb of 2D-3D correspondences: {}", pts2d.size());
	return true;
}

} // namespace RELOCALIZATION
} // namespace PIPELINES
} // namespace SolAR
