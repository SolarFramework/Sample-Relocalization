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
    try {
        declareInterface<api::pipeline::IRelocalizationPipeline>(this);
        LOG_DEBUG("Components injection declaration");        
        declareInjectable<storage::IMapManager>(m_mapManager);
        declareInjectable<api::pipeline::IMapUpdatePipeline>(m_mapUpdatePipeline, true);
        declareInjectable<api::reloc::IKeyframeRetriever>(m_kfRetriever);
        declareInjectable<api::features::IKeypointDetector>(m_keypointsDetector);
        declareInjectable<api::features::IDescriptorsExtractor>(m_descriptorExtractor);
        declareInjectable<api::features::IDescriptorMatcher>(m_matcher);
        declareInjectable<api::solver::pose::I2D3DCorrespondencesFinder>(m_corr2D3DFinder);
        declareInjectable<api::solver::pose::I3DTransformSACFinderFrom2D3D>(m_pnpRansac);
        declareInjectable<api::features::IMatchesFilter>(m_matchesFilter);

        LOG_DEBUG("All component injections declared");

        LOG_DEBUG("Initialize instance attributes");
        m_minNbInliers = 0;
        m_initOK = false;
        m_cameraOK = false;
    }
    catch (xpcf::Exception & e) {
        LOG_ERROR("The following exception has been caught {}", e.what());
    }
	LOG_DEBUG(" SolARRelocalizationPipeline constructor");
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
    LOG_DEBUG("SolARRelocalizationPipeline::init");

    m_initOK = true;

    // Test if map must be requested to a remote map update pipeline (primarily)
    // or loaded from file

    if (m_mapUpdatePipeline != nullptr) {

        LOG_DEBUG("Get initial map from a remote map update pipeline");

        LOG_DEBUG("Map Update pipeline URL = {}",
                 m_mapUpdatePipeline->bindTo<xpcf::IConfigurable>()->getProperty("channelUrl")->getStringValue());

        SRef<Map> map;

        try {
            if (m_mapUpdatePipeline->init() == FrameworkReturnCode::_SUCCESS) {

                if (m_mapUpdatePipeline->getMapRequest(map) == FrameworkReturnCode::_SUCCESS) {
                    if (map != nullptr) {
                        LOG_DEBUG("Map nb points = {}", map->getConstPointCloud()->getNbPoints());
                        m_mapManager->setMap(map);
                        m_keyframeCollection = map->getConstKeyframeCollection();
                        LOG_DEBUG("NB keyframes = {}", m_keyframeCollection->getNbKeyframes());
                        LOG_INFO("Get map from remote map update pipeline");
                    }
                    else {
                        LOG_INFO("Initial map is empty");
                    }
                    return FrameworkReturnCode::_SUCCESS;
                }
                else {
                    LOG_ERROR("Can not get initial map from remote map update pipeline");
                    return FrameworkReturnCode::_ERROR_;
                }
            }
            else {
                LOG_ERROR("Can not initialize remote map update pipeline");
                return FrameworkReturnCode::_ERROR_;
            }

        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised during remote request to map update pipeline: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else if (m_mapManager != nullptr) {

        LOG_DEBUG("Load initial map from local file");

        // Load map from file
        if (m_mapManager->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
            SRef<Map> map;
            m_mapManager->getMap(map);
            LOG_DEBUG("Map nb points = {}", map->getConstPointCloud()->getNbPoints());
            m_keyframeCollection = map->getConstKeyframeCollection();
            LOG_DEBUG("NB keyframes = {}", m_keyframeCollection->getNbKeyframes());
            LOG_INFO("Load map done");
            return FrameworkReturnCode::_SUCCESS;
        }
        else {
            LOG_ERROR("Cannot load map");
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Initial map not defined");
        return FrameworkReturnCode::_ERROR_;
    }
}

FrameworkReturnCode SolARRelocalizationPipeline::setCameraParameters(const CameraParameters & cameraParams) 
{
	LOG_DEBUG("SolARRelocalizationPipeline::setCameraParameters");
    m_calibration = cameraParams.intrinsic;
    m_distortion = cameraParams.distortion;
    m_pnpRansac->setCameraParameters(m_calibration, m_distortion);
    LOG_DEBUG("Camera intrinsic / distortion = {} / {}", m_calibration, m_distortion);
    m_cameraOK = true;
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationPipeline::getCameraParameters(CameraParameters & cameraParams) const 
{
    LOG_DEBUG("SolARRelocalizationPipeline::getCameraParameters");
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

FrameworkReturnCode SolARRelocalizationPipeline::start() 
{
    LOG_DEBUG("SolARRelocalizationPipeline::start");

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

FrameworkReturnCode SolARRelocalizationPipeline::stop()
{
    LOG_DEBUG("SolARRelocalizationPipeline::stop");
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationPipeline::relocalizeProcessRequest(const SRef<SolAR::datastructure::Image> image,
                                                                          SolAR::datastructure::Transform3Df& pose, float_t & confidence) 
{
    LOG_DEBUG("SolARRelocalizationPipeline::relocalizeProcessRequest");
    confidence = 0;
    if ((m_initOK) && (m_cameraOK)) {

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
                m_keyframeCollection->getKeyframe(it, retKeyframe);
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
            if (m_pnpRansac->estimate(pts2D, pts3D, inliers, pose) == FrameworkReturnCode::_SUCCESS) {
                LOG_DEBUG(" pnp inliers size: {} / {}", inliers.size(), pts3D.size());
                frame->setPose(pose);
                std::vector<Point2Df> pts2DInliers;
                for (const auto& it : inliers)
                    pts2DInliers.push_back(pts2D[it]);

                LOG_DEBUG("Got the new pose");
                return FrameworkReturnCode::_SUCCESS;
            }
            else {
                LOG_DEBUG("Failed to get the new pose");
                return FrameworkReturnCode::_ERROR_;
            }
        }
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
