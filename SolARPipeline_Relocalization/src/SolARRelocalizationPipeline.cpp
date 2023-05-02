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
#include "core/Timer.h"
#include "boost/log/core/core.hpp"
#include <cmath>
#include <unordered_map>

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
    using namespace datastructure;
namespace PIPELINES {
namespace RELOCALIZATION {

#define NB_PROCESS_KEYFRAMES 5
#define THRES_NB_RELOC_FAILS 5
// extend list of 2D/3D correspondences from local map's unmatched 3D cloud points 
#define REGION_MATCH_FROM_LOCAL_MAP

// Public methods

SolARRelocalizationPipeline::SolARRelocalizationPipeline():ConfigurableBase(xpcf::toUUID<SolARRelocalizationPipeline>())
{    
    try {
        declareInterface<api::pipeline::IRelocalizationPipeline>(this);
        LOG_DEBUG("Components injection declaration");        
        declareInjectable<storage::IMapManager>(m_mapManager);
        declareInjectable<api::pipeline::IMapUpdatePipeline>(m_mapUpdatePipeline, true);
        declareInjectable<api::reloc::IKeyframeRetriever>(m_kfRetriever);
        declareInjectable<api::features::IDescriptorsExtractorFromImage>(m_descriptorExtractor);
        declareInjectable<api::features::IDescriptorMatcher>(m_matcher);
        declareInjectable<api::features::IDescriptorMatcherRegion>(m_matcherRegion);
        declareInjectable<api::solver::pose::I2D3DCorrespondencesFinder>(m_corr2D3DFinder);
        declareInjectable<api::solver::pose::I3DTransformSACFinderFrom2D3D>(m_pnpRansac);
        declareInjectable<api::solver::pose::I2DTransformFinder>(m_findHomography);
        declareInjectable<api::geom::IProject>(m_projector);
        declareInjectable<api::features::IMatchesFilter>(m_matchesFilter);
		declareInjectable<api::geom::IUndistortPoints>(m_undistortKeypoints);
        declareInjectable<api::storage::ICameraParametersManager>(m_cameraParametersManager);

        LOG_DEBUG("All component injections declared");

        LOG_DEBUG("Initialize instance attributes");
        m_minNbInliers = 0;
        m_confidenceSigma = 0.f;
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

    m_confidenceSigma = 2.f * static_cast<float>(m_minNbInliers);

    if (m_started)
        stop();

    if (m_mapUpdatePipeline != nullptr) {

        LOG_DEBUG("Map Update pipeline URL = {}",
                 m_mapUpdatePipeline->bindTo<xpcf::IConfigurable>()->getProperty("channelUrl")->getStringValue());

        try {
            if (m_mapUpdatePipeline->init() != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Can not initialize remote map update pipeline");
				return FrameworkReturnCode::_ERROR_;
            }            
        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised during remote request to map update pipeline: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Map Update pipeline not defined");
    }

    if (m_initOK) {
        LOG_WARNING("Pipeline has already been initialized");
        return FrameworkReturnCode::_SUCCESS;
    }

    m_initOK = true;

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationPipeline::setCameraParameters(const CameraParameters & cameraParams) 
{
	LOG_DEBUG("SolARRelocalizationPipeline::setCameraParameters");

    if (!m_initOK) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }
    m_camParams = cameraParams;

    // add current camera parameters to the map manager
    SRef<CameraParameters> camParams = xpcf::utils::make_shared<CameraParameters>(m_camParams);
    m_mapManager->addCameraParameters(camParams);
    m_camParamsID = camParams->id;

    LOG_DEBUG("Camera intrinsic / distortion:\n{}\n{}", m_camParams.intrinsic, m_camParams.distortion);
    LOG_DEBUG("Set camera parameters for the map update service");
    m_cameraOK = true;
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationPipeline::getCameraParameters(CameraParameters & cameraParams) const 
{
    LOG_DEBUG("SolARRelocalizationPipeline::getCameraParameters");

    if (!m_initOK) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

    if (m_cameraOK) {
        cameraParams = m_camParams;
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

    if (!m_cameraOK){
        LOG_ERROR("Camera parameters have not been set");
        return FrameworkReturnCode::_ERROR_;
    }

    if (!m_started) {     
        if (!m_mapUpdatePipeline) {
			LOG_DEBUG("Load initial map from local file");
			// Load map from file
			if (m_mapManager->loadFromFile() == FrameworkReturnCode::_SUCCESS) {

                // add current camera parameters to the map manager
                SRef<CameraParameters> camParams = xpcf::utils::make_shared<CameraParameters>(m_camParams);
                m_mapManager->addCameraParameters(camParams);
                m_camParamsID = camParams->id;

                SRef<Map> map;
				m_mapManager->getMap(map);
				LOG_DEBUG("Map nb points = {}", map->getConstPointCloud()->getNbPoints());
				m_keyframeCollection = map->getConstKeyframeCollection();
				LOG_DEBUG("NB keyframes = {}", m_keyframeCollection->getNbKeyframes());
				LOG_DEBUG("Load map done");
				m_isMap = true;
			}
			else {
				LOG_ERROR("Cannot load map from local file");
				return FrameworkReturnCode::_ERROR_;
			}
		}
        else {
            // Force sub map request to map update
            m_isMap = false;
        }
        m_started = true;        
    }
    else {
        LOG_INFO("Pipeline already started");
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationPipeline::stop()
{
    LOG_DEBUG("SolARRelocalizationPipeline::stop");

    if (!m_initOK) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

    if (!m_cameraOK){
        LOG_ERROR("Camera parameters have not been set");
        return FrameworkReturnCode::_ERROR_;
    }

    if (m_started) {

        m_started = false;
    }
    else {
        LOG_INFO("Pipeline already stopped");
    }

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationPipeline::relocalizeProcessRequest(const SRef<SolAR::datastructure::Image> image,
                                                                          SolAR::datastructure::Transform3Df& pose,
                                                                          float_t & confidence, const Transform3Df& poseCoarse)
{
    LOG_DEBUG("SolARRelocalizationPipeline::relocalizeProcessRequest");

    confidence = 0.f;

    if (m_started) {

        LOG_DEBUG("=> Detection and extraction");

		std::vector<Keypoint> keypoints, undistortedKeypoints;
		SRef<DescriptorBuffer> descriptors;
		if (m_descriptorExtractor->extract(image, keypoints, descriptors) != FrameworkReturnCode::_SUCCESS)
			return FrameworkReturnCode::_ERROR_;
        m_undistortKeypoints->undistort(keypoints, m_camParams, undistortedKeypoints);
        SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, undistortedKeypoints, descriptors, image, m_camParamsID, poseCoarse);

		if (m_mapUpdatePipeline && !m_isMap) {
			SRef<Map> subMap;

            if (m_mapUpdatePipeline->getSubmapRequest(frame, subMap) == FrameworkReturnCode::_SUCCESS){
				m_mapManager->setMap(subMap);
                m_keyframeCollection = subMap->getConstKeyframeCollection();                                
				LOG_DEBUG("Get submap successfully");
				LOG_DEBUG("Number cloud points of map: {}", subMap->getConstPointCloud()->getNbPoints());
				LOG_DEBUG("Number keyframes of map: {}", m_keyframeCollection->getNbKeyframes());
            }
            else{
                LOG_DEBUG("Cannot get submap");
                return FrameworkReturnCode::_ERROR_;
            }
        }

        // keyframes retrieval
        std::vector<uint32_t> retKeyframesId;
        if (m_kfRetriever->retrieve(frame, retKeyframesId) == FrameworkReturnCode::_SUCCESS) {
            LOG_DEBUG("Number of retrieved keyframes: {}", retKeyframesId.size());
            Timer clock;
            std::vector<uint32_t> processKeyframesId;
            if (retKeyframesId.size() <= NB_PROCESS_KEYFRAMES)
                processKeyframesId.swap(retKeyframesId);
            else
                processKeyframesId.insert(processKeyframesId.begin(), retKeyframesId.begin(), retKeyframesId.begin() + NB_PROCESS_KEYFRAMES);
            std::map<uint32_t, SRef<CloudPoint>> allCorres2D3D;
            std::map<uint32_t, std::vector<uint32_t>> mapKeypointCloudPts;
#ifdef REGION_MATCH_FROM_LOCAL_MAP
            Transform2Df maxKfHomography;
            uint32_t maxKfId = 0;
            std::vector<DescriptorMatch> maxMatchesKf2Frame;
            std::unordered_map<uint32_t, bool> cloudPtIn;  // if cloud point is included in 2D/3D correspondences list 
            float maxMatchingScore = 0.f;
#endif
            for (const auto& it : processKeyframesId) {
                SRef<Keyframe> retKeyframe;
                m_keyframeCollection->getKeyframe(it, retKeyframe);
                std::vector<std::pair<uint32_t, SRef<CloudPoint>>> corres2D3D;
                std::vector<DescriptorMatch> matchesKf2Frame;
                bool isFound = fnFind2D3DCorrespondences(frame, retKeyframe, corres2D3D, matchesKf2Frame);
                if (isFound) {
                    for (const auto &corr : corres2D3D)
                        mapKeypointCloudPts[corr.first].push_back(corr.second->getId());
                }
#ifdef REGION_MATCH_FROM_LOCAL_MAP
                if (static_cast<int>(matchesKf2Frame.size()) >= m_minNbInliers && static_cast<int>(matchesKf2Frame.size()) > static_cast<int>(maxMatchesKf2Frame.size())) {
                    maxKfId = it;
                    maxMatchesKf2Frame = matchesKf2Frame;
                }
                for (const auto& visi : retKeyframe->getVisibility())
                    cloudPtIn[visi.second] = false;
                for (const auto& match : matchesKf2Frame)
                    if (mapKeypointCloudPts.find(match.getIndexInDescriptorB()) != mapKeypointCloudPts.end())
                        maxMatchingScore = std::max<float>(maxMatchingScore, match.getMatchingScore());
#endif
            }

            // generate global 2D/3D correspondences 
            SRef<SolAR::datastructure::Map> map;
            m_mapManager->getMap(map);
            for (const auto& item : mapKeypointCloudPts) {
                // each keypoint could be mapped to several cloud points 
                std::unordered_map<uint32_t, int> frequencyMap;
                for (const auto& cloudPtId : item.second)
                    frequencyMap[cloudPtId]++;
                // find the cloud point with the biggest frequency value 
                uint32_t maxFreqPtId = 0;
                int maxFreq = 0;
                for (const auto& element : frequencyMap) {
                    if (element.second > maxFreq) {
                        maxFreqPtId = element.first;
                        maxFreq = element.second;
                    }
                }
                // add to global 2D/3D correspondences list 
                if ( maxFreq >= 1 ) {
                    SRef<CloudPoint> point;
                    map->getConstPointCloud()->getPoint(maxFreqPtId, point);
                    allCorres2D3D[item.first] = point;
#ifdef REGION_MATCH_FROM_LOCAL_MAP
                    cloudPtIn[maxFreqPtId] = true;
#endif
                }
            }

#ifdef REGION_MATCH_FROM_LOCAL_MAP
            if (!maxMatchesKf2Frame.empty()) {
                Timer clockRegionMatch;
                // estimate homography 
                SRef<Keyframe> retKeyframe;
                m_keyframeCollection->getKeyframe(maxKfId, retKeyframe);
                std::vector<Point2Df> kfPts, framePts;
                for (const auto& match : maxMatchesKf2Frame) {
                    auto kfKp = retKeyframe->getUndistortedKeypoint(match.getIndexInDescriptorA());
                    auto frameKp = frame->getUndistortedKeypoint(match.getIndexInDescriptorB());
                    kfPts.emplace_back(kfKp.getX(), kfKp.getY());
                    framePts.emplace_back(frameKp.getX(), frameKp.getY());
                }

                if (m_findHomography->find(kfPts, framePts, maxKfHomography) == api::solver::pose::Transform2DFinder::TRANSFORM2D_ESTIMATION_OK) {
                    // find local map unseen cloud points
                    std::vector<SRef<CloudPoint>> localMapUnseen;
                    for (const auto& cpStatus : cloudPtIn) {
                        if (!cpStatus.second) {
                            SRef<CloudPoint> point;
                            map->getConstPointCloud()->getPoint(cpStatus.first, point);
                            localMapUnseen.push_back(point);
                        }
                    }

                    // match local map unseen to local regions of frame image 
                    uint32_t imgWidth = frame->getView()->getWidth();
                    uint32_t imgHeight = frame->getView()->getHeight();
                    SRef<CameraParameters> camParams;
                    if (m_cameraParametersManager->getCameraParameters(retKeyframe->getCameraID(), camParams) != FrameworkReturnCode::_SUCCESS) {
                        LOG_WARNING("Camera parameteres with id {} does not exists in the camera parameters manager", retKeyframe->getCameraID());
                        return FrameworkReturnCode::_ERROR_;
                    }

                    std::vector<Point2Df> projected2DPts;
                    m_projector->project(localMapUnseen, retKeyframe->getPose(), *camParams, projected2DPts);

                    // using homography to find pixel in frame 
                    std::vector<Point2Df> projected2DPtsCandidates;
                    std::vector<SRef<CloudPoint>> localMapUnseenCandidates;
                    for (auto i = 0; i < projected2DPts.size(); i++) {
                        auto p = projected2DPts[i];
                        if (p[0] > 0 && p[0] < imgWidth && p[1]>0 && p[1] < imgHeight) {
                            float u = maxKfHomography(0, 0)*p[0] + maxKfHomography(0, 1)*p[1] + maxKfHomography(0, 2);
                            float v = maxKfHomography(1, 0)*p[0] + maxKfHomography(1, 1)*p[1] + maxKfHomography(1, 2);
                            float s = maxKfHomography(2, 0)*p[0] + maxKfHomography(2, 1)*p[1] + maxKfHomography(2, 2);
                            if (s != 0.f) {
                                projected2DPtsCandidates.push_back({ u / s, v / s });
                                localMapUnseenCandidates.push_back(localMapUnseen[i]);
                            }
                        }
                    }

                    std::vector<SRef<DescriptorBuffer>> desAllLocalMapUnseenCandidates;
                    for (const auto& cp : localMapUnseenCandidates)
                        desAllLocalMapUnseenCandidates.push_back(cp->getDescriptor());

                    std::vector<DescriptorMatch> allMatches;
                    m_matcherRegion->match(projected2DPtsCandidates, desAllLocalMapUnseenCandidates, frame, allMatches, 0, maxMatchingScore*0.5f);
                    for (auto &it_match : allMatches) {
                        auto idx_3d = it_match.getIndexInDescriptorA();
                        auto idx_2d = it_match.getIndexInDescriptorB();
                        if (allCorres2D3D.find(idx_2d) == allCorres2D3D.end()) {
                            SRef<CloudPoint> point;
                            map->getConstPointCloud()->getPoint(localMapUnseenCandidates[idx_3d]->getId(), point);
                            allCorres2D3D[idx_2d] = point;
                        }
                    }
                }
                LOG_DEBUG("Region match local map to frame took {} ms", clockRegionMatch.elapsed());
            }
#endif
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
            if (m_pnpRansac->estimate(pts2D, pts3D, m_camParams, inliers, pose) == FrameworkReturnCode::_SUCCESS) {
                LOG_DEBUG(" pnp inliers size: {} / {}", inliers.size(), pts3D.size());

                frame->setPose(pose);
				m_isMap = true;
				m_nbRelocFails = 0;
                // compute confidence score from number of inliers 
                if (inliers.size() <= m_minNbInliers)
                    confidence = 0.f;
                else {
                    // when nb inliers = 2*sigma + m_minNbInliers, confidence is close to 1 
                    confidence = 1.f - std::exp(- (static_cast<float>(inliers.size()) - static_cast<float>(m_minNbInliers)) * (static_cast<float>(inliers.size()) - static_cast<float>(m_minNbInliers)) 
                        / (2. * m_confidenceSigma * m_confidenceSigma));
                    LOG_DEBUG("Confidence score = {}", confidence);
                }
                LOG_DEBUG("Got the new pose: relocalization successful");
                LOG_DEBUG("Reloc took {} ms", clock.elapsed());
                return FrameworkReturnCode::_SUCCESS;
            }
        }
		m_nbRelocFails++;
		if (m_mapUpdatePipeline && (m_nbRelocFails >= THRES_NB_RELOC_FAILS))
			m_isMap = false;
		LOG_DEBUG("Failed to relocalization");
    }
    else {
        if (!m_initOK) {
            LOG_ERROR("Pipeline has not been initialized");
        }
        if (!m_cameraOK){
            LOG_ERROR("Camera parameters have not been set");
        }
        if (!m_started){
            LOG_ERROR("Pipeline has not been started");
        }
    }
    return FrameworkReturnCode::_ERROR_;
}

FrameworkReturnCode SolARRelocalizationPipeline::getMapRequest(SRef<SolAR::datastructure::Map>& map) const
{
	LOG_DEBUG("PipelineRelocalization getMapRequest");
	if (!m_isMap)
		return FrameworkReturnCode::_ERROR_;
	m_mapManager->getMap(map);
	return FrameworkReturnCode::_SUCCESS;
}

// Private methods

bool SolARRelocalizationPipeline::fnFind2D3DCorrespondences(const SRef<Frame> &frame, const SRef<Keyframe>& candidateKf, std::vector<std::pair<uint32_t, SRef<CloudPoint>>> &corres2D3D, std::vector<DescriptorMatch>& matchesKfToFrame)
{
	// feature matching to reference keyframe			
	std::vector<DescriptorMatch> matches;
	m_matcher->match(candidateKf->getDescriptors(), frame->getDescriptors(), matches);
    if (!frame->getPose().isApprox(Transform3Df::Identity()) && !candidateKf->getPose().isApprox(Transform3Df::Identity())) {
        // if both frame and candidateKf have valid pose (coarse pose needing to be refined), we can compute fundamental matrix and perform safer filtering 
        // get camera parameters 
        SRef<CameraParameters> camParamsFrame, camParamsKeyframe;
        if (m_cameraParametersManager->getCameraParameters(frame->getCameraID(), camParamsFrame) != FrameworkReturnCode::_SUCCESS) {
            LOG_ERROR("Camera parameters with id {} does not exists in the camera parameters manager", frame->getCameraID());
            return false;
        }
        if (m_cameraParametersManager->getCameraParameters(candidateKf->getCameraID(), camParamsKeyframe) != FrameworkReturnCode::_SUCCESS) {
            LOG_ERROR("Camera parameters with id {} does not exists in the camera parameters manager", candidateKf->getCameraID());
            return false;
        }
        m_matchesFilter->filter(matches, matches, candidateKf->getUndistortedKeypoints(), frame->getUndistortedKeypoints(),
            candidateKf->getPose(), frame->getPose(), camParamsKeyframe->intrinsic, camParamsFrame->intrinsic);
    }
    else {
        // RANSAC based filtering, could have random behaviors in some cases
        m_matchesFilter->filter(matches, matches, candidateKf->getUndistortedKeypoints(), frame->getUndistortedKeypoints());
    }
    // output keypoint matches 
    matchesKfToFrame = matches;
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
