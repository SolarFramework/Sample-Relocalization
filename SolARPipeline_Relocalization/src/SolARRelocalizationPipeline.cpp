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
#define THRES_NB_RELOC_FAILS 5

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
        declareInjectable<api::solver::pose::I2D3DCorrespondencesFinder>(m_corr2D3DFinder);
        declareInjectable<api::solver::pose::I3DTransformSACFinderFrom2D3D>(m_pnpRansac);
        declareInjectable<api::features::IMatchesFilter>(m_matchesFilter);
		declareInjectable<api::geom::IUndistortPoints>(m_undistortKeypoints);

        LOG_DEBUG("All component injections declared");

        LOG_DEBUG("Initialize instance attributes");
        m_minNbInliers = 0;
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

    if (m_initOK) {
        LOG_WARNING("Pipeline has already been initialized");
        return FrameworkReturnCode::_SUCCESS;
    }

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

    m_calibration = cameraParams.intrinsic;
    m_distortion = cameraParams.distortion;
    m_pnpRansac->setCameraParameters(m_calibration, m_distortion);
	m_undistortKeypoints->setCameraParameters(m_calibration, m_distortion);
    LOG_DEBUG("Camera intrinsic / distortion = {} / {}", m_calibration, m_distortion);

    LOG_DEBUG("Set camera parameters for the map update service");

	if (m_mapUpdatePipeline != nullptr) {
		try {
			if (m_mapUpdatePipeline->setCameraParameters(cameraParams) != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Error while setting camera parameters for the map update service");
				return FrameworkReturnCode::_ERROR_;
			}
		}
		catch (const std::exception &e) {
			LOG_ERROR("Exception raised during remote request to the map update service: {}", e.what());
			return FrameworkReturnCode::_ERROR_;
		}
	}

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

    if (!m_cameraOK){
        LOG_ERROR("Camera parameters have not been set");
        return FrameworkReturnCode::_ERROR_;
    }

    if (!m_started) {     
		if (m_mapUpdatePipeline) {
			LOG_DEBUG("Start remote map update pipeline");
			if (m_mapUpdatePipeline->start() != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Cannot start Map Update pipeline");
				return FrameworkReturnCode::_ERROR_;
			}			
		}
		else {
			LOG_DEBUG("Load initial map from local file");
			// Load map from file
			if (m_mapManager->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
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
		if (m_mapUpdatePipeline) {
			LOG_DEBUG("Stop remote map update pipeline");
			if (m_mapUpdatePipeline->stop() != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Cannot stop Map Update pipeline");
			}
		}

        m_started = false;
    }
    else {
        LOG_INFO("Pipeline already stopped");
    }

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationPipeline::relocalizeProcessRequest(const SRef<SolAR::datastructure::Image> image,
                                                                          SolAR::datastructure::Transform3Df& pose, float_t & confidence) 
{
    LOG_DEBUG("SolARRelocalizationPipeline::relocalizeProcessRequest");
    confidence = 0;
    if (m_started) {

        LOG_DEBUG("=> Detection and extraction");

		std::vector<Keypoint> keypoints, undistortedKeypoints;
		SRef<DescriptorBuffer> descriptors;
		if (m_descriptorExtractor->extract(image, keypoints, descriptors) != FrameworkReturnCode::_SUCCESS)
			return FrameworkReturnCode::_ERROR_;
		m_undistortKeypoints->undistort(keypoints, undistortedKeypoints);
		SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, undistortedKeypoints, descriptors, image, Transform3Df::Identity());

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
				m_isMap = true;
				m_nbRelocFails = 0;
                LOG_DEBUG("Got the new pose: relocalization successful");
                return FrameworkReturnCode::_SUCCESS;
            }
            else {
				m_nbRelocFails++;
				if (m_mapUpdatePipeline && (m_nbRelocFails >= THRES_NB_RELOC_FAILS))
					m_isMap = false;
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
        if (!m_cameraOK){
            LOG_ERROR("Camera parameters have not been set");
            return FrameworkReturnCode::_ERROR_;
        }
        if (!m_started){
            LOG_ERROR("Pipeline has not been started");
            return FrameworkReturnCode::_ERROR_;
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

bool SolARRelocalizationPipeline::fnFind2D3DCorrespondences(const SRef<Frame> &frame, const SRef<Keyframe>& candidateKf, std::vector<std::pair<uint32_t, SRef<CloudPoint>>> &corres2D3D)
{
	// feature matching to reference keyframe			
	std::vector<DescriptorMatch> matches;
	m_matcher->match(candidateKf->getDescriptors(), frame->getDescriptors(), matches);
	m_matchesFilter->filter(matches, matches, candidateKf->getUndistortedKeypoints(), frame->getUndistortedKeypoints());
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
