/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
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

#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"
#include "core/Log.h"
#include "api/input/devices/ICamera.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DOverlay.h"
#include "api/display/I2DOverlay.h"
#include "api/display/I3DPointsViewer.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/pose/I3DTransformSACFinderFrom2D3D.h"
#include "api/storage/IMapManager.h"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;
namespace xpcf  = org::bcom::xpcf;

#define NB_PROCESS_KEYFRAMES 5

int main(int argc, char *argv[])
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif
    LOG_ADD_LOG_TO_CONSOLE();
    try {
        /* instantiate component manager */
        /* this is needed in dynamic mode */
        SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

		/* Load configuration file */
        std::string configxml = std::string("SolARSample_Relocalization_Mono_conf.xml");
		if (argc == 2)
			configxml = std::string(argv[1]);

        if(xpcfComponentManager->load(configxml.c_str())!=org::bcom::xpcf::_SUCCESS)
        {
			LOG_ERROR("Failed to load the configuration file {}", configxml.c_str());
            return -1;
        }

        /* Declare and create components */
        LOG_INFO("Start creating components");
		auto camera = xpcfComponentManager->resolve<input::devices::ICamera>();
		auto imageViewer = xpcfComponentManager->resolve<display::IImageViewer>();
		auto overlay3D = xpcfComponentManager->resolve<display::I3DOverlay>();
		auto overlay2D = xpcfComponentManager->resolve<display::I2DOverlay>();
		auto viewer3D = xpcfComponentManager->resolve<display::I3DPointsViewer>();
		auto mapManager = xpcfComponentManager->resolve<storage::IMapManager>();
		auto keyframeRetriever = xpcfComponentManager->resolve<reloc::IKeyframeRetriever>();
		auto keypointsDetector = xpcfComponentManager->resolve<features::IKeypointDetector>();
		auto descriptorExtractor = xpcfComponentManager->resolve<features::IDescriptorsExtractor>();
		auto matcher = xpcfComponentManager->resolve<features::IDescriptorMatcher>();
		auto corr2D3DFinder = xpcfComponentManager->resolve<solver::pose::I2D3DCorrespondencesFinder>();
		auto pnpRansac = xpcfComponentManager->resolve<api::solver::pose::I3DTransformSACFinderFrom2D3D>();
		auto matchesFilter = xpcfComponentManager->resolve<features::IMatchesFilter>();
		LOG_INFO("Components created!");

		/* Start camera capture */
		if (camera->start() == FrameworkReturnCode::_ERROR_)
		{
			LOG_ERROR("Cannot start loader");
			return -1;
		}
		LOG_INFO("Started!");

		/* Get and set camera intrinsics parameters for components */
		CameraParameters camParams;
		camParams = camera->getParameters();
		overlay3D->setCameraParameters(camParams.intrinsic, camParams.distortion);
		pnpRansac->setCameraParameters(camParams.intrinsic, camParams.distortion);
		LOG_DEBUG("Loaded intrinsics \n{}\n\n{}", camParams.intrinsic, camParams.distortion);

		/* Get min number of inliers to valid a pose by pnp ransac */
		int minNbInliers = pnpRansac->bindTo<xpcf::IConfigurable>()->getProperty("minNbInliers")->getIntegerValue();

		/* Load map from file */
		if (mapManager->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
			LOG_INFO("Load map done!");
		}
		else {
			LOG_ERROR("Cannot load map");
			return 1;
		}		

		// get map
		SRef<Map> map;
		mapManager->getMap(map);					

		// get point cloud
		std::vector<SRef<CloudPoint>> pointCloud;
		map->getConstPointCloud()->getAllPoints(pointCloud);

		// get keyframe
		std::vector<SRef<Keyframe>> allKeyframes;
		std::vector<Transform3Df> allKeyframePoses;
		const SRef<KeyframeCollection>& keyframeCollection = map->getConstKeyframeCollection();
		keyframeCollection->getAllKeyframes(allKeyframes);
		for (const auto & kf : allKeyframes)
			allKeyframePoses.push_back(kf->getPose());

		// get keyframe retrieval
		keyframeRetriever->setKeyframeRetrieval(map->getConstKeyframeRetrieval());

		LOG_INFO("Number of initial point cloud: {}", pointCloud.size());
		LOG_INFO("Number of initial keyframes: {}", allKeyframes.size());

		/* 2D-3D correspondences finder function */
		auto fnFind2D3DCorrespondences = [&matcher, &matchesFilter, &corr2D3DFinder, &pnpRansac, &minNbInliers](const SRef<Frame> &frame, const SRef<Keyframe>& candidateKf, std::vector<std::pair<uint32_t, SRef<CloudPoint>>> &corres2D3D) {
			// feature matching to reference keyframe			
			std::vector<DescriptorMatch> matches;
			matcher->match(candidateKf->getDescriptors(), frame->getDescriptors(), matches);
			matchesFilter->filter(matches, matches, candidateKf->getKeypoints(), frame->getKeypoints());
			if (matches.size() < minNbInliers)
				return false;
			// find 2D-3D point correspondences
			std::vector<Point2Df> pts2d;
			std::vector<Point3Df> pts3d;
			std::vector<DescriptorMatch> foundMatches;
			std::vector<DescriptorMatch> remainingMatches;
			corr2D3DFinder->find(candidateKf, frame, matches, pts3d, pts2d, corres2D3D, foundMatches, remainingMatches);
			LOG_DEBUG("Nb of 2D-3D correspondences: {}", pts2d.size());
			return true;
		};

        /* Relocalization for each image */
		std::vector<Transform3Df> framePoses;
		int nbProcessFrame(0);
		clock_t start = clock();
		while (true)
		{
			// get image
			SRef<Image> image;
			if (camera->getNextImage(image) != FrameworkReturnCode::_SUCCESS) {
				LOG_ERROR("Error during capture");
				break;
			}
			// feature extraction image
			std::vector<Keypoint> keypoints;
			keypointsDetector->detect(image, keypoints);
			SRef<DescriptorBuffer> descriptors;
			descriptorExtractor->extract(image, keypoints, descriptors);
			SRef<Frame> frame = xpcf::utils::make_shared<Frame>(keypoints, descriptors, image, Transform3Df::Identity());
			// Relocalization
			std::vector <uint32_t> retKeyframesId;
			std::vector<Transform3Df> bestRetKeyframePoses;
			if (keyframeRetriever->retrieve(frame, retKeyframesId) == FrameworkReturnCode::_SUCCESS) {				
				std::vector <uint32_t> processKeyframesId;
				if (retKeyframesId.size() <= NB_PROCESS_KEYFRAMES)
					processKeyframesId.swap(retKeyframesId);
				else
					processKeyframesId.insert(processKeyframesId.begin(), retKeyframesId.begin(), retKeyframesId.begin() + NB_PROCESS_KEYFRAMES);
				LOG_DEBUG("Number of retrieved keyframes: {}", processKeyframesId.size());
				std::map<uint32_t, SRef<CloudPoint>> allCorres2D3D;
				for (const auto& it : processKeyframesId) {
					SRef<Keyframe> retKeyframe;
					keyframeCollection->getKeyframe(it, retKeyframe);
					std::vector < std::pair<uint32_t, SRef<CloudPoint>>> corres2D3D;
					bool isFound = fnFind2D3DCorrespondences(frame, retKeyframe, corres2D3D);
					if (isFound) {
						bestRetKeyframePoses.push_back(retKeyframe->getPose());
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
				for (const auto & corr : allCorres2D3D) {
					pts2D.push_back(Point2Df(keypoints[corr.first].getX(), keypoints[corr.first].getY()));
					pts3D.push_back(Point3Df(corr.second->getX(), corr.second->getY(), corr.second->getZ()));
				}
				// pnp ransac
				std::vector<uint32_t> inliers;
				Transform3Df pose;
				if (pnpRansac->estimate(pts2D, pts3D, inliers, pose) == FrameworkReturnCode::_SUCCESS) {
					LOG_DEBUG(" pnp inliers size: {} / {}", inliers.size(), pts3D.size());
					frame->setPose(pose);
					framePoses.push_back(pose);
					std::vector<Point2Df> pts2DInliers;
					for (const auto& it : inliers)
						pts2DInliers.push_back(pts2D[it]);
					overlay2D->drawCircles(pts2DInliers, frame->getView());
					overlay3D->draw(pose, frame->getView());					
				}
			}
			// display image
			if (imageViewer->display(image) == SolAR::FrameworkReturnCode::_STOP)
				break;
			// display point cloud and poses
			if (viewer3D->display(pointCloud, frame->getPose(), bestRetKeyframePoses, framePoses, {}, allKeyframePoses) == FrameworkReturnCode::_STOP)
				break;
			nbProcessFrame++;
        }

		// display stats on frame rate
		clock_t end = clock();
		double duration = double(end - start) / CLOCKS_PER_SEC;
		printf("\n\nElasped time is %.2lf seconds.\n", duration);
		printf("Number of processed frame per second : %8.2f\n", nbProcessFrame / duration);

		// display all relocalization camera poses
		while (true) {						 
			if (viewer3D->display(pointCloud, Transform3Df::Identity(), framePoses, {}, {}, allKeyframePoses) == FrameworkReturnCode::_STOP)
				break;
		}
    }

    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catch : {}", e.what());
        return -1;
    }
    return 0;
}
