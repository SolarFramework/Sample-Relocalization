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
#include "core/Log.h"
#include "xpcf/xpcf.h"
#include "api/pipeline/IRelocalizationPipeline.h"
#include "api/input/devices/ICamera.h"
#include "api/input/devices/IARDevice.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"
#include "api/storage/IPointCloudManager.h"
#include "api/storage/IKeyframesManager.h"

namespace xpcf  = org::bcom::xpcf;

using namespace SolAR;
using namespace SolAR::api;
using namespace datastructure;

// Nb images between 2 pipeline requests
#define NB_IMAGES_BETWEEN_REQUESTS 10

int main(int argc, char *argv[]){

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    try {
        LOG_INFO("Get component manager instance");
        SRef<xpcf::IComponentManager> componentMgr = xpcf::getComponentManagerInstance();

        std::string configxml = std::string("SolARPipelineTest_Relocalization_conf.xml");
		if (argc == 2)
			configxml = std::string(argv[1]);

        xpcf::XPCFErrorCode errorLoad = componentMgr->load(configxml.c_str());
        if (errorLoad != xpcf::_SUCCESS) {
            LOG_ERROR("The file {} has an error", configxml);
            return -1;
        }
        auto gRelocalizationPipeline = componentMgr->resolve<pipeline::IRelocalizationPipeline>();
		auto gCamera = componentMgr->resolve<input::devices::ICamera>();
        auto gArDevice = componentMgr->resolve<input::devices::IARDevice>();
        CameraRigParameters camRigParams = gArDevice->getCameraParameters();
        CameraParameters camParams = camRigParams.cameraParams[0];
		auto gImageViewer = componentMgr->resolve<display::IImageViewer>();
		auto gViewer3D = componentMgr->resolve<display::I3DPointsViewer>();
		auto gPointCloudManager = componentMgr->resolve<storage::IPointCloudManager>();
		auto gKeyframeManager = componentMgr->resolve<storage::IKeyframesManager>();
		// init pipeline
		if (gRelocalizationPipeline->init() != FrameworkReturnCode::_SUCCESS) {
			LOG_ERROR("Cannot init relocalization pipeline");
			return -1;
		}
		// start camera
		if (gCamera->start() != FrameworkReturnCode::_SUCCESS) {
			LOG_ERROR("Cannot start camera");
			return -1;
		}
		gRelocalizationPipeline->setCameraParameters(camParams);
        // start pipeline
		if (gRelocalizationPipeline->start() != FrameworkReturnCode::_SUCCESS) {
			LOG_ERROR("Cannot start relocalization pipeline");
			return -1;
		}

		// display function
        auto fnDisplay = [&gKeyframeManager, &gPointCloudManager, &gViewer3D](const std::vector<Transform3Df>& framePoses) {
            // get all keyframes and point cloud
            std::vector<Transform3Df>   keyframePoses;
            std::vector<SRef<Keyframe>> allKeyframes;
            gKeyframeManager->getAllKeyframes(allKeyframes);
            for (auto const &it : allKeyframes)
                keyframePoses.push_back(it->getPose());
            std::vector<SRef<CloudPoint>> pointCloud;
            gPointCloudManager->getAllPoints(pointCloud);
            // display point cloud
            if (framePoses.size() == 0
                || gViewer3D->display(pointCloud, framePoses.back(), framePoses, {}, {}, keyframePoses) == FrameworkReturnCode::_STOP)
                return false;
            else
                return true;
        };

		// relocalize
        unsigned int nb_images = NB_IMAGES_BETWEEN_REQUESTS;
		SRef<Image> image;
		std::vector<Transform3Df> framePoses;
        while (gCamera->getNextImage(image) == FrameworkReturnCode::_SUCCESS) {	
			if (nb_images != NB_IMAGES_BETWEEN_REQUESTS) {
				nb_images++;
				continue;
			}				
			else
				nb_images = 0;            			
			float_t confidence = 0;
			Transform3Df pose;
            if (gRelocalizationPipeline->relocalizeProcessRequest(image, pose, confidence) == FrameworkReturnCode::_SUCCESS) {
                LOG_DEBUG("Relocalization succeeds");
                framePoses.push_back(pose);
            }
            else
				LOG_DEBUG("Relocalization fails");
            if (gImageViewer->display(image) == SolAR::FrameworkReturnCode::_STOP) break;
            fnDisplay(framePoses);
        }         
		// display all relocalization camera poses
		while (true) {
            if (!fnDisplay(framePoses))
				break;
		}
    }
    catch (xpcf::InjectableNotFoundException e)
    {
        LOG_ERROR ("The following exception in relation to a unfound injectable has been catched: {}", e.what());
        return -1;
    }
    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catched: {}", e.what());
        return -1;
    }

    return 0;
}





