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
#include <signal.h>

#include "api/pipeline/IRelocalizationPipeline.h"
#include "api/input/devices/IARDevice.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"
#include "api/storage/IPointCloudManager.h"

namespace xpcf  = org::bcom::xpcf;

using namespace SolAR;
using namespace SolAR::api;
using namespace datastructure;

// Nb images between 2 pipeline requests
#define NB_IMAGES_BETWEEN_REQUESTS 10

#define INDEX_USE_CAMERA 0

// Global Mapping Pipeline Multithreads instance
SRef<pipeline::IRelocalizationPipeline> gRelocalizationPipeline = 0;

// Function called when interruption signal is triggered
static void SigInt(int signo) {

    LOG_INFO("\n\n===> Program interruption\n");

    LOG_INFO("Stop relocalization pipeline process");

    if (gRelocalizationPipeline != 0)
        gRelocalizationPipeline->stop();

    LOG_INFO("End of test");

    exit(0);
}

int main(int argc, char *argv[]){

#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    // Signal interruption function (Ctrl + C)
    signal(SIGINT, SigInt);

    try {
        LOG_INFO("Get component manager instance");
        SRef<xpcf::IComponentManager> componentMgr = xpcf::getComponentManagerInstance();

        std::string configxml = std::string("SolARPipelineTest_Relocalization_conf.xml");
		if (argc == 2)
			configxml = std::string(argv[1]);

        LOG_INFO("Load Client Remote Mapping Pipeline configuration file: {}", configxml);
        xpcf::XPCFErrorCode errorLoad = componentMgr->load(configxml.c_str());

        if (errorLoad != xpcf::_SUCCESS)
        {
            LOG_ERROR("The file {} has an error", configxml);
            return -1;
        }

        LOG_INFO("Resolve IRelocalizationPipeline interface");
        gRelocalizationPipeline = componentMgr->resolve<pipeline::IRelocalizationPipeline>();

        if (gRelocalizationPipeline->init() == FrameworkReturnCode::_SUCCESS )
        {
            LOG_INFO("Resolve components used");
            auto arDevice = componentMgr->resolve<input::devices::IARDevice>();
            auto imageViewerResult = componentMgr->resolve<display::IImageViewer>();
            auto viewer3D = componentMgr->resolve<display::I3DPointsViewer>();
            auto pointCloudManager = componentMgr->resolve<storage::IPointCloudManager>();

            // Connect remotely to the HoloLens streaming app
            if (arDevice->start() == FrameworkReturnCode::_SUCCESS) {

                LOG_INFO("Set relocalization pipeline camera parameters");

                // Load camera intrinsics parameters
                CameraParameters camParams;
                camParams = arDevice->getParameters(0);

                if (gRelocalizationPipeline->setCameraParameters(camParams) == FrameworkReturnCode::_SUCCESS) {

                    LOG_INFO("Start relocalization pipeline process");

                    std::vector<Transform3Df> framePoses;
                    std::vector<SRef<CloudPoint>> pointCloud;

                    if (gRelocalizationPipeline->start() == FrameworkReturnCode::_SUCCESS) {

                        LOG_INFO("\n\n***** Control+C to stop *****\n");

                        // get point cloud to display
                        pointCloudManager->getAllPoints(pointCloud);

                        unsigned int nb_images = 0;

                        // Wait for interruption
                        while (true) {

                            std::vector<SRef<Image>> images;
                            std::vector<Transform3Df> poses;
                            std::chrono::system_clock::time_point timestamp;

                            // Get data from hololens files
                            if (arDevice->getData(images, poses, timestamp) == FrameworkReturnCode::_SUCCESS) {

                                SRef<Image> image = images[INDEX_USE_CAMERA];
                                Transform3Df pose = Transform3Df::Identity();;
                                float_t confidence = 0;

                                if (imageViewerResult->display(image) == SolAR::FrameworkReturnCode::_STOP) {
                                    LOG_INFO("Cannot display image");
                                    return -1;
                                }

                                if (nb_images == NB_IMAGES_BETWEEN_REQUESTS) {
                                    nb_images = 0;

                                    LOG_INFO("Send an image to relocalization pipeline");

                                    if (gRelocalizationPipeline->relocalizeProcessRequest(image, pose, confidence) == FrameworkReturnCode::_SUCCESS) {
                                        LOG_INFO("New pose calculated by relocalization pipeline");
                                        framePoses.push_back(pose);
                                    }
                                    else {
                                        LOG_INFO("Failed to calculate pose for this image");
                                    }

                                    if (viewer3D->display(pointCloud, pose, {}, framePoses) == FrameworkReturnCode::_STOP){
                                        LOG_INFO("Cannot display result");
                                        return -1;
                                    }
                                }

                                nb_images ++;
                            }
                            else {
                                LOG_INFO("No more images to send");
                            }
                        }
                    }
                    else {
                        LOG_INFO("Cannot start relocalization pipeline");
                        return -1;
                    }
                }
                else {
                    LOG_INFO("Cannot set camera parameters");
                    return -1;
                }
            }
            else {
                LOG_INFO("Cannot start AR device loader");
                return -1;
            }
        }
        else {
            LOG_INFO("Cannot init relocalization pipeline");
            return -1;
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

/*
            // Set camera parameters
            CameraParameters camParam = pipeline->getCameraParameters();
            overlay3DComponent->setCameraParameters(camParam.intrinsic, camParam.distortion);

            unsigned char* r_imageData=new unsigned char[camParam.resolution.width * camParam.resolution.height * 3];
            SRef<Image> camImage=xpcf::utils::make_shared<Image>(r_imageData,camParam.resolution.width, camParam.resolution.height, Image::LAYOUT_BGR, Image::INTERLEAVED, Image::TYPE_8U);

            Transform3Df pose;
			std::vector<Transform3Df> framePoses;
			std::vector<SRef<CloudPoint>> pointCloud;
            clock_t start, end;
            start = clock();
            int count = 0;
            if (pipeline->start(camImage->data()) == FrameworkReturnCode::_SUCCESS)
            {
				// get point cloud to display				
				pointCloudManager->getAllPoints(pointCloud);
				
                while (true)
                {
					pose = Transform3Df::Identity();
                    sink::SinkReturnCode returnCode = pipeline->update(pose);
                    if (returnCode == sink::SinkReturnCode::_ERROR) {
						pipeline->stop();
						break;
					}
                    if (returnCode == sink::SinkReturnCode::_NOTHING)
                        continue;
                    if ((returnCode == sink::SinkReturnCode::_NEW_POSE) || (returnCode == sink::SinkReturnCode::_NEW_POSE_AND_IMAGE))
                    {
                        //LOG_INFO("pose.matrix():\n {} \n",pose.matrix())
                        overlay3DComponent->draw(pose, camImage);
						framePoses.push_back(pose);
                    }

                    if ((imageViewerResult->display(camImage) == SolAR::FrameworkReturnCode::_STOP) || 
						(viewer3D->display(pointCloud, pose, {}, framePoses) == FrameworkReturnCode::_STOP)){
                        pipeline->stop();
                        break;
                    }

                    count++;
                 }
            }
            end = clock();
            double duration = double(end - start) / CLOCKS_PER_SEC;
            printf("\n\nElasped time is %.2lf seconds.\n", duration);
            printf("Number of processed frame per second : %8.2f\n", count / duration);

			// display all relocalization camera poses
			while (true) {
				if (viewer3D->display(pointCloud, Transform3Df::Identity(), framePoses) == FrameworkReturnCode::_STOP)
					break;
			}

            delete[] r_imageData;
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
*/
}





