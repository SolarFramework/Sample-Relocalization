/**
 * @copyright Copyright (c) 2022 B-com http://www.b-com.com/
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

#include <xpcf/xpcf.h>
#include "xpcf/threading/BaseTask.h"
#include <iostream>
#include <boost/log/core.hpp>
#include <boost/thread/thread.hpp>
#include <signal.h>

#include "core/Log.h"
#include "api/pipeline/IAsyncRelocalizationPipeline.h"
#include "api/input/devices/IARDevice.h"
#include "api/display/IImageViewer.h"

using namespace std;
using namespace SolAR;
using namespace SolAR::api;
using namespace SolAR::datastructure;
namespace xpcf=org::bcom::xpcf;

#define INDEX_USE_CAMERA 0

// Global relocalization and mapping front end Pipeline instance
SRef<pipeline::IAsyncRelocalizationPipeline> gRelocalizationAndMappingFrontendPipeline = 0;

// Function called when interruption signal is triggered
static void SigInt(int signo) {

    LOG_INFO("\n\n===> Program interruption\n");

    LOG_INFO("Stop relocalization and mapping front end pipeline process");

    if (gRelocalizationAndMappingFrontendPipeline != 0)
        gRelocalizationAndMappingFrontendPipeline->stop();

    LOG_INFO("End of test");

    exit(0);
}


///
/// \brief Test application for SolARPipeline_MappingAndRelocalizationFrontend
/// only with relocalization service
///

int main(int argc, char ** argv)
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();

    LOG_SET_DEBUG_LEVEL();

    // XPCF Component Manager
    SRef<xpcf::IComponentManager> xpcfComponentManager = 0;

    // Components used by test app
    SRef<input::devices::IARDevice> arDevice = 0;
    SRef<display::IImageViewer> imageViewer = 0;

    // Signal interruption function (Ctrl + C)
    signal(SIGINT, SigInt);

    // Default configuration file
    char * config_file = (char *)"SolARPipelineTest_Frontend_RelocOnly_conf.xml";

    if (argc > 1) {
        // Get pipeline configuration file path and name from main args
        config_file = argv[1];
    }

    try {
        LOG_INFO("Get Component Manager instance");

        xpcfComponentManager = xpcf::getComponentManagerInstance();

        LOG_INFO("Load Pipeline configuration file");

        if (xpcfComponentManager->load(config_file) == org::bcom::xpcf::_SUCCESS)
        {
            // Create Pipeline component
            gRelocalizationAndMappingFrontendPipeline = xpcfComponentManager->resolve<pipeline::IAsyncRelocalizationPipeline>();

            LOG_INFO("Mapping and relocalization front end pipeline component created");
        }
        else {
            LOG_ERROR("Failed to load the configuration file {}", config_file);
            return -1;
        }

        LOG_INFO("Initialize the pipeline in \'relocalization only\' mode");

        if (gRelocalizationAndMappingFrontendPipeline->init(SolAR::api::pipeline::RELOCALIZATION_ONLY)
                != FrameworkReturnCode::_SUCCESS) {
            LOG_ERROR("Error while initializing the mapping and relocalization front end pipeline");
            return -1;
        }

        LOG_INFO("Get camera parameters");

        arDevice = xpcfComponentManager->resolve<input::devices::IARDevice>();
        LOG_INFO("Producer client: AR device component created");

        imageViewer = xpcfComponentManager->resolve<SolAR::api::display::IImageViewer>();
        LOG_INFO("Remote producer client: AR device component created");

        if (arDevice->start() == FrameworkReturnCode::_SUCCESS) {
            // Load camera intrinsics parameters
            CameraRigParameters camRigParams = arDevice->getCameraParameters();
            CameraParameters camParams = camRigParams.cameraParams[INDEX_USE_CAMERA];

            LOG_INFO("Set camera paremeters for the pipeline");

            if (gRelocalizationAndMappingFrontendPipeline->setCameraParameters(camParams) != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while setting camera parameters for the mapping and relocalization front end pipeline");
                return -1;
            }

            LOG_INFO("Start the pipeline");

            if (gRelocalizationAndMappingFrontendPipeline->start() != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while initializing the mapping and relocalization front end pipeline");
                return -1;
            }

            LOG_INFO("Read images and poses from hololens files");
            LOG_INFO("\n\n***** Control+C to stop *****\n");

            // Wait for interruption
            while (true) {
                std::vector<SRef<Image>> images;
                std::vector<Transform3Df> poses;
                std::chrono::system_clock::time_point timestamp;

                // Read next image and pose
                if (arDevice->getData(images, poses, timestamp) == FrameworkReturnCode::_SUCCESS) {

                    SRef<Image> image = images[INDEX_USE_CAMERA];
                    Transform3Df pose = poses[INDEX_USE_CAMERA];
                    api::pipeline::TransformStatus transform3DStatus;
                    Transform3Df transform3D;
                    float_t confidence;
                    api::pipeline::MappingStatus mappingStatus;

                    LOG_INFO("Send image and pose to pipeline");

                    image->setImageEncoding(Image::ENCODING_JPEG);
                    image->setImageEncodingQuality(80);

                    // Send data to mapping and relocalization front end pipeline
                    gRelocalizationAndMappingFrontendPipeline->relocalizeProcessRequest(
                                {image}, {pose}, /* fixedPose */ false,
                                { .0f, .0f, .0f, .0f,
                                  .0f, .0f, .0f, .0f,
                                  .0f, .0f, .0f, .0f,
                                  .0f, .0f, .0f, .0f },
                                timestamp, transform3DStatus, transform3D, confidence, mappingStatus);

                    if (transform3DStatus == api::pipeline::NEW_3DTRANSFORM) {
                        LOG_INFO("New 3D transformation = {}", transform3D.matrix());
                    }
                    else if (transform3DStatus == api::pipeline::PREVIOUS_3DTRANSFORM) {
                        LOG_INFO("Previous 3D transformation = {}", transform3D.matrix());
                    }
                    else {
                        LOG_INFO("No 3D transformation matrix");
                    }

                    // Display image sent
                    imageViewer->display(image);
                }
                else {
                    LOG_INFO("No more images to send");

                    LOG_INFO("Stop relocalization and mapping front end pipeline process");

                    if (gRelocalizationAndMappingFrontendPipeline != 0)
                        gRelocalizationAndMappingFrontendPipeline->stop();

                    LOG_INFO("End of test");

                    exit(0);
                }
            }
        }
        else {
            LOG_INFO("Cannot start AR device loader");
            return -1;
        }
    }
    catch (xpcf::Exception & e) {
        LOG_ERROR("The following exception has been caught {}", e.what());
        return -1;
    }

    return 0;
}
