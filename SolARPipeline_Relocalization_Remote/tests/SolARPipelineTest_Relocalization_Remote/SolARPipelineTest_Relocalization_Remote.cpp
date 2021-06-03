// Copyright (C) 2017-2019 Jonathan Müller <jonathanmueller.dev@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <iostream>

#include <unistd.h>
#include <cxxopts.hpp>

#include <xpcf/xpcf.h>
#include <xpcf/api/IComponentManager.h>
#include <xpcf/core/helpers.h>
#include <boost/log/core.hpp>
#include <signal.h>

#include "core/Log.h"
#include "api/pipeline/IRelocalizationPipeline.h"
#include "api/input/devices/IARDevice.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DPointsViewer.h"
#include "api/storage/IMapManager.h"
#include "api/storage/IPointCloudManager.h"

using namespace std;
using namespace SolAR;
using namespace SolAR::api;
using namespace SolAR::datastructure;
namespace xpcf=org::bcom::xpcf;

// Nb images between 2 pipeline requests
#define NB_IMAGES_BETWEEN_REQUESTS 10

#define INDEX_USE_CAMERA 0

// Global Relocalization Pipeline instance
SRef<pipeline::IRelocalizationPipeline> gRelocalizationPipeline = 0;

// print help options
void print_help(const cxxopts::Options& options)
{
    cout << options.help({""}) << '\n';
}

// print error message
void print_error(const string& msg)
{
    cerr << msg << '\n';
}

// Function called when interruption signal is triggered
static void SigInt(int signo) {

    LOG_INFO("\n\n===> Program interruption\n");

    LOG_INFO("Stop relocalization pipeline process");

    if (gRelocalizationPipeline != 0)
        gRelocalizationPipeline->stop();

    LOG_INFO("End of test");

    exit(0);
}

int main(int argc, char* argv[])
{
    #if NDEBUG
        boost::log::core::get()->set_logging_enabled(false);
    #endif

    LOG_ADD_LOG_TO_CONSOLE();

    // Signal interruption function (Ctrl + C)
    signal(SIGINT, SigInt);

    cxxopts::Options option_list("SolARPipelineTest_Relocalization_Remote",
                                 "SolARPipelineTest_Relocalization_Remote - The commandline interface to the xpcf grpc client test application.\n");
    option_list.add_options()
            ("h,help", "display this help and exit")
            ("v,version", "display version information and exit")
            ("f,file", "xpcf grpc client configuration file",
             cxxopts::value<string>());

    auto options = option_list.parse(argc, argv);
    if (options.count("help")) {
        print_help(option_list);
        return 0;
    }
    else if (options.count("version"))
    {
        cout << "SolARPipelineTest_Relocalization_Remote version 0.9.3 \n";
        cout << '\n';
        return 0;
    }
    else if (!options.count("file") || options["file"].as<string>().empty()) {
        print_error("missing one of file or database dir argument");
        return 1;
    }

    try {
        LOG_INFO("Get component manager instance");
        SRef<xpcf::IComponentManager> componentMgr = xpcf::getComponentManagerInstance();

        string file = options["file"].as<string>();
        LOG_INFO("Load Client Remote Relocalization Pipeline configuration file: {}", file);

        if (componentMgr->load(file.c_str()) == org::bcom::xpcf::_SUCCESS)
        {
            LOG_INFO("Resolve IRelocalizationPipeline interface");
            gRelocalizationPipeline = componentMgr->resolve<SolAR::api::pipeline::IRelocalizationPipeline>();

            if (gRelocalizationPipeline->init() == FrameworkReturnCode::_SUCCESS )
            {
                LOG_INFO("Resolve components used");
                auto arDevice = componentMgr->resolve<input::devices::IARDevice>();
                auto imageViewerResult = componentMgr->resolve<display::IImageViewer>();
                auto viewer3D = componentMgr->resolve<display::I3DPointsViewer>();
                auto mapperManager = componentMgr->resolve<storage::IMapManager>();
                auto pointCloudManager = componentMgr->resolve<storage::IPointCloudManager>();

                // Load map from file
                if (mapperManager->loadFromFile() != FrameworkReturnCode::_SUCCESS) {
                    LOG_INFO("Load map failed!");

                    return -1;
                }

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
        else {
            LOG_INFO("Failed to load Client Remote Relocalization Pipeline configuration file: {}", file);
            return -1;
        }
    }
    catch (xpcf::Exception & e) {
        LOG_INFO("The following exception has been caught: {}", e.what());
        return -1;
    }

    return 0;
}
