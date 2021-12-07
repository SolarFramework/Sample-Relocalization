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
#include "SolARMappingAndRelocalizationFrontendPipeline.h"
#include "core/Log.h"
#include "boost/log/core/core.hpp"

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
    using namespace datastructure;
namespace PIPELINES {
namespace RELOCALIZATION {

// Set the number of images between to requests to the relocalization service
#define NB_IMAGES_BETWEEN_RELOCALIZATION_REQUESTS 10

// Public methods

SolARMappingAndRelocalizationFrontendPipeline::SolARMappingAndRelocalizationFrontendPipeline():ConfigurableBase(xpcf::toUUID<SolARMappingAndRelocalizationFrontendPipeline>())
{    
    try {
        declareInterface<api::pipeline::IAsyncRelocalizationPipeline>(this);

        LOG_DEBUG("Components injection declaration");
        declareInjectable<api::pipeline::IRelocalizationPipeline>(m_relocalizationService, true);
        declareInjectable<api::pipeline::IMappingPipeline>(m_mappingService, true);

        LOG_DEBUG("All component injections declared");

        LOG_DEBUG("Set the asynchronous task for relocalization");

        // Relocalization processing function
        if (m_relocalizationTask == nullptr) {
            auto fnRelocalizationProcessing = [&]() {
                processRelocalization();
            };

            m_relocalizationTask = new xpcf::DelegateTask(fnRelocalizationProcessing);
        }
    }
    catch (xpcf::Exception & e) {
        LOG_ERROR("The following exception has been caught {}", e.what());
    }
    LOG_DEBUG(" SolARMappingAndRelocalizationFrontendPipeline constructor");
}

void SolARMappingAndRelocalizationFrontendPipeline::onInjected() {

    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::onInjected");

    // Get properties

}

SolARMappingAndRelocalizationFrontendPipeline::~SolARMappingAndRelocalizationFrontendPipeline()
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline destructor")

    delete m_relocalizationTask;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::init()
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::init");

    // Initialize class members
    m_T_M_W = Transform3Df::Identity();
    m_T_M_W_status = NO_3DTRANSFORM;
    m_confidence = 0;
    m_nb_relocalization_images = NB_IMAGES_BETWEEN_RELOCALIZATION_REQUESTS;

    if (m_relocalizationService != nullptr){

        LOG_DEBUG("Relocalization service URL = {}",
                 m_relocalizationService->bindTo<xpcf::IConfigurable>()->getProperty("channelUrl")->getStringValue());

        LOG_DEBUG("Initialize the relocalization service");

        try {
            if (m_relocalizationService->init() != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while initializing the relocalization service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised during remote request to the relocalization service: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Relocalization service instance not created");
        return FrameworkReturnCode::_ERROR_;
    }

    if (m_mappingService != nullptr){

        LOG_DEBUG("Mapping service URL = {}",
                 m_mappingService->bindTo<xpcf::IConfigurable>()->getProperty("channelUrl")->getStringValue());

        LOG_DEBUG("Initialize the mapping service");

        try {
            if (m_mappingService->init() != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while initializing the mapping service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised during remote request to the mapping service: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Mapping service instance not created");
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::setCameraParameters(const CameraParameters & cameraParams)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::setCameraParameters");

    if (m_relocalizationService != nullptr){

        LOG_DEBUG("Set camera parameters for the relocalization service");

        try {
            if (m_relocalizationService->setCameraParameters(cameraParams) != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while setting camera parameters for the relocalization service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised during remote request to the relocalization service: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Relocalization service instance not created");
        return FrameworkReturnCode::_ERROR_;
    }

    if (m_mappingService != nullptr){

        LOG_DEBUG("Set camera parameters for the mapping service");

        try {
            if (m_mappingService->setCameraParameters(cameraParams) != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while setting camera parameters for the mapping service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised during remote request to the mapping service: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Mapping service instance not created");
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::getCameraParameters(CameraParameters & cameraParams) const
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::getCameraParameters");

    if (m_relocalizationService != nullptr){

        LOG_DEBUG("Get camera parameters from the relocalization service");

        try {
            if (m_relocalizationService->getCameraParameters(cameraParams) != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while getting camera parameters from the relocalization service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised during remote request to the relocalization service: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Relocalization service instance not created");
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::start()
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::start");

    if (m_relocalizationService != nullptr){

        LOG_DEBUG("Start the relocalization service");

        try {
            if (m_relocalizationService->start() != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while starting the relocalization service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised during remote request to the relocalization service: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Relocalization service instance not created");
        return FrameworkReturnCode::_ERROR_;
    }

    if (m_mappingService != nullptr){

        LOG_DEBUG("Start the mapping service");

        try {
            if (m_mappingService->start() != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while starting the mapping service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised during remote request to the mapping service: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Mapping service instance not created");
        return FrameworkReturnCode::_ERROR_;
    }

    LOG_DEBUG("Start relocalization task");
    m_relocalizationTask->start();

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::stop()
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::stop");

    LOG_DEBUG("Stop relocalization task");
    m_dropBufferRelocalization.empty();
    m_relocalizationTask->stop();

    if (m_relocalizationService != nullptr){

        LOG_DEBUG("Stop the relocalization service");

        try {
            if (m_relocalizationService->stop() != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while stopping the relocalization service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised during remote request to the relocalization service: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Relocalization service instance not created");
        return FrameworkReturnCode::_ERROR_;
    }

    if (m_mappingService != nullptr){

        LOG_DEBUG("Stop the mapping service");

        try {
            if (m_mappingService->stop() != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while stopping the mapping service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised during remote request to the mapping service: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Mapping service instance not created");
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::relocalizeProcessRequest(
                                            const SRef<SolAR::datastructure::Image> image,
                                            const SolAR::datastructure::Transform3Df & pose,
                                            const std::chrono::system_clock::time_point & timestamp,
                                            TransformStatus & transform3DStatus,
                                            SolAR::datastructure::Transform3Df & transform3D,
                                            float_t & confidence)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::relocalizeProcessRequest");

    // Give 3D transformation matrix if available
    transform3DStatus = m_T_M_W_status;
    transform3D = m_T_M_W;
    confidence = m_confidence;

    if (m_nb_relocalization_images == NB_IMAGES_BETWEEN_RELOCALIZATION_REQUESTS) {

        LOG_DEBUG("Push image and pose for relocalization task");

        m_dropBufferRelocalization.push(std::make_pair(image, pose));

        m_nb_relocalization_images = 0;
    }
    else
        m_nb_relocalization_images ++;

    // Send image and pose to mapping service (if 3D transformation matrix is available)
    if (m_T_M_W_status != NO_3DTRANSFORM) {

        LOG_DEBUG("Send image and pose to mapping service");
        m_mappingService->mappingProcessRequest(image, m_T_M_W * pose);

        // Update 3D transformation matrix status
        if (m_T_M_W_status == NEW_3DTRANSFORM) {
            LOG_DEBUG("New 3D transformation matrix sent");
            m_T_M_W_status = PREVIOUS_3DTRANSFORM;
        }
    }
/***** temporaire ****/
    else {
        LOG_DEBUG("Send image and pose to mapping service");
        m_mappingService->mappingProcessRequest(image, m_T_M_W * pose);
    }
/***** temporaire ****/


    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::get3DTransformRequest(
                                            TransformStatus & transform3DStatus,
                                            SolAR::datastructure::Transform3Df & transform3D,
                                            float_t & confidence) const
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::get3DTransformRequest");

    // Give 3D transformation matrix if available
    transform3DStatus = m_T_M_W_status;
    transform3D = m_T_M_W;
    confidence = m_confidence;

    return FrameworkReturnCode::_SUCCESS;
}

void SolARMappingAndRelocalizationFrontendPipeline::processRelocalization()
{
    std::pair<SRef<Image>, Transform3Df> imagePose;

    if (!m_dropBufferRelocalization.tryPop(imagePose)) {
        xpcf::DelegateTask::yield();
        return;
    }

    SRef<Image> image = imagePose.first;
    Transform3Df pose = imagePose.second;

    LOG_DEBUG("Send image and pose to relocalization service");

    Transform3Df new_pose;
    float confidence;

    if (m_relocalizationService->relocalizeProcessRequest(image, new_pose, confidence) == SolAR::FrameworkReturnCode::_SUCCESS) {
        LOG_DEBUG("Relocalization succeeded");

        LOG_DEBUG("Client original pose: {}", pose.matrix());
        LOG_DEBUG("SolAR new pose: {}", new_pose.matrix());

        // Calculate new 3D transformation matrix
        m_T_M_W = new_pose * pose.inverse();
        m_T_M_W_status = NEW_3DTRANSFORM;

        LOG_DEBUG("Transformation matrix from client to SolAR: {}", m_T_M_W.matrix());
    }
    else
    {
        LOG_DEBUG("Relocalization failed");
    }
}

} // namespace RELOCALIZATION
} // namespace PIPELINES
} // namespace SolAR
