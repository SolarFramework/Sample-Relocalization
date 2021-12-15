/**
 * @copyright Copyright (c) 2021 B-com http://www.b-com.com/
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
#define NB_IMAGES_BETWEEN_RELOCALIZATION_REQUESTS 5

// Public methods

SolARMappingAndRelocalizationFrontendPipeline::SolARMappingAndRelocalizationFrontendPipeline():ConfigurableBase(xpcf::toUUID<SolARMappingAndRelocalizationFrontendPipeline>())
{    
    try {
        declareInterface<api::pipeline::IAsyncRelocalizationPipeline>(this);

        LOG_DEBUG("Components injection declaration");
        declareInjectable<api::pipeline::IRelocalizationPipeline>(m_relocalizationService, true);
        declareInjectable<api::pipeline::IMappingPipeline>(m_mappingService, true);
        declareInjectable<api::input::files::ITrackableLoader>(m_trackableLoader, true);
        declareInjectable<api::solver::pose::ITrackablePose>(m_trackablePose, true);

        LOG_DEBUG("All component injections declared");

        LOG_DEBUG("Set the asynchronous task for relocalization");

        // Relocalization processing function
        if (m_relocalizationTask == nullptr) {
            auto fnRelocalizationProcessing = [&]() {
                processRelocalization();
            };

            m_relocalizationTask = new xpcf::DelegateTask(fnRelocalizationProcessing);
        }

        // Relocalization Marker processing function
        if (m_relocalizationMarkerTask == nullptr) {
            auto fnRelocalizationMarkerProcessing = [&]() {
                processRelocalizationMarker();
            };

            m_relocalizationMarkerTask = new xpcf::DelegateTask(fnRelocalizationMarkerProcessing);
        }

        // Mapping processing function
        if (m_mappingTask == nullptr) {
            auto fnMappingProcessing = [&]() {
                processMapping();
            };

            m_mappingTask = new xpcf::DelegateTask(fnMappingProcessing);
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
    delete m_relocalizationMarkerTask;
    delete m_mappingTask;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::init()
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::init");

    // Initialize class members
    m_T_M_W = Transform3Df::Identity();
    m_T_M_W_status = NO_3DTRANSFORM;
    m_confidence = 1;
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

    if ((m_trackableLoader != nullptr) && (m_trackablePose != nullptr)) {

        LOG_DEBUG("Load and set Trackable object");

        SRef<Trackable> trackable;

        if (m_trackableLoader->loadTrackable(trackable) != FrameworkReturnCode::_SUCCESS)
        {
            LOG_ERROR("Cannot load trackable object");
            return FrameworkReturnCode::_ERROR_;
        }
        else
        {
            if (m_trackablePose->setTrackable(trackable) != FrameworkReturnCode::_SUCCESS)
            {
                LOG_ERROR("Cannot set trackable object to trackable pose estimator");
                return FrameworkReturnCode::_ERROR_;
            }
        }
    }
    else {
        LOG_ERROR("Trackable loader and pose instance not created");
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

    if (m_trackablePose != nullptr) {

        LOG_DEBUG("Set camera parameters for the trackable pose");

        m_trackablePose->setCameraParameters(cameraParams.intrinsic, cameraParams.distortion);
    }
    else {
        LOG_ERROR("Trackable pose instance not created");
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
    m_relocalizationMarkerTask->start();
    m_mappingTask->start();

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::stop()
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::stop");

    LOG_DEBUG("Stop relocalization task");
    m_dropBufferRelocalization.empty();
    m_dropBufferMapping.empty();
    m_mappingService->stop();
    m_relocalizationTask->stop();
    m_relocalizationMarkerTask->stop();

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

        if (m_T_M_W_status != NO_3DTRANSFORM)
            m_dropBufferRelocalization.push(std::make_pair(image, pose));

        m_dropBufferRelocalizationMarker.push(std::make_pair(image, pose));

        m_nb_relocalization_images = 0;
    }
    else
        m_nb_relocalization_images ++;

    // Send image and pose to mapping service (if 3D transformation matrix is available)
    if (m_T_M_W_status != NO_3DTRANSFORM) {

        LOG_DEBUG("Push image and pose for mapping task");
        m_dropBufferMapping.push(std::make_pair(image, pose));

        // Update 3D transformation matrix status
        if (m_T_M_W_status == NEW_3DTRANSFORM) {
            LOG_DEBUG("New 3D transformation matrix sent");
            m_T_M_W_status = PREVIOUS_3DTRANSFORM;
        }
    }

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

    // No image encoding to send to relocalization service
    image->setImageEncoding(Image::ENCODING_NONE);

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

void SolARMappingAndRelocalizationFrontendPipeline::processRelocalizationMarker()
{
    std::pair<SRef<Image>, Transform3Df> imagePose;

    if (!m_dropBufferRelocalizationMarker.tryPop(imagePose)) {
        xpcf::DelegateTask::yield();
        return;
    }

    SRef<Image> image = imagePose.first;
    Transform3Df pose = imagePose.second;
    Transform3Df new_pose;

    LOG_DEBUG("Relocalization marker processing");

    if (m_trackablePose->estimate(image, new_pose) == FrameworkReturnCode::_SUCCESS) {

        LOG_INFO("=> Relocalization marker succeeded");
        LOG_INFO("Hololens pose: \n{}", pose.matrix());
        LOG_INFO("World pose: \n{}", new_pose.matrix());

        m_T_M_W = new_pose * pose.inverse();
        m_T_M_W_status = NEW_3DTRANSFORM;

        // To force a relocalization request
        m_nb_relocalization_images = NB_IMAGES_BETWEEN_RELOCALIZATION_REQUESTS;

        LOG_INFO("Transformation matrix from Hololens to World: \n{}", m_T_M_W.matrix());
    }
}

void SolARMappingAndRelocalizationFrontendPipeline::processMapping()
{
    std::pair<SRef<Image>, Transform3Df> imagePose;

    if (!m_dropBufferMapping.tryPop(imagePose)) {
        xpcf::DelegateTask::yield();
        return;
    }

    SRef<Image> image = imagePose.first;
    Transform3Df pose = imagePose.second;

    // No image encoding to send to mapping service
    image->setImageEncoding(Image::ENCODING_NONE);

    LOG_DEBUG("Send image and pose to mapping service");

    if (m_mappingService->mappingProcessRequest(image, m_T_M_W * pose) != SolAR::FrameworkReturnCode::_SUCCESS) {
        LOG_DEBUG("Mapping processing request failed");
    }
}

} // namespace RELOCALIZATION
} // namespace PIPELINES
} // namespace SolAR
