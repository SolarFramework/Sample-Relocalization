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

// Public methods

SolARMappingAndRelocalizationFrontendPipeline::SolARMappingAndRelocalizationFrontendPipeline():ConfigurableBase(xpcf::toUUID<SolARMappingAndRelocalizationFrontendPipeline>())
{
    try {
        declareInterface<api::pipeline::IAsyncRelocalizationPipeline>(this);

        LOG_DEBUG("Components injection declaration");
        declareInjectable<api::pipeline::IMappingPipeline>(m_mappingService, "Mapping",  true);
        declareInjectable<api::pipeline::IMappingPipeline>(m_mappingStereoService, "StereoMapping",  true);
        declareInjectable<api::pipeline::IRelocalizationPipeline>(m_relocalizationService, "RelocalizationMap", true);
        declareInjectable<api::pipeline::IRelocalizationPipeline>(m_relocalizationMarkerService, "RelocalizationMarkers", true);
        declareInjectable<api::pipeline::IMapUpdatePipeline>(m_mapupdateService, true);
        declareProperty("nbSecondsBetweenRequest", m_nbSecondsBetweenRelocRequest);
        declareProperty("nbRelocRequest", m_nbRelocTransformMatrixRequest);
        declareProperty("thresholdTranslationRatio", m_thresTranslationRatio);
        declareProperty("minCumulativeDistance", m_minCumulativeDistance);
        declareProperty("maxDistanceRelocMatrix", m_maxDistanceRelocMatrix);
        declareProperty("thresholdRelocConfidence", m_thresRelocConfidence);

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

    // Stop services if needed
    if (m_started)
        stop();

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

    if (m_relocalizationMarkerService != nullptr){

        LOG_DEBUG("Relocalization marker service URL = {}",
                 m_relocalizationMarkerService->bindTo<xpcf::IConfigurable>()->getProperty("channelUrl")->getStringValue());

        LOG_DEBUG("Initialize the relocalization marker service");

        try {
            if (m_relocalizationMarkerService->init() != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while initializing the relocalization marker service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised during remote request to the relocalization marker service: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Relocalization marker service instance not created");
        return FrameworkReturnCode::_ERROR_;
    }

    if (m_PipelineMode == RELOCALIZATION_AND_MAPPING){

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

        if (m_mappingStereoService != nullptr){

            LOG_DEBUG("Mapping Stereo service URL = {}",
                     m_mappingStereoService->bindTo<xpcf::IConfigurable>()->getProperty("channelUrl")->getStringValue());

            LOG_DEBUG("Initialize the mapping stereo service");

            try {
                if (m_mappingStereoService->init() != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while initializing the mapping stereo service");
                }
                else {
                    m_stereoMappingOK = true;
                }
            }  catch (const std::exception &e) {
                LOG_DEBUG("Exception raised during remote request to the mapping stereo service: {}", e.what());
            }
        }
        else {
            LOG_DEBUG("Mapping stereo service instance not created");
        }
    }

    if (m_init) {
        LOG_WARNING("Pipeline has already been initialized");
        return FrameworkReturnCode::_SUCCESS;
    }

    m_init = true;

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::init(PipelineMode pipelineMode)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::init(PipelineMode)");

    m_PipelineMode = pipelineMode;

    return init();
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::setCameraParameters(const CameraParameters & cameraParams)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::setCameraParameters");

    if (!m_init) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

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

    if (m_relocalizationMarkerService != nullptr){

        LOG_DEBUG("Set camera parameters for the relocalization marker service");

        try {
            if (m_relocalizationMarkerService->setCameraParameters(cameraParams) != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while setting camera parameters for the relocalization marker service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const std::exception &e) {
            LOG_ERROR("Exception raised during remote request to the relocalization marker service: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Relocalization marker service instance not created");
        return FrameworkReturnCode::_ERROR_;
    }

    if (m_PipelineMode == RELOCALIZATION_AND_MAPPING){

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
    }

    m_cameraOK = true;

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::setCameraParameters(const SolAR::datastructure::CameraParameters & cameraParams1,
                                                                                       const SolAR::datastructure::CameraParameters & cameraParams2)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::setCameraParameters(stereo)");

    if (!m_init) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

    if (m_PipelineMode == RELOCALIZATION_AND_MAPPING){

        if (m_stereoMappingOK){

            LOG_DEBUG("Set camera parameters for the mapping stereo service");

            try {
                if (m_mappingStereoService->setCameraParameters(cameraParams1, cameraParams2) != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while setting camera parameters for the mapping stereo service");
                    return FrameworkReturnCode::_ERROR_;
                }
            }  catch (const std::exception &e) {
                LOG_ERROR("Exception raised during remote request to the mapping stereo service: {}", e.what());
                return FrameworkReturnCode::_ERROR_;
            }
        }
        else {
            LOG_DEBUG("Mapping service stereo instance not created");
        }
    }

    return setCameraParameters(cameraParams1);
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::setRectificationParameters(const SolAR::datastructure::RectificationParameters & rectCam1,
                                                                                              const SolAR::datastructure::RectificationParameters & rectCam2)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::setRectificationParameters");

    if (!m_init) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

    if (m_PipelineMode == RELOCALIZATION_AND_MAPPING){

        if (m_stereoMappingOK){

            LOG_DEBUG("Set rectification parameters for the mapping stereo service");

            try {
                if (m_mappingStereoService->setRectificationParameters(rectCam1, rectCam2) != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while setting rectification parameters for the mapping stereo service");
                    return FrameworkReturnCode::_ERROR_;
                }
            }  catch (const std::exception &e) {
                LOG_ERROR("Exception raised during remote request to the mapping stereo service: {}", e.what());
                return FrameworkReturnCode::_ERROR_;
            }
        }
        else {
            LOG_DEBUG("Mapping service stereo instance not created");
            return FrameworkReturnCode::_SUCCESS;
        }
    }

    m_rectificationOK = true;

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::getCameraParameters(CameraParameters & cameraParams) const
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::getCameraParameters");

    if (!m_cameraOK){
        LOG_ERROR("Camera parameters have not been set");
        return FrameworkReturnCode::_ERROR_;
    }

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

    if (!m_init) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

    if (!m_cameraOK){
        LOG_ERROR("Camera parameters have not been set");
        return FrameworkReturnCode::_ERROR_;
    }

    if (!m_started) {

        LOG_DEBUG("Initialize instance attributes");

        // Initialize class members
        set3DTransformWorld(Transform3Df::Identity());
        set3DTransformSolAR(Transform3Df::Identity());
        m_T_status = NO_3DTRANSFORM;
        m_confidence = 1;
        m_mappingStatus = BOOTSTRAP;
        m_isNeedReloc = true;
        m_vector_reloc_transf_matrix.clear();
        setLastPose(Transform3Df(Maths::Matrix4f::Zero()));
        if (m_PipelineMode == RELOCALIZATION_ONLY)
            m_maxTimeRequest = 0;
        else
            m_maxTimeRequest = m_nbSecondsBetweenRelocRequest;
        m_cumulativeDistance = 0.f;

        LOG_DEBUG("Empty buffers");

        std::pair<SRef<Image>, Transform3Df> imagePose;
        m_dropBufferRelocalization.tryPop(imagePose);
        m_dropBufferRelocalizationMarker.tryPop(imagePose);
        DropBufferMappingEntry imagePoses;
        m_dropBufferMapping.tryPop(imagePoses);
/*
        m_dropBufferRelocalization.clear();
        m_dropBufferRelocalizationMarker.clear();
        m_dropBufferMapping.clear();
*/
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

        if (m_relocalizationMarkerService != nullptr){

            LOG_DEBUG("Start the relocalization marker service");

            try {
                if (m_relocalizationMarkerService->start() != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while starting the relocalization marker service");
                    return FrameworkReturnCode::_ERROR_;
                }
            }  catch (const std::exception &e) {
                LOG_ERROR("Exception raised during remote request to the relocalization marker service: {}", e.what());
                return FrameworkReturnCode::_ERROR_;
            }
        }
        else {
            LOG_ERROR("Relocalization marker service instance not created");
            return FrameworkReturnCode::_ERROR_;
        }

        if (m_PipelineMode == RELOCALIZATION_AND_MAPPING){

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

            if ((m_stereoMappingOK) && (m_rectificationOK)){

                LOG_DEBUG("Start the mapping stereo service");

                try {
                    if (m_mappingStereoService->start() != FrameworkReturnCode::_SUCCESS) {
                        LOG_ERROR("Error while starting the mapping stereo service");
                        return FrameworkReturnCode::_ERROR_;
                    }
                }  catch (const std::exception &e) {
                    LOG_ERROR("Exception raised during remote request to the mapping stereo service: {}", e.what());
                    return FrameworkReturnCode::_ERROR_;
                }
            }
            else {
                LOG_DEBUG("Mapping service stereo instance not created");
            }
        }

        if (!m_tasksStarted) {
            LOG_DEBUG("Start relocalization task");
            m_relocalizationTask->start();
            m_relocalizationMarkerTask->start();

            LOG_DEBUG("Start mapping task");
            m_mappingTask->start();

            m_tasksStarted = true;
        }

        m_started = true;
    }
    else {
        LOG_ERROR("Pipeline already started");
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::stop()
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::stop");

    if (!m_init) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

    if (!m_cameraOK){
        LOG_ERROR("Camera parameters have not been set");
        return FrameworkReturnCode::_ERROR_;
    }

    if (m_started) {

        m_started = false;

        if (m_tasksStarted) {
            LOG_DEBUG("Stop relocalization task");
            m_relocalizationTask->stop();
            m_relocalizationMarkerTask->stop();

            LOG_DEBUG("Stop mapping task");
            m_mappingTask->stop();

            m_tasksStarted = false;
        }

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

        if (m_relocalizationMarkerService != nullptr){

            LOG_DEBUG("Stop the relocalization marker service");

            try {
                if (m_relocalizationMarkerService->stop() != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while stopping the relocalization marker service");
                    return FrameworkReturnCode::_ERROR_;
                }
            }  catch (const std::exception &e) {
                LOG_ERROR("Exception raised during remote request to the relocalization marker service: {}", e.what());
                return FrameworkReturnCode::_ERROR_;
            }
        }
        else {
            LOG_ERROR("Relocalization marker service instance not created");
            return FrameworkReturnCode::_ERROR_;
        }

        if (m_PipelineMode == RELOCALIZATION_AND_MAPPING){

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

            if ((m_stereoMappingOK) && (m_rectificationOK)){

                LOG_DEBUG("Stop the mapping stereo service");

                try {
                    if (m_mappingStereoService->stop() != FrameworkReturnCode::_SUCCESS) {
                        LOG_ERROR("Error while stopping the mapping stereo service");
                        return FrameworkReturnCode::_ERROR_;
                    }
                }  catch (const std::exception &e) {
                    LOG_ERROR("Exception raised during remote request to the mapping stereo service: {}", e.what());
                    return FrameworkReturnCode::_ERROR_;
                }
            }
            else {
                LOG_DEBUG("Mapping service stereo instance not created");
            }
        }
    }
    else {
        LOG_INFO("Pipeline already stopped");
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::relocalizeProcessRequest(const std::vector<SRef<SolAR::datastructure::Image>> & images,
                                                                                            const std::vector<SolAR::datastructure::Transform3Df> & poses,
                                                                                            bool fixedPose,
                                                                                            const SolAR::datastructure::Transform3Df & worldTransform,
                                                                                            const std::chrono::system_clock::time_point & timestamp,
                                                                                            TransformStatus & transform3DStatus,
                                                                                            SolAR::datastructure::Transform3Df & transform3D,
                                                                                            float_t & confidence,
                                                                                            MappingStatus & mappingStatus)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::relocalizeProcessRequest");

    if (m_started) {

        // Check if pose is valid
        if (!poses[0].matrix().isZero()) {

            // Update cumulative distance
            if (m_mappingStatus != BOOTSTRAP) {
                // transform exists -> already relocalized -> last pose exists
                Transform3Df lastPose;
                if (getLastPose(lastPose, DEVICE_POSE) != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Failed to get last pose");
                    return FrameworkReturnCode::_ERROR_;
                }
                LOG_DEBUG("Last pose = {}", lastPose.matrix());
                LOG_DEBUG("Current pose = {}", poses[0].matrix());
                Vector3f diffTranslation(lastPose(0, 3)-poses[0](0, 3), lastPose(1, 3)-poses[0](1, 3), lastPose(2, 3)-poses[0](2, 3));
                LOG_DEBUG("Distance between 2 last poses = {}", diffTranslation.norm());
                m_cumulativeDistance = m_cumulativeDistance + diffTranslation.norm();
            }

            // Store last pose received
            setLastPose(poses[0]);

            // Give 3D transformation matrix if available
            transform3DStatus = m_T_status;
            transform3D = get3DTransformWorld();
            confidence = m_confidence;
            mappingStatus = m_mappingStatus;

            // Relocalization
            if (checkNeedReloc()){
                LOG_DEBUG("Push image and pose for relocalization task");
                m_dropBufferRelocalization.push(std::make_pair(images[0], poses[0]));
                if (m_mappingStatus == BOOTSTRAP)
                    m_dropBufferRelocalizationMarker.push(std::make_pair(images[0], poses[0]));
            }

            // Mapping if the pipeline mode is mapping and found 3D Transform
            if ((m_PipelineMode == RELOCALIZATION_AND_MAPPING) && (m_T_status != NO_3DTRANSFORM)) {
                LOG_DEBUG("Push image and pose for mapping task");
                m_dropBufferMapping.push({ images, poses, /* fixedpose = */ fixedPose, worldTransform });
            }
        }
    }
    else {
        if (!m_init) {
            LOG_ERROR("Pipeline has not been initialized");
        }
        if (!m_cameraOK){
            LOG_ERROR("Camera parameters have not been set");
        }
        if (!m_started){
            LOG_ERROR("Pipeline has not been started");
        }
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::get3DTransformRequest(
                                            TransformStatus & transform3DStatus,
                                            SolAR::datastructure::Transform3Df & transform3D,
                                            float_t & confidence)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::get3DTransformRequest");

    if (m_started) {

        // Give 3D transformation matrix if available
        transform3DStatus = m_T_status;
        if (m_T_status == NEW_3DTRANSFORM) {
            m_T_status = PREVIOUS_3DTRANSFORM;
        }
        transform3D = get3DTransformWorld();
        confidence = m_confidence;
    }
    else {
        LOG_ERROR("Pipeline not started!");
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::getLastPose(
                                SolAR::datastructure::Transform3Df & pose,
                                const PoseType poseType) const
{
    if (!m_started) {
        LOG_ERROR("Pipeline not started!");
        return FrameworkReturnCode::_ERROR_;
    }
    if (!m_lastPose.matrix().isZero()) {
        std::lock_guard<std::mutex> lock(m_mutexLastPose);
        if (poseType == DEVICE_POSE) {
            // Return last pose in device coordinate system
            pose = m_lastPose;
        }
        else if (poseType == SOLAR_POSE) {
            // Return last pose in SolAR coordinate system
            if (m_T_status != NO_3DTRANSFORM) {
                pose = m_T_M_SolAR * m_lastPose;
            }
            else {
                LOG_DEBUG("No 3D transformation matrix");
                return FrameworkReturnCode::_ERROR_;
            }
        }
        else {
            LOG_ERROR("Unknown pose type");
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_DEBUG("No available pose");
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::getMapRequest(SRef<SolAR::datastructure::Map> & map) const
{
    if (m_mapupdateService != nullptr) {
        return m_mapupdateService->getMapRequest(map);
    }
    else {
        return FrameworkReturnCode::_ERROR_;
    }
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::resetMap() const
{
    if (m_mapupdateService != nullptr) {
        return m_mapupdateService->resetMap();
    }
    else {
        return FrameworkReturnCode::_ERROR_;
    }
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::getPointCloudRequest(SRef<SolAR::datastructure::PointCloud> & pointCloud) const
{
    if (m_mapupdateService != nullptr) {
        return m_mapupdateService->getPointCloudRequest(pointCloud);
    }
    else {
        return FrameworkReturnCode::_ERROR_;
    }
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

    try {
        Transform3Df poseCoarse = Transform3Df::Identity();
        auto curTSolAR = get3DTransformSolAR();
        if (!curTSolAR.isApprox(Transform3Df::Identity()))  // if T defined compute coarse pose in SolAR
            poseCoarse = curTSolAR*pose;
        if (m_relocalizationService->relocalizeProcessRequest(image, new_pose, confidence, poseCoarse) == SolAR::FrameworkReturnCode::_SUCCESS) {
            LOG_INFO("Relocalization succeeded");
            LOG_DEBUG("Client original pose: \n{}", pose.matrix());
            LOG_DEBUG("SolAR new pose: \n{}", new_pose.matrix());

            // test reloc pose by comparing it to device pose, reject the reloc if big difference is observed
            if (m_mappingStatus != BOOTSTRAP) {
                auto poseArrInSolar = m_T_M_SolAR*pose;
                Vector3f dist(poseArrInSolar(0, 3)-new_pose(0, 3), poseArrInSolar(1, 3)-new_pose(1, 3), poseArrInSolar(2, 3)-new_pose(2, 3));
                LOG_DEBUG("Pose distance = {} / cumulative distance = {} / min cumulative distance = {} / ratio = {} / cumulative distance*ration = {}",
                         dist.norm(), m_cumulativeDistance, m_minCumulativeDistance, m_thresTranslationRatio, m_cumulativeDistance*m_thresTranslationRatio);
                if ((m_cumulativeDistance > m_minCumulativeDistance) && (dist.norm() > m_cumulativeDistance*m_thresTranslationRatio)) {
                    LOG_WARNING("SolAR reloc pose is rejected because translation vector too different from that in AR runtime pose");
                    m_cumulativeDistance = 0.f; // reset cumulative distance
                    return;
                }
            }
            else {
                if (confidence < m_thresRelocConfidence) {
                    LOG_WARNING("Reloc confidence score {} is lower than {} wait for next reloc", confidence, m_thresRelocConfidence);
                    return;
                }
                LOG_INFO("Reloc with confidence score {} is used to initialize the transform from AR runtime to SolAR", confidence);
            }

            LOG_INFO("Transformation matrix from client to SolAR:\n{}", (new_pose * pose.inverse()).matrix());
            findTransformation(new_pose * pose.inverse());
            m_cumulativeDistance = 0.f; // reset cumulative distance when relocalized
        }
    }  catch (const std::exception &e) {
        LOG_ERROR("Exception raised during remote request to the relocalization service: {}", e.what());

        m_mappingStatus = TRACKING_LOST;
        m_T_status = NO_3DTRANSFORM;
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

    // No image encoding to send to relocalization service
    image->setImageEncoding(Image::ENCODING_NONE);

    LOG_DEBUG("Send image and pose to relocalization marker service");

    Transform3Df new_pose;
    float confidence;

    if (m_relocalizationMarkerService->relocalizeProcessRequest(image, new_pose, confidence) == SolAR::FrameworkReturnCode::_SUCCESS) {
        LOG_INFO("Relocalization Marker succeeded");
        LOG_DEBUG("Client original pose: \n{}", pose.matrix());
        LOG_DEBUG("SolAR new pose: \n{}", new_pose.matrix());
        LOG_INFO("Transformation matrix from client to SolAR:\n{}", (new_pose * pose.inverse()).matrix());
        findTransformation(new_pose * pose.inverse());
    }
}

void SolARMappingAndRelocalizationFrontendPipeline::processMapping()
{
    DropBufferMappingEntry imagePoses;
    if (!m_dropBufferMapping.tryPop(imagePoses)) {
        xpcf::DelegateTask::yield();
        return;
    }

    std::vector<SRef<Image>> images = imagePoses.images;
    std::vector<Transform3Df> poses = imagePoses.poses;
    bool fixedPose = imagePoses.fixedPose;
    Transform3Df worldTransform = imagePoses.worldTransform;

    if (fixedPose && m_isTransformS2WSet) { // update T_ARr_World if GT pose (GT pose is defined to be fixed, i.e. cannot change during BA)
        // here we get as input T_ARr_World we need adjust T_ARr_SolAR accordingly
        // both transforms are used to compensate for pose drift:
        // T_ARr_World*pose_ARr --> right pose in World & T_ARr_SolAR*pose_ARr --> right pose in SolAR
        auto cur_TW = get3DTransformWorld();
        auto cur_TS = get3DTransformSolAR();
        LOG_INFO("Mapping: receiving 2nd, 3rd, ... fixedPose current t_w:\n {} \n current t_s: \n {}", cur_TW.matrix(), cur_TS.matrix());
        set3DTransformSolAR(cur_TS*cur_TW.inverse()*worldTransform);
        set3DTransformWorld(worldTransform);
        auto cur_TW2 = get3DTransformWorld();
        auto cur_TS2 = get3DTransformSolAR();
        LOG_INFO("Mapping: after correction \n t_w: \n {}  t_s : \n {}", cur_TW2.matrix(), cur_TS2.matrix());
    }

    // No image encoding to send to mapping service
    for (auto & image : images)
        image->setImageEncoding(Image::ENCODING_NONE);

    LOG_DEBUG("Send image and pose to mapping service");
    Transform3Df updatedT_M_W;
    MappingStatus mappingStatus;
    if (fixedPose && !m_isTransformS2WSet) {
        set3DTransformWorld(worldTransform);
        Transform3Df curT_M_SolAR = get3DTransformSolAR();
        LOG_INFO("Mapping: receiving 1st fixedPose current t_w:\n {} \n current t_s: \n {}", worldTransform.matrix(), curT_M_SolAR.matrix());
        if (m_stereoMappingOK) {
            // TODO: implement stereo mapping case
            LOG_ERROR("GT pose in stereo case is not yet implemented");
        }
        else {
            if (m_mappingService->set3DTransformSolARToWorld(worldTransform * curT_M_SolAR.inverse()) != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Failed to set transform solar to world in map");
                return;
            }
        }
        m_isTransformS2WSet = true;  // set only once SolAR to World transform
    }

    // Mono or stereo images?
    Transform3Df curT_M_W = get3DTransformWorld();
    if ((images.size() >= 2) && (m_stereoMappingOK) && (m_rectificationOK)) {
        LOG_DEBUG("Stereo mapping processing");
        // TODO: implement stereo mapping pipeline to take into account the case where fixedPose=True
        if (m_mappingStereoService->mappingProcessRequest(images, poses, /* fixedPose = */ fixedPose, curT_M_W, updatedT_M_W, mappingStatus) == SolAR::FrameworkReturnCode::_SUCCESS) {
            LOG_DEBUG("Mapping stereo status: {}", mappingStatus);
            m_mappingStatus = mappingStatus;
            if (!(updatedT_M_W * curT_M_W.inverse()).isApprox(Transform3Df::Identity())) {
                LOG_INFO("New transform found by loop closure:\n{}", updatedT_M_W.matrix());
                auto cur_TW = get3DTransformWorld();
                auto cur_TS = get3DTransformSolAR();
                set3DTransformSolAR(cur_TS*cur_TW.inverse()*updatedT_M_W);
                set3DTransformWorld(updatedT_M_W);
                m_T_status = NEW_3DTRANSFORM;
            }
        }
        else {
            LOG_ERROR("Mapping stereo processing request failed");
        }
    }
    else {
        LOG_DEBUG("Mono mapping processing");

        if (m_mappingService->mappingProcessRequest(images, poses, fixedPose, curT_M_W, updatedT_M_W, mappingStatus) == SolAR::FrameworkReturnCode::_SUCCESS) {
            LOG_DEBUG("Mapping status: {}", mappingStatus);
            m_mappingStatus = mappingStatus;
            if (!(updatedT_M_W * curT_M_W.inverse()).isApprox(Transform3Df::Identity())) {
                LOG_INFO("New transform found by loop closure:\n{}", updatedT_M_W.matrix());
                auto cur_TW = get3DTransformWorld();
                auto cur_TS = get3DTransformSolAR();
                set3DTransformSolAR(cur_TS*cur_TW.inverse()*updatedT_M_W);
                set3DTransformWorld(updatedT_M_W);
                m_T_status = NEW_3DTRANSFORM;
            }
        }
        else {
            LOG_ERROR("Mapping processing request failed");
        }
    }
}

void SolARMappingAndRelocalizationFrontendPipeline::findTransformation(Transform3Df transform)
{
    std::unique_lock<std::mutex> lock(m_mutexFindTransform);
    m_vector_reloc_transf_matrix.push_back(transform);
    // find mean transformation
    if (m_vector_reloc_transf_matrix.size() == m_nbRelocTransformMatrixRequest) {
        Transform3Df transform3D = transform3DAverage(m_vector_reloc_transf_matrix);
        LOG_INFO("Mean transformation matrix from device to SolAR:\n{}", transform3D.matrix());
        if (m_T_M_SolAR.isApprox(Transform3Df::Identity())) { // has not been initialized
            set3DTransformWorld(transform3D); // set T_ARr_to_World
            set3DTransformSolAR(transform3D);  // set T_ARr_to_SolAR
        }
        else { // here we get as input T_ARr_SolAR and we adjust T_ARr_World
            Transform3Df curT_M_W = get3DTransformWorld();
            Transform3Df curT_M_SolAR = get3DTransformSolAR();
            set3DTransformWorld(curT_M_W*curT_M_SolAR.inverse()*transform3D);  // adjust T_ARr_World
            set3DTransformSolAR(transform3D);
        }

        if (m_mappingStatus == BOOTSTRAP)
            m_mappingStatus = MAPPING;
        m_T_status = NEW_3DTRANSFORM;
        m_relocTimer.restart();
        m_isNeedReloc = false;
        m_vector_reloc_transf_matrix.clear();
    }
}

/// @brief check if need to relocalize
bool SolARMappingAndRelocalizationFrontendPipeline::checkNeedReloc()
{
    if (!m_isNeedReloc && (m_relocTimer.elapsed() > m_maxTimeRequest * 1000))
        m_isNeedReloc = true;
    return m_isNeedReloc;
}

/// @brief get 3D transform World
Transform3Df SolARMappingAndRelocalizationFrontendPipeline::get3DTransformWorld()
{
    std::unique_lock<std::mutex> lock(m_mutexTransformWorld);
    return m_T_M_World;
}

/// @brief get 3D transform SolAR
Transform3Df SolARMappingAndRelocalizationFrontendPipeline::get3DTransformSolAR()
{
    std::unique_lock<std::mutex> lock(m_mutexTransformSolAR);
    return m_T_M_SolAR;
}

/// @brief set 3D transform World
void SolARMappingAndRelocalizationFrontendPipeline::set3DTransformWorld(const Transform3Df& transform3D)
{
    std::unique_lock<std::mutex> lock(m_mutexTransformWorld);
    m_T_M_World = transform3D;
}

/// @brief set 3D transform SolAR
void SolARMappingAndRelocalizationFrontendPipeline::set3DTransformSolAR(const Transform3Df& transform3D)
{
    std::unique_lock<std::mutex> lock(m_mutexTransformSolAR);
    m_T_M_SolAR = transform3D;
    LOG_INFO("m_T_M_SolAR = {}", m_T_M_SolAR.matrix());
}

/// @brief set last pose
void SolARMappingAndRelocalizationFrontendPipeline::setLastPose(const Transform3Df& lastPose)
{
    std::unique_lock<std::mutex> lock(m_mutexLastPose);
    m_lastPose = lastPose;
}

} // namespace RELOCALIZATION
} // namespace PIPELINES
} // namespace SolAR
