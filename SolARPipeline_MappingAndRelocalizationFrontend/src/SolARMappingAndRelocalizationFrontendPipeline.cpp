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
        declareInjectable<api::pipeline::IMappingPipeline>(m_mappingService, true);
        declareInjectable<api::pipeline::IRelocalizationPipeline>(m_relocalizationService, "Map", true);
        declareInjectable<api::pipeline::IRelocalizationPipeline>(m_relocalizationMarkerService, "Marker", true);
        declareProperty("nbSecondsBetweenRequest", m_nbSecondsBetweenRelocRequest);
		declareProperty("nbRelocRequest", m_nbRelocTransformMatrixRequest);

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
        set3DTransform(Transform3Df::Identity());
        m_T_M_W_status = NO_3DTRANSFORM;
        m_confidence = 1;
        m_isNeedReloc = true;
        m_vector_reloc_transf_matrix.clear();
        setLastPose(Transform3Df(Maths::Matrix4f::Zero()));
        if (m_PipelineMode == RELOCALIZATION_ONLY)
            m_maxTimeRequest = 0;
        else
            m_maxTimeRequest = m_nbSecondsBetweenRelocRequest;

        LOG_DEBUG("Empty buffers");

        std::pair<SRef<Image>, Transform3Df> imagePose;
        m_dropBufferRelocalization.tryPop(imagePose);
        m_dropBufferRelocalizationMarker.tryPop(imagePose);
        m_dropBufferMapping.tryPop(imagePose);
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
        }
    }
    else {
        LOG_INFO("Pipeline already stopped");
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

    if (m_started) {

        // Check if pose is valid
        if (!pose.matrix().isZero()) {

            // Store last pose received
            setLastPose(pose);

            // Give 3D transformation matrix if available
            transform3DStatus = m_T_M_W_status;
            transform3D = get3DTransform();
            confidence = m_confidence;

            // Relocalization
            if (checkNeedReloc()){
                LOG_DEBUG("Push image and pose for relocalization task");
                m_dropBufferRelocalization.push(std::make_pair(image, pose));
                m_dropBufferRelocalizationMarker.push(std::make_pair(image, pose));
            }

            // Mapping if the pipeline mode is mapping and found 3D Transform
            if ((m_PipelineMode == RELOCALIZATION_AND_MAPPING) && (m_T_M_W_status != NO_3DTRANSFORM)) {
                LOG_DEBUG("Push image and pose for mapping task");
                m_dropBufferMapping.push(std::make_pair(image, pose));
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
        transform3DStatus = m_T_M_W_status;
        if (m_T_M_W_status == NEW_3DTRANSFORM) {
            m_T_M_W_status = PREVIOUS_3DTRANSFORM;
        }
        transform3D = get3DTransform();
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
        std::unique_lock<std::mutex> lock(m_mutexLastPose);
        if (poseType == DEVICE_POSE) {
            // Return last pose in device coordinate system
            pose = m_lastPose;
        }
        else if (poseType == SOLAR_POSE) {
            // Return last pose in SolAR coordinate system
            if (m_T_M_W_status != NO_3DTRANSFORM) {
                pose = m_T_M_W * m_lastPose;
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
        LOG_INFO("Relocalization succeeded");
        LOG_DEBUG("Client original pose: \n{}", pose.matrix());
        LOG_DEBUG("SolAR new pose: \n{}", new_pose.matrix());
		LOG_INFO("Transformation matrix from client to SolAR:\n{}", (new_pose * pose.inverse()).matrix());
        findTransformation(new_pose * pose.inverse());
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
    Transform3Df updatedT_M_W;
    MappingStatus mappingStatus;
    Transform3Df curT_M_W = get3DTransform();
    if (m_mappingService->mappingProcessRequest(image, pose, curT_M_W, updatedT_M_W, mappingStatus) == SolAR::FrameworkReturnCode::_SUCCESS) {
        LOG_INFO("Mapping status: {}", mappingStatus);
        if (!(updatedT_M_W * curT_M_W.inverse()).isApprox(Transform3Df::Identity())) {
            LOG_INFO("New transform found by loop closure:\n{}", updatedT_M_W.matrix());
            set3DTransform(updatedT_M_W);
            m_T_M_W_status = NEW_3DTRANSFORM;
        }
    }
    else {
        LOG_DEBUG("Mapping processing request failed");
    }
}

void SolARMappingAndRelocalizationFrontendPipeline::findTransformation(Transform3Df transform)
{
    std::unique_lock<std::mutex> lock(m_mutexFindTransform);
	m_vector_reloc_transf_matrix.push_back(transform);
	// find mean transformation
	if (m_vector_reloc_transf_matrix.size() == m_nbRelocTransformMatrixRequest) {
		Vector3f translations(0.f, 0.f, 0.f); 
		Vector3f eulers(0.f, 0.f, 0.f);
		for (auto t : m_vector_reloc_transf_matrix) {
			translations += t.translation();
			Vector3f e = t.rotation().eulerAngles(0, 1, 2);
			if (e[0] < 0) e[0] += 2 * SOLAR_PI;
			if (e[1] < 0) e[1] += 2 * SOLAR_PI;
			if (e[2] < 0) e[2] += 2 * SOLAR_PI;
			eulers += e;
		}
		translations /= m_nbRelocTransformMatrixRequest;
		eulers /= m_nbRelocTransformMatrixRequest;
		Maths::Matrix3f rot;
		rot = Maths::AngleAxisf(eulers[0], Vector3f::UnitX())
			* Maths::AngleAxisf(eulers[1], Vector3f::UnitY())
			* Maths::AngleAxisf(eulers[2], Vector3f::UnitZ());
        Transform3Df transform3D;
        transform3D.linear() = rot;
        transform3D.translation() = translations;
        LOG_INFO("Mean transformation matrix from device to SolAR:\n{}", transform3D.matrix());
        set3DTransform(transform3D);
        m_T_M_W_status = NEW_3DTRANSFORM;
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

/// @brief get 3D transform
Transform3Df SolARMappingAndRelocalizationFrontendPipeline::get3DTransform()
{
    std::unique_lock<std::mutex> lock(m_mutexTransform);
    return m_T_M_W;
}

/// @brief set 3D transform
void SolARMappingAndRelocalizationFrontendPipeline::set3DTransform(const Transform3Df& transform3D)
{
    std::unique_lock<std::mutex> lock(m_mutexTransform);
    m_T_M_W = transform3D;
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
