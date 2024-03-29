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
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/random_generator.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>

using namespace std;

const string RELOCALIZATION_MARKERS_CONF_FILE = "./SolARService_MappingAndRelocFrontend_RelocMarkers_conf.xml";
const string RELOCALIZATION_CONF_FILE = "./SolARService_MappingAndRelocFrontend_Relocalization_conf.xml";
const string MAPPING_CONF_FILE = "./SolARService_MappingAndRelocFrontend_Mapping_conf.xml";
const string MAPPING_STEREO_CONF_FILE = "./SolARService_MappingAndRelocFrontend_Mapping_Stereo_conf.xml";

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
        declareInjectable<api::pipeline::IServiceManagerPipeline>(m_serviceManager, true);
        declareInjectable<api::pipeline::IMapUpdatePipeline>(m_mapupdateService, true);
        declareProperty("nbSecondsBetweenRequest", m_nbSecondsBetweenRelocRequest);
		declareProperty("nbRelocRequest", m_nbRelocTransformMatrixRequest);
        declareProperty("thresholdTranslationRatio", m_thresTranslationRatio);
        declareProperty("minCumulativeDistance", m_minCumulativeDistance);
        declareProperty("minTransformationDistance", m_minTransformationDistance);
        declareProperty("minDiffTranslationDistance", m_minDiffTranslationDistance);
        declareProperty("thresholdRelocConfidence", m_thresRelocConfidence);
        declareProperty("poseDisparityToleranceInit", m_poseDisparityToleranceInit);
        declareProperty("poseDisparityTolerance", m_poseDisparityTolerance);
        declareProperty("clientActivityDelay", m_clientActivityDelay);
        declareProperty("minRelocalizationDelay", m_minRelocalizationDelay);

        LOG_DEBUG("All component injections declared");

        LOG_DEBUG("Set the asynchronous task for relocalization");        

        // Clients activity task
        if (m_clientsActivityTask == nullptr) {
            auto fnClientsActivity = [&]() {
                testClientsActivity();
            };

            m_clientsActivityTask = new xpcf::DelegateTask(fnClientsActivity);
        }

        LOG_DEBUG("Start clients activity task");
        m_clientsActivityTask->start();

        // Relocalization processing function
        if (m_relocalizationTask == nullptr) {
            auto fnRelocalizationProcessing = [&]() {
                processRelocalization();
            };

            m_relocalizationTask = new xpcf::DelegateTask(fnRelocalizationProcessing);
        }

        // Relocalization Markers processing function
        if (m_relocalizationMarkersTask == nullptr) {
            auto fnRelocalizationMarkersProcessing = [&]() {
                processRelocalizationMarkers();
            };

            m_relocalizationMarkersTask = new xpcf::DelegateTask(fnRelocalizationMarkersProcessing);
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

    if (m_tasksStarted) {
        LOG_DEBUG("Stop clients activity task");
        m_clientsActivityTask->stop();

        LOG_DEBUG("Stop relocalization task");
        m_relocalizationTask->stop();
        m_relocalizationMarkersTask->stop();

        LOG_DEBUG("Stop mapping task");
        m_mappingTask->stop();

        m_tasksStarted = false;
    }

    delete m_clientsActivityTask;
    delete m_relocalizationTask;
    delete m_relocalizationMarkersTask;
    delete m_mappingTask;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::registerClient(string & uuid)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::registerClient");

    boost::uuids::uuid boost_uuid = boost::uuids::random_generator()();

    uuid = boost::lexical_cast<string>(boost_uuid);

    SRef<ClientContext> clientContext = xpcf::utils::make_shared<ClientContext>();

    // Add the new client and its services to the map
    unique_lock<mutex> lock(m_mutexClientMap);
    m_clientsMap.insert(pair<string, SRef<ClientContext>>(uuid, clientContext));
    lock.unlock();

    LOG_DEBUG("New client registered with UUID: {}", uuid);

    if (!m_tasksStarted) {
        LOG_DEBUG("Start relocalization tasks");
        m_relocalizationTask->start();
        m_relocalizationMarkersTask->start();

        LOG_DEBUG("Start mapping task");
        m_mappingTask->start();

        m_tasksStarted = true;
    }

    LOG_DEBUG("Start client activity timer");
    clientContext->m_clientActivityTimer.restart();

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::unregisterClient(const string & uuid)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::unregisterClient");

    // Stop services used by the client if needed
    stop(uuid);

    // Remove the client and its services from the map
    unique_lock<mutex> lock(m_mutexClientMap);
    auto it = m_clientsMap.find(uuid);
    if (it != m_clientsMap.end())
        m_clientsMap.erase(it);

    if ((m_clientsMap.size() == 0) && (m_tasksStarted)) {
        LOG_DEBUG("Stop relocalization tasks");
        m_relocalizationTask->stop();
        m_relocalizationMarkersTask->stop();

        LOG_DEBUG("Stop mapping task");
        m_mappingTask->stop();

        m_tasksStarted = false;
    }

    lock.unlock();

    LOG_DEBUG("Client unregistered with UUID: {}", uuid);

    try {
        if (m_serviceManager != nullptr) {
            m_serviceManager->unlockService(ServiceType::RELOCALIZATION_SERVICE, uuid);
            m_serviceManager->unlockService(ServiceType::RELOCALIZATION_MARKERS_SERVICE, uuid);
            m_serviceManager->unlockService(ServiceType::MAPPING_SERVICE, uuid);
            m_serviceManager->unlockService(ServiceType::MAPPING_STEREO_SERVICE, uuid);
        }
        else {
            LOG_ERROR("Service Manager instance not created");
            return FrameworkReturnCode::_ERROR_;
        }
    }
    catch (xpcf::Exception & e) {
        LOG_ERROR("The following exception has been caught {}", e.what());
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::getAllClientsUUID(std::vector<std::string> & uuidList) const
{
    unique_lock<mutex> lock(m_mutexClientMap);

    try {
        for (const auto& [k, v] : m_clientsMap) {
            uuidList.insert(uuidList.begin(),k);
        }
        return FrameworkReturnCode::_SUCCESS;
    }
    catch (const exception &e) {
        LOG_ERROR("The following exception has been caught {}", e.what());
        return FrameworkReturnCode::_ERROR_;
    }
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::init(const string & uuid)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::init");

    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(uuid);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", uuid);
        return FrameworkReturnCode::_ERROR_;
    }

    LOG_DEBUG("Restart client activity timer");
    clientContext->m_clientActivityTimer.restart();

    LOG_DEBUG("Try to get and lock services for client: {}", uuid);

    if ((clientContext->m_relocalizationURL == "") && (!getAndLockServices(uuid))) {
        LOG_ERROR("Error trying to get and lock services for client: {}", uuid);
        return FrameworkReturnCode::_ERROR_;
    }

    // Stop services if needed
    if (clientContext->m_started)
        stop();

    if (clientContext->m_relocalizationService != nullptr){

        LOG_DEBUG("Relocalization service URL = {}",
                  clientContext->m_relocalizationService->bindTo<xpcf::IConfigurable>()->
                  getProperty("channelUrl")->getStringValue());

        LOG_DEBUG("Initialize the relocalization service");

        try {
            if (clientContext->m_relocalizationService->init() != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while initializing the relocalization service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const exception &e) {
            LOG_ERROR("Exception raised during remote request to the relocalization service: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Relocalization service instance not created");
        return FrameworkReturnCode::_ERROR_;
    }

    if (clientContext->m_relocalizationMarkersService != nullptr){

        LOG_DEBUG("Relocalization markers service URL = {}",
                  clientContext->m_relocalizationMarkersService->bindTo<xpcf::IConfigurable>()->
                  getProperty("channelUrl")->getStringValue());

        LOG_DEBUG("Initialize the relocalization markers service");

        try {
            if (clientContext->m_relocalizationMarkersService->init() != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while initializing the relocalization markers service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const exception &e) {
            LOG_ERROR("Exception raised during remote request to the relocalization markers service: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Relocalization markers service instance not created");
        return FrameworkReturnCode::_ERROR_;
    }

    if (clientContext->m_PipelineMode == RELOCALIZATION_AND_MAPPING){

        if (clientContext->m_mappingService != nullptr){

            LOG_DEBUG("Mapping service URL = {}",
                      clientContext->m_mappingService->bindTo<xpcf::IConfigurable>()->
                      getProperty("channelUrl")->getStringValue());

            LOG_DEBUG("Initialize the mapping service");

            try {
                if (clientContext->m_mappingService->init(clientContext->m_relocalizationURL) != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while initializing the mapping service");
                    return FrameworkReturnCode::_ERROR_;
                }
            }  catch (const exception &e) {
                LOG_ERROR("Exception raised during remote request to the mapping service: {}", e.what());
                return FrameworkReturnCode::_ERROR_;
            }
        }
        else {
            LOG_ERROR("Mapping service instance not created");
            return FrameworkReturnCode::_ERROR_;
        }

        if (clientContext->m_mappingStereoService != nullptr){

            LOG_DEBUG("Mapping Stereo service URL = {}",
                      clientContext->m_mappingStereoService->bindTo<xpcf::IConfigurable>()->
                      getProperty("channelUrl")->getStringValue());

            LOG_DEBUG("Initialize the mapping stereo service");

            try {
                if (clientContext->m_mappingStereoService->init(clientContext->m_relocalizationURL) != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while initializing the mapping stereo service");
                }
                else {
                    clientContext->m_stereoMappingOK = true;
                }
            }  catch (const exception &e) {
                LOG_DEBUG("Exception raised during remote request to the mapping stereo service: {}", e.what());
            }
        }
        else {
            LOG_DEBUG("Mapping stereo service instance not created");
        }
    }

    if (clientContext->m_init) {
        LOG_WARNING("Pipeline has already been initialized");
        return FrameworkReturnCode::_SUCCESS;
    }

    clientContext->m_init = true;

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::init(const string & uuid, PipelineMode pipelineMode)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::init(PipelineMode)");

    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(uuid);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", uuid);
        return FrameworkReturnCode::_ERROR_;
    }

    clientContext->m_PipelineMode = pipelineMode;

    if (pipelineMode == RELOCALIZATION_ONLY) {
        // Unlock mono mapping service
        if (clientContext->m_mappingService != nullptr) {
            m_serviceManager->unlockService(ServiceType::MAPPING_SERVICE, uuid);
            clientContext->m_mappingService = nullptr;
        }
        // Unlock stereo mapping service
        if (clientContext->m_mappingStereoService != nullptr) {
            m_serviceManager->unlockService(ServiceType::MAPPING_STEREO_SERVICE, uuid);
            clientContext->m_mappingStereoService = nullptr;
        }
    }

    return init(uuid);
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::getProcessingMode(const string & uuid,
                                                                                     PipelineMode & pipelineMode) const
{
    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(uuid);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", uuid);
        return FrameworkReturnCode::_ERROR_;
    }

    pipelineMode = clientContext->m_PipelineMode;

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::setCameraParameters(const string & uuid, const CameraParameters & cameraParams)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::setCameraParameters");

    LOG_DEBUG("Camera intrinsic / distortion:\n{}\n{}", cameraParams.intrinsic, cameraParams.distortion);

    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(uuid);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", uuid);
        return FrameworkReturnCode::_ERROR_;
    }

    LOG_DEBUG("Restart client activity timer");
    clientContext->m_clientActivityTimer.restart();

    if (!clientContext->m_init) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

    if (clientContext->m_relocalizationService != nullptr){

        LOG_DEBUG("Set camera parameters for the relocalization service");

        try {
            if (clientContext->m_relocalizationService->setCameraParameters(cameraParams) != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while setting camera parameters for the relocalization service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const exception &e) {
            LOG_ERROR("Exception raised during remote request to the relocalization service: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Relocalization service instance not created");
        return FrameworkReturnCode::_ERROR_;
    }

    if (clientContext->m_relocalizationMarkersService != nullptr){

        LOG_DEBUG("Set camera parameters for the relocalization markers service");

        try {
            if (clientContext->m_relocalizationMarkersService->setCameraParameters(cameraParams) != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while setting camera parameters for the relocalization markers service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const exception &e) {
            LOG_ERROR("Exception raised during remote request to the relocalization markers service: {}", e.what());
            return FrameworkReturnCode::_ERROR_;
        }
    }
    else {
        LOG_ERROR("Relocalization markers service instance not created");
        return FrameworkReturnCode::_ERROR_;
    }

    if (clientContext->m_PipelineMode == RELOCALIZATION_AND_MAPPING){

        if (clientContext->m_mappingService != nullptr){

            LOG_DEBUG("Set camera parameters for the mapping service");

            try {
                if (clientContext->m_mappingService->setCameraParameters(cameraParams) != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while setting camera parameters for the mapping service");
                    return FrameworkReturnCode::_ERROR_;
                }
            }  catch (const exception &e) {
                LOG_ERROR("Exception raised during remote request to the mapping service: {}", e.what());
                return FrameworkReturnCode::_ERROR_;
            }
        }
        else {
            LOG_ERROR("Mapping service instance not created");
            return FrameworkReturnCode::_ERROR_;
        }
    }

    clientContext->m_cameraOK = true;

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::setCameraParameters(const string & uuid,
                                                                                       const SolAR::datastructure::CameraParameters & cameraParams1,
                                                                                       const SolAR::datastructure::CameraParameters & cameraParams2)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::setCameraParameters(stereo)");

    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(uuid);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", uuid);
        return FrameworkReturnCode::_ERROR_;
    }

    LOG_DEBUG("Restart client activity timer");
    clientContext->m_clientActivityTimer.restart();

    if (!clientContext->m_init) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

    if (clientContext->m_PipelineMode == RELOCALIZATION_AND_MAPPING){

        if (clientContext->m_stereoMappingOK){

            LOG_DEBUG("Set camera parameters for the mapping stereo service");

            try {
                if (clientContext->m_mappingStereoService->setCameraParameters(cameraParams1, cameraParams2) != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while setting camera parameters for the mapping stereo service");
                    return FrameworkReturnCode::_ERROR_;
                }
            }  catch (const exception &e) {
                LOG_ERROR("Exception raised during remote request to the mapping stereo service: {}", e.what());
                return FrameworkReturnCode::_ERROR_;
            }
        }
        else {
            LOG_DEBUG("Mapping service stereo instance not created");
        }
    }

    return setCameraParameters(uuid, cameraParams1);
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::setRectificationParameters(const string & uuid,
                                                                                              const SolAR::datastructure::RectificationParameters & rectCam1,
                                                                                              const SolAR::datastructure::RectificationParameters & rectCam2)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::setRectificationParameters");

    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(uuid);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", uuid);
        return FrameworkReturnCode::_ERROR_;
    }

    LOG_DEBUG("Restart client activity timer");
    clientContext->m_clientActivityTimer.restart();

    if (!clientContext->m_init) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

    if (clientContext->m_PipelineMode == RELOCALIZATION_AND_MAPPING){

        if (clientContext->m_stereoMappingOK){

            LOG_DEBUG("Set rectification parameters for the mapping stereo service");

            try {
                if (clientContext->m_mappingStereoService->setRectificationParameters(rectCam1, rectCam2) != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while setting rectification parameters for the mapping stereo service");
                    return FrameworkReturnCode::_ERROR_;
                }
            }  catch (const exception &e) {
                LOG_ERROR("Exception raised during remote request to the mapping stereo service: {}", e.what());
                return FrameworkReturnCode::_ERROR_;
            }
        }
        else {
            LOG_DEBUG("Mapping service stereo instance not created");
            return FrameworkReturnCode::_SUCCESS;
        }
    }

    clientContext->m_rectificationOK = true;

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::getCameraParameters(const string & uuid,
                                                                                       CameraParameters & cameraParams) const
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::getCameraParameters");

    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(uuid);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", uuid);
        return FrameworkReturnCode::_ERROR_;
    }

    if (!clientContext->m_cameraOK){
        LOG_ERROR("Camera parameters have not been set");
        return FrameworkReturnCode::_ERROR_;
    }

    if (clientContext->m_relocalizationService != nullptr){

        LOG_DEBUG("Get camera parameters from the relocalization service");

        try {
            if (clientContext->m_relocalizationService->getCameraParameters(cameraParams) != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Error while getting camera parameters from the relocalization service");
                return FrameworkReturnCode::_ERROR_;
            }
        }  catch (const exception &e) {
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

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::start(const string & uuid)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::start");

    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(uuid);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", uuid);
        return FrameworkReturnCode::_ERROR_;
    }

    LOG_DEBUG("Restart client activity timer");
    clientContext->m_clientActivityTimer.restart();

    if (!clientContext->m_init) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

    if (!clientContext->m_cameraOK){
        LOG_ERROR("Camera parameters have not been set");
        return FrameworkReturnCode::_ERROR_;
    }

    if (!clientContext->m_started) {

        LOG_DEBUG("Initialize instance attributes");

        // Initialize class members
        clientContext->m_T_M_SolARInit = Transform3Df::Identity();
        set3DTransformWorld(clientContext, Transform3Df::Identity());
        set3DTransformSolAR(clientContext, Transform3Df::Identity());
        clientContext->m_T_status = NO_3DTRANSFORM;
        clientContext->m_confidence = 1;
        clientContext->m_mappingStatus = BOOTSTRAP;
        clientContext->m_isNeedReloc = true;
        clientContext->m_vector_reloc_transf_matrix.clear();
        setLastPose(clientContext, Transform3Df(Maths::Matrix4f::Zero()));
        if (clientContext->m_PipelineMode == RELOCALIZATION_ONLY)
            clientContext->m_maxTimeRequest = 0;
        else
            clientContext->m_maxTimeRequest = m_nbSecondsBetweenRelocRequest;
        clientContext->m_cumulativeDistance = 0.f;
/*
        LOG_DEBUG("Empty buffers");

        tuple<string, SRef<Image>, Transform3Df> imagePose;
        m_dropBufferRelocalization.tryPop(imagePose);
        m_dropBufferRelocalizationMarkers.tryPop(imagePose);
        DropBufferMappingEntry imagePoses;
        m_dropBufferMapping.tryPop(imagePoses);

        m_dropBufferRelocalization.clear();
        m_dropBufferRelocalizationMarkers.clear();
        m_dropBufferMapping.clear();
*/
        if (clientContext->m_relocalizationService != nullptr){

            LOG_DEBUG("Start the relocalization service");

            try {
                if (clientContext->m_relocalizationService->start() != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while starting the relocalization service");
                    return FrameworkReturnCode::_ERROR_;
                }
            }  catch (const exception &e) {
                LOG_ERROR("Exception raised during remote request to the relocalization service: {}", e.what());
                return FrameworkReturnCode::_ERROR_;
            }
        }
        else {
            LOG_ERROR("Relocalization service instance not created");
            return FrameworkReturnCode::_ERROR_;
        }

        if (clientContext->m_relocalizationMarkersService != nullptr){

            LOG_DEBUG("Start the relocalization markers service");

            try {
                if (clientContext->m_relocalizationMarkersService->start() != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while starting the relocalization markers service");
                    return FrameworkReturnCode::_ERROR_;
                }
            }  catch (const exception &e) {
                LOG_ERROR("Exception raised during remote request to the relocalization markers service: {}", e.what());
                return FrameworkReturnCode::_ERROR_;
            }
        }
        else {
            LOG_ERROR("Relocalization markers service instance not created");
            return FrameworkReturnCode::_ERROR_;
        }

        if (clientContext->m_PipelineMode == RELOCALIZATION_AND_MAPPING){

            if (clientContext->m_mappingService != nullptr){

                LOG_DEBUG("Start the mapping service");

                try {
                    if (clientContext->m_mappingService->start() != FrameworkReturnCode::_SUCCESS) {
                        LOG_ERROR("Error while starting the mapping service");
                        return FrameworkReturnCode::_ERROR_;
                    }
                }  catch (const exception &e) {
                    LOG_ERROR("Exception raised during remote request to the mapping service: {}", e.what());
                    return FrameworkReturnCode::_ERROR_;
                }
            }
            else {
                LOG_ERROR("Mapping service instance not created");
                return FrameworkReturnCode::_ERROR_;
            }

            if ((clientContext->m_stereoMappingOK) && (clientContext->m_rectificationOK)){

                LOG_DEBUG("Start the mapping stereo service");

                try {
                    if (clientContext->m_mappingStereoService->start() != FrameworkReturnCode::_SUCCESS) {
                        LOG_ERROR("Error while starting the mapping stereo service");
                        return FrameworkReturnCode::_ERROR_;
                    }
                }  catch (const exception &e) {
                    LOG_ERROR("Exception raised during remote request to the mapping stereo service: {}", e.what());
                    return FrameworkReturnCode::_ERROR_;
                }
            }
            else {
                LOG_DEBUG("Mapping service stereo instance not created");
            }
        }

        LOG_DEBUG("Start relocalization delay timer");
        clientContext->m_clientRelocDelay.restart();

        clientContext->m_started = true;
    }
    else {
        LOG_ERROR("Pipeline already started");
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::stop(const string & uuid)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::stop");

    FrameworkReturnCode return_code = FrameworkReturnCode::_SUCCESS;

    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(uuid);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", uuid);
        return FrameworkReturnCode::_ERROR_;
    }

    LOG_DEBUG("Restart client activity timer");
    clientContext->m_clientActivityTimer.restart();

    if (!clientContext->m_init) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

    if (!clientContext->m_cameraOK){
        LOG_ERROR("Camera parameters have not been set");
        return FrameworkReturnCode::_ERROR_;
    }

    if (clientContext->m_started) {

        clientContext->m_started = false;

        if (clientContext->m_relocalizationService != nullptr){

            LOG_DEBUG("Stop the relocalization service");

            try {
                if (clientContext->m_relocalizationService->stop() != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while stopping the relocalization service");
                    return_code = FrameworkReturnCode::_ERROR_;
                }
            }  catch (const exception &e) {
                LOG_ERROR("Exception raised during remote request to the relocalization service: {}", e.what());
                return_code = FrameworkReturnCode::_ERROR_;
            }
        }
        else {
            LOG_ERROR("Relocalization service instance not created");
            return_code = FrameworkReturnCode::_ERROR_;
        }

        if (clientContext->m_relocalizationMarkersService != nullptr){

            LOG_DEBUG("Stop the relocalization markers service");

            try {
                if (clientContext->m_relocalizationMarkersService->stop() != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while stopping the relocalization markers service");
                    return_code = FrameworkReturnCode::_ERROR_;
                }
            }  catch (const exception &e) {
                LOG_ERROR("Exception raised during remote request to the relocalization markers service: {}", e.what());
                return_code = FrameworkReturnCode::_ERROR_;
            }
        }
        else {
            LOG_ERROR("Relocalization markers service instance not created");
            return_code = FrameworkReturnCode::_ERROR_;
        }

        if (clientContext->m_PipelineMode == RELOCALIZATION_AND_MAPPING){

            if (clientContext->m_mappingService != nullptr){

                LOG_DEBUG("Stop the mapping service");

                try {
                    if (clientContext->m_mappingService->stop() != FrameworkReturnCode::_SUCCESS) {
                        LOG_ERROR("Error while stopping the mapping service");
                        return_code = FrameworkReturnCode::_ERROR_;
                    }
                }  catch (const exception &e) {
                    LOG_ERROR("Exception raised during remote request to the mapping service: {}", e.what());
                    return_code = FrameworkReturnCode::_ERROR_;
                }
            }
            else {
                LOG_ERROR("Mapping service instance not created");
            }

            if ((clientContext->m_stereoMappingOK) && (clientContext->m_rectificationOK)){

                LOG_DEBUG("Stop the mapping stereo service");

                try {
                    if (clientContext->m_mappingStereoService->stop() != FrameworkReturnCode::_SUCCESS) {
                        LOG_ERROR("Error while stopping the mapping stereo service");
                        return_code = FrameworkReturnCode::_ERROR_;
                    }
                }  catch (const exception &e) {
                    LOG_ERROR("Exception raised during remote request to the mapping stereo service: {}", e.what());
                    return_code = FrameworkReturnCode::_ERROR_;
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

    return return_code;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::relocalizeProcessRequest(const string & uuid,
                                                                                            const vector<SRef<SolAR::datastructure::Image>> & images,
                                                                                            const vector<SolAR::datastructure::Transform3Df> & poses,
                                                                                            bool fixedPose,
                                                                                            const SolAR::datastructure::Transform3Df & worldTransform,
                                                                                            const chrono::system_clock::time_point & timestamp,
                                                                                            TransformStatus & transform3DStatus,
                                                                                            SolAR::datastructure::Transform3Df & transform3D,
                                                                                            float_t & confidence,
                                                                                            MappingStatus & mappingStatus)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::relocalizeProcessRequest");

    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(uuid);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", uuid);
        return FrameworkReturnCode::_ERROR_;
    }

    // Restart client activity timer
    clientContext->m_clientActivityTimer.restart();

    if (clientContext->m_started) {

        // Give 3D transformation matrix if available
        transform3DStatus = clientContext->m_T_status;
        transform3D = get3DTransformWorld(clientContext);
        confidence = clientContext->m_confidence;
        mappingStatus = clientContext->m_mappingStatus;

        // If relocalization mode: test delay since last processed request
        if (clientContext->m_PipelineMode == RELOCALIZATION_ONLY) {
            if (clientContext->m_clientRelocDelay.elapsed() < m_minRelocalizationDelay) {
                LOG_DEBUG("Delay between last relocalization request too short: request not processed");
                if (clientContext->m_T_status == NEW_3DTRANSFORM) {
                    transform3DStatus = PREVIOUS_3DTRANSFORM;
                }
                return FrameworkReturnCode::_SUCCESS;
            }
            else {
                LOG_DEBUG("Delay between last relocalization request valid: request will be processed");
                clientContext->m_clientRelocDelay.restart();
            }
        }

        LOG_DEBUG("poses[0] = {}",poses[0].matrix());

        // Check if pose is valid
        if (!poses[0].matrix().isZero()) {

            // Update cumulative distance
            if (clientContext->m_mappingStatus != BOOTSTRAP) {
                // transform exists -> already relocalized -> last pose exists
                Transform3Df lastPose;
                if (getLastPose(uuid, lastPose, DEVICE_POSE) != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Failed to get last pose");
                    return FrameworkReturnCode::_ERROR_;
                }
                LOG_DEBUG("Last pose = {}", lastPose.matrix());
                LOG_DEBUG("Current pose = {}", poses[0].matrix());
                Vector3f diffTranslation(lastPose(0, 3)-poses[0](0, 3), lastPose(1, 3)-poses[0](1, 3), lastPose(2, 3)-poses[0](2, 3));
                LOG_DEBUG("Distance between 2 last poses = {}", diffTranslation.norm());
                if (diffTranslation.norm() > m_minDiffTranslationDistance)
                    clientContext->m_cumulativeDistance = clientContext->m_cumulativeDistance + diffTranslation.norm();
            }

            // Store last pose received
            setLastPose(clientContext, poses[0]);

            // Relocalization
            if (checkNeedReloc(clientContext)){
                LOG_DEBUG("Push image and pose for relocalization task");
                if (clientContext->m_PipelineMode == RELOCALIZATION_AND_MAPPING) { 
                    if (clientContext->m_mappingStatus == BOOTSTRAP) { 
                        // keep trying reloc
                        m_dropBufferRelocalizationMarkers.push(make_tuple(uuid, images[0], poses[0]));
                        m_dropBufferRelocalization.push(make_tuple(uuid, images[0], poses[0]));
                    }
                    else {
                        // try reloc once, then wait for m_nbSecondsBetweenRelocRequest
                        m_dropBufferRelocalizationMarkers.push(make_tuple(uuid, images[0], poses[0]));
                        clientContext->m_relocTimer.restart();
                        clientContext->m_isNeedReloc = false;
                    } 
                }
                else if (clientContext->m_PipelineMode == RELOCALIZATION_ONLY){
                    // keep trying reloc   
                    m_dropBufferRelocalization.push(make_tuple(uuid, images[0], poses[0]));
                }
                else {
                    LOG_ERROR("Unsupported pipeline mode");
                    return FrameworkReturnCode::_ERROR_;
                } 
            }

            // Mapping if the pipeline mode is mapping and found 3D Transform
            if ((clientContext->m_PipelineMode == RELOCALIZATION_AND_MAPPING)
             && (clientContext->m_T_status != NO_3DTRANSFORM)) {
                LOG_DEBUG("Push image and pose for mapping task");
                m_dropBufferMapping.push({ uuid, images, poses, /* fixedpose = */ fixedPose, worldTransform });
            }
        }
    }
    else {
        if (!clientContext->m_init) {
            LOG_ERROR("Pipeline has not been initialized");
        }
        if (!clientContext->m_cameraOK){
            LOG_ERROR("Camera parameters have not been set");
        }
        if (!clientContext->m_started){
            LOG_ERROR("Pipeline has not been started");
        }
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::get3DTransformRequest(const string & uuid,
                                                                                         TransformStatus & transform3DStatus,
                                                                                         SolAR::datastructure::Transform3Df & transform3D,
                                                                                         float_t & confidence)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::get3DTransformRequest");

    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(uuid);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", uuid);
        return FrameworkReturnCode::_ERROR_;
    }

    if (clientContext->m_started) {

        // Give 3D transformation matrix if available
        transform3DStatus = clientContext->m_T_status;
        if (clientContext->m_T_status == NEW_3DTRANSFORM) {
            clientContext->m_T_status = PREVIOUS_3DTRANSFORM;
        }
        transform3D = get3DTransformWorld(clientContext);
        confidence = clientContext->m_confidence;
    }
    else {
        LOG_ERROR("Pipeline not started!");
        return FrameworkReturnCode::_ERROR_;
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::getLastPose(const string & uuid,
                                                                               SolAR::datastructure::Transform3Df & pose,
                                                                               const PoseType poseType) const
{
    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(uuid);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", uuid);
        return FrameworkReturnCode::_ERROR_;
    }

    if (!clientContext->m_started) {
        LOG_ERROR("Pipeline not started!");
        return FrameworkReturnCode::_ERROR_;
    }
    if (!clientContext->m_lastPose.matrix().isZero()) {
        std::lock_guard<std::mutex> lock(clientContext->m_mutexLastPose);
        if (poseType == DEVICE_POSE) {
            // Return last pose in device coordinate system
            pose = clientContext->m_lastPose;
        }
        else if (poseType == SOLAR_POSE) {
            // Return last pose in SolAR coordinate system
            if (clientContext->m_T_status != NO_3DTRANSFORM) {
                pose = clientContext->m_T_M_SolAR * clientContext->m_lastPose;
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

// Private

bool SolARMappingAndRelocalizationFrontendPipeline::getAndLockServices(const std::string & clientUUID)
{
    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(clientUUID);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", clientUUID);
        return false;
    }

    try {
        if (m_serviceManager != nullptr) {

            SRef<xpcf::IComponentManager> cmpMgr = xpcf::getComponentManagerInstance();

            // Try to get and lock a Relocalization Service URL for the new client

            if (m_serviceManager->getAndLockService(ServiceType::RELOCALIZATION_SERVICE, clientUUID, clientContext->m_relocalizationURL)
                    == FrameworkReturnCode::_SUCCESS) {

                LOG_DEBUG("Relocalization Service URL given by the Service Manager:{}", clientContext->m_relocalizationURL);

                // create configuration file
                createConfigurationFile(ServiceType::RELOCALIZATION_SERVICE, clientContext->m_relocalizationURL);

                if (cmpMgr->load(RELOCALIZATION_CONF_FILE.c_str()) != org::bcom::xpcf::_SUCCESS) {
                    LOG_ERROR("Failed to load properties configuration file: {}", RELOCALIZATION_CONF_FILE);
                    m_serviceManager->unlockService(ServiceType::RELOCALIZATION_SERVICE, clientUUID);
                    return false;
                }

                // Get the Relocalization Service proxy
                clientContext->m_relocalizationService = cmpMgr->resolve<api::pipeline::IRelocalizationPipeline>();

                LOG_DEBUG("A Relocalization service ({}) has been locked for the client", clientContext->m_relocalizationURL);
            }
            else {
                LOG_ERROR("No available Relocalization service");
                return false;
            }

            // Try to get and lock a Relocalization Markers Service URL for the new client
            string relocalizationMarkersURL = "";

            if (m_serviceManager->getAndLockService(ServiceType::RELOCALIZATION_MARKERS_SERVICE, clientUUID, relocalizationMarkersURL)
                    == FrameworkReturnCode::_SUCCESS) {

                LOG_DEBUG("Relocalization Markers Service URL given by the Service Manager:{}", relocalizationMarkersURL);

                // create configuration file
                createConfigurationFile(ServiceType::RELOCALIZATION_MARKERS_SERVICE, relocalizationMarkersURL);

                if (cmpMgr->load(RELOCALIZATION_MARKERS_CONF_FILE.c_str()) != org::bcom::xpcf::_SUCCESS) {
                    LOG_ERROR("Failed to load properties configuration file: {}", RELOCALIZATION_MARKERS_CONF_FILE);
                    m_serviceManager->unlockService(ServiceType::RELOCALIZATION_MARKERS_SERVICE, clientUUID);
                    m_serviceManager->unlockService(ServiceType::RELOCALIZATION_SERVICE, clientUUID);
                    return false;
                }

                // Get the Relocalization Markers Service proxy
                clientContext->m_relocalizationMarkersService = cmpMgr->resolve<api::pipeline::IRelocalizationPipeline>();

                LOG_DEBUG("A Relocalization Markers service ({}) has been locked for the client", relocalizationMarkersURL);
            }
            else {
                LOG_ERROR("No available Relocalization Markers service");
                m_serviceManager->unlockService(ServiceType::RELOCALIZATION_SERVICE, clientUUID);
                return false;
            }

            if (clientContext->m_PipelineMode == RELOCALIZATION_AND_MAPPING) {

                // Try to get and lock a Mapping Service URL for the new client
                string mappingURL = "";

                if (m_serviceManager->getAndLockService(ServiceType::MAPPING_SERVICE, clientUUID, mappingURL)
                        == FrameworkReturnCode::_SUCCESS) {

                    LOG_DEBUG("Mapping Service URL given by the Service Manager:{}", mappingURL);

                    // create configuration file
                    createConfigurationFile(ServiceType::MAPPING_SERVICE, mappingURL);

                    if (cmpMgr->load(MAPPING_CONF_FILE.c_str()) != org::bcom::xpcf::_SUCCESS) {
                        LOG_ERROR("Failed to load properties configuration file: {}", MAPPING_CONF_FILE);
                        m_serviceManager->unlockService(ServiceType::RELOCALIZATION_MARKERS_SERVICE, clientUUID);
                        m_serviceManager->unlockService(ServiceType::RELOCALIZATION_SERVICE, clientUUID);
                        m_serviceManager->unlockService(ServiceType::MAPPING_SERVICE, clientUUID);
                        return false;
                    }

                    // Get the Mapping Service proxy
                    clientContext->m_mappingService = cmpMgr->resolve<api::pipeline::IMappingPipeline>();

                    LOG_DEBUG("A Mapping service ({}) has been locked for the client", mappingURL);
                }
                else {
                    LOG_ERROR("No available Mapping service");
                    m_serviceManager->unlockService(ServiceType::RELOCALIZATION_MARKERS_SERVICE, clientUUID);
                    m_serviceManager->unlockService(ServiceType::RELOCALIZATION_SERVICE, clientUUID);
                    return false;
                }

                // Try to get and lock a Mapping Stereo Service URL for the new client
                string mappingStereoURL = "";

                if (m_serviceManager->getAndLockService(ServiceType::MAPPING_STEREO_SERVICE, clientUUID, mappingStereoURL)
                        == FrameworkReturnCode::_SUCCESS) {

                    LOG_DEBUG("Mapping Stereo Service URL given by the Service Manager:{}", mappingStereoURL);

                    // create configuration file
                    createConfigurationFile(ServiceType::MAPPING_STEREO_SERVICE, mappingStereoURL);

                    if (cmpMgr->load(MAPPING_STEREO_CONF_FILE.c_str()) != org::bcom::xpcf::_SUCCESS) {
                        LOG_ERROR("Failed to load properties configuration file: {}", MAPPING_STEREO_CONF_FILE);
                        m_serviceManager->unlockService(ServiceType::RELOCALIZATION_MARKERS_SERVICE, clientUUID);
                        m_serviceManager->unlockService(ServiceType::RELOCALIZATION_SERVICE, clientUUID);
                        m_serviceManager->unlockService(ServiceType::MAPPING_SERVICE, clientUUID);
                        m_serviceManager->unlockService(ServiceType::MAPPING_STEREO_SERVICE, clientUUID);
                        return false;
                    }

                    // Get the Mapping Stereo Service proxy
                    clientContext->m_mappingStereoService = cmpMgr->resolve<api::pipeline::IMappingPipeline>();

                    LOG_DEBUG("A Mapping Stereo service ({}) has been locked for the client", mappingStereoURL);
                }
                else {
                    LOG_WARNING("No available Stereo Mapping service => use mono camera mapping instead");
                }
            }

            return true;
        }
        else {
            LOG_ERROR("Service Manager instance not created");
            return false;
        }
    }
    catch (xpcf::Exception & e) {
        LOG_ERROR("The following exception has been caught {}", e.what());
        return false;
    }
}

void SolARMappingAndRelocalizationFrontendPipeline::createConfigurationFile(const ServiceType serviceType,
                                                                            const string & serviceURL) const
{
    LOG_DEBUG("Create service configuration file with URL: {}", serviceURL);

    // Open/create configuration file

    string filePath = "";

    if (serviceType == ServiceType::RELOCALIZATION_SERVICE)
        filePath = RELOCALIZATION_CONF_FILE;
    else if (serviceType == ServiceType::RELOCALIZATION_MARKERS_SERVICE)
        filePath = RELOCALIZATION_MARKERS_CONF_FILE;
    else if (serviceType == ServiceType::MAPPING_SERVICE)
        filePath = MAPPING_CONF_FILE;
    else if (serviceType == ServiceType::MAPPING_STEREO_SERVICE)
        filePath = MAPPING_STEREO_CONF_FILE;

    if (filePath != "")
    {
        ofstream confFile(filePath, ofstream::out);

        // Check if file was successfully opened for writing
        if (confFile.is_open())
        {
            confFile << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\" ?>" << endl;
            confFile << "<xpcf-registry autoAlias=\"true\">" << endl << endl;
            confFile << "<properties>" << endl << endl;
            confFile << "    <!-- gRPC proxy configuration-->" << endl;
            if (serviceType == ServiceType::RELOCALIZATION_SERVICE)
                confFile << "    <configure component=\"IRelocalizationPipeline_grpcProxy\">" << endl;
            else if (serviceType == ServiceType::RELOCALIZATION_MARKERS_SERVICE)
                confFile << "    <configure component=\"IRelocalizationPipeline_grpcProxy\">" << endl;
            else if (serviceType == ServiceType::MAPPING_SERVICE)
                confFile << "    <configure component=\"IMappingPipeline_grpcProxy\">" << endl;
            else if (serviceType == ServiceType::MAPPING_STEREO_SERVICE)
                confFile << "    <configure component=\"IMappingPipeline_grpcProxy\">" << endl;
            confFile << "        <property name=\"channelUrl\" access=\"rw\" type=\"string\" value=\""
                     << serviceURL << "\"/>" << endl;
            confFile << "        <property name=\"channelCredentials\" access=\"rw\" type=\"uint\" value=\"0\"/>" << endl;
            confFile << "    </configure>" << endl << endl;
            confFile << "</properties>" << endl << endl;
            confFile << "</xpcf-registry>" << endl;

            confFile.close();
        }
        else {
            LOG_ERROR("Error when creating the service configuration file");
        }
    }
    else {
        LOG_ERROR("Error when creating the service configuration file");
    }
}

SRef<ClientContext> SolARMappingAndRelocalizationFrontendPipeline::getClientContext(const string & clientUUID) const
{
    SRef<ClientContext> clientContext = nullptr;

    unique_lock<mutex> lock(m_mutexClientMap);

    auto it = m_clientsMap.find(clientUUID);

    if (it != m_clientsMap.end()) {
        clientContext = it->second;
    }
    else {
        LOG_DEBUG("No context found for client: {}", clientUUID);
    }

    return clientContext;
}

void SolARMappingAndRelocalizationFrontendPipeline::testClientsActivity()
{
    unique_lock<mutex> lock(m_mutexClientMap);

    if (m_clientsMap.size() > 0) {
        for (const auto& [k, v] : m_clientsMap) {
            if ((v != nullptr) && (v->m_clientActivityTimer.elapsed() > (m_clientActivityDelay * 1000))) {
                LOG_INFO("No activity for client: {}", k);
                lock.unlock();
                // Stop services dedicated to client
                if (v->m_started)
                    stop(k.c_str());
                // Unregister client to unlock services
                unregisterClient(k.c_str());
                lock.lock();
            }
        }
    }

    lock.unlock();

    std::this_thread::sleep_for(m_clientActivityDelay * 100ms);
}

void SolARMappingAndRelocalizationFrontendPipeline::processRelocalization()
{
    tuple<string, SRef<Image>, Transform3Df> imagePose;
    if (!m_dropBufferRelocalization.tryPop(imagePose)) {
        xpcf::DelegateTask::yield();
        return;
    }

    string clientUUID = get<0>(imagePose);
    SRef<Image> image = get<1>(imagePose);
    Transform3Df pose = get<2>(imagePose);

    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(clientUUID);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", clientUUID);
        return;
    }

    if (clientContext->m_relocalizationService == nullptr)
        return;

    // No image encoding to send to relocalization service
    image->setImageEncoding(Image::ENCODING_NONE);

    LOG_DEBUG("Send image and pose to relocalization service");

    Transform3Df new_pose;
    float confidence;

    try {
        if (clientContext->m_relocalizationService->relocalizeProcessRequest(image, new_pose, confidence) == SolAR::FrameworkReturnCode::_SUCCESS) {
            LOG_INFO("Relocalization succeeded");
            LOG_DEBUG("Client original pose: \n{}", pose.matrix());
            LOG_DEBUG("SolAR new pose: \n{}", new_pose.matrix());

            auto curTransform = new_pose * pose.inverse();

            // test reloc pose by comparing it to device pose, reject the reloc if big difference is observed
            if (clientContext->m_mappingStatus == BOOTSTRAP) {
                if (confidence <= m_thresRelocConfidence) {
                    LOG_WARNING("Reloc confidence score {} is lower than or equal to {} wait for next reloc", confidence, m_thresRelocConfidence);
                    return;
                }
                LOG_INFO("Reloc with confidence score {} is used to initialize the transform from AR runtime to SolAR", confidence);
            }

            LOG_INFO("Transformation matrix from client to SolAR:\n{}", curTransform.matrix());
            findTransformation(clientContext, curTransform);
        }
        else {
            if (clientContext->m_T_status == NEW_3DTRANSFORM)
                clientContext->m_T_status = PREVIOUS_3DTRANSFORM;
        } 
    }  catch (const exception &e) {
        LOG_ERROR("Exception raised during remote request to the relocalization service: {}", e.what());

        clientContext->m_mappingStatus = TRACKING_LOST;
        clientContext->m_T_status = NO_3DTRANSFORM;
    }
}

void SolARMappingAndRelocalizationFrontendPipeline::processRelocalizationMarkers()
{
    tuple<string, SRef<Image>, Transform3Df> imagePose;
    if (!m_dropBufferRelocalizationMarkers.tryPop(imagePose)) {
        xpcf::DelegateTask::yield();
        return;
    }

    string clientUUID = get<0>(imagePose);
    SRef<Image> image = get<1>(imagePose);
    Transform3Df pose = get<2>(imagePose);

    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(clientUUID);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", clientUUID);
        return;
    }

    if (clientContext->m_relocalizationMarkersService == nullptr)
        return;

    // No image encoding to send to relocalization service
    image->setImageEncoding(Image::ENCODING_NONE);

    LOG_DEBUG("Send image and pose to relocalization markers service");

    Transform3Df new_pose;
    float confidence;

    if (clientContext->m_relocalizationMarkersService->relocalizeProcessRequest(image, new_pose, confidence) == SolAR::FrameworkReturnCode::_SUCCESS) {
        LOG_INFO("Relocalization Markers succeeded");
        LOG_DEBUG("Client original pose: \n{}", pose.matrix());
        LOG_DEBUG("SolAR new pose: \n{}", new_pose.matrix());
        LOG_INFO("Transformation matrix from client to SolAR:\n{}", (new_pose * pose.inverse()).matrix());
        findTransformation(clientContext, new_pose * pose.inverse());
    }
}

void SolARMappingAndRelocalizationFrontendPipeline::processMapping()
{
    DropBufferMappingEntry imagePoses;

    if (!m_dropBufferMapping.tryPop(imagePoses)) {
        xpcf::DelegateTask::yield();
        return;
    }

    string clientUUID = imagePoses.uuid;
    vector<SRef<Image>> images = imagePoses.images;
    vector<Transform3Df> poses = imagePoses.poses;

    // Get context for current client
    SRef<ClientContext> clientContext = getClientContext(clientUUID);
    if (clientContext == nullptr) {
        LOG_ERROR("Unknown client with UUID: {}", clientUUID);
        return;
    }

    bool fixedPose = imagePoses.fixedPose;
    Transform3Df worldTransform = imagePoses.worldTransform;
    if (fixedPose && clientContext->m_isTransformS2WSet) { // update T_ARr_World if GT pose (GT pose is defined to be fixed, i.e. cannot change during BA)
        // here we get as input T_ARr_World we need adjust T_ARr_SolAR accordingly
        // both transforms are used to compensate for pose drift:
        // T_ARr_World*pose_ARr --> right pose in World & T_ARr_SolAR*pose_ARr --> right pose in SolAR
        auto cur_TW = get3DTransformWorld(clientContext);
        auto cur_TS = get3DTransformSolAR(clientContext);
        LOG_INFO("Mapping: receiving 2nd, 3rd, ... fixedPose current t_w:\n {} \n current t_s: \n {}", cur_TW.matrix(), cur_TS.matrix());
        set3DTransformSolAR(clientContext, cur_TS*cur_TW.inverse()*worldTransform);
        set3DTransformWorld(clientContext, worldTransform);
        auto cur_TW2 = get3DTransformWorld(clientContext);
        auto cur_TS2 = get3DTransformSolAR(clientContext);
        LOG_INFO("Mapping: after correction \n t_w: \n {}  t_s : \n {}", cur_TW2.matrix(), cur_TS2.matrix());
    }

    // No image encoding to send to mapping service
    for (auto & image : images)
        image->setImageEncoding(Image::ENCODING_NONE);

    LOG_DEBUG("Send image and pose to mapping service");
    Transform3Df updatedT_M_W;
    MappingStatus mappingStatus;
    if (fixedPose && !clientContext->m_isTransformS2WSet) {
        set3DTransformWorld(clientContext, worldTransform);
        Transform3Df curT_M_SolAR = get3DTransformSolAR(clientContext);
        LOG_INFO("Mapping: receiving 1st fixedPose current t_w:\n {} \n current t_s: \n {}", worldTransform.matrix(), curT_M_SolAR.matrix());
        if (clientContext->m_stereoMappingOK) {
            // TODO: implement stereo mapping case
            LOG_ERROR("GT pose in stereo case is not yet implemented");
        }
        else {
            if (clientContext->m_mappingService->set3DTransformSolARToWorld(worldTransform * curT_M_SolAR.inverse()) != FrameworkReturnCode::_SUCCESS) {
                LOG_ERROR("Failed to set transform solar to world in map");
                return;
            }
        }
        clientContext->m_isTransformS2WSet = true;  // set only once SolAR to World transform
    }

    // Mono or stereo images?
    Transform3Df curT_M_W = get3DTransformWorld(clientContext);
    if ((images.size() >= 2) && (clientContext->m_stereoMappingOK) && (clientContext->m_rectificationOK)) {
        LOG_DEBUG("Stereo mapping processing");
        // TODO: implement stereo mapping pipeline to take into account the case where fixedPose=True

        // Unlock mono mapping service
        if (clientContext->m_mappingService != nullptr) {
            m_serviceManager->unlockService(ServiceType::MAPPING_SERVICE, clientUUID);
            clientContext->m_mappingService = nullptr;
        }

        if (clientContext->m_mappingStereoService == nullptr)
            return;

        if (clientContext->m_mappingStereoService->mappingProcessRequest(images, poses, /* fixedPose = */ fixedPose, curT_M_W, updatedT_M_W, mappingStatus) == SolAR::FrameworkReturnCode::_SUCCESS) {
            LOG_DEBUG("Mapping stereo status: {}", mappingStatus);
            clientContext->m_mappingStatus = mappingStatus;
            if (!(updatedT_M_W * curT_M_W.inverse()).isApprox(Transform3Df::Identity())) {
                LOG_INFO("New transform found by loop closure:\n{}", updatedT_M_W.matrix());
                auto cur_TW = get3DTransformWorld(clientContext);
                auto cur_TS = get3DTransformSolAR(clientContext);
                set3DTransformSolAR(clientContext, cur_TS*cur_TW.inverse()*updatedT_M_W);
                set3DTransformWorld(clientContext, updatedT_M_W);
                clientContext->m_T_status = NEW_3DTRANSFORM;
            }
        }
        else {
            LOG_ERROR("Mapping stereo processing request failed");
        }
    }
    else {
        LOG_DEBUG("Mono mapping processing");

        // Unlock stereo mapping service
        if (clientContext->m_mappingStereoService != nullptr) {
            m_serviceManager->unlockService(ServiceType::MAPPING_STEREO_SERVICE, clientUUID);
            clientContext->m_mappingStereoService = nullptr;
        }

        if (clientContext->m_mappingService == nullptr)
            return;

        if (clientContext->m_mappingService->mappingProcessRequest(images, poses, fixedPose, curT_M_W, updatedT_M_W, mappingStatus) == SolAR::FrameworkReturnCode::_SUCCESS) {
            LOG_DEBUG("Mapping status: {}", mappingStatus);
            clientContext->m_mappingStatus = mappingStatus;
            if (!(updatedT_M_W * curT_M_W.inverse()).isApprox(Transform3Df::Identity())) {
                LOG_INFO("New transform found by loop closure:\n{}", updatedT_M_W.matrix());
                auto cur_TW = get3DTransformWorld(clientContext);
                auto cur_TS = get3DTransformSolAR(clientContext);
                set3DTransformSolAR(clientContext, cur_TS*cur_TW.inverse()*updatedT_M_W);
                set3DTransformWorld(clientContext, updatedT_M_W);
                clientContext->m_T_status = NEW_3DTRANSFORM;
            }
        }
        else {
            LOG_ERROR("Mapping processing request failed");
        }
    }
}

bool SolARMappingAndRelocalizationFrontendPipeline::findTransformation(const SRef<ClientContext> clientContext,
                                                                       Transform3Df transform)
{
    unique_lock<mutex> lock(clientContext->m_mutexFindTransform);
    clientContext->m_vector_reloc_transf_matrix.push_back(transform);
    
    if (clientContext->m_vector_reloc_transf_matrix.size() < m_nbRelocTransformMatrixRequest) 
        return false;
    
    // check if the transforms in m_vector_reloc_transf_matrix are consistent with each other 
    if (clientContext->m_mappingStatus == BOOTSTRAP && clientContext->m_vector_reloc_transf_matrix.size() >= 2) {
        auto transform0 = clientContext->m_vector_reloc_transf_matrix[0];
        for (auto i = 1; i<clientContext->m_vector_reloc_transf_matrix.size(); i++) {
            std::vector<float> translationDiff = {
                transform0(0, 3) - clientContext->m_vector_reloc_transf_matrix[i](0, 3),
                transform0(1, 3) - clientContext->m_vector_reloc_transf_matrix[i](1, 3),
                transform0(2, 3) - clientContext->m_vector_reloc_transf_matrix[i](2, 3)
            };
            if (std::any_of(translationDiff.begin(), translationDiff.end(), [this](const auto& v) {return std::abs(v)>m_poseDisparityToleranceInit;})) {
                // during bootstrap phase, if not consensus, clear and return
                clientContext->m_vector_reloc_transf_matrix.clear();
                LOG_INFO("Pose not stable");
                return false;
            }
        }
    }

    Transform3Df transform3D = transform3DAverage(clientContext->m_vector_reloc_transf_matrix);

    LOG_INFO("Mean transformation matrix from device to SolAR:\n{}", transform3D.matrix());
    if (clientContext->m_T_M_SolAR.isApprox(Transform3Df::Identity())) { // has not been initialized
        set3DTransformWorld(clientContext, transform3D); // set T_ARr_to_World
        set3DTransformSolAR(clientContext, transform3D);  // set T_ARr_to_SolAR
        clientContext->m_T_M_SolARInit = transform3D;
        LOG_INFO("m_T_M_SolARInit = {}", clientContext->m_T_M_SolARInit.matrix());
    }
    else { // here we get as input T_ARr_SolAR and we adjust T_ARr_World
        // compute distance between current mean transform and init transform  
        Vector3f dist(clientContext->m_T_M_SolARInit(0, 3)-transform3D(0, 3),
                        clientContext->m_T_M_SolARInit(1, 3)-transform3D(1, 3),
                        clientContext->m_T_M_SolARInit(2, 3)-transform3D(2, 3));
        LOG_INFO("Distance between new and init T is {} on cumu. dist. {}", dist.norm(), clientContext->m_cumulativeDistance);
        LOG_DEBUG("Pose distance = {} / cumulative distance = {} / min cumulative distance = {} / ratio = {} / cumulative distance*ration = {}",
                    dist.norm(), clientContext->m_cumulativeDistance, m_minCumulativeDistance, m_thresTranslationRatio, clientContext->m_cumulativeDistance*m_thresTranslationRatio);
        // if within min cumulative distance, identify bad transform using m_minTransformationDistance
        if (clientContext->m_cumulativeDistance <= m_minCumulativeDistance) {
            if (dist.norm() > m_minTransformationDistance) {
                clientContext->m_vector_reloc_transf_matrix.pop_back();
                clientContext->m_T_status = PREVIOUS_3DTRANSFORM;
                LOG_INFO("Reject reloc pose because distance is already {} while still within min cumulative distance {}", dist.norm(), m_minCumulativeDistance);
                return false;
            }
        }
        else { // beyond min cumulative distance 
            if ( (dist.norm() > m_minTransformationDistance)
            && (dist.norm() > clientContext->m_cumulativeDistance*m_thresTranslationRatio)) {
            clientContext->m_vector_reloc_transf_matrix.pop_back();
            clientContext->m_T_status = PREVIOUS_3DTRANSFORM;
            LOG_INFO("Reject reloc pose because distance is {} on cumulated distance {} ", dist.norm(), clientContext->m_cumulativeDistance);
            return false;
        }
        }

        Transform3Df curT_M_W = get3DTransformWorld(clientContext);
        Transform3Df curT_M_SolAR = get3DTransformSolAR(clientContext);
        set3DTransformWorld(clientContext, curT_M_W*curT_M_SolAR.inverse()*transform3D);  // adjust T_ARr_World
        set3DTransformSolAR(clientContext, transform3D);
    }
    if (clientContext->m_mappingStatus == BOOTSTRAP)
        clientContext->m_mappingStatus = MAPPING;
    clientContext->m_T_status = NEW_3DTRANSFORM;
    clientContext->m_relocTimer.restart();
    clientContext->m_isNeedReloc = false;
    // reloc success, delete the oldest transform from the list 
    clientContext->m_vector_reloc_transf_matrix.erase(clientContext->m_vector_reloc_transf_matrix.begin());
    LOG_INFO("New reloc sent to client");
    return true;
}

/// @brief check if need to relocalize
bool SolARMappingAndRelocalizationFrontendPipeline::checkNeedReloc(const SRef<ClientContext> clientContext)
{
    if (!clientContext->m_isNeedReloc
     && (clientContext->m_relocTimer.elapsed() > clientContext->m_maxTimeRequest * 1000))
        clientContext->m_isNeedReloc = true;
    return clientContext->m_isNeedReloc;
}

/// @brief get 3D transform World
Transform3Df SolARMappingAndRelocalizationFrontendPipeline::get3DTransformWorld(const SRef<ClientContext> clientContext)
{
    unique_lock<mutex> lock(clientContext->m_mutexTransformWorld);
    return clientContext->m_T_M_World;
}

/// @brief get 3D transform SolAR
Transform3Df SolARMappingAndRelocalizationFrontendPipeline::get3DTransformSolAR(const SRef<ClientContext> clientContext)
{
    std::unique_lock<std::mutex> lock(clientContext->m_mutexTransformSolAR);
    return clientContext->m_T_M_SolAR;
}

/// @brief set 3D transform World
void SolARMappingAndRelocalizationFrontendPipeline::set3DTransformWorld(const SRef<ClientContext> clientContext,
                                                                        const Transform3Df& transform3D)
{
    unique_lock<mutex> lock(clientContext->m_mutexTransformWorld);
    clientContext->m_T_M_World = transform3D;
}

/// @brief set 3D transform SolAR
void SolARMappingAndRelocalizationFrontendPipeline::set3DTransformSolAR(const SRef<ClientContext> clientContext,
                                                                        const Transform3Df& transform3D)
{
    std::unique_lock<std::mutex> lock(clientContext->m_mutexTransformSolAR);
    clientContext->m_T_M_SolAR = transform3D;
}

/// @brief set last pose
void SolARMappingAndRelocalizationFrontendPipeline::setLastPose(const SRef<ClientContext> clientContext,
                                                                const Transform3Df& lastPose)
{
    unique_lock<mutex> lock(clientContext->m_mutexLastPose);
    clientContext->m_lastPose = lastPose;
}

} // namespace RELOCALIZATION
} // namespace PIPELINES
} // namespace SolAR
