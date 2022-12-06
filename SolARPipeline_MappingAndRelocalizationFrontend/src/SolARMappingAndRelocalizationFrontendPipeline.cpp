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

        LOG_DEBUG("All component injections declared");

        LOG_DEBUG("Set the asynchronous task for relocalization");

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

    delete m_relocalizationTask;
    delete m_relocalizationMarkersTask;
    delete m_mappingTask;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::registerClient(string & uuid)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::registerClient");

    boost::uuids::uuid boost_uuid = boost::uuids::random_generator()();

    uuid = boost::lexical_cast<string>(boost_uuid);

    Client_Services clientServices;
    clientServices.relocalizationService = nullptr;
    clientServices.mappingService = nullptr;
    clientServices.mappingStereoService = nullptr;

    try {
        if (m_serviceManager != nullptr) {

            SRef<xpcf::IComponentManager> cmpMgr = xpcf::getComponentManagerInstance();

            // Try to get and lock a Relocalization Service URL for the new client

            if (m_serviceManager->getAndLockService(ServiceType::RELOCALIZATION_SERVICE, uuid, m_relocalizationURL)
                    == FrameworkReturnCode::_SUCCESS) {

                LOG_DEBUG("Relocalization Service URL given by the Service Manager:{}", m_relocalizationURL);

                // create configuration file
                createConfigurationFile(ServiceType::RELOCALIZATION_SERVICE, m_relocalizationURL);

                if (cmpMgr->load(RELOCALIZATION_CONF_FILE.c_str()) != org::bcom::xpcf::_SUCCESS) {
                    LOG_ERROR("Failed to load properties configuration file: {}", RELOCALIZATION_CONF_FILE);
                    m_serviceManager->unlockService(ServiceType::RELOCALIZATION_SERVICE, uuid);
                    return FrameworkReturnCode::_ERROR_;
                }

                // Get the Relocalization Service proxy
                clientServices.relocalizationService = cmpMgr->resolve<api::pipeline::IRelocalizationPipeline>();

                LOG_DEBUG("A Relocalization service ({}) has been locked for the client", m_relocalizationURL);
            }
            else {
                LOG_ERROR("No available Relocalization service");
                return FrameworkReturnCode::_ERROR_;
            }

            // Try to get and lock a Relocalization Markers Service URL for the new client
            string relocalizationMarkersURL = "";

            if (m_serviceManager->getAndLockService(ServiceType::RELOCALIZATION_MARKERS_SERVICE, uuid, relocalizationMarkersURL)
                    == FrameworkReturnCode::_SUCCESS) {

                LOG_DEBUG("Relocalization Markers Service URL given by the Service Manager:{}", relocalizationMarkersURL);

                // create configuration file
                createConfigurationFile(ServiceType::RELOCALIZATION_MARKERS_SERVICE, relocalizationMarkersURL);

                if (cmpMgr->load(RELOCALIZATION_MARKERS_CONF_FILE.c_str()) != org::bcom::xpcf::_SUCCESS) {
                    LOG_ERROR("Failed to load properties configuration file: {}", RELOCALIZATION_MARKERS_CONF_FILE);
                    m_serviceManager->unlockService(ServiceType::RELOCALIZATION_MARKERS_SERVICE, uuid);
                    return FrameworkReturnCode::_ERROR_;
                }

                // Get the Relocalization Markers Service proxy
                clientServices.relocalizationMarkersService = cmpMgr->resolve<api::pipeline::IRelocalizationPipeline>();

                LOG_DEBUG("A Relocalization Markers service ({}) has been locked for the client", relocalizationMarkersURL);
            }
            else {
                LOG_ERROR("No available Relocalization Markers service");
                return FrameworkReturnCode::_ERROR_;
            }

            // Try to get and lock a Mapping Service URL for the new client
            string mappingURL = "";

            if (m_serviceManager->getAndLockService(ServiceType::MAPPING_SERVICE, uuid, mappingURL)
                    == FrameworkReturnCode::_SUCCESS) {

                LOG_DEBUG("Mapping Service URL given by the Service Manager:{}", mappingURL);

                // create configuration file
                createConfigurationFile(ServiceType::MAPPING_SERVICE, mappingURL);

                if (cmpMgr->load(MAPPING_CONF_FILE.c_str()) != org::bcom::xpcf::_SUCCESS) {
                    LOG_ERROR("Failed to load properties configuration file: {}", MAPPING_CONF_FILE);
                    m_serviceManager->unlockService(ServiceType::RELOCALIZATION_SERVICE, uuid);
                    m_serviceManager->unlockService(ServiceType::MAPPING_SERVICE, uuid);
                    return FrameworkReturnCode::_ERROR_;
                }

                // Get the Mapping Service proxy
                clientServices.mappingService = cmpMgr->resolve<api::pipeline::IMappingPipeline>();

                LOG_DEBUG("A Mapping service ({}) has been locked for the client", mappingURL);
            }
            else {
                LOG_ERROR("No available Mapping service");
                m_serviceManager->unlockService(ServiceType::RELOCALIZATION_SERVICE, uuid);
                return FrameworkReturnCode::_ERROR_;
            }

            // Try to get and lock a Mapping Stereo Service URL for the new client
            string mappingStereoURL = "";

            if (m_serviceManager->getAndLockService(ServiceType::MAPPING_STEREO_SERVICE, uuid, mappingStereoURL)
                    == FrameworkReturnCode::_SUCCESS) {

                LOG_DEBUG("Mapping Stereo Service URL given by the Service Manager:{}", mappingStereoURL);

                // create configuration file
                createConfigurationFile(ServiceType::MAPPING_STEREO_SERVICE, mappingStereoURL);

                if (cmpMgr->load(MAPPING_STEREO_CONF_FILE.c_str()) != org::bcom::xpcf::_SUCCESS) {
                    LOG_ERROR("Failed to load properties configuration file: {}", MAPPING_STEREO_CONF_FILE);
                    m_serviceManager->unlockService(ServiceType::RELOCALIZATION_SERVICE, uuid);
                    m_serviceManager->unlockService(ServiceType::MAPPING_SERVICE, uuid);
                    m_serviceManager->unlockService(ServiceType::MAPPING_STEREO_SERVICE, uuid);
                    return FrameworkReturnCode::_ERROR_;
                }

                // Get the Mapping Stereo Service proxy
                clientServices.mappingStereoService = cmpMgr->resolve<api::pipeline::IMappingPipeline>();

                LOG_DEBUG("A Mapping Stereo service ({}) has been locked for the client", mappingStereoURL);
            }
            else {
                LOG_WARNING("No available Stereo Mapping service => use mono camera mapping instead");
            }
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

    // Add the new client and its services to the map
    unique_lock<mutex> lock(m_mutexClientMap);
    m_clientsMap.insert(pair<string, Client_Services>(uuid, clientServices));

    LOG_DEBUG("New client registered with UUID: {}", uuid);

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::unregisterClient(const string uuid)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::unregisterClient");

    // Remove the client and its services from the map
    unique_lock<mutex> lock(m_mutexClientMap);
    auto it = m_clientsMap.find(uuid);
    if (it != m_clientsMap.end())
        m_clientsMap.erase(it);
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

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::init(const string uuid)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::init");

    // Stop services if needed
    if (m_started)
        stop();

    // Get service instances for current client
    SRef<api::pipeline::IRelocalizationPipeline> relocalizationService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IRelocalizationPipeline>(
                getServiceForClient(uuid, ServiceType::RELOCALIZATION_SERVICE));
    SRef<api::pipeline::IRelocalizationPipeline> relocalizationMarkersService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IRelocalizationPipeline>(
                getServiceForClient(uuid, ServiceType::RELOCALIZATION_MARKERS_SERVICE));
    SRef<api::pipeline::IMappingPipeline> mappingService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IMappingPipeline>(
                getServiceForClient(uuid, ServiceType::MAPPING_SERVICE));
    SRef<api::pipeline::IMappingPipeline> mappingStereoService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IMappingPipeline>(
                getServiceForClient(uuid, ServiceType::MAPPING_STEREO_SERVICE));

    if (relocalizationService != nullptr){

        LOG_DEBUG("Relocalization service URL = {}",
                 relocalizationService->bindTo<xpcf::IConfigurable>()->getProperty("channelUrl")->getStringValue());

        LOG_DEBUG("Initialize the relocalization service");

        try {
            if (relocalizationService->init() != FrameworkReturnCode::_SUCCESS) {
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

    if (relocalizationMarkersService != nullptr){

        LOG_DEBUG("Relocalization markers service URL = {}",
                 relocalizationMarkersService->bindTo<xpcf::IConfigurable>()->getProperty("channelUrl")->getStringValue());

        LOG_DEBUG("Initialize the relocalization markers service");

        try {
            if (relocalizationMarkersService->init() != FrameworkReturnCode::_SUCCESS) {
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

    if (m_PipelineMode == RELOCALIZATION_AND_MAPPING){

        if (mappingService != nullptr){

            LOG_DEBUG("Mapping service URL = {}",
                     mappingService->bindTo<xpcf::IConfigurable>()->getProperty("channelUrl")->getStringValue());

            LOG_DEBUG("Initialize the mapping service");

            try {
                if (mappingService->init(m_relocalizationURL) != FrameworkReturnCode::_SUCCESS) {
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

        if (mappingStereoService != nullptr){

            LOG_DEBUG("Mapping Stereo service URL = {}",
                     mappingStereoService->bindTo<xpcf::IConfigurable>()->getProperty("channelUrl")->getStringValue());

            LOG_DEBUG("Initialize the mapping stereo service");

            try {
                if (mappingStereoService->init(m_relocalizationURL) != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while initializing the mapping stereo service");
                }
                else {
                    m_stereoMappingOK = true;
                }
            }  catch (const exception &e) {
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

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::init(const string uuid, PipelineMode pipelineMode)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::init(PipelineMode)");

    m_PipelineMode = pipelineMode;

    return init(uuid);
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::getProcessingMode(const string uuid,
                                                                                     PipelineMode & pipelineMode) const
{
    pipelineMode = m_PipelineMode;

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::setCameraParameters(const string uuid, const CameraParameters & cameraParams)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::setCameraParameters");

    if (!m_init) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

    // Get service instances for current client
    SRef<api::pipeline::IRelocalizationPipeline> relocalizationService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IRelocalizationPipeline>(
                getServiceForClient(uuid, ServiceType::RELOCALIZATION_SERVICE));
    SRef<api::pipeline::IRelocalizationPipeline> relocalizationMarkersService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IRelocalizationPipeline>(
                getServiceForClient(uuid, ServiceType::RELOCALIZATION_MARKERS_SERVICE));
    SRef<api::pipeline::IMappingPipeline> mappingService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IMappingPipeline>(
                getServiceForClient(uuid, ServiceType::MAPPING_SERVICE));

    if (relocalizationService != nullptr){

        LOG_DEBUG("Set camera parameters for the relocalization service");

        try {
            if (relocalizationService->setCameraParameters(cameraParams) != FrameworkReturnCode::_SUCCESS) {
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

    if (relocalizationMarkersService != nullptr){

        LOG_DEBUG("Set camera parameters for the relocalization markers service");

        try {
            if (relocalizationMarkersService->setCameraParameters(cameraParams) != FrameworkReturnCode::_SUCCESS) {
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

    if (m_PipelineMode == RELOCALIZATION_AND_MAPPING){

        if (mappingService != nullptr){

            LOG_DEBUG("Set camera parameters for the mapping service");

            try {
                if (mappingService->setCameraParameters(cameraParams) != FrameworkReturnCode::_SUCCESS) {
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

    m_cameraOK = true;

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::setCameraParameters(const string uuid,
                                                                                       const SolAR::datastructure::CameraParameters & cameraParams1,
                                                                                       const SolAR::datastructure::CameraParameters & cameraParams2)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::setCameraParameters(stereo)");

    if (!m_init) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

    // Get service instances for current client
    SRef<api::pipeline::IMappingPipeline> mappingStereoService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IMappingPipeline>(
                getServiceForClient(uuid, ServiceType::MAPPING_STEREO_SERVICE));

    if (m_PipelineMode == RELOCALIZATION_AND_MAPPING){

        if (m_stereoMappingOK){

            LOG_DEBUG("Set camera parameters for the mapping stereo service");

            try {
                if (mappingStereoService->setCameraParameters(cameraParams1, cameraParams2) != FrameworkReturnCode::_SUCCESS) {
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

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::setRectificationParameters(const string uuid,
                                                                                              const SolAR::datastructure::RectificationParameters & rectCam1,
                                                                                              const SolAR::datastructure::RectificationParameters & rectCam2)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::setRectificationParameters");

    if (!m_init) {
        LOG_ERROR("Pipeline has not been initialized");
        return FrameworkReturnCode::_ERROR_;
    }

    // Get service instances for current client
    SRef<api::pipeline::IMappingPipeline> mappingStereoService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IMappingPipeline>(
                getServiceForClient(uuid, ServiceType::MAPPING_STEREO_SERVICE));

    if (m_PipelineMode == RELOCALIZATION_AND_MAPPING){

        if (m_stereoMappingOK){

            LOG_DEBUG("Set rectification parameters for the mapping stereo service");

            try {
                if (mappingStereoService->setRectificationParameters(rectCam1, rectCam2) != FrameworkReturnCode::_SUCCESS) {
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

    m_rectificationOK = true;

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::getCameraParameters(const string uuid,
                                                                                       CameraParameters & cameraParams) const
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::getCameraParameters");

    if (!m_cameraOK){
        LOG_ERROR("Camera parameters have not been set");
        return FrameworkReturnCode::_ERROR_;
    }

    // Get service instances for current client
    SRef<api::pipeline::IRelocalizationPipeline> relocalizationService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IRelocalizationPipeline>(
                getServiceForClient(uuid, ServiceType::RELOCALIZATION_SERVICE));

    if (relocalizationService != nullptr){

        LOG_DEBUG("Get camera parameters from the relocalization service");

        try {
            if (relocalizationService->getCameraParameters(cameraParams) != FrameworkReturnCode::_SUCCESS) {
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

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::start(const string uuid)
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

    // Get service instances for current client
    SRef<api::pipeline::IRelocalizationPipeline> relocalizationService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IRelocalizationPipeline>(
                getServiceForClient(uuid, ServiceType::RELOCALIZATION_SERVICE));
    SRef<api::pipeline::IRelocalizationPipeline> relocalizationMarkersService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IRelocalizationPipeline>(
                getServiceForClient(uuid, ServiceType::RELOCALIZATION_MARKERS_SERVICE));
    SRef<api::pipeline::IMappingPipeline> mappingService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IMappingPipeline>(
                getServiceForClient(uuid, ServiceType::MAPPING_SERVICE));
    SRef<api::pipeline::IMappingPipeline> mappingStereoService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IMappingPipeline>(
                getServiceForClient(uuid, ServiceType::MAPPING_STEREO_SERVICE));

    if (!m_started) {

        LOG_DEBUG("Initialize instance attributes");

        // Initialize class members
        set3DTransform(Transform3Df::Identity());
        m_T_M_W_status = NO_3DTRANSFORM;
        m_confidence = 1;
        m_mappingStatus = BOOTSTRAP;
        m_isNeedReloc = true;
        m_vector_reloc_transf_matrix.clear();
        setLastPose(Transform3Df(Maths::Matrix4f::Zero()));
        if (m_PipelineMode == RELOCALIZATION_ONLY)
            m_maxTimeRequest = 0;
        else
            m_maxTimeRequest = m_nbSecondsBetweenRelocRequest;

        LOG_DEBUG("Empty buffers");

        tuple<string, SRef<Image>, Transform3Df> imagePose;
        m_dropBufferRelocalization.tryPop(imagePose);
        m_dropBufferRelocalizationMarkers.tryPop(imagePose);
        tuple<string, vector<SRef<Image>>, vector<Transform3Df>> imagePoses;
        m_dropBufferMapping.tryPop(imagePoses);
/*
        m_dropBufferRelocalization.clear();
        m_dropBufferRelocalizationMarkers.clear();
        m_dropBufferMapping.clear();
*/
        if (relocalizationService != nullptr){

            LOG_DEBUG("Start the relocalization service");

            try {
                if (relocalizationService->start() != FrameworkReturnCode::_SUCCESS) {
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

        if (relocalizationMarkersService != nullptr){

            LOG_DEBUG("Start the relocalization markers service");

            try {
                if (relocalizationMarkersService->start() != FrameworkReturnCode::_SUCCESS) {
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

        if (m_PipelineMode == RELOCALIZATION_AND_MAPPING){

            if (mappingService != nullptr){

                LOG_DEBUG("Start the mapping service");

                try {
                    if (mappingService->start() != FrameworkReturnCode::_SUCCESS) {
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

            if ((m_stereoMappingOK) && (m_rectificationOK)){

                LOG_DEBUG("Start the mapping stereo service");

                try {
                    if (mappingStereoService->start() != FrameworkReturnCode::_SUCCESS) {
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

        if (!m_tasksStarted) {
            LOG_DEBUG("Start relocalization task");
            m_relocalizationTask->start();
            m_relocalizationMarkersTask->start();

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

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::stop(const string uuid)
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

    // Get service instances for current client
    SRef<api::pipeline::IRelocalizationPipeline> relocalizationService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IRelocalizationPipeline>(
                getServiceForClient(uuid, ServiceType::RELOCALIZATION_SERVICE));
    SRef<api::pipeline::IRelocalizationPipeline> relocalizationMarkersService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IRelocalizationPipeline>(
                getServiceForClient(uuid, ServiceType::RELOCALIZATION_MARKERS_SERVICE));
    SRef<api::pipeline::IMappingPipeline> mappingService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IMappingPipeline>(
                getServiceForClient(uuid, ServiceType::MAPPING_SERVICE));
    SRef<api::pipeline::IMappingPipeline> mappingStereoService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IMappingPipeline>(
                getServiceForClient(uuid, ServiceType::MAPPING_STEREO_SERVICE));

    if (m_started) {

        m_started = false;

        if (m_tasksStarted) {
            LOG_DEBUG("Stop relocalization task");
            m_relocalizationTask->stop();
            m_relocalizationMarkersTask->stop();

            LOG_DEBUG("Stop mapping task");
            m_mappingTask->stop();

            m_tasksStarted = false;
        }

        if (relocalizationService != nullptr){

            LOG_DEBUG("Stop the relocalization service");

            try {
                if (relocalizationService->stop() != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while stopping the relocalization service");
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

        if (relocalizationMarkersService != nullptr){

            LOG_DEBUG("Stop the relocalization markers service");

            try {
                if (relocalizationMarkersService->stop() != FrameworkReturnCode::_SUCCESS) {
                    LOG_ERROR("Error while stopping the relocalization markers service");
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

        if (m_PipelineMode == RELOCALIZATION_AND_MAPPING){

            if (mappingService != nullptr){

                LOG_DEBUG("Stop the mapping service");

                try {
                    if (mappingService->stop() != FrameworkReturnCode::_SUCCESS) {
                        LOG_ERROR("Error while stopping the mapping service");
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

            if ((m_stereoMappingOK) && (m_rectificationOK)){

                LOG_DEBUG("Stop the mapping stereo service");

                try {
                    if (mappingStereoService->stop() != FrameworkReturnCode::_SUCCESS) {
                        LOG_ERROR("Error while stopping the mapping stereo service");
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
    }
    else {
        LOG_INFO("Pipeline already stopped");
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::relocalizeProcessRequest(const string uuid,
                                                                                            const vector<SRef<SolAR::datastructure::Image>> & images,
                                                                                            const vector<SolAR::datastructure::Transform3Df> & poses,
                                                                                            const chrono::system_clock::time_point & timestamp,
                                                                                            TransformStatus & transform3DStatus,
                                                                                            SolAR::datastructure::Transform3Df & transform3D,
                                                                                            float_t & confidence,
                                                                                            MappingStatus & mappingStatus)
{
    LOG_DEBUG("SolARMappingAndRelocalizationFrontendPipeline::relocalizeProcessRequest");

    if (m_started) {

        // Check if pose is valid
        if (!poses[0].matrix().isZero()) {

            // Store last pose received
            setLastPose(poses[0]);

            // Give 3D transformation matrix if available
            transform3DStatus = m_T_M_W_status;
            transform3D = get3DTransform();
            confidence = m_confidence;
            mappingStatus = m_mappingStatus;

            // Relocalization
            if (checkNeedReloc()){
                LOG_DEBUG("Push image and pose for relocalization task");
                m_dropBufferRelocalization.push(make_tuple(uuid, images[0], poses[0]));
                m_dropBufferRelocalizationMarkers.push(make_tuple(uuid, images[0], poses[0]));
            }

            // Mapping if the pipeline mode is mapping and found 3D Transform
            if ((m_PipelineMode == RELOCALIZATION_AND_MAPPING) && (m_T_M_W_status != NO_3DTRANSFORM)) {
                LOG_DEBUG("Push image and pose for mapping task");
                m_dropBufferMapping.push(make_tuple(uuid, images, poses));
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

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::get3DTransformRequest(const string uuid,
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

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::getLastPose(const string uuid,
                                                                               SolAR::datastructure::Transform3Df & pose,
                                                                               const PoseType poseType) const
{
    if (!m_started) {
        LOG_ERROR("Pipeline not started!");
        return FrameworkReturnCode::_ERROR_;
    }
    if (!m_lastPose.matrix().isZero()) {
        unique_lock<mutex> lock(m_mutexLastPose);
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

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::getMapRequest(const string uuid,
                                                                                 SRef<SolAR::datastructure::Map> & map) const
{
    if (m_mapupdateService != nullptr) {
        return m_mapupdateService->getMapRequest(map);
    }
    else {
        return FrameworkReturnCode::_ERROR_;
    }
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::resetMap(const string uuid) const
{
    if (m_mapupdateService != nullptr) {
        return m_mapupdateService->resetMap();
    }
    else {
        return FrameworkReturnCode::_ERROR_;
    }
}

FrameworkReturnCode SolARMappingAndRelocalizationFrontendPipeline::getPointCloudRequest(const string uuid,
                                                                                        SRef<SolAR::datastructure::PointCloud> & pointCloud) const
{
    if (m_mapupdateService != nullptr) {
        return m_mapupdateService->getPointCloudRequest(pointCloud);
    }
    else {
        return FrameworkReturnCode::_ERROR_;
    }
}

// Private

void SolARMappingAndRelocalizationFrontendPipeline::createConfigurationFile(const ServiceType serviceType,
                                                                            const string serviceURL) const
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

SRef<api::pipeline::IPipeline> SolARMappingAndRelocalizationFrontendPipeline::getServiceForClient(
                                                            const string clientUUID,
                                                            const ServiceType serviceType) const
{
    SRef<api::pipeline::IPipeline> serviceInstance = nullptr;

    unique_lock<mutex> lock(m_mutexClientMap);

    auto it = m_clientsMap.find(clientUUID);

    if (it != m_clientsMap.end()) {
        switch(serviceType) {
            case ServiceType::RELOCALIZATION_SERVICE:
                serviceInstance = it->second.relocalizationService;
                break;
            case ServiceType::RELOCALIZATION_MARKERS_SERVICE:
                serviceInstance = it->second.relocalizationMarkersService;
                break;
            case ServiceType::MAPPING_SERVICE:
                serviceInstance = it->second.mappingService;
                break;
            case ServiceType::MAPPING_STEREO_SERVICE:
                serviceInstance = it->second.mappingStereoService;
                break;
        }
    }
    else {
        LOG_DEBUG("No service instances found for client: {}", clientUUID);
    }

    return serviceInstance;
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

    // Get service instances for current client
    SRef<api::pipeline::IRelocalizationPipeline> relocalizationService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IRelocalizationPipeline>(
                getServiceForClient(clientUUID, ServiceType::RELOCALIZATION_SERVICE));

    if (relocalizationService == nullptr)
        return;

    // No image encoding to send to relocalization service
    image->setImageEncoding(Image::ENCODING_NONE);

    LOG_DEBUG("Send image and pose to relocalization service");

    Transform3Df new_pose;
    float confidence;

    try {
        if (relocalizationService->relocalizeProcessRequest(image, new_pose, confidence) == SolAR::FrameworkReturnCode::_SUCCESS) {
            LOG_INFO("Relocalization succeeded");
            LOG_DEBUG("Client original pose: \n{}", pose.matrix());
            LOG_DEBUG("SolAR new pose: \n{}", new_pose.matrix());
            LOG_INFO("Transformation matrix from client to SolAR:\n{}", (new_pose * pose.inverse()).matrix());
            findTransformation(new_pose * pose.inverse());
        }
    }  catch (const exception &e) {
        LOG_ERROR("Exception raised during remote request to the relocalization service: {}", e.what());

        m_mappingStatus = TRACKING_LOST;
        m_T_M_W_status = NO_3DTRANSFORM;
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

    // Get service instances for current client
    SRef<api::pipeline::IRelocalizationPipeline> relocalizationMarkersService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IRelocalizationPipeline>(
                getServiceForClient(clientUUID, ServiceType::RELOCALIZATION_MARKERS_SERVICE));

    if (relocalizationMarkersService == nullptr)
        return;

    // No image encoding to send to relocalization service
    image->setImageEncoding(Image::ENCODING_NONE);

    LOG_DEBUG("Send image and pose to relocalization markers service");

    Transform3Df new_pose;
    float confidence;

    if (relocalizationMarkersService->relocalizeProcessRequest(image, new_pose, confidence) == SolAR::FrameworkReturnCode::_SUCCESS) {
        LOG_INFO("Relocalization Markers succeeded");
        LOG_DEBUG("Client original pose: \n{}", pose.matrix());
        LOG_DEBUG("SolAR new pose: \n{}", new_pose.matrix());
        LOG_INFO("Transformation matrix from client to SolAR:\n{}", (new_pose * pose.inverse()).matrix());
        findTransformation(new_pose * pose.inverse());
    }
}

void SolARMappingAndRelocalizationFrontendPipeline::processMapping()
{
    tuple<string, vector<SRef<Image>>, vector<Transform3Df>> imagePoses;

    if (!m_dropBufferMapping.tryPop(imagePoses)) {
        xpcf::DelegateTask::yield();
        return;
    }

    string clientUUID = get<0>(imagePoses);
    vector<SRef<Image>> images = get<1>(imagePoses);
    vector<Transform3Df> poses = get<2>(imagePoses);

    // Get service instances for current client
    SRef<api::pipeline::IMappingPipeline> mappingService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IMappingPipeline>(
                getServiceForClient(clientUUID, ServiceType::MAPPING_SERVICE));
    SRef<api::pipeline::IMappingPipeline> mappingStereoService =
            xpcf::utils::dynamic_pointer_cast<api::pipeline::IMappingPipeline>(
                getServiceForClient(clientUUID, ServiceType::MAPPING_STEREO_SERVICE));

    // No image encoding to send to mapping service
    for (auto & image : images)
        image->setImageEncoding(Image::ENCODING_NONE);

    LOG_DEBUG("Send image and pose to mapping service");
    Transform3Df updatedT_M_W;
    MappingStatus mappingStatus;
    Transform3Df curT_M_W = get3DTransform();

    // Mono or stereo images?
    if ((images.size() >= 2) && (m_stereoMappingOK) && (m_rectificationOK)) {
        LOG_DEBUG("Stereo mapping processing");

        if (mappingStereoService == nullptr)
            return;

        if (mappingStereoService->mappingProcessRequest(images, poses, curT_M_W, updatedT_M_W, mappingStatus) == SolAR::FrameworkReturnCode::_SUCCESS) {
            LOG_DEBUG("Mapping stereo status: {}", mappingStatus);
            m_mappingStatus = mappingStatus;
            if (!(updatedT_M_W * curT_M_W.inverse()).isApprox(Transform3Df::Identity())) {
                LOG_INFO("New transform found by loop closure:\n{}", updatedT_M_W.matrix());
                set3DTransform(updatedT_M_W);
                m_T_M_W_status = NEW_3DTRANSFORM;
            }
        }
        else {
            LOG_ERROR("Mapping stereo processing request failed");
        }
    }
    else {
        LOG_DEBUG("Mono mapping processing");

        if (mappingService == nullptr)
            return;

        if (mappingService->mappingProcessRequest(images, poses, curT_M_W, updatedT_M_W, mappingStatus) == SolAR::FrameworkReturnCode::_SUCCESS) {
            LOG_DEBUG("Mapping status: {}", mappingStatus);
            m_mappingStatus = mappingStatus;
            if (!(updatedT_M_W * curT_M_W.inverse()).isApprox(Transform3Df::Identity())) {
                LOG_INFO("New transform found by loop closure:\n{}", updatedT_M_W.matrix());
                set3DTransform(updatedT_M_W);
                m_T_M_W_status = NEW_3DTRANSFORM;
            }
        }
        else {
            LOG_ERROR("Mapping processing request failed");
        }
    }
}

void SolARMappingAndRelocalizationFrontendPipeline::findTransformation(Transform3Df transform)
{
    unique_lock<mutex> lock(m_mutexFindTransform);
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
        if (m_mappingStatus == BOOTSTRAP)
            m_mappingStatus = MAPPING;
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
    unique_lock<mutex> lock(m_mutexTransform);
    return m_T_M_W;
}

/// @brief set 3D transform
void SolARMappingAndRelocalizationFrontendPipeline::set3DTransform(const Transform3Df& transform3D)
{
    unique_lock<mutex> lock(m_mutexClientMap);
    m_T_M_W = transform3D;
}

/// @brief set last pose
void SolARMappingAndRelocalizationFrontendPipeline::setLastPose(const Transform3Df& lastPose)
{
    unique_lock<mutex> lock(m_mutexLastPose);
    m_lastPose = lastPose;
}

} // namespace RELOCALIZATION
} // namespace PIPELINES
} // namespace SolAR
