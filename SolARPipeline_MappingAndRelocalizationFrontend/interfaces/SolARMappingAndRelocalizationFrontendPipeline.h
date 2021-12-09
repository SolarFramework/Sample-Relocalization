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

#ifndef SOLARMAPPINGANDRELOCALIZATIONFRONTENDPIPELINE_H
#define SOLARMAPPINGANDRELOCALIZATIONFRONTENDPIPELINE_H

#if _WIN32
#ifdef SolARPipelineMappingAndRelocalizationFrontend_API_DLLEXPORT
#define SOLARPIPELINE_MAPPINGANDRELOCALIZATIONFRONTEND_EXPORT_API __declspec(dllexport)
#else //SolARPipelineMappingAndRelocalizationFrontend_API_DLLEXPORT
#define SOLARPIPELINE_MAPPINGANDRELOCALIZATIONFRONTEND_EXPORT_API __declspec(dllimport)
#endif //SolARPipelineMappingAndRelocalizationFrontend_API_DLLEXPORT
#else //_WIN32
#define SOLARPIPELINE_MAPPINGANDRELOCALIZATIONFRONTEND_EXPORT_API
#endif //_WIN32

#include "xpcf/core/traits.h"
#include "xpcf/component/ConfigurableBase.h"
#include "xpcf/threading/BaseTask.h"
#include "xpcf/threading/DropBuffer.h"
#include "api/pipeline/IAsyncRelocalizationPipeline.h"
#include "api/pipeline/IRelocalizationPipeline.h"
#include "api/pipeline/IMappingPipeline.h"
#include "api/input/files/ITrackableLoader.h"
#include "api/solver/pose/ITrackablePose.h"

#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"
#include "core/Log.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
using namespace api;
using namespace api::pipeline;
namespace PIPELINES {
namespace RELOCALIZATION {

/**
 * @class SolARMappingAndRelocalizationFrontendPipeline
 * @brief Implementation of a mapping and an asynchronous relocalization vision pipeline
 * <TT>UUID: ec8f11a8-56b2-11ec-bf63-0242ac130002</TT>
 *
 * @SolARComponentInjectablesBegin
 * @SolARComponentInjectablesEnd
 *
 */

class SOLARPIPELINE_MAPPINGANDRELOCALIZATIONFRONTEND_EXPORT_API SolARMappingAndRelocalizationFrontendPipeline :
    public org::bcom::xpcf::ConfigurableBase,
    public api::pipeline::IAsyncRelocalizationPipeline
{
  public:

    SolARMappingAndRelocalizationFrontendPipeline();
    ~SolARMappingAndRelocalizationFrontendPipeline() override;

    /// @brief Method called when all component injections have been done
    void onInjected() override;

    void unloadComponent() override final {}

    /// @brief Initialization of the pipeline
    /// @return FrameworkReturnCode::_SUCCESS if the init succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode init() override;

    /// @brief Set the camera parameters
    /// @param[in] cameraParams: the camera parameters (its resolution and its focal)
    /// @return FrameworkReturnCode::_SUCCESS if the camera parameters are correctly set, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode setCameraParameters(const SolAR::datastructure::CameraParameters & cameraParams) override;

    /// @brief Get the camera parameters
    /// @param[out] cameraParams: the camera parameters (its resolution and its focal)
    /// @return FrameworkReturnCode::_SUCCESS if the camera parameters are correctly returned, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode getCameraParameters(SolAR::datastructure::CameraParameters & cameraParams) const override;

    /// @brief Start the pipeline
    /// @return FrameworkReturnCode::_SUCCESS if the stard succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode start() override;

    /// @brief Stop the pipeline.
    /// @return FrameworkReturnCode::_SUCCESS if the stop succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode stop() override;

    /// @brief Request the asynchronous relocalization pipeline to process a new image to calculate
    /// the corresponding 3D transformation to the SolAR coordinates system
    /// @param[in] image: the image to process
    /// @param[in] pose: the original pose in the client coordinates system
    /// @param[in] timestamp: the timestamp of the image
    /// @param[out] transform3DStatus: the status of the current 3D transformation matrix
    /// @param[out] transform3D : the current 3D transformation matrix (if available)
    /// @param[out] confidence: the confidence score of the 3D transformation matrix
    /// @return FrameworkReturnCode::_SUCCESS if the data are ready to be processed, else FrameworkReturnCode::_ERROR_
    virtual FrameworkReturnCode relocalizeProcessRequest(const SRef<SolAR::datastructure::Image> image,
                                                         const SolAR::datastructure::Transform3Df & pose,
                                                         const std::chrono::system_clock::time_point & timestamp,
                                                         TransformStatus & transform3DStatus,
                                                         SolAR::datastructure::Transform3Df & transform3D,
                                                         float_t & confidence) override;

    /// @brief Request the asynchronous relocalization pipeline to get the 3D transform to the SolAR coordinates system
    /// @param[out] transform3DStatus: the status of the current 3D transformation matrix
    /// @param[out] transform3D : the current 3D transformation matrix (if available)
    /// @param[out] confidence: the confidence score of the 3D transformation matrix
    /// @return FrameworkReturnCode::_SUCCESS if the 3D transform is available, else FrameworkReturnCode::_ERROR_
    virtual FrameworkReturnCode get3DTransformRequest(TransformStatus & transform3DStatus,
                                                      SolAR::datastructure::Transform3Df & transform3D,
                                                      float_t & confidence) const override;

  private:

    /// @brief send requests to the relocalization service
    void processRelocalization();

    /// @brief relocalization based on markers
    void processRelocalizationMarker();

  private:

    // Relocalization and mapping services
    SRef<api::pipeline::IRelocalizationPipeline>	m_relocalizationService;
    SRef<api::pipeline::IMappingPipeline>           m_mappingService;
    // Trackable objects management
    SRef<api::input::files::ITrackableLoader>       m_trackableLoader;
    SRef<api::solver::pose::ITrackablePose>         m_trackablePose;



    // Delegate task dedicated to relocalization processing
    xpcf::DelegateTask * m_relocalizationTask = nullptr;
    xpcf::DelegateTask * m_relocalizationMarkerTask = nullptr;

    // Drop buffer used by the relocalization task
    xpcf::DropBuffer<std::pair<SRef<datastructure::Image>, datastructure::Transform3Df>> m_dropBufferRelocalization;
    xpcf::DropBuffer<std::pair<SRef<datastructure::Image>, datastructure::Transform3Df>> m_dropBufferRelocalizationMarker;

    // 3D transformation matrix from client to SolAR coordinates system
    Transform3Df m_T_M_W = Transform3Df::Identity();
    TransformStatus m_T_M_W_status = NO_3DTRANSFORM;
    float_t m_confidence = 0;

    int8_t m_nb_relocalization_images; // Nb images since last relocalization
};

} // namespace RELOCALIZATION
} // namespace PIPELINES
} // namespace SolAR

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::PIPELINES::RELOCALIZATION::SolARMappingAndRelocalizationFrontendPipeline,
                            "ec8f11a8-56b2-11ec-bf63-0242ac130002",
                            "SolARMappingAndRelocalizationFrontendPipeline",
                            "SolARMappingAndRelocalizationFrontendPipeline implements api::pipeline::IAsyncRelocalizationPipeline interface");

#endif // SOLARMAPPINGANDRELOCALIZATIONFRONTENDPIPELINE_H
