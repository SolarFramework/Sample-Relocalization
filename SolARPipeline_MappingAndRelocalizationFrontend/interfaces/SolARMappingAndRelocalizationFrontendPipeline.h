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
#include "api/pipeline/IMapUpdatePipeline.h"

#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"
#include "core/Log.h"
#include "core/Timer.h"

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

    /// @brief Init the pipeline and specify the mode for the pipeline processing
    /// @param[in] pipelineMode: mode to use for pipeline processing
    /// @return FrameworkReturnCode::_SUCCESS if the mode is correctly initialized, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode init(PipelineMode pipelineMode) override;

    /// @brief Set the camera parameters
    /// @param[in] cameraParams: the camera parameters (its resolution and its focal)
    /// @return FrameworkReturnCode::_SUCCESS if the camera parameters are correctly set, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode setCameraParameters(const SolAR::datastructure::CameraParameters & cameraParams) override;

    /// @brief Set the rectification parameters (use for stereo camera)
    /// @param[in] rectCam1 the rectification parameters of the first camera
    /// @param[in] rectCam2 the rectification parameters of the second camera
    /// @return FrameworkReturnCode::_SUCCESS if the rectification parameters are correctly set, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode setRectificationParameters(const SolAR::datastructure::RectificationParameters & rectCam1,
                                                   const SolAR::datastructure::RectificationParameters & rectCam2) override;

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
    /// @param[in] images the images to process
    /// @param[in] poses the poses associated to images in the client coordinates system
    /// @param[in] timestamp the timestamp of the image
    /// @param[out] transform3DStatus the status of the current 3D transformation matrix
    /// @param[out] transform3D the current 3D transformation matrix (if available)
    /// @param[out] confidence the confidence score of the 3D transformation matrix
    /// @return FrameworkReturnCode::_SUCCESS if the data are ready to be processed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode relocalizeProcessRequest(const std::vector<SRef<SolAR::datastructure::Image>> & images,
                                                 const std::vector<SolAR::datastructure::Transform3Df> & poses,
                                                 const std::chrono::system_clock::time_point & timestamp,
                                                 TransformStatus & transform3DStatus,
                                                 SolAR::datastructure::Transform3Df & transform3D,
                                                 float_t & confidence) override;

    /// @brief Request the asynchronous relocalization pipeline to get the 3D transform offset
    /// between the device coordinate system and the SolAR coordinate system
    /// @param[out] transform3DStatus the status of the current 3D transformation matrix
    /// @param[out] transform3D the current 3D transformation matrix (if available)
    /// @param[out] confidence the confidence score of the 3D transformation matrix
    /// @return FrameworkReturnCode::_SUCCESS if the 3D transform is available, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode get3DTransformRequest(TransformStatus & transform3DStatus,
                                              SolAR::datastructure::Transform3Df & transform3D,
                                              float_t & confidence) override;

    /// @brief Return the last pose processed by the pipeline
    /// @param[out] pose the last pose if available
    /// @param[in] poseType the type of the requested pose
    ///            - in the SolAR coordinate system (by default)
    ///            - in the device coordinate system
    /// @return FrameworkReturnCode::_SUCCESS if the last pose is available, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode getLastPose(SolAR::datastructure::Transform3Df & pose,
                                    const PoseType poseType = SOLAR_POSE) const override;

    /// @brief Reset the map stored by the map update pipeline
    /// @return FrameworkReturnCode::_SUCCESS if the map is correctly reset, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode resetMap() const override;

  private:

    /// @brief send requests to the relocalization service
    void processRelocalization();

    /// @brief relocalization based on markers
    void processRelocalizationMarker();

    /// @brief send requests to the mapping service
    void processMapping();

	/// @brief find transformation matrix
	void findTransformation(Transform3Df transform);

    /// @brief check if need to relocalize
    bool checkNeedReloc();

    /// @brief get 3D transform
    Transform3Df get3DTransform();

    /// @brief set 3D transform
    void set3DTransform(const Transform3Df& transform3D);

    /// @brief set last pose
    void setLastPose(const Transform3Df& lastPose);

  private:

    // Relocalization, mapping and map update services
    SRef<api::pipeline::IRelocalizationPipeline>	m_relocalizationService;
    SRef<api::pipeline::IRelocalizationPipeline>	m_relocalizationMarkerService;
    SRef<api::pipeline::IMappingPipeline>           m_mappingService;
    SRef<api::pipeline::IMapUpdatePipeline>         m_mapupdateService;

    bool m_init = false;            // Indicate if initialization has been made
    bool m_cameraOK = false;        // Indicate if camera parameters has been set
    bool m_started = false;         // Indicate if pipeline il started
    bool m_tasksStarted = false;    // Indicate if tasks are started

    // Delegate tasks dedicated to relocalization and mapping processing
    xpcf::DelegateTask * m_relocalizationTask = nullptr;
    xpcf::DelegateTask * m_relocalizationMarkerTask = nullptr;
    xpcf::DelegateTask * m_mappingTask = nullptr;

    // Drop buffer used by the relocalization task
    xpcf::DropBuffer<std::pair<SRef<datastructure::Image>, datastructure::Transform3Df>> m_dropBufferRelocalization;
    xpcf::DropBuffer<std::pair<SRef<datastructure::Image>, datastructure::Transform3Df>> m_dropBufferRelocalizationMarker;
    xpcf::DropBuffer<std::pair<std::vector<SRef<datastructure::Image>>, std::vector<datastructure::Transform3Df>>> m_dropBufferMapping;

    // 3D transformation matrix from client to SolAR coordinates system
    SolAR::datastructure::Transform3Df  m_T_M_W;
    std::mutex                          m_mutexTransform;
    std::atomic<TransformStatus>        m_T_M_W_status;
    float_t m_confidence = 0;

    int m_nbRelocTransformMatrixRequest = 3;
    int m_maxTimeRequest;
    int m_nbSecondsBetweenRelocRequest = 30;
    std::atomic_bool m_isNeedReloc;
    Timer m_relocTimer;

    // Vector of 3D transformation matrix given by Relocalization service
    std::vector<SolAR::datastructure::Transform3Df> m_vector_reloc_transf_matrix;

    // Last pose received
    SolAR::datastructure::Transform3Df  m_lastPose;
    mutable std::mutex                  m_mutexLastPose;

    std::mutex                          m_mutexFindTransform;
};

} // namespace RELOCALIZATION
} // namespace PIPELINES
} // namespace SolAR

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::PIPELINES::RELOCALIZATION::SolARMappingAndRelocalizationFrontendPipeline,
                            "ec8f11a8-56b2-11ec-bf63-0242ac130002",
                            "SolARMappingAndRelocalizationFrontendPipeline",
                            "SolARMappingAndRelocalizationFrontendPipeline implements api::pipeline::IAsyncRelocalizationPipeline interface");

#endif // SOLARMAPPINGANDRELOCALIZATIONFRONTENDPIPELINE_H
