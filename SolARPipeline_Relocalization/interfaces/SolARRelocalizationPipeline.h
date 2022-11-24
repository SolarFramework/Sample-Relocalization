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

#ifndef SOLARRELOCALIZATIONPIPELINE_H
#define SOLARRELOCALIZATIONPIPELINE_H

#if _WIN32
#ifdef SolARPipelineRelocalization_API_DLLEXPORT
#define SOLARPIPELINE_RELOCALIZATION_EXPORT_API __declspec(dllexport)
#else //SolARPipeline_Relocalization_API_DLLEXPORT
#define SOLARPIPELINE_RELOCALIZATION_EXPORT_API __declspec(dllimport)
#endif //SolARPipeline_Relocalization_API_DLLEXPORT
#else //_WIN32
#define SOLARPIPELINE_RELOCALIZATION_EXPORT_API
#endif //_WIN32

#include "xpcf/core/traits.h"
#include "xpcf/component/ConfigurableBase.h"
#include "api/pipeline/IRelocalizationPipeline.h"

#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"
#include "xpcf/threading/BaseTask.h"
#include "xpcf/threading/DropBuffer.h"
#include "core/Log.h"
#include "api/input/devices/ICamera.h"
#include "api/display/IImageViewer.h"
#include "api/features/IDescriptorsExtractorFromImage.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/pose/I3DTransformSACFinderFrom2D3D.h"
#include "api/storage/IMapManager.h"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "api/image/IImageConvertor.h"
#include "api/geom/IUndistortPoints.h"
#include "api/pipeline/IMapUpdatePipeline.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
using namespace api;
using namespace api::pipeline;
using namespace api::storage;
using namespace api::reloc;
namespace PIPELINES {
namespace RELOCALIZATION {

/**
 * @class SolARRelocalizationPipeline
 * @brief Implementation of a relocalization vision pipeline
 * <TT>UUID: 890d718b-3feb-44db-a16f-1330386d5fb2</TT>
 *
 * @SolARComponentInjectablesBegin
 * @SolARComponentInjectable{SolAR::api::input::devices::ICamera}
 * @SolARComponentInjectable{SolAR::api::features::IDescriptorsExtractorFromImage}
 * @SolARComponentInjectable{SolAR::api::features::IDescriptorMatcher}
 * @SolARComponentInjectable{SolAR::api::features::IMatchesFilter}
 * @SolARComponentInjectable{SolAR::api::solver::pose::I3DTransformSACFinderFrom2D3D}
 * @SolARComponentInjectable{SolAR::api::solver::pose::IMapper}
 * @SolARComponentInjectable{SolAR::api::solver::pose::I2D3DCorrespondencesFinder}
 * @SolARComponentInjectable{SolAR::api::image::IImageConvertor}
 * @SolARComponentInjectable{SolAR::api::geom::IUndistortPoints}
 * @SolARComponentInjectablesEnd
 *
 */

class SOLARPIPELINE_RELOCALIZATION_EXPORT_API SolARRelocalizationPipeline : public org::bcom::xpcf::ConfigurableBase,
    public api::pipeline::IRelocalizationPipeline
{
public:
	SolARRelocalizationPipeline();
    ~SolARRelocalizationPipeline() override;

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

    /// @brief Request the relocalization pipeline to process a new image to calculate the corresponding pose
    /// @param[in] image: the image to process
    /// @param[out] pose: the new calculated pose
    /// @param[out] confidence: the confidence score
    /// @return FrameworkReturnCode::_SUCCESS if the processing is successful, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode relocalizeProcessRequest(const SRef<SolAR::datastructure::Image> image,
                                                 SolAR::datastructure::Transform3Df& pose, float_t & confidence) override;

	/// @brief Request to the relocalization pipeline to get the map
	/// @param[out] map the output map
	/// @return FrameworkReturnCode::_SUCCESS if the map is available, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode getMapRequest(SRef<SolAR::datastructure::Map> & map) const override;

private:

    // 2D-3D correspondences finder function
	bool fnFind2D3DCorrespondences(const SRef<Frame> &frame, const SRef<Keyframe>& candidateKf, std::vector<std::pair<uint32_t, SRef<CloudPoint>>> &corres2D3D);

private:

	// State flag of the pipeline
	bool m_initOK = false, m_cameraOK = false, m_started = false, m_isMap = false;
    int m_minNbInliers;	
	int m_nbRelocFails;

    // Camera parameters
    CameraParameters                                        m_camParams;
    uint32_t                                                m_camParamsID;

	// keyframe collection
	SRef<KeyframeCollection>								m_keyframeCollection;

	// storage components
    SRef<reloc::IKeyframeRetriever>                         m_kfRetriever;
    SRef<storage::IMapManager>								m_mapManager;
    SRef<pipeline::IMapUpdatePipeline>                      m_mapUpdatePipeline;
    SRef<features::IDescriptorsExtractorFromImage>          m_descriptorExtractor;
    SRef<features::IDescriptorMatcher>                      m_matcher;
    SRef<solver::pose::I2D3DCorrespondencesFinder>          m_corr2D3DFinder;
    SRef<api::solver::pose::I3DTransformSACFinderFrom2D3D>  m_pnpRansac;
    SRef<features::IMatchesFilter>                          m_matchesFilter;
    SRef<geom::IUndistortPoints>							m_undistortKeypoints;
};

} // namespace RELOCALIZATION
} // namespace PIPELINES
} // namespace SolAR

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::PIPELINES::RELOCALIZATION::SolARRelocalizationPipeline,
                            "890d718b-3feb-44db-a16f-1330386d5fb2",
                            "SolARRelocalizationPipeline",
                            "SolARRelocalizationPipeline implements api::pipeline::IRelocalizationPipeline interface");

#endif // SOLARRELOCALIZATIONPIPELINE_H
