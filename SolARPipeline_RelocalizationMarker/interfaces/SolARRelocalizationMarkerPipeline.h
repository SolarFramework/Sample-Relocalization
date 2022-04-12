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

#ifndef SOLARRELOCALIZATIONMARKERPIPELINE_H
#define SOLARRELOCALIZATIONMARKERPIPELINE_H

#if _WIN32
#ifdef SolARPipelineRelocalizationMarker_API_DLLEXPORT
#define SOLARPIPELINE_RELOCALIZATIONMARKER_EXPORT_API __declspec(dllexport)
#else //SolARPipeline_RelocalizationMarker_API_DLLEXPORT
#define SOLARPIPELINE_RELOCALIZATIONMARKER_EXPORT_API __declspec(dllimport)
#endif //SolARPipeline_RelocalizationMarker_API_DLLEXPORT
#else //_WIN32
#define SOLARPIPELINE_RELOCALIZATIONMARKER_EXPORT_API
#endif //_WIN32

#include "xpcf/core/traits.h"
#include "xpcf/component/ConfigurableBase.h"
#include "api/pipeline/IRelocalizationPipeline.h"

#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"
#include "core/Log.h"
#include "api/input/files/IWorldGraphLoader.h"
#include "api/features/I2DTrackablesDetector.h"
#include "api/geom/IProject.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"

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
 * @class SolARRelocalizationMarkerPipeline
 * @brief Implementation of a relocalization pipeline based on multi markers (fiducial or QR code)
 * <TT>UUID: c61aeaf1-2126-4a85-8d4b-5358ae6a32d0</TT>
 *
 * @SolARComponentInjectablesBegin
 * @SolARComponentInjectable{SolAR::api::input::files::IWorldGraphLoader}
 * @SolARComponentInjectable{SolAR::api::features::I2DTrackablesDetector}
 * @SolARComponentInjectable{SolAR::api::geom::IProject}
 * @SolARComponentInjectable{SolAR::api::solver::pose::I3DTransformFinderFrom2D3D}
 * @SolARComponentInjectablesEnd
 *
 */

class SOLARPIPELINE_RELOCALIZATIONMARKER_EXPORT_API SolARRelocalizationMarkerPipeline : public org::bcom::xpcf::ConfigurableBase,
    public api::pipeline::IRelocalizationPipeline
{
public:
	SolARRelocalizationMarkerPipeline();
    ~SolARRelocalizationMarkerPipeline() override;

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
                                                 SolAR::datastructure::Transform3Df& pose,
                                                 float_t & confidence) override;

    /// @brief Request to the relocalization pipeline to get the map
    /// @param[out] map the output map
    /// @return FrameworkReturnCode::_SUCCESS if the map is available, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode getMapRequest(SRef<SolAR::datastructure::Map> & map) const override {return FrameworkReturnCode::_NOT_IMPLEMENTED; };

private:
    /// @brief Fiducial markers detection
    void fiducialMarkersDetection(const SRef<SolAR::datastructure::Image> image,
                                  std::vector<std::vector<SolAR::datastructure::Point2Df>> & corners);

    /// @brief QR codes detection
    void qrCodesDetection(const SRef<SolAR::datastructure::Image> image,
                          std::vector<std::vector<SolAR::datastructure::Point2Df>> & corners);

private:
    // Camera parameters
    CamCalibration                                              m_calibration;
    CamDistortion                                               m_distortion;
    bool                                                        m_cameraOK = false;
    bool                                                        m_initOK = false;
    float                                                       m_maxReprojError = 2.f;
    int                                                         m_nbFiducialMarkers;
    int                                                         m_nbQRCodes;
    std::vector<std::vector<SolAR::datastructure::Point3Df>>	m_QRCodes3DPoints;
    std::vector<std::vector<SolAR::datastructure::Point3Df>>	m_fiducials3DPoints;
	// Components
    SRef<input::files::IWorldGraphLoader>                       m_worldGraphLoader;
    SRef<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>	m_pnp;
    SRef<SolAR::api::features::I2DTrackablesDetector>           m_QRCodesDetector;
    SRef<SolAR::api::features::I2DTrackablesDetector>           m_fiducialMarkersDetector;
    SRef<SolAR::api::geom::IProject>							m_projector;
};

} // namespace RELOCALIZATION
} // namespace PIPELINES
} // namespace SolAR

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::PIPELINES::RELOCALIZATION::SolARRelocalizationMarkerPipeline,
                            "c61aeaf1-2126-4a85-8d4b-5358ae6a32d0",
                            "SolARRelocalizationMarkerPipeline",
                            "SolARRelocalizationMarkerPipeline implements api::pipeline::IRelocalizationPipeline interface");

#endif // SOLARRELOCALIZATIONMARKERPIPELINE_H
