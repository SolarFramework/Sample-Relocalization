#ifndef SOLARRELOCALIZATIONPIPELINE_H
#define SOLARRELOCALIZATIONPIPELINE_H

#if _WIN32
#ifdef SolARRelocalizationPipeline_API_DLLEXPORT
#define SOLARRELOCALIZATIONPIPELINE_EXPORT_API __declspec(dllexport)
#else //SOLARRELOCALIZATIONPIPELINE_API_DLLEXPORT
#define SOLARRELOCALIZATIONPIPELINE_EXPORT_API __declspec(dllimport)
#endif //SOLARRELOCALIZATIONPIPELINE_API_DLLEXPORT
#else //_WIN32
#define SOLARRELOCALIZATIONPIPELINE_EXPORT_API
#endif //_WIN32

#include "xpcf/core/traits.h"
#include "xpcf/component/ConfigurableBase.h"
#include "api/pipeline/IPoseEstimationPipeline.h"

#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"
#include "xpcf/threading/BaseTask.h"
#include "xpcf/threading/DropBuffer.h"
#include "core/Log.h"
#include "api/input/devices/ICamera.h"
#include "api/display/IImageViewer.h"
#include "api/display/I2DOverlay.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/pose/I3DTransformSACFinderFrom2D3D.h"
#include "api/solver/map/IMapper.h"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "api/image/IImageConvertor.h"

#ifdef USE_OPENGL
#include "api/sink/ISinkPoseTextureBuffer.h"
#else
#include "api/sink/ISinkPoseImage.h"
#endif
#include "api/source/ISourceImage.h"

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
using namespace api;
using namespace api::sink;
using namespace api::pipeline;
namespace PIPELINES {

class SOLARRELOCALIZATIONPIPELINE_EXPORT_API SolARRelocalizationPipeline : public org::bcom::xpcf::ConfigurableBase,
	public api::pipeline::IPoseEstimationPipeline
{
public:
	SolARRelocalizationPipeline();
	~SolARRelocalizationPipeline();

	//// @brief Initialization of the pipeline
	/// Initialize the pipeline by providing a reference to the component manager loaded by the PipelineManager.
	/// @param[in] componentManager a shared reference to the component manager which has loaded the components and configuration in the pipleine manager
	FrameworkReturnCode init(SRef<xpcf::IComponentManager> xpcfComponentManager) override;

	/// @brief Provide the camera parameters
	/// @return the camera parameters (its resolution and its focal)
	CameraParameters getCameraParameters() override;

	/// @brief Starts the pipeline and provides a texture buffer which will be updated when required.
	/// @param[in] textureHandle a pointer to the texture buffer which will be updated at each call of the update method.

	/// @brief Start the pipeline
	/// @return FrameworkReturnCode::_ERROR_ by default as the pipeline needs to be construct with an imageDataBuffer as parameter
	FrameworkReturnCode start() override { return FrameworkReturnCode::_ERROR_; }

#ifdef USE_OPENGL
	FrameworkReturnCode start(void* textureHandle) override;
#else
	FrameworkReturnCode start(void* imageDataBuffer) override;
#endif

	/// @brief Stop the pipeline.
	FrameworkReturnCode stop() override;

	/// @brief update the pipeline
	/// Get the new pose and update the texture buffer with the image that has to be displayed
	SinkReturnCode update(Transform3Df& pose) override;

	SourceReturnCode loadSourceImage(void* sourceTextureHandle, int width, int height) override;

	void unloadComponent() override final;

private:
	// Pose estimation function
	bool fnPoseEstimation(const SRef<Frame> &frame, const SRef<Keyframe>& candidateKf, Transform3Df& pose, std::vector<Point2Df>& pts2dInliers);

	// Camera image capture task
	void fnCamImageCapture();

	// Keypoint detection task
	void fnDetection();

	// Feature extraction task
	void fnExtraction();

	// Relocalization task
	void fnRelocalization();
private:

	// State flag of the pipeline
	bool m_stopFlag = false, m_initOK = true, m_startedOK = false, m_haveToBeFlip;
	int m_minNbInliers;
	CamCalibration                                      m_calibration;
	CamDistortion                                       m_distortion;

	// storage components
	SRef<IPointCloudManager>							m_pointCloudManager;
	SRef<IKeyframesManager>								m_keyframesManager;
	SRef<ICovisibilityGraph>							m_covisibilityGraph;
	SRef<reloc::IKeyframeRetriever>						m_kfRetriever;
	SRef<solver::map::IMapper>							m_mapper;
	SRef<input::devices::ICamera>						m_camera;
	SRef<image::IImageConvertor>						m_imageConvertorUnity;
	SRef<features::IKeypointDetector>					m_keypointsDetector;
	SRef<features::IDescriptorsExtractor>				m_descriptorExtractor;
	SRef<api::display::I2DOverlay>						m_2DOverlay;
	SRef<features::IDescriptorMatcher>					m_matcher;
	SRef<solver::pose::I2D3DCorrespondencesFinder>		m_corr2D3DFinder;
	SRef<api::solver::pose::I3DTransformSACFinderFrom2D3D>m_pnpRansac;
	SRef<features::IMatchesFilter>						m_matchesFilter;
	SRef<sink::ISinkPoseImage>							m_sink;  
	SRef<source::ISourceImage>							m_source;	

	// buffers
	xpcf::DropBuffer<SRef<Image>>						m_dropBufferCamImageCapture;
	xpcf::DropBuffer<SRef<Frame>>						m_dropBufferKeypoints;
	xpcf::DropBuffer<SRef<Frame>>						m_dropBufferFrameDescriptors;

	// tasks
	xpcf::DelegateTask*									m_taskCameraImagesCapture;
	xpcf::DelegateTask*									m_taskDetection;
	xpcf::DelegateTask*									m_taskExtraction;
	xpcf::DelegateTask*									m_taskReloc;

};

}//namespace PIPELINES
}//namespace SolAR

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::PIPELINES::SolARRelocalizationPipeline,
	"890d718b-3feb-44db-a16f-1330386d5fb2",
	"SolARRelocalizationPipeline",
	"Relocalization pipeline");
#endif // SOLARRELOCALIZATIONPIPELINE_H
