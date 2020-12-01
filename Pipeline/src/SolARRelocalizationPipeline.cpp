#include "xpcf/module/ModuleFactory.h"
#include "SolARRelocalizationPipeline.h"
#include "core/Log.h"
#include "boost/log/core/core.hpp"
#include <cmath>
#define NB_PROCESS_KEYFRAMES 5
#define NB_REQUIRED_INLIERS 100

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::PIPELINES::SolARRelocalizationPipeline)

namespace SolAR {
using namespace datastructure;
using namespace api::pipeline;
namespace PIPELINES {

SolARRelocalizationPipeline::SolARRelocalizationPipeline() :ConfigurableBase(xpcf::toUUID<SolARRelocalizationPipeline>())
{
	declareInterface<api::pipeline::IPoseEstimationPipeline>(this);
	declareInjectable<input::devices::ICamera>(m_camera);
	declareInjectable<IPointCloudManager>(m_pointCloudManager);
	declareInjectable<IKeyframesManager>(m_keyframesManager);
	declareInjectable<ICovisibilityGraph>(m_covisibilityGraph);
	declareInjectable<reloc::IKeyframeRetriever>(m_kfRetriever);
	declareInjectable<solver::map::IMapper>(m_mapper);
	declareInjectable<features::IKeypointDetector>(m_keypointsDetector);
	declareInjectable<features::IDescriptorsExtractor>(m_descriptorExtractor);
	declareInjectable<api::display::I2DOverlay>(m_2DOverlay);
	declareInjectable<features::IDescriptorMatcher>(m_matcher);
	declareInjectable<solver::pose::I2D3DCorrespondencesFinder>(m_corr2D3DFinder);
	declareInjectable<api::solver::pose::I3DTransformSACFinderFrom2D3D>(m_pnpRansac);
	declareInjectable<features::IMatchesFilter>(m_matchesFilter);
	declareInjectable<image::IImageConvertor>(m_imageConvertorUnity);
	declareInjectable<sink::ISinkPoseImage>(m_sink);
	declareInjectable<source::ISourceImage>(m_source);

	m_stopFlag = false;
	m_startedOK = false;

	LOG_DEBUG(" Pipeline constructor");
}


SolARRelocalizationPipeline::~SolARRelocalizationPipeline()
{
	LOG_DEBUG(" Pipeline destructor")
}

FrameworkReturnCode SolARRelocalizationPipeline::init(SRef<xpcf::IComponentManager> xpcfComponentManager)
{
	// component creation
	try {
		// initialize components requiring the camera intrinsic and distortion parameters
		m_calibration = m_camera->getIntrinsicsParameters();
		m_distortion = m_camera->getDistortionParameters();
		m_pnpRansac->setCameraParameters(m_calibration, m_distortion);
		m_minNbInliers = m_pnpRansac->bindTo<xpcf::IConfigurable>()->getProperty("minNbInliers")->getIntegerValue();
	}
	catch (xpcf::Exception e)
	{
		LOG_ERROR("Exception catched: {}", e.what());
		return FrameworkReturnCode::_ERROR_;
	}

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationPipeline::start(void* imageDataBuffer)
{
	if (m_initOK == false)
	{
		return FrameworkReturnCode::_ERROR_;
	}
	m_stopFlag = false;

	m_sink->setImageBuffer((unsigned char*)imageDataBuffer);

	if (!m_haveToBeFlip) {
		if (m_camera->start() != FrameworkReturnCode::_SUCCESS)
		{
			LOG_ERROR("Camera cannot start")
				return FrameworkReturnCode::_ERROR_;
		}
	}

	// Load map from file
	if (m_mapper->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
		LOG_INFO("Load map done!");
	}
	else {
		LOG_ERROR("Cannot load map");
		return FrameworkReturnCode::_ERROR_;
	}
	// create and start threads
	auto cameraImagesThread = [this]() {fnCamImageCapture(); };
	auto detectionThread = [this]() {fnDetection(); };
	auto extractionThread = [this]() {fnExtraction(); };
	auto relocalizationThread = [this]() {fnRelocalization(); };

	m_taskCameraImagesCapture = new xpcf::DelegateTask(cameraImagesThread);
	m_taskDetection = new xpcf::DelegateTask(detectionThread);
	m_taskExtraction = new xpcf::DelegateTask(extractionThread);
	m_taskReloc = new xpcf::DelegateTask(relocalizationThread);

	m_taskCameraImagesCapture->start();
	m_taskDetection->start();
	m_taskExtraction->start();
	m_taskReloc->start();

	LOG_INFO("Threads have started");
	m_startedOK = true;

	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARRelocalizationPipeline::stop()
{
	m_stopFlag = true;
	m_camera->stop();

	if (m_taskCameraImagesCapture != nullptr)
		m_taskCameraImagesCapture->stop();
	if (m_taskDetection != nullptr)
		m_taskDetection->stop();
	if (m_taskExtraction != nullptr)
		m_taskExtraction->stop();
	if (m_taskReloc != nullptr)
		m_taskReloc->stop();

	if (!m_initOK)
	{
		LOG_WARNING("Try to stop a pipeline that has not been initialized");
		return FrameworkReturnCode::_ERROR_;
	}
	if (!m_startedOK)
	{
		LOG_WARNING("Try to stop a pipeline that has not been started");
		return FrameworkReturnCode::_ERROR_;
	}
	LOG_INFO("Pipeline has stopped \n");

	return FrameworkReturnCode::_SUCCESS;
}

SourceReturnCode SolARRelocalizationPipeline::loadSourceImage(void* sourceTextureHandle, int width, int height)
{
	m_haveToBeFlip = true;
	return m_source->setInputTexture((unsigned char *)sourceTextureHandle, width, height);
}

SinkReturnCode SolARRelocalizationPipeline::update(Transform3Df& pose)
{
	if (m_stopFlag)
		return SinkReturnCode::_ERROR;
	else
		return m_sink->tryGet(pose);
}

CameraParameters SolARRelocalizationPipeline::getCameraParameters()
{
	CameraParameters camParam;
	if (m_camera)
	{
		camParam = m_camera->getParameters();
	}
	return camParam;
}

void SolARRelocalizationPipeline::fnCamImageCapture() {

	SRef<Image> view;
	if (m_stopFlag || !m_initOK || !m_startedOK)
		return;
	if (m_haveToBeFlip) {
		m_source->getNextImage(view);
		m_imageConvertorUnity->convert(view, view, Image::ImageLayout::LAYOUT_RGB);
	}
	else if (m_camera->getNextImage(view) != FrameworkReturnCode::_SUCCESS) {
		m_stopFlag = true;
		return;
	}
	m_dropBufferCamImageCapture.push(view);
};

void SolARRelocalizationPipeline::fnDetection() {
	SRef<Image>  image;
	if (!m_dropBufferCamImageCapture.tryPop(image)) {
		xpcf::DelegateTask::yield();
		return;
	}
	std::vector<Keypoint> keypoints;	
	m_keypointsDetector->detect(image, keypoints);
	m_dropBufferKeypoints.push(xpcf::utils::make_shared<Frame>(keypoints, nullptr, image, Transform3Df::Identity()));
};

void SolARRelocalizationPipeline::fnExtraction()
{
	SRef<Frame> frame;
	if (!m_dropBufferKeypoints.tryPop(frame)) {
		xpcf::DelegateTask::yield();
		return;
	}
	SRef<DescriptorBuffer>	descriptors;
	m_descriptorExtractor->extract(frame->getView(), frame->getKeypoints(), descriptors);
	frame->setDescriptors(descriptors);
	m_dropBufferFrameDescriptors.push(frame);
}

bool SolARRelocalizationPipeline::fnPoseEstimation(const SRef<Frame> &frame, const SRef<Keyframe>& candidateKf, Transform3Df& pose, std::vector<Point2Df>& pts2dInliers)
{
	// feature matching to reference keyframe			
	std::vector<DescriptorMatch> matches;
	m_matcher->match(candidateKf->getDescriptors(), frame->getDescriptors(), matches);
	m_matchesFilter->filter(matches, matches, candidateKf->getKeypoints(), frame->getKeypoints());
	// find 2D-3D point correspondences
	std::vector<Point2Df> pts2d;
	std::vector<Point3Df> pts3d;
	std::vector < std::pair<uint32_t, SRef<CloudPoint>>> corres2D3D;
	std::vector<DescriptorMatch> foundMatches;
	std::vector<DescriptorMatch> remainingMatches;
	m_corr2D3DFinder->find(candidateKf, frame, matches, pts3d, pts2d, corres2D3D, foundMatches, remainingMatches);
	if (pts2d.size() < m_minNbInliers)
		return false;
	// pnp ransac
	std::vector<uint32_t> inliers;
	if (m_pnpRansac->estimate(pts2d, pts3d, inliers, pose) == FrameworkReturnCode::_SUCCESS) {
		LOG_DEBUG(" pnp inliers size: {} / {}", inliers.size(), pts3d.size());
		for (const auto& it : inliers)
			pts2dInliers.push_back(std::move(pts2d[it]));
		return true;
	}
	else
		return false;
}

void SolARRelocalizationPipeline::fnRelocalization()
{
	SRef<Frame> frame;
	if (!m_dropBufferFrameDescriptors.tryPop(frame)) {
		xpcf::DelegateTask::yield();
		return;
	}
	// keyframes retrieval
	std::vector <uint32_t> retKeyframesId;
	if (m_kfRetriever->retrieve(frame, retKeyframesId) == FrameworkReturnCode::_SUCCESS) {
		LOG_DEBUG("Number of retrieved keyframes: {}", retKeyframesId.size());
		std::vector <uint32_t> processKeyframesId;
		if (retKeyframesId.size() <= NB_PROCESS_KEYFRAMES)
			processKeyframesId.swap(retKeyframesId);
		else
			processKeyframesId.insert(processKeyframesId.begin(), retKeyframesId.begin(), retKeyframesId.begin() + NB_PROCESS_KEYFRAMES);
		Transform3Df bestPose;
		std::vector<Point2Df> bestPts2dInliers;
		for (const auto& it : processKeyframesId) {
			SRef<Keyframe> retKeyframe;
			m_keyframesManager->getKeyframe(it, retKeyframe);
			Transform3Df pose;
			std::vector<Point2Df> pts2dInliers;
			bool isFoundPose = fnPoseEstimation(frame, retKeyframe, pose, pts2dInliers);
			if (isFoundPose && (pts2dInliers.size() > bestPts2dInliers.size())) {
				bestPose = pose;
				bestPts2dInliers.swap(pts2dInliers);
			}
			if (bestPts2dInliers.size() > NB_REQUIRED_INLIERS)
				break;
		}
		if (bestPts2dInliers.size() > 0) {
			frame->setPose(bestPose);
			m_2DOverlay->drawCircles(bestPts2dInliers, frame->getView());
			m_sink->set(frame->getPose(), frame->getView());
		}
		else
			m_sink->set(frame->getView());
		LOG_DEBUG("Number of best inliers: {}", bestPts2dInliers.size());
	}
}

}//namespace PIPELINES
}//namespace SolAR
