/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
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

#include <boost/log/core.hpp>
#include "core/Log.h"
#include "xpcf/xpcf.h"

#include "api/pipeline/IPoseEstimationPipeline.h"
#include "api/display/IImageViewer.h"
#include "api/display/I3DOverlay.h"
#include "api/display/I3DPointsViewer.h"
#include "api/storage/IPointCloudManager.h"

namespace xpcf  = org::bcom::xpcf;

using namespace SolAR;
using namespace SolAR::api;

int main(int argc, char *argv[]){
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();
    try{
        SRef<xpcf::IComponentManager> componentMgr = xpcf::getComponentManagerInstance();
		std::string configxml = std::string("PipelineRelocalization.xml");
		if (argc == 2)
			configxml = std::string(argv[1]);
        xpcf::XPCFErrorCode errorLoad = componentMgr->load(configxml.c_str());
        if (errorLoad != xpcf::_SUCCESS)
        {
            LOG_ERROR("The file {} has an error", configxml);
			return 1;
        }

        auto pipeline = componentMgr->resolve<pipeline::IPoseEstimationPipeline>();

        if (pipeline->init(componentMgr) == FrameworkReturnCode::_SUCCESS )
        {
            auto imageViewerResult = componentMgr->resolve<display::IImageViewer>();
            auto overlay3DComponent = componentMgr->resolve<display::I3DOverlay>();
			auto viewer3D = componentMgr->resolve<display::I3DPointsViewer>();
			auto pointCloudManager = componentMgr->resolve<storage::IPointCloudManager>();
						
            // Set camera parameters
            CameraParameters camParam = pipeline->getCameraParameters();
            overlay3DComponent->setCameraParameters(camParam.intrinsic, camParam.distortion);

            unsigned char* r_imageData=new unsigned char[camParam.resolution.width * camParam.resolution.height * 3];
            SRef<Image> camImage=xpcf::utils::make_shared<Image>(r_imageData,camParam.resolution.width,camParam.resolution.height,SolAR::Image::LAYOUT_BGR,SolAR::Image::INTERLEAVED,SolAR::Image::TYPE_8U);

            Transform3Df pose;
			std::vector<Transform3Df> framePoses;
			std::vector<SRef<CloudPoint>> pointCloud;
            clock_t start, end;
            start = clock();
            int count = 0;
            if (pipeline->start(camImage->data()) == FrameworkReturnCode::_SUCCESS)
            {
				// get point cloud to display				
				pointCloudManager->getAllPoints(pointCloud);
				
                while (true)
                {
					pose = Transform3Df::Identity();
                    sink::SinkReturnCode returnCode = pipeline->update(pose);
                    if (returnCode == SinkReturnCode::_ERROR) {
						pipeline->stop();
						break;
					}
                    if (returnCode == sink::SinkReturnCode::_NOTHING)
                        continue;
                    if ((returnCode == sink::SinkReturnCode::_NEW_POSE) || (returnCode == sink::SinkReturnCode::_NEW_POSE_AND_IMAGE))
                    {
                        //LOG_INFO("pose.matrix():\n {} \n",pose.matrix())
                        overlay3DComponent->draw(pose, camImage);
						framePoses.push_back(pose);
                    }

                    if ((imageViewerResult->display(camImage) == SolAR::FrameworkReturnCode::_STOP) || 
						(viewer3D->display(pointCloud, pose, {}, framePoses) == FrameworkReturnCode::_STOP)){
                        pipeline->stop();
                        break;
                    }

                    count++;
                 }
            }
            end = clock();
            double duration = double(end - start) / CLOCKS_PER_SEC;
            printf("\n\nElasped time is %.2lf seconds.\n", duration);
            printf("Number of processed frame per second : %8.2f\n", count / duration);

			// display all relocalization camera poses
			while (true) {
				if (viewer3D->display(pointCloud, Transform3Df::Identity(), framePoses) == FrameworkReturnCode::_STOP)
					break;
			}

            delete[] r_imageData;
        }
    }
    catch (xpcf::InjectableNotFoundException e)
    {
        LOG_ERROR ("The following exception in relation to a unfound injectable has been catched: {}", e.what());
        return -1;
    }
    catch (xpcf::Exception e)
    {
        LOG_ERROR ("The following exception has been catched: {}", e.what());
        return -1;
    }

}





