#include <rapp/robot/Vision.hpp>
#include "VisionImpl.hpp"

namespace rapp {
namespace robot {

Vision::Vision(int argc, char * argv[]) {
	pimpl = new VisionImpl(argc, argv);
}

Vision::~Vision() {
	delete pimpl;
}

cv::Mat Vision::captureImage(std::string cameraId, int cameraResolution){
	pimpl->captureImage(cameraID, cameraResolution);
}

bool Vision::setCameraParams(int cameraId, int cameraParameterId, int newValue ){
	pimpl->setCameraParams(cameraID, cameraParameterId, newValue);
}

cv::Mat Vision::getTransform(std::string chainName, int space){
	pimpl->getTransform(chainName, space);
}


} // namespace rapp
} // namespace robot

