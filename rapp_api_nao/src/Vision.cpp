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
	pimpl->captureImage(cameraId, cameraResolution);
}

bool Vision::setCameraParams(int cameraId, int cameraParameterId, int newValue ){
	pimpl->setCameraParams(cameraId, cameraParameterId, newValue);
}

cv::Mat Vision::getTransform(std::string chainName, int space){
	pimpl->getTransform(chainName, space);
}


} // namespace rapp
} // namespace robot

//######################################################################
//######################################################################

namespace rappPlatform {
namespace robot {

Vision::Vision(int argc, char * argv[]) {
	pimpl = new VisionImpl(argc, argv);
}

Vision::~Vision() {
	delete pimpl;
}

std::vector< std::vector<double> > Vision::faceDetect(cv::Mat image, std::string cameraId, int cameraResolution){
	pimpl->faceDetect(cv::Mat image, std::string cameraId, int cameraResolution);
}

struct Vision::QRcodeDetection Vision::qrCodeDetection(cv::Mat &cv_frame, zbar::ImageScanner &set_zbar, cv::Mat &robotToCameraMatrix_){
	pimpl->qrCodeDetection(cv::Mat &cv_frame, zbar::ImageScanner &set_zbar, cv::Mat &robotToCameraMatrix_)
}


} // namespace robot
} // namespace rappPlatform
