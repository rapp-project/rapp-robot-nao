#include <rapp/robot/Vision.hpp>
#include "VisionImpl.hpp"

//#include <rapp/object/QRCode3D/QRCode3D.hpp>

namespace rapp {
namespace robot {

Vision::Vision(int argc, char * argv[]) {
	pimpl = new VisionImpl(argc, argv);
}

Vision::~Vision() {
	delete pimpl;
}

rapp::object::picture Vision::captureImage(std::string cameraId, int cameraResolution){
	rapp::object::picture Image = pimpl->captureImage(cameraId, cameraResolution);
	return Image;
}

bool Vision::setCameraParams(int cameraId, int cameraParameterId, int newValue ){
	bool isDone;
	isDone = pimpl->setCameraParams(cameraId, cameraParameterId, newValue);
	return isDone;
}

cv::Mat Vision::getTransform(std::string chainName, int space){
	cv::Mat Mat;
	Mat = pimpl->getTransform(chainName, space);
	return Mat;
}

} // namespace rapp
} // namespace robot

//######################################################################
//######################################################################

namespace rappPlatform {
namespace robot {

VisionDyn::VisionDyn(int argc, char * argv[]) {
	pimpl = new VisionDynImpl(argc, argv);
}

VisionDyn::~VisionDyn() {
	delete pimpl;
}

std::vector< std::vector <float> > VisionDyn::faceDetect(rapp::object::picture image, std::string cameraId, int cameraResolution){
	std::vector< std::vector<float> > MatrixOfFaces;
	MatrixOfFaces = pimpl->faceDetect(image, cameraId, cameraResolution);
	return MatrixOfFaces;
}


rapp::object::QRCode3D VisionDyn::qrCodeDetection(rapp::object::picture image, cv::Mat &robotToCameraMatrix_, float landmarkTheoreticalSize){
	rapp::object::QRCode3D QRCodeStruct;
	QRCodeStruct = pimpl->qrCodeDetection(image, robotToCameraMatrix_,landmarkTheoreticalSize);
	return QRCodeStruct;
}

} // namespace robot
} // namespace rappPlatform

