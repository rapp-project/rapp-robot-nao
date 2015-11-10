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

cv::Mat Vision::captureImage(std::string cameraId, int cameraResolution){
	cv::Mat Image;
	Image = pimpl->captureImage(cameraId, cameraResolution);
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


 std::vector< std::vector<float> > Vision::faceDetect(cv::Mat &image, std::string cameraId, int cameraResolution){
	std::vector< std::vector<float> > MatrixOfFaces;
	MatrixOfFaces = pimpl->faceDetect(image, cameraId, cameraResolution);
	return MatrixOfFaces;
}


//obiekt w API
rapp::object::QRCode3D Vision::qrCodeDetection(cv::Mat &cv_frame, cv::Mat &robotToCameraMatrix_){
	rapp::object::QRCode3D QRCodeStruct;
	QRCodeStruct = pimpl->qrCodeDetection(cv_frame, robotToCameraMatrix_);
	return QRCodeStruct;
}


} // namespace rapp
} // namespace robot

//######################################################################
//######################################################################

/*namespace rappPlatform {
namespace robot {

VisionDyn::VisionDyn(int argc, char * argv[]) {
	pimpl = new VisionDynImpl(argc, argv);
}

VisionDyn::~VisionDyn() {
	delete pimpl;
}

std::vector< std::vector<double> > Vision::faceDetect(cv::Mat &image, std::string cameraId, int cameraResolution){
	std::vector< std::vector<double> > MatrixOfFaces;
	MatrixOfFaces = pimpl->faceDetect(image, cameraId, cameraResolution);
	return MatrixOfFaces;
}

//obiekt w API
rapp::object::QRCode3D Vision::qrCodeDetection(cv::Mat &cv_frame, cv::Mat &robotToCameraMatrix_){
	QRCode3D QRCodeStruct;
	//QRCodeStruct = pimpl->qrCodeDetection(cv_frame, set_zbar, robotToCameraMatrix_);
	return QRCodeStruct;
}


} // namespace robot
} // namespace rappPlatform
*/
