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

rapp::object::Matrix2D Vision::getTransform(std::string chainName, int space){
	rapp::object::Matrix2D MatStruct;
	MatStruct.clear();
	MatStruct = pimpl->getTransform(chainName, space);
	return MatStruct;
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


rapp::object::QRCode3D VisionDyn::qrCodeDetection(rapp::object::picture image, std::vector<std::vector<float>> robotToCameraMatrix, float landmarkTheoreticalSize){
	rapp::object::QRCode3D QRCodeStruct;
	cv::Mat tmpMat;
	/*std::vector<float> tmpRows;
	std::vector<std::vector<float>> tmpMat;
	for(int i=0;i<4;i++){
		tmpRows.clear();
		for(int j=0;j<4;j++)
		tmpRows.push_back(robotToCameraMatrix.matrix4x4.at(0)[i][j]);
		tmpMat.push_back(tmpRows);
	}//*/
	QRCodeStruct = pimpl->qrCodeDetection(image, tmpMat,landmarkTheoreticalSize);//robotToCameraMatrix.matrix4x4.at(0)
	return QRCodeStruct;
}

} // namespace robot
} // namespace rappPlatform

