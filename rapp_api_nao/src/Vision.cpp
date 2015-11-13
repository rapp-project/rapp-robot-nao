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

rapp::object::picture Vision::captureImage(int camera_id, int camera_resolution, const std::string & encoding){
	rapp::object::picture Image = pimpl->captureImage(camera_id, camera_resolution, encoding);
	return Image;
}

bool Vision::setCameraParam(int camera_id, int camera_parameter_id, int new_value){
	bool isDone;
	isDone = pimpl->setCameraParam(camera_id,camera_parameter_id,new_value);
	return isDone;
}

std::vector<uint8_t> Vision::setCameraParams(int camera_id, std::vector<uint32_t> camera_parameter_ids, std::vector<uint32_t> new_values){
	std::vector<uint8_t> isDone;
	isDone = pimpl->setCameraParams(camera_id,camera_parameter_ids, new_values);
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

std::vector< std::vector <float> > VisionDyn::faceDetect(rapp::object::picture image, int camera_id, int camera_resolution){
	std::vector< std::vector<float> > MatrixOfFaces;
	MatrixOfFaces = pimpl->faceDetect(image, camera_id, camera_resolution);
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

