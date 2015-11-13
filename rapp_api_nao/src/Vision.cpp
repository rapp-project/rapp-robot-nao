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
	try{
		QRCodeStruct = pimpl->qrCodeDetection(image, robotToCameraMatrix,landmarkTheoreticalSize);//robotToCameraMatrix.matrix4x4.at(0)
	}
	catch(const std::runtime_error& re)
	{
		// speciffic handling for runtime_error
		std::cerr << "Runtime error: " << re.what() << std::endl;
	}catch(const std::exception& ex)
	{
		// speciffic handling for all exceptions extending std::exception, except
		// std::runtime_error which is handled explicitly
		std::cerr << "Error occurred: " << ex.what() << std::endl;
	}catch(...){
		std::cerr << "Unknown failure occured. Possible memory corruption" << std::endl;
	}
	return QRCodeStruct;
}

} // namespace robot
} // namespace rappPlatform

