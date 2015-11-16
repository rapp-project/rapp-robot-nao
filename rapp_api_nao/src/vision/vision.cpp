#include <rapp/robot/vision/vision.hpp>
#include "VisionImpl.hpp"


namespace rapp {
namespace robot {

vision::vision(int argc, char * argv[]) {
	pimpl = new VisionImpl(argc, argv);
}

vision::~vision() {
	delete pimpl;
}

rapp::object::picture::Ptr vision::captureImage (int camera_id, int camera_resolution, const std::string & encoding) {
	rapp::object::picture::Ptr image = pimpl->captureImage(camera_id, camera_resolution, encoding);
	return image;
}

bool vision::setCameraParam(int camera_id, int camera_parameter_id, int new_value){
	bool isDone;
	isDone = pimpl->setCameraParam(camera_id,camera_parameter_id,new_value);
	return isDone;
}

std::map<int, bool> vision::setCameraParams(int camera_id, const std::map<int, int> & params){
	std::map<int, bool> isDone;
	std::vector<unsigned int> par;
	std::vector<unsigned int> val;
	for(auto p: params) {
		par.push_back(p.first);
		val.push_back(p.second);
	}
	isDone = pimpl->setCameraParams(camera_id, par, val);
	return isDone;
}

//######################################################################
std::vector< std::vector <float> > vision::faceDetect(rapp::object::picture image, int camera_id, int camera_resolution){
	std::vector< std::vector<float> > MatrixOfFaces;
	MatrixOfFaces = pimpl->faceDetect(image, camera_id, camera_resolution);
	return MatrixOfFaces;
}


rapp::object::QRCode3D vision::qrCodeDetection(rapp::object::picture image, std::vector<std::vector<float>> robotToCameraMatrix, float landmarkTheoreticalSize, double & camera_matrix[3][3]){
	rapp::object::QRCode3D QRCodeStruct;
	try{
		QRCodeStruct = pimpl->qrCodeDetection(image, robotToCameraMatrix,landmarkTheoreticalSize, camera_matrix);//robotToCameraMatrix.matrix4x4.at(0)
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
} // namespace rapp

