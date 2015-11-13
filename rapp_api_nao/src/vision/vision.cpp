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

} // namespace robot
} // namespace rapp

