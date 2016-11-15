#include <rapp-robots-api/vision/vision.hpp>
#include "VisionImpl.hpp"



#include <boost/filesystem.hpp>
#include <boost/property_tree/json_parser.hpp>

// Short alias for this namespace
namespace pt = boost::property_tree;
namespace fs = boost::filesystem;

std::string expand_user(std::string path) {
	if (not path.empty() and path[0] == '~') {
		assert(path.size() == 1 or path[1] == '/');  // or other error handling
		char const* home = getenv("HOME");
		if (home or (home = getenv("USERPROFILE"))) {
			path.replace(0, 1, home);
		}
		else {
			char const *hdrive = getenv("HOMEDRIVE");
			char const *hpath = getenv("HOMEPATH");
			assert(hdrive);  // or other error handling
			assert(hpath);
			path.replace(0, 1, std::string(hdrive) + hpath);
		}
	}
	return path;
}

struct camera_info {
	// camera matrix
	std::vector<float> K;

	// distortion coeffs
	std::vector<float> D;

	// projection matrix
	std::vector<float> P;
};

// read an array from json ptree
template <typename T>
std::vector<T> as_vector(pt::ptree const& pt, pt::ptree::key_type const& key) {
	std::vector<T> r;
	for (auto& item : pt.get_child(key))
		r.push_back(item.second.get_value<T>());
	return r;
}




namespace rapp {
namespace robot {

vision::vision(int argc, char * argv[]) {
	pimpl = new VisionImpl(argc, argv);
}

vision::~vision() {
	delete pimpl;
}

rapp::object::picture::Ptr vision::capture_image (int camera_id, int camera_resolution, const std::string & encoding) {
	rapp::object::picture::Ptr image = pimpl->captureImage(camera_id, camera_resolution, encoding);
	return image;
}

bool vision::set_camera_param(int camera_id, int camera_parameter_id, int new_value){
	bool isDone;
	isDone = pimpl->setCameraParam(camera_id,camera_parameter_id,new_value);
	return isDone;
}

std::map<int, bool> vision::set_camera_params(int camera_id, const std::map<int, int> & params){
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
std::vector< std::vector <float> > vision::face_detect(rapp::object::picture image, int camera_id, int camera_resolution){
	std::vector< std::vector<float> > MatrixOfFaces;
	MatrixOfFaces = pimpl->faceDetect(image, camera_id, camera_resolution);
	return MatrixOfFaces;
}


rapp::object::qr_code_3d vision::qr_code_detection(rapp::object::picture::Ptr image, std::vector<std::vector<float>> robotToCameraMatrix, double camera_matrix[][3], float landmarkTheoreticalSize){
	rapp::object::qr_code_3d QRCodeStruct;
	try{
		QRCodeStruct = pimpl->qrCodeDetection(image, robotToCameraMatrix, camera_matrix, landmarkTheoreticalSize);//robotToCameraMatrix.matrix4x4.at(0)
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

vision::camera_info vision::load_camera_info(int camera_id) {
	vision::camera_info cam;
	std::string path = expand_user("~/.config/rapp_data/cam/") + std::to_string(camera_id) + ".json";
	if ( !boost::filesystem::exists( path ) )
	{
		std::cout << "Can't find calibration info for camera " << camera_id << ". Loading defaults." << std::endl;
		cam.K = { 1138.12,       0, 658,
		                0, 1156.84, 484,
			        0,       0,   1};
		cam.D = {0, 0, 0, 0, 0};
		cam.P = cam.K;
	} else {
		pt::ptree tree;
		pt::read_json(path, tree);
		cam.K = as_vector<float>(tree, "K");
		cam.D = as_vector<float>(tree, "D");
		cam.P = as_vector<float>(tree, "P");
	}

	return cam;

}

// Add a list
template<typename T>
pt::ptree put_vector(const std::vector<T> vec) {
	pt::ptree ret_node;
	for (auto &val: vec)
	{
		// Create an unnamed node containing the value
		pt::ptree tmp_node;
		tmp_node.put("", val);
	
		// Add this node to the list.
		ret_node.push_back(std::make_pair("", tmp_node));
	}
	return ret_node;
}

void vision::save_camera_info(int camera_id, vision::camera_info info) {
	std::string path = expand_user("~/.config/rapp_data/cam/") + std::to_string(camera_id) + ".json";
	// create a backup if the file exists
	if ( fs::exists( path ) ) {
		std::string bak_path = expand_user("~/.config/rapp_data/cam/") + std::to_string(camera_id) + ".bak";
		fs::copy_file(path, bak_path, fs::copy_option::overwrite_if_exists);
	}

	pt::ptree tree;
	tree.add_child("K", put_vector(info.K));
	tree.add_child("D", put_vector(info.D));
	tree.add_child("P", put_vector(info.P));
	pt::write_json(std::cout, tree);
	pt::write_json(path, tree);
}

} // namespace robot
} // namespace rapp

