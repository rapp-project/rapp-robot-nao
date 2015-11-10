#include <rapp/robot/Navigation.hpp>
#include <rapp/robot/Vision.hpp>


#include "NavigationImpl.hpp"
#include <rapp/objects/path/path.hpp>
#include <rapp/objects/pose/pose.hpp>
#include <rapp/objects/poseStamped/poseStamped.hpp>
namespace rapp {
namespace robot {

Navigation::Navigation(int argc, char * argv[]) {
	pimpl = new NavigationImpl(argc, argv);
}

Navigation::~Navigation() {
	delete pimpl;
}
	bool Navigation::moveTo(float x, float y, float theta){
		bool status;
		status = pimpl->moveTo( x, y, theta);		
		return status;

	}
	bool Navigation::moveVel(float x, float y, float theta){
		bool status;
		status = pimpl->moveVel(x, y,  theta);		
		return status;
	}
	// bool Navigation::moveHead(float yaw,float pitch){
	// 	pimpl->moveHead(yaw,pitch);		
	// }
	bool Navigation::moveStop(){
		bool status;
		status = pimpl->moveStop();				
		return status;
	}
	bool Navigation::moveJoint(std::vector<std::string> joint, std::vector<float> angle){
		bool status;
		status = pimpl->moveJoint(joint, angle);			
		return status;	
	}
	// bool Navigation::removeStiffness(std::string joint){
	// 	pimpl->removeStiffness(joint);		
	// }
	bool Navigation::takePredefinedPosture(std::string posture, float speed){
		bool status;
		status = pimpl->takePredefinedPosture(posture, speed);			
		return status;	
	}
	// bool Navigation::visOdom(){
	// 	pimpl->visOdom();		
	// }
	bool Navigation::lookAtPoint(float x, float y, float z){
		bool status;
		status = pimpl->lookAtPoint(x, y, z);			
		return status;	
	}
	bool Navigation::rest(std::string posture){
		bool status;
		status = pimpl->rest(posture);		
		return status;
	}
	bool Navigation::moveAlongPath(rapp::object::Path path){
		bool status;
		status = pimpl->moveAlongPath(path);
		return status;		
	}
	rapp::object::PoseStamped Navigation::getRobotPose(){
		rapp::object::PoseStamped pose;
		pose = pimpl->getRobotPose();
		return pose;

	}
	bool Navigation::setGlobalPose(rapp::object::Pose pose){

		bool status;
		status = pimpl->setGlobalPose(pose);
		return status;
	}

} // namespace rapp
} // namespace robot

namespace rappPlatform {
namespace robot {
	/*
	rapp::object::Pose pose = rappPlatform::robot::getRobotPoseFromQRcodeMap(rapp::object::Matrix matrix,rapp::object::QRcodeMap QRmap){
	rapp::object::Matrix_2D QR_in_Robot_matrix = matrix[0]
	

	}
	
	*/

	rapp::object::Pose qrCodeLocalization(rapp::object::QRcodeMap QRmap){
	/*
	rapp::object::picture image = rapp::robot::Vision::captureImage("0",3);
	rapp::object::Matrix robot_camera_transform = rapp::robot::Vision::getTransform("CameraTop",0);
	rapp::object::Matrix_2D matrix = rappPlatform::robot::qrCodeDetection(image, robot_camera_transform);	
	rapp::object::Pose pose = rappPlatform::robot::getRobotPoseFromQRcodeMap(matrix, QRmap);
	return pose
	*/
}





} // namespace robot
} // namespace rappPlatform
