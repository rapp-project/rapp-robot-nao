/**
 * @class Navigation
 * @brief Class which defines the interface for Robot navigation capabilities (movement, localization)
 * @date 10-August-2015
 * @author Wojciech Dudek <wojciechsbox@gmail.com>
 * @note This class uses pimpl pattern to make ABI as stable as possible when deploying new library versions
 */


#include <rapp/robot/navigation/navigation.hpp>

#include "NavigationImpl.hpp"

namespace rapp {
namespace robot {

navigation::navigation(int argc, char ** argv ) {
	pimpl = new NavigationImpl(argc, argv);
}

navigation::~navigation() {
	delete pimpl;
}
	bool navigation::moveTo(float x, float y, float theta){
		bool status;
		status = pimpl->moveTo( x, y, theta);		
		return status;

	}
	bool navigation::moveVel(float x, float y, float theta){
		bool status;
		status = pimpl->moveVel(x, y,  theta);		
		return status;
	}
	// bool navigation::moveHead(float yaw,float pitch){
	// 	pimpl->moveHead(yaw,pitch);		
	// }
	bool navigation::moveStop(){
		bool status;
		status = pimpl->moveStop();				
		return status;
	}
	bool navigation::moveJoint(std::vector<std::string> joint, std::vector<float> angle, float speed){
		bool status;
		status = pimpl->moveJoint(joint, angle, speed);			
		return status;	
	}
	// bool navigation::removeStiffness(std::string joint){
	// 	pimpl->removeStiffness(joint);		
	// }
	bool navigation::takePredefinedPosture(std::string posture, float speed){
		bool status;
		status = pimpl->takePredefinedPosture(posture, speed);			
		return status;	
	}
	// bool navigation::visOdom(){
	// 	pimpl->visOdom();		
	// }
	bool navigation::lookAtPoint(float x, float y, float z){
		bool status;
		status = pimpl->lookAtPoint(x, y, z);			
		return status;	
	}
	bool navigation::rest(std::string posture){
		bool status;
		status = pimpl->rest(posture);		
		return status;
	}
	bool navigation::moveAlongPath(rapp::object::Path path){
		bool status;
		status = pimpl->moveAlongPath(path);
		return status;		
	}
	rapp::object::PoseStamped navigation::getRobotPose(){
		rapp::object::PoseStamped pose;
		pose = pimpl->getRobotPose();
		return pose;

	}
	bool navigation::setGlobalPose(rapp::object::PoseStamped rapp_pose){

		bool status;
		status = pimpl->setGlobalPose(rapp_pose);
		return status;
	}
std::vector<std::vector<float>> navigation::getTransform(std::string chainName, int space){
	std::vector<std::vector<float>> MatStruct;
	MatStruct.clear();
	MatStruct = pimpl->getTransform(chainName, space);
	return MatStruct;
}
} // namespace rapp
} // namespace robot
