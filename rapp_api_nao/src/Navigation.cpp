/**
 * @class Navigation
 * @brief Class which defines the interface for Robot navigation capabilities (movement, localization)
 * @date 10-August-2015
 * @author Wojciech Dudek <wojciechsbox@gmail.com>
 * @note This class uses pimpl pattern to make ABI as stable as possible when deploying new library versions
 */


#include <rapp/robot/Navigation.hpp>
#include <rapp/robot/Vision.hpp>


#include "NavigationImpl.hpp"
#include <rapp/objects/path/path.hpp>
#include <rapp/objects/pose/pose.hpp>
#include <rapp/objects/poseStamped/poseStamped.hpp>
namespace rapp {
namespace robot {

Navigation::Navigation(int argc, char ** argv ) {
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
	bool Navigation::moveJoint(std::vector<std::string> joint, std::vector<float> angle, float speed){
		bool status;
		status = pimpl->moveJoint(joint, angle, speed);			
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
	bool Navigation::setGlobalPose(rapp::object::PoseStamped rapp_pose){

		bool status;
		status = pimpl->setGlobalPose(rapp_pose);
		return status;
	}

} // namespace rapp
} // namespace robot
