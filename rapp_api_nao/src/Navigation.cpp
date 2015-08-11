#include <rapp/robot/Navigation.hpp>
#include "NavigationImpl.hpp"

namespace rapp {
namespace robot {

Navigation::Navigation(int argc, char * argv[]) {
	pimpl = new NavigationImpl(argc, argv);
}

Navigation::~Navigation() {
	delete pimpl;
}
	void Navigation::moveTo(float x, float y, float theta){
		pimpl->moveTo( x, y, theta);

	}
	void Navigation::moveVel(float x, float y, float theta){
		pimpl->moveVel(x, y,  theta);
	}
	void Navigation::moveHead(float yaw,float pitch){
		pimpl->moveHead(yaw,pitch);		
	}
	void Navigation::moveStop(){
		pimpl->moveStop();		
	}
	void Navigation::moveJoint(std::string joint, float angle){
		pimpl->moveJoint(joint, angle);		
	}
	void Navigation::removeStiffness(std::string joint){
		pimpl->removeStiffness(joint);		
	}
	void Navigation::takePredefinedPosture(std::string pose){
		pimpl->takePredefinedPosture(pose);		
	}
	void Navigation::visOdom(){
		pimpl->visOdom();		
	}
	void Navigation::lookAtPoint(float x, float y, float z){
		pimpl->lookAtPoint(x, y, z);		
	}

} // namespace rapp
} // namespace robot

