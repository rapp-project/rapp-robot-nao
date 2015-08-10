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
		pimpl->moveTo(float x, float y, float theta);

	}
	void Navigation::moveVel(float x, float y, float theta){
		pimpl->moveVel(float x, float y, float theta);
	}
	void Navigation::moveHead(float yaw,float pitch){
		pimpl->moveHead(float yaw,float pitch);		
	}
	void Navigation::moveStop(){
		pimpl->moveStop();		
	}
	void Navigation::moveJoint(std::string joint, float angle){
		pimpl->moveJoint(std::string joint, float angle);		
	}
	void Navigation::removeStiffness(std::string joint){
		pimpl->removeStiffness(std::string joint);		
	}
	void Navigation::takePredefinedPosture(std::string pose){
		pimpl->takePredefinedPosture(std::string pose);		
	}
	void Navigation::visOdom(){
		pimpl->visOdom();		
	}
	void Navigation::lookAtPoint(float x, float y, float z){
		pimpl->lookAtPoint(float x, float y, float z);		
	}

} // namespace rapp
} // namespace robot

