#include <rapp/robot/Navigation.hpp>
#include "NavigationImpl.hpp"

#include <rapp/robot/Navigation.hpp>
#include <rapp/objects/Path/Path.hpp>
#include <rapp/objects/Pose/Pose.hpp>
#include <rapp/objects/PoseStamped/PoseStamped.hpp>
namespace rapp {
namespace robot {

Navigation::Navigation(int argc, char * argv[]) {
	pimpl = new NavigationImpl(argc, argv);
}

Navigation::~Navigation() {
	delete pimpl;
}
	bool Navigation::moveTo(float x, float y, float theta){
		pimpl->moveTo( x, y, theta);

	}
	bool Navigation::moveVel(float x, float y, float theta){
		pimpl->moveVel(x, y,  theta);
	}
	// bool Navigation::moveHead(float yaw,float pitch){
	// 	pimpl->moveHead(yaw,pitch);		
	// }
	bool Navigation::moveStop(){
		pimpl->moveStop();		
	}
	bool Navigation::moveJoint(std::vector<std::string> joint, std::vector<float> angle){
		pimpl->moveJoint(joint, angle);		
	}
	// bool Navigation::removeStiffness(std::string joint){
	// 	pimpl->removeStiffness(joint);		
	// }
	bool Navigation::takePredefinedPosture(std::string posture){
		pimpl->takePredefinedPosture(posture);		
	}
	// bool Navigation::visOdom(){
	// 	pimpl->visOdom();		
	// }
	bool Navigation::lookAtPoint(float x, float y, float z){
		pimpl->lookAtPoint(x, y, z);		
	}
	bool Navigation::rest(std::string posture){
		pimpl->rest(posture);		
	}
	bool Navigation::moveAlongPath(rapp::object::Path path){
		pimpl->moveAlongPath(path);		
	}
	rapp::object::PoseStamped Navigation::getRobotPose(){
		pimpl->getRobotPose();		
	}
	bool Navigation::setGlobalPose(rapp::object::Pose pose){
		pimpl->setGlobalPose(pose);		
	}
	// rapp::objects::Path Navigation::pathPlanner_2D(rapp::objects::Pose start, rapp::objects::Pose goal, rapp::objects::OccupancyGrid map){
	// 	pimpl->pathPlanner_2D(start, goal, map);		
	// }
 //    rapp::objects::Pose Navigation::qrCodeLocalization(cv::Mat image, rapp::objects::QRcodeMap QRmap){
	// 	pimpl->qrCodeLocalization(image, QRmap);		

 //    }
} // namespace rapp
} // namespace robot

