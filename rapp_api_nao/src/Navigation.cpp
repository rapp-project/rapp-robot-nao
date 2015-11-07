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
		nav_msgs::Path path2;
		path2.header.seq = path.header.seq;
		path2.header.frame_id = path.header.frame_id;
		path2.header.stamp.sec = path.header.stamp.sec;
		path2.header.stamp.nsec = path.header.stamp.nsec;
		for (uint32_t i=0; i < path.poses.size();i++){

			path2.poses.at(i).header.seq = path.poses.at(i).header.seq;
			path2.poses.at(i).header.frame_id = path.poses.at(i).header.frame_id;
			path2.poses.at(i).header.stamp.sec = path.poses.at(i).header.stamp.sec;
			path2.poses.at(i).header.stamp.nsec = path.poses.at(i).header.stamp.nsec;
			path2.poses.at(i).pose.position.x = path.poses.at(i).pose.position.x;
			path2.poses.at(i).pose.position.y = path.poses.at(i).pose.position.y;
			path2.poses.at(i).pose.position.z = path.poses.at(i).pose.position.z;
			path2.poses.at(i).pose.orientation.x = path.poses.at(i).pose.orientation.x;
			path2.poses.at(i).pose.orientation.y = path.poses.at(i).pose.orientation.y;
			path2.poses.at(i).pose.orientation.z = path.poses.at(i).pose.orientation.z;
			path2.poses.at(i).pose.orientation.w = path.poses.at(i).pose.orientation.w;
		}
		//path2 = path;
		status = pimpl->moveAlongPath(path2);
		return status;		
	}
	rapp::object::PoseStamped Navigation::getRobotPose(){
		geometry_msgs::PoseStamped resp;
		rapp::object::PoseStamped pose;
		resp = pimpl->getRobotPose();
		pose.header.seq = resp.header.seq;
		pose.header.frame_id = resp.header.frame_id;
		pose.header.stamp.sec = resp.header.stamp.sec;
		pose.header.stamp.nsec = resp.header.stamp.nsec;
		pose.pose.position.x = resp.pose.position.x;
		pose.pose.position.y = resp.pose.position.y;
		pose.pose.position.z = resp.pose.position.z;

		pose.pose.orientation.x = resp.pose.orientation.x;
		pose.pose.orientation.y = resp.pose.orientation.y;	
		pose.pose.orientation.z = resp.pose.orientation.z;
		pose.pose.orientation.w = resp.pose.orientation.w;
		return pose;

	}
	bool Navigation::setGlobalPose(rapp::object::Pose pose){
		geometry_msgs::Pose pose2;
		pose2.position.x = pose.position.x;
		pose2.position.y = pose.position.y;
		pose2.position.z = pose.position.z; 
		pose2.orientation.x = pose.orientation.x; 
		pose2.orientation.y = pose.orientation.y; 
		pose2.orientation.z = pose.orientation.z; 
		pose2.orientation.w = pose.orientation.w; 
		bool resp;
		resp = pimpl->setGlobalPose(pose2);
		return resp;
	}
	// rapp::objects::Path Navigation::pathPlanner_2D(rapp::objects::Pose start, rapp::objects::Pose goal, rapp::objects::OccupancyGrid map){
	// 	pimpl->pathPlanner_2D(start, goal, map);		
	// }
 //    rapp::objects::Pose Navigation::qrCodeLocalization(cv::Mat image, rapp::objects::QRcodeMap QRmap){
	// 	pimpl->qrCodeLocalization(image, QRmap);		

 //    }
} // namespace rapp
} // namespace robot

