//#####################
// written by Wojciech Dudek
//#####################
#include "NavigationImpl.hpp"

//#include "string.h"
namespace rapp {
namespace robot {

NavigationImpl::NavigationImpl(int argc,char **argv){
		ros::init(argc, argv,"NavigationImpl_library");
		n = new ros::NodeHandle();

		}
NavigationImpl::~NavigationImpl() {
}
	bool NavigationImpl::moveTo(float x, float y, float theta){	

		client_moveTo = n->serviceClient<rapp_ros_naoqi_wrappings::MoveTo>("rapp_moveTo");
		  rapp_ros_naoqi_wrappings::MoveTo srv;
		  srv.request.x = x;
		  srv.request.y = y;
		  srv.request.theta = theta;
		  if (client_moveTo.call(srv))
		  {
	  	  	return srv.response.status;
	  	  	ROS_INFO_STREAM("Service ended with status:\n" <<srv.response.status);

			}
		  else
		  {

	  	  	return false;
		    ROS_ERROR("Failed to call service moveTo"); 
		  }

	}

	bool NavigationImpl::moveVel(float x, float y, float theta){	

		client_moveVel = n->serviceClient<rapp_ros_naoqi_wrappings::MoveVel>("rapp_moveVel");

		  rapp_ros_naoqi_wrappings::MoveVel srv;
		  srv.request.velocity_x = x;
		  srv.request.velocity_y = y;
		  srv.request.velocity_theta = theta;
		  if (client_moveVel.call(srv))
		  {
		    return srv.response.status;
	  	  	ROS_INFO("Nao moved ");
		  }
		  else
		  {
	  	  	return false;
		    ROS_ERROR("Failed to call service moveVel"); 
		  }
	}

// HeadYaw	 HeadPitch Min	HeadPitch Max	 	HeadYaw	  HeadPitch Min  	HeadPitch Max
// 	         (degrees)						      		(radians)
// -119.52		-25.73			18.91			-2.086017	-0.449073		0.330041
// -87.49		-18.91			11.46			-1.526988	-0.330041		0.200015
// -62.45		-24.64			17.19			-1.089958	-0.430049		0.300022
// -51.74		-27.50			18.91			-0.903033	-0.479965		0.330041
// -43.32		-31.40			21.20			-0.756077	-0.548033		0.370010
// -27.85		-38.50			24.18			-0.486074	-0.671951		0.422021
//   0.0		-38.50			29.51			 0.000000	-0.671951		0.515047
//  27.85		-38.50			24.18			 0.486074	-0.671951		0.422021
//  43.32		-31.40			21.20			 0.756077	-0.548033		0.370010
//  51.74		-27.50			18.91			 0.903033	-0.479965		0.330041
//  62.45		-24.64			17.19			 1.089958	-0.430049		0.300022
//  87.49		-18.91			11.46			 1.526988	-0.330041		0.200015
//  119.52		-25.73			18.91			 2.086017	-0.449073		0.330041

	// bool NavigationImpl::moveHead(float yaw,float pitch){
	// 	client_moveHead = n->serviceClient<rapp_ros_naoqi_wrappings::MoveHead>("rapp_moveHead");
		

	// 	  rapp_ros_naoqi_wrappings::MoveHead srv;
	// 	  srv.request.pitch = pitch;
	// 	  srv.request.yaw = yaw;
	// 	  if (client_moveHead.call(srv))
	// 	  {
	// 	  	ROS_INFO("Nao-s head position is: \n");
	//   	  	ROS_INFO_STREAM("Yaw: "<<srv.response.yaw_now);
	// 	  	ROS_INFO_STREAM("Pitch: "<<srv.response.pitch_now);
	// 	  }
	// 	  else
	// 	  {
	// 	    ROS_ERROR("Failed to call service moveHead"); 
	// 	  }
	// }
	bool NavigationImpl::moveJoint(std::vector<std::string> joint, std::vector<float> angle){
		client_moveJoint = n->serviceClient<rapp_ros_naoqi_wrappings::MoveJoint>("rapp_moveJoint");

		rapp_ros_naoqi_wrappings::MoveJoint srv;
		//memcpy(&srv.request.joint_name, &joint, sizeof(joint));
		//memcpy(&srv.request.joint_angle, &angle, sizeof(angle));
		srv.request.joint_name = joint;
		srv.request.joint_angle = angle;

	 	if (client_moveJoint.call(srv))
	 	  {
	   	  return srv.response.status;
	  	  	//	ROS_INFO_STREAM(srv.request.joint_name<<" position is: \n"<<srv.response.angle_now);
	 	  }
	 	else
	 	  {
	  	  	return false;
	 	    ROS_ERROR("Failed to call service moveJoint"); 
	 	  }
    }	
	// bool NavigationImpl::removeStiffness(std::string joint){
	// 	client_removeStiffness = n->serviceClient<rapp_ros_naoqi_wrappings::RemoveStiffness>("rapp_removeStiffness");
		

	// 	  rapp_ros_naoqi_wrappings::RemoveStiffness srv;
	// 	  srv.request.joint_name = joint;

	// 	  if (client_removeStiffness.call(srv))
	// 	  {
	//   	  	ROS_INFO_STREAM(srv.request.joint_name<<" stiffness is off");
	// 	  }
	// 	  else
	// 	  {
	// 	    ROS_ERROR("Failed to call service removeStiffness"); 
	// 	  }
	// }	
	bool NavigationImpl::takePredefinedPosture(std::string posture, float speed){
		client_takePredefinedPosture = n->serviceClient<rapp_ros_naoqi_wrappings::TakePredefinedPosture>("rapp_takePredefinedPosture");
		

		  rapp_ros_naoqi_wrappings::TakePredefinedPosture srv;
		  srv.request.pose = posture;
		  srv.request.speed = speed;

		  if (client_takePredefinedPosture.call(srv))
		  {
	  	  	return srv.response.status;
	  	  	//ROS_INFO_STREAM(srv.request.pose<<" stiffness is off");
		  }
		  else
		  {
	  	  	return false;
		    ROS_ERROR("Failed to call service takePredefinedPosture"); 
		  }
	}	
	bool NavigationImpl::moveStop(){

		client_moveStop = n->serviceClient<rapp_ros_naoqi_wrappings::MoveStop>("rapp_moveStop");
		  rapp_ros_naoqi_wrappings::MoveStop srv;
		  if (client_moveStop.call(srv))
		  {
		    return srv.response.status;
	  	  	ROS_INFO("Nao has stopped");
		  }
		  else
		  {
	  	  	return false;
		    ROS_ERROR("Failed to call service moveStop"); 
		  }
	}
	// bool NavigationImpl::visOdom(){

	// 	client_moveStop = n->serviceClient<rapp_ros_naoqi_wrappings::MoveStop>("rapp_moveStop");
	// 	  rapp_ros_naoqi_wrappings::MoveStop srv;
	// 	  srv.request.stop_signal = true;
	// 	  if (client_moveStop.call(srv))
	// 	  {
	// 	    ROS_INFO("Nao has been localized via QR-code");
	// 	  }
	// 	  else
	// 	  {
	// 	    ROS_ERROR("Failed to call service visOdom"); 
	// 	  }
	// }
	bool NavigationImpl::lookAtPoint(float pointX,float pointY,float pointZ ){

		client_lookAtPoint = n->serviceClient<rapp_ros_naoqi_wrappings::LookAtPoint>("rapp_lookAtPoint");
		  rapp_ros_naoqi_wrappings::LookAtPoint srv;
		  srv.request.pointX = pointX;
		  srv.request.pointY = pointY;
		  srv.request.pointZ = pointZ;

		  if (client_lookAtPoint.call(srv))
		  {
		   return srv.response.status;
	  	  	ROS_INFO_STREAM("Service ended with status:\n" <<srv.response.status);
		  }
		  else
		  {
	  	  	return false;
		    ROS_ERROR("REUEST FAILED:  lookAtPoint"); 
		  }
	}
	bool NavigationImpl::rest(std::string posture){
		client_rest = n->serviceClient<rapp_ros_naoqi_wrappings::Rest>("rapp_rest");
		  rapp_ros_naoqi_wrappings::Rest srv;
		  srv.request.posture = posture;
		  if (client_rest.call(srv))
		  {
		    return srv.response.status;
	  	  	ROS_INFO("Nao is in rest state");
		  }
		  else
		  {
	  	  	return false;
		    ROS_ERROR("Failed to call service rest"); 
		  }


	}
	bool NavigationImpl::moveAlongPath(rapp::object::Path path){

			nav_msgs::Path path_ros;
			path_ros.header.seq = path.header.seq;
			path_ros.header.frame_id = path.header.frame_id;
			path_ros.header.stamp.sec = path.header.stamp.sec;
			path_ros.header.stamp.nsec = path.header.stamp.nsec;
			for (uint32_t i=0; i < path.poses.size();i++){

				path_ros.poses.at(i).header.seq = path.poses.at(i).header.seq;
				path_ros.poses.at(i).header.frame_id = path.poses.at(i).header.frame_id;
				path_ros.poses.at(i).header.stamp.sec = path.poses.at(i).header.stamp.sec;
				path_ros.poses.at(i).header.stamp.nsec = path.poses.at(i).header.stamp.nsec;
				path_ros.poses.at(i).pose.position.x = path.poses.at(i).pose.position.x;
				path_ros.poses.at(i).pose.position.y = path.poses.at(i).pose.position.y;
				path_ros.poses.at(i).pose.position.z = path.poses.at(i).pose.position.z;
				path_ros.poses.at(i).pose.orientation.x = path.poses.at(i).pose.orientation.x;
				path_ros.poses.at(i).pose.orientation.y = path.poses.at(i).pose.orientation.y;
				path_ros.poses.at(i).pose.orientation.z = path.poses.at(i).pose.orientation.z;
				path_ros.poses.at(i).pose.orientation.w = path.poses.at(i).pose.orientation.w;
			}


		client_moveAlongPath = n->serviceClient<rapp_ros_naoqi_wrappings::MoveAlongPath>("rapp_moveAlongPath");
		  rapp_ros_naoqi_wrappings::MoveAlongPath srv;
  		  srv.request.path = path_ros;

		  if (client_moveAlongPath.call(srv))
		  {
		    return srv.response.status;
	  	  	ROS_INFO("Nao moved along path");
		  }
		  else
		  {
	  	  	return false;
		    ROS_ERROR("Failed to call service MoveAlongPath"); 
		  }





	}
	rapp::object::PoseStamped NavigationImpl::getRobotPose(){

		client_getRobotPose = n->serviceClient<rapp_ros_naoqi_wrappings::GetRobotPose>("rapp_getRobotPose");
		  rapp_ros_naoqi_wrappings::GetRobotPose srv;
		  geometry_msgs::PoseStamped pose_ros;
			rapp::object::PoseStamped pose;

		  	pose_ros = srv.response.pose;

		  if (client_getRobotPose.call(srv))
		  {
	  	  	ROS_INFO("Nao returned his pose");
		  	
		  	

			
			pose.header.seq = pose_ros.header.seq;
			pose.header.frame_id = pose_ros.header.frame_id;
			pose.header.stamp.sec = pose_ros.header.stamp.sec;
			pose.header.stamp.nsec = pose_ros.header.stamp.nsec;
			pose.pose.position.x = pose_ros.pose.position.x;
			pose.pose.position.y = pose_ros.pose.position.y;
			pose.pose.position.z = pose_ros.pose.position.z;

			pose.pose.orientation.x = pose_ros.pose.orientation.x;
			pose.pose.orientation.y = pose_ros.pose.orientation.y;	
			pose.pose.orientation.z = pose_ros.pose.orientation.z;
			pose.pose.orientation.w = pose_ros.pose.orientation.w;
		    
		    return pose;
		  }
		  else
		  {
		    return pose;
		    ROS_ERROR("Failed to call service getRobotPose"); 
		  }


	}
	bool NavigationImpl::setGlobalPose(rapp::object::PoseStamped rapp_pose){
		client_setGlobalPose = n->serviceClient<rapp_ros_naoqi_wrappings::SetGlobalPose>("rapp_setGlobalPose");
		rapp_ros_naoqi_wrappings::SetGlobalPose srv;

		geometry_msgs::PoseStamped pose_ros;
		pose_ros.pose.position.x = rapp_pose.pose.position.x;
		pose_ros.pose.position.y = rapp_pose.pose.position.y;
		pose_ros.pose.position.z = rapp_pose.pose.position.z; 
		pose_ros.pose.orientation.x = rapp_pose.pose.orientation.x; 
		pose_ros.pose.orientation.y = rapp_pose.pose.orientation.y; 
		pose_ros.pose.orientation.z = rapp_pose.pose.orientation.z; 
		pose_ros.pose.orientation.w = rapp_pose.pose.orientation.w; 
		
		pose_ros.header.seq = rapp_pose.header.seq;
		pose_ros.header.stamp.sec = rapp_pose.header.stamp.sec;
		pose_ros.header.stamp.nsec = rapp_pose.header.stamp.nsec;
		pose_ros.header.frame_id = rapp_pose.header.frame_id;


  		  srv.request.pose = pose_ros;
		  if (client_setGlobalPose.call(srv))
		  {
	  	  	ROS_INFO("Nao is localized");
		    return srv.response.status;
		  }
		  else
		  {
		    return false;
		    ROS_ERROR("Failed to call service setGlobalPose"); 
		  }




	}
	// rapp::objects::Path NavigationImpl::pathPlanner_2D(rapp::objects::Pose start, rapp::objects::Pose goal, rapp::objects::OccupancyGrid map){



	// }
 //    rapp::objects::Pose NavigationImpl::qrCodeLocalization(cv::Mat image, rapp::objects::QRcodeMap QRmap){


 //    }

} // namespace robot
} // namespace rapp
