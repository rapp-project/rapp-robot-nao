/**
 * @class Navigation
 * @brief Class which defines the interface for Robot navigation capabilities (movement, localization)
 * @date 10-August-2015
 * @author Wojciech Dudek <wojciechsbox@gmail.com>
 * @note This class uses pimpl pattern to make ABI as stable as possible when deploying new library versions
 */

#include "NavigationImpl.hpp"

namespace rapp {
namespace robot {

NavigationImpl::NavigationImpl(int argc, char ** argv ){
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
	bool NavigationImpl::moveJoint(std::vector<std::string> joint, std::vector<float> angle, float speed){
		client_moveJoint = n->serviceClient<rapp_ros_naoqi_wrappings::MoveJoint>("rapp_moveJoint");

		rapp_ros_naoqi_wrappings::MoveJoint srv;
		//memcpy(&srv.request.joint_name, &joint, sizeof(joint));
		//memcpy(&srv.request.joint_angle, &angle, sizeof(angle));
		srv.request.joint_name = joint;
		srv.request.joint_angle = angle;
		srv.request.speeds = speed;

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
	bool NavigationImpl::moveAlongPath(std::vector<rapp::object::pose_stamped> poses){

			nav_msgs::Path poses_ros;
		  	geometry_msgs::PoseStamped pose_ros;
		  	poses_ros.poses.clear();
			for (uint32_t i=0; i < poses.size();i++){
				pose_ros.header.seq = poses.at(i).header.seq_;
				pose_ros.header.frame_id = poses.at(i).header.frameid_;
				pose_ros.header.stamp.sec = poses.at(i).header.stamp_.sec();
				pose_ros.header.stamp.nsec = poses.at(i).header.stamp_.nanosec();
				pose_ros.pose.position.x = poses.at(i).pose.position.x;
				pose_ros.pose.position.y = poses.at(i).pose.position.y;
				pose_ros.pose.position.z = poses.at(i).pose.position.z;
				pose_ros.pose.orientation.x = poses.at(i).pose.orientation.x;
				pose_ros.pose.orientation.y = poses.at(i).pose.orientation.y;
				pose_ros.pose.orientation.z = poses.at(i).pose.orientation.z;
				pose_ros.pose.orientation.w = poses.at(i).pose.orientation.w;
				poses_ros.poses.push_back(pose_ros);
			}
	  	  	ROS_INFO_STREAM("poses: "<< poses.size());

	  	  	ROS_INFO_STREAM("poses_ros: "<< poses_ros.poses.size());

		client_moveAlongPath = n->serviceClient<rapp_ros_naoqi_wrappings::MoveAlongPath>("rapp_moveAlongPath");
		  rapp_ros_naoqi_wrappings::MoveAlongPath srv;
  		  srv.request.poses = poses_ros.poses;

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
	rapp::object::pose_stamped NavigationImpl::getRobotPose(){

		client_getRobotPose = n->serviceClient<rapp_ros_naoqi_wrappings::GetRobotPose>("rapp_getRobotPose");
		  rapp_ros_naoqi_wrappings::GetRobotPose srv;
		  geometry_msgs::PoseStamped pose_ros;
			rapp::object::pose_stamped pose;

		  	pose_ros = srv.response.pose;

		  if (client_getRobotPose.call(srv))
		  {
	  	  	ROS_INFO("Nao returned his pose");
		  	
			auto sec = std::chrono::seconds(srv.response.pose.header.stamp.sec);
			auto nsec = std::chrono::nanoseconds(srv.response.pose.header.stamp.nsec);		  	
		  	std::chrono::nanoseconds ns(sec+nsec);
			
			pose.header.seq_ = srv.response.pose.header.seq;
			pose.header.frameid_ = srv.response.pose.header.frame_id;
			pose.header.stamp_=ns;// = srv.response.pose.header.stamp.sec;
			pose.pose.position.x = srv.response.pose.pose.position.x;
			pose.pose.position.y = srv.response.pose.pose.position.y;
			pose.pose.position.z = srv.response.pose.pose.position.z;

			pose.pose.orientation.x = srv.response.pose.pose.orientation.x;
			pose.pose.orientation.y = srv.response.pose.pose.orientation.y;	
			pose.pose.orientation.z = srv.response.pose.pose.orientation.z;
			pose.pose.orientation.w = srv.response.pose.pose.orientation.w;
		    
		    return pose;
		  }
		  else
		  {
		    return pose;
		    ROS_ERROR("Failed to call service getRobotPose"); 
		  }


	}
	bool NavigationImpl::setGlobalPose(rapp::object::pose rapp_pose){
		client_setGlobalPose = n->serviceClient<rapp_ros_naoqi_wrappings::SetGlobalPose>("rapp_setGlobalPose");
		rapp_ros_naoqi_wrappings::SetGlobalPose srv;
		ros::Time time_now = ros::Time::now();
		geometry_msgs::PoseStamped pose_ros;
		pose_ros.pose.position.x = rapp_pose.position.x;
		pose_ros.pose.position.y = rapp_pose.position.y;
		pose_ros.pose.position.z = rapp_pose.position.z; 
		pose_ros.pose.orientation.x = rapp_pose.orientation.x; 
		pose_ros.pose.orientation.y = rapp_pose.orientation.y; 
		pose_ros.pose.orientation.z = rapp_pose.orientation.z; 
		pose_ros.pose.orientation.w = rapp_pose.orientation.w; 
		
		pose_ros.header.seq = 0;
		pose_ros.header.stamp.sec = time_now.sec;
		pose_ros.header.stamp.nsec = time_now.nsec;
		pose_ros.header.frame_id = "map";


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
std::vector<std::vector<float>> NavigationImpl::getTransform(std::string chainName, int space){
	client_getTransform = n->serviceClient<rapp_ros_naoqi_wrappings::GetTransform>("rapp_get_transform");
	rapp_ros_naoqi_wrappings::GetTransform srv;
	srv.request.chainName = chainName;
	srv.request.space = space;
	cv::Mat transformMatrix = cv::Mat::zeros(4,4,cv::DataType<float>::type);
	std::vector<float> rows;
	std::vector<std::vector<float>> end_transform;
	
	if (client_getTransform.call(srv)) //
	{
		/*
		transformMatrix[0][0] = srv.response.transformMatrix.r11[0]; transformMatrix[0][1] = srv.response.transformMatrix.r12[0];
		transformMatrix[0][2] = srv.response.transformMatrix.r13[0]; transformMatrix[0][3] = srv.response.transformMatrix.r14[0];
		transformMatrix[1][0] = srv.response.transformMatrix.r21[0]; transformMatrix[1][1] = srv.response.transformMatrix.r22[0];
		transformMatrix[1][2] = srv.response.transformMatrix.r23[0]; transformMatrix[1][3] = srv.response.transformMatrix.r24[0];
		transformMatrix[2][0] = srv.response.transformMatrix.r31[0]; transformMatrix[2][1] = srv.response.transformMatrix.r32[0];
		transformMatrix[2][2] = srv.response.transformMatrix.r33[0]; transformMatrix[2][3] = srv.response.transformMatrix.r34[0];
		transformMatrix[3][0] = srv.response.transformMatrix.r41[0]; transformMatrix[3][1] = srv.response.transformMatrix.r42[0];
		transformMatrix[3][2] = srv.response.transformMatrix.r43[0]; transformMatrix[3][3] = srv.response.transformMatrix.r44[0];
		//*/
		//
		transformMatrix.at<float>(0,0) = srv.response.transformMatrix.r11[0]; transformMatrix.at<float>(0,1) = srv.response.transformMatrix.r12[0];
		transformMatrix.at<float>(0,2) = srv.response.transformMatrix.r13[0]; transformMatrix.at<float>(0,3) = srv.response.transformMatrix.r14[0];
		transformMatrix.at<float>(1,0) = srv.response.transformMatrix.r21[0]; transformMatrix.at<float>(1,1) = srv.response.transformMatrix.r22[0];
		transformMatrix.at<float>(1,2) = srv.response.transformMatrix.r23[0]; transformMatrix.at<float>(1,3) = srv.response.transformMatrix.r24[0];
		transformMatrix.at<float>(2,0) = srv.response.transformMatrix.r31[0]; transformMatrix.at<float>(2,1) = srv.response.transformMatrix.r32[0];
		transformMatrix.at<float>(2,2) = srv.response.transformMatrix.r33[0]; transformMatrix.at<float>(2,3) = srv.response.transformMatrix.r34[0];
		transformMatrix.at<float>(3,0) = srv.response.transformMatrix.r41[0]; transformMatrix.at<float>(3,1) = srv.response.transformMatrix.r42[0];
		transformMatrix.at<float>(3,2) = srv.response.transformMatrix.r43[0]; transformMatrix.at<float>(3,3) = srv.response.transformMatrix.r44[0];
		//*/
		ROS_INFO("[Rapp get transform] - Transformation matrix computed");
	}
	else
	{
		//Failed to call service rapp_get_image
		ROS_ERROR("[Rapp get transform] - Error calling service rapp_get_transform");
	}
	
	end_transform.clear();
	for (int i=0;i<transformMatrix.rows;i++){
		rows.clear();
		for (int j=0;j<transformMatrix.cols;j++){
			rows.push_back(transformMatrix.at<float>(i,j));
		}
		end_transform.push_back(rows);
	}
	
	return end_transform;//tmpVec;
}

	
} // namespace robot
} // namespace rapp

