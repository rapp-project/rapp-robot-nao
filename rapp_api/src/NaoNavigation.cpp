//#####################
// written by Wojciech Dudek
//#####################
#include "rapp_api/NaoNavigation.h"
#include "ros/ros.h"
#include "rapp_ros_naoqi_wrappings/MoveTo.h"
#include "rapp_ros_naoqi_wrappings/MoveVel.h"
#include "rapp_ros_naoqi_wrappings/MoveHead.h"
#include "rapp_ros_naoqi_wrappings/MoveStop.h"
#include "rapp_ros_naoqi_wrappings/MoveJoint.h"
#include "rapp_ros_naoqi_wrappings/Rest.h"
#include "rapp_ros_naoqi_wrappings/MoveAlongPath.h"
#include "rapp_ros_naoqi_wrappings/GetRobotPose.h"
#include "rapp_ros_naoqi_wrappings/SetGlobalPose.h"
// #include "rapp_ros_naoqi_wrappings/PathPlanner_2D.h"
// #include "rapp_ros_naoqi_wrappings/QRcodeLocalization.h"
#include "rapp_ros_naoqi_wrappings/TakePredefinedPosture.h"

NaoNavigation::NaoNavigation(int argc,char **argv){
		ros::init(argc, argv,"NaoNavigation");
		n = new ros::NodeHandle();

		}
	bool NaoNavigation::moveTo(float x, float y, float theta){	

		client_moveTo = n->serviceClient<rapp_ros_naoqi_wrappings::MoveTo>("rapp_moveTo");

		  rapp_ros_naoqi_wrappings::MoveTo srv;
		  srv.request.x = x;
		  srv.request.y = y;
		  srv.request.theta = theta;
		  if (client_moveTo.call(srv))
		  {
	  	  	return srv.response.status
 			ROS_INFO_STREAM("Service ended with status:\n" <<srv.response.status);
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service MoveTo"); 
		    return false;
		  }
	}
	bool NaoNavigation::moveVel(float x, float y, float theta){	

		client_moveVel = n->serviceClient<rapp_ros_naoqi_wrappings::MoveVel>("rapp_moveVel");

		  rapp_ros_naoqi_wrappings::MoveVel srv;
		  srv.request.velocity_x = x;
		  srv.request.velocity_y = y;
		  srv.request.velocity_theta = theta;
		  if (client_moveVel.call(srv))
		  {
		    ROS_INFO("Nao moved ");
	  	  	return srv.response.status
		    
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service MoveVel"); 
		    return false;
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

	// void NaoNavigation::moveHead(float yaw,float pitch){
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
	// 	    ROS_ERROR("Failed to call service MoveHead"); 
	// 	  }
	// }
	bool NaoNavigation::moveJoint(std::vector<std::string> joint, std::vector<float> angle, float speeds){
		client_moveJoint = n->serviceClient<rapp_ros_naoqi_wrappings::MoveJoint>("rapp_moveJoint");

		rapp_ros_naoqi_wrappings::MoveJoint srv;
		//memcpy(&srv.request.joint_name, &joint, sizeof(joint));
		//memcpy(&srv.request.joint_angle, &angle, sizeof(angle));
		srv.request.joint_name = joint;
		srv.request.joint_angle = angle;
		srv.request.speeds = speeds;

		if (client_moveJoint.call(srv))
		  {
	  	  	return srv.response.status
	  	  	//ROS_INFO_STREAM(srv.request.joint_name<<" position is: \n"<<srv.response.angle_now);
		  }
		else
		  {
		    ROS_ERROR("Failed to call service moveJoint");
		    return false;

		  }
	}	
	// void NaoNavigation::removeStiffness(std::string joint){
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
	bool NaoNavigation::takePredefinedPosture(std::string posture){
		client_takePredefinedPosture = n->serviceClient<rapp_ros_naoqi_wrappings::TakePredefinedPosture>("rapp_takePredefinedPosture");
		

		  rapp_ros_naoqi_wrappings::TakePredefinedPosture srv;
		  srv.request.pose = posture;

		  if (client_takePredefinedPosture.call(srv))
		  {
	  	  	return srv.response.status
	  	  	//ROS_INFO_STREAM(srv.request.pose<<" stiffness is off");
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service takePredefinedPosture"); 
		    return false;
		  }
	}	
	bool NaoNavigation::moveStop(){

		client_moveStop = n->serviceClient<rapp_ros_naoqi_wrappings::MoveStop>("rapp_moveStop");
		  rapp_ros_naoqi_wrappings::MoveStop srv;
		  srv.request.stop_signal = true;
		  if (client_moveStop.call(srv))
		  {
	  	  	return srv.response.status
		    ROS_INFO("Nao has stopped");
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service MoveStop"); 
		    return false;
		  }
	}
	bool NaoNavigation::rest(std::string posture){
		client_rest = n->serviceClient<rapp_ros_naoqi_wrappings::Rest>("rapp_rest");
		  rapp_ros_naoqi_wrappings::Rest srv;
		  srv.request.posture = posture;
		  if (client_rest.call(srv))
		  {
	  	  	return srv.response.status
		    ROS_INFO("Nao is in rest state");
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service rest"); 
		    return false;
		  }


	}
	bool NaoNavigation::moveAlongPath(rapp::objects::Path path){
		client_MoveAlongPath = n->serviceClient<rapp_ros_naoqi_wrappings::MoveAlongPath>("rapp_moveAlongPath");
		  rapp_ros_naoqi_wrappings::MoveAlongPath srv;
  		  srv.request.path = path;

		  if (client_MoveAlongPath.call(srv))
		  {
	  	  	return srv.response.status
		    ROS_INFO("Nao moved along path");
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service MoveAlongPath"); 
		    return false;
		  }



	}
	rapp::objects::PoseStamped NaoNavigation::getRobotPose(){
		client_GetRobotPose = n->serviceClient<rapp_ros_naoqi_wrappings::GetRobotPose>("rapp_getRobotPose");
		  rapp_ros_naoqi_wrappings::GetRobotPose srv;
		  if (client_MoveAlongPath.call(srv))
		  {
		  	rapp::objects::PoseStamped service_response = srv.response.pose
		  	return service_response
		    ROS_INFO("Nao returned his pose");
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service getRobotPose");
		    return false;
		  }


	}
	bool NaoNavigation::setGlobalPose(rapp::objects::Pose pose){
		client_SetGlobalPose = n->serviceClient<rapp_ros_naoqi_wrappings::SetGlobalPose>("rapp_setGlobalPose");
		  rapp_ros_naoqi_wrappings::SetGlobalPose srv;
  		  srv.request.pose = pose;
		  if (client_MoveAlongPath.call(srv))
		  {
	  	  	return srv.response.status
		    ROS_INFO("Nao is localized");
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service setGlobalPose"); 
	  	  	return false;

		  }



	}
	// rapp::objects::Path NaoNavigation::pathPlanner_2D(rapp::objects::Pose start, rapp::objects::Pose goal, rapp::objects::OccupancyGrid map){
	// 	client_PathPlanner_2D = n->serviceClient<rapp_ros_naoqi_wrappings::PathPlanner_2D>("rapp_pathPlanner_2D");
	// 	  rapp_ros_naoqi_wrappings::PathPlanner_2D srv;
 //  		  srv.request.pose = pose;
 //  		  srv.request.pose = pose;
 //  		  srv.request.pose = pose;
	// 	  if (client_MoveAlongPath.call(srv))
	// 	  {
	// 	    ROS_INFO("Nao is localized");
	// 	  }
	// 	  else
	// 	  {
	// 	    ROS_ERROR("Failed to call service pathPlanner_2D"); 
	// 	  }


	// }
 //    rapp::objects::Pose NaoNavigation::qrCodeLocalization(cv::Mat image, rapp::objects::QRcodeMap QRmap){


 //    }
