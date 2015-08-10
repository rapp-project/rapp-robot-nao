//#####################
// written by Wojciech Dudek
//#####################

#include "NavigationImpl.hpp"
//#include "string.h"

NavigationImpl::NavigationImpl(int argc,char **argv){
		ros::init(argc, argv,"NavigationImpl_library");
		n = new ros::NodeHandle();

		}

	void NavigationImpl::moveTo(float x, float y, float theta){	

		client_moveTo = n->serviceClient<rapp_ros_naoqi_wrappings::MoveTo>("rapp_moveTo");
		  rapp_ros_naoqi_wrappings::MoveTo srv;
		  srv.request.destination_x = x;
		  srv.request.destination_y = y;
		  srv.request.destination_theta = theta;
		  if (client_moveTo.call(srv))
		  {
		   ROS_INFO_STREAM("Service ended with status:\n" <<srv.response.status);

			}
		  else
		  {
		    ROS_ERROR("Failed to call service moveTo"); 
		  }

	}

	void NavigationImpl::moveVel(float x, float y, float theta){	

		client_moveVel = n->serviceClient<rapp_ros_naoqi_wrappings::MoveVel>("rapp_moveVel");

		  rapp_ros_naoqi_wrappings::MoveVel srv;
		  srv.request.velocity_x = x;
		  srv.request.velocity_y = y;
		  srv.request.velocity_theta = theta;
		  if (client_moveVel.call(srv))
		  {
		    ROS_INFO("Nao moved ");
		  }
		  else
		  {
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

	void NavigationImpl::moveHead(float yaw,float pitch){
		client_moveHead = n->serviceClient<rapp_ros_naoqi_wrappings::MoveHead>("rapp_moveHead");
		

		  rapp_ros_naoqi_wrappings::MoveHead srv;
		  srv.request.pitch = pitch;
		  srv.request.yaw = yaw;
		  if (client_moveHead.call(srv))
		  {
		  	ROS_INFO("Nao-s head position is: \n");
	  	  	ROS_INFO_STREAM("Yaw: "<<srv.response.yaw_now);
		  	ROS_INFO_STREAM("Pitch: "<<srv.response.pitch_now);
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service moveHead"); 
		  }
	}
	void NavigationImpl::moveJoint(std::string joint, float angle){
		client_moveJoint = n->serviceClient<rapp_ros_naoqi_wrappings::MoveJoint>("rapp_moveJoint");

		rapp_ros_naoqi_wrappings::MoveJoint srv;
		srv.request.joint_name = joint;
		srv.request.joint_angle = angle;

		if (client_moveJoint.call(srv))
		  {
	  	  	ROS_INFO_STREAM(srv.request.joint_name<<" position is: \n"<<srv.response.angle_now);
		  }
		else
		  {
		    ROS_ERROR("Failed to call service moveJoint"); 
		  }
	}	
	void NavigationImpl::removeStiffness(std::string joint){
		client_removeStiffness = n->serviceClient<rapp_ros_naoqi_wrappings::RemoveStiffness>("rapp_removeStiffness");
		

		  rapp_ros_naoqi_wrappings::RemoveStiffness srv;
		  srv.request.joint_name = joint;

		  if (client_removeStiffness.call(srv))
		  {
	  	  	ROS_INFO_STREAM(srv.request.joint_name<<" stiffness is off");
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service removeStiffness"); 
		  }
	}	
	void NavigationImpl::takePredefinedPosture(std::string pose){
		client_takePredefinedPosture = n->serviceClient<rapp_ros_naoqi_wrappings::TakePredefinedPosture>("rapp_takePredefinedPosture");
		

		  rapp_ros_naoqi_wrappings::TakePredefinedPosture srv;
		  srv.request.pose = pose;

		  if (client_takePredefinedPosture.call(srv))
		  {
	  	  	//ROS_INFO_STREAM(srv.request.pose<<" stiffness is off");
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service takePredefinedPosture"); 
		  }
	}	
	void NavigationImpl::moveStop(){

		client_moveStop = n->serviceClient<rapp_ros_naoqi_wrappings::MoveStop>("rapp_moveStop");
		  rapp_ros_naoqi_wrappings::MoveStop srv;
		  srv.request.stop_signal = true;
		  if (client_moveStop.call(srv))
		  {
		    ROS_INFO("Nao has stopped");
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service moveStop"); 
		  }
	}
	void NavigationImpl::visOdom(){

		client_moveStop = n->serviceClient<rapp_ros_naoqi_wrappings::MoveStop>("rapp_moveStop");
		  rapp_ros_naoqi_wrappings::MoveStop srv;
		  srv.request.stop_signal = true;
		  if (client_moveStop.call(srv))
		  {
		    ROS_INFO("Nao has been localized via QR-code");
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service visOdom"); 
		  }
	}
	void NavigationImpl::lookAtPoint(float pointX,float pointY,float pointZ ){

		client_lookAtPoint = n->serviceClient<rapp_ros_naoqi_wrappings::LookAtPoint>("rapp_lookAtPoint");
		  rapp_ros_naoqi_wrappings::LookAtPoint srv;
		  srv.request.pointX = pointX;
		  srv.request.pointY = pointY;
		  srv.request.pointZ = pointZ;

		  if (client_lookAtPoint.call(srv))
		  {
		   ROS_INFO_STREAM("Service ended with status:\n" <<srv.response.status);
		  }
		  else
		  {
		    ROS_ERROR("REUEST FAILED:  lookAtPoint"); 
		  }
	}