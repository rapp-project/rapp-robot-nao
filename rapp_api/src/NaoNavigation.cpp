#include "rapp_api/NaoNavigation.h"
#include "ros/ros.h"
#include "rapp_ros_naoqi_wrappings/MoveTo.h"
#include "rapp_ros_naoqi_wrappings/MoveVel.h"
#include "rapp_ros_naoqi_wrappings/MoveHead.h"
#include "rapp_ros_naoqi_wrappings/MoveStop.h"
#include "rapp_ros_naoqi_wrappings/MoveGetCollisionStatus.h"
#include "rapp_ros_naoqi_wrappings/UpdatePose.h"
#include "rapp_ros_naoqi_wrappings/GetPose.h"
#include "rapp_ros_naoqi_wrappings/GetPlan.h"

NaoNavigation::NaoNavigation(int argc,char **argv){
		ros::init(argc, argv,"MoveTo_client");
		n = new ros::NodeHandle();

		}

	void NaoNavigation::moveTo(float x, float y, float theta){	

		client_moveTo = n->serviceClient<rapp_ros_naoqi_wrappings::MoveTo>("rapp_moveTo");
		  rapp_ros_naoqi_wrappings::MoveTo srv;
		  srv.request.destination_x = x;
		  srv.request.destination_y = y;
		  srv.request.destination_theta = theta;
		  if (client_moveTo.call(srv))
		  {
		  	if (srv.response.isDestinationReached == true){

		    ROS_INFO("Nao moved ");

		  	}else{
		    ROS_INFO("Nao sees obstacle");

		  	}
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service MoveTo"); 
		  }

	}

	void NaoNavigation::moveVel(float x, float y, float theta){	

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
		    ROS_ERROR("Failed to call service MoveVel"); 
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

	void NaoNavigation::moveHead(float yaw,float pitch){
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
		    ROS_ERROR("Failed to call service MoveHead"); 
		  }
	}
	void NaoNavigation::moveStop(){

		client_moveStop = n->serviceClient<rapp_ros_naoqi_wrappings::MoveStop>("rapp_moveStop");
		  rapp_ros_naoqi_wrappings::MoveStop srv;
		  srv.request.stop_signal = true;
		  if (client_moveStop.call(srv))
		  {
		    ROS_INFO("Nao has stopped");
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service MoveStop"); 
		  }
	}
	//Last collision status - obstacle position [x,y,0]
	void NaoNavigation::moveGetCollisionStatus(){

		client_moveGetCollisionStatus = n->serviceClient<rapp_ros_naoqi_wrappings::MoveGetCollisionStatus>("rapp_moveGetCollisionStatus");

		  rapp_ros_naoqi_wrappings::MoveGetCollisionStatus srv;
		  srv.request.get_status = true;
		  if (client_moveGetCollisionStatus.call(srv))
		  {
		    ROS_INFO("Got collision status");
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service MoveGetCollisionStatus"); 
		  }
	}
	void NaoNavigation::updatePose(){

		client_updatePose = n->serviceClient<rapp_ros_naoqi_wrappings::UpdatePose>("rapp_updatePose");
		  rapp_ros_naoqi_wrappings::UpdatePose srv;
		  srv.request.update_pose = true;
		  if (client_updatePose.call(srv))
		  {
		    ROS_INFO("Request: POSE_UPDATE has been sent");
		  }
		  else
		  {
		    ROS_ERROR("REUEST FAILED:  POSE_UPDATE"); 
		  }
	}

	void NaoNavigation::getPose(){

		client_getPose = n->serviceClient<rapp_ros_naoqi_wrappings::GetPose>("rapp_getPose");
		  rapp_ros_naoqi_wrappings::GetPose srv;
		  srv.request.get_pose = true;
		  if (client_getPose.call(srv))
		  {
		    ROS_INFO("Request: GET_POSE has been sent");
		  }
		  else
		  {
		    ROS_ERROR("REUEST FAILED:  GET_POSE"); 
		  }
	}
	void NaoNavigation::getPlan(){

		client_getPlan = n->serviceClient<rapp_ros_naoqi_wrappings::GetPlan>("rapp_getPlan");
		bool get_plan = true;
		  rapp_ros_naoqi_wrappings::GetPlan srv;
		  srv.request.get_plan = get_plan;
		  if (client_getPlan.call(srv))
		  {
		    ROS_INFO("Request: GET_PLAN has been sent");
		  }
		  else
		  {
		    ROS_ERROR("REUEST FAILED:  GET_PLAN"); 
		  }
	}