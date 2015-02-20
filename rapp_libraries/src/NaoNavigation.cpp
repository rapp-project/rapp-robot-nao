#include "NaoNavigation.h"
#include "ros/ros.h"
#include "rapp_robot_agent/MoveTo.h"
#include "rapp_robot_agent/MoveVel.h"

NaoNavigation::NaoNavigation(int argc,char **argv){
		ros::init(argc, argv,"MoveTo_client");
		n = new ros::NodeHandle();
		}

	void NaoNavigation::moveTo(float x, float y, float theta){	

		client_moveTo = n->serviceClient<rapp_robot_agent::MoveTo>("rapp_moveTo");
		  rapp_robot_agent::MoveTo srv;
		  srv.request.destination_x = x;
		  srv.request.destination_y = y;
		  srv.request.destination_theta = theta;
		  if (client_moveTo.call(srv))
		  {
		    ROS_INFO("Nao moved ");
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service MoveTo"); 
		  }

	}

	void NaoNavigation::moveVel(float x, float y, float theta){	

		client_moveVel = n->serviceClient<rapp_robot_agent::MoveVel>("rapp_moveVel");

		  rapp_robot_agent::MoveVel srv;
		  srv.request.velocity_x = x;
		  srv.request.velocity_y = y;
		  srv.request.velocity_theta = theta;
		  if (client_moveVel.call(srv))
		  {
		    ROS_INFO("Nao moved ");
		  }
		  else
		  {
		    ROS_ERROR("Failed to call service MoveTo"); 
		  }
	}

//moveHead(rad,rad,rad)

	// void moveHead(float roll, float pitch, float yaw){

	// 	fractionMaxSpeed = 0.1
	// 	proxy_motion.setAngles('HeadRoll',roll,fractionMaxSpeed)

	// 	proxy_motion.setAngles('HeadYaw',yaw,fractionMaxSpeed)

	// 	proxy_motion.setAngles('HeadPitch',pitch,fractionMaxSpeed)
	// }