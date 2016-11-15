/**
 * @class Navigation
 * @brief Class which defines the interface for Robot navigation capabilities (movement, localization)
 * @date 10-August-2015
 * @author Wojciech Dudek <wojciechsbox@gmail.com>
 * @note This class uses pimpl pattern to make ABI as stable as possible when deploying new library versions
 */


#include <rapp-robots-api/navigation/navigation.hpp>

#include "NavigationImpl.hpp"

namespace rapp {
namespace robot {

navigation::navigation(int argc, char ** argv ) {
	pimpl = new NavigationImpl(argc, argv);
}

navigation::~navigation() {
	delete pimpl;
}
	bool navigation::point_arm(float x, float y, float z){
		bool status;
		status = pimpl->pointArm( x, y, z);		
		return status;

	}
	bool navigation::move_to(float x, float y, float theta){
		bool status;
		status = pimpl->moveTo( x, y, theta);		
		return status;

	}
	bool navigation::move_vel(float x, float y, float theta){
		bool status;
		status = pimpl->moveVel(x, y,  theta);		
		return status;
	}
	// bool navigation::moveHead(float yaw,float pitch){
	// 	pimpl->moveHead(yaw,pitch);		
	// }
	bool navigation::move_stop(){
		bool status;
		status = pimpl->moveStop();				
		return status;
	}
	bool navigation::move_joint(std::vector<std::string> joint, std::vector<float> angle, float speed){
		bool status = false;

		static const std::string arr[] = { "Body","Head","LArm","LLeg","RLeg","RArm","HeadYaw","LShoulderPitch","LHipYawPitch","RHipYawPitch","RShoulderPitch", "HeadPitch","LShoulderRoll","LHipRoll","RHipRoll","RShoulderRoll", "LElbowYaw","LHipPitch","RHipPitch","RElbowYaw", "LElbowRoll","LKneePitch","RKneePitch","RElbowRoll", "LWristYaw","LAnklePitch","RAnklePitch","RWristYaw", "LHand","RAnkleRoll","LAnkleRoll","RHand"};
		std::vector<std::string> joints_map(arr, arr + sizeof(arr) / sizeof(arr[0]) );
		std::sort (joints_map.begin(), joints_map.end());
		for (unsigned int i=0; i<joint.size();i++){
			if (std::binary_search (joints_map.begin(), joints_map.end(), joint.at(i))){
				status = pimpl->moveJoint(joint, angle, speed);			
		 	}else{
	    		std::cout << "\nInput joint/jointChain does not exist!\nAvaliable joints/jointChains are:\n";
			    for (unsigned int i = 0; i < joints_map.size(); i++)
			    {
			        std::cout << " " << joints_map.at(i) << ", ";
			    }
			        std::cout << "\n";

	    		status = false;
	    		break;
		 	}
		}
		return status;	
	}
	// bool navigation::removeStiffness(std::string joint){
	// 	pimpl->removeStiffness(joint);		
	// }
	bool navigation::take_predefined_posture(std::string posture, float speed){

		static const std::string arr[] = { "Crouch","Sit","SitRelax","LyingBelly","LyingBack","Stand","StandInit","StandZero"};
		std::vector<std::string> take_posture_map(arr, arr + sizeof(arr) / sizeof(arr[0]) );
		std::sort (take_posture_map.begin(), take_posture_map.end());
		bool status;

 		if (std::binary_search (take_posture_map.begin(), take_posture_map.end(), posture)){
			status = pimpl->takePredefinedPosture(posture, speed);		
	 	}else{
    		std::cout << "\nInput posture is not defnined!\nDefined postures are:\n";
		    for (unsigned int i = 0; i < take_posture_map.size(); i++)
		    {
		        std::cout << " " << take_posture_map.at(i) << "\n";
		    }
    		status = false;
	 	}
		
		return status;	
	}
	// bool navigation::visOdom(){
	// 	pimpl->visOdom();		
	// }
	bool navigation::look_at_point(float x, float y, float z){
		bool status;
		status = pimpl->lookAtPoint(x, y, z);			
		return status;	
	}
	bool navigation::rest(std::string posture){

		static const std::string arr[] = { "Crouch","Sit","SitRelax","LyingBelly","LyingBack"};
		std::vector<std::string> rest_posture_map(arr, arr + sizeof(arr) / sizeof(arr[0]) );

		std::sort (rest_posture_map.begin(), rest_posture_map.end());
		bool status;

 		if (std::binary_search (rest_posture_map.begin(), rest_posture_map.end(), posture)){
			status = pimpl->rest(posture);		
	 	}else{
    		std::cout << "\nInput posture is not safe!\nSafe postures are:\n";
		    for (unsigned int i = 0; i < rest_posture_map.size(); i++)
		    {
		        std::cout << " " << rest_posture_map.at(i) << "\n";
		    }
    		status = false;
	 	}

		return status;
	}

	bool navigation::move_along_path(std::vector<rapp::object::pose_stamped> poses){
		bool status;
		status = pimpl->moveAlongPath(poses);
		return status;		
	}
	rapp::object::pose_stamped navigation::get_global_pose(){
		rapp::object::pose_stamped pose;
		pose = pimpl->getRobotPose();
		return pose;

	}
	bool navigation::set_global_pose(rapp::object::pose rapp_pose){

		bool status;
		status = pimpl->setGlobalPose(rapp_pose);
		return status;
	}
	std::vector<std::vector<float>> navigation::get_transform(std::string chainName, int space){
		std::vector<std::vector<float>> MatStruct;
		MatStruct.clear();
		if (space == 0 ||space == 1 || space == 2 || space == 3){
			static const std::string arr[] = { "Head","LArm","RArm","LLeg","RLeg","Torso","CameraTop","CameraBottom","MicroFront","MicroRear","MicroLeft","MicroRight","Accelerometer","Gyrometer","Laser","LFsrFR","LFsrFL","LFsrRR","LFsrRL","RFsrFR","RFsrFL","RFsrRR", "RFsrRL", "USSensor1", "USSensor2", "USSensor3", "USSensor4"};
			std::vector<std::string> chainNames (arr, arr + sizeof(arr) / sizeof(arr[0]) );
			
			std::sort (chainNames.begin(), chainNames.end());

	 		if (std::binary_search (chainNames.begin(), chainNames.end(), chainName)){
				MatStruct = pimpl->getTransform(chainName, space);
		 	}else{
	    		std::cout << "Input chainName is not defined!\nAvailabre chainNames are:\n Head LArm RArm LLeg RLeg Torso CameraTop CameraBottom MicroFront MicroRear MicroLeft MicroRight Accelerometer Gyrometer Laser LFsrFR LFsrFL LFsrRR LFsrRL RFsrFR RFsrFL RFsrRR RFsrRL USSensor1 USSensor2 USSensor3 USSensor4\n";
		 	}
		}else{
	    	std::cout << "Input space not exist!, \n space: Task frame {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2, FRAME_GLOBAL = 3}\n";

		}
	return MatStruct;
}
} // namespace rapp
} // namespace robot
