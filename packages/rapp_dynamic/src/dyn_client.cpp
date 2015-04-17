//#####################
// written by Jan Figat & Wojciech Dudek
//#####################

#include "NaoVision.h"
#include "NaoNavigation.h"
#include "math.h"
float getGoalTheta(float x_goal, float y_goal){
	float theta=0;
		if (y_goal>0){
			//#
			//#   1. quadrant in relation to map frame 
			//##
			if (x_goal>0){
				std::cout<< "1. quadrant"<<std::endl;
				theta =atan(y_goal/x_goal) ;
			}
			//##
			//#   2. quadrant in relation to map frame
			//##
			
			else if (x_goal<0){
				std::cout<< "2. quadrant"<<std::endl;
					theta =-atan(y_goal/x_goal);
			}
			//#
			//#  goal is on Y axis 
			
			else {
				//# Y axis (positive)
				std::cout<< "Y axis positive"<<std::endl;
					theta = M_PI/2;
			}
		}

		//##
		//#  3. & 4. quadrants of map frame
		//##				

		else if (y_goal< 0){
			//##
			//#   4. quadrant in relation to map frame
			//##
			if (x_goal<0){
				std::cout<< "3. quadrant"<<std::endl;
				theta =-M_PI/2 - atan(x_goal/y_goal);

			}
			//##
			//#   3. quadrant in relation to map frame 
			//##
			else if (x_goal>0){
				std::cout<< "4. quadrant"<<std::endl;
				theta =M_PI/2 + atan(x_goal/y_goal);
			}
			else {
				//# Y axis (negative)
				std::cout<< "Y axis negative"<<std::endl;
					theta = -M_PI/2;
			}
		}

		//goal in front of Nao
		else {

				if (x_goal>0){
					
					theta = 0;
				}
				if (x_goal<0){
					
					theta = M_PI;
				}

		}


return theta;
}
main(int argc, char **argv)
{
	//NaoVision NaoVision_(argc,argv);
	NaoVision NAO_Vision(argc,argv);
	sensor_msgs::Image frame;
	cv::Mat robotToCameraMatrix;
	NaoNavigation Nao(argc,argv);
ros::NodeHandle nh;
	//NaoVision_.QRcodeDetection QRcodeDetectionStruct;
	NaoVision::QRcodeDetection QRcodeDetectionStruct;
	double theta_marker;
	std::vector<double> vector1,vector2, vector3, euler_y;
	// Create a zbar reader
	zbar::ImageScanner scanner;
	bool NaoMoved = false;
	float yaw= 0;
	float headYaw= 0;
	float y_goal = 0;

	float x_goal = 0;
	float theta_goal = 0;
	float distance = 0;
while (NaoMoved != true && yaw<3){
	frame = NAO_Vision.captureImage("top - adaptive auto exposure 2");//for hightlights --2; weighted average scene Brightness--1; Adaptive weighted auto exposure for lowlights--3
	robotToCameraMatrix = NAO_Vision.getTransform("CameraTop", 2); //From Camera to waist (robot) coordinate system; FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2
	if (frame.height == 0 || frame.width == 0) //frame is empty
		std::cout<<"Camera frame is empty"<<std::endl;
	else
	{

		std::cout<<"QRcode detection ..."<<std::endl;
		QRcodeDetectionStruct.clear();
		QRcodeDetectionStruct = NAO_Vision.qrCodeDetection(frame, scanner, robotToCameraMatrix);
		std::cout<<QRcodeDetectionStruct.QRmessage.size();
		//std::cout<<"test";

		for (int i=0; i<QRcodeDetectionStruct.QRmessage.size(); i++)
		{

			std::cout<<"QRcode message: \t"<<QRcodeDetectionStruct.QRmessage[i]<<std::endl;
			std::cout<<"QRcode in robot coordinate system:\n"<<QRcodeDetectionStruct.LandmarkInRobotCoordinate[i]<<std::endl;

		}
	}
	if (QRcodeDetectionStruct.QRmessage.size()>0){

		vector1.clear();
		vector2.clear();
		vector3.clear();
		vector1.push_back(QRcodeDetectionStruct.LandmarkInRobotCoordinate[0].at<double>(0,0));
		vector2.push_back(QRcodeDetectionStruct.LandmarkInRobotCoordinate[0].at<double>(1,0)); //modyfikacja
		vector3.push_back(QRcodeDetectionStruct.LandmarkInRobotCoordinate[0].at<double>(2,0)); //modyfikacja
		//vector2.push_back(QRcodeDetectionStruct.LandmarkInRobotCoordinate[0].at<double>(0,1));
		x_goal = QRcodeDetectionStruct.LandmarkInRobotCoordinate[0].at<double>(0,3);
		y_goal = QRcodeDetectionStruct.LandmarkInRobotCoordinate[0].at<double>(1,3);
		
		euler_y = NAO_Vision.compute_euler_y(vector3); //modyfikacja
		theta_marker = NAO_Vision.compute_euler_z(vector2,vector1,euler_y)[0]; //modyfikacja
		//theta_marker = NAO_Vision.compute_euler_z(vector1,vector2)[0];
		nh.setParam("/marker/pose/x",QRcodeDetectionStruct.LandmarkInRobotCoordinate[0].at<double>(0,3)-0.5);
		nh.setParam("/marker/pose/y",QRcodeDetectionStruct.LandmarkInRobotCoordinate[0].at<double>(1,3));
		nh.setParam("/marker/pose/t",theta_marker);
		std::cout<<"rotation of QR-code around OZ axis in robot frame: "<<theta_marker*180.f/M_PI<<" in degrees"<<std::endl;

		theta_goal = getGoalTheta(x_goal,y_goal);

		Nao.moveTo(0,0,theta_goal); //modyfikacja
		distance = sqrt(x_goal*x_goal+y_goal*y_goal);
		Nao.moveTo(distance,0,0);
		NaoMoved = true;
		/*frame = NAO_Vision.captureImage("top - adaptive auto exposure 2");//for hightlights --2; weighted average scene Brightness--1; Adaptive weighted auto exposure for lowlights--3
		robotToCameraMatrix = NAO_Vision.getTransform("CameraTop", 2); //From Camera to waist (robot) coordinate system; FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2
		
		if (frame.height == 0 || frame.width == 0) //frame is empty
		std::cout<<"Camera frame is empty"<<std::endl;
		else
			{

		std::cout<<"QRcode detection ..."<<std::endl;
		QRcodeDetectionStruct.clear();
		QRcodeDetectionStruct = NAO_Vision.qrCodeDetection(frame, scanner, robotToCameraMatrix);
		std::cout<<QRcodeDetectionStruct.QRmessage.size();
		std::cout<<"test";

			for (int i=0; i<QRcodeDetectionStruct.QRmessage.size(); i++)
			{

			std::cout<<"QRcode message: \t"<<QRcodeDetectionStruct.QRmessage[i]<<std::endl;
			std::cout<<"QRcode in robot coordinate system:\n"<<QRcodeDetectionStruct.LandmarkInRobotCoordinate[i]<<std::endl;

			}
		}
		if (QRcodeDetectionStruct.QRmessage.size()>0){
			std::cout<<"HeadYaw : "<<-theta<<std::endl;
			Nao.moveTo(QRcodeDetectionStruct.LandmarkInRobotCoordinate[0].at<double>(0,3)-0.7,QRcodeDetectionStruct.LandmarkInRobotCoordinate[0].at<double>(1,3),0);
   			NaoMoved = true;
   		}*/
   	}
   		if (NaoMoved !=true){
   		Nao.moveHead(yaw-1.5,0);
   		yaw=yaw+0.6;
   		headYaw = yaw-1.5;
   		}
}

   return 0;
}
