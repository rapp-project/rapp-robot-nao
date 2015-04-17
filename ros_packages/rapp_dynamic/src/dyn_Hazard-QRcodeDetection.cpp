//#####################
// written by Jan Figat
//#####################

#include "NaoVision.h"

main(int argc, char **argv)
{
	//NaoVision NaoVision_(argc,argv);
	NaoVision NAO_Vision(argc,argv);
	sensor_msgs::Image frame;
	cv::Mat robotToCameraMatrix;

	//NaoVision_.QRcodeDetection QRcodeDetectionStruct;
	NaoVision::QRcodeDetection QRcodeDetectionStruct;
	NaoVision::QRcodeHazardDetection QRcodeHazardStruct;

	// Create a zbar reader
	zbar::ImageScanner scanner;

	double x_rot, y_rot, z_rot;
	
	frame = NAO_Vision.captureImage("top - adaptive auto exposure 3");//for hightlights --2; weighted average scene Brightness--1; Adaptive weighted auto exposure for lowlights--3
	robotToCameraMatrix = NAO_Vision.getTransform("CameraTop", 2); //From Camera to waist (robot) coordinate system ; FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2
	if (frame.height == 0 || frame.width == 0) //frame is empty
		std::cout<<"Camera frame is empty"<<std::endl;
	else //camera frame is not empty
	{
		std::cout<<"QRcode detection ..."<<std::endl;
		QRcodeDetectionStruct = NAO_Vision.qrCodeDetection(frame, scanner, robotToCameraMatrix); // QRcode detection
		if (QRcodeDetectionStruct.QRmessage.size()>=2) 
			QRcodeHazardStruct = NAO_Vision.openDoorDetection(QRcodeDetectionStruct.LandmarkInRobotCoordinate, QRcodeDetectionStruct.QRmessage); // Hazard detection -- open door detection while using QRcodes

		for (int i=0; i<QRcodeDetectionStruct.QRmessage.size(); i++) //QRcode detected
		{
			std::cout<<"QRcode message: \t"<<QRcodeDetectionStruct.QRmessage[i]<<std::endl;
			std::cout<<"QRcode in robot coordinate system:\n"<<QRcodeDetectionStruct.LandmarkInRobotCoordinate[i]<<std::endl;
		}
		if (QRcodeHazardStruct.isHazardFound == true) //Open door detected
		{
			//std::cout<<"Open door detected"<<std::endl;
			for (int i=0; i<QRcodeHazardStruct.openedObject.size(); i++)
			{
				std::cout<<"Detected that "<<QRcodeHazardStruct.openedObject[i]<<" is open"<<std::endl;
				std::cout<<"Position of detected hazard:\n\t"<<QRcodeHazardStruct.hazardPosition_x[i]<<",\t"<<QRcodeHazardStruct.hazardPosition_y[i]<<std::endl;
			}
		}
		
	}

   return 0;
}
