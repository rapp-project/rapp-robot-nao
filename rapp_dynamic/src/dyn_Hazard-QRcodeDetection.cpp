//#####################
// written by Jan Figat
//#####################

#include "NaoVision.h"
#include "NaoCommunication.h"

main(int argc, char **argv)
{
	NaoCommunication NAO_Communication(argc,argv);
	NaoVision NAO_Vision(argc,argv);
	sensor_msgs::Image frame;
	cv::Mat robotToCameraMatrix;

	//NaoVision_.QRcodeDetection QRcodeDetectionStruct;
	NaoVision::QRcodeDetection QRcodeDetectionStruct;
	NaoVision::QRcodeHazardDetection QRcodeHazardStruct;

	// Create a zbar reader
	zbar::ImageScanner scanner;

	double x_rot, y_rot, z_rot;
	std::string text_message;
	int count;
	bool check;
	count=0;
	check = true;
	while (check==true)
{
	frame = NAO_Vision.captureImage("top - adaptive auto exposure 1"/*Auto Exposure Algorithm*/,2/*camera resolution*/);//for hightlights --2; weighted average scene Brightness--1; Adaptive weighted auto exposure for lowlights--3;; 3-4VGA,2-VGA,1-QVGA (camera resolution)
	robotToCameraMatrix.release();//clears the robotToCameraMatrix
	robotToCameraMatrix = NAO_Vision.getTransform("CameraTop", 2); //From Camera to waist (robot) coordinate system ; FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2
	if (frame.height == 0 || frame.width == 0) //frame is empty
	{
		std::cout<<"Camera frame is empty"<<std::endl;
		count+=1;
		if (count>4) check=false;
	}
	else //camera frame is not empty
	{
		std::cout<<"QRcode detection ..."<<std::endl;
		QRcodeDetectionStruct = NAO_Vision.qrCodeDetection(frame, scanner, robotToCameraMatrix); // QRcode detection
		if (QRcodeDetectionStruct.QRmessage.size()>=2) 
			QRcodeHazardStruct = NAO_Vision.openDoorDetection(QRcodeDetectionStruct.LandmarkInRobotCoordinate, QRcodeDetectionStruct.QRmessage); // Hazard detection -- open door detection while using QRcodes
		//std::cout<<QRcodeDetectionStruct.QRmessage.size()<<std::endl;

		for (int i=0; i<QRcodeDetectionStruct.QRmessage.size(); i++) //QRcode detected
		{
			std::size_t found = QRcodeDetectionStruct.QRmessage[i].find_first_of(";");
			//std::cout<<found;
			text_message = QRcodeDetectionStruct.QRmessage[i].substr(0,found);
			//std::cout<<text_message;
		
			//NAO_Vision.textToSpeech( QRcodeDetectionStruct.QRmessage[i], "English" );//reads the QRcode message
		//	NAO_Vision.textToSpeech("Q R code detected ", "English");
		//	NAO_Vision.textToSpeech( text_message, "English" );//reads the QRcode message
			std::cout<<"QRcode message: \t"<<QRcodeDetectionStruct.QRmessage[i]<<std::endl;
			std::cout<<"QRcode in robot coordinate system:\n"<<QRcodeDetectionStruct.LandmarkInRobotCoordinate[i]<<std::endl;
		}
		if (QRcodeHazardStruct.isHazardFound == true) //Open door detected
		{
			//std::cout<<"Open door detected"<<std::endl;
			for (int i=0; i<QRcodeHazardStruct.openedObject.size(); i++)
			{
				//std::cout<<"OPEN: "<<QRcodeHazardStruct.openedObject[i]<<std::endl;
				std::size_t found = QRcodeHazardStruct.openedObject[i].find_first_of(";");
				if (found<QRcodeHazardStruct.openedObject[i].size() && found>0)
					text_message = QRcodeHazardStruct.openedObject[i].substr(0,found);
				else
					text_message = QRcodeHazardStruct.openedObject[i];
				//NAO_Vision.textToSpeech( QRcodeHazardStruct.openedObject[i] + std::string(" is open"), "English" );
				NAO_Communication.textToSpeech( text_message + std::string(" is open") , "English");
				std::cout<<"Detected that "<<QRcodeHazardStruct.openedObject[i]<<" is open"<<std::endl;
				std::cout<<"Position of detected hazard:\n\t"<<QRcodeHazardStruct.hazardPosition_x[i]<<",\t"<<QRcodeHazardStruct.hazardPosition_y[i]<<std::endl;
			}
		}
}//while
		
	}

   return 0;
}
