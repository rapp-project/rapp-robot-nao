//#####################
// written by Jan Figat
//#####################

#include "NaoVision.h"
//#include "NaoNavigation.h"
#include "NaoCommunication.h"


#include <iostream>
#include <fstream>


#include "math.h"

using namespace std;
main(int argc, char **argv)
{
	//AL_SOUND_FORMAT * buffer;
	std::vector<std::string> audioBuffer;
    
	//NaoVision NaoVision_(argc,argv);
	NaoVision NAO_Vision(argc,argv);
	sensor_msgs::Image frame;
	cv::Mat robotToCameraMatrix;
	NaoCommunication Nao_Communication(argc,argv);
	ros::NodeHandle nh;
	NaoVision::QRcodeDetection QRcodeDetectionStruct;
	std::vector<double> vector1,vector2, vector3, euler_y;
	
	// Create a zbar reader
	zbar::ImageScanner scanner;

	// Communication
	bool communication = true;
	std::string recognizedWord;
	std::string fileName;
	std::vector<std::string> vectorFileName;
	std::string message;
	std::string dictionary[5];
	dictionary[0]="Voice";
	dictionary[1]="Code";
	int energy;
	int iterations=-1;
	const double waiting_time=5.0;
	bool once = true;
	bool end = false;

	// Vision -- QR code detection
	int count=0;
	bool check = false;
	
	
	if ( communication == true)
	{
		printf("voice message recording program\n");
		fflush(stdout);
		//recognizedWord = Nao_Communication.recognizeWord(dictionary, 2);
		//if (recognizedWord == "Voice")
		{
			Nao_Communication.textToSpeech( "Recording the message", "English" );
			while (once==true)
			{
				audioBuffer.clear();
				energy = Nao_Communication.microphoneEnergy("front");
				if (energy>2700)
				{	
					iterations ++;
					
					while (iterations<= waiting_time/0.085 || end==false)
					{
						if (iterations< waiting_time/0.085)
						{
							//Nao_Communication.audio_buffer_vector -- vector of chars
							Nao_Communication.voiceRecord(true, Nao_Communication.audio_buffer_vector);
							if (Nao_Communication.microphoneEnergy("front")> 2700)
							{
								iterations=0; //reseting the waiting time
							}
							else
							{
								iterations++;
								sleep(0.085); //waiting //# microphone buffer = 170ms
							}
						}	
						else
						{
							
							Nao_Communication.voiceRecord(false, Nao_Communication.audio_buffer_vector);//ends the recording
							std::cout<<"iterations = "<<iterations<<std::endl;
							iterations++;
							end = true;
						}

						

					}
					once=false;
				}
				
			}
			std::cout<<"buffer size = "<<Nao_Communication.audio_buffer_vector.size()<<std::endl;
			
			/*
			fstream myfile;
  			myfile.open ("file_path1.raw", ios::in | ios::out | ios::app );//| ios::binary
  			for(int i=0;i<Nao_Communication.audio_buffer_vector.size();i++)
  			{
  				//for(int j=0;j<BUFSIZE;j++)
  				{
  					myfile << Nao_Communication.audio_buffer_vector[i];//.ch;//[j];
  					//std::cout<<i<<std::endl;
  					//std::cout<<audioBuffer[i]<<std::endl;
  				}
  			}
  			myfile.close();
			*/

			//Nao_Communication.voiceRecord("voice_record", 2700, waitingTime, audioBuffer); // recording the message until the silence will last for waitingTime[s] // will return raw files
			// sending audioBuffer

			//Nao_Communication.voiceRecord("voice_record", 2700, waitingTime, vectorFileName); // recording the message until the silence will last for waitingTime[s] // will return raw files
			/*message = "Message recorded into the file from '" + vectorFileName.front() + "' to '" + vectorFileName.back();
			Nao_Communication.textToSpeech( message, "English" );
			for (size_t i = 0; i < vectorFileName.size(); i++ )
			{
				// audio file streaming
				// send vectorFileName[i];
				std::cout<<vectorFileName[i]<<std::endl;
				fflush(stdout);
			}*/
				
		}	
	}
	
	while( check == true)
	{
		frame = NAO_Vision.captureImage("top - adaptive auto exposure 2",2);//for hightlights --2; weighted average scene Brightness--1; Adaptive weighted auto exposure for lowlights--3
		robotToCameraMatrix.release();//clears the robotToCameraMatrix
		robotToCameraMatrix = NAO_Vision.getTransform("CameraTop", 2); //From Camera to waist (robot) coordinate system; FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2
		if (frame.height == 0 || frame.width == 0) //frame is empty
		{
			std::cout<<"Camera frame is empty"<<std::endl;
			count+=1;
			if (count>4) check=false;
		}
		else
		{

			std::cout<<"QRcode detection ..."<<std::endl;
			QRcodeDetectionStruct.clear();
			QRcodeDetectionStruct = NAO_Vision.qrCodeDetection(frame, scanner, robotToCameraMatrix);
			//std::cout<<QRcodeDetectionStruct.QRmessage.size();

			for (int i=0; i<QRcodeDetectionStruct.QRmessage.size(); i++)
			{
				std::cout<<"QRcode message: \t"<<QRcodeDetectionStruct.QRmessage[i]<<std::endl;
		//		std::cout<<"QRcode in robot coordinate system:\n"<<QRcodeDetectionStruct.LandmarkInRobotCoordinate[i]<<std::endl;
			}
		}
	}
   return 0;
}
