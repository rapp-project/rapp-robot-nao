#include "VisionImpl.hpp"

namespace rapp {
namespace robot {

VisionImpl::VisionImpl(int argc,char **argv) {
	ros::init(argc, argv,"Vision_client");
	n = new ros::NodeHandle();
}

VisionImpl::~VisionImpl() {
}

rapp::object::picture VisionImpl::captureImage(int camera_id, int camera_resolution, const std::string & encoding)
{
	if (!client_captureImage) {
		ROS_DEBUG("Invalid service client, creating new one...");
		double secs = ros::Time::now().toSec();
		client_captureImage = n->serviceClient<rapp_ros_naoqi_wrappings::GetImage>("rapp_capture_image", true);
		double sec2 = ros::Time::now().toSec();
		ROS_DEBUG("Creating service client took %lf seconds", sec2-secs);
	} else {
		ROS_DEBUG("Service client valid.");
	}

	rapp_ros_naoqi_wrappings::GetImage srv;
	srv.request.request = camera_id;
	srv.request.resolution = camera_resolution;
	sensor_msgs::Image img;
			
	if (client_captureImage.call(srv)) {
		img = srv.response.frame;
		ROS_INFO("[Vision] - Image captured");
	} else {
		//Failed to call service rapp_get_image
		ROS_ERROR("[Vision] - Error calling service rapp_capture_image");
	}

	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception e) {
		ROS_ERROR("[Vision] cv_bridge exception: %s", e.what());
		return rapp::object::picture("");
	}
	//return cv_ptr->image.clone();	
	//
	int size = cv_ptr->image.total() * cv_ptr->image.elemSize();
	rapp::types::byte * bytes = new rapp::types::byte[size];
	std::memcpy(bytes,cv_ptr->image.data,size * sizeof(rapp::types::byte));
	return rapp::object::picture(bytes); //not tested
}


bool VisionImpl::setCameraParam(int camera_id, int camera_parameter_id, int new_value)
{
	client_setCameraParam = n->serviceClient<rapp_ros_naoqi_wrappings::SetCameraParam>("rapp_set_camera_parameter");
	rapp_ros_naoqi_wrappings::SetCameraParam srv;
	srv.request.cameraId = camera_id;
	srv.request.cameraParameterId = camera_parameter_id;
	srv.request.newValue = new_value;
	bool isSet=false;
			
	if (client_setCameraParam.call(srv))
	{
		isSet = srv.response.isSet;
		if (isSet == true)
			ROS_INFO("[Rapp Set Camera Parameter] - New parameter value was set");
		else
			ROS_INFO("[Rapp Set Camera Parameter] - New parameter value wasn't set");
	}
	else
	{
		//Failed to call service rapp_set_camera_parameter
		ROS_ERROR("[Rapp Set Camera Parameter] - Error calling service rapp_set_camera_parameter");
	}
	return isSet;
}

std::vector<uint8_t> VisionImpl::setCameraParams(int camera_id, std::vector<uint32_t> camera_parameter_ids, std::vector<uint32_t> new_values)
{
	client_setCameraParams = n->serviceClient<rapp_ros_naoqi_wrappings::SetCameraParams>("rapp_set_camera_parameters");
	rapp_ros_naoqi_wrappings::SetCameraParams srv;
	srv.request.cameraId = camera_id;
	srv.request.cameraParameterIds = camera_parameter_ids;
	srv.request.newValues = new_values;
	std::vector<uint8_t> isSetList;
	if (camera_parameter_ids.size() == new_values.size()){
	if (client_setCameraParams.call(srv))
	{
		isSetList = srv.response.isSetList;
		for (unsigned int i=0;i<camera_parameter_ids.size();i++)
			if (isSetList[i] == true)
				ROS_INFO("[Rapp Set Camera Parameters] - New parameter value was set");
			else
				ROS_INFO("[Rapp Set Camera Parameters] - New parameter value wasn't set");
	}
	else
	{
		//Failed to call service rapp_set_camera_parameter
		ROS_ERROR("[Rapp Set Camera Parameters] - Error calling service rapp_set_camera_parameter");
	}
	}
	else
		ROS_INFO("[Rapp Set Camera Parameters] - different size of vectors"); //camera_parameter_ids.size()!=new_values.size() 
	return isSetList;
}

rapp::object::Matrix2D VisionImpl::getTransform(std::string chainName, int space)
{
	client_getTransform = n->serviceClient<rapp_ros_naoqi_wrappings::GetTransform>("rapp_get_transform");
	rapp_ros_naoqi_wrappings::GetTransform srv;
	srv.request.chainName = chainName;
	srv.request.space = space;
	cv::Mat transformMatrix = cv::Mat::zeros(4,4,cv::DataType<float>::type);
	std::vector<float> rows;
	rapp::object::Matrix2D transformStruct;
	
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
	
	transformStruct.clear();
	for (int i=0;i<transformMatrix.rows;i++){
		rows.clear();
		for (int j=0;j<transformMatrix.cols;j++){
			rows.push_back(transformMatrix.at<float>(i,j));
		}
		transformStruct.matrix.push_back(rows);
	}
	
	return transformStruct;//tmpVec;
}

} // namespace robot
} // namespace rapp


//######################################################################
//######################################################################


namespace rappPlatform {
namespace robot {

VisionDynImpl::VisionDynImpl(int argc,char **argv) {
	ros::init(argc, argv,"Dynamic_vision_client");
	n = new ros::NodeHandle();
	
	//data from camera calibration
	// Top camera intrinsic matrix -- from camera calibration //for 1280x960
	VisionDynImpl::camera_top_matrix_3[0][0] = 182.0992346 / 0.16;
	VisionDynImpl::camera_top_matrix_3[0][2] = 658.7582;
	VisionDynImpl::camera_top_matrix_3[1][1] = 185.0952141 / 0.16;
	VisionDynImpl::camera_top_matrix_3[1][2] = 484.2186;
	VisionDynImpl::camera_top_matrix_3[2][2] = 1.0;
	VisionDynImpl::camera_top_matrix_3[0][1] = 0.0; VisionDynImpl::camera_top_matrix_3[1][0] = 0.0; VisionDynImpl::camera_top_matrix_3[2][0] = 0.0; VisionDynImpl::camera_top_matrix_3[2][1] = 0.0;
	
	//for 640x480
	VisionDynImpl::camera_top_matrix_2[0][0] = 91.0496173 / 0.16; VisionDynImpl::camera_top_matrix_2[0][2] = 329.3791;
	VisionDynImpl::camera_top_matrix_2[1][1] = 92.5476071 / 0.16; VisionDynImpl::camera_top_matrix_2[1][2] = 242.1093;
	VisionDynImpl::camera_top_matrix_2[2][2] = 1.0;
	VisionDynImpl::camera_top_matrix_2[0][1] = 0.0; VisionDynImpl::camera_top_matrix_2[1][0] = 0.0; VisionDynImpl::camera_top_matrix_2[2][0] = 0.0; VisionDynImpl::camera_top_matrix_2[2][1] = 0.0;

	//for 320x240
	VisionDynImpl::camera_top_matrix_1[0][0] = 0.5*91.0496173 / 0.16; VisionDynImpl::camera_top_matrix_1[0][2] = 0.5*329.3791;
	VisionDynImpl::camera_top_matrix_1[1][1] = 0.5*92.5476071 / 0.16; VisionDynImpl::camera_top_matrix_1[1][2] = 0.5*242.1093;
	VisionDynImpl::camera_top_matrix_1[2][2] = 1.0;
	VisionDynImpl::camera_top_matrix_1[0][1] = 0.0; VisionDynImpl::camera_top_matrix_1[1][0] = 0.0; VisionDynImpl::camera_top_matrix_1[2][0] = 0.0; VisionDynImpl::camera_top_matrix_1[2][1] = 0.0;
	
}

VisionDynImpl::~VisionDynImpl() {
}


//enum cameraID{'TopCamera', 'BottomCamera', '0','1'};
std::vector< std::vector <float> > VisionDynImpl::faceDetect(rapp::object::picture image, int camera_id, int camera_resolution) {

	if(!client_faceDetect){
		ROS_DEBUG("Invalid service client, creating new one...");
		double secs = ros::Time::now().toSec();
		client_faceDetect = n->serviceClient<rapp_ros_naoqi_wrappings::FaceDetect>("rapp_face_detect");
		double sec2 = ros::Time::now().toSec();
		ROS_DEBUG("Creating service client took %lf seconds", sec2-secs);
	} else {
		ROS_DEBUG("Service client valid.");
	}
	
	std::vector< std::vector <float> > FaceDetectVector;//The vector containing both the position of the Center of the faces, and the size of faces in relation to the image
	std::vector< float > CenterOfFace_X;//The position of the center of the face
	std::vector< float > CenterOfFace_Y;//The position of the center of the face
	std::vector< float > FaceSize_X; //the face size in relation to the image
	std::vector< float > FaceSize_Y; //the face size in relation to the image
	
	
	rapp_ros_naoqi_wrappings::FaceDetect srv;
	//srv.request.image = img;
	if(camera_id==0) //cameraId=="top camera" || cameraId=="TopCamera" || cameraId=="0" || cameraId=="Top Camera" || cameraId=="topCamera" || cameraId=="top_camera" || cameraId=="TOP_CAMERA")
		srv.request.cameraId=0; //top camera
	else
		srv.request.cameraId=1; //bottom camera
	//srv.request.cameraId = cameraId;
	srv.request.resolution = camera_resolution;
	//sensor_msgs::Image img;
			
	if (client_captureImage.call(srv)) {
		CenterOfFace_X = srv.response.centerOfFace_X;
		CenterOfFace_Y = srv.response.centerOfFace_Y;
		FaceSize_X = srv.response.faceSize_X;
		FaceSize_Y = srv.response.faceSize_Y;
		
		if (CenterOfFace_X.size()!=0)
			ROS_INFO("[Face Detection] - Face was detected");
			//rectangle(image, cv::Point((int)(CenterOfFace_X[0]-(FaceSize_X[[0]/2)), (int)(CenterOfFace_Y[0]-(FaceSize_Y[0]/2))), cv::Point((int)(CenterOfFace_X[0]+(FaceSize_X[0]/2)), (int)(CenterOfFace_Y[0]+(FaceSize_Y[0]/2))), cv::Scalar(0,255,0), 2); /selecting the first detected face
		
	} else {
		//Failed to call service rapp_get_image
		ROS_ERROR("[Face detection] - Error calling service rapp_face_detect");
	}
	FaceDetectVector.clear();
	FaceDetectVector.push_back(CenterOfFace_X);
	FaceDetectVector.push_back(CenterOfFace_Y);
	FaceDetectVector.push_back(FaceSize_X);
	FaceDetectVector.push_back(FaceSize_Y);
	
	return FaceDetectVector;
}


/*
template<typename _Tp> static  std::vector<std::vector<_Tp> > toVec(const cv::Mat_<_Tp> matIn) {
	std::vector<std::vector<_Tp> > vecOut(matIn.rows);
	for (int i = 0; i < matIn.rows; ++i) {
		vecOut[i].resize(matIn.cols);
		for (int j = 0; j < matIn.cols; ++j) {
			//vecOut[i][j] =  matIn.at<_Tp>(i, j);
			vecOut[i][j] =  matIn.at(i, j);
		}
	}
	return vecOut;
}


template<typename _Tp> static  cv::Mat toMat(const std::vector<std::vector<_Tp> > vecIn) {
	cv::Mat_<_Tp> matOut(vecIn.size(), vecIn.at(0).size());
	for (int i = 0; i < matOut.rows; ++i) {
		for (int j = 0; j < matOut.cols; ++j) {
			matOut(i, j) = vecIn.at(i).at(j);
		}
	}
	return matOut;
}

cv::Mat bytesToMat(rapp::types::byte * bytes,int width,int height)
{
    cv::Mat image = cv::Mat(height,width,CV_8UC3,bytes).clone();
    return image;
}

rapp::types::byte * matToBytes(cv::Mat image)
{
   int size = image.total() * image.elemSize();
   rapp::types::byte * bytes = new rapp::types::byte[size];
   std::memcpy(bytes,image.data,size * sizeof(rapp::types::byte));
   return bytes;
}
*/

//rapp::object::QRCode3D VisionDynImpl::qrCodeDetection(rapp::object::picture image_, rapp::object::Matrix4x4 robotToCameraMatrix, float landmarkTheoreticalSize)
//rapp::object::QRCode3D VisionDynImpl::qrCodeDetection(rapp::object::picture imgFrame, std::vector<std::vector<float>> robotToCameraMatrix, float landmarkTheoreticalSize)
rapp::object::QRCode3D VisionDynImpl::qrCodeDetection(rapp::object::picture imgFrame, cv::Mat robotToCameraMatrix, float landmarkTheoreticalSize)
{
	zbar::ImageScanner set_zbar;

	//landmarkTheoreticalSize=0.16f;
	
	// initializing the structure QRcodeDetectionStruct -- set to default
	rapp::object::QRCode3D QRcodeDetectionStruct;
	if (sizeof(imgFrame)<10) return QRcodeDetectionStruct;
	QRcodeDetectionStruct.clear();

	//cv::Mat cv_frame, frame_grayscale;
	//cv_frame = bytesToMat(imgFrame,width,height);
	
	////cv::Mat from the std::vector<byte>
	cv::Mat cv_mat(imgFrame.bytearray(),true);
	////decoding the image
	//cv::Mat cv_frame(cv::imdecode(cv_mat,1)); //put 0 if you want greyscale
	cv::Mat frame_grayscale(cv::imdecode(cv_mat,1));//decoding the image to the gray scale
	//frame_grayscale = bytesToMat(image_,width,height);
	
	//boost::shared_ptr<void const> tracked_object;
	//cv_frame = cv_bridge::toCvShare(frame_, tracked_object, frame_.encoding)->image; //conversion from sensor_msgs::Image to cv::Mat

	// Convert to grayscale
	//cv::cvtColor(cv_frame, frame_grayscale, CV_BGR2GRAY);
	
	// Obtain image data
	int width = frame_grayscale.cols;
	int height = frame_grayscale.rows;
	uchar *raw = (uchar *)(frame_grayscale.data);

	// // ZBar
	set_zbar.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
	// Wrap image data
	zbar::Image image(width, height, "Y800", raw, width * height);
	// Scan the image for barcodes
	set_zbar.scan(image);

	// Extract results
	int counter = 0;
	
	// Camera Intrinsic Matrix -- from Camera calibration
	double camera_matrix[3][3];
	if (width == 1280 && height == 960){
		for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		camera_matrix[i][j] = camera_top_matrix_3[i][j];
		//cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, camera_top_matrix_3);
	}else if (width == 640 && height == 480){
		for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		camera_matrix[i][j] = camera_top_matrix_2[i][j];
		//cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, camera_top_matrix_2);
	}else if (width == 320 && height == 240){
		for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		camera_matrix[i][j] = camera_top_matrix_1[i][j];
		//cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, camera_top_matrix_1);
	}else
	{
		std::cout << "Wrong camera intrinsic matrix, the position may be computed wrongly" << std::endl;
		for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		camera_matrix[i][j] = camera_top_matrix_2[i][j];
		//cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, rapp::robot::VisionImpl::camera_top_matrix_2);
	}

	for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
		std::vector<cv::Point3f> object_points;
		std::vector<cv::Point2f> pixel_coords;

		pixel_coords.clear();
		pixel_coords.push_back(cv::Point2f(symbol->get_location_x(0), symbol->get_location_y(0)));
		pixel_coords.push_back(cv::Point2f(symbol->get_location_x(1), symbol->get_location_y(1)));
		pixel_coords.push_back(cv::Point2f(symbol->get_location_x(2), symbol->get_location_y(2)));
		pixel_coords.push_back(cv::Point2f(symbol->get_location_x(3), symbol->get_location_y(3)));

		cv::Mat rvec;//(3,1,cv::DataType<float>::type);
		cv::Mat tvec;//(3,1,cv::DataType<float>::type);
		cv::Mat rotationMatrix;//(3,3,cv::DataType<float>::type);
		cv::Mat cameraIntrinsicMatrix;
		cv::Mat distCoeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type);
		cv::Mat rotation_matrix, translation_matrix;
		cv::Mat landmarkToCameraTransform = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
		cv::Mat cameraToLandmarkTransformMatrix = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
		cv::Mat robotToLandmarkMatrix = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
		cv::Mat Rotx_minus90 = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
		cv::Mat Rotz_minus90 = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
		cv::Mat Mat_I = cv::Mat::zeros(4, 4, cv::DataType<double>::type);
		cv::Mat robotToCameraMat;// = cv::Mat(4, 4, cv::CV_32FC1, robotToCameraMatrix);//initialization from the given data
		//cv::Mat robotToCameraMat = cv::Mat::zeros(4, 4, cv::DataType<float>::type);
		robotToCameraMat = robotToCameraMatrix;

		//initialization from the given data
		//for(int i=0;i<4;i++)
		//for(int j=0;j<4;j++)
		//robotToCameraMat.at<float>(i,j)=robotToCameraMatrix[i][j];
		
		//$$$$$$$$$$$$$$$$$$
		std::vector<double> m00, m01, m02, m10, m12, m20, m21, m22, euler1, euler2, euler3;
		//const double PI = 3.14159265359f;

		// Model for SolvePnP // x-> y^
		object_points.clear();

		//z^y<-
		object_points.push_back(cv::Point3f(0, landmarkTheoreticalSize / 2, landmarkTheoreticalSize / 2));
		object_points.push_back(cv::Point3f(0, landmarkTheoreticalSize / 2, -landmarkTheoreticalSize / 2));
		object_points.push_back(cv::Point3f(0, -landmarkTheoreticalSize / 2, -landmarkTheoreticalSize / 2));
		object_points.push_back(cv::Point3f(0, -landmarkTheoreticalSize / 2, landmarkTheoreticalSize / 2));

		// Camera Intrinsic Matrix -- from Camera calibration
		cameraIntrinsicMatrix = cv::Mat(3, 3, cv::DataType<double>::type, camera_matrix);

		cv::solvePnP(cv::Mat(object_points), cv::Mat(pixel_coords), cameraIntrinsicMatrix, distCoeffs, rvec, tvec, false);//, CV_ITERATIVE );

		cv::Rodrigues(rvec, rotationMatrix);


		// landmarkToCameraTransform computation
		for (int i = 0; i<3; i++)
		{
			for (int j = 0; j<3; j++)
				landmarkToCameraTransform.at<double>(i, j) = rotationMatrix.at<double>(i, j);
		}
		landmarkToCameraTransform.at<double>(0, 3) = tvec.at<double>(0, 0); //x->y^ : x
		landmarkToCameraTransform.at<double>(1, 3) = tvec.at<double>(1, 0); //x->y^ : y
		landmarkToCameraTransform.at<double>(2, 3) = tvec.at<double>(2, 0); //x->y^ : z
		landmarkToCameraTransform.at<double>(3, 3) = 1.0;


		Rotx_minus90.at<double>(0, 0) = 1.f;
		Rotx_minus90.at<double>(1, 1) = 0.f; Rotx_minus90.at<double>(1, 2) = +1.f;
		Rotx_minus90.at<double>(2, 1) = -1.f; Rotx_minus90.at<double>(2, 2) = 0.f;
		Rotx_minus90.at<double>(3, 3) = 1.f;

		Rotz_minus90.at<double>(0, 0) = 0.f; Rotz_minus90.at<double>(0, 1) = +1.f;
		Rotz_minus90.at<double>(1, 0) = -1.f; Rotz_minus90.at<double>(1, 1) = 0.f;
		Rotz_minus90.at<double>(2, 2) = 1.f;
		Rotz_minus90.at<double>(3, 3) = 1.f;


		//#####################			
		//## Transformation from the QR-code coordinate system to the camera coordinate system
		cameraToLandmarkTransformMatrix = Rotz_minus90*Rotx_minus90*landmarkToCameraTransform;
		//#####################

		//#####################						
		//## Transformation from the camera coordinate system to the robot coordinate system
		robotToLandmarkMatrix = robotToCameraMat*cameraToLandmarkTransformMatrix;
		//#####################			

		//cv::Mat -> vector<vector<double>>
		//QRcodeDetectionStruct.LandmarkInCameraCoordinate.push_back(cameraToLandmarkTransformMatrix);
		//QRcodeDetectionStruct.LandmarkInRobotCoordinate.push_back(robotToLandmarkMatrix);

		counter++;
		QRcodeDetectionStruct.QRmessage.push_back(symbol->get_data());
		QRcodeDetectionStruct.isQRcodeFound = true;

	}
	QRcodeDetectionStruct.numberOfQRcodes = counter;

	//return QRcodeDetectionStruct; // zmienic
	//rapp::object::QRCode3D QRCodeStruct;
	
	//to do QRcodeDetectionStruct -> QRCodeStruct
	return QRcodeDetectionStruct;
}
} // namespace robot
} // namespace rappPlatform

