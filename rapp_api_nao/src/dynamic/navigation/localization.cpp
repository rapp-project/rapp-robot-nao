#include <rapp/dynamic/navigation/localization.hpp>
			#include <tf/transform_datatypes.h>

namespace rapp {
namespace dynamic {
  	localization::localization(int argc, char ** argv){
  	}
  	localization::~localization(){
  	}



	geometry_msgs::Pose transform_Pose_to_ROS(rapp::object::Pose rapp_pose){
		geometry_msgs::Pose ros_pose;
		ros_pose.position.x = rapp_pose.position.x;
		ros_pose.position.y = rapp_pose.position.y;
		ros_pose.position.z = rapp_pose.position.z;

		ros_pose.orientation.x = rapp_pose.orientation.x;
		ros_pose.orientation.y = rapp_pose.orientation.y;
		ros_pose.orientation.z = rapp_pose.orientation.z;
		ros_pose.orientation.w = rapp_pose.orientation.w;
		return ros_pose;
	}
	rapp::object::Pose transform_Pose_to_RAPP(geometry_msgs::Pose ros_pose){
		rapp::object::Pose rapp_pose;
		rapp_pose.position.x = ros_pose.position.x;
		rapp_pose.position.y = ros_pose.position.y;
		rapp_pose.position.z = ros_pose.position.z;

		rapp_pose.orientation.x = ros_pose.orientation.x;
		rapp_pose.orientation.y = ros_pose.orientation.y;
		rapp_pose.orientation.z = ros_pose.orientation.z;
		rapp_pose.orientation.w = ros_pose.orientation.w;
		return rapp_pose;
	}
	Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
			  Eigen::Affine3d rx =
			      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
			  Eigen::Affine3d ry =
			      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
			  Eigen::Affine3d rz =
			      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
			  return rz * ry * rx;
			}
std::vector<std::vector<double>> MatrixMul(std::vector<std::vector<double>> vectorsA, std::vector<std::vector<double>> vectorsB ,  int size)
{
	double matrixA[size][size];
	double matrixB[size][size];
	double matrixC[size][size];
	std::vector<std::vector<double>> vectorsC;
	vectorsC.clear();
std::vector<double> row;
		for(int i=0; i<size; i++){
			for(int j=0; j<size;j++){
				matrixA[i][j] = vectorsA[i][j];
				matrixB[i][j] = vectorsB[i][j];

			}
		}	
	
    int i, j, k;

    for(i=0; i<size; i++)                                      // liczy wiersze 
    for(j=0; j<size; j++)                                      // liczy kolumny
    matrixC[i][j]=0;   
		

    for(i=0; i<size; i++)
    for(j=0; j<size; j++)
    for(k=0; k<size; k++)
    matrixC[i][j] += matrixA[i][k] * matrixB[k][j]; 

	row.clear();
	for(int i=0; i<size; i++){
		for(int j=0; j<size;j++){
					
			row.push_back(matrixC[i][j]);
		}
			vectorsC.push_back(row);
			row.clear();
	}		

	// std::cout<< "rozmieszczanie:" <<std::endl;
	// 			std::cout<< "vectorC:" <<std::endl;
	// 	for(int i=0; i<size; i++){
	// 		for(int j=0; j<size;j++){
	// 			std::cout<< vectorsC[i][j] << "  ";
	// 		}
	// 		std::cout<< std::endl;
	// 	}	
		return vectorsC;
}

std::vector<std::vector<double>> computeMatricFromPose(double x,double y,double z,double ox,double oy,double oz){
		
		/*
		# NEW approach
		*/

		std::vector<double> row;

		std::vector<std::vector<double>> Z_c_g;
		row.push_back(cos(oz));
		row.push_back(-sin(oz));
		row.push_back(0);
		Z_c_g.push_back(row);
		row.clear();

		row.push_back(sin(oz));
		row.push_back(cos(oz));
		row.push_back(0);
		Z_c_g.push_back(row);
		row.clear();

		row.push_back(0);
		row.push_back(0);
		row.push_back(1);
		Z_c_g.push_back(row);
		row.clear();

		std::vector<std::vector<double>> Y_c_g;
		row.push_back(cos(oy));
		row.push_back(0);
		row.push_back(sin(oy));
		Y_c_g.push_back(row);
		row.clear();

		row.push_back(0);
		row.push_back(1);
		row.push_back(0);
		Y_c_g.push_back(row);
		row.clear();

		row.push_back(-sin(oy));
		row.push_back(0);
		row.push_back(cos(oy));
		Y_c_g.push_back(row);
		row.clear();

		std::vector<std::vector<double>> X_c_g;
		row.push_back(1);
		row.push_back(0);
		row.push_back(0);
		X_c_g.push_back(row);
		row.clear();

		row.push_back(0);
		row.push_back(cos(ox));
		row.push_back(-sin(ox));
		X_c_g.push_back(row);
		row.clear();

		row.push_back(0);
		row.push_back(sin(ox));
		row.push_back(cos(ox));
		X_c_g.push_back(row);
		row.clear();

			// double new_Z[3][3];
			// double new_Y[3][3];
			// double new_X[3][3];

		// std::cout<< "Z_c_g:" <<std::endl;
		// for(int i=0; i<3; i++){
		// 	for(int j=0; j<3;j++){
		// 		std::cout<< Z_c_g[i][j] << "  ";
		// 		new_Z[i][j] = Z_c_g[i][j];
		// 	}
		// 	std::cout<< std::endl;
		// }	
		// 			std::cout<< std::endl;
		// 	std::cout<< std::endl;

		// std::cout<< "Y_c_g:" <<std::endl;
		// for(int i=0; i<3; i++){
		// 	for(int j=0; j<3;j++){
		// 		std::cout<< Y_c_g[i][j] << "  ";
		// 		new_Y[i][j] = Y_c_g[i][j];

		// 	}
		// 	std::cout<< std::endl;
		// }	
		// 			std::cout<< std::endl;

		// 	std::cout<< std::endl;

		// std::cout<< "X_c_g:" <<std::endl;
		// for(int i=0; i<3; i++){
		// 	for(int j=0; j<3;j++){
		// 		std::cout<< X_c_g[i][j] << "  ";
		// 		new_X[i][j] = X_c_g[i][j];
		// 	}
		// 	std::cout<< std::endl;
		// // }	
		// 			std::cout<< std::endl;

		// 	std::cout<< std::endl;
		// 	// double new_Z[3][3];
		// 	// double new_Y[3][3];
			// double new_X[3][3];
			// * new_Z[0] =  &Z_c_g.at(0)[0];
			// * new_Z[1] =  &Z_c_g.at(1)[0];
			// * new_Z[2] =  &Z_c_g.at(2)[0];

			// * new_Y[0] =  &Y_c_g.at(0)[0];
			// * new_Y[1] =  &Y_c_g.at(1)[0];
			// * new_Y[2] =  &Y_c_g.at(2)[0];

			// * new_X[0] =  &X_c_g.at(0)[0];
			// * new_X[1] =  &X_c_g.at(1)[0];
			// * new_X[2] =  &X_c_g.at(2)[0];

std::vector<std::vector<double>>  ZY;
std::vector<std::vector<double>>   R_c_g;
ZY = MatrixMul(Z_c_g, Y_c_g,  3);
	
R_c_g=MatrixMul(ZY, X_c_g,  3);
		//std::vector<std::vector<double>>  R_c_g = Z_c_g*Y_c_g*X_c_g;

		// std::cout<< "R_c_g:" <<std::endl;
		// for(int i=0; i<3; i++){
		// 	for(int j=0; j<3;j++){
		// 		std::cout<< R_c_g[i][j] << "  ";
		// 	}
		// 	std::cout<< std::endl;
		// }	


		std::vector<std::vector<double>> T_c_g;
		row.clear();
		for(int i=0; i<3; i++){
			for(int j=0; j<3;j++){
							// std::cout<< R_c_g[i][j] << " "<<std::endl;

				row.push_back(R_c_g[i][j]);
			}
			if(i==0)
				row.push_back(x);
			if(i==1)
				row.push_back(y);
			if(i==2)
				row.push_back(z);
			if(i==2)
				row.push_back(1);

				T_c_g.push_back(row);
				row.clear();
		}

		for(int j=0; j<3;j++){
						// std::cout<< R_c_g[i][j] << " "<<std::endl;

			row.push_back(0);
		}
		
		row.push_back(1);
		T_c_g.push_back(row);
		

		// T_c_g.at(0).at(3) = x;
		// T_c_g.at(1).at(3) = y;
		// T_c_g.at(2).at(3) = z;
		// T_c_g.at(3).at(3) = 1;
		// std::cout<< "T_c_g:" <<std::endl;

		// T_c_g.at(3).at(0) = 0;
		// T_c_g.at(3).at(1) = 0;
		// T_c_g.at(3).at(2) = 0;
		// T_c_g.at(3).at(3) = 1;	

		// std::cout<< "T_c_g:" <<std::endl;
		// for(int i=0; i<4; i++){
		// 	for(int j=0; j<4;j++){
		// 		std::cout<< T_c_g[i][j] << "  ";
		// 	}
		// 	std::cout<< std::endl;
		// }		



		return T_c_g;

	}
	rapp::object::Pose getRobotPoseFromQRcodeMap(rapp::object::QRCode3D QRcodes,std::vector<std::vector<float>> camera_to_robot_matrix, rapp::object::QRcodeMap QRmap){

		if (QRcodes.numberOfQRcodes > 0){

			// check if one of the detected QRcode is defined in the QRcodeMap

			int k = 0;
			bool QRcode_found_in_map = false;
			int mapped_qrcode_ordinal_nr = -1;
			int found_qrcode_ordinal_nr = -1;
			while ( !QRcode_found_in_map && k < QRcodes.numberOfQRcodes ) {
				std::vector<std::string>::iterator find_iterator;
				find_iterator = std::find(QRmap.labels.begin(), QRmap.labels.end(), QRcodes.QRmessage[k]);
				if ( find_iterator != QRmap.labels.end() ){
					QRcode_found_in_map = true;
					found_qrcode_ordinal_nr = k;
					mapped_qrcode_ordinal_nr = std::distance(QRmap.labels.begin(),find_iterator);
				}
				else{
					k += 1;
					QRcode_found_in_map = false;
				}
			}
			//check if any of detected QRcodes is stored in the QRmap
			if (mapped_qrcode_ordinal_nr != -1){

			// compute transformation matrix from map origin to detected QRcode using QRcodeMap
			geometry_msgs::Pose QRcode_ROS_pose;
			Eigen::Affine3d map_to_QRcode_transform; 


			// // ... NEW APPROACH
			 tf::Quaternion q(QRmap.poses[mapped_qrcode_ordinal_nr].orientation.x,QRmap.poses[mapped_qrcode_ordinal_nr].orientation.y, QRmap.poses[mapped_qrcode_ordinal_nr].orientation.z, QRmap.poses[mapped_qrcode_ordinal_nr].orientation.w);
			 tf::Matrix3x3 m(q);
			 double roll, pitch, yaw;
			 m.getRPY(roll, pitch, yaw);
			 // std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
			std::vector<std::vector<double>> T_g_qr = computeMatricFromPose(QRmap.poses[mapped_qrcode_ordinal_nr].position.x,QRmap.poses[mapped_qrcode_ordinal_nr].position.y,
			 																		QRmap.poses[mapped_qrcode_ordinal_nr].position.z, roll, pitch, yaw);




			map_to_QRcode_transform.matrix() << T_g_qr[0][0], T_g_qr[0][1], T_g_qr[0][2], T_g_qr[0][3],
												T_g_qr[1][0], T_g_qr[1][1], T_g_qr[1][2], T_g_qr[1][3],
												T_g_qr[2][0], T_g_qr[2][1], T_g_qr[2][2], T_g_qr[2][3],
												T_g_qr[3][0], T_g_qr[3][1], T_g_qr[3][2], T_g_qr[3][3];

			 tf::Quaternion q0(QRmap.poses[0].orientation.x,QRmap.poses[0].orientation.y, QRmap.poses[0].orientation.z, QRmap.poses[0].orientation.w);
			 tf::Matrix3x3 m0(q0);
			 double roll0, pitch0, yaw0;
			 m0.getRPY(roll0, pitch0, yaw0);
			std::vector<std::vector<double>> T_g_qr0 = computeMatricFromPose(QRmap.poses[0].position.x,QRmap.poses[0].position.y,
			 																		QRmap.poses[0].position.z, roll0, pitch0, yaw0);

			// QRcode_ROS_pose = transform_Pose_to_ROS(QRmap.poses[mapped_qrcode_ordinal_nr]);

			// tf::poseMsgToEigen( QRcode_ROS_pose, map_to_QRcode_transform);
				// std::cout<< "QR_matrix1:" <<std::endl;
				// for(int i=0; i<4; i++){
				// 	for(int j=0; j<4;j++){
				// 		std::cout<< T_g_qr0[i][j] << "  ";
				// 	}
				// 	std::cout<< std::endl;
				// }		

				// 				std::cout<< "QR_matrix2:" <<std::endl;
				// for(int i=0; i<4; i++){
				// 	for(int j=0; j<4;j++){
				// 		std::cout<< T_g_qr[i][j] << "  ";
				// 	}
				// 	std::cout<< std::endl;
				// }	

						// 		// QRmap.poses[mapped_qrcode_ordinal_nr];
			// 		// double ax = QRmap.orientation_x[mapped_qrcode_ordinal_nr];
			// 		// double ay = QRmap.orientation_y[mapped_qrcode_ordinal_nr];
			// 		// double az = QRmap.orientation_z[mapped_qrcode_ordinal_nr];

					
			// 		// Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
			// 		// Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
			// 		// Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
			// 		// rotation_from_map = rz * ry * rx;

			// 		// double trans_x_from_map = QRmap.position_x[mapped_qrcode_ordinal_nr];
			// 		// double trans_y_from_map = QRmap.position_y[mapped_qrcode_ordinal_nr];
			// 		// double trans_z_from_map = QRmap.position_z[mapped_qrcode_ordinal_nr];
			// 		// Eigen::Affine3d translation_from_map(Eigen::Translation3d(Eigen::Vector3d(trans_x_from_map,trans_y_from_map,trans_z_from_map)));

			// 		// Eigen::Matrix4d map_to_QRcode_matrix = (translation_from_map * rotation_from_map).matrix();

			// // build transformation matrix from QRcode to robot and invert it 
			std::vector<std::vector<double>> T_r_qr;
			T_r_qr = QRcodes.LandmarkInCameraCoordinate[found_qrcode_ordinal_nr];

	
			Eigen::Matrix4d matrix_robot_to_QRcode;
			Eigen::Vector3d translation_robot_to_QRcode( T_r_qr[0][3],T_r_qr[1][3],T_r_qr[2][3]);

			 matrix_robot_to_QRcode << T_r_qr[0][0], T_r_qr[0][1], T_r_qr[0][2],T_r_qr[0][3],
			  									T_r_qr[1][0], T_r_qr[1][1], T_r_qr[1][2],T_r_qr[1][3],
			  									T_r_qr[2][0], T_r_qr[2][1], T_r_qr[2][2],T_r_qr[2][3],
			  									0,0,0,1;
			  Eigen::Affine3d Affine3d_robot_to_QRcode, Affine3d_T_qr_r;

			  Affine3d_robot_to_QRcode.matrix()=matrix_robot_to_QRcode;
			  Affine3d_T_qr_r = Affine3d_robot_to_QRcode.inverse(); 

			  std::vector<std::vector<double>> T_g_r;
			  //MatrixMul( Affine3d_T_qr_r.matrix(),T_g_qr, T_g_r,4);

				// std::cout<< "KONIEC:" <<std::endl;
				// for(int i=0; i<4; i++){
				// 	for(int j=0; j<4;j++){
				// 		std::cout<< T_g_r[i][j] << "  ";
				// 	}
				// 	std::cout<< std::endl;
				// }		

			  //rotation_matrix_robot_to_QRcode;//.toRotationMatrix();//.matrix() = rotation_matrix_robot_to_QRcode//create_rotation_matrix(1.0, 1.0, 1.0);
			  //r.matrix().rightCols<1>() = translation_robot_to_QRcode;//Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(1,1,2)));

			  //Eigen::Matrix4d m = (t * r).matrix(); // Option 1


			 //Eigen::Matrix3d rotation_matrix_robot_to_QRcode;
			// rotation_matrix_robot_to_QRcode << robot_to_QRcode_vectors[0][0], robot_to_QRcode_vectors[0][1], robot_to_QRcode_vectors[0][2],
			// 									robot_to_QRcode_vectors[1][0], robot_to_QRcode_vectors[1][1], robot_to_QRcode_vectors[1][2],
			// 									robot_to_QRcode_vectors[2][0], robot_to_QRcode_vectors[2][1], robot_to_QRcode_vectors[2][2];

			// Eigen::Quaterniond quaternion_robot_to_QRcode(rotation_matrix_robot_to_QRcode);
			// Eigen::Vector3d translation_robot_to_QRcode( robot_to_QRcode_vectors[0][3],robot_to_QRcode_vectors[1][3],robot_to_QRcode_vectors[2][3]);

			// Eigen::Matrix4d robot_to_QRcode_transform, QRcode_to_robot_transform;
			// robot_to_QRcode_transform.setIdentity();
			// robot_to_QRcode_transform.block<3,3>(0,0) = rotation_matrix_robot_to_QRcode;
			// robot_to_QRcode_transform.rightCols<1>() = translation_robot_to_QRcode;
			// QRcode_to_robot_transform = robot_to_QRcode_transform.inverse(); 


			// compute transformation from map to robot
				// !!!!!!!!!!!!!!!!!!!!!!!!!!!!
			geometry_msgs::Pose robot_in_map_pose_ROS;
			rapp::object::Pose robot_in_map_pose_RAPP;
			Eigen::Affine3d map_to_camera_transform, camera_to_robot_transform, map_to_robot_transform;
			double mT_g_r[4][4];
			for(int i=0; i<4; i++){
			for(int j=0; j<4;j++){
				mT_g_r[i][j] = T_g_r[i][j];

			}
			std::cout<< std::endl;
		} 
		camera_to_robot_transform.matrix() <<  camera_to_robot_matrix[0][0], camera_to_robot_matrix[0][1], camera_to_robot_matrix[0][2],camera_to_robot_matrix[0][3],
			  									camera_to_robot_matrix[1][0], camera_to_robot_matrix[1][1], camera_to_robot_matrix[1][2],camera_to_robot_matrix[1][3],
			  									camera_to_robot_matrix[2][0], camera_to_robot_matrix[2][1], camera_to_robot_matrix[2][2],camera_to_robot_matrix[2][3],
			  									0,0,0,1;
			map_to_camera_transform = map_to_QRcode_transform * Affine3d_T_qr_r;
			map_to_robot_transform = map_to_camera_transform*camera_to_robot_transform;
				// 			std::cout<< "CAMERA:" <<std::endl;
				// for(int i=0; i<4; i++){
				// 	for(int j=0; j<4;j++){
				// 		std::cout<< map_to_camera_transform(i,j) << "  ";
				// 	}
				// 	std::cout<< std::endl;
				// }	
			tf::poseEigenToMsg(map_to_robot_transform, robot_in_map_pose_ROS );
			robot_in_map_pose_RAPP = transform_Pose_to_RAPP(robot_in_map_pose_ROS);

			return robot_in_map_pose_RAPP;
		}else{
				std::cout << "any of detected QRcodes is stored in the QRmap";
		
		}
			// tf::quaternionTFToEigen(&quaternion_robot_to_QRcode)
			// geometry_msgs::Pose QRcode_in_robot_pose;
			// QRcode_in_robot_pose.pose.position.x = robot_to_QRcode_vectors[0][0]

			// Eigen::Matrix4d robot_to_QRcode_matrix;
			// for (int i = 0; i<4;i++){
			// 	for (int j = 0; j<4;j++){
			// 	robot_to_QRcode_matrix(i,j) = robot_to_QRcode_vectors[i][j];
			// 	}
			// }
			
			// QRcode_to_robot_matrix = robot_to_QRcode_matrix.inverse();

			// // compute transformation from map to robot
			// Eigen::Matrix4d map_to_robot_matrix = map_to_QRcode_matrix * QRcode_to_robot_matrix;
		
			// Eigen::Matrix4d robot_to_map_matrix = map_to_robot_matrix.inverse();

			// // ext



		}
		else{
				std::cout << "No QRcodes in passed object";
		}


	}
	rapp::object::QRcodeMap localization::loadQRcodeMap(const char* MapPath){
		tinyxml2::XMLDocument doc;
    	doc.LoadFile( MapPath );
    	int n_QRcodes;

		tinyxml2::XMLElement* element_QRcodes = doc.FirstChildElement("QRcodes");
		element_QRcodes->QueryIntAttribute("list_length",&n_QRcodes);
		rapp::object::Pose Pose;
		std::vector<rapp::object::Pose> vec_Pose;
		std::vector<std::string> vec_Labels;

		tinyxml2::XMLNode* node_QR_1 = doc.FirstChildElement("QRcodes")->FirstChildElement();
		double pos_x, pos_y, pos_z;
		double orient_x,orient_y,orient_z,orient_w;
		std::string qr_label;
		

		tinyxml2::XMLElement* next_element = node_QR_1->ToElement();
		int i=0;
		do{
			next_element->FirstChildElement("position")->QueryDoubleAttribute("x",&pos_x);
			next_element->FirstChildElement("position")->QueryDoubleAttribute("y",&pos_y);
			next_element->FirstChildElement("position")->QueryDoubleAttribute("z",&pos_z);

			Pose.position.x = pos_x;
			Pose.position.y = pos_y;
			Pose.position.z = pos_z;

			next_element->FirstChildElement("orientation")->QueryDoubleAttribute("x",&orient_x);
			next_element->FirstChildElement("orientation")->QueryDoubleAttribute("y",&orient_y);
			next_element->FirstChildElement("orientation")->QueryDoubleAttribute("z",&orient_z);
			next_element->FirstChildElement("orientation")->QueryDoubleAttribute("w",&orient_w);

			Pose.orientation.x = orient_x;
			Pose.orientation.y = orient_y;
			Pose.orientation.z = orient_z;
			Pose.orientation.w = orient_w;

			qr_label = next_element->Attribute("label");

			Pose.orientation.x = orient_x;

			vec_Pose.push_back(Pose);
			vec_Labels.push_back(qr_label);

			next_element = node_QR_1->NextSiblingElement();
			i++;
		}while(i<n_QRcodes);

		rapp::object::QRcodeMap QRmap;
		QRmap.labels = vec_Labels;
		QRmap.poses = vec_Pose;
		return QRmap;
	}
		rapp::object::Pose localization::qrCodeLocalization(rapp::object::QRCode3D QRcodes,std::vector<std::vector<float>> camera_to_robot_matrix, const char* MapPath){

		rapp::object::QRcodeMap QRmap = localization::loadQRcodeMap(MapPath);
		rapp::object::Pose pose;
		pose = getRobotPoseFromQRcodeMap(QRcodes, camera_to_robot_matrix, QRmap);
		return pose;

	}
	rapp::object::Pose localization::qrCodeLocalization(rapp::object::QRCode3D QRcodes,std::vector<std::vector<float>> camera_to_robot_matrix, rapp::object::QRcodeMap QRmap){
			
		//char *argv2[64];
		// int argc2 =0;
		// rapp::robot::vision Vision_lib;
		// //rappPlatform::robot::VisionDyn VisionDyn_lib(argc2,argv2);//(0,'a');a
		// //rappPlatform::robot::NavigationDynImpl NavigationDyn_lib(argc2,argv2);//(0,'a');
		// auto image = Vision_lib.captureImage(0,3,"png");
		// rapp::object::Matrix2D robot_camera_transform;
		// //robot_camera_transform = Vision_lib.getTransform("CameraTop",0);

		// rapp::object::QRCode3D QRcodes;
		// float landmarkTheoreticalSize = 0.16f;
		// //QRcodes = VisionDyn_lib.qrCodeDetection(image, robot_camera_transform.matrix ,landmarkTheoreticalSize);	

		rapp::object::Pose pose;
		pose = getRobotPoseFromQRcodeMap(QRcodes, camera_to_robot_matrix, QRmap);
		return pose;

	}

} // namespace rapp
} // namespace dynamic

