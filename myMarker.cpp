

#include "stdafx.h"
#include "myMarker.h"


CMarkerBasics::CMarkerBasics()
{
	//markerImg.create(0, 0, CV_8UC1);
	T_CameraToMarker = Eigen::Isometry3d::Identity();
	T_DepthToCamera = Eigen::Isometry3d::Identity();
	T_MarkerToWorld = Eigen::Isometry3d::Identity();
	T_DepthToWorld = Eigen::Isometry3d::Identity();
	dictionary = aruco::getPredefinedDictionary(markerType);
}

CMarkerBasics::~CMarkerBasics()
{

}

void CMarkerBasics::GenerateMarkerFile(int markerId, int markerSize, char* fileName)
{
	Mat markerImg;

	aruco::drawMarker(dictionary, markerId, markerSize, markerImg, 1);

	imwrite(fileName, markerImg);

}

void CMarkerBasics::DetectMarkers(Mat inputImg)
{
	vector<int> markerIds;
	vector<vector<Point2f> > markerCorners, rejectedCandidates;
	aruco::DetectorParameters detectParams;

	aruco::detectMarkers(inputImg, dictionary, markerCorners, markerIds, detectParams, rejectedCandidates);
	//aruco::detectMarkers(inputImg, dictionary, markerCorners, markerIds);
	//cv::aruco::estimatePoseSingleMarkers
	if (markerIds.size() > 0)
	{
		aruco::drawDetectedMarkers(inputImg, markerCorners, markerIds);
	}
	
	vector<Vec3d> rvecs, tvecs;
	aruco::estimatePoseSingleMarkers(markerCorners, 0.211, cameraMatrix, distCoeffs, rvecs, tvecs);					//0.109
	//cout << rvecs.size() << endl << tvecs.size() << endl;
	if (markerIds.size() > 0)
	{
		aruco::drawAxis(inputImg, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 0.1055);
		CalFinalTransformation(rvecs[0], tvecs[0]);
		//cout << T_DepthToWorld.matrix() << endl;
	}
	//aruco::drawAxis(inputImg, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 0.1);

}  


void CMarkerBasics::ReadCameraInfoYaml(char* fileName)
{
	FileStorage cameraInfoFile(fileName, FileStorage::READ);

	cameraInfoFile["cameraMatrix"] >> cameraMatrix;
	cameraInfoFile["distortionCoefficients"] >> distCoeffs;

	cout << "cameraMatrix:" << cameraMatrix << endl
		<< "distortionCoefficients" << distCoeffs << endl;

	//cout << cameraMatrix.at<double>(0, 0) << endl;
	//cameraMatrix.at<double>(0, 0) /= 2;
	//cameraMatrix.at<double>(0, 2) /= 2;
	//cameraMatrix.at<double>(1, 1) /= 2;
	//cameraMatrix.at<double>(1, 2) /= 2;

	//cout << cameraMatrix.type() << endl << cameraMatrix.size() << endl << cameraMatrix.channels() << endl;
}

//void CMarkerBasics::CalTransformation()
//{
//
//}

void CMarkerBasics::GenerateCharucoBoard(int numX, int numY, float lenSquare, float lenMarker, char* fileName, bool writeFlag)
{
	board = aruco::CharucoBoard::create(numX, numY, lenSquare, lenMarker, dictionary);

	Mat boardImg;
	board.draw(Size(600, 500), boardImg, 10, 1);
	
	if (writeFlag)
	{
		imwrite(fileName, boardImg);
	}
	
}


void CMarkerBasics::DetectCharucoBoard(Mat inputImg)
{
	vector<int> markerIds;
	vector<vector<Point2f> > markerCorners;

	aruco::detectMarkers(inputImg, dictionary, markerCorners, markerIds);

	Mat currentCharucoCorners, currentCharucoIds;
	if (markerIds.size() > 0)
	{
		aruco::drawDetectedMarkers(inputImg, markerCorners);
		if (board.ids.size() > 0)
		{
			aruco::interpolateCornersCharuco(markerCorners, markerIds, inputImg, board, currentCharucoCorners, currentCharucoIds);
			if (currentCharucoCorners.total() > 0)
			{
				aruco::drawDetectedCornersCharuco(inputImg, currentCharucoCorners, currentCharucoIds);
			}
		}
		else
		{
			cout << "Create a board first!" << endl;
		}

	}

	imshow("CharucoBoard", inputImg);
	char key = waitKey(10);
	if (key == 'c' && markerIds.size() > 0)
	{
		cout << "Frame captured." << endl;

		allCorners.push_back(markerCorners);
		allIds.push_back(markerIds);

		allCharucoCorners.push_back(currentCharucoCorners);
		allCharucoIds.push_back(currentCharucoIds);
		allImgs.push_back(inputImg);
		imgSize = inputImg.size();
	}

	
}

void CMarkerBasics::CalibrateCharucoBoard(char* fileName, bool writeFlag)
{
	//if (allIds.size() < 1)
	//{
	//	cerr << "Not enough captures for calibration." << endl;
	//	return;
	//}

	if (allCharucoIds.size() < 4)
	{
		cerr << "Not enough captures for calibration." << endl;
	}
	
	vector<Mat> rvecs, tvecs;
	double repError;

	//InputArrayOfArrays _charucoIds = (InputArrayOfArrays)allCharucoIds;
	//InputArrayOfArrays _charucoCorners = (InputArrayOfArrays)allCharucoCorners;
	//cout << (unsigned int)_charucoIds.getMat(0).total() << endl;
	//cout << _charucoCorners.getMat(0).total() << endl;
	repError = aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, board, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs);

	cout << "Rep Error: " << repError << endl;
	cout << "cameraMatrix:" << cameraMatrix << endl
		<< "distortionCoefficients" << distCoeffs << endl;

	if (writeFlag)
	{
		WriteCameraInfo(fileName, repError);
	}
}


bool CMarkerBasics::WriteCameraInfo(char* fileName, double repError)
{
	FileStorage fs(fileName, FileStorage::WRITE);

	if (!fs.isOpened())
		return false;

	fs << "image_width" << imgSize.width;
	fs << "image_height" << imgSize.height;

	fs << "cameraMatrix" << cameraMatrix;
	fs << "distortionCoefficients" << distCoeffs;

	fs << "ReprojectionError" << repError;

	return true;

}


void CMarkerBasics::CalFinalTransformation(Vec3d rvec, Vec3d tvec)
{
	//turn rvec to matrix R
	cv::Mat R;

	cv::Rodrigues(rvec, R);

	Eigen::Matrix3d r;
	cv::cv2eigen(R, r);

	

	Eigen::AngleAxisd angle(r);

	T_CameraToMarker = angle;
	T_CameraToMarker(0, 3) = tvec(0);
	T_CameraToMarker(1, 3) = tvec(1);
	T_CameraToMarker(2, 3) = tvec(2);


	T_DepthToWorld = T_DepthToCamera * T_CameraToMarker * T_MarkerToWorld;

	//cout << T.matrix() << endl;
	//Eigen::Translation<double, 3> trans(tvec(0), tvec(1), tvec(2));



	




}



void CMarkerBasics::ReadTransformMatrix(char* fileNameD2C, char* fileNameM2W)
{
	FileStorage d2c(fileNameD2C, FileStorage::READ);
	//FileStorage m2w(fileNameM2W, FileStorage::READ);

	Mat m1, m2;
	d2c["rotation"] >> m1;
	d2c["translation"] >> m2;

	T_DepthToCamera(0, 0) = m1.at<double>(0, 0);
	T_DepthToCamera(0, 1) = m1.at<double>(0, 1);
	T_DepthToCamera(0, 2) = m1.at<double>(0, 2);
	T_DepthToCamera(1, 0) = m1.at<double>(1, 0);
	T_DepthToCamera(1, 1) = m1.at<double>(1, 1);
	T_DepthToCamera(1, 2) = m1.at<double>(1, 2);
	T_DepthToCamera(2, 0) = m1.at<double>(2, 0);
	T_DepthToCamera(2, 1) = m1.at<double>(2, 1);
	T_DepthToCamera(2, 2) = m1.at<double>(2, 2);

	T_DepthToCamera(0, 3) = m2.at<double>(0, 0);
	T_DepthToCamera(1, 3) = m2.at<double>(1, 0);
	T_DepthToCamera(2, 3) = m2.at<double>(2, 0);

	T_MarkerToWorld = Eigen::Isometry3d::Identity();

	//cameraInfoFile["distortionCoefficients"] >> distCoeffs;

	//cout << "cameraMatrix:" << cameraMatrix << endl
	//	<< "distortionCoefficients" << distCoeffs << endl;

	//cout << cameraMatrix.at<double>(0, 0) << endl;
	//cameraMatrix.at<double>(0, 0) /= 2;
	//cameraMatrix.at<double>(0, 2) /= 2;
	//cameraMatrix.at<double>(1, 1) /= 2;
	//cameraMatrix.at<double>(1, 2) /= 2;

	//cout << cameraMatrix.type() << endl << cameraMatrix.size() << endl << cameraMatrix.channels() << endl;
}

bool CMarkerBasics::WriteTransformMatrix(char* fileName)
{
	FileStorage fs(fileName, FileStorage::WRITE);

	if (!fs.isOpened())
		return false;

	cv::Mat M;
	cv::eigen2cv(T_CameraToMarker.matrix(), M);
	fs << "T_CameraToMarker" << M;

	return true;
}