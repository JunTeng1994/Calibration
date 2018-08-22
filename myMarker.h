

#include <Windows.h>
#include <vector>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>



using namespace std;
using namespace cv;


class CMarkerBasics
{

	static const aruco::PREDEFINED_DICTIONARY_NAME markerType = aruco::DICT_4X4_50;



public:

	CMarkerBasics();
	~CMarkerBasics();

	void GenerateMarkerFile(int markerId, int markerSize, char* fileName);
	void DetectMarkers(Mat inputImg);

	void ReadCameraInfoYaml(char* fileName);

	void ReadTransformMatrix(char* fileNameD2C, char* fileNameM2W);

	//void CalTransformation();

	void GenerateCharucoBoard(int numX, int numY, float lenSquare, float lenMarker, char* fileName = "", bool writeFlag = false);
	void DetectCharucoBoard(Mat inputImg);

	void CalibrateCharucoBoard(char* fileName = "", bool writeFlag = false);

	bool WriteCameraInfo(char* fileName, double repError);

	void CalFinalTransformation(Vec3d rvec, Vec3d tvec);

	bool WriteTransformMatrix(char* fileName);

	Mat cameraMatrix, distCoeffs;

	Eigen::Isometry3d T_DepthToCamera;
	Eigen::Isometry3d T_MarkerToWorld;
	Eigen::Isometry3d T_CameraToMarker;
	Eigen::Isometry3d T_DepthToWorld;



private:



	aruco::CharucoBoard board;

	vector<Mat> allCharucoCorners;
	vector<Mat> allCharucoIds;

	vector<vector<vector<Point2f> > >allCorners;
	vector<vector<int> > allIds;
	vector<Mat> allImgs;
	Size imgSize;

	aruco::Dictionary dictionary;



	/*vector<int> markerIds;
	vector<vector<Point2f> > markerCorners, rejectedCandidates;
	aruco::DetectorParameters detectParams;*/

	
	
};