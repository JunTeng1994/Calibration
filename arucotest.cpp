// arucotest.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"

#include "myKinect.h"
#include "myMarker.h"
//#include <cv.h>
//#include <highgui.h>


using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	//cv::Mat markerImage;
	//cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	//cv::aruco::drawMarker(dictionary, 1, 1000, markerImage, 1);
	//cv::imwrite("mark.jpg", markerImage);
	//cv::imshow("Mark", markerImage);
	//cv::waitKey(0);

	//IplImage * test;
	//test = cvLoadImage("D:\\Sample_8.bmp");//图片路径
	//cvNamedWindow("test_demo", 1);
	//cvShowImage("test_demo", test);
	//cvWaitKey(0);
	//cvDestroyWindow("test_demo");
	//cvReleaseImage(&test);

	////cv::Mat inputImage;
	////inputImage = cv::imread("fuck.jpg");
	//////for (int i = 0; i < inputImage.rows; i++)
	//////{
	//////	for (int j = 0; j < inputImage.cols; j++)
	//////	{
	//////		inputImage.at<>
	//////	}
	//////}
	////cv::flip(inputImage, inputImage, 1);
	////cv::Size size;
	////size.height = 378;
	////size.width = 504;
	////cv::Mat _input;
	////cv::resize(inputImage, _input, cv::Size(inputImage.size().width / 4, inputImage.size().height / 4));
	////cout << _input.size() << endl;
	////cv::imshow("in", _input);
	////vector<int> markerIds;
	////vector<vector<cv::Point2f> > markerCorners, rejectedCandidates;
	////cv::aruco::DetectorParameters parameters;
	////cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
	////cv::aruco::detectMarkers(_input, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

	////cv::Mat outputImage = _input.clone();
	////
	////cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
	////cv::imshow("out", outputImage);
	////cv::waitKey(0);



	CKinectBasics myKinect;
	HRESULT hr = myKinect.InitializeDefaultSensor();

	CMarkerBasics myMarker;

	//myMarker.GenerateMarkerFile(1, 1800, "mark_1_1800.jpg");

	myMarker.ReadCameraInfoYaml("CameraInfo.yaml");
	//myMarker.ReadCameraInfoYaml("calib_color.yaml");
	//myMarker.ReadTransformMatrix("calib_pose.yaml", "");
	cout << myMarker.T_DepthToCamera.matrix() << endl;
	//myMarker.GenerateCharucoBoard(5, 7, 0.04, 0.02, "charucoBoard.jpg");
	cout << '\r\r\r' << myMarker.T_DepthToWorld.matrix() << endl;

	Eigen::Isometry3d s1 = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d s2 = Eigen::Isometry3d::Identity();
	
	if (SUCCEEDED(hr))
	{
		while (1)
		{
			myKinect.Update();
			//cv::Mat detectImg = myKinect.colorImg.clone();
			cv::Mat detectImg;

			//myKinect.colorImg.copyTo(buffImg);
			cv::resize(myKinect.colorImg, detectImg, cv::Size(myKinect.colorImg.size().width / 2, myKinect.colorImg.size().height / 2));
			cv::flip(detectImg, detectImg, 1);
			//cv::imwrite("fuck.jpg", detectImg);
			//detectImg = cv::imread("fuck.jpg");
			cv::cvtColor(detectImg, detectImg, CV_BGRA2BGR);
			//cout << detectImg.channels() << endl;
			//cout << detectImg.type() << endl;
			//cv::resize(myKinect.colorImg, detectImg, cv::Size(myKinect.colorImg.cols / 4, myKinect.colorImg.rows / 4));
			if (detectImg.data)
			{
				//myMarker.DetectCharucoBoard(detectImg);

			
				
				//myMarker.DetectMarkers(detectImg);
				//
				//
				//cv::imshow("Detect Markers", detectImg);
				cv::Mat calibImg;
				cv::undistort(detectImg, calibImg, myMarker.cameraMatrix, myMarker.distCoeffs);
				

				myMarker.DetectMarkers(calibImg);

				cv::imshow("Calibrated Image", calibImg);

				
			}

			
			char key = (char)cv::waitKey(10);

			if (key == 's')
			{
				s1 = s2;
				s2 = myMarker.T_DepthToWorld;
				cout << "***************" << endl;
				cout << s2.matrix() << endl;
				Eigen::Isometry3d diffs = s2 * s1.inverse();
				cout << diffs.matrix() << endl;

				cout << sqrt(diffs(0, 3) * diffs(0, 3) + diffs(1, 3) * diffs(1, 3) + diffs(2, 3) * diffs(2, 3)) << endl;
				myMarker.WriteTransformMatrix("tramsformmatrix.yaml");
			}
			if (key == 27)
			{
				break;
			}
		}

		//myMarker.CalibrateCharucoBoard("CameraInfo.yaml",true);
	}
	else
	{
		cout << "Kinect initialization failed!" << endl;
	}

	return 0;
}

