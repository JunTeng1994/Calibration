#pragma once

#include <Kinect.h>
#include <opencv2/opencv.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>


// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

class CKinectBasics
{
	//param of kinect
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;

	static const int		cColorWidth = 1920;
	static const int		cColorHeight = 1080;

public:
	CKinectBasics();
	~CKinectBasics();

	void                    Update();//main processing function
	HRESULT                 InitializeDefaultSensor();
	void					ProcessColor(RGBQUAD* pBuffer, int nwidth, int nHeight);


	cv::Mat colorImg;
	cv::Mat depthImg;


private:
	IKinectSensor*          m_pKinectSensor;//kinect source
	IColorFrameReader*		m_pColorFrameReader;
	ICoordinateMapper*      m_pCoordinateMapper;//transformation
	IDepthFrameReader*      m_pDepthFrameReader;
	RGBQUAD*				m_pColorRGBX;

	

	
};