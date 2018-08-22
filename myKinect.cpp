#include "stdafx.h"
#include "myKinect.h"
#include <iostream>

using namespace std;


CKinectBasics::CKinectBasics():
m_pKinectSensor(NULL),
m_pColorFrameReader(NULL),
m_pDepthFrameReader(NULL),
m_pCoordinateMapper(NULL),
m_pColorRGBX(NULL)
{
	colorImg.create(cColorHeight, cColorWidth, CV_8UC4);
	colorImg.setTo(0);
	depthImg.create(cDepthHeight, cDepthWidth, CV_8UC1);
	depthImg.setTo(0);

	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
}

CKinectBasics::~CKinectBasics()
{
	SafeRelease(m_pColorFrameReader);
	SafeRelease(m_pDepthFrameReader);
	SafeRelease(m_pCoordinateMapper);

	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}
	SafeRelease(m_pKinectSensor);
}


HRESULT CKinectBasics::InitializeDefaultSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		IColorFrameSource* pColorFrameSource = NULL;
		IDepthFrameSource* pDepthFrameSource = NULL;
		
		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}

		SafeRelease(pColorFrameSource);
		SafeRelease(pDepthFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		cout << "Kinect initialization failed!" << endl;
		return E_FAIL;
	}

	/*colorImg.create(cColorHeight, cColorWidth, CV_8UC4);
	colorImg.setTo(0);
	depthImg.create(cDepthHeight, cDepthWidth, CV_8UC1);
	depthImg.setTo(0);*/

	return hr;
}

void CKinectBasics::Update()
{
	if (!m_pColorFrameReader || !m_pDepthFrameReader)
	{
		return;
	}

	IColorFrame* pColorFrame = NULL;
	IDepthFrame* pDepthFrame = NULL;

	HRESULT hr = S_OK;

	if (SUCCEEDED(hr))
	{
		hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);
	}

	if (SUCCEEDED(hr))
	{
		UINT16 *depthArray = new UINT16[cDepthHeight * cDepthWidth];
		pDepthFrame->CopyFrameDataToArray(cDepthHeight * cDepthWidth, depthArray);

		uchar* depthData = (uchar*)depthImg.data;
		
		for (int i = 0; i < cDepthHeight*cDepthWidth; ++i)
		{
			*depthData = depthArray[i];
			++depthData;
		}
		delete[] depthArray;

		//cv::imshow("depthImg", depthImg);
	}
	SafeRelease(pDepthFrame);			//frame is must be released to get next frame

	if (SUCCEEDED(hr))
	{
		hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
	}

	if (SUCCEEDED(hr))
	{
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nBufferSize = 0;
		RGBQUAD* pBuffer = NULL;
		
		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}

		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, reinterpret_cast<BYTE**>(&pBuffer));
			}
			else if (m_pColorRGBX)
			{
				pBuffer = m_pColorRGBX;
				nBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
			}
		}
		if (SUCCEEDED(hr))
		{
			colorImg.data = reinterpret_cast<BYTE*>(pBuffer);
			//ProcessColor(pBuffer, cColorWidth, cColorHeight);
			cv::Mat showColorImg;
			cv::resize(colorImg, showColorImg, cv::Size(cColorWidth / 4, cColorHeight / 4));
			//cv::imshow("colorImg", showColorImg);

			//vector<int> markerIds;
			//vector<vector<cv::Point2f> > markerCorners, rejectedCandidates;
			//cv::aruco::DetectorParameters detectParams;

			///*aruco::detectMarkers(inputImg, dictionary, markerCorners, markerIds, detectParams, rejectedCandidates);*/
			//cv::aruco::detectMarkers(showColorImg, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), markerCorners, markerIds);
		}
		
	}
	SafeRelease(pColorFrame);
}

void CKinectBasics::ProcessColor(RGBQUAD* pBuffer, int nWidth, int nHeight)
{
	if (pBuffer && (nWidth == cColorWidth) && (nHeight == cColorHeight))
	{

	}
}