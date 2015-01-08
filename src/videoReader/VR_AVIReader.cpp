/*
 * SL_AVIReader.cpp
 *
 *  Created on: 2012-4-10
 *      Author: elezoud
 */
#include "VR_AVIReader.h"
#include "SL_error.h"

AVIReader::~AVIReader(){
	if(videoCap)
		cvReleaseCapture(&videoCap);
}

bool AVIReader::open() {
	if (videoCap)
		cvReleaseCapture(&videoCap);
	videoCap = cvCaptureFromFile(filePath.c_str());
	if (!videoCap){
		repErr("Cannot open %s\n", filePath.c_str());
		return false;
	}
	_w = (int) cvGetCaptureProperty(videoCap, CV_CAP_PROP_FRAME_WIDTH);
	_h = (int) cvGetCaptureProperty(videoCap, CV_CAP_PROP_FRAME_HEIGHT);

	//assert(_w > 0 && _h > 0);
	if (_w <=0 || _h <= 0){
			repErr("ERROR: Fail to open the USB camera \n");
			return false;
	}
	_timestamp = 0;
	return true;
}

bool AVIReader::grabFrame() {
	assert(videoCap);
	return cvGrabFrame(videoCap);
}
//skip: setting skip frame numbers can only be used with video
void AVIReader::skip(int nSkippedFrame) {
	cvSetCaptureProperty(videoCap, CV_CAP_PROP_POS_FRAMES, nSkippedFrame);
}
void AVIReader::grabReadFrame(int Frame, unsigned char* imgdata) {
	skip(Frame);
	grabFrame();
	readCurFrameRGB(imgdata);
}
void AVIReader::readCurFrameRGB(unsigned char* imgdata) {
	assert(videoCap);
	IplImage* img = cvRetrieveFrame(videoCap);
	memcpy(imgdata, img->imageData, _w * _h * 3);
}
void AVIReader::readCurFrameGray(unsigned char* grayImgData) {
	assert(videoCap);
	IplImage* img = cvRetrieveFrame(videoCap);
	cv::Mat rawFrame(img);
	cv::Mat videoFrame(_h, _w, CV_8UC1, grayImgData);
	cv::cvtColor(rawFrame, videoFrame, CV_RGB2GRAY);
}
void AVIReader::readCurFrame(unsigned char* rgbdata,
		unsigned char* graydata) {
	assert(videoCap);
	IplImage* img = cvRetrieveFrame(videoCap);
	cv::Mat rawFrame(img);

	cv::Mat rgbImg(_h, _w, CV_8UC3, rgbdata);
	cv::cvtColor(rawFrame, rgbImg, CV_BGR2RGB);

	cv::Mat videoFrame(_h, _w, CV_8UC1, graydata);
	cv::cvtColor(rawFrame, videoFrame, CV_RGB2GRAY);
}
int AVIReader::getTotalFrame() {
	return (int)cvGetCaptureProperty(videoCap, CV_CAP_PROP_FRAME_COUNT);
}

void AVIReader::releaseCamera(){
	cvReleaseCapture(&videoCap);
}

double AVIReader::getTimeStamp(){
	return _timestamp++;
}
