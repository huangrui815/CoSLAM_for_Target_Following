/*
 * VR_ROSReader.cpp
 *
 *  Created on: Dec 9, 2014
 *      Author: rui
 */

#include "VR_ROSReader.h"
//#include "SL_error.h"

ROSReader::~ROSReader(){

}

bool ROSReader::open(){
}

bool ROSReader::grabFrame(){
	if(_imgs.empty()){
		return false;
	}
	_imgs.back().copyTo(_currImg);
	_currTs = _ts.back();
//	_imgs.pop_front();
//	_ts.pop_front();

	if (!_opened){
		_w = (int) _currImg.cols;
		_h = (int) _currImg.rows;
		_opened = true;
	}
	return true;
}


void ROSReader::readCurFrameRGB(unsigned char* imgdata) {
//	memcpy(imgdata, _currImg.data, _w * _h * 3);
}

void ROSReader::readCurFrameGray(unsigned char* grayImgData) {
//	cv::Mat videoFrame(_h, _w, CV_8UC1, grayImgData);
//	cv::cvtColor(_currImg, videoFrame, CV_RGB2GRAY);
	memcpy(grayImgData, _currImg.data, _w * _h);
}

void ROSReader::readCurFrame(unsigned char* rgbdata,
		unsigned char* graydata) {

//	cv::Mat rgbImg(_h, _w, CV_8UC3, rgbdata);
//	cv::cvtColor(_currImg, rgbImg, CV_BGR2RGB);

//	cv::Mat videoFrame(_h, _w, CV_8UC1, graydata);
//	cv::cvtColor(rgbImg, videoFrame, CV_RGB2GRAY);
	memcpy(graydata, _currImg.data, _w * _h);
}

double ROSReader::getTimeStamp(){
//	double ts = _tm.get_pass_time();
//	return (uint32_t)(ts+0.5);
	return _currTs;
}




