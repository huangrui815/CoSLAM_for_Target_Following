/*
 * VR_ROSReader.h
 *
 *  Created on: Dec 9, 2014
 *      Author: rui
 */

#ifndef VR_ROSREADER_H_
#define VR_ROSREADER_H_

#include "VR_VideoReader.h"
#include <vector>
#include "tools/SL_Tictoc.h"
using namespace std;

class ROSReader:public VideoReader {
protected:
	TimeMeasurer _tm;
	uint32_t _timstamp;
public:
	int _camid;
	list<double> _ts;
	list<cv::Mat> _imgs;
	list<int> _seqs;
	cv::Mat _currImg;
	double _currTs;
	int _currSeq;
	bool _opened;

public:
	ROSReader() :
		_camid(-1){
			avi = false;
			_opened = false;
	}
	ROSReader(int camid):
		_camid(camid){
		avi = false;
		_opened = false;
	}
	virtual ~ROSReader();
	virtual bool open();
	virtual bool grabFrame();
	virtual void readCurFrameRGB(unsigned char* imgdata);
	virtual void readCurFrameGray(unsigned char* grayImgData);
	virtual void readCurFrame(unsigned char* rgbdata, unsigned char* graydata);
	virtual double getTimeStamp();
};




#endif /* VR_ROSREADER_H_ */
