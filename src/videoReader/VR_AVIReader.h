/*
 * SL_AviReader.h
 *
 *  Created on: 2010-11-7
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#ifndef SL_AVIREADER_H_
#define SL_AVIREADER_H_
#include <string>
#include "VR_VideoReader.h"

class AVIReader:public VideoReader {
protected:
	uint32_t _timestamp;
	CvCapture* videoCap;
public:
	std::string filePath;
public:
	AVIReader() :
		videoCap(0){
			avi = true;
	}
	virtual ~AVIReader();
	virtual bool open();
	virtual bool grabFrame();
	virtual void readCurFrameRGB(unsigned char* imgdata);
	virtual void readCurFrameGray(unsigned char* grayImgData);
	virtual void readCurFrame(unsigned char* rgbdata, unsigned char* graydata);
	virtual double getTimeStamp();
	virtual void skip(int nSkippedFrame);
	virtual int getTotalFrame();

	void grabReadFrame(int Frame, unsigned char* imgdata);
	void releaseCamera();
};
#endif /* SL_AVIREADER_H_ */
