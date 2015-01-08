/*
.; * SL_AviReader.h
 *
 *  Created on: 2010-11-7
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#ifndef SL_VideoREADER_H_
#define SL_VideoREADER_H_
#include <opencv2/opencv.hpp>
#include <stdint.h>
class VideoReader {
public:
	bool avi;
	int _w,_h;
public:
	VideoReader() :avi(false),_w(0),_h(0) {
	}
	virtual ~VideoReader(){};
	virtual bool open() = 0;
	virtual bool grabFrame() = 0;
	virtual void readCurFrameRGB(unsigned char* imgdata) = 0;
	virtual void readCurFrameGray(unsigned char* grayImgData) = 0;
	virtual void readCurFrame(unsigned char* rgbdata, unsigned char* graydata) = 0;
	virtual double getTimeStamp() = 0;

	virtual void skip(int nSkippedFrame){};
	virtual int getTotalFrame(){return 0;};
};
#endif /* SL_AVIREADER_H_ */
