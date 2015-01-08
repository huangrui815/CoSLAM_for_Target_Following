/*
 * SL_USBCamReader.h
 *
 *  Created on: 2012-4-10
 *      Author: elezoud
 */

#ifndef SL_USBCAMREADER_H_
#define SL_USBCAMREADER_H_
#include "VR_VideoReader.h"
#include "tools/SL_Tictoc.h"

class USBCamReader:public VideoReader {
protected:
	CvCapture* videoCap;
	TimeMeasurer _tm;
	uint32_t _timstamp; 
public:
	int _camid;
public:
	USBCamReader() :
		videoCap(0), _camid(-1){
			avi = false;
	}
	USBCamReader(int camid):
		videoCap(0),_camid(camid){
		avi = false;
	}
	virtual ~USBCamReader();
	virtual bool open();
	virtual bool grabFrame();
	virtual void readCurFrameRGB(unsigned char* imgdata);
	virtual void readCurFrameGray(unsigned char* grayImgData);
	virtual void readCurFrame(unsigned char* rgbdata, unsigned char* graydata);
	virtual double getTimeStamp();
};
#endif /* SL_USBCAMREADER_H_ */
