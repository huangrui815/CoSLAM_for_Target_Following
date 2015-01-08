/*
* SL_BaseKLTTracker.h
*
*/
#ifndef BASEKLTTRACKER_H_
#define BASEKLTTRACKER_H_

#include "imgproc/SL_Image.h"
#include "math/SL_Matrix.h"

#define KLT_NEWLY_DETECTED 1
#define KLT_TRACKED 0
#define KLT_INVALID -1
#define KLT_MAX_FEATURE_NUM 3000

class BaseKLTTracker{
public:
	BaseKLTTracker(){}
	virtual ~BaseKLTTracker(){}
	virtual void detect(const ImgG& img) = 0;
	virtual int trackRedetect(const ImgG& img) = 0;
	virtual int track(const ImgG& img) = 0;
	/*feed extern feature points to tracker*/
	virtual void reset(const ImgG& img, const Mat_f& featPts) = 0;
	/*flag: !0 means tracked from previous frame, 1 is newly created and -1 means invalidated track.*/
	virtual void readFeatPoints(Mat_d& featPts, Mat_i& flag) = 0;
};

#endif
