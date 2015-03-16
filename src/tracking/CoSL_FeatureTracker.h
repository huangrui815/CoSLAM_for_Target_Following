/*
 * GPUKLT.h
 *
 *  Created on: Mar 24, 2011
 *      Author: Danping Zou
 */

#ifndef FEATURE_TRACKER_H_
#define FEATURE_TRACKER_H_


#include "GPUKLT.h"
#include "SL_Track2D.h"
#include "CVKLTTracker.h"

//class Point2D{
//public:
//	Point2D():x(0),y(0),id(0),frmId(0){};
//	Point2D(int x0, int y0, int id0, int frmId0):x(x0), y(y0), id(id0), frmId(frmId0){};
//	float x, y;
//	int id;
//	int frmId;
//};
class PointsInImage{
public:
	PointsInImage(){};
	void clear(){ptMap.clear();};
	void add(int id, FeaturePoint* pt){
		if(ptMap.count(id) == 0)
			ptMap[id] = pt;
	}
	map<int, FeaturePoint*> ptMap;
	vector<FeaturePoint*> pts;
	vector<int> ptsId;
};

class ImagePair{
public:
	ImagePair():frmId0(0), frmId1(0){};
	void clear(){pointPair.clear();pointPair_reverse.clear();frmId0 = 0; frmId1 = 0;};
	void add(int id0, int id1){
		pointPair[id0] = id1;
		pointPair_reverse[id1] = id0;
	}

	map<int, int> pointPair, pointPair_reverse;
	int frmId0, frmId1;
};


class FeatureTracker {
public:
	int frame_;
	int camid_;
	int W_, H_;
	int m_nMaxCorners;
	int trackedFeatureNum_;
	int detectedFeatureNum_;
	double trackRatio;	//# of tracked features / # of feature points in the previous frame
	//intrinsic parameters
	Mat_d K_, invK_, kUnDist_;
	double _kc[5];
	Track2D *m_tks;
//	GPUKLTTracker klt_;
	CVKLTTracker klt_;

	std::vector<FeaturePoint*> _features;
	std::map<unsigned int, int> _featureId2TrackId;
	bool _bLost;

	vector<ImagePair> imgPairs;
	vector<PointsInImage> ptsInImage;
	bool firstTracking;

	float *_corners;
	ImgG oldImg;
	ImgG bufferImg;
	bool _bRetrack;
public:
	void updateTracks(const Mat_d& pts, const Mat_i& flag);
	void updateTracks01(const Mat_d& pts, const Mat_i& flag, const ImgG& img,
				FeaturePoints& ips);
	void updateTracks02(FeaturePoints& ips,
			vector<unsigned int>& featureId,
			vector<float>& features);
public:
	FeatureTracker();
	virtual ~FeatureTracker();
	void setIntrinsicParam(const double* K, const double* invK,
			const double* k_ud, double* kc) {
		K_.cloneFrom(K, 3, 3);
		invK_.cloneFrom(invK, 3, 3);
		kUnDist_.cloneFrom(k_ud, 7, 1);
		memcpy(_kc, kc, 5 * sizeof(double));
	}
public:
	int currentFrame();
	//routines for KLT tracking
	void openGPU(int W, int H,	V3D_GPU::KLT_SequenceTrackerConfig* pCfg);
	void init(int camId, int W, int H,	V3D_GPU::KLT_SequenceTrackerConfig* pCfg);
	
	//detect corners at the first frame to start KLT tracking
	//  frame : start frame
	//	img : 8-bit gray image
	int first(int frame, const unsigned char* imgData, FeaturePoints& ips);
	int first(int f, const unsigned char* imgData,
				std::vector<FeaturePoint*>& pExistFeat, FeaturePoints& ips);

	int next(const unsigned char* imgData, FeaturePoints& ips);
	int next(FeaturePoints& ips,
			vector<unsigned int>& featureId,
			vector<float>& features);

	int feedExternFeatPoints(std::vector<FeaturePoint*>& externFeatPts);

	void rollBack();

	void getFeatPoints(vector<Track2DNode*>& featPts);
	void getCurrFeatures();
	void getMappedFlags();
	void provideCurrFeatures();
	void reset(const ImgG& img, const vector<FeaturePoint*>& vecPts,FeaturePoints& ips);
	void undistorPointNew(double* K, double* kc, double* x, double* x_undist);
	
	int getNumOfFeatPts(){
		return trackedFeatureNum_ + detectedFeatureNum_;
	}
	int getNumDynamicFeats();
	bool readFMatrix(string path);
	bool readFMatrix01(string path);
};
#endif /* GPUKLT_H_ */
