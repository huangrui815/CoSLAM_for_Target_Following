#ifndef OPENCVKLTTRACKER_H_
#define OPENCVKLTTRACKER_H_
#include "CoSL_BaseKLTTracker.h"
#include "SL_Track2D.h"
#include "videoReader/VR_AVIReader.h"
#include "opencv2/ocl/ocl.hpp"
//#include "tracking/BriefExtractor.h"
typedef float slfloat;
using namespace std;
using namespace cv::ocl;

#define KLT_MAX_FEATURE_NUM 1000// 2 cams
//#define KLT_MAX_FEATURE_NUM 500// 3 cams
class BriefExtractor;

class CVKLTTracker : public BaseKLTTracker{
public:
	int _blkWidth;
    int _minDist;
	int _pyLevel;
	slfloat _stopEPS;
	slfloat _maxErr;
	slfloat _positionChangeMax;
	slfloat _speedChangeMax;

	//////////////Detection constant///////////////////////////
	float _trackingDistanceConstraint;

	cv::Mat gray, prevGray, image, grayBackup;
	cv::Mat _detectMask;
	vector<cv::Point2f> points[2], pointsBackup;
	vector<int> pts2trackId, _pts2trackIdBackup;

	Mat_d _oldFeatPts;
	Mat_d _featPts, _featPtsBackup;
	Mat_i _flag, _flagBackup, _flagMapped;

	int _numMappedTracks;

	int _numDynamicFeatPts;

	AVIReader aviReader;

	bool _bLost;
	int _recoverFrmNum;

	int mCount;
	cv::Ptr<cv::FeatureDetector> _goodFeatdetector;
	cv::Ptr<cv::FeatureDetector> _detector;
	cv::Ptr<cv::DescriptorExtractor> _extractor;
	cv::Mat _desc; // descriptors for initial features
	BriefExtractor* mBriefExtractor;
	cv::Mat mDesc;

	PyrLKOpticalFlow d_pyrLK;
	oclMat _oclPrevGray, _oclCurrGray;
	oclMat d_prevPts, d_nextPts, d_status;

	class DetectionParam
	{
	public:
		int maxCorners;
		slfloat qualityLevel;
		slfloat minDistance;
		int blockSize;
		bool useHarrisDetector;
		slfloat k;

		DetectionParam(){
			maxCorners = KLT_MAX_FEATURE_NUM;
			qualityLevel = 0.01f;
			minDistance = 5.0f;
			blockSize = 30;
			useHarrisDetector = false;
			k = 0.04;
		}
	};

protected:
	void _genPointMask(cv::Mat& mask);
	void _advanceFrame();
	int _track(const ImgG& img, int& nTracked);
	int _track_gpu(const ImgG& img, int& nTracked);

	void download(const oclMat& d_mat, vector<cv::Point2f>& vec);
	void download(const oclMat& d_mat, vector<uchar>& vec);

public:
	CVKLTTracker();
	virtual ~CVKLTTracker();
	void open(int width,int height,  int _blkWidth = 3, int minDist = 10, int pyLevel = 3, slfloat stopEPS = 0.001);
	void close();
	
	virtual void detect(const ImgG& img);
	virtual int track(const ImgG& img);
	virtual int trackRedetect(const ImgG& img);
	int rollBack();
	int rematch();

	virtual void reset(const ImgG& img, const Mat_f& featPts);
	void provideCurrFeatures(const ImgG& img, float* corners);

	virtual void feedExternPoints(const ImgG& img, const Mat_f& featPts);
	virtual void readFeatPoints(Mat_d& featPts, Mat_i& flag);
};

#endif
