#ifndef OPENCVKLTTRACKER_H_
#define OPENCVKLTTRACKER_H_
#include "CoSL_BaseKLTTracker.h"
#include "SL_Track2D.h"
#include "videoReader/VR_AVIReader.h"

typedef float slfloat;
using namespace std;
#define KLT_MAX_FEATURE_NUM 750// 2 cams
//#define KLT_MAX_FEATURE_NUM 500// 3 cams

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
	Mat_i _flag_falseStatic;

	int _numMappedTracks;

	int _numDynamicFeatPts;

	AVIReader aviReader;

	bool _bLost;
	int _recoverFrmNum;

	cv::Ptr<cv::FeatureDetector> _goodFeatdetector;
	cv::Ptr<cv::FeatureDetector> _detector;
	cv::Ptr<cv::DescriptorExtractor> _extractor;
	cv::Mat _desc;

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
	void _genPointMask(ImgG& mask);
	void _advanceFrame();
	int _track(const ImgG& img, int& nTracked);
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
