#include "CVKLTTracker.h"
#include "opencv2/opencv.hpp"
#include "opencv2/nonfree/features2d.hpp"

using namespace cv;

CVKLTTracker::CVKLTTracker(){
	_blkWidth = 5;
	_pyLevel = 0;
	_stopEPS = 0.01f;
	_maxErr = 100000;
	_positionChangeMax = 1000000;
	_speedChangeMax = 200000000.0;
	_bLost = false;
	_recoverFrmNum = 0;
	_trackingDistanceConstraint = 45 *45;
									//30 * 30; //for tsing hua videos (752 x 480)
	                             // 400; for ardrone video (630 x 360)
}
CVKLTTracker::~CVKLTTracker() {

}
void CVKLTTracker::_genPointMask(ImgG& mask) {
	mask.resize(gray.cols, gray.rows);
	mask.fill(255);
	cv::Mat cvMask(mask.m, mask.n, CV_8UC1, mask.data);

	int margin = 20;
	for (int x = 0; x < gray.rows; x++){
		for (int y = 0; y < gray.cols; y++){
			int id = y + x * gray.cols;
			if (x < margin || x > gray.rows - margin || y < margin || y > gray.cols - margin){
				cvMask.data[id] = 0;
			}
			else
				cvMask.data[id] = 255;
		}
	}

	for (int i = 0; i < KLT_MAX_FEATURE_NUM; i++) {
		if (_flag[i] >= 0) {
			cv::circle(cvMask,
					cv::Point2f(_featPts[2 * i], _featPts[2 * i + 1]),
					20, cv::Scalar(0), -1);
		}
	}
}

void CVKLTTracker::_advanceFrame() {
//	memcpy(_img1.data, _img2.data, _img2.w * _img2.h * sizeof(uchar));
	std::swap(points[0], points[1]);
	cv::swap(prevGray, gray);
	_oldFeatPts.cloneFrom(_featPts);
}

void CVKLTTracker::open(int width, int height, int blkWidth /* = 12 */,int minDist,
		int pyLevel /* = 4 */, slfloat stopEPS /* = 1 */) {

//	_img1.resize(height, width);
//	_img2.resize(height, width);

	_blkWidth = blkWidth;
    _minDist = minDist;
	_pyLevel = pyLevel;
	_stopEPS = stopEPS;

	_oldFeatPts.resize(KLT_MAX_FEATURE_NUM, 2);
	_featPts.resize(KLT_MAX_FEATURE_NUM, 2);

	_flag.resize(KLT_MAX_FEATURE_NUM, 1);
	_flagMapped.resize(KLT_MAX_FEATURE_NUM, 1);
	_flagMapped.fill(0);
	_numMappedTracks = 0;
	_oldFeatPts.fill(-1);

	gray.create(height, width, CV_8UC1);
	prevGray.create(height, width, CV_8UC1);
	grayBackup.create(height, width, CV_8UC1);
	_detectMask.create(gray.rows, gray.cols, CV_8UC1);
	int margin = 20;
	for (int x = 0; x < gray.rows; x++){
		for (int y = 0; y < gray.cols; y++){
			int id = y + x * gray.cols;
			if (x < margin || x > gray.rows - margin || y < margin || y > gray.cols - margin){
				_detectMask.data[id] = 0;
			}
			else
				_detectMask.data[id] = 1;
		}
	}
}

void CVKLTTracker::detect(const ImgG& img) {
//	memcpy(_img2.data, img.data, _img2.w * _img2.h * sizeof(uchar));

	memcpy(gray.data, img.data, img.h * img.w * sizeof(uchar));

//	DetectionParam param;
//	param.blockSize = _blkWidth;
//	param.minDistance = _minDist;
//	GoodFeaturesToTrackDetector detector(param.maxCorners, param.qualityLevel,
//			param.minDistance, param.blockSize, param.useHarrisDetector,
//			param.k);
	const int MAX_COUNT = KLT_MAX_FEATURE_NUM;
	TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
	Size subPixWinSize(10,10), winSize(31,31);
	goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, _detectMask, 3, 0, 0.04);
	cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);

//	Mat cvImg(_img2.m, _img2.n, CV_8UC1, _img2.data);
//	vector<KeyPoint> keyPts;
//	detector.detect(cvImg, keyPts);
//
//	int npts = (int) keyPts.size();
	int npts = points[1].size();

	for (int i = 0; i < KLT_MAX_FEATURE_NUM; i++) {
		if (i < npts) {
			_featPts[2 * i] = points[1][i].x;
			_featPts[2 * i + 1] = points[1][i].y;
			_flag[i] = KLT_NEWLY_DETECTED;
			pts2trackId.push_back(i);
		} else {
			_flag[i] = KLT_INVALID;
		}
	}
	_advanceFrame();
}

int CVKLTTracker::_track(const ImgG& img, int& nTracked) {
	assert(_flag.m == KLT_MAX_FEATURE_NUM && _featPts.m == KLT_MAX_FEATURE_NUM);
    assert(!_featPts.empty());

//	memcpy(_img2.data, img.data, _img2.w * _img2.h * sizeof(uchar));
    memcpy(gray.data, img.data, img.h * img.w * sizeof(uchar));
//    Mat gray(img.h, img.w, CV_8UC1, img.data);

//	Mat cvImg1(_img1.h, _img1.w, CV_8UC1, _img1.data);
//	Mat cvImg2(_img2.h, _img2.w, CV_8UC1, _img2.data);

//	cv::imshow("img1", cvImg1);
//	cv::imshow("img2", cvImg2);
//	cv::waitKey(-1);

    if (points[0].empty())
    	return -1;

//	int npts = 0;
//	vector<int> ids;
//	for (int i = 0; i < KLT_MAX_FEATURE_NUM; i++) {
//		if (_flag[i] >= 0) {
//			npts++;
//			ids.push_back(i);
//		}
//	}
//
//	Mat cvPrePts(npts, 2, CV_32FC1);
//	Mat cvNextPts(npts, 2, CV_32FC1);
	vector<uchar> status;
	vector<float> err;

	if(prevGray.empty())
		gray.copyTo(prevGray);

	TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
	Size subPixWinSize(10,10), winSize(31,31);
	calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
										 3, termcrit,cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);

	Mat_i _oldFlag;
	_oldFlag.cloneFrom(_flag);

	_flag.fill(-1);

	nTracked = 0;
	int nTrackedMapped = 0;
	int prevFeatNum = points[0].size();

	size_t i,k;
	int numErr = 0;
	int numFlow = 0;
	vector<int> pts2trackId_backup = pts2trackId;
	for (int i = k = 0; i < points[1].size(); i++) {
		int ii = pts2trackId[i];
		double flowDist = pow(points[1][i].x - _oldFeatPts.data[2 * ii],2) +
				pow(points[1][i].y - _oldFeatPts[2 * ii + 1], 2);

		int margin = 20;

		if ( points[1][i].x < margin || points[1][i].x > gray.cols - margin
				|| points[1][i].y < margin || points[1][i].y > gray.rows - margin){
			status[i] = 0;
		}

		if (status[i] == 0)
			numErr++;
		if (flowDist >= _trackingDistanceConstraint)
			numFlow++;

		if (status[i] != 0 && flowDist < _trackingDistanceConstraint/*&& cvErr[k] < _maxErr*/) {
			_featPts.data[2 * ii] = points[1][i].x;
			_featPts.data[2 * ii + 1] = points[1][i].y;
			pts2trackId[k] = ii;
			points[1][k++] = points[1][i];
			_flag.data[ii] = KLT_TRACKED;
			nTracked++;

			if (_flagMapped[ii] == 1)
				nTrackedMapped++;
		}
		else {
			_featPts.data[2 * ii] = -1;
			_featPts.data[2 * ii + 1] = -1;
			_flag.data[ii] = KLT_INVALID;
		}
	}

	float trackRatio = nTracked * 1.0f / prevFeatNum;
	float trackMappedRatio = nTrackedMapped * 1.0f / _numMappedTracks;

	printf("Track ratio: %f, mapped tracks: %f, numErr: %d, numFlow: %d\n", trackRatio, trackMappedRatio,
			numErr, numFlow);


//////////////////////////////////////////////////////////////////////////////////////
	// If map is tracked

	_featPtsBackup.cloneFrom(_oldFeatPts);
	_flagBackup.cloneFrom(_oldFlag);

	_pts2trackIdBackup.reserve(pts2trackId_backup.size());
	_pts2trackIdBackup = pts2trackId_backup;
	prevGray.copyTo(grayBackup);
	pointsBackup.reserve(points[0].size());
	pointsBackup = points[0];

	if (trackMappedRatio < 0.6){
		return -1;
	}
//	// Map is lost but the tracking is not stable
//	else if (_bLost && trackRatio < 0.9){
//		_recoverFrmNum = 0;
//		return -2;
//	}
//	// Map is lost and the tracking is stable
//	else if (_bLost && trackRatio >= 0.9){
//		_recoverFrmNum += 1;
//		// if the stable state lasts for a while
//		if (_recoverFrmNum < 5)
//			return -3;
//		else
//		{
//			_bLost = false;
//			_recoverFrmNum = 0;
//			nTracked = relocalise();
//			std::cout << "Relocalise: tracked points: " << nTracked << std::endl;
////			return -4;
//		}
//	}

/////////////////////////////////////////////////////////////////////////////////////

//	if (nTracked * 1.0f / prevFeatNum < 0.1){
//		points[1].clear();
//		points[1].resize(points[0].size());
//		_flag.fill(-1);
//		_featPts.fill(-1);
//		nTracked = k = rematch();
//	}

	points[1].resize(nTracked);
	pts2trackId.resize(nTracked);
	return nTracked;
}

int CVKLTTracker::rollBack(){
	//reset the tracker state
	_featPts.fill(-1);
//	_oldFeatPts.copyFrom(_featPtsBackup);
	_flag.fill(-1);
	pts2trackId = _pts2trackIdBackup;
	grayBackup.copyTo(prevGray);
	points[0] = pointsBackup;
	points[1] = points[0];

//	rematch();
}
int CVKLTTracker::rematch(){
	std::vector<KeyPoint> keypoints_1, keypoints_2, temp;
	FILE* pFile = fopen("/home/rui/SFM_data/ARDrone/data01/feat02.txt", "w");
	for (int i = 0; i < points[0].size(); i++){
		  KeyPoint pt(points[0][i].x, points[0][i].y, 6.0);
		  keypoints_1.push_back(pt);
		  fprintf(pFile, "%f %f\n", points[0][i].x, points[0][i].y);
	}
	fclose(pFile);

	FastFeatureDetector detector(20);
	detector.detect( gray, temp);
	for (int i = 0; i < temp.size(); i++){
	  KeyPoint key(temp[i].pt.x, temp[i].pt.y, 6.0);
	  keypoints_2.push_back(key);
	}
	SiftDescriptorExtractor extractor;
	Mat descriptors_1, descriptors_2;

	extractor.compute( prevGray, keypoints_1, descriptors_1 );
	extractor.compute( gray, keypoints_2, descriptors_2 );
	FlannBasedMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match( descriptors_1, descriptors_2, matches );
	std::vector<cv::Point2f> pts1;
	std::vector<cv::Point2f> pts2;

	for( int i = 0; i < matches.size(); i++ )
	{
		//-- Get the keypoints from the good matches
		pts1.push_back( keypoints_1[ matches[i].queryIdx ].pt );
		pts2.push_back( keypoints_2[ matches[i].trainIdx ].pt );
	}
	vector<uchar> status;
//	cv::findHomography(pts1, pts2, CV_RANSAC, 15, status);
	cv::findFundamentalMat(pts1, pts2, FM_RANSAC, 5, 0.99, status);

	///////////////////////////////////////////////////////////////////////////////////////////////

	cv::imwrite("prevGray.jpg", prevGray);
	cv::imwrite("gray.jpg", gray);

	int ii = 0;
	for (vector<DMatch>::iterator matchIter = matches.begin(); matchIter != matches.end();){
		  if (status[ii] == 0)
			  matchIter = matches.erase(matchIter);
		  else
			  matchIter++;
		  ii = ii + 1;
	}

	printf("good match number: %d\n", matches.size());



	  //-- Draw only "good" matches
	Mat img_matches;
	drawMatches( prevGray, keypoints_1, gray, keypoints_2,
	               matches, img_matches, Scalar::all(-1), Scalar::all(-1),
	               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	  //-- Show detected matches
	imshow( "Good Matches", img_matches );

	//  for( int i = 0; i < (int)good_matches.size(); i++ )
	//  { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }

	waitKey(0);

	///////////////////////////////////////////////////////////////////////////////////////////////

	size_t k = 0;
	int nTracked = 0;
	for (size_t i = 0; i < pts1.size(); i++){
		int ii = pts2trackId[matches[i].queryIdx];
		if (status[i] == 0){
			_featPts.data[2 * ii] = -1;
			_featPts.data[2 * ii + 1] = -1;
			_flag[ii] = KLT_INVALID;
			continue;
		}

		_featPts.data[2 * ii] = pts2[i].x;
		_featPts.data[2 * ii + 1] = pts2[i].y;
		pts2trackId[k] = ii;
		points[1][k++] = pts2[i];
		_flag[ii] = KLT_TRACKED;
		nTracked++;
	}
	return k;
}

inline bool _keyPointCompare(const cv::KeyPoint& pt1, const cv::KeyPoint& pt2) {
	return pt1.response > pt2.response;
}
int CVKLTTracker::track(const ImgG& img) {
	int nTracked = 0;
	int trackRes = _track(img, nTracked);
	printf("Tracked result: %d\n", trackRes);
	if (trackRes < 0)
		return trackRes;

	_advanceFrame();

	return nTracked;
}

int CVKLTTracker::trackRedetect(const ImgG& img) {
    //track existing feature points first
	int nTracked;
	int trackRes = _track(img, nTracked);
	printf("Tracked number: %d\n", nTracked);
	if (trackRes < 0)
		return trackRes;

//    DetectionParam param;
//	param.blockSize = _blkWidth;
//	param.minDistance = _minDist;
    
//	GoodFeaturesToTrackDetector detector(param.maxCorners, param.qualityLevel,
//			param.minDistance, param.blockSize, param.useHarrisDetector,
//			param.k);
	TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
	Size subPixWinSize(10,10), winSize(31,31);

	GoodFeaturesToTrackDetector detector(KLT_MAX_FEATURE_NUM, 0.01, 20, 3, 0, 0.04);

//	Mat cvImg(_img2.m, _img2.n, CV_8UC1, _img2.data);

    //generate point masks
	ImgG mask;
	_genPointMask(mask);

	vector<KeyPoint> keyPts;
	Mat cvMask(mask.m, mask.n, CV_8UC1, mask.data);
	detector.detect(gray, keyPts, cvMask);

    //
	int nptsnew = (int) keyPts.size();
	if (nptsnew + nTracked >= KLT_MAX_FEATURE_NUM)
		nptsnew = KLT_MAX_FEATURE_NUM - nTracked;

	points[1].resize(nTracked + nptsnew);
	pts2trackId.resize(nTracked + nptsnew);

	std::nth_element(keyPts.begin(), keyPts.begin() + nptsnew, keyPts.end(),
			_keyPointCompare);

	for (int i = 0, k = 0; i < KLT_MAX_FEATURE_NUM && k < nptsnew; i++) {
		if (_flag[i] < 0) {
			_featPts[2 * i] = keyPts[k].pt.x;
			_featPts[2 * i + 1] = keyPts[k].pt.y;
			_flag[i] = KLT_NEWLY_DETECTED;
			points[1][nTracked + k].x = keyPts[k].pt.x;
			points[1][nTracked + k].y = keyPts[k].pt.y;
			pts2trackId[nTracked + k] = i;
			k++;
		}
	}
	_advanceFrame();

	return nTracked + nptsnew;
}

void CVKLTTracker::reset(const ImgG& img, const Mat_f& featPts){
	_featPts.fill(-1);
	_flag.fill(-1);
	pts2trackId.clear();
	points[0].clear();
	points[1].clear();

	memcpy(gray.data, img.data, img.h * img.w * sizeof(uchar));

	int nTracked = 0;
	for (int i = 0; i < KLT_MAX_FEATURE_NUM; i++) {
		if (i < featPts.m) {
			_featPts[2 * i] = featPts.data[2 * i];
			_featPts[2 * i + 1] = featPts.data[2 * i + 1];
			points[1].push_back(cv::Point2f(featPts.data[2 * i], featPts.data[2 * i + 1]));
			pts2trackId.push_back(i);
			_flag[i] = KLT_NEWLY_DETECTED;
			nTracked++;
		} else {
			_flag[i] = KLT_INVALID;
		}
	}

	ImgG mask;
	_genPointMask(mask);
	Mat cvMask(mask.m, mask.n, CV_8UC1, mask.data);

	GoodFeaturesToTrackDetector detector(512, 0.01, 20, 3, 0, 0.04);
	vector<KeyPoint> temp;
	detector.detect(gray, temp, cvMask);

//	const int MAX_COUNT = KLT_MAX_FEATURE_NUM;
//	TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
//	Size subPixWinSize(10,10), winSize(31,31);
//	goodFeaturesToTrack(gray, temp, MAX_COUNT, 0.01, 10, cvMask, 3, 0, 0.04);
//	cornerSubPix(gray, temp, subPixWinSize, Size(-1,-1), termcrit);

	int nptsnew = (int) temp.size();
	if (nptsnew + nTracked >= KLT_MAX_FEATURE_NUM)
		nptsnew = KLT_MAX_FEATURE_NUM - nTracked;

	points[1].resize(nTracked + nptsnew);
	pts2trackId.resize(nTracked + nptsnew);

	std::nth_element(temp.begin(), temp.begin() + nptsnew, temp.end(),
			_keyPointCompare);

	for (int i = 0, k = 0; i < KLT_MAX_FEATURE_NUM && k < nptsnew; i++) {
		if (_flag[i] < 0) {
			_featPts[2 * i] = temp[k].pt.x;
			_featPts[2 * i + 1] = temp[k].pt.y;
			_flag[i] = KLT_NEWLY_DETECTED;
			points[1][nTracked + k].x = temp[k].pt.x;
			points[1][nTracked + k].y = temp[k].pt.y;
			pts2trackId[nTracked + k] = i;
			k++;
		}
	}
	_advanceFrame();
}

void CVKLTTracker::provideCurrFeatures(const ImgG& img, float* corners){

}
void CVKLTTracker::feedExternPoints(const ImgG& img, const Mat_f& pts) {
	assert(!img.empty());
	for (int i = 0; i < KLT_MAX_FEATURE_NUM; i++) {
		if (i < pts.m) {
			_featPts[2 * i] = pts[2 * i];
			_featPts[2 * i + 1] = pts[2 * i + 1];
			_flag[i] = KLT_NEWLY_DETECTED;
		} else
			_flag[i] = KLT_INVALID;
	}
	_advanceFrame();
}

void CVKLTTracker::readFeatPoints(Mat_d& pts, Mat_i& flag) {
	pts.cloneFrom(_featPts);
	flag.cloneFrom(_flag);
}
