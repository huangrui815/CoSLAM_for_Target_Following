#include "SL_CoSLAM.h"
#include "SL_Relocalization.h"
#include "SL_GlobParam.h"
#include "slam/SL_MapPoint.h"
#include "slam/SL_CoSLAMHelper.h"

#include "geometry/SL_Triangulate.h"

#include "tools/SL_TypeConversion.h"
#include "tools/SL_WriteRead.h"

#include "imgproc/SL_ImageIO.h"
//#include "APP_SynObj.h"

#include <numeric>

bool registerToKeyFrame(int camId, const KeyPose* pose, const ImgG& curImg,
		vector<MapPoint*>& matchedMapPts, Mat_d& matched2DPts) {

	assert(pose && !pose->img.empty());
	assert(!curImg.empty());

	vector<MapPoint*> mappts;
	pose->getStaticMapPoints(mappts);
	Mat_d pts3d;

	vecMapPt2Mat(mappts, pts3d);
	if( pts3d.m  == 0)
		return false;

	Mat_d keyPts(pts3d.m, 2);

	project(pose->K, pose->cam->R, pose->cam->t, pts3d.m,
			pts3d.data, keyPts.data);

	//2.find the correspondences of the projected feature points in the current image
	Mat_d curPts;
	Mat_uc flag;

	int nMatched = trackFeatureKLT(pose->img, curImg, keyPts, curPts, flag,
			23, 5, 0.001);

	if (nMatched < 0.6 * keyPts.m)
		return false;

	matched2DPts.resize(nMatched, 2);
	int k = 0;
	for (int i = 0; i < pts3d.m; i++) {
		if (flag[i] > 0) {
			matched2DPts[2 * k] = curPts[2 * i];
			matched2DPts[2 * k + 1] = curPts[2 * i + 1];
			matchedMapPts.push_back(mappts[i]);
			k++;
		}
	}
	return true;
}

Relocalizer::Relocalizer(CoSLAM* slam_, int camId) :
		_coSLAM(slam_), _camId(camId) {
}

Relocalizer::Relocalizer():
	_coSLAM(0),_camId(-1){
}

void Relocalizer::setCamId(int camId){
	_camId = camId;
}

void Relocalizer::setCoSLAM(CoSLAM* slam_){
	_coSLAM = slam_;
}

void Relocalizer::reset() {
	numTrackedFeatPts.clear();
}

bool Relocalizer::isNumChangeLittle(const deque<int>& nums) {
	float sum = accumulate(nums.begin(), nums.end(), 0.0f);
	float mean = sum / nums.size();

	float std = 0;
	for (size_t i = 0; i < nums.size(); i++) {
		float d = nums[i] - mean;
		std += d * d;
	}

	std = sqrt(std / (nums.size() - 1));

	if (std < RELOC_MAX_NUM_STD)
		return true;
	return false;
}

bool Relocalizer::isCameraSteady() {
	//FeatureTracker& featTracker = slam.tracker.featTracker;
	int camId_ = _camId;
	FeatureTracker& featTracker = _coSLAM->slam[camId_].m_tracker;
	if ((int) numTrackedFeatPts.size() < RELOC_FRM_NUM) {
		numTrackedFeatPts.push_back(featTracker.trackedFeatureNum_);
		printf("numTrackedFeatPts.size() add featTracker.trackedFeatureNum_: %d, %d\n",
				numTrackedFeatPts.size(), featTracker.trackedFeatureNum_);
		return false;
	} else {
		numTrackedFeatPts.pop_front();
		numTrackedFeatPts.push_back(featTracker.trackedFeatureNum_);

		if (isNumChangeLittle(numTrackedFeatPts))
			return true;
	}
	return false;
}

bool Relocalizer::tryToRecover() {
	//CameraTracker& tracker = slam.tracker;
	int camId = _camId;

	SingleSLAM& tracker = _coSLAM->slam[camId];
	enterBACriticalSection();
	KeyPose* kf = searchKeyPosebyThumbImage(SLAMParam::MAX_COST_KEY_FRM);
	leaveBACriticalSection();

	if (!kf)
		return false;

	vector<MapPoint*> matchedMapPts;
	Mat_d matched2DPts, matched2DPts_dist;

	cout << "key frame matched!" << endl;

	bool bReg = registerToKeyFrame(camId, kf, tracker.m_img, matchedMapPts,
			matched2DPts);
	if (!bReg)
		return false;

	cout << "success in registering to the key pose!" << endl;

	tracker.getDistortedCoord(matched2DPts, matched2DPts_dist);

	//add to feature point list
	/*==============(critical section)=================*/
	enterBACriticalSection();
	vector<FeaturePoint*> vecFeatPts;
	int frameId = _coSLAM->curFrame;
	// Remove the tracked feature points in current frame
	tracker.m_featPts.removeFrame(frameId);

	for (int i = 0; i < matched2DPts.m; i++) {
		FeaturePoint* fp = tracker.m_featPts.add(frameId, camId,
				matched2DPts_dist[2 * i], matched2DPts_dist[2 * i + 1],
				matched2DPts[2 * i], matched2DPts[2 * i + 1]);

		assert (matchedMapPts[i]);
		fp->mpt = matchedMapPts[i];
		fp->mpt->lastFrame = _coSLAM->curFrame;
		fp->mpt->state = STATE_MAPPOINT_CURRENT;
		fp->mpt->addFeature(_camId, fp);
		_coSLAM->curMapPts.add(fp->mpt);
		vecFeatPts.push_back(fp);
	}
	leaveBACriticalSection();

	tracker.m_tracker.reset(tracker.m_img, vecFeatPts, _coSLAM->slam[_camId].m_featPts);

	double R[9], t[3];
	return tracker.poseUpdate3D(kf->cam->R, kf->cam->t, R, t,false);
}

KeyPose* Relocalizer::searchKeyPosebyThumbImage(double maxcost) {
	int camId = _camId;

	//get the thumbnail image of the current frame
	ImgG imgThumb;
	getThumbImage(_coSLAM->slam[camId].m_smallImg, imgThumb, _coSLAM->slam[camId].maxThumbW);

	KeyPose* p = 0;
	KeyPose* min_kp = 0;
	double min_cost = DBL_MAX;
	//search the keyframes
	for (p = _coSLAM->slam[camId].m_keyPose.first(); p; p = p->next) {
		assert(!p->imgThumb.empty());
		double cost = compareThumbImage(imgThumb, p->imgThumb);
		if (cost < min_cost) {
			min_cost = cost;
			min_kp = p;
		}
	}

	// search the closest N keyPoses
	for (p = _pastNKeyPoses.first(); p; p = p->next) {
			assert(!p->imgThumb.empty());
			double cost = compareThumbImage(imgThumb, p->imgThumb);
			if (cost < min_cost) {
				min_cost = cost;
				min_kp = p;
			}
	}

	cout << "min cost:" << min_cost << endl;
	if (min_cost < maxcost)
		return min_kp;
	return 0;
}

void Relocalizer::cacheNewKeyPose(SingleSLAM& slam){
	KeyPose* pose = 0;
	CamPoseItem* cam = slam.m_camPos.current();
	pose = new KeyPose(slam.currentFrame(), cam);
	pose->setNumMappedPoints(slam.m_nMappedStaticPts);

	pose->setImage(slam.m_img);
	pose->setSmallImage(slam.m_smallImg, slam.m_smallScale);
	pose->setThumbImage(slam.m_smallImg, slam.maxThumbW);
	pose->setCameraIntrinsic(slam.K.data);
	pose->setFeatPoints(slam.m_featPts.getFrameHead(slam.currentFrame()),
			slam.m_featPts.getFrameTail(slam.currentFrame()));

	if( _pastNKeyPoses.num < RELOC_MAX_CACHE_FRAME)
		_pastNKeyPoses.push_back(pose);
	else{
		delete _pastNKeyPoses.pop_front();
		_pastNKeyPoses.push_back(pose);
	}
}

//#include "tools/GUI_ImageViewer.h"
//#include "tools/SL_DrawCorners.h"
//int klt_main() {
//
//	ImgG I1, I2;
//
//	imread(I1, "/home/tsou/I1.png");
//	imread(I2, "/home/tsou/I2.png");
//
//	Mat_d pts;
//	readMat(pts, "/home/tsou/keypts.txt");
//
//	Mat_d ptsnew;
//	Mat_uc flag;
//	int nMatch = trackFeatureKLT(I1, I2, pts, ptsnew, flag);
//	
//	ImgRGB outImg;
//	drawMatching(I1,pts,I2,ptsnew,outImg);
//	
//	imshow("outImg", outImg);
//	cout << "nMatch:" << nMatch << endl;
//	cv::waitKey(-1);
//	return 0;
//}
