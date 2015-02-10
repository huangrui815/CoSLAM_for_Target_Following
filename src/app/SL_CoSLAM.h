/*
 * CoSLAM.h
 *
 *  Created on: 2011-1-3
 *      Author: Danping Zou
 */

#ifndef COSLAM_H_
#define COSLAM_H_
#include "SL_SingleSLAM.h"
#include "SL_Relocalization.h"

#include "math/SL_Matrix.h"
#include "slam/SL_Define.h"
#include "slam/SL_MapPointList.h"
#include "slam/SL_KeyPoseList.h"
#include "slam/SL_CameraGroup.h"

#include "SL_MergeCameraGroup.h"

#include "SL_CoSLAMRobustBA.h"
#include "tools/SL_Print.h"
#include <deque>
#include <map>
#include <tf2/LinearMath/Transform.h>

using namespace std;

void enterBACriticalSection();
void leaveBACriticalSection();

#define READY_FOR_KEY_FRAME_DECREASE 1
#define READY_FOR_KEY_FRAME_VIEWANGLE 2
#define READY_FOR_KEY_FRAME_TRANSLATION 3

#define SLAM_STATE_READY 0
#define SLAM_STATE_MAP_INIT 1
#define SLAM_STATE_NORMAL 2
#define SLAM_STATE_RELOCATE 3
#define SLAM_STATE_RECOVERED 4
#define SLAM_STATE_LOST -1

class CoSLAM {
public:
	CoSLAM();
	~CoSLAM();
	static CoSLAM* ptr;
	int state[SLAM_MAX_NUM];

	double _scale; // slam unit * _scale = 1 meter
	double scale_global2Cam;
	double R_global2Cam[9];
	double t_global2Cam[3];
	tf2::Transform T_global2Cam;

public:
	//to store dynamic points
	vector<vector<Point3dId> > m_dynPts;
	//for camera grouping
	double viewOverlapCost[SLAM_MAX_NUM * SLAM_MAX_NUM];
	//store the shared feature points
	vector<double> sharedPoints[SLAM_MAX_NUM][SLAM_MAX_NUM];
	vector<double> sharedConvexHull[SLAM_MAX_NUM][SLAM_MAX_NUM];

	CameraGroup m_groups[SLAM_MAX_NUM];
	int m_groupNum;
	int m_groupId[SLAM_MAX_NUM];
	SingleSLAM slam[SLAM_MAX_NUM];
	int _frameId[SLAM_MAX_NUM];

	/* relocalization*/
	Relocalizer* m_relocalizer;

	/* map points*/
	MapPointList curMapPts;
	MapPointList actMapPts;
	MapPointList iactMapPts;
	MapPointList falseMapPts;

	int m_nSkipFrame;
	int m_nInitFrame; //frames used for map initialization ( single camera)
	int m_nFirstFrame; //first frame after calling map initialization
	int numCams; //number of cameras
	int curFrame; //current video frame
	int numActFrm; //number of frames to store active map points

	double m_minViewAngleChange;
	double m_initCamTranslation;
	double m_minCamTranslation;
	double m_mappedPtsReduceRatio;

	int m_lastFrmIntraMapping;
	int m_lastFrmInterMapping;
	int m_firstFrmBundle;
	int m_lastFrmBundle;
	int m_lastFrmGroupMerge;
	int m_lastReleaseFrm;
	int m_mergedgid;
	
	KeyFrameList m_keyFrms;

	enum {
		COSLAM_PARALLEL_INIT = 0,
		COSLAM_PARALLEL_READFRAME = 1,
		COSLAM_PARALLEL_POSEUPDATE = 2
	};

	int m_nStatic;
	int m_nDynamic;
	int m_nStaticFeat[SLAM_MAX_NUM];
	int m_nDynamicFeat[SLAM_MAX_NUM];
	int m_intraCamPoseUpdateFail[SLAM_MAX_NUM];

	//store the unmapped dynamic feature points
	std::vector<FeaturePoint*> m_unMappedDynFtPts[SLAM_MAX_NUM];

	bool m_easyToFail;

	std::map<int, int> m_keyFrameId;
	int m_nKeyFrame;

	//timings
	double m_tmPerStep;
	double m_tmReadFrame;
	double m_tmFeatureTracking;
	double m_tmPoseUpdate;
	double m_tmCameraGrouping;
	double m_tmNewMapPoints;
	double m_tmIntraCamMapping;
	double m_tmMapClassify;
	double m_tmCurMapRegister;
	double m_tmActMapRegister;

	//for relocalization
	int _frmNumAfterReloc;
public:
	void processOneFrame();
public:
	void addCam(const char* calFilePath);
	void addVideo(const char* videoFilePath, const char* calFilePath,
			int startFrame = 0);
	void setSkipFrame(int nSkipFrame) {
		m_nSkipFrame = nSkipFrame;
	}
	void setInitFrame(int nInitFrame) {
		m_nInitFrame = nInitFrame;
	}
	void init(bool bOffline);

	void readFrame();
	bool grabReadFrame();
	bool virtualReadFrame();

	/*map initialization*/
	bool initMap();
	bool initMapSingleCam();
	bool initMapMultiCam();
	bool calibGlobal2Cam();
	bool rigidTransformEsti(vector<cv::Point3f>& ptsA, vector<cv::Point3f>& ptsB,
			double R[9], double t[3]);
	double calibScale(vector<cv::Point3f>& ptsA, vector<cv::Point3f>& ptsB);
	bool transformCamPose2Global(CamPoseItem* cam, double P_global[3], double rpy[3]);

	/*feature tracking*/
	void featureTracking();
	void featureReceiving();

	/*pose estimation*/
	void poseUpdate(bool *bEstPose);
	void parallelPoseUpdate(bool* bEstPose, bool largeErr);
	bool interCamPoseUpdate();

	/*key frame selection*/
	/* get the center of the map points generated in the view #camId*/
	void getCurMapCenterViewFrom(int camId, double center[3]);

	/* check if the number of feature points with corresponding map points decrease
	 * significantly from the last key frame.*/
	bool IsMappedPtsDecreaseBelow(int camId, double ratio);
	/* check if the #camId-th camera have already to insert a new key pose*/
	/* 0 - not ready, 1 - large decreasing speed of the number of mapped static points, 2 - large view change*/
	int IsReadyForKeyFrame(int camId);

	KeyFrame* addKeyFrame(int readyForKeyFrame[SLAM_MAX_NUM]);
	/*generate new map points*/
	int genNewMapPoints(bool& merged);
	int genNewMapPointsInterCam(bool bUseSURF);

	/*camera grouping*/
	void getViewOverlapCosts(double vcosts[SLAM_MAX_NUM * SLAM_MAX_NUM],
			int minOverlapNum, double minOverlapAreaRatio);
	int cameraGrouping(); //group cameras into different groups, return the number of groups

	/* merge camera groups if the cameras from different group merge */
	std::deque<MergeCameraGroup*> m_requestMCGs;
	bool mergeCamGroups(KeyFrame* newFrame);

	/*key frame matching & bundle adjustment*/
	std::deque<RobustBundleRTS*> m_requestBAs;
	bool bundleAdjustment(int keyFrameNum, int maxOldKeyFrmNum);
	bool requestForBA(int keyFrameNum, int maxOldKeyFrmNum, int maxIter,
			int innerMaxIter);

	/*release the memory occupied by the feature points*/
	void releaseFeatPts(int frame);
	void releaseKeyPoseImgs(int frame);

	void mapPointsClassify(double pixelVar);
	void mapStateUpdate();

	/* functions for mapPointsRegister */
	//void getNearestCams(Mat_i& nearCam);
	int searchNearestCamForMapPt(const Mat_d& camDist, const MapPoint* p,
			int iCam);
	void getCurrentCamDist(Mat_d& camDist);
	bool compareFeaturePt(const ImgG& scaledImg1, double imgScale1,
			const ImgG& scaledImg2, double imgScale2, const FeaturePoint* pt1,
			const FeaturePoint* pt2);

	bool staticCheckMergability(const MapPoint* mp, const FeaturePoint* fp,
			double pixelVar);
	bool checkUnify(MapPoint* pt1, MapPoint* pt2, double M[], double cov[],
			double maxRpErr);
	void refineMapPoint(MapPoint* p);

	int getNearestFeatPtForReg(const MapPoint* mp, const CamPoseItem* cam0);
	int currentMapPointsRegister(double pixelErrVar, bool bMerge);

	int curStaticPointsRegInGroup(const CameraGroup& camGroup,
			double pixelErrVar, bool bMerge);
	bool curStaticPointRegInGroup(const CameraGroup& camGroup, Mat_d& camDist,
			MapPoint* p, double pixelErrVar, bool bMerge);

	/* register the dynamic map points in current frame*/
	int curDynamicPointsRegInGroup(const CameraGroup& camGroup,
			double pixelErrVar, bool bMerge);
	bool curDynamicPointRegInGroup(const CameraGroup& camGroup, Mat_d& camDist,
			MapPoint* p, double pixelErrVar, bool bMerge);
	/* register the active map points to each view in a group*/
	int activeMapPointsRegister(double pixelErrVar);
	int activeMapPointsRegisterInGroup(const CameraGroup& camGroup,
			double pixelErrVar);
	bool activeMapPointRegisterInGroup(const CameraGroup& camGroup, MapPoint* p,
			double pixelErrVar);
	void getNumDynamicStaticPoints();

	void getMapPts(int firstFrame, int lastFrame,
			std::vector<MapPoint*>& mapPoints);

	void getAllStaticMapPoints(std::vector<MapPoint*>& mptPts) const;
	/* get all map points on key frames*/
	void getAllStaticMapPtsAtKeyFrms(std::vector<MapPoint*>& mapPoints) const;
	/* get all feature points on the key frames*/
	void getAllFeatPtsAtKeyFrms(int c, std::vector<FeaturePoint*>& featPoints);

	/* adjust current camera poses*/
	void adjustCurrentCamPoses();
	/* store the dynamic points*/
	void storeDynamicPoints();

	int getCurFrameInVideo(int c) const {
		return slam[c].startFrameInVideo + curFrame + 1;
	}
	int getFrameInVideo(int c, int frame) const {
		return slam[c].startFrameInVideo + frame + 1;
	}

	void startRelocalization(int camId);
	bool doRelocalization(int camId);
	void endRelocalization(int camId);
	bool allCamerasNormal();
	bool allCamerasInGroupNormal(int groupId);
public:
	//for debug
	void printCamGroup();
	void pause();
	void drawResultAllViews();
	//for debug
	void exportResultsVer1(const char timeStr[]) const;
	void exportResults(const char timeStr[]) const;
	//void saveCurrentFrame(const char timeStr[]) const;
	void saveCurrentImages(const char* dirPath) const;
};

#endif /* COSLAM_H_ */
