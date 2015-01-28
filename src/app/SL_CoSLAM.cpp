/*
 * CoSLAM.cpp
 *
 *  Created on: 2011-1-3
 *      Author: Danping Zou
 */
#include "SL_CoSLAM.h"
#include "SL_InitMap.h"
#include "SL_NewMapPointsInterCam.h"
#include "SL_GlobParam.h"

#include "slam/SL_CoSLAMHelper.h"
#include "SL_MergeCameraGroup.h"
#include "gui/MyApp.h"

#include "tools/SL_Tictoc.h"
#include "tools/GUI_ImageViewer.h"
#include "videoReader/VR_AVIReader.h"

#include "geometry/SL_Triangulate.h"
#include "geometry/SL_Geometry.h"
#include "imgproc/SL_ImageIO.h"
#include "calibration/SL_CalibTwoCam.h"
#include <cassert>
#include <set>
#include <cstdio>

void enterBACriticalSection() {
	pthread_mutex_lock(&MyApp::s_mutexBA);
}
void leaveBACriticalSection() {
	pthread_mutex_unlock(&MyApp::s_mutexBA);
}

CoSLAM* CoSLAM::ptr = 0;
CoSLAM::CoSLAM() :
		m_nSkipFrame(0), m_nInitFrame(0), m_nFirstFrame(0), numCams(0), curFrame(
				0), numActFrm(250), m_minViewAngleChange(5.0), m_initCamTranslation(
				0.0), m_minCamTranslation(0.0), m_mappedPtsReduceRatio(0.93), m_lastFrmIntraMapping(
				0), m_lastFrmInterMapping(0), m_firstFrmBundle(0), m_lastFrmBundle(
				0), m_lastFrmGroupMerge(-1), m_lastReleaseFrm(-1), m_mergedgid(
				-1), m_nStatic(0), m_nDynamic(0), m_easyToFail(false), m_nKeyFrame(
				0),m_relocalizer(0),_frmNumAfterReloc(-1) {

	memset(viewOverlapCost, 0, sizeof(double) * SLAM_MAX_NUM * SLAM_MAX_NUM);
	memset(m_nDynamicFeat, 0, sizeof(int) * SLAM_MAX_NUM);
	memset(m_nStaticFeat, 0, sizeof(int) * SLAM_MAX_NUM);
	memset(m_groupId, 0, sizeof(int) * SLAM_MAX_NUM);

	CoSLAM::ptr = this;
	for (int i = 0; i < SLAM_MAX_NUM; i++)
		state[i] = SLAM_STATE_READY;
}
CoSLAM::~CoSLAM() {
	//clear the queued BAs
	while (!m_requestBAs.empty()) {
		//start of critical section
		enterBACriticalSection();
		RobustBundleRTS* pBA = m_requestBAs.back();
		m_requestBAs.pop_back();
		leaveBACriticalSection();
		//end of critical section
		delete pBA;
	}
	delete [] m_relocalizer;
}
void CoSLAM::processOneFrame() {
	featureTracking();
}
void CoSLAM::addCam(const char* calFilePath){
	slam[numCams].calFilePath = calFilePath;
	slam[numCams].camId = numCams;
	numCams++;
}

void CoSLAM::addVideo(const char* videoFilePath, const char* calFilePath,
		int startFrame) {
	slam[numCams].videoFilePath = videoFilePath;
	slam[numCams].calFilePath = calFilePath;
	slam[numCams].startFrameInVideo = startFrame;
	slam[numCams].camId = numCams;
	numCams++;
}
void CoSLAM::init(bool bOffline) {
	int minFrmNum = 0;
	for (int i = 0; i < numCams; i++) {
		if (bOffline){
		((AVIReader*)slam[i].videoReader)->filePath =slam[i].videoFilePath; 
			slam[i].videoReader->open();
			slam[i].videoReader->skip(slam[i].startFrameInVideo + m_nSkipFrame);
			if (minFrmNum < slam[i].videoReader->getTotalFrame())
				minFrmNum = slam[i].videoReader->getTotalFrame();

			slam[i].startFrameInVideo += m_nSkipFrame;
		}

		pthread_mutex_lock(&MyApp::_mutexImg);
		while(!slam[i].videoReader->grabFrame())
			pthread_cond_wait(&MyApp::_condImg, &MyApp::_mutexImg);
		pthread_mutex_unlock(&MyApp::_mutexImg);

		//////////////////////////////////////////////////////////
		slam[i].m_rgb.resize(slam[i].videoReader->_w, slam[i].videoReader->_h);
		slam[i].m_img.resize(slam[i].videoReader->_w, slam[i].videoReader->_h);
		slam[i].m_img_draw.resize(slam[i].videoReader->_w, slam[i].videoReader->_h);

		slam[i].K.resize(3, 3);
		slam[i].k_c.resize(5, 1);
		readIntrinDistParam(slam[i].calFilePath.c_str(), slam[i].K,
				slam[i].k_c);
		slam[i].iK.resize(3, 3);
		slam[i].k_ud.resize(7, 1);

		matInv(3, slam[i].K.data, slam[i].iK.data);
		invDistorParam(slam[i].videoReader->_w, slam[i].videoReader->_h,
				slam[i].iK.data, slam[i].k_c.data, slam[i].k_ud.data);
		printf("load cam %d intrinsic model: \n", i);
		print(slam[i].K);
	}
	Param::nTotalFrame = minFrmNum;

	//initialize the relocalizer
	m_relocalizer = new Relocalizer[numCams];

	//initialize the camera groups
	m_groupNum = 1;
	for (int i = 0; i < numCams; i++) {
		m_groups[0].addCam(i);
		m_relocalizer[i].setCoSLAM(this);
		m_relocalizer[i].setCamId(i);
	}
}

void CoSLAM::readFrame() {
	for (int i = 0; i < numCams; i++) {
		slam[i].readFirstFrame();
	}
}
void* _parallelReadNextFrame(void* param) {
	int camId = (intptr_t) param;
	CoSLAM::ptr->slam[camId].grabReadFrame();
	return 0;
}
bool CoSLAM::grabReadFrame() {
	TimeMeasurer tm;
	tm.tic();

	//	for (int i = 0; i < numCams; i++) {
	//		slam[i].readNextFrame();
	//	}

	pthread_t threads[SLAM_MAX_NUM];
//	for (int i = 1; i < numCams; i++) {
//		pthread_create(&threads[i], 0, _parallelReadNextFrame, (void*) i);
//	}
	if (!slam[0].grabReadFrame())
		return false;

	for (int i = 1; i < numCams; i++) {
//		pthread_join(threads[i], 0);
		slam[i].grabReadFrame();
	}
	curFrame++;
	m_tmReadFrame = tm.toc();
	return true;
}

bool CoSLAM::virtualReadFrame() {
	TimeMeasurer tm;
	tm.tic();

	for (int i = 0; i < numCams; i++){
//		MyApp::_cap[i].set(CV_CAP_PROP_POS_FRAMES, _frameId[i]);
//		cv::Mat img, gray;
//		MyApp::_cap[i] >> img;
//		cvtColor(img, gray, CV_BGR2GRAY);
//		memcpy(slam[i].m_rgb.data, img.data, img.cols * img.rows *3);
//		memcpy(slam[i].m_img_draw.data, gray.data, img.cols * img.rows);
//		cv::Mat cvImg(slam[i].m_img.rows, slam[i].m_img.cols, CV_8UC1, slam[i].m_img.data);
//		cv::Mat cvSmallImg(slam[i].m_smallImg.rows, slam[i].m_smallImg.cols, CV_8UC1,
//				slam[i].m_smallImg.data);
//		cv::resize(cvImg, cvSmallImg, cv::Size(slam[i].m_smallImg.cols, slam[i].m_smallImg.rows));
	}

	curFrame++;
	m_tmReadFrame = tm.toc();
	ROS_INFO("VirtualReadFrame completed\n");
	return true;
}

bool CoSLAM::initMap() {
	if (numCams == 1)
		return initMapSingleCam();
	return initMapMultiCam();
}
#include "tools/SL_DrawCorners.h"
bool CoSLAM::initMapSingleCam() {
	//1.tracking feature points for a set of frames
	int nFeatPts = slam[0].initTracker(curFrame);
	if (nFeatPts <= 0)
		repErr("not feature point is detected at frame %d.",
				getCurFrameInVideo(0));
	slam[0].grabReadFrame();
	for (int i = 0; i < m_nInitFrame; i++) {
		nFeatPts = slam[0].trackFeaturePoints();
		if (nFeatPts <= 0)
			repErr("not feature point is detected at frame %d.",
					getCurFrameInVideo(0));
		slam[0].grabReadFrame();
		curFrame++;
	}

	m_nFirstFrame = curFrame;
	vector<FeaturePoint*> vecPts1, vecPts2;
	for (int i = 0; i < slam[0].m_tracker.m_nMaxCorners; i++) {
		Track2D& tk = slam[0].m_tracker.m_tks[i];
		if (tk.empty())
			continue;
		if (tk.f1 == 0 && tk.f2 == curFrame) {
			vecPts2.push_back(tk.tail->pt);
			vecPts1.push_back(tk.head.next->pt);
		}
	}

	//2.estimate the initial camera poses
	Mat_d matPts1, matPts2;
	featPoint2Mat(vecPts1, matPts1);
	featPoint2Mat(vecPts2, matPts2);

	CalibTwoCam calib;
	calib.setIntrinParam(slam[0].K.data, slam[0].K.data);
	calib.setDistorParam(slam[0].W, slam[0].H, slam[0].k_c, slam[0].k_c);
	calib.setMatchedPoints(matPts1, matPts2);
	calib.estimateEMat();

	vector<int> inlierInd;
	calib.getInlierInd(inlierInd);

	Mat_d R1, t1, R2, t2, pts1, pts2;
	calib.outputInlierNormPoints(pts1, pts2);
	calib.outputRTs(R1, t1, R2, t2);

	int npts = pts1.m;

	//3. triangulate initial map points
	Mat_d Ms(npts, 3);
	binTriangulatePoints(R1, t1, R2, t2, npts, pts1.data, pts2.data, Ms.data);

	for (int i = 0; i < npts; i++) {
		MapPoint* pM = curMapPts.add(Ms.data + 3 * i, curFrame);
		getBinTriangulateCovMat(slam[0].K, R1, t1, slam[0].K, R2, t2, pM->M,
				pM->cov, Const::PIXEL_ERR_VAR);

		int featId = inlierInd[i];
		pM->addFeature(0, vecPts2[featId]);
		for (FeaturePoint* pm = vecPts2[featId]; pm; pm = pm->preFrame)
			pm->mpt = pM;
		pM->nccBlks[0].computeScaled(slam[0].m_smallImg, slam[0].m_smallScale,
				vecPts2[featId]->m[0], vecPts2[featId]->m[1]);
	}

	//test
	cout << "reconstructed number:" << npts << endl;
	slam[0].m_camPos.add(curFrame, slam[0]._ts, 0, R2.data, t2.data);
	slam[0].initTracker(curFrame);
	m_minCamTranslation = getCameraDistance(R1, t1, R2, t2) * 0.1;

	//4. add key frame
	//add the first frame as a key frame
	KeyFrame* keyFrame = m_keyFrms.add(curFrame);
	keyFrame->setCamNum(numCams);
	keyFrame->setMapPtsNum(curMapPts.getNum());
	keyFrame->setCamGroups(m_groups, m_groupNum);
	keyFrame->setKeyPose(0, slam[0].addKeyPose(true));

	m_nKeyFrame++;
	return true;
}

bool CoSLAM::initMapMultiCam() {
	printf("start initMapMultiCam at frame: %d\n", curFrame);
	InitMap initMapper;
	vector<FeaturePoints*> pFeatPts;
	for (int i = 0; i < numCams; i++) {
		initMapper.addCam(slam[i]);
		pFeatPts.push_back(&slam[i].m_featPts);
	}
	//generate 3D map points 
	if(!initMapper.apply_new_corner(curFrame, pFeatPts, curMapPts)){
		for (int i =0; i < numCams; i++){
			pFeatPts[i]->clear();
		}
		curMapPts.clear();
		return false;
	}


//	initMapper.save(curFrame, "initMapper", pFeatPts, curMapPts);

//	test
//	cout << "The initial map has been saved!" << endl;
//	initMapper.load(curFrame, "initMapper", pFeatPts, curMapPts);
//	updateDisplayData();
//	pause();

	//copy the camera orders
	m_groups[0].setCams(initMapper.camOrder);
	m_groupNum = 1;

	RobustBundleRTS bundler;

	//add the first frame as a key frame
	KeyFrame* keyFrame = m_keyFrms.add(curFrame);
	keyFrame->setCamNum(numCams);
	keyFrame->setMapPtsNum(curMapPts.getNum());
	keyFrame->setCamGroups(m_groups, m_groupNum);

	m_nKeyFrame++;

	for (int i = 0; i < numCams; i++) {
		slam[i].getNumMappedStaticPts();

		CamPoseItem* pCamPos = slam[i].m_camPos.add(curFrame, slam[i]._ts, slam[i].camId,
				initMapper.camR[i], initMapper.camT[i]);

		keyFrame->setKeyPose(i, slam[i].addKeyPose(true));
		//for bundle adjustment
		bundler.addKeyCamera(slam[i].K.data, pCamPos);

		//initialize the GPUKLT tracker
		vector<FeaturePoint*> existFeatPts;
		slam[i].m_featPts.getFrame(curFrame, existFeatPts);
		printf("debug0: m_featPts at frame %d has size: %d\n", curFrame, existFeatPts.size());

		slam[i].initTracker(curFrame, existFeatPts);
		slam[i].sendInitFeaturePoints();
	}

	//set the 3D and 2D correspondences for bundle adjustment
	for (int i = 0; i < numCams; i++) {
		FeaturePoint* pFeatHead = slam[i].m_featPts.hd.next;
		FeaturePoint* pFeatTail = slam[i].m_featPts.tail;
		for (FeaturePoint* p = pFeatHead; p != pFeatTail; p = p->next) {
			if (p->mpt && p->mpt->isLocalStatic() && p->mpt->numVisCam > 1)
				bundler.addCorrespondingPoint(p->mpt, p);
		}
	}

	//call bundle adjustment to refine the initial camera poses
	bundler.setCoSLAM(this);
	bundler.apply(2, 1, 5, 50);

	//test
	//pause();

	//compute average distance between cameras
	m_initCamTranslation = 0;
	int n = 0;
	for (int i = 0; i < numCams; i++) {
		CamPoseItem* pCamPos1 = slam[i].m_camPos.current();
		for (int j = i + 1; j < numCams; j++) {
			CamPoseItem* pCamPos2 = slam[j].m_camPos.current();
			m_initCamTranslation += getCamDist(pCamPos1, pCamPos2);
			n++;
		}
	}
	m_initCamTranslation /= n;
	m_minCamTranslation = m_initCamTranslation / 4.5;

	for (int i = 0; i < numCams; i++) {
		slam[i].updateCamParamForFeatPts(slam[i].K.data,
				slam[i].m_camPos.current());
	}

	double org0[3], org1[3];
	getCamCenter(slam[initMapper.camOrder[0]].m_camPos.current(), org0);
	getCamCenter(slam[initMapper.camOrder[1]].m_camPos.current(), org1);
	double dist = 0;

	for (int i = 0; i < 3; i++){
		dist += (org0[i] - org1[i]) * (org0[i] - org1[i]);
	}
	dist = std::sqrt(dist);
	_scale = 1.0 / dist;

	m_nFirstFrame = curFrame;
	return true;
}

bool CoSLAM::calibGlobal2Cam(){
	vector<cv::Point3f> marker_poses_global;
	geometry_msgs::PoseStamped poses;
	cv::Point2f imgLocs;
	int numMarkers = 4;
	Mat_d ms(numCams, 2);
	Mat_d nms(numCams, 2);
	Mat_d Ks(numCams, 9);
	Mat_d Rs(numCams, 9);
	Mat_d Ts(numCams, 3);

	vector<cv::Point3f> marker_poses_cam;

	for (int ii = 0; ii < numCams; ii++){
		while(MyApp::markerList[ii].back().markers.size() != MyApp::_numMarkers)
		{
			cout << "Not enough markers detected in cam: " << ii << endl;
		}
	}


	FILE* fid = fopen("debug_calibGlobal2Cam.txt", "w");

	for (int i = 0; i < numMarkers; i++){
		// for each marker

		//get the marker id in the first camera
		int id0 = MyApp::markerList[0].back().markers[i].id;
		int id_markers = i;

		for (int j = 0; j < numCams; j++){
			//if not the first camera, find the marker id in the other camera markerlist
			if (j > 0){
				for (int kk = 0; kk < numMarkers; kk++){
					if (id0 == MyApp::markerList[j].back().markers[kk].id)
						id_markers = kk;
				}
			}
			// go through all the cameras to get the location in the image
			poses = MyApp::markerList[j].back().markers[id_markers].pose;
			//assign the normalized coordinate
			nms[2*j] = poses.pose.position.x / poses.pose.position.z;
			nms[2*j + 1] = poses.pose.position.y / poses.pose.position.z;
			CamPoseItem* cam = slam[j].m_camPos.current();
			memcpy(Ks.data + 9 * j, slam[j].K.data, sizeof(double) * 9);
			memcpy(Rs.data + 9 * j, cam->R, sizeof(double) * 9);
			memcpy(Ts.data + 3 * j, cam->t, sizeof(double) * 3);

			printf("%d ", MyApp::markerList[j].back().markers[id_markers].id);
			printf("%lf %lf %lf\n", poses.pose.position.x,
					poses.pose.position.y, poses.pose.position.z);
		}
		double M[3];
		triangulateMultiView(numCams, Rs.data, Ts.data, nms.data, M);
		marker_poses_cam.push_back(cv::Point3f(M[0], M[1], M[2]));
		printf("C: %lf %lf %lf\n", M[0], M[1], M[2]);
		cout << "i: " << i <<endl;
		cout << "size: " << MyApp::markerList[0].back().markers.size() << endl;

		int id = MyApp::markerList[0].back().markers[i].id;
		cout << "id: " << id <<endl;

		marker_poses_global.push_back(MyApp::markerPoseGlobal[id]);
		printf("G: %lf %lf %lf\n", MyApp::markerPoseGlobal[id].x, MyApp::markerPoseGlobal[id].y,
				MyApp::markerPoseGlobal[id].z);
	}

	scale_global2Cam = calibScale(marker_poses_global, marker_poses_cam);
	for (int i = 0; i < marker_poses_cam.size(); i++){
		printf("%lf %lf %lf\n", marker_poses_cam[i].x, marker_poses_cam[i].y, marker_poses_cam[i].z);
	}
	rigidTransformEsti(marker_poses_cam, marker_poses_global, R_global2Cam, t_global2Cam);
	fprintf(fid, "scale: %lf\n", scale_global2Cam);
	fprintf(fid, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
			R_global2Cam[0], R_global2Cam[1], R_global2Cam[2],
			R_global2Cam[3], R_global2Cam[4], R_global2Cam[5],
			R_global2Cam[6], R_global2Cam[7], R_global2Cam[8]);
	fprintf(fid, "%lf %lf %lf\n",
				t_global2Cam[0], t_global2Cam[1], t_global2Cam[2]);
	fclose(fid);

	return true;
}

bool CoSLAM::transformCamPose2Global(CamPoseItem* cam, double P_global[3]){
	double P_cam[3];
	getCamCenter(cam, P_cam);
	P_cam[0] *= scale_global2Cam;
	P_cam[1] *= scale_global2Cam;
	P_cam[2] *= scale_global2Cam;

	double R[9], t[3];
	memcpy(R, R_global2Cam, 9 * sizeof(double));
	memcpy(t, t_global2Cam, 3 * sizeof(double));

	P_global[0] = R[0] * P_cam[0] + R[1] * P_cam[1] + R[2] * P_cam[2] + t[0];
	P_global[1] = R[3] * P_cam[0] + R[4] * P_cam[1] + R[5] * P_cam[2] + t[1];
	P_global[2] = R[6] * P_cam[0] + R[7] * P_cam[1] + R[8] * P_cam[2] + t[2];
	return true;
}

double CoSLAM::calibScale(vector<cv::Point3f>& ptsA, vector<cv::Point3f>& ptsB){
	int nPts = ptsA.size();
	int m = nPts * (nPts - 1) / 2;
	double* A = new double[m];
	double* B = new double[m];
	int kk = 0;
	for (int ii = 0; ii < nPts-1; ii++){
		for (int jj = ii+1; jj < nPts; jj++){
			A[kk] = cv::norm(ptsA[ii]-ptsA[jj]);
			B[kk] = cv::norm(ptsB[ii]-ptsB[jj]);
			kk++;
		}
	}
	double scale_est = 0;
	dgelsyFor(m,1,1,B,A,&scale_est);
	for (int i = 0; i < nPts; i++){
		ptsB[i] = ptsB[i] * scale_est;
	}

	delete A;
	delete B;
	return scale_est;
}

bool CoSLAM::rigidTransformEsti(vector<cv::Point3f>& ptsA, vector<cv::Point3f>& ptsB,
		double* R, double* t){
	//Compute centroids
	cv::Mat centroid_A(3,1, CV_32F, cv::Scalar(0));
	cv::Mat centroid_B(3,1, CV_32F, cv::Scalar(0));

	int numPts = ptsA.size();
	for (int i = 0; i< ptsA.size(); i++){
		centroid_A.at<float>(0,0) += ptsA[i].x;
		centroid_A.at<float>(0,1) += ptsA[i].y;
		centroid_A.at<float>(0,2) += ptsA[i].z;

		centroid_B.at<float>(0,0) += ptsB[i].x;
		centroid_B.at<float>(0,1) += ptsB[i].y;
		centroid_B.at<float>(0,2) += ptsB[i].z;
	}
	centroid_A = centroid_A / numPts;
	centroid_B = centroid_B / numPts;

	cout << "centroid_A" << centroid_A <<endl;
	cout << "centroid_B" << centroid_B <<endl;

	//Compute H
	float H_data[9];
	for (int i = 0; i < numPts; i++){
		H_data[0] += (ptsA[i].x - centroid_A.at<float>(0,0)) * (ptsB[i].x - centroid_B.at<float>(0,0));
		H_data[1] += (ptsA[i].x - centroid_A.at<float>(0,0)) * (ptsB[i].y - centroid_B.at<float>(0,1));
		H_data[2] += (ptsA[i].x - centroid_A.at<float>(0,0)) * (ptsB[i].z - centroid_B.at<float>(0,2));

		H_data[3] += (ptsA[i].y - centroid_A.at<float>(0,1)) * (ptsB[i].x - centroid_B.at<float>(0,0));
		H_data[4] += (ptsA[i].y - centroid_A.at<float>(0,1)) * (ptsB[i].y - centroid_B.at<float>(0,1));
		H_data[5] += (ptsA[i].y - centroid_A.at<float>(0,1)) * (ptsB[i].z - centroid_B.at<float>(0,2));

		H_data[6] += (ptsA[i].z - centroid_A.at<float>(0,2)) * (ptsB[i].x - centroid_B.at<float>(0,0));
		H_data[7] += (ptsA[i].z - centroid_A.at<float>(0,2)) * (ptsB[i].y - centroid_B.at<float>(0,1));
		H_data[8] += (ptsA[i].z - centroid_A.at<float>(0,2)) * (ptsB[i].z - centroid_B.at<float>(0,2));
	}

	cv::Mat H(3,3, CV_32F, H_data);
	cv::Mat w, u, vt;
	cv::SVD::compute(H, w, u, vt);
	cv::Mat ut, v;
	cv::transpose(u, ut);
	cv::transpose(vt, v);
	cv::Mat ret_R = v * ut;
	if (cv::determinant(ret_R) < 0){
		v.at<float>(0,2) = v.at<float>(0,2) * -1;
		v.at<float>(1,2) = v.at<float>(1,2) * -1;
		v.at<float>(2,2) = v.at<float>(2,2) * -1;
		ret_R = v * ut;
	}
	cv::Mat ret_t = ret_R * centroid_A * -1 + centroid_B;

	cout << "ret_R" << ret_R <<endl;
	cout << "ret_t" << ret_t << endl;

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			R[i *3 + j] = ret_R.at<float>(i,j);
		}
		t[i] = ret_t.at<float>(i,0);
	}
}

void CoSLAM::featureTracking() {
	TimeMeasurer tm;
	tm.tic();
	for (int i = 0; i < numCams; i++)
			slam[i].trackFeaturePoints();
	m_tmFeatureTracking = tm.toc();
}

void CoSLAM::featureReceiving() {
	TimeMeasurer tm;
	tm.tic();

	pthread_mutex_lock(&MyApp::_mutexFeatures);

	while (MyApp::featuresList[0].empty())
		pthread_cond_wait(&MyApp::_condFeatures, &MyApp::_mutexFeatures);

	for (int i = 0; i < numCams; i++){
		vector<float>& fts = MyApp::featuresList[i].back().points;
		vector<unsigned int>& featureId = MyApp::featuresList[i].back().featureId;
		double ts = MyApp::featuresList[i].back().header.stamp.toSec();
		uchar* smallImg = (uchar*)MyApp::featuresList[i].back().smallImg.data();
		slam[i].receiveFeaturePoints(ts, featureId, fts, smallImg);
		_frameId[i] = MyApp::featuresList[i].back().frameId;
		ROS_INFO("Receive frame id: %d\n", _frameId[i]);
	}
	pthread_mutex_unlock(&MyApp::_mutexFeatures);
	m_tmFeatureTracking = tm.toc();

//	cv::Mat img(slam[0].m_smallImg.rows, slam[0].m_smallImg.cols, CV_8UC1, slam[0].m_smallImg.data);
//	cv::imshow("small img", img);
}

#include "SL_InterCamPoseEstimator.h"
bool CoSLAM::interCamPoseUpdate() {
	const int minStaticNum = 40;
	vector<Track2DNode*> nodes[SLAM_MAX_NUM];

	m_easyToFail = false;
	for (int i = 0; i < numCams; i++) {
		m_intraCamPoseUpdateFail[i] = 0;
	}
	for (int i = 0; i < numCams; i++) {
		int num = slam[i].getStaticMappedTrackNodes(nodes[i]);
		if (num < minStaticNum) {
			//if the number of static feature points in this view is small 
			m_easyToFail = true;
			m_intraCamPoseUpdateFail[i] = 2;
		}
		if (m_intraCamPoseUpdateFail[i] == false) {
			//check their bounding box
			double minX = 1e+6;
			double minY = 1e+6;
			double maxX = 0;
			double maxY = 0;
			for (size_t k = 0; k < nodes[i].size(); k++) {
				double x = nodes[i][k]->x;
				double y = nodes[i][k]->y;
				if (x < minX)
					minX = x;
				if (y < minY)
					minY = y;
				if (x > maxX)
					maxX = x;
				if (y > maxY)
					maxY = y;
			}
			if ((maxX - minX) < slam[i].m_img.w * 0.25
					|| (maxY - minY) < slam[i].m_img.h * 0.25) {
				m_easyToFail = true;
				m_intraCamPoseUpdateFail[i] = 1;
			}
		}
	}
	if (!m_easyToFail)
		return false;

	for (int i = 0; i < numCams; i++) {
		slam[i].propagateFeatureStates();
	}
	//	MyApp::bStop = true;
	//	redrawAllViews();
	//	while (MyApp::bStop) {
	//	};

	//inter-camera pose update
	InterCamPoseEstimator poseEstimator;
	poseEstimator.setCoSLAM(this);
	poseEstimator.addMapPoints();
	poseEstimator.apply();

	for (int i = 0; i < numCams; i++) {
		slam[i].detectDynamicFeaturePoints(20, 5, 3, Const::MAX_EPI_ERR);
	}

	//	for (int i = 0; i < numCams; i++) {
	//		CamPos* oldPos = slam[i].m_lastKeyPosFrm->cam;
	//		CamPos* newPos = slam[i].m_camPos.add(curFrame, slam[i].camId, poseEstimator.Rs.data + i * 9,
	//				poseEstimator.Ts.data + i * 3);
	//		slam[i].m_lastKeyPosDist = getCamDist(oldPos, newPos);
	//	}
	//	MyApp::bStop = true;
	//	redrawAllViews();
	//	while (MyApp::bStop) {
	//	};
	return true;
}
void CoSLAM::poseUpdate(bool *bEstPose) {
	TimeMeasurer tm;
	//	if (!interCamPoseUpdate()) {
	//		m_easyToFail = false;
	//		parallelPoseUpdate();
	//	}
	tm.tic();
	enterBACriticalSection();
	//	//test
	//	logInfo("pose estimation!\n");
//	if (!interCamPoseUpdate())
//		parallelPoseUpdate();
//	} else
//	if (curFrame > 100)
//		interCamPoseUpdate();
//	else

	if (curFrame >= m_lastFrmGroupMerge && curFrame < m_lastFrmGroupMerge + 60)
		parallelPoseUpdate(bEstPose, true);
	else
		parallelPoseUpdate(bEstPose, false);
	//	leaveBACriticalSection();
	//	CoSLAM::ptr->pause();
	//	enterBACriticalSection();

	leaveBACriticalSection();
	m_tmPoseUpdate = tm.toc();

	tm.tic();
	enterBACriticalSection();
	mapStateUpdate();
	leaveBACriticalSection();

	enterBACriticalSection();
//	mapPointsClassify(12.0);
	mapPointsClassify(3.0);
	leaveBACriticalSection();
	m_tmMapClassify = tm.toc();
}

//void* _parallelPoseUpdate(void* param) {
//	CoSLAM* pSLAM = CoSLAM::ptr;
//	int camId = (intptr_t) param;
//	if (pSLAM->slam[camId].poseUpdate3D(false) < 0)
//		return (void*) -1;
//	pSLAM->slam[camId].detectDynamicFeaturePoints(20, 5, 50, Const::MAX_EPI_ERR);
//	return 0;
//}
void CoSLAM::parallelPoseUpdate(bool* bEstPose, bool largeErr) {
	double R[9], t[3];

	if (numCams == 1) {
		bEstPose[0] = slam[0].poseUpdate3D(slam[0].m_camPos.current()->R,
				slam[0].m_camPos.current()->t, R, t, largeErr);
		if (bEstPose[0])
			slam[0].detectDynamicFeaturePoints(20, 5, 3, Const::MAX_EPI_ERR);
		return;
	}
	for (int i = 0; i < numCams; i++) {
		if (state[i] == SLAM_STATE_NORMAL){
			bEstPose[i] = slam[i].poseUpdate3D(slam[i].m_camPos.current()->R,
							slam[i].m_camPos.current()->t, R, t, largeErr);
			if(!bEstPose[i])
				startRelocalization(i);
		}
		//printf("bEstPose[%d]: %d\n", i, bEstPose[i]);
		if(bEstPose[i])
			slam[i].detectDynamicFeaturePoints(20, 5, 3,
					largeErr ? Const::MAX_EPI_ERR * 5 : Const::MAX_EPI_ERR);
	}
//	pthread_t threads[SLAM_MAX_NUM];
//	for (int i = 1; i < numCams; i++) {
//		pthread_create(&threads[i], 0, _parallelPoseUpdate, (void*) i);
//	}
//	if (slam[0].poseUpdate3D(largeErr) < 0)
//		repErr("intra-camera pose estimation failed at camera #%d!", 0);
//	slam[0].detectDynamicFeaturePoints(20, 5, 3, Const::MAX_EPI_ERR);
//	for (int i = 1; i < numCams; i++) {
//		void* ptr = 0;
//		pthread_join(threads[i], &ptr);
//		if ((intptr_t) ptr < 0)
//			repErr("intra-camera pose estimation failed at camera #%d!", i);
//	}
//#ifdef USE_OPENMP
//#pragma omp parallel for num_threads(SLAM_CPU_CORE_NUM)
//#endif
}
void CoSLAM::mapPointsClassify(double pixelVar) {
	MapPoint* pCurMapHead = curMapPts.getHead();
	if (!pCurMapHead)
	{
		printf("mapPointsClassify returned : no map point is found\n");
		return;
	}

	const int FRAME_NUM_FOR_NEWPOINT = 30;
	const int FRAME_NUM_FOR_DONTMOVE = 50;
	const int NUM_FRAME_CHECK_STATIC = 60;

	int numFalse = 0;
	for (MapPoint* p = pCurMapHead; p; p = p->next) {
		if (p->isUncertain() || p->isLocalDynamic()) {
			//check if the map point is a false point or a dynamic point
			if (p->numVisCam == 1) {
				p->setFalse();
				numFalse++;
				continue;
			}
			double M[3], cov[9];
			//check if the map point (with either 'uncertain' or 'dynamic' flag) is a false map point
			//triangulateMultiView(numViews, Rs, Ts, nms, M, 0);
			if (p->isUncertain()) {
				if (p->bNewPt) {
					//dynamic or static
					bool isStatic = isStaticPoint(numCams, p, pixelVar, M, cov,
							NUM_FRAME_CHECK_STATIC);
					if (isStatic) {
						if (p->lastFrame - p->firstFrame
								> FRAME_NUM_FOR_NEWPOINT) {
							p->setLocalStatic();
							p->bNewPt = false;
							p->updatePosition(M, cov);
						}
						//continue; - keep checking 
					} else {
						if (isDynamicPoint(numCams, p, pixelVar, M, cov)) {
							p->setLocalDynamic();
							p->updatePosition(M, cov);
							p->bNewPt = false;
						} else {
							p->setFalse();
							//test
							//printf("2\n");
						}
					}
				} else {
					//dynamic or false
					if (isDynamicPoint(numCams, p, pixelVar, M, cov)) {
						p->setLocalDynamic();
						p->updatePosition(M, cov);
					} else {
						int outlierViewId = isStaticRemovable(numCams, p,
								pixelVar, M, cov, NUM_FRAME_CHECK_STATIC);
						if (outlierViewId >= 0) {
							FeaturePoint* fp = p->pFeatures[outlierViewId];
							fp->mpt = 0;
							p->pFeatures[outlierViewId] = 0;
							p->numVisCam--;
							p->setLocalStatic();
							p->updatePosition(M, cov);
						} else {
							p->setFalse();
							//test
							//printf("3\n");
						}
					}
					//					}
				}
			}

			else if (p->isLocalDynamic()) {
				//can return to static or be a false point
				if (isDynamicPoint(numCams, p, pixelVar, M, cov)) {
					if (isLittleMove(numCams, p, pixelVar, M, cov)) {
						p->staticFrameNum++;
						if (p->staticFrameNum > FRAME_NUM_FOR_DONTMOVE) {
							double M0[3], cov0[9];
							if (isStaticPoint(numCams, p, pixelVar, M0, cov0,
									NUM_FRAME_CHECK_STATIC)) {
								//go back to static points
								p->setLocalStatic();
								for (int i = 0; i < numCams; i++) {
									if (p->pFeatures[i])
										p->pFeatures[i]->type =
												TYPE_FEATPOINT_STATIC;
								}
								p->updatePosition(M0, cov0);
							} else {
								p->staticFrameNum = 0;
								p->updatePosition(M, cov);
							}
						} else
							p->updatePosition(M, cov);
					} else {
						p->staticFrameNum = 0;
						p->updatePosition(M, cov);
					}
					p->updatePosition(M, cov);
				} else
					p->setFalse();
			}
		}
	}
}
//void CoSLAM::mapPointsClassify(double maxRpErr) {
//	MapPoint* pActMapHead = actMapPts.getHead();
//	if (!pActMapHead)
//		repErr("no map point is found");
//
//	const int FRAME_NUM_FOR_NEWPOINT = 20;
//	const int FRAME_NUM_FOR_DONTMOVE = 5;
//
//	int numFalse = 0;
//	for (MapPoint* p = pActMapHead; p; p = p->next) {
//		if (p->type == TYPE_MAPPOINT_UNCERTAIN) {
//			//check if the map point is a false point or a dynamic point
//			if (p->numVisCam == 1) {
//				p->setFalse();
//				numFalse++;
//				continue;
//			} else {
//				double M[3], cov[9];
//				if (isRemovable(numCams, p, maxRpErr, M, cov) < 0)
//					p->setFalse();
//				else {
//					p->updatePosition(M, cov);
//					p->setStatic();
//				}
//			}
//		} else if (p->type == TYPE_MAPPOINT_DYNAMIC) {
//			if (p->numVisCam == 1) {
//				p->setFalse();
//				numFalse++;
//				continue;
//			}
//			//update the position of dynamic points
//			double M[3], cov[9];
//			//can return to static or be a false point
//			if (isDynamicPoint(numCams, p, maxRpErr, M, cov)) {
//				if (isLittleMove(numCams, p, maxRpErr, M, cov)) {
//					p->staticFrameNum++;
//					if (p->staticFrameNum > FRAME_NUM_FOR_DONTMOVE) {
//						double M0[3], cov0[9];
//						if (isStaticPoint(numCams, p, maxRpErr, M0, cov0)) {
//							//go back to static points
//							p->setStatic();
//							for (int i = 0; i < numCams; i++) {
//								if (p->pFeatures[i])
//									p->pFeatures[i]->type = TYPE_FEATPOINT_STATIC;
//							}
//							p->updatePosition(M0, cov0);
//						} else {
//							p->staticFrameNum = 0;
//							p->updatePosition(M, cov);
//						}
//					} else
//						p->updatePosition(M, cov);
//
//				} else {
//					p->staticFrameNum = 0;
//					p->updatePosition(M, cov);
//				}
//			} else
//				p->setFalse();
//		}
//	}
//}
int CoSLAM::searchNearestCamForMapPt(const Mat_d& camDist, const MapPoint* p,
		int iCam) {
	int jCam = -1;
	double dMin = DBL_MAX;
	for (int j = 0; j < numCams; j++) {
		if (p->pFeatures[j]) {
			double d = camDist.data[iCam * numCams + j];
			if (d < dMin) {
				jCam = j;
				dMin = d;
			}
		}
	}
	return jCam;
}
void CoSLAM::getCurrentCamDist(Mat_d& camDist) {
	camDist.resize(numCams, numCams);
	//compute the distance matrix for cameras
#ifdef USE_OPENMP
#pragma omp parallel for num_threads(SLAM_CPU_CORE_NUM)
#endif
	for (int i = 0; i < numCams; i++) {
		camDist.data[i * numCams + i] = -1;
		for (int j = i + 1; j < numCams; j++) {
			double d = getCamDist(slam[i].m_camPos.current(),
					slam[j].m_camPos.current());
			camDist.data[i * numCams + j] = d;
			camDist.data[j * numCams + i] = d;
		}
	}
}
bool CoSLAM::compareFeaturePt(const ImgG& scaledImg1, double imgScale1,
		const ImgG& scaledImg2, double imgScale2, const FeaturePoint* pt1,
		const FeaturePoint* pt2) {

	NCCBlock nccblk1, nccblk2;
	getScaledNCCBlock(scaledImg1, imgScale1, pt1->xo, pt1->yo, nccblk1);
	getScaledNCCBlock(scaledImg2, imgScale2, pt2->xo, pt2->yo, nccblk2);

	double ncc = matchNCCBlock(&nccblk1, &nccblk2);
	if (ncc >= 0.75)
		return true;

	return true;
	//return false;
}
bool CoSLAM::checkUnify(MapPoint* mpt1, MapPoint* mpt2, double M[],
		double cov[], double pixelErrVar) {
	double Ks[SLAM_MAX_NUM * 36], Rs[SLAM_MAX_NUM * 36], ts[SLAM_MAX_NUM * 12],
			ms[SLAM_MAX_NUM * 8], nms[SLAM_MAX_NUM * 8];
	int numView = 0;
	for (int i = 0; i < numCams; i++) {
		if (mpt1->pFeatures[i]) {
			MapPoint* p = mpt1;
			double iK[9];
			getInvK(slam[i].K.data, iK);
			FeaturePoint* fp = p->pFeatures[i];
			double* R0 = fp->cam->R;
			double* t0 = fp->cam->t;

			doubleArrCopy(Ks, numView, slam[i].K.data, 9);
			doubleArrCopy(Rs, numView, R0, 9);
			doubleArrCopy(ts, numView, t0, 3);
			doubleArrCopy(ms, numView, fp->m, 2);
			normPoint(iK, fp->m, nms + 2 * numView);
			numView++;

			double C0[3];
			getCameraCenter(R0, t0, C0);

			FeaturePoint* maxFp = 0;
			double maxAngle = 0;
			fp = fp->preFrame;
			while (fp) {
				double C[3];
				getCameraCenter(fp->cam->R, fp->cam->t, C);
				double angle = getAbsRadiansBetween(p->M, C0, C);
				if (angle > maxAngle) {
					maxAngle = angle;
					maxFp = fp;
				}
				fp = fp->preFrame;
			}

			if (maxFp) {
				doubleArrCopy(Ks, numView, slam[i].K.data, 9);
				doubleArrCopy(Rs, numView, maxFp->cam->R, 9);
				doubleArrCopy(ts, numView, maxFp->cam->t, 3);
				doubleArrCopy(ms, numView, maxFp->m, 2);
				normPoint(iK, maxFp->m, nms + 2 * numView);
				numView++;
			}
		}
		if (mpt2->pFeatures[i]) {
			MapPoint* p = mpt2;
			double iK[9];
			getInvK(slam[i].K.data, iK);
			FeaturePoint* fp = p->pFeatures[i];
			double* R0 = fp->cam->R;
			double* t0 = fp->cam->t;

			doubleArrCopy(Ks, numView, slam[i].K.data, 9);
			doubleArrCopy(Rs, numView, R0, 9);
			doubleArrCopy(ts, numView, t0, 3);
			doubleArrCopy(ms, numView, fp->m, 2);
			normPoint(iK, fp->m, nms + 2 * numView);
			numView++;

			double C0[3];
			getCameraCenter(R0, t0, C0);

			FeaturePoint* maxFp = 0;
			double maxAngle = 0;
			fp = fp->preFrame;
			while (fp) {
				double C[3];
				getCameraCenter(fp->cam->R, fp->cam->t, C);
				double angle = getAbsRadiansBetween(p->M, C0, C);
				if (angle > maxAngle) {
					maxAngle = angle;
					maxFp = fp;
				}
				fp = fp->preFrame;
			}

			if (maxFp) {
				doubleArrCopy(Ks, numView, slam[i].K.data, 9);
				doubleArrCopy(Rs, numView, maxFp->cam->R, 9);
				doubleArrCopy(ts, numView, maxFp->cam->t, 3);
				doubleArrCopy(ms, numView, maxFp->m, 2);
				normPoint(iK, maxFp->m, nms + 2 * numView);
				numView++;
			}
		}
	}

	triangulateMultiView(numView, Rs, ts, nms, M);
	getTriangulateCovMat(numView, Ks, Rs, ts, M, cov, pixelErrVar);

	//check the reprojection error
	for (int i = 0; i < numView; i++) {
		double rm[2], var[4], ivar[4];
		project(Ks + 9 * i, Rs + 9 * i, ts + 3 * i, M, rm);
		getProjectionCovMat(Ks + 9 * i, Rs + 3 * i, ts + 3 * i, M, cov, var,
				pixelErrVar);
		mat22Inv(var, ivar);
		if (mahaDist2(rm, ms + 2 * i, ivar) > 1.0)
			return false;
	}
	return true;
}
void CoSLAM::refineMapPoint(MapPoint* p) {
	double Ks[SLAM_MAX_NUM * 18], Rs[SLAM_MAX_NUM * 18], ts[SLAM_MAX_NUM * 6],
			ms[SLAM_MAX_NUM * 4], nms[SLAM_MAX_NUM * 4];
	int numView = 0;
	for (int i = 0; i < numCams; i++) {
		if (p->pFeatures[i]) {
			double iK[9];
			getInvK(slam[i].K.data, iK);
			FeaturePoint* fp = p->pFeatures[i];
			double* R0 = fp->cam->R;
			double* t0 = fp->cam->t;

			doubleArrCopy(Ks, numView, slam[i].K.data, 9);
			doubleArrCopy(Rs, numView, R0, 9);
			doubleArrCopy(ts, numView, t0, 3);
			doubleArrCopy(ms, numView, fp->m, 2);
			normPoint(iK, fp->m, nms + 2 * numView);
			numView++;

			double C0[3];
			getCameraCenter(R0, t0, C0);
			FeaturePoint* maxFp = 0;
			double maxAngle = 0;
			fp = fp->preFrame;
			while (fp) {
				double C[3];
				getCameraCenter(fp->cam->R, fp->cam->t, C);
				double angle = getAbsRadiansBetween(p->M, C0, C);
				if (angle > maxAngle) {
					maxAngle = angle;
					maxFp = fp;
				}
				fp = fp->preFrame;
			}

			if (maxFp) {
				doubleArrCopy(Ks, numView, slam[i].K.data, 9);
				doubleArrCopy(Rs, numView, maxFp->cam->R, 9);
				doubleArrCopy(ts, numView, maxFp->cam->t, 3);
				doubleArrCopy(ms, numView, maxFp->m, 2);
				normPoint(iK, maxFp->m, nms + 2 * numView);
				numView++;
			}
		}
	}
	triangulateMultiView(numView, Rs, ts, nms, p->M);
	getTriangulateCovMat(numView, Ks, Rs, ts, p->M, p->cov,
			Const::PIXEL_ERR_VAR);
}
bool CoSLAM::staticCheckMergability(const MapPoint* mp, const FeaturePoint* fp,
		double pixelVar) {
	bool bMergable = true;
	for (const FeaturePoint* p = fp; p; p = p->preFrame) {
		double rm[2], var[4], ivar[4];
		project(p->K, p->cam->R, p->cam->t, mp->M, rm);
		getProjectionCovMat(p->K, p->cam->R, p->cam->t, mp->M, mp->cov, var,
				pixelVar);
		mat22Inv(var, ivar);
		if (mahaDist2(rm, p->m, ivar) > 1.0) {
			bMergable = false;
			break;
		}
	}
	return bMergable;
}
bool CoSLAM::curStaticPointRegInGroup(const CameraGroup& camGroup,
		Mat_d& camDist, MapPoint* p, double pixelErrVar, bool bMerge) {
	bool bReg = false;
	if (p->isLocalStatic() && p->numVisCam > 0) {
		for (int i = 0; i < camGroup.num; i++) {
			int iCam = camGroup.camIds[i];
			if (p->pFeatures[iCam] && p->pFeatures[iCam]->f == curFrame)
				continue;
			double m[2], var[4];
			if (isAtCameraBack(slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M))
				continue;

			project(slam[iCam].K.data, slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M, m);
			if (m[0] < 0 || m[0] >= slam[iCam].videoReader->_w || m[1] < 0
					|| m[1] >= slam[iCam].videoReader->_h)
				continue;

			getProjectionCovMat(slam[iCam].K.data,
					slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M, p->cov, var,
					pixelErrVar);

			FeaturePoint* pFeat = searchMahaNearestFeatPt(slam[iCam].m_featPts,
					curFrame, m, var, pixelErrVar * 3);

			if (pFeat > 0 && pFeat->type != TYPE_FEATPOINT_DYNAMIC) {
				if (pFeat->mpt == 0) {
					int jCam = searchNearestCamForMapPt(camDist, p, iCam);
					assert(jCam >= 0 && p->pFeatures[jCam]);
					if (jCam >= 0 && p->pFeatures[jCam]) {
						FeaturePoint* pFeatTmp = p->pFeatures[jCam];
						if (compareFeaturePt(slam[iCam].m_smallImg,
								slam[iCam].m_smallScale, slam[jCam].m_smallImg,
								slam[jCam].m_smallScale, pFeat, pFeatTmp)) {
							if (staticCheckMergability(p, pFeat, pixelErrVar)) {

								FeaturePoint* fp = pFeat->preFrame;
								while (fp) {
									fp->mpt = p;
									fp = fp->preFrame;
								}

								if (p->pFeatures[iCam]) {
									//already have a 2D point at the old frame
									p->pFeatures[iCam]->nextFrame = pFeat;
									pFeat->preFrame = p->pFeatures[iCam];
								}
								p->addFeature(iCam, pFeat);
								//update the NCC block
								p->nccBlks[iCam].computeScaled(
										slam[iCam].m_smallImg,
										slam[iCam].m_smallScale, pFeat->xo,
										pFeat->yo);
								bReg = true;
							}
						}
					}
				} else {
					if (!bMerge)
						return bReg;
					if (!pFeat->mpt->isLocalStatic())
						continue;
					if (p == pFeat->mpt)
						continue;
					//assert(p != pFeat->mpt);
					//conflict, check if the two map points can be unified
					double M[3], cov[9];
					if (checkUnify(p, pFeat->mpt, M, cov, pixelErrVar)) {
						p->updatePosition(M, cov);
						//merge the two map points
						pFeat->mpt->numVisCam = 0;
						pFeat->mpt->setFalse();

						//unify the two map points
						for (int v = 0; v < numCams; v++) {
							FeaturePoint* pFt = pFeat->mpt->pFeatures[v];
							if (pFt && !p->pFeatures[v]) {
								pFeat->mpt->pFeatures[v] = 0;
								pFt->mpt = 0;
								p->addFeature(v, pFt);
								for (FeaturePoint* fp = pFt->preFrame; fp; fp =
										fp->preFrame)
									fp->mpt = p;
							}
						}

						//update the visibility
						p->numVisCam = 0;
						for (int v = 0; v < numCams; v++) {
							if (p->pFeatures[v])
								p->numVisCam++;
						}
						bReg = true;
						return bReg;
					}
				}
			}
		}
	}
	return bReg;
}
int CoSLAM::currentMapPointsRegister(double pixelErrVar, bool bMerge) {
	//time measurement
	TimeMeasurer tm;
	tm.tic();
	//register static points
	int nReg = 0;
	for (int i = 0; i < m_groupNum; i++) {
		if(allCamerasInGroupNormal(i))
			nReg += curStaticPointsRegInGroup(m_groups[i], pixelErrVar, bMerge);
	}

	//register dynamic points
	for (int i = 0; i < m_groupNum; i++) {
		if(allCamerasInGroupNormal(i))
			nReg += curDynamicPointsRegInGroup(m_groups[i], pixelErrVar, bMerge);
	}
	//time measurement
	m_tmCurMapRegister = tm.toc();
	return nReg;
}

int CoSLAM::curStaticPointsRegInGroup(const CameraGroup& camGroup,
		double pixelErrVar, bool bMerge) {
	enterBACriticalSection();
	//	//test
	//	logInfo("currentMapPointsRegisterInGroup!\n");
	Mat_d camDist;
	getCurrentCamDist(camDist);

	int nTotalRegged = 0;

	for (int i = 0; i < camGroup.num; i++) {
		int iCam = camGroup.camIds[i];

		MapPoint* pCurMapHead = curMapPts.getHead();
		if (!pCurMapHead)
			repErr("no map point is found");

		std::vector<MapPoint*> vecMapPts;
		for (MapPoint* p = pCurMapHead; p; p = p->next) {
			if (p->isCertainStatic() && p->pFeatures[iCam]
					&& p->pFeatures[iCam]->f == curFrame) {
				vecMapPts.push_back(p);
			}
		}
		size_t nPts = vecMapPts.size();
		std::vector<bool> flagReged;
		flagReged.assign(nPts, false);

		std::vector<MapPoint*> vecReggedMapPts;
		vecReggedMapPts.reserve(nPts);
		int nRegged = 0;
		for (size_t i = 0; i < nPts; i++) {
			MapPoint* p = vecMapPts[i];
			if (p->isCertainStatic()) {
				flagReged[i] = curStaticPointRegInGroup(camGroup, camDist, p,
						pixelErrVar, bMerge);
				if (flagReged[i]) {
					vecReggedMapPts.push_back(p);
					nRegged++;
				}
			}
		}
		for (int i = 0; i < nRegged; i++) {
			//update 3D position
			MapPoint* p = vecReggedMapPts[i];
			refineMapPoint(p);
		}
		nTotalRegged += nRegged;
	}
	leaveBACriticalSection();
	return nTotalRegged;
}

int CoSLAM::curDynamicPointsRegInGroup(const CameraGroup& camGroup,
		double pixelErrVar, bool bMerge) {
	enterBACriticalSection();
	//	//test
	//	logInfo("currentMapPointsRegisterInGroup!\n");
	Mat_d camDist;
	getCurrentCamDist(camDist);

	int nTotalRegged = 0;

	for (int i = 0; i < camGroup.num; i++) {
		int iCam = camGroup.camIds[i];
		MapPoint* pCurMapHead = curMapPts.getHead();
		if (!pCurMapHead)
			repErr("no map point is found");

		std::vector<MapPoint*> vecMapPts;
		for (MapPoint* p = pCurMapHead; p; p = p->next) {
			if (p->isCertainDynamic() && p->pFeatures[iCam]
					&& p->pFeatures[iCam]->f == curFrame) {
				vecMapPts.push_back(p);
			}
		}
		size_t nPts = vecMapPts.size();
		std::vector<bool> flagReged;
		flagReged.assign(nPts, false);

		std::vector<MapPoint*> vecReggedMapPts;
		vecReggedMapPts.reserve(nPts);
		int nRegged = 0;
		for (size_t i = 0; i < nPts; i++) {
			MapPoint* p = vecMapPts[i];
			if (p->isCertainDynamic()) {
				flagReged[i] = curDynamicPointRegInGroup(camGroup, camDist, p,
						pixelErrVar, bMerge);
				if (flagReged[i]) {
					vecReggedMapPts.push_back(p);
					nRegged++;
				}
			}
		}
		for (int i = 0; i < nRegged; i++) {
			//update 3D position
			MapPoint* p = vecReggedMapPts[i];
			refineMapPoint(p);
		}
		nTotalRegged += nRegged;
	}
	leaveBACriticalSection();
	return nTotalRegged;
}
bool CoSLAM::curDynamicPointRegInGroup(const CameraGroup& camGroup,
		Mat_d& camDist, MapPoint* p, double pixelErrVar, bool bMerge) {
	bool bReg = false;
	if (p->isCertainDynamic() && p->numVisCam > 0) {
		for (int i = 0; i < camGroup.num; i++) {
			int iCam = camGroup.camIds[i];
			if (p->pFeatures[iCam] && p->pFeatures[iCam]->f == curFrame)
				continue;
			double m[2], var[4];
			if (isAtCameraBack(slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M))
				continue;

			project(slam[iCam].K.data, slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M, m);
			if (m[0] < 0 || m[0] >= slam[iCam].videoReader->_w || m[1] < 0
					|| m[1] >= slam[iCam].videoReader->_h)
				continue;

			getProjectionCovMat(slam[iCam].K.data,
					slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M, p->cov, var,
					pixelErrVar);
			FeaturePoint* pFeat = searchMahaNearestFeatPt(slam[iCam].m_featPts,
					curFrame, m, var, pixelErrVar * 4);

			if (pFeat > 0 && pFeat->type == TYPE_FEATPOINT_DYNAMIC) {
				if (pFeat->mpt == 0) {
					int jCam = searchNearestCamForMapPt(camDist, p, iCam);
					assert(jCam >= 0 && p->pFeatures[jCam]);
					if (jCam >= 0 && p->pFeatures[jCam]) {
						FeaturePoint* pFeatTmp = p->pFeatures[jCam];
						if (compareFeaturePt(slam[iCam].m_smallImg,
								slam[iCam].m_smallScale, slam[jCam].m_smallImg,
								slam[jCam].m_smallScale, pFeat, pFeatTmp)) {
							if (staticCheckMergability(p, pFeat, pixelErrVar)) {

								FeaturePoint* fp = pFeat->preFrame;
								while (fp) {
									fp->mpt = p;
									fp = fp->preFrame;
								}
								if (p->pFeatures[iCam]) {
									//already have a 2D point at the old frame
									p->pFeatures[iCam]->nextFrame = pFeat;
									pFeat->preFrame = p->pFeatures[iCam];
								}
								p->addFeature(iCam, pFeat);
								//update the NCC block
								p->nccBlks[iCam].computeScaled(
										slam[iCam].m_smallImg,
										slam[iCam].m_smallScale, pFeat->xo,
										pFeat->yo);
								bReg = true;
							}
						}
					}
				} else {
					//TODO
					//					if (!bMerge)
					return bReg;
					if (pFeat->mpt->isCertainDynamic())
						continue;
					if (p == pFeat->mpt)
						continue;
					//assert(p != pFeat->mpt);
					//conflict, check if the two map points can be unified
					double M[3], cov[9];
					if (checkUnify(p, pFeat->mpt, M, cov, pixelErrVar)) {
						p->updatePosition(M, cov);
						//merge the two map points
						pFeat->mpt->numVisCam = 0;
						pFeat->mpt->setFalse();

						//unify the two map points
						for (int v = 0; v < numCams; v++) {
							FeaturePoint* pFt = pFeat->mpt->pFeatures[v];
							if (pFt && !p->pFeatures[v]) {
								pFeat->mpt->pFeatures[v] = 0;
								pFt->mpt = 0;
								p->addFeature(v, pFt);
								for (FeaturePoint* fp = pFt->preFrame; fp; fp =
										fp->preFrame)
									fp->mpt = p;
							}
						}

						//update the visibility
						p->numVisCam = 0;
						for (int v = 0; v < numCams; v++) {
							if (p->pFeatures[v])
								p->numVisCam++;
						}
						bReg = true;
						return bReg;
					}
				}
			}
		}
	}
	return bReg;
}

int CoSLAM::getNearestFeatPtForReg(const MapPoint* mp,
		const CamPoseItem* cam0) {
	int minI = -1;
	double minDist = DBL_MAX;
	for (int i = 0; i < numCams; i++) {
		const FeaturePoint* fp = mp->pFeatures[i];
		if (fp) {
			double dist = getCamDist(fp->cam, cam0);
			if (dist < minDist) {
				minI = i;
				minDist = dist;
			}
		}
	}
	return minI;
}
int CoSLAM::activeMapPointsRegister(double pixelErrVar) {
	int nReg = 0;
	for (int i = 0; i < m_groupNum; i++) {
		nReg += activeMapPointsRegisterInGroup(m_groups[i], pixelErrVar);
	}
	return nReg;
}
int CoSLAM::activeMapPointsRegisterInGroup(const CameraGroup& camGroup,
		double pixelErrVar) {
	//time measurement
	TimeMeasurer tm;
	tm.tic();

	/*==============(critical section)=================*/
	enterBACriticalSection();
	MapPoint* pActMapHead = actMapPts.getHead();
	leaveBACriticalSection();
	/*------------------------------------------------*/

	if (!pActMapHead) {
		return 0;
	}

	vector<MapPoint*> vecActMapPts;
	vecActMapPts.reserve(actMapPts.getNum() * 2);

	/*==============(critical section)=================*/
	enterBACriticalSection();
	for (MapPoint* p = pActMapHead; p; p = p->next) {
		vecActMapPts.push_back(p);
	}
	actMapPts.clearWithoutRelease();
	size_t nPts = vecActMapPts.size();
	leaveBACriticalSection();
	/*------------------------------------------------*/

	Mat_i flagReged(nPts, 1);

	/*==============(critical section)=================*/
	enterBACriticalSection();
	//	#pragma omp parallel for num_threads(4)
	for (size_t i = 0; i < nPts; i++) {
		flagReged.data[i] = 0;
		MapPoint* p = vecActMapPts[i];
		if (p->isCertainStatic()) {
			flagReged.data[i] = activeMapPointRegisterInGroup(camGroup, p,
					pixelErrVar);
		}
	}
	leaveBACriticalSection();
	/*------------------------------------------------*/

	/*==============(critical section)=================*/
	enterBACriticalSection();
	int nReg = 0;
	for (size_t i = 0; i < nPts; i++) {
		MapPoint* p = vecActMapPts[i];
		if (flagReged.data[i] == 0) {
			actMapPts.add(p);
		} else {
			p->lastFrame = curFrame;
			p->state = STATE_MAPPOINT_CURRENT;
			curMapPts.add(p);
			nReg++;
		}
	}
	leaveBACriticalSection();
	/*------------------------------------------------*/

	//time measurement
	m_tmActMapRegister = tm.toc();
	return nReg;
}
bool CoSLAM::activeMapPointRegisterInGroup(const CameraGroup& camGroup,
		MapPoint* p, double pixelErrVar) {
	bool bReg = false;
	//register the active map point to the feature points at the current frame
	if (p->isCertainStatic() && p->numVisCam > 0 && p->numVisCam < numCams) {
		bool bNotInGroup = true;
		for (int i = 0; i < camGroup.num; i++) {
			int iCam = camGroup.camIds[i];
			if (p->pFeatures[iCam])
				bNotInGroup = false;
		}
		if (bNotInGroup) {
			return false;
		}
		for (int i = 0; i < camGroup.num; i++) {
			int iCam = camGroup.camIds[i];
			double m[2], var[4];
			if (isAtCameraBack(slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M))
				continue;

			//project the active map point to the current view
			project(slam[iCam].K.data, slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M, m);
			if (m[0] < 0 || m[0] >= slam[iCam].videoReader->_w || m[1] < 0
					|| m[1] >= slam[iCam].videoReader->_h)
				continue;

			getProjectionCovMat(slam[iCam].K.data,
					slam[iCam].m_camPos.current()->R,
					slam[iCam].m_camPos.current()->t, p->M, p->cov, var,
					pixelErrVar * 2.5);

			FeaturePoint* pFeat = searchMahaNearestFeatPt(slam[iCam].m_featPts,
					curFrame, m, var, pixelErrVar * 3.0);

			if (pFeat > 0) {
				if (pFeat->mpt == 0) {
					int jCam = getNearestFeatPtForReg(p,
							slam[iCam].m_camPos.current());
					assert(jCam >= 0);
					//compute the feature points with NCC blocks
					NCCBlock blk;
					getScaledNCCBlock(slam[iCam].m_smallImg,
							slam[iCam].m_smallScale, pFeat->xo, pFeat->yo, blk);
					if (matchNCCBlock(&blk, &p->nccBlks[jCam]) > 0.60) {
						if (staticCheckMergability(p, pFeat, pixelErrVar)) {
							if (jCam == iCam) {
								p->pFeatures[jCam]->nextFrame = pFeat;
								pFeat->preFrame = p->pFeatures[jCam];
							}
							p->addFeature(iCam, pFeat);

							//update the NCC block
							p->nccBlks[iCam].copy(blk);

							refineMapPoint(p);
							bReg = true;
						}
					}
				}
			}
		}
	}
	return bReg;
}
void CoSLAM::mapStateUpdate() {
	MapPoint* pCurMapHead = curMapPts.getHead();
	if (!pCurMapHead){
		printf("mapStateUpdate(): !pCurMapHead\n");
		return;
	}

	//put the map points with no corresponding feature points at the current frame into the `actMapPts' list
	for (MapPoint* p = pCurMapHead; p; p = p->next) {
		p->updateVisCamNum(curFrame);
		if (p->lastFrame < curFrame) {
			MapPoint* pt = p;
			p = curMapPts.remove(pt);
			if (pt->isFalse())
				falseMapPts.add(pt);
			else if (pt->isCertainStatic()) {
				pt->state = STATE_MAPPOINT_ACTIVE;
				actMapPts.add(pt);
			} else {
				pt->state = STATE_MAPPOINT_INACTIVE;
				iactMapPts.add(pt);
			}
		}
	}
	printf("mapStateUpdate: curMapPts (%d), actMapPts (%d), iactMapPts (%d)\n",
			curMapPts.getNum(), actMapPts.getNum(), iactMapPts.getNum());

	MapPoint* pActMapHead = actMapPts.getHead();
	if (!pActMapHead)
		return;

	//put the map points that have no corresponding image point into the `iactMapPts' list
	for (MapPoint* p = pActMapHead; p; p = p->next) {
		if (p->lastFrame < curFrame - numActFrm) {
			MapPoint* pt = p;
			p = actMapPts.remove(pt);
			if (pt->isFalse())
				falseMapPts.add(pt);
			else {
				pt->state = STATE_MAPPOINT_INACTIVE;
				iactMapPts.add(pt);
			}
		} else if (p->lastFrame == curFrame) {
			//put back to the `curMapPts'
			MapPoint* pt = p;
			p = actMapPts.remove(pt);
			curMapPts.add(pt);
		}
	}assert(curMapPts.getNum() == curMapPts.count());
	assert(actMapPts.getNum() == actMapPts.count());
	assert(iactMapPts.getNum() == iactMapPts.count());

	printf("mapStateUpdate: curMapPts (%d), actMapPts (%d), iactMapPts (%d)\n",
			curMapPts.getNum(), actMapPts.getNum(), iactMapPts.getNum());

}
void CoSLAM::getCurMapCenterViewFrom(int camId, double center[3]) {
	const FeaturePoint* pHead = slam[camId].m_featPts.getFrameHead(curFrame);
	const FeaturePoint* pTail = slam[camId].m_featPts.getFrameTail(curFrame);

	center[0] = 0;
	center[1] = 0;
	center[2] = 0;

	int nPts = 0;
	for (const FeaturePoint* fp = pHead; fp && fp != pTail; fp = fp->next) {
		if (fp->mpt && !fp->mpt->isFalse()) {
			center[0] += fp->mpt->x;
			center[1] += fp->mpt->y;
			center[2] += fp->mpt->z;
			nPts++;
		}
	}
	//assert(nPts > 0);
	if (nPts == 0)
		pause();

	center[0] /= nPts;
	center[1] /= nPts;
	center[2] /= nPts;
}

bool CoSLAM::IsMappedPtsDecreaseBelow(int camId, double ratio) {
	const FeaturePoint* pHead = slam[camId].m_featPts.getFrameHead(curFrame);
	const FeaturePoint* pTail = slam[camId].m_featPts.getFrameTail(curFrame);

	int num = 0;
	int lastNum = slam[camId].m_keyPose.tail->nMappedPts;
	int lastFrame = slam[camId].m_keyPose.tail->frame;

	for (const FeaturePoint* fp = pHead; fp && fp != pTail; fp = fp->next) {
		if (fp->mpt && fp->mpt->firstFrame <= lastFrame)
			num++;
	}
	//test
	//logInfo("frame:%d lastFrame:%d lastNum:%d num:%d\n", curFrame, lastFrame, lastNum, num);
	if (num < lastNum * ratio) {
		return true;
	}
	if (num < 30)
		return true;
	return false;
}
int CoSLAM::IsReadyForKeyFrame(int camId) {
	double center[3];
	getCurMapCenterViewFrom(camId, center);
	if (IsMappedPtsDecreaseBelow(camId, m_mappedPtsReduceRatio))
		return READY_FOR_KEY_FRAME_DECREASE;
	if (slam[camId].getViewAngleChangeSelf(center) > m_minViewAngleChange)
		return READY_FOR_KEY_FRAME_VIEWANGLE;
	if (slam[camId].getCameraTranslationSelf() > m_minCamTranslation)
		return READY_FOR_KEY_FRAME_TRANSLATION;
	return 0;
}
KeyFrame* CoSLAM::addKeyFrame(int readyForKeyFrame[SLAM_MAX_NUM]) {
	KeyFrame* pKeyFrame = m_keyFrms.add(curFrame);
	m_keyFrameId[curFrame] = m_nKeyFrame++;
	for (int i = 0; i < numCams; i++)
		pKeyFrame->setKeyPose(i, slam[i].addKeyPose(readyForKeyFrame[i] > 0));

	//store some useful information at the current key frame
	pKeyFrame->setCamNum(numCams);
	pKeyFrame->setMapPtsNum(curMapPts.getNum());
	pKeyFrame->setCamGroups(m_groups, m_groupNum);
	pKeyFrame->storeDynamicMapPoints(curMapPts);

	return pKeyFrame;
}
int CoSLAM::genNewMapPoints() {
	if (!allCamerasNormal())
		return -1;

	int num = 0;
	getNumDynamicStaticPoints();

	//check if there is one camera ready for insert a new key frame
	int readyForKeyFrame[SLAM_MAX_NUM];
	int nReady = 0;
	bool decrease = false;
	for (int i = 0; i < numCams; i++) {
		slam[i].getNumMappedStaticPts();
		readyForKeyFrame[i] = IsReadyForKeyFrame(i);
		if (readyForKeyFrame[i] > 0)
			nReady++;
		if (readyForKeyFrame[i] == READY_FOR_KEY_FRAME_DECREASE)
			decrease = true;
	}
	if (nReady > 0) {
		for (int i = 0; i < numCams; i++) {
			if ((readyForKeyFrame[i] > 1 && numCams > 1)
					|| (readyForKeyFrame[i] > 0 && numCams == 1)) {
				if (m_groupId[i] == m_mergedgid
						&& curFrame > m_lastFrmGroupMerge
						&& curFrame < m_lastFrmGroupMerge + 130)
					continue;
				enterBACriticalSection();
				vector<MapPoint*> newMapPts;
				int iNum = slam[i].newMapPoints(newMapPts);
				num += iNum;
				m_lastFrmIntraMapping = curFrame;
				for (size_t k = 0; k < newMapPts.size(); k++) {
					curMapPts.add(newMapPts[k]);
				}
				leaveBACriticalSection();

			}
		}

		if (decrease) {
			//for bundle adjsutment and group merging
			//add a new key frame
			KeyFrame* pKeyFrame = addKeyFrame(readyForKeyFrame);

			//reset the pointers to previous feature points to those on the previous key frame.
			//resetPrevFeaturePoints(pKeyFrame);

			//check if there are camera groups that can be merged
			bool merged = false;
			if (m_groupNum > 1)
				merged = mergeCamGroups(pKeyFrame);

			//send a request for calling bundle adjustment	
			//for less than 4 cameras
			//requestForBA(15, 2, 2, 30);

			//for more than 4 cameras
			if (curFrame > m_lastFrmGroupMerge + 2) {
				requestForBA(5, 2, 2, 30);
			}

			for (int i = 0; i < numCams; i++) {
				if (readyForKeyFrame[i] == READY_FOR_KEY_FRAME_DECREASE
						&& numCams > 1) {
					enterBACriticalSection();
					vector<MapPoint*> newMapPts;
					int iNum = slam[i].newMapPoints(newMapPts);
//					//test
//					logInfo("%d new map points are generated at %d!\n", iNum,
//							i);
					num += iNum;
					m_lastFrmIntraMapping = curFrame;
					for (size_t k = 0; k < newMapPts.size(); k++) {
						curMapPts.add(newMapPts[k]);
					}
					leaveBACriticalSection();
				}
			}
		}
	}

//	if (curFrame % 280 == 0) {
////		pause();
//		KeyFrame* pKeyFrame = addKeyFrame(readyForKeyFrame);
//		//reset the pointers to previous feature points to those on the previous key frame.
//		//resetPrevFeaturePoints(pKeyFrame);
//		//check if there are camera groups that can be merged
//		if (m_groupNum > 1)
//			mergeCamGroups(pKeyFrame);
//	}

	if (numCams == 1)
		return num;

	if (decrease || curFrame - m_lastFrmInterMapping > 3) {
		num += genNewMapPointsInterCam(false);
		m_lastFrmInterMapping = curFrame;
		logInfo("curFrame:%d, m_lastFrmInterMapping:%d, %d new points\n",
				curFrame, m_lastFrmInterMapping, num);
		//pause();
	}
	return num;
}

void CoSLAM::adjustCurrentCamPoses() {
	//	//Do nothing
	//	return;

	//	InterCamPoseEstimator poseEstimator;
	//	poseEstimator.setCoSLAM(this);
	//	poseEstimator.addMapPoints();
	//	poseEstimator.apply();
	//
	//	//update the camera poses
	//	for (int i = 0; i < numCams; i++) {
	//		//CamPos* camPos = slam[i].m_camPos.add(curFrame, slam[i].camId, poseEstimator.Rs.data + i * 9, poseEstimator.Ts.data + i * 3);
	//		CamPos* camPos = slam[i].m_camPos.current();
	//		memcpy(camPos->R, poseEstimator.Rs.data + i * 9, sizeof(double) * 9);
	//		memcpy(camPos->t, poseEstimator.Ts.data + i * 3, sizeof(double) * 3);
	//	}

	using namespace std;
	vector<MapPoint*> vecMapPoints;
	vector<vector<FeaturePoint*> > vecFeatPoints;

	vector<Point3d> vecPoint3d;
	vector<vector<Meas2D> > vecMeas2D;
	vector<Mat_d> Ks, Rs, Ts;

	for (int c = 0; c < numCams; c++) {
		CamPoseItem* camPos = slam[c].m_camPos.current();
		Ks.push_back(Mat_d(3, 3, slam[c].K.data));
		Rs.push_back(Mat_d(3, 3, camPos->R));
		Ts.push_back(Mat_d(3, 1, camPos->t));
	}

	int nStatic = 0;
	for (MapPoint* mpt = curMapPts.getHead(); mpt; mpt = mpt->next) {
		if (mpt->isCertainStatic()) {
			vecMapPoints.push_back(mpt);
			vecPoint3d.push_back(Point3d(mpt->x, mpt->y, mpt->z));
			vecFeatPoints.push_back(vector<FeaturePoint*>());
			vecMeas2D.push_back(vector<Meas2D>());
			for (int c = 0; c < numCams; c++) {
				FeaturePoint* fp = mpt->pFeatures[c];
				if (fp) {
					vecFeatPoints.back().push_back(fp);
					vecMeas2D.back().push_back(Meas2D(c, fp->x, fp->y));
				}
			}
			nStatic++;
		}
	}

	for (MapPoint* mpt = curMapPts.getHead(); mpt; mpt = mpt->next) {
		if (mpt->bNewPt) {
			vecMapPoints.push_back(mpt);
			vecPoint3d.push_back(Point3d(mpt->x, mpt->y, mpt->z));
			vecFeatPoints.push_back(vector<FeaturePoint*>());
			vecMeas2D.push_back(vector<Meas2D>());
			for (int c = 0; c < numCams; c++) {
				FeaturePoint* fp = mpt->pFeatures[c];
				if (fp) {
					vecFeatPoints.back().push_back(fp);
					vecMeas2D.back().push_back(Meas2D(c, fp->x, fp->y));
				}
			}
		}
	}

	//test
	printf("nstatic:%d\n", nStatic);
	bundleAdjustRobust(0, Ks, Rs, Ts, nStatic, vecPoint3d, vecMeas2D, 6, 5, 30);

	//write back 
	for (int c = 0; c < numCams; c++) {
		CamPoseItem* camPos = slam[c].m_camPos.current();
		memcpy(camPos->R, Rs[c].data, sizeof(double) * 9);
		memcpy(camPos->t, Ts[c].data, sizeof(double) * 3);
	}

	for (size_t i = 0; i < vecMapPoints.size(); i++) {
		MapPoint* mpt = vecMapPoints[i];
		memcpy(mpt->M, vecPoint3d[i].M, sizeof(double) * 3);
	}
}
bool CoSLAM::mergeCamGroups(KeyFrame* newFrame) {
	MergeCameraGroup mcg;
	mcg.setCurrentFrame(newFrame);
	for (int i = 0; i < numCams; i++)
		mcg.setImageSize(i, slam[i].m_img.w, slam[i].m_img.h);

	if( curFrame < 5600 && curFrame < m_lastFrmGroupMerge + 130)
		return false;
	
	if (((curFrame > 1475 && curFrame < 1530)
			|| (curFrame > 1758 && curFrame < 1850)
			|| (curFrame > 2800 && curFrame < 3000)
			|| (curFrame > 3600 && curFrame < 3900)
			|| (curFrame > 4150 && curFrame < 4241) || (curFrame > 5600))

	&& mcg.checkPossibleMergable(10, 0.5, Param::maxDistRatio) > 0) {
		//test
		//pause();
#ifdef WIN32
		Sleep(3000);
#else
		sleep(3);
#endif
		//store the images and feature points
		for (int i = 0; i < numCams; i++)
			newFrame->pPose[i]->setImage(slam[i].m_img);

		mcg.storeFeaturePoints();
		mcg.printMergeInfo();

		if (mcg.matchMergableCameras() > 0) {
			//start to compute the new positions of key camera poses
			//test
			logInfo("before correction!\n");
			mcg.printMergeInfo();
			MyApp::bCancelBA = true;
			//test
			//pause();

			//search for the first key frame when all the separated cameras are in the same group.
			mcg.searchFirstKeyFrameForMerge();

			mcg.constructGraphForKeyFrms();
			logInfo("keyFrms\n");
			//pause();

			mcg.constructGraphForAllFrms();
			logInfo("allFrms\n");
			//pause();

			enterBACriticalSection();
			mcg.recomputeKeyCamPoses();
			leaveBACriticalSection();

			logInfo("recomputed keyframes\n");
			//pause();
			enterBACriticalSection();
			mcg.recomputeAllCameraPoses();

			m_mergedgid = mcg.mergeMatchedGroups(m_groups, m_groupNum);

			//update group id
			for (int g = 0; g < m_groupNum; ++g) {
				for (int i = 0; i < m_groups[g].num; ++i) {
					int c = m_groups[g].camIds[i];
					m_groupId[c] = g;
				}
			}
			leaveBACriticalSection();

			printCamGroup();
			logInfo("camera positions have been updated!\n");
			//test
			//pause();

			enterBACriticalSection();
			vector<MapPoint*> keyMapPoints;
			int f_start =
					mcg.getFirstFrame() < m_lastReleaseFrm ?
							m_lastReleaseFrm : mcg.getFirstFrame();

			int f_end = mcg.getLastFrame();
			getMapPts(f_start, f_end, keyMapPoints);
			mcg.recomputeMapPoints(keyMapPoints, Const::PIXEL_ERR_VAR);
			leaveBACriticalSection();
			logInfo("point positions have been updated!\n");
			//test
			//pause();

			currentMapPointsRegister(20.0, true);
			logInfo("feature points have been merged!\n");

			m_lastFrmGroupMerge = curFrame;
			//pause();
			return true;
		}
	}
	return false;
}
void CoSLAM::getNumDynamicStaticPoints() {
	//check whether there are too many dynamic map points
	MapPoint* pM = curMapPts.getHead();
	m_nStatic = 0;
	m_nDynamic = 0;
	for (int j = 0; j < numCams; j++) {
		m_nStaticFeat[j] = 0;
		m_nDynamicFeat[j] = 0;
	}
	for (; pM; pM = pM->next) {
		if (pM->isCertainStatic())
			m_nStatic++;
		else if (pM->isCertainDynamic())
			m_nDynamic++;

		for (int j = 0; j < numCams; j++) {
			if (pM->pFeatures[j]) {
				if (pM->isCertainStatic())
					m_nStaticFeat[j]++;
				else if (pM->isCertainDynamic())
					m_nDynamicFeat[j]++;
			}
		}
	}
	printf("m_nStatic %d, m_nDynamic %d\n", m_nStatic, m_nDynamic);
	printf("m_nStaticFeat[0] %d, m_nStaticFeat[1] %d, m_nDynamicFeat[0] %d, m_nDynamicFeat[1] %d\n",
			m_nStaticFeat[0], m_nStaticFeat[1], m_nDynamicFeat[0], m_nDynamicFeat[1]);
}
//b#define USE_GPUSURF
int CoSLAM::genNewMapPointsInterCam(bool bUseSURF) {
	int num = 0;
	if (bUseSURF) {
#ifdef USE_GPUSURF
		TimeMeasurer tm;
		tm.tic();
		NewMapPtsSURF matcher;

		const int thresNum = 400;
		for (int i = 0; i < m_groupNum; i++) {
			if (m_groups[i].num <= 1)
			continue;
			bool runMatching = false;
			enterBACriticalSection();
			for (int k = 0; k < m_groups[i].num; k++) {
				int camId = m_groups[i].camIds[k];
				if (m_nStaticFeat[camId] < thresNum) {
					runMatching = true;
					break;
				}
			}
			if (curFrame < 5)
			runMatching = false;
			if (runMatching) {
				matcher.setInputs(m_groups[i], this, curFrame);
				num += matcher.run();
				matcher.output();
				//test
				//logInfo("frame:%d, group (#%d) : inter-camera mapping :%d \n", curFrame, i, num);
			}
			leaveBACriticalSection();
		}
		m_tmNewMapPoints = tm.toc();
#endif
	} else {
		TimeMeasurer tm;
		tm.tic();
		NewMapPtsNCC matcher;

		//to avoid frequently calling inter-camera mapping if there are enough map points already
		const int thresNum = Param::nMaxMapPts;
		for (int i = 0; i < m_groupNum; i++) {
			if (m_groups[i].num <= 1)
				continue;
			bool runMatching = false;
			enterBACriticalSection();
			for (int k = 0; k < m_groups[i].num; k++) {
				int camId = m_groups[i].camIds[k];
				if (m_nStaticFeat[camId] < thresNum) {
					runMatching = true;
					break;
				}
			}
			//skip first five frames
			if (curFrame < 5)
				runMatching = false;

			if (runMatching) {
				matcher.setInputs(m_groups[i], this, curFrame);
				num += matcher.run();
				matcher.output();
			}

			leaveBACriticalSection();
		}
		m_tmNewMapPoints = tm.toc();
	}
	return num;
}

void CoSLAM::getViewOverlapCosts(double vcosts[SLAM_MAX_NUM * SLAM_MAX_NUM],
		int minOverlapNum, double minOverlapAreaRatio) {
	int nSharePoints[SLAM_MAX_NUM * SLAM_MAX_NUM];
	memset(nSharePoints, 0, numCams * numCams * sizeof(int));

	//store the shared points : sharePoints[iCam][jCam] represents feature points between camera #iCam and #jCam.
	for (int i = 0; i < numCams; i++) {
		for (int j = 0; j < numCams; j++) {
			if (i == j)
				continue;
			sharedPoints[i][j].clear();
			sharedPoints[i][j].reserve(8192);
			sharedConvexHull[i][j].clear();
			sharedConvexHull[i][j].reserve(1024);
		}
	}

	//compute the number of shared feature points between cameras
	for (const MapPoint* pt = curMapPts.getHead(); pt; pt = pt->next) {
		if (pt->numVisCam == 1 || pt->isFalse())
			continue;
		int viewIds[SLAM_MAX_NUM];
		double points[SLAM_MAX_NUM * 2];
		int nIds = 0;
		for (int i = 0; i < numCams; i++) {
			const FeaturePoint* fp = pt->pFeatures[i];
			if (fp && fp->f == curFrame) {
				points[2 * nIds] = fp->x;
				points[2 * nIds + 1] = fp->y;
				viewIds[nIds++] = i;
			}
		}

		for (int i = 0; i < nIds; i++) {
			int iCam = viewIds[i];
			for (int j = i + 1; j < nIds; j++) {
				int jCam = viewIds[j];
				nSharePoints[iCam * numCams + jCam]++;
				nSharePoints[jCam * numCams + iCam]++;

				double ix = points[2 * i];
				double iy = points[2 * i + 1];

				double jx = points[2 * j];
				double jy = points[2 * j + 1];

				sharedPoints[iCam][jCam].push_back(ix);
				sharedPoints[iCam][jCam].push_back(iy);
				sharedPoints[jCam][iCam].push_back(jx);
				sharedPoints[jCam][iCam].push_back(jy);
			}
		}
	}

	for (int i = 0; i < numCams; i++) {
		for (int j = 0; j < numCams; j++) {
			if (j == i)
				continue;
			get2DConvexHull(sharedPoints[i][j], sharedConvexHull[i][j]);
		}
	}

	for (int i = 0; i < numCams; i++) {
		vcosts[i * numCams + i] = -1;
		for (int j = i + 1; j < numCams; j++) {
			if (nSharePoints[i * numCams + j] < minOverlapNum) {
				vcosts[i * numCams + j] = -1;
				vcosts[j * numCams + i] = -1;
			} else {
				//check the overlap area
				double area1 = getPolyArea(sharedConvexHull[i][j]);
				double area2 = getPolyArea(sharedConvexHull[j][i]);

				double iArea = slam[i].videoReader->_w * slam[i].videoReader->_h;
				double jArea = slam[j].videoReader->_w * slam[j].videoReader->_h;

				if (area1 < minOverlapAreaRatio * iArea
						|| area2 < minOverlapAreaRatio * jArea) {
					vcosts[i * numCams + j] = -1;
					vcosts[j * numCams + i] = -1;
				} else {
					vcosts[i * numCams + j] = nSharePoints[i * numCams + j];
					vcosts[j * numCams + i] = vcosts[i * numCams + j];
				}
			}
		}
	}
}

#include "tools/SL_Print.h"
int CoSLAM::cameraGrouping() {
	if (numCams == 1)
		return 1;
	getViewOverlapCosts(viewOverlapCost, 0, 0.0);

	//check distances between cameras
	for (int i = 0; i < numCams; ++i) {
		//cout << i << " : ";
		for (int j = i + 1; j < numCams; ++j) {
			double M1[3], M2[3];
			getCamCenter(slam[i].m_camPos.current(), M1);
			getCamCenter(slam[j].m_camPos.current(), M2);

			//test
			double d = dist3(M1, M2);
			//cout << " " << j << "-" << d << " ";
			if (viewOverlapCost[i * numCams + j] > 0
					&& dist3(M1, M2)
							> m_initCamTranslation * Param::maxDistRatio) {

				viewOverlapCost[i * numCams + j] = -1;
				viewOverlapCost[j * numCams + i] = -1;
			}
		}
		cout << endl;
	}
	//cout << m_initCamTranslation * Param::maxDistRatio << endl;

	//divide the cameras into groups according to the overlap costs 
	//each connect component acts as a camera group
	int VQ[SLAM_MAX_NUM], CON[SLAM_MAX_NUM], flag[SLAM_MAX_NUM], nVQ = 0, nCON =
			0;
	m_groupNum = 0;
	fill_n(flag, numCams, 0);

	for (int i = 0; i < numCams; i++) {
		if (0 == flag[i]) {
			//a new connected components
			nVQ = nCON = 0;
			CON[nCON++] = i;
			VQ[nVQ++] = i;
			flag[i] = 1;

			while (nVQ > 0) {
				//pop from the stack
				int iCam = VQ[nVQ - 1];
				nVQ--;

				//add the connected vertices into the stack
				for (int j = 0; j < numCams; j++) {
					if (j != iCam && flag[j] == 0
							&& viewOverlapCost[iCam * numCams + j] > 0) {
						CON[nCON++] = j;
						VQ[nVQ++] = j;
						flag[j] = 1;
					}
				}
			}
			//output the connect components
			m_groups[m_groupNum].clear();
			for (int k = 0; k < nCON; k++) {
				m_groups[m_groupNum].addCam(CON[k]);
				m_groupId[CON[k]] = m_groupNum;
			}
			m_groupNum++;
		}
	}
	return m_groupNum;
}

/*
 * process the queued BA requests
 */
void* _bundleAdjustmentThread(void* param) {
	std::deque<RobustBundleRTS*>& queueBAs =
			*((std::deque<RobustBundleRTS*>*) param);
	MyApp::bBusyBAing = true;
	while (!queueBAs.empty()) {
		//start of critical section
		enterBACriticalSection();
		RobustBundleRTS* pBA = queueBAs.front();
		queueBAs.pop_front();
		leaveBACriticalSection();
		//end of critical section
		pBA->run();
		//start of critical section
		enterBACriticalSection();
		//test
		logInfo("BA!\n");
		if (!MyApp::bCancelBA)
			pBA->output();
		int firstKeyFrm_ = pBA->firstKeyFrame->f;
		int secondKeyFrm_ = pBA->lastKeyFrame->f;
		delete pBA;
		MyApp::bCancelBA = false;
		leaveBACriticalSection();
		//end of critical section
		//test
		logInfo("bundle adjustment completed!(%d - %d)\n",
				firstKeyFrm_, secondKeyFrm_);
	}
	MyApp::bBusyBAing = false;
	//test
	logInfo("bundle adjustment completed2!\n");
	return 0;
}
bool CoSLAM::requestForBA(int keyFrameNum, int maxOldKeyFrmNum, int maxIter,
		int innerMaxIter) {
	//get key frames for bundle adjustment
	KeyFrame* pLastKF = m_keyFrms.current();
	KeyFrame* pKF = pLastKF;

	int nKeyFrm = 0;
	int nPts = 0;

	KeyFrame* pFirstKF = 0;
	for (; nKeyFrm < keyFrameNum && pKF; nKeyFrm++, pKF = pKF->prev) {
		pFirstKF = pKF;
		nPts += pKF->nMapPts;
	}

	if (nKeyFrm < keyFrameNum) {
		logInfo(
				"not enough key frames for bundle adjustment[last %d, now %d]\n",
				m_lastFrmBundle, curFrame);
		return false;
	}
	if (MyApp::bBusyBAing) {
		logInfo(
				"waiting for the end of the last bundle adjustment[last %d, now %d]\n",
				m_lastFrmBundle, curFrame);
		return false;
	}
	//record the last key frame for bundle adjustment
	m_firstFrmBundle = pFirstKF->f;
	m_lastFrmBundle = pLastKF->f;
	RobustBundleRTS* pBA = new RobustBundleRTS();

	//allocate enough memory
	pBA->setCoSLAM(this);
	pBA->setFirstKeyFrame(pFirstKF);
	pBA->setLastKeyFrame(pLastKF);
	pBA->addKeyFrames();
	pBA->addPoints();
	pBA->setParameters(2, numCams * maxOldKeyFrmNum, maxIter, innerMaxIter);

	//start of critical section
	enterBACriticalSection();
	m_requestBAs.push_back(pBA);
	leaveBACriticalSection();
	//end of critical section

	if (!MyApp::bBusyBAing) {
		//start a new thread for bundle adjustment
		pthread_t thread;
		pthread_create(&thread, 0, _bundleAdjustmentThread, &m_requestBAs);
	}

	return true;
}

void CoSLAM::releaseFeatPts(int frame) {
	enterBACriticalSection();
	for (int i = 0; i < numCams; i++) {
		slam[i].removeFeatPts(frame);
	}
	leaveBACriticalSection();
}
void CoSLAM::releaseKeyPoseImgs(int frame) {
	enterBACriticalSection();
	for (int i = 0; i < numCams; ++i) {
		slam[i].removeKeyPoseImgs(frame);
	}
	leaveBACriticalSection();
}
#include "opencv2/opencv.hpp"
void CoSLAM::pause() {
	updateDisplayData();
	MyApp::bStop = true;
	redrawAllViews();
	while (MyApp::bStop) {
	};
}
void CoSLAM::printCamGroup() {
	for (int i = 0; i < m_groupNum; ++i) {
		cout << "group " << i << ": ";
		for (int j = 0; j < m_groups[i].num; ++j) {
			cout << m_groups[i].camIds[j] << " ";
		}
		cout << endl;
	}
}
void CoSLAM::drawResultAllViews() {
	char wndName[256];
	ImgRGB outImg;
	for (int i = 0; i < numCams; i++) {
		int rows = slam[i].H;
		int cols = slam[i].W;
		outImg.resize(cols, rows);

		cv::Mat cvImg(rows, cols, CV_8UC1, slam[i].m_img.data);
		cv::Mat cvOutImg(rows, cols, CV_8UC3, outImg.data);
		cv::cvtColor(cvImg, cvOutImg, CV_GRAY2RGB);

		int f = curFrame;

		FeaturePoints& fps = slam[i].m_featPts;
		FeaturePoint* pHead = fps.getFrameHead(f);
		FeaturePoint* pTail = fps.getFrameTail(f);

		for (FeaturePoint* pt = pHead; pt != pTail->next; pt = pt->next) {
			if (pt->mpt) {
				cv::circle(cvOutImg, cv::Point2f(pt->xo, pt->yo),
						3 * (pt->mpt->numVisCam - 1), cv::Scalar(0, 255, 0), 1,
						CV_AA);
				//draw re-projected points
				double m[2];
				project(slam[i].K, slam[i].m_camPos.current()->R,
						slam[i].m_camPos.current()->t, pt->mpt->M, m);
				cv::line(cvOutImg, cv::Point2f(pt->xo, pt->yo),
						cv::Point2f(m[0], m[1]), cv::Scalar(255, 0, 0), 2,
						CV_AA);
			} else {
				cv::circle(cvOutImg, cv::Point2f(pt->xo, pt->yo), 3,
						cv::Scalar(200, 200, 200), 1, CV_AA);
			}
		}
		sprintf(wndName, "cam%d", i);
		imshow(wndName, outImg);
	}
	cv::waitKey(-1);
}
void CoSLAM::getMapPts(int firstFrame, int lastFrame,
		vector<MapPoint*>& mapPoints) {
	//scan all maps
	MapPointList* mptLst = &iactMapPts;
	//test
	int k = 0;
	for (MapPoint* mpt = mptLst->getHead(); mpt; mpt = mpt->next) {
		k++;
		assert(mpt);
		if (mpt->lastFrame < firstFrame || mpt->firstFrame > lastFrame) {
			continue;
		}
		if (mpt->isCertainStatic())
			mapPoints.push_back(mpt);
	}
	mptLst = &actMapPts;
	for (MapPoint* mpt = mptLst->getHead(); mpt; mpt = mpt->next) {
		assert(mpt);
		if (mpt->lastFrame < firstFrame || mpt->firstFrame > lastFrame) {
			continue;
		}
		if (mpt->isCertainStatic())
			mapPoints.push_back(mpt);
	}
	mptLst = &curMapPts;
	for (MapPoint* mpt = mptLst->getHead(); mpt; mpt = mpt->next) {
		assert(mpt);
		if (mpt->lastFrame < firstFrame || mpt->firstFrame > lastFrame) {
			continue;
		}
		if (mpt->isCertainStatic())
			mapPoints.push_back(mpt);
	}
}
void CoSLAM::getAllStaticMapPoints(vector<MapPoint*>& mptPts) const {
//	for (MapPoint* mpt = curMapPts.getHead(); mpt; mpt = mpt->next) {
//		if (mpt->isCertainStatic())
//			mptPts.push_back(mpt);
//	}
//
//	for (MapPoint* mpt = actMapPts.getHead(); mpt; mpt = mpt->next) {
//		if (mpt->isCertainStatic())
//			mptPts.push_back(mpt);
//	}
//
//	for (MapPoint* mpt = iactMapPts.getHead(); mpt; mpt = mpt->next) {
//		if (mpt->isCertainStatic())
//			mptPts.push_back(mpt);
//	}
	set<MapPoint*> mptSet;
	int num = 0;
	for (int c = 0; c < numCams; c++) {
		for (int f = 0; f <= curFrame; f++) {
			vector<FeaturePoint*> vecFeatPts;
			slam[c].m_featPts.getFrame(f, vecFeatPts);
			for (size_t i = 0; i < vecFeatPts.size(); i++) {
				if (vecFeatPts[i]->mpt
						&& vecFeatPts[i]->mpt->isCertainStatic()) {
					vecFeatPts[i]->mpt->id = (longInt) vecFeatPts[i]->mpt;
					mptSet.insert(vecFeatPts[i]->mpt);
					num++;
				}
			}
		}
	}
	mptPts.clear();
	mptPts.reserve(2 * num);
	for (set<MapPoint*>::iterator iter = mptSet.begin(); iter != mptSet.end();
			iter++) {
		mptPts.push_back(*iter);
	}
}
void CoSLAM::getAllStaticMapPtsAtKeyFrms(vector<MapPoint*>& mapPoints) const {
	mapPoints.clear();
	for (KeyFrame* kf = m_keyFrms.head.next; kf; kf = kf->next) {
		for (int c = 0; c < kf->nCam; c++) {
			FeaturePoint* pHead = kf->pPose[c]->pHead;
			FeaturePoint* pTail = kf->pPose[c]->pTail;
			for (FeaturePoint* fp = pHead; fp != pTail->next; fp = fp->next) {
				if (fp->mpt && !fp->mpt->isFalse()
						&& fp->mpt->isCertainStatic()) {
					mapPoints.push_back(fp->mpt);
				}
			}
		}
	}
}
void CoSLAM::getAllFeatPtsAtKeyFrms(int c, vector<FeaturePoint*>& featPoints) {
	featPoints.clear();
	for (KeyFrame* kf = m_keyFrms.head.next; kf; kf = kf->next) {
		FeaturePoint* pHead = kf->pPose[c]->pHead;
		FeaturePoint* pTail = kf->pPose[c]->pTail;
		for (FeaturePoint* fp = pHead; fp != pTail->next; fp = fp->next)
			featPoints.push_back(fp);
	}
}
void CoSLAM::storeDynamicPoints() {
	if (numCams == 1)
		return;
	using namespace std;
	std::vector<Point3dId> pts;
	for (MapPoint* mpt = curMapPts.getHead(); mpt; mpt = mpt->next) {
		if (mpt->isCertainDynamic()) {
			pts.push_back(Point3dId(mpt->x, mpt->y, mpt->z, mpt->id));
		}
	}
	m_dynPts.push_back(pts);
}

#include <time.h>
void CoSLAM::exportResultsVer1(const char timeStr[]) const {
	using namespace std;
	char dirPath[256];
	sprintf(dirPath, "/home/rui/workspace/coslam/slam_results/%s", timeStr);
#ifdef WIN32
	mkdir(dirPath);
#else
	mkdir(dirPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif

	char filePath[256];

	//save the information of the input video sequences 
	//and camera parameters
	sprintf(filePath, "%s/input_videos.txt", dirPath);
	ofstream file(filePath);
	if (!file)
		repErr("cannot open file %s to write!\n", filePath);
	for (int c = 0; c < numCams; c++) {
		file << slam[c].videoFilePath << endl;
		for (size_t i = 0; i < 9; i++)
			file << slam[c].K[i] << " ";
		file << endl;
		for (size_t i = 0; i < 5; i++)
			file << slam[c].k_c.data[i] << " ";
		file << endl;
		file << slam[c].W << " " << slam[c].H << endl;
	}
	file.close();
	cout << filePath << " has been saved!" << endl;

	//save map points
	sprintf(filePath, "%s/mappts.txt", dirPath);

	file.open(filePath);
	if (!file)
		repErr("cannot open file to write!\n");

	set<MapPoint*> mptSet;
	vector<MapPoint*> mapPoints;
	getAllStaticMapPoints(mapPoints);

	file << mapPoints.size() << endl;
	for (size_t i = 0; i < mapPoints.size(); i++) {
		//test
		mptSet.insert(mapPoints[i]);

		file << mapPoints[i]->id << endl;
		file << mapPoints[i]->x << " " << mapPoints[i]->y << " "
				<< mapPoints[i]->z << endl;
		for (size_t i = 0; i < 9; i++)
			file << mapPoints[i]->cov[i] << " ";
		file << endl;
	}
	file.close();
	cout << filePath << " has been saved!" << endl;

	//save camera poses
	FILE* camFile;
	for (int c = 0; c < numCams; c++) {
		sprintf(filePath, "%s/%d_campose.txt", dirPath, c);
//		file.open(filePath);
		camFile = fopen(filePath, "w");
//		if (!file)
//			repErr("cannot open file '%s' to write!\n", filePath);

		int nCam = slam[c].m_camPos.size();
//		file << nCam << endl;
		fprintf(camFile, "%d\n", nCam);
		for (CamPoseItem* cam = slam[c].m_camPos.first(); cam;
				cam = cam->next) {

//			file << getFrameInVideo(c, cam->f) << " " << std::setprecision(8) <<  cam->ts << " ";
////			printf("%d %lf ", getFrameInVideo(c, cam->f), cam->ts);
//			for (size_t i = 0; i < 9; i++)
//				file << cam->R[i] << " ";
//			file << cam->t[0] << " " << cam->t[1] << " " << cam->t[2] << endl;

			fprintf(camFile, "%d %.6lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
					getFrameInVideo(c, cam->f),
					cam->ts,
					cam->R[0], cam->R[1], cam->R[2],
					cam->R[3], cam->R[4], cam->R[5],
					cam->R[6], cam->R[7], cam->R[8],
					cam->t[0], cam->t[1], cam->t[2]);

		}
//		file.close();
		fclose(camFile);
		cout << filePath << " has been saved!" << endl;
	}

	//save feature points
	for (int c = 0; c < numCams; c++) {
		sprintf(filePath, "%s/%d_featpts.txt", dirPath, c);
		file.open(filePath);
		if (!file)
			repErr("cannot open file '%s' to write!\n", filePath);

		int nf = 0;
		for (int f = 0; f <= curFrame; f++)
			if (f >= slam[c].m_camPos.first()->f)
				nf++;
		file << nf << endl;

		for (int f = 0; f <= curFrame; f++) {
			if (f < slam[c].m_camPos.first()->f)
				continue;
			vector<FeaturePoint*> vecFeatPts;
			slam[c].m_featPts.getFrame(f, vecFeatPts);
			vector<FeaturePoint*> staticPts;
			for (size_t i = 0; i < vecFeatPts.size(); i++) {
				if (vecFeatPts[i]->mpt
						&& vecFeatPts[i]->mpt->isCertainStatic()) {
					staticPts.push_back(vecFeatPts[i]);
					assert(mptSet.count(vecFeatPts[i]->mpt) > 0);
				}
			}
			file << getFrameInVideo(c, f) << " " << staticPts.size() << endl;
			for (size_t i = 0; i < staticPts.size(); i++) {
				file << staticPts[i]->mpt->id << " " << staticPts[i]->x << " "
						<< staticPts[i]->y << " " << staticPts[i]->xo << " " <<
						staticPts[i]->yo << "\n";
			}
			file << endl;
		}
		file.close();
		cout << filePath << " has been saved!" << endl;
	}

	//save reprojection error
	for (int c = 0; c < numCams; c++) {
		sprintf(filePath, "%s/%d_reprojErrStatic.txt", dirPath, c);
		file.open(filePath);
		if (!file)
			repErr("cannot open file '%s' to write!\n", filePath);

		for (vector<float>::iterator it= MyApp::s_reprojErrStatic[c].begin();
				it != MyApp::s_reprojErrStatic[c].end();it++){
			file << *it << endl;
		}
		file.close();
		cout << filePath << " has been saved!" << endl;

		sprintf(filePath, "%s/%d_reprojErrDynamic.txt", dirPath, c);
		file.open(filePath);
		if (!file)
			repErr("cannot open file '%s' to write!\n", filePath);

		for (vector<float>::iterator it= MyApp::s_reprojErrDynamic[c].begin();
				it != MyApp::s_reprojErrDynamic[c].end();it++){
			file << *it << endl;
		}
		file.close();
		cout << filePath << " has been saved!" << endl;

		sprintf(filePath, "%s/%d_framenumber.txt", dirPath, c);
		file.open(filePath);
		if (!file)
			repErr("cannot open file '%s' to write!\n", filePath);

		for (vector<int>::iterator it= MyApp::s_frameNumber[c].begin();
				it != MyApp::s_frameNumber[c].end();it++){
			file << *it << endl;
		}
		file.close();
		cout << filePath << " has been saved!" << endl;
	}
}
void CoSLAM::exportResults(const char timeStr[]) const {
	exportResultsVer1(timeStr);
}

//void CoSLAM::saveCurrentFrame(const char timeStr[]) const {
//	char dirPath[256];
//	sprintf(dirPath, "/home/zou/slam_results/%s", timeStr);
//	mkdir(dirPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
//
//	char filePath[256];
//	sprintf(filePath, "%s/%d_result.txt", dirPath, curFrame);
//	ofstream ofs(filePath);
//	if (!ofs)
//		repErr("cannot open file %s to write!\n", filePath);
//
//	boost::archive::text_oarchive oa(ofs);
//	//output map points and current feature points
//	using namespace std;
//	vector<Point3d> vecMapPts;
//	vector<vector<Meas2D> > vecFeatPts;
//	vecFeatPts.resize(numCams);
//	for (const MapPoint * mpt = curMapPts.getHead(); mpt; mpt = mpt->next) {
//		if (mpt->isCertainStatic()) {
//			vecMapPts.push_back(Point3d(mpt->x, mpt->y, mpt->z));
//			for (int i = 0; i < numCams; i++) {
//				const FeaturePoint* fp = mpt->pFeatures[i];
//				if (fp && fp->f == curFrame)
//					vecFeatPts[i].push_back(Meas2D(i, fp->x, fp->y));
//			}
//		}
//	}
//
//	vector<Mat_d> Ks, Rs, Ts;
//	for (int i = 0; i < numCams; i++) {
//		Ks.push_back(Mat_d(3, 3, slam[i].K.data));
//		Rs.push_back(Mat_d(3, 3, slam[i].m_camPos.current()->R));
//		Ts.push_back(Mat_d(3, 1, slam[i].m_camPos.current()->t));
//	}
//
//	oa << vecMapPts;
//	oa << vecFeatPts;
//	oa << Ks;
//	oa << Rs;
//	oa << Ts;
//
//	ofs.close();
//
//	//save the images
//	for (int i = 0; i < numCams; i++) {
//		sprintf(filePath, "/home/zou/slam_results/%s/%d_img_%d.ppm", timeStr,
//				curFrame, i);
//		savePPM(slam[i].m_rgb, filePath);
//	}
//	//test
//	logInfo("The result at frame #%d has been saved!\n", curFrame);
//}
void CoSLAM::saveCurrentImages(const char* dirPath) const {
#ifdef WIN32
	mkdir(dirPath);
#else
	mkdir(dirPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
	for (int i = 0; i < numCams; i++) {
		char filePath[1024];
		sprintf(filePath, "%s/cam_%d.pgm", dirPath, i);
		savePGM(slam[i].m_img, filePath);
	}
	logInfo("save images OK!\n");
}

void CoSLAM::startRelocalization(int camId) {
	state[camId] = SLAM_STATE_RELOCATE;
	_frmNumAfterReloc = -1;
	m_relocalizer[camId].reset();
}

bool CoSLAM::doRelocalization(int camid){
	if (m_relocalizer[camid].isCameraSteady())
			return m_relocalizer[camid].tryToRecover();
	return false;
}

void CoSLAM::endRelocalization(int camid){
	state[camid] = SLAM_STATE_RECOVERED;
	_frmNumAfterReloc = Param::frmNumAfterRelocalization;
	cout << "Recovered!" << endl;
}

bool CoSLAM::allCamerasNormal(){
	for (int i = 0; i < numCams; i++)
		if ( state[i] != SLAM_STATE_NORMAL)
			return false;
	return true;
}

bool CoSLAM::allCamerasInGroupNormal(int groupId){
	if (groupId > m_groupNum)
		return false;

	for (int i = 0; i < m_groups[groupId].num; i++) {
		int iCam = m_groups[groupId].camIds[i];
		if (state[iCam] != SLAM_STATE_NORMAL)
			return false;
	}
	return true;
}
