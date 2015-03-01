/*
 * APP_ROSMain.cpp
 *
 *  Created on: Dec 9, 2014
 *      Author: rui
 */

#include "gui/CoSLAMThread.h"
#include "app/SL_GlobParam.h"
#include "app/SL_CoSLAM.h"
#include "tools/SL_Tictoc.h"
#include "tools/SL_Timing.h"

#include "gui/MyApp.h"
#include "slam/SL_SLAMHelper.h"

bool receiveFeatures = false;

bool ROSMain_features() {
	CoSLAM& coSLAM = MyApp::coSLAM;
	/////////////////////////1.GPU initilization/////////////////////////
	//initialization for CG;
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutCreateWindow(" ");
	glutHideWindow();

	glewInit();

	V3D_GPU::Cg_ProgramBase::initializeCg();

	//////////////////////////2.read video information//////////////////
	try {
//		for(int c = 0; c < coSLAM.numCams; c++){
//			coSLAM.slam[c].videoReader = &MyApp::aviReader[c];
//		}

		coSLAM.init(false);
		MyApp::bInitSucc = true;
		logInfo("Loading video sequences.. OK!\n");
	} catch (SL_Exception& e) {
		logInfo(e.what());
#ifdef WIN32
		wxMessageBox(e.what());
#endif
		return 0;
	}

	//notify the GUI thread to create GUIs
	MyApp::broadcastCreateGUI();

	//wait for the accomplishment of creating GUIs
	MyApp::waitCreateGUI();

	for (int i = 0; i < coSLAM.numCams; i++){
		MyApp::videoWnd[i]->setSLAMData(i, &coSLAM);
		vector<float> reprojErrStatic, reprojErrDynamic;
		vector<int> frameNumber;
		MyApp::s_reprojErrDynamic.push_back(reprojErrStatic);
		MyApp::s_reprojErrStatic.push_back(reprojErrDynamic);
		MyApp::s_frameNumber.push_back(frameNumber);
	}


	MyApp::modelWnd1->setSLAMData(&coSLAM);
	MyApp::modelWnd2->setSLAMData(&coSLAM);
	MyApp::broadcastCreateGUI();

	//for measuring the timings
	Timings timingsPerStep;
	Timings timingsReadFrame;
	Timings timingsNewMapPonits;

	/* start the SLAM process*/
	try {
		coSLAM.readFrame();
		//copy the data to buffers for display
		updateDisplayData();
		//initialise the map points
		for (int i = 0; i < coSLAM.numCams; i++) {
			printf("slam[%d].m_camPos.size(): %d\n", i, coSLAM.slam[i].m_camPos.size());
		}
//		tic();
//		coSLAM.initMap();
//		toc();
		cout <<"debug\n";

//		redis->setPose(1, 2, 3.6);

		while (!MyApp::bExit){
			pthread_mutex_lock(&MyApp::_mutexImg);
			if (!coSLAM.grabReadFrame())
				pthread_cond_wait(&MyApp::_condImg, &MyApp::_mutexImg);
			pthread_mutex_unlock(&MyApp::_mutexImg);

//			//copy the data to buffers for display
			updateDisplayData();
			//printf("current frame: %d\n", coSLAM.curFrame);
			if (MyApp::bStartInit){
				MyApp::publishMapInit();
				printf("Start initializing map...\n");
				coSLAM.curFrame = 0;
				//initialise the map points
				if (coSLAM.initMap()){

					// Need this in real test
					coSLAM.calibGlobal2Cam();
//					return 0;

					for (int i = 0; i < coSLAM.numCams; i++)
						coSLAM.state[i] = SLAM_STATE_NORMAL;
					printf("Init map success!\n");
					break;
				}
				else{
					MyApp::bStartInit = false;
					MyApp::publishMapInit();
				}
			}
		}
		updateDisplayData();
//		coSLAM.pause();

		for (int i = 0; i < coSLAM.numCams; i++) {
			printf("slam[%d].m_camPos.size(): %d\n", i, coSLAM.slam[i].m_camPos.size());
			coSLAM.state[i] = SLAM_STATE_NORMAL;
		}

//		return 0;
//		coSLAM.pause();

		int endFrame = SLAMParam::nTotalFrame - SLAMParam::nSkipFrame
				- SLAMParam::nInitFrame - 10;
//		int endFrame = 500;

//		endFrame = 1500;

		int i = 0;
		// init estimation flag
		bool bEstPose[SLAM_MAX_NUM];
		for (int i = 0; i < SLAM_MAX_NUM; i++){
			bEstPose[i] = false;
		}

		vector<double> tmStepVec;


		vector<double> poseSent[SLAM_MAX_NUM];
//		vector<double> rosTime;

		while (!MyApp::bExit) {
//			while (MyApp::bStop) {/*stop*/
//			}

//			if (MyApp::_mergeable){
//				if (MyApp::_imgReady[0] && MyApp::_imgReady[1]){
//					coSLAM.grabReadFrame();
//					MyApp::_imgAvailableForMerge = true;
//				}
//				else
//					continue;
//			}

			i++;
			TimeMeasurer tmPerStep;
			tmPerStep.tic();


			if (!receiveFeatures){
				coSLAM.grabReadFrame();
				coSLAM.featureTracking();
			}
			else{
				coSLAM.featureReceiving();
				coSLAM.virtualReadFrame();
			}

			coSLAM.poseUpdate(bEstPose);
			//Use redis to send over the poses


//			for (int i = 0; i < coSLAM.numCams; i++){
//				double org[3];
////				getCamCenter(coSLAM.slam[i].m_camPos.current(), org);
//				coSLAM.transformCamPose2Global(coSLAM.slam[i].m_camPos.current(), org);
//
//				MyApp::redis[i]->setPose(org[0], org[1], org[2]);
//
//				double ts = coSLAM.slam[i].m_camPos.current()->ts;
//				poseSent[i].push_back(ts);
//				poseSent[i].push_back(org[0]);
//				poseSent[i].push_back(org[1]);
//				poseSent[i].push_back(org[2]);
//				rosTime.push_back(ts);
//			}
//			if (MyApp::bStartMove){
//				MyApp::redis_start->setCommand("go");
//				MyApp::bStartMove = false;
//			}

			//coSLAM.pause();
			coSLAM.cameraGrouping();
			//existing 3D to 2D points robust
			coSLAM.activeMapPointsRegister(Const::PIXEL_ERR_VAR);

			TimeMeasurer tmNewMapPoints;
			tmNewMapPoints.tic();

			if (receiveFeatures){
				coSLAM.genNewMapPoints_new();
				if (MyApp::_mergeable)
					MyApp::triggerClients();
			}
			else
			{
				bool merge = false;
				coSLAM.genNewMapPoints(merge);
			}
//
//			coSLAM.m_tmNewMapPoints = tmNewMapPoints.toc();
//			cout << "coSLAM.m_tmNewMapPoints" << coSLAM.m_tmNewMapPoints << endl;

			//point registration
			coSLAM.currentMapPointsRegister(Const::PIXEL_ERR_VAR,
					i % 50 == 0 ? true : false);

			coSLAM.storeDynamicPoints();

			updateDisplayData();
			redrawAllViews();

//			Sleep(50);

			if (i % 500 == 0) {
				//coSLAM.releaseFeatPts(coSLAM.curFrame - 500);
				coSLAM.releaseKeyPoseImgs(coSLAM.curFrame - 500);
				coSLAM.m_lastReleaseFrm = coSLAM.curFrame;
			}

			coSLAM.m_tmPerStep = tmPerStep.toc();
			tmStepVec.push_back(coSLAM.m_tmPerStep);

			//Send pose
			for (int i = 0; i < coSLAM.numCams; i++)
			{
				if (coSLAM.slam[i].m_camPos.size() > 0){
					double org[3], rpy[3];
					CamPoseItem* cam = coSLAM.slam[i].m_camPos.current();
					double ts = cam->ts;
					getCamCenter(cam, org);
					coSLAM.transformCamPose2Global(cam, org, rpy);

					double targetPos[3];
					targetPos[0] = targetPos[1] = targetPos[2] = 0;
					double theta = 0;
					if(coSLAM.dynObjPresent){
						coSLAM.transformTargetPos2Global(cam->currDynPos, targetPos);
						//printf("targetPos: %lf %lf %lf\n", targetPos[0], targetPos[1], targetPos[2]);
						coSLAM.slam[i].projectTargetToCam(cam, cam->currDynPos);
						theta = atan2(coSLAM.slam[i]._targetPosInCam[0], coSLAM.slam[i]._targetPosInCam[2]);
//							double H = targetPos[2] * 2;
//							double Z = coSLAM.slam[i]._targetPosInCam[2];
						MyApp::redis[i]->setPoseTarget(ts, 1, theta, org[0], org[1], rpy[2], targetPos[0], targetPos[1], 0.9);
						MyApp::redis_dynObj->setDynObj(targetPos[0], targetPos[1], targetPos[2]);
//						printf("currDynPos: %lf %lf %lf\n", cam->currDynPos[0], cam->currDynPos[1], cam->currDynPos[2]);
					}
					else
						MyApp::redis[i]->setPoseTarget(ts, 0, theta, org[0], org[1], rpy[2], targetPos[0], targetPos[1], 0.9);

//					printf("targetPos: %lf %lf %lf\n", targetPos[0], targetPos[1], targetPos[2]);
//					printf("org[2]: %lf %lf %lf\n", org[0], org[1], rpy[2]);
				}
			}
		}

		cout << " the result is saved at " << MyApp::timeStr << endl;
		coSLAM.exportResults(MyApp::timeStr);

		FILE* fid = fopen("/home/rui/coslamTime.txt","w");
		for (int i = 0; i < tmStepVec.size(); i++){
			fprintf(fid, "%lf\n", tmStepVec[i]);
		}
		fclose(fid);

//		FILE* fid = fopen("slam_timing.txt","w");
//		for (int i = 0; i < tmStepVec.size(); i++)
//			fprintf(fid, "%f\n", tmStepVec[i]);
//		fclose(fid);
//
//		fid = fopen("poseSent0.txt","w");
//		for (int i = 0; i < poseSent[0].size(); i = i + 4)
//			fprintf(fid, "%lf %lf %lf %lf\n", poseSent[0][i],
//					poseSent[0][i+1], poseSent[0][i+2], poseSent[0][i+3]);
//		fclose(fid);
//
//		fid = fopen("poseSent1.txt","w");
//		for (int i = 0; i < poseSent[1].size(); i = i + 4)
//			fprintf(fid, "%lf %lf %lf %lf\n", poseSent[1][i],
//					poseSent[1][i+1], poseSent[1][i+2], poseSent[1][i+3]);
//		fclose(fid);
//
//		FILE* fid = fopen("rosTime.txt","w");
//		for (int i = 0; i < rosTime.size(); i = i + coSLAM.numCams + 1){
//			for (int j = 0; j <= coSLAM.numCams; j++){
//				fprintf(fid, "%lf ", rosTime[i+j]);
//			}
//			fprintf(fid, "\n");
//		}
//		fclose(fid);

		logInfo("slam finished\n");
	} catch (SL_Exception& e) {
		logInfo(e.what());
	} catch (std::exception& e) {
#ifdef WIN32
		wxMessageBox(e.what());
#endif
		logInfo("%s\n", e.what());
		logInfo("slam failed!\n");
#ifdef WIN32
		wxMessageBox(e.what());
#endif
	}

	logInfo("\nslam stopped!\n");
	return 0;
}



