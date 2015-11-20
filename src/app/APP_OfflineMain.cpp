#include "gui/CoSLAMThread.h"
#include "app/SL_GlobParam.h"
#include "app/SL_CoSLAM.h"
#include "tools/SL_Tictoc.h"
#include "tools/SL_Timing.h"

#include "gui/MyApp.h"

bool offlineMain() {
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
		for(int c = 0; c < coSLAM.numCams; c++){
			coSLAM.slam[c].videoReader = &MyApp::aviReader[c];
		}

		coSLAM.init(true);
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
		tic();
		coSLAM.initMap();
		toc();

		for (int i = 0; i < coSLAM.numCams; i++) {
			printf("slam[%d].m_camPos.size(): %d\n", i, coSLAM.slam[i].m_camPos.size());
			coSLAM.state[i] = SLAM_STATE_NORMAL;
		}
		updateDisplayData();
		redrawAllViews();
//
//		coSLAM.pause();
//		return 0;

		int endFrame = SLAMParam::nTotalFrame - SLAMParam::nSkipFrame
				- SLAMParam::nInitFrame - 10;

		cout << "Total frame: " << SLAMParam::nTotalFrame << " skip frame: " << SLAMParam::nSkipFrame <<
				"init frame: " << SLAMParam::nInitFrame <<endl;
//		int endFrame = 500;

//		endFrame = 1500;

		// init estimation flag
		bool bEstPose[SLAM_MAX_NUM];
		for (int i = 0; i < SLAM_MAX_NUM; i++){
			bEstPose[i] = false;
		}

		vector<double> tmStepVec;
		MyApp::bStop = true;
		for (int i = 0; i < endFrame && !MyApp::bExit; i++) {

//				redrawAllViews();
//			MyApp::bStop = true;
				while (MyApp::bStop) {
					Sleep(50);
				};
			TimeMeasurer tmPerStep;
			tmPerStep.tic();

			coSLAM.grabReadFrame();
			for (int i = 0; i < coSLAM.numCams; i++){
			cv::Mat cvImg(coSLAM.slam[i].m_img.rows, coSLAM.slam[i].m_img.cols, CV_8UC1,
								coSLAM.slam[i].m_img.data);
			MyApp::s_camFrames[i].push_back(cvImg.clone());
			}

			coSLAM.featureTracking();
			coSLAM.poseUpdate(bEstPose);

			//coSLAM.pause();
			coSLAM.cameraGrouping();
			//existing 3D to 2D points robust
			coSLAM.activeMapPointsRegister(Const::PIXEL_ERR_VAR);

			TimeMeasurer tmNewMapPoints;
			tmNewMapPoints.tic();

			bool merge = false;
			coSLAM.genNewMapPoints(merge);
			coSLAM.m_tmNewMapPoints = tmNewMapPoints.toc();
			//cout << "coSLAM.m_tmNewMapPoints" << coSLAM.m_tmNewMapPoints << endl;

			//point registration
			coSLAM.currentMapPointsRegister(Const::PIXEL_ERR_VAR,
					i % 50 == 0 ? true : false);

			coSLAM.storeDynamicPoints();

			if (coSLAM.dynObjPresent){
				for (int i=0; i < coSLAM.numCams; i++){
					coSLAM.slam[i].m_camPos.current()->setDynPos(coSLAM.dynObjPos);
//					printf("dynObjPos: %lf %lf %lf\n", coSLAM.dynObjPos[0], coSLAM.dynObjPos[1], coSLAM.dynObjPos[2]);
					double targetPos[3];
					coSLAM.transformTargetPos2Global(coSLAM.dynObjPos, targetPos);
//					printf("targetPos: %lf %lf %lf\n", targetPos[0], targetPos[1], targetPos[2]);
					double theta = atan2(coSLAM.slam[i]._targetPosInCam[0], coSLAM.slam[i]._targetPosInCam[2]);
					double H = targetPos[2] * 2;
					double Z = coSLAM.slam[i]._targetPosInCam[2];
//					printf("cam: %d, theta: %lf, H: %lf, Z: %lf\n", i, theta * 180 / 3.1415926, H, Z);
				}
			}

			updateDisplayData();
			redrawAllViews();

			coSLAM.m_tmPerStep = tmPerStep.toc();
			tmStepVec.push_back(coSLAM.m_tmPerStep);
			Sleep(3);

			if (i % 500 == 0) {
				//coSLAM.releaseFeatPts(coSLAM.curFrame - 500);
				coSLAM.releaseKeyPoseImgs(coSLAM.curFrame - 500);
				coSLAM.m_lastReleaseFrm = coSLAM.curFrame;
			}

			if (merge){
				coSLAM.pause();
			}
		}
		cout << " the result is saved at " << MyApp::timeStr << endl;
		coSLAM.exportResults(MyApp::timeStr);

		FILE* fid = fopen("slam_timing.txt","w");
		for (int i = 0; i < tmStepVec.size(); i++)
			fprintf(fid, "%f\n", tmStepVec[i]);
		fclose(fid);

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
