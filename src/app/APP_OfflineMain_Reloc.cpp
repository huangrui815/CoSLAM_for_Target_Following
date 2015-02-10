#include "gui/CoSLAMThread.h"
#include "app/SL_GlobParam.h"
#include "app/SL_CoSLAM.h"
#include "tools/SL_Tictoc.h"
#include "tools/SL_Timing.h"

#include "gui/MyApp.h"

bool offlineMainReloc() {
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
		coSLAM.pause();

		int endFrame = SLAMParam::nTotalFrame - SLAMParam::nSkipFrame
				- SLAMParam::nInitFrame - 10;

		endFrame = 2000;

		// init estimation flag
		bool bEstPose[SLAM_MAX_NUM];
		for (int i = 0; i < SLAM_MAX_NUM; i++){
			bEstPose[i] = false;
		}

		for (int i = 0; i < endFrame; i++) {
			while (MyApp::bStop) {/*stop*/
			}
			TimeMeasurer tmPerStep;
			tmPerStep.tic();

			// for debug
			if( !coSLAM.curMapPts.getHead())
				printf(" !coSLAM.curMapPts.getHead() \n");

			std::cout << "debug1: " <<
					coSLAM.curMapPts.getHead()->x << " " << coSLAM.curMapPts.getNum()
					<< endl;

			coSLAM.grabReadFrame();
			coSLAM.featureTracking();
			coSLAM.poseUpdate(bEstPose);

			std::cout << "debug2: " <<
					coSLAM.curMapPts.getHead()->x << " " << coSLAM.curMapPts.getNum()
					<< endl;

			for (int i = 0; i < coSLAM.numCams; i++){
				if (coSLAM.state[i] == SLAM_STATE_RELOCATE)
					if (coSLAM.doRelocalization(i))
						coSLAM.endRelocalization(i);
			}

			std::cout << "debug3: " <<
					coSLAM.curMapPts.getHead()->x << " " << coSLAM.curMapPts.getNum()
					<< endl;

			//coSLAM.pause();
			coSLAM.cameraGrouping();
			//existing 3D to 2D points robust
			coSLAM.activeMapPointsRegister(Const::PIXEL_ERR_VAR);


			std::cout << "debug4: " <<
					coSLAM.curMapPts.getHead()->x << " " << coSLAM.curMapPts.getNum()
					<< endl;

			TimeMeasurer tmNewMapPoints;
			tmNewMapPoints.tic();

			bool merge = false;
			coSLAM.genNewMapPoints(merge);
			coSLAM.m_tmNewMapPoints = tmNewMapPoints.toc();

			std::cout << "debug5: " <<
					coSLAM.curMapPts.getHead()->x << " " << coSLAM.curMapPts.getNum()
					<< endl;

			for (int i = 0; i < coSLAM.numCams; i++){
				if(coSLAM.state[i] == SLAM_STATE_NORMAL)
					coSLAM.m_relocalizer[i].cacheNewKeyPose(coSLAM.slam[i]);
			}

			//point registration
			coSLAM.currentMapPointsRegister(Const::PIXEL_ERR_VAR,
					i % 50 == 0 ? true : false);

			std::cout << "debug6: " <<
					coSLAM.curMapPts.getHead()->x << " " << coSLAM.curMapPts.getNum()
					<< endl;

			coSLAM.storeDynamicPoints();

			std::cout << "debug7: " <<
					coSLAM.curMapPts.getHead()->x << " " << coSLAM.curMapPts.getNum()
					<< endl;

			updateDisplayData();
			redrawAllViews();

			coSLAM.m_tmPerStep = tmPerStep.toc();
			Sleep(50);

			if (i % 500 == 0) {
				//coSLAM.releaseFeatPts(coSLAM.curFrame - 500);
				coSLAM.releaseKeyPoseImgs(coSLAM.curFrame - 500);
				coSLAM.m_lastReleaseFrm = coSLAM.curFrame;
			}

			std::cout << "debug8: " <<
					coSLAM.curMapPts.getHead()->x << " " << coSLAM.curMapPts.getNum()
					<< endl;

			for (int i = 0; i < coSLAM.numCams; i++)
				if ( coSLAM.state[i] == SLAM_STATE_RECOVERED)
					coSLAM.state[i] = SLAM_STATE_NORMAL;
		}
		cout << " the result is saved at " << MyApp::timeStr << endl;
		coSLAM.exportResults(MyApp::timeStr);
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
