/*
 * SL_GlobParam.h
 *
 *  Created on: 2011-7-26
 *      Author: zou
 */

#ifndef SL_GLOBPARAM_H_
#define SL_GLOBPARAM_H_
#include <string>
#include <vector>
using namespace std;
//#define DEBUG_MODE 0
class SLAMParam {
public:
	/* number of cameras*/
	static int nCam;
	/* number of frames to skip from the first frame*/
	static int nSkipFrame;
	/* total number of frames*/
	static int nTotalFrame;
	/* number of frames used for initialization for single camera SLAM*/
	static int nInitFrame;
	/*minimum feature track length for tri-angulation*/
	static int nMinFeatTrkLen;
	/*maximum reprojection error*/
	static double maxErr;
	/*maximum camera distance between cameras in the same group*/
	static double maxDistRatio;
	/*maximum map points in each frame*/
	static int nMaxMapPts;
	/*minimum mapped feature points percentage
	 * that we consider the tracking is normal
	 */
	static float minMappedRatio;
	/*maximum cost for seeking the similar key frame in re-localization*/
	static double MAX_COST_KEY_FRM;

	static int frmNumAfterRelocalization;

	// Ratio of map points reconstructed for successful initilization
	static double minRatioInitMapPts;

	
	static vector<string> videoFilePath;
	static vector<string> camFilePath;
	
	/* KLT parameters for tracking*/
	static float minCornerness; //*feature detection
	static int nLevels;
	static int windowWidth;
	static float convergeThreshold;
	static float SSD_Threshold; //*tracking
	static bool trackWithGain;
};

class Const {
public:
	static double MAX_EPI_ERR;
	static double PIXEL_ERR_VAR;
};
#endif /* SL_GLOBPARAM_H_ */
