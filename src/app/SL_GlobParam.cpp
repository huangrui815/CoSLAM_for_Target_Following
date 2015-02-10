/*
 * SL_GlobParam.cpp
 *
 *  Created on: 2011-7-26
 *      Author: zou
 */

#include "SL_GlobParam.h"
#include "SL_error.h"
#include <fstream>
#include <iostream>

int SLAMParam::nCam = 0;
int SLAMParam::nSkipFrame = 0;
int SLAMParam::nTotalFrame = 0;
int SLAMParam::nInitFrame = 100;

int SLAMParam::nMinFeatTrkLen = 20;
double SLAMParam::maxErr = 10.0;
double SLAMParam::maxDistRatio = 6.0;

int SLAMParam::nMaxMapPts = 800;
float SLAMParam::minMappedRatio = 0.08;
vector<string> SLAMParam::videoFilePath;
vector<string> SLAMParam::camFilePath;
//vector<int> Param::m_offset;

/* KLT parameters for tracking*/
float SLAMParam::minCornerness = 200; //for feature detection
int SLAMParam::nLevels = 6;
int SLAMParam::windowWidth = 6;
float SLAMParam::convergeThreshold = 1.0f;
float SLAMParam::SSD_Threshold = 8000; //for feature tracking
bool SLAMParam::trackWithGain = true;

double Const::MAX_EPI_ERR = 5.0;
double Const::PIXEL_ERR_VAR = 4.0; //2 pixels error

double SLAMParam::MAX_COST_KEY_FRM = 80; // threshold for key frame matching

int SLAMParam::frmNumAfterRelocalization = 50;
double SLAMParam::minRatioInitMapPts = 0.1;
