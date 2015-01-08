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

int Param::nCam = 0;
int Param::nSkipFrame = 0;
int Param::nTotalFrame = 0;
int Param::nInitFrame = 100;

int Param::nMinFeatTrkLen = 20;
double Param::maxErr = 10.0;
double Param::maxDistRatio = 6.0;

int Param::nMaxMapPts = 800;
float Param::minMappedRatio = 0.08;
vector<string> Param::videoFilePath;
vector<string> Param::camFilePath;
//vector<int> Param::m_offset;

/* KLT parameters for tracking*/
float Param::minCornerness = 200; //for feature detection
int Param::nLevels = 6;
int Param::windowWidth = 6;
float Param::convergeThreshold = 1.0f;
float Param::SSD_Threshold = 8000; //for feature tracking
bool Param::trackWithGain = true;

double Const::MAX_EPI_ERR = 5.0;
double Const::PIXEL_ERR_VAR = 4.0; //2 pixels error

double Param::MAX_COST_KEY_FRM = 80; // threshold for key frame matching

int Param::frmNumAfterRelocalization = 50;
double Param::minRatioInitMapPts = 0.1;
