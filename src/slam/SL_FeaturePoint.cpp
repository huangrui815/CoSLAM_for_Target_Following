/*
 * SL_ImgPoint.cpp
 *
 *  Created on: 2010-11-21
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#include "SL_FeaturePoint.h"
#include "SL_MapPoint.h"
#include "SL_error.h"
#include <cstddef>
#include <stdint.h>

FeaturePoint::FeaturePoint() :
		xo(0),yo(0),x(0), y(0), id(0), f(-1), camId(-1), type(0), reprojErr(0), mpt(0), mpt_id(
				-1), pre(0), next(0), cam(0), preFrame(0), nextFrame(0), bKeyFrm(
				false) {
	id = (longInt) this;

}
FeaturePoint::FeaturePoint(int f1, int id, double x1, double y1) :
		xo(x1),yo(y1),x(x1), y(y1),id(0), f(f1), camId(id), type(0), reprojErr(0), mpt(0), mpt_id(
				-1), pre(0), next(0), cam(0), preFrame(0), nextFrame(0), bKeyFrm(
				false) {
	id = (longInt) this;
}

FeaturePoint::FeaturePoint(int f1, int id, double x1, double y1, double ud_x1, double ud_y1) :
		xo(x1),yo(y1),x(ud_x1), y(ud_y1),id(0), f(f1), camId(id), type(0), reprojErr(0), mpt(0), mpt_id(
				-1), pre(0), next(0), cam(0), preFrame(0), nextFrame(0), bKeyFrm(
				false) {
	id = (longInt) this;
}

FeaturePoint::FeaturePoint(const FeaturePoint& other) {
	operator=(other);
}
FeaturePoint& FeaturePoint::operator=(const FeaturePoint& other) {
	if (&other != this) {
		id = other.id;
		f = other.f;
		camId = other.camId;
		xo = other.xo;
		yo = other.yo;
		x = other.x;
		y = other.y;
		type = other.type;
		reprojErr = other.reprojErr;
		mpt = other.mpt;
		mpt_id = other.mpt_id;
		pre = other.pre;
		next = other.next;
		cam = other.cam;
		preFrame = other.preFrame;
		nextFrame = other.nextFrame;
		bKeyFrm = other.bKeyFrm;
	}
	return *this;
}
FeaturePoint::~FeaturePoint() {
}


