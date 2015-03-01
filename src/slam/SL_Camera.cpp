/*
 * SL_Camera.cpp
 *
 *  Created on: 2010-11-21
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#include "SL_Camera.h"
#include "math/SL_Matrix.h"

CamPose::CamPose() :
		f(-1), camId(-1) {
	dynObjPresent = false;
	currDynPos[0] = 0;
	currDynPos[1] = 0;
	currDynPos[2] = 0;
}
CamPose::CamPose(const double* R_, const double* t_) :
		f(-1), camId(-1) {
	memcpy(R, R_, sizeof(double) * 9);
	memcpy(t, t_, sizeof(double) * 3);
	dynObjPresent = false;
}
CamPose::CamPose(const CamPose& other) {
	operator =(other);
}
CamPose& CamPose::operator =(const CamPose& other) {
	if (&other != this) {
		f = other.f;
		camId = other.camId;
		memcpy(R, other.R, sizeof(double) * 9);
		memcpy(t, other.t, sizeof(double) * 3);
		dynObjPresent = other.dynObjPresent;
	}
	return *this;
}

void CamPose::setDynPos(double* pos){
	currDynPos[0] = pos[0];
	currDynPos[1] = pos[1];
	currDynPos[2] = pos[2];
	dynObjPresent = true;
}

CamPoseItem::CamPoseItem() :
		CamPose(), pre(0), next(0) {
}
CamPoseItem::CamPoseItem(double* R_, double* t_) :
		CamPose(R_, t_), pre(0), next(0) {
}

CamPoseList::CamPoseList() :
		num(0), tail(0) {
}
CamPoseList::~CamPoseList() {
	clear();
}
void CamPoseList::clear() {
	CamPoseItem* p = over_head.next;
	while (p) {
		CamPoseItem* q = p;
		p = p->next;
		delete q;
		q = 0;
	}
	over_head.next = 0;
	tail = 0;
	num = 0;
}
CamPoseItem* CamPoseList::add(int f, double ts_, int camId, const double* R,
		const double* t) {
	CamPoseItem* cam = new CamPoseItem();
	cam->f = f;
	cam->ts = ts_;
	cam->camId = camId;
	memcpy(cam->R, R, sizeof(double) * 9);
	memcpy(cam->t, t, sizeof(double) * 3);

	if (tail == 0) {
		over_head.next = cam;
	} else {
		tail->next = cam;
		cam->pre = tail;
	}
	tail = cam;
	num++;
	return cam;
}
