/*
 * CoSLAMThread.h
 *
 *  Created on: 2011-1-16
 *      Author: Danping Zou
 */
#ifndef COSLAMTHREAD_H_
#define COSLAMTHREAD_H_
#include "wx/wx.h"
#include "wx/event.h"
#include "wx/thread.h"
#include "pthread.h"

#ifdef WIN32
#include <io.h>
#include <process.h>
#else
#include <unistd.h>
#define Sleep(tm) {usleep(tm*1000);}
#endif

DECLARE_EVENT_TYPE(EventUpdateViews,-1)
;

class CoSLAMThread: public wxThread {
public:
	CoSLAMThread();
	~CoSLAMThread();
protected:
	virtual ExitCode Entry();
};

bool offlineMain();
bool offlineMainReloc();
bool usbCamMain();
bool ardroneMain();
bool ardroneROSMain();

bool ROSMain();
bool ROSMain_features();

void updateDisplayData();
void redrawAllViews();

#endif /* COSLAMTHREAD_H_ */
