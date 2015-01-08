/*
 * MainDialog.h
 *
 *  Created on: 2011-10-19
 *      Author: tsou
 */

#ifndef MAINDIALOG_H_
#define MAINDIALOG_H_

#include <wx/string.h>
#include <wx/stattext.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/filepicker.h>
#include <wx/sizer.h>
#include <wx/statline.h>
#include <wx/textctrl.h>
#include <wx/button.h>
#include <wx/dialog.h>
#include <wx/msgdlg.h>

#include "app/SL_GlobParam.h"
#include <ostream>
#include <istream>
using namespace std;

class MainDlg: public wxDialog {
private:

protected:
	wxStaticText* m_staticText21;
	wxFilePickerCtrl* m_filePickerVideo;
	wxStaticText* m_staticText22;
	wxFilePickerCtrl* m_filePickerCal;
	wxStaticText* m_staticText25;
	wxStaticLine* m_staticline6;
	wxStaticText* m_staticText26;
	wxTextCtrl* m_textStartFrm;
	wxStaticText* m_staticText27;
	wxTextCtrl* m_textInitFrm;
	wxStaticText* m_staticText28;
	wxTextCtrl* m_textKLTThres;
	wxStaticLine* m_staticline3;
	wxStaticText* m_staticText7;
	wxTextCtrl* m_textMinCornerness;
	wxButton* m_start;
	wxButton* m_exit;

	// Virtual event handlers, overide them in your derived class
	virtual void OnSelectVideo(wxFileDirPickerEvent& event) {
		wxString wxstr = m_filePickerVideo->GetPath();
		Param::videoFilePath.resize(1);
		Param::videoFilePath[0] = wxstr.ToStdString();
		event.Skip();
	}
	virtual void OnSelectCal(wxFileDirPickerEvent& event) {
		wxString wxstr = m_filePickerCal->GetPath();
		Param::camFilePath.resize(1);
		Param::camFilePath[0] = wxstr.ToStdString();
		event.Skip();
	}
	virtual void OnStart(wxCommandEvent& event) {
		if (Param::videoFilePath.empty()
				|| Param::videoFilePath[0] == "") {
			wxMessageBox("Please select video file!");
			return;
		}
		if (Param::camFilePath.empty()) {
			wxString wxstr = m_filePickerCal->GetPath();
			Param::camFilePath.resize(1);
			Param::camFilePath[0] = wxstr.ToStdString();
			if (Param::camFilePath[0] == "") {
				wxMessageBox("Please select calibration file!");
				return;
			}
			event.Skip();
		}
	}
	virtual void OnExit(wxCommandEvent& event) {
		event.Skip();
	}

public:

	MainDlg(wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title =
			wxT("MoSLAM by Danping Zou"), const wxPoint& pos =
			wxDefaultPosition, const wxSize& size = wxDefaultSize, long style =
			wxDEFAULT_DIALOG_STYLE);
	~MainDlg();

}
;

#endif /* MAINDIALOG_H_ */
