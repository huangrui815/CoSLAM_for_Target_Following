/*
 * MainDialog.cpp
 *
 *  Created on: 2011-10-19
 *      Author: tsou
 */

#include "MainDialog.h"

MainDlg::MainDlg(wxWindow* parent, wxWindowID id, const wxString& title,
		const wxPoint& pos, const wxSize& size, long style) :
		wxDialog(parent, id, title, pos, size, style) {
	this->SetSizeHints(wxSize(600, 300), wxDefaultSize);

	wxBoxSizer* bSizer20;
	bSizer20 = new wxBoxSizer(wxVERTICAL);

	wxFlexGridSizer* fgSizer8;
	fgSizer8 = new wxFlexGridSizer(2, 2, 0, 0);
	fgSizer8->AddGrowableCol(1);
	fgSizer8->SetFlexibleDirection(wxBOTH);
	fgSizer8->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

	m_staticText21 = new wxStaticText(this, wxID_ANY, wxT("Select a video:"),
			wxDefaultPosition, wxDefaultSize, 0);
	m_staticText21->Wrap(-1);
	fgSizer8->Add(m_staticText21, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	m_filePickerVideo = new wxFilePickerCtrl(this, wxID_ANY, wxEmptyString,
			wxT("Select a video (avi)"), wxT("*.avi"), wxDefaultPosition,
			wxDefaultSize, wxFLP_OPEN);
	
	fgSizer8->Add(m_filePickerVideo, 1, wxALL | wxEXPAND, 5);

	m_staticText22 = new wxStaticText(this, wxID_ANY, wxT("Calibration file:"),
			wxDefaultPosition, wxDefaultSize, 0);
	m_staticText22->Wrap(-1);
	fgSizer8->Add(m_staticText22, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	m_filePickerCal = new wxFilePickerCtrl(this, wxID_ANY,
			wxT("/home/tsou/Video/sonycx700E/sonycal.txt"),
			wxT("Select a calibration file"), wxT("*.txt"),
			wxDefaultPosition, wxDefaultSize, wxFLP_USE_TEXTCTRL);
	m_filePickerCal->SetPath(wxT("/home/tsou/Video/sonycx700E/sonycal.txt"));
	
	fgSizer8->Add(m_filePickerCal, 1, wxALL | wxEXPAND, 5);

	bSizer20->Add(fgSizer8, 0, wxEXPAND, 5);

	wxBoxSizer* bSizer23;
	bSizer23 = new wxBoxSizer(wxVERTICAL);

	wxBoxSizer* bSizer24;
	bSizer24 = new wxBoxSizer(wxVERTICAL);

	m_staticText25 = new wxStaticText(this, wxID_ANY, wxT("Parameters:"),
			wxDefaultPosition, wxDefaultSize, 0);
	m_staticText25->Wrap(-1);
	bSizer24->Add(m_staticText25, 0, wxALL, 5);

	wxBoxSizer* bSizer25;
	bSizer25 = new wxBoxSizer(wxVERTICAL);

	m_staticline6 = new wxStaticLine(this, wxID_ANY, wxDefaultPosition,
			wxDefaultSize, wxLI_HORIZONTAL);
	bSizer25->Add(m_staticline6, 0, wxEXPAND | wxALL, 5);

	wxFlexGridSizer* fgSizer10;
	fgSizer10 = new wxFlexGridSizer(4, 2, 0, 0);
	fgSizer10->AddGrowableCol(1);
	fgSizer10->SetFlexibleDirection(wxBOTH);
	fgSizer10->SetNonFlexibleGrowMode(wxFLEX_GROWMODE_SPECIFIED);

	fgSizer10->SetMinSize(wxSize(300, -1));
	m_staticText26 = new wxStaticText(this, wxID_ANY, wxT("Start frame:"),
			wxDefaultPosition, wxDefaultSize, 0);
	m_staticText26->Wrap(-1);
	fgSizer10->Add(m_staticText26, 0, wxALL, 5);

	m_textStartFrm = new wxTextCtrl(this, wxID_ANY, wxEmptyString,
			wxDefaultPosition, wxDefaultSize, 0);
	fgSizer10->Add(m_textStartFrm, 1, wxALL | wxEXPAND, 5);

	m_staticText27 = new wxStaticText(this, wxID_ANY, wxT("Initial frames:"),
			wxDefaultPosition, wxDefaultSize, 0);
	m_staticText27->Wrap(-1);
	fgSizer10->Add(m_staticText27, 0, wxALL, 5);

	m_textInitFrm = new wxTextCtrl(this, wxID_ANY, wxEmptyString,
			wxDefaultPosition, wxDefaultSize, 0);
	fgSizer10->Add(m_textInitFrm, 1, wxALL | wxEXPAND, 5);

	m_staticText28 = new wxStaticText(this, wxID_ANY, wxT("KLT threshold:"),
			wxDefaultPosition, wxDefaultSize, 0);
	m_staticText28->Wrap(-1);
	fgSizer10->Add(m_staticText28, 0, wxALL, 5);

	m_textKLTThres = new wxTextCtrl(this, wxID_ANY, wxEmptyString,
			wxDefaultPosition, wxDefaultSize, 0);
	fgSizer10->Add(m_textKLTThres, 0, wxALL | wxEXPAND, 5);
	m_staticText7 = new wxStaticText(this, wxID_ANY, wxT("Min cornerness:"),
			wxDefaultPosition, wxDefaultSize, 0);
	m_staticText7->Wrap(-1);
	fgSizer10->Add(m_staticText7, 0, wxALL, 5);

	m_textMinCornerness = new wxTextCtrl(this, wxID_ANY, wxEmptyString,
			wxDefaultPosition, wxDefaultSize, 0);
	fgSizer10->Add(m_textMinCornerness, 1, wxALL | wxEXPAND, 5);

	bSizer25->Add(fgSizer10, 0, wxALIGN_CENTER_HORIZONTAL, 5);

	bSizer24->Add(bSizer25, 1, wxEXPAND, 5);

	bSizer23->Add(bSizer24, 1, wxEXPAND, 5);

	bSizer20->Add(bSizer23, 1, wxEXPAND, 5);

	wxBoxSizer* bSizer6;
	bSizer6 = new wxBoxSizer(wxVERTICAL);

	bSizer6->SetMinSize(wxSize(-1, 60));
	m_staticline3 = new wxStaticLine(this, wxID_ANY, wxDefaultPosition,
			wxDefaultSize, wxLI_HORIZONTAL);
	bSizer6->Add(m_staticline3, 0, wxEXPAND | wxALL, 5);

	wxBoxSizer* bSizer26;
	bSizer26 = new wxBoxSizer(wxHORIZONTAL);

	m_start = new wxButton(this, wxID_OK, wxT("&Start"), wxDefaultPosition,
			wxDefaultSize, 0);
	bSizer26->Add(m_start, 0, wxALL, 5);

	m_exit = new wxButton(this, wxID_CANCEL, wxT("&Exit"), wxDefaultPosition,
			wxDefaultSize, 0);
	bSizer26->Add(m_exit, 0, wxALL, 5);

	bSizer6->Add(bSizer26, 0, wxALIGN_CENTER_HORIZONTAL, 5);
	bSizer20->Add(bSizer6, 0, wxEXPAND, 5);

	this->SetSizer(bSizer20);
	this->Layout();
	bSizer20->Fit(this);

	// Connect Events
	m_filePickerVideo->Connect(wxEVT_COMMAND_FILEPICKER_CHANGED,
			wxFileDirPickerEventHandler( MainDlg::OnSelectVideo ), NULL, this);
	m_filePickerCal->Connect(wxEVT_COMMAND_FILEPICKER_CHANGED,
			wxFileDirPickerEventHandler( MainDlg::OnSelectCal ), NULL, this);
	m_start->Connect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( MainDlg::OnStart ), NULL, this);
	m_exit->Connect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( MainDlg::OnExit ), NULL, this);

	//update the paramter value
	ostream outStartFrm(m_textStartFrm);
	outStartFrm << Param::nSkipFrame;
	ostream outInitFrm(m_textInitFrm);
	outInitFrm << Param::nInitFrame;
	ostream outKLTThres(m_textKLTThres);
	outKLTThres << Param::SSD_Threshold;
	ostream outMinCorner(m_textMinCornerness);
	outMinCorner << Param::minCornerness;

}

MainDlg::~MainDlg() {
	// Disconnect Events
	m_filePickerVideo->Disconnect(wxEVT_COMMAND_FILEPICKER_CHANGED,
			wxFileDirPickerEventHandler( MainDlg::OnSelectVideo ), NULL, this);
	m_filePickerCal->Disconnect(wxEVT_COMMAND_FILEPICKER_CHANGED,
			wxFileDirPickerEventHandler( MainDlg::OnSelectCal ), NULL, this);
	m_start->Disconnect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( MainDlg::OnStart ), NULL, this);
	m_exit->Disconnect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( MainDlg::OnExit ), NULL, this);
}
