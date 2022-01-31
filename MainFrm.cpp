// This MFC Samples source code demonstrates using MFC Microsoft Office Fluent User Interface 
// (the "Fluent UI") and is provided only as referential material to supplement the 
// Microsoft Foundation Classes Reference and related electronic documentation 
// included with the MFC C++ library software.  
// License terms to copy, use or distribute the Fluent UI are available separately.  
// To learn more about our Fluent UI licensing program, please visit 
// http://msdn.microsoft.com/officeui.
//
// Copyright (C) Microsoft Corporation
// All rights reserved.

// MainFrm.cpp : implementation of the CMainFrame class
//

#include "stdafx.h"

#include "FlowFieldVisWithLeapMotion.h"

#include "MainFrm.h"

#include "FlowFieldVisWithLeapMotionView.h"

#include "FieldLine.h"
#include "LeapC.h"
#include "LeapMotionCallback.h"

#include <string>

using namespace std;

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

extern SampleListener listener;
extern LeapMotionConnection::Callbacks ConnectionCallbacks;

// CMainFrame
IMPLEMENT_DYNCREATE(CMainFrame, CFrameWndEx)

BEGIN_MESSAGE_MAP(CMainFrame, CFrameWndEx)
	ON_WM_CREATE()
	ON_COMMAND_RANGE(ID_VIEW_APPLOOK_WIN_2000, ID_VIEW_APPLOOK_OFF_2007_AQUA, &CMainFrame::OnApplicationLook)
	ON_REGISTERED_MESSAGE(AFX_WM_ON_CHANGE_RIBBON_CATEGORY, OnChangeRibbonCategory)
END_MESSAGE_MAP()

// CMainFrame construction/destruction
CMainFrame::CMainFrame()
{
	// TODO: add member initialization code here
	theApp.m_nAppLook = theApp.GetInt(_T("ApplicationLook"), ID_VIEW_APPLOOK_OFF_2007_BLUE);
}

CMainFrame::~CMainFrame()
{
}

int CMainFrame::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CFrameWndEx::OnCreate(lpCreateStruct) == -1)
		return -1;

	BOOL bNameValid;
	// set the visual manager and style based on persisted value
	OnApplicationLook(theApp.m_nAppLook);

	m_wndRibbonBar.Create(this);
	InitializeRibbon();

	// enable Visual Studio 2005 style docking window behavior
	CDockingManager::SetDockingMode(DT_SMART);
	// enable Visual Studio 2005 style docking window auto-hide behavior
	EnableAutoHidePanes(CBRS_ALIGN_ANY);

	// Outlook bar is created and docking on the left side should be allowed.
	EnableDocking(CBRS_ALIGN_LEFT);
	EnableAutoHidePanes(CBRS_ALIGN_RIGHT);

	this->SetTitle(_T("Flow Field Visualization with Leap Motion"));

	return 0;
}

BOOL CMainFrame::PreCreateWindow(CREATESTRUCT& cs)
{
	if( !CFrameWndEx::PreCreateWindow(cs) )
		return FALSE;
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return TRUE;
}

void CMainFrame::InitializeRibbon()
{
	BOOL bNameValid;

	// Add "Query" category:
	CMFCRibbonCategory *pCategoryQuery = m_wndRibbonBar.AddCategory(_T("Query Tools"), IDB_FILESMALL, IDB_FILELARGE);

	// ADD "Query by LM" panel:
	CMFCRibbonPanel *pQueryByLM = pCategoryQuery->AddPanel(_T(""));
	pQueryByLM->Add(new CMFCRibbonButton(ID_QUERY_BY_LM, _T("Query by LM"), 8, 8));

	// ADD "Query by Sketch" panel:
	CMFCRibbonPanel *pQueryBySketch = pCategoryQuery->AddPanel(_T(""));
	pQueryBySketch->Add(new CMFCRibbonButton(ID_QUERY_BY_SKETCH, _T("Query by Sketch"), 8, 8));

	// ADD "Query by 3D File" panel:
	CMFCRibbonPanel *pQueryBy3DFile = pCategoryQuery->AddPanel(_T(""));
	pQueryBy3DFile->Add(new CMFCRibbonButton(ID_QUERY_BY_3DFILE, _T("Query by 3DFile"), 1, 1));

	// ADD "Query by 2D File" panel:
	CMFCRibbonPanel *pQueryBy2DFile = pCategoryQuery->AddPanel(_T(""));
	pQueryBy2DFile->Add(new CMFCRibbonButton(ID_QUERY_BY_2DFILE, _T("Query by 2DFile"), 1, 1));

	// ADD "Clear LM Query Panel" panel:
	CMFCRibbonPanel *pClearLMQueryPanel = pCategoryQuery->AddPanel(_T(""));
	pClearLMQueryPanel->Add(new CMFCRibbonButton(ID_CLEAR_LM_QUERY_PANEL, _T("Clear LM Query Panel"), 4, 4));

	// ADD "Clear Sketch Query Panel" panel:
	CMFCRibbonPanel *pClearSketchQueryPanel = pCategoryQuery->AddPanel(_T(""));
	pClearSketchQueryPanel->Add(new CMFCRibbonButton(ID_CLEAR_SKETCH_QUERY_PANEL, _T("Clear Sketch Query Panel"), 4, 4));

	// ADD "Save 3D LM Curve" panel:
	CMFCRibbonPanel *pSave3DCurve = pCategoryQuery->AddPanel(_T(""));
	pSave3DCurve->Add(new CMFCRibbonButton(ID_SAVE_3D_CURVE, _T("Save 3D Curve"), 2, 2));

	// ADD "Save 2D Sketch Curve" panel:
	CMFCRibbonPanel *pSave2DCurve = pCategoryQuery->AddPanel(_T(""));
	pSave2DCurve->Add(new CMFCRibbonButton(ID_SAVE_2D_CURVE, _T("Save 2D Curve"), 2, 2));

	// Add "Set Similarity Threshold" panel:
	CMFCRibbonPanel *pSetSimilarityThreshold = pCategoryQuery->AddPanel(_T("3D Angle Similarity Threshold"), 
		m_PanelImages.ExtractIcon(0));
	CMFCRibbonSlider *pSimilarityThresholdSlider = new CMFCRibbonSlider(ID_SIMILILARITY_THRESHOLD1, 100);
	pSimilarityThresholdSlider->SetZoomButtons();
	pSimilarityThresholdSlider->SetRange(0, 20);
	pSimilarityThresholdSlider->SetPos(10);
	pSimilarityThresholdSlider->SetZoomIncrement(1);
	pSetSimilarityThreshold->Add(pSimilarityThresholdSlider);
	CMFCRibbonEdit *pSimilarityThresholdEdit = new CMFCRibbonEdit(ID_SIMILILARITY_THRESHOLD2, 140, _T(""));
	pSimilarityThresholdEdit->SetEditText(_T("0.5"));
	pSetSimilarityThreshold->Add(pSimilarityThresholdEdit);

	// Add "Set CSimilarity Threshold" panel:
	CMFCRibbonPanel *pSetCurSimilarityThreshold = pCategoryQuery->AddPanel(_T("3D Curvature Similarity Threshold"), m_PanelImages.ExtractIcon(0));
	CMFCRibbonSlider *pCurSimilarityThresholdSlider = new CMFCRibbonSlider(ID_CURSIMILILARITY_THRESHOLD1, 100);
	pCurSimilarityThresholdSlider->SetZoomButtons();
	pCurSimilarityThresholdSlider->SetRange(0, 20);
	pCurSimilarityThresholdSlider->SetPos(1);
	pCurSimilarityThresholdSlider->SetZoomIncrement(1);
	pSetCurSimilarityThreshold->Add(pCurSimilarityThresholdSlider);
	CMFCRibbonEdit *pCurSimilarityThresholdEdit = new CMFCRibbonEdit(ID_CURSIMILILARITY_THRESHOLD2, 140, _T(""));
	pCurSimilarityThresholdEdit->SetEditText(_T("0.1"));
	pSetCurSimilarityThreshold->Add(pCurSimilarityThresholdEdit);

	// Add "Set Similarity Threshold" panel:
	CMFCRibbonPanel *pSet2DSimilarityThreshold = pCategoryQuery->AddPanel(_T("2D Angle Similarity Threshold"), m_PanelImages.ExtractIcon(0));
	CMFCRibbonSlider *p2DSimilarityThresholdSlider = new CMFCRibbonSlider(ID_2DSIMILILARITY_THRESHOLD1, 100);
	p2DSimilarityThresholdSlider->SetZoomButtons();
	p2DSimilarityThresholdSlider->SetRange(0, 50);
	p2DSimilarityThresholdSlider->SetPos(5);
	p2DSimilarityThresholdSlider->SetZoomIncrement(1);
	pSet2DSimilarityThreshold->Add(p2DSimilarityThresholdSlider);
	CMFCRibbonEdit *p2DSimilarityThresholdEdit = new CMFCRibbonEdit(ID_2DSIMILILARITY_THRESHOLD2, 140, _T(""));
	p2DSimilarityThresholdEdit->SetEditText(_T("0.1"));
	pSet2DSimilarityThreshold->Add(p2DSimilarityThresholdEdit);

	// Add "Set CSimilarity Threshold" panel:
	CMFCRibbonPanel *pSet2DCurSimilarityThreshold = pCategoryQuery->AddPanel(_T("2D Curvature Similarity Threshold"), m_PanelImages.ExtractIcon(0));
	CMFCRibbonSlider *p2DCurSimilarityThresholdSlider = new CMFCRibbonSlider(ID_2DCURSIMILILARITY_THRESHOLD1, 100);
	p2DCurSimilarityThresholdSlider->SetZoomButtons();
	p2DCurSimilarityThresholdSlider->SetRange(0, 50);
	p2DCurSimilarityThresholdSlider->SetPos(5);
	p2DCurSimilarityThresholdSlider->SetZoomIncrement(1);
	pSet2DCurSimilarityThreshold->Add(p2DCurSimilarityThresholdSlider);
	CMFCRibbonEdit *p2DCurSimilarityThresholdEdit = new CMFCRibbonEdit(ID_2DCURSIMILILARITY_THRESHOLD2, 140, _T(""));
	p2DCurSimilarityThresholdEdit->SetEditText(_T("0.1"));
	pSet2DCurSimilarityThreshold->Add(p2DCurSimilarityThresholdEdit);

	// Add "Change the Granularity of Bin" panel:
	CMFCRibbonPanel *pChangeTheGranularityOfBin = pCategoryQuery->AddPanel(_T("Granularity of Angle Bin"), m_PanelImages.ExtractIcon(0));
	CMFCRibbonSlider *pGranularityOfBinSlider = new CMFCRibbonSlider(ID_GRANULARITY_OF_BIN_SLIDER, 100);
	pGranularityOfBinSlider->SetZoomButtons();
	pGranularityOfBinSlider->SetRange(5, 20);
	pGranularityOfBinSlider->SetPos(10);
	pGranularityOfBinSlider->SetZoomIncrement(5);
	pChangeTheGranularityOfBin->Add(pGranularityOfBinSlider);
	CMFCRibbonEdit *pGranularityOfBinEdit = new CMFCRibbonEdit(ID_GRANULARITY_OF_BIN_EDITOR, 140, _T(""));
	pGranularityOfBinEdit->SetEditText(_T("10"));
	pChangeTheGranularityOfBin->Add(pGranularityOfBinEdit);

	// Add "Change the Granularity of Bin" panel:
	CMFCRibbonPanel *pChangeTheGranularityOfCurBin = pCategoryQuery->AddPanel(_T("Granularity of Curvature Bin"), m_PanelImages.ExtractIcon(0));
	CMFCRibbonSlider *pGranularityOfCurBinSlider = new CMFCRibbonSlider(ID_GRANULARITY_OF_CUR_BIN_SLIDER, 100);
	pGranularityOfCurBinSlider->SetZoomButtons();
	pGranularityOfCurBinSlider->SetRange(1, 10);
	pGranularityOfCurBinSlider->SetPos(5);
	pGranularityOfCurBinSlider->SetZoomIncrement(1);
	pChangeTheGranularityOfCurBin->Add(pGranularityOfCurBinSlider);
	CMFCRibbonEdit *pGranularityOfCurBinEdit = new CMFCRibbonEdit(ID_GRANULARITY_OF_CUR_BIN_EDITOR, 140, _T(""));
	pGranularityOfCurBinEdit->SetEditText(_T("5"));
	pChangeTheGranularityOfCurBin->Add(pGranularityOfCurBinEdit);

	// ADD "Open Config (for Query)" panel:
	CMFCRibbonPanel *pOpenConfigQ = pCategoryQuery->AddPanel(_T(""));
	pOpenConfigQ->Add(new CMFCRibbonButton(ID_OPEN_CONFIG_Q, _T("Open Config"), 8, 8));
}

BOOL CMainFrame::CreateOutlookBar(CMFCOutlookBar& bar, UINT uiID, CMFCShellTreeCtrl& tree, CCalendarBar& calendar, int nInitialWidth)
{
	CWindowDC dc(NULL);

	bar.SetMode2003();

	BOOL bNameValid;
	CString strTemp;
	bNameValid = strTemp.LoadString(IDS_SHORTCUTS);
	ASSERT(bNameValid);
	if (!bar.Create(strTemp, this, CRect(0, 0, nInitialWidth, 32000), uiID, WS_CHILD | WS_VISIBLE | CBRS_LEFT))
	{
		return FALSE; // fail to create
	}

	CMFCOutlookBarTabCtrl* pOutlookBar = (CMFCOutlookBarTabCtrl*)bar.GetUnderlyingWindow();

	if (pOutlookBar == NULL)
	{
		ASSERT(FALSE);
		return FALSE;
	}

	pOutlookBar->EnableInPlaceEdit(TRUE);

	static UINT uiPageID = 1;

	DWORD dwPaneStyle = AFX_DEFAULT_TOOLBAR_STYLE | CBRS_FLOAT_MULTI;

	// can float, can autohide, can resize, CAN NOT CLOSE
	DWORD dwStyle = AFX_CBRS_FLOAT | AFX_CBRS_AUTOHIDE | AFX_CBRS_RESIZE;

	CRect rectDummy(0, 0, 0, 0);
	const DWORD dwTreeStyle = WS_CHILD | WS_VISIBLE | TVS_HASLINES | TVS_LINESATROOT | TVS_HASBUTTONS;

	tree.Create(dwTreeStyle, rectDummy, &bar, 1200);
	bNameValid = strTemp.LoadString(IDS_FOLDERS);
	ASSERT(bNameValid);
	pOutlookBar->AddControl(&tree, strTemp, 2, TRUE, dwStyle);

	calendar.Create(rectDummy, &bar, 1201);
	bNameValid = strTemp.LoadString(IDS_CALENDAR);
	ASSERT(bNameValid);
	pOutlookBar->AddControl(&calendar, strTemp, 3, TRUE, dwStyle);

	bar.SetPaneStyle(bar.GetPaneStyle() | CBRS_TOOLTIPS | CBRS_FLYBY | CBRS_SIZE_DYNAMIC);

	pOutlookBar->SetImageList(theApp.m_bHiColorIcons ? IDB_PAGES_HC : IDB_PAGES, 24);
	pOutlookBar->SetToolbarImageList(theApp.m_bHiColorIcons ? IDB_PAGES_SMALL_HC : IDB_PAGES_SMALL, 16);
	pOutlookBar->RecalcLayout();

	BOOL bAnimation = theApp.GetInt(_T("OutlookAnimation"), TRUE);
	CMFCOutlookBarTabCtrl::EnableAnimation(bAnimation);

	bar.SetButtonsFont(&afxGlobalData.fontBold);

	return TRUE;
}

// CMainFrame diagnostics

#ifdef _DEBUG
void CMainFrame::AssertValid() const
{
	CFrameWndEx::AssertValid();
}

void CMainFrame::Dump(CDumpContext& dc) const
{
	CFrameWndEx::Dump(dc);
}
#endif //_DEBUG


// CMainFrame message handlers

void CMainFrame::OnApplicationLook(UINT id)
{
	CWaitCursor wait;

	theApp.m_nAppLook = id;

	switch (theApp.m_nAppLook)
	{
	case ID_VIEW_APPLOOK_WIN_2000:
		CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManager));
		break;

	case ID_VIEW_APPLOOK_OFF_XP:
		CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerOfficeXP));
		break;

	case ID_VIEW_APPLOOK_WIN_XP:
		CMFCVisualManagerWindows::m_b3DTabsXPTheme = TRUE;
		CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerWindows));
		break;

	case ID_VIEW_APPLOOK_OFF_2003:
		CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerOffice2003));
		CDockingManager::SetDockingMode(DT_SMART);
		break;

	case ID_VIEW_APPLOOK_VS_2005:
		CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerVS2005));
		CDockingManager::SetDockingMode(DT_SMART);
		break;

	default:
		switch (theApp.m_nAppLook)
		{
		case ID_VIEW_APPLOOK_OFF_2007_BLUE:
			CMFCVisualManagerOffice2007::SetStyle(CMFCVisualManagerOffice2007::Office2007_LunaBlue);
			break;

		case ID_VIEW_APPLOOK_OFF_2007_BLACK:
			CMFCVisualManagerOffice2007::SetStyle(CMFCVisualManagerOffice2007::Office2007_ObsidianBlack);
			break;

		case ID_VIEW_APPLOOK_OFF_2007_SILVER:
			CMFCVisualManagerOffice2007::SetStyle(CMFCVisualManagerOffice2007::Office2007_Silver);
			break;

		case ID_VIEW_APPLOOK_OFF_2007_AQUA:
			CMFCVisualManagerOffice2007::SetStyle(CMFCVisualManagerOffice2007::Office2007_Aqua);
			break;
		}

		CMFCVisualManager::SetDefaultManager(RUNTIME_CLASS(CMFCVisualManagerOffice2007));
		CDockingManager::SetDockingMode(DT_SMART);
	}

	RedrawWindow(NULL, NULL, RDW_ALLCHILDREN | RDW_INVALIDATE | RDW_UPDATENOW | RDW_FRAME | RDW_ERASE);

	theApp.WriteInt(_T("ApplicationLook"), theApp.m_nAppLook);
}

float CMainFrame::GetSliderPos(UINT nID)
{
	CMFCRibbonSlider *pSlider = (CMFCRibbonSlider *)m_wndRibbonBar.FindByID(nID);
	int pos = pSlider->GetPos();
	int max = pSlider->GetRangeMax();
	int min = pSlider->GetRangeMin();
	float res = static_cast<float>(pos) / static_cast<float>(max - min);
	return res;
}

int CMainFrame::GetSliderPosInt(UINT nID)
{
	CMFCRibbonSlider *pSlider = (CMFCRibbonSlider *)m_wndRibbonBar.FindByID(nID);
	int pos = pSlider->GetPos();
	return pos;
}

int CMainFrame::GetEditValue(UINT nID)
{
	CMFCRibbonEdit *pEdit = (CMFCRibbonEdit *)m_wndRibbonBar.FindByID(nID);
	CString str = pEdit->GetEditText();
	int value = _ttoi(str);
	return value;
}

void CMainFrame::SetSliderPos(UINT nID, const float v)
{
	CMFCRibbonSlider *pSlider = (CMFCRibbonSlider *)m_wndRibbonBar.FindByID(nID);
	int max = pSlider->GetRangeMax();
	int min = pSlider->GetRangeMin();
	int pos = static_cast<int>(v * (max - min));
	pSlider->SetPos(pos);
}

void CMainFrame::SetSliderPosInt(UINT nID, const int v)
{
	CMFCRibbonSlider *pSlider = (CMFCRibbonSlider *)m_wndRibbonBar.FindByID(nID);
	pSlider->SetPos(v);
}

void CMainFrame::SetEditValue(UINT nID, const int v)
{
	CMFCRibbonEdit *pEdit = (CMFCRibbonEdit *)m_wndRibbonBar.FindByID(nID);
	CString str;
	str.Format(_T("%d"), v);
	pEdit->SetEditText(str);
}

void CMainFrame::SetPane1Text(CString str)
{
	CMFCRibbonStatusBarPane *pPane = (CMFCRibbonStatusBarPane *)m_wndStatusBar.FindByID(ID_STATUSBAR_PANE1);
	pPane->SetText(str);
	pPane->Redraw();
}

void CMainFrame::SetPane2Text(CString str)
{
	CMFCRibbonStatusBarPane *pPane = (CMFCRibbonStatusBarPane *)m_wndStatusBar.FindByID(ID_STATUSBAR_PANE2);
	pPane->SetText(str);
	pPane->Redraw();
}

void CMainFrame::SetEditText(UINT nID, CString str)
{
	CMFCRibbonEdit *pEdit = (CMFCRibbonEdit *)m_wndRibbonBar.FindByID(nID);
	pEdit->SetEditText(str);
	pEdit->Redraw();
}

CString CMainFrame::GetEditText(UINT nID)
{
	CMFCRibbonEdit *pEdit = (CMFCRibbonEdit *)m_wndRibbonBar.FindByID(nID);
	CString str = pEdit->GetEditText();
	pEdit->Redraw();
	return str;
}

BOOL CMainFrame::OnCreateClient(LPCREATESTRUCT lpcs, CCreateContext* pContext)
{
	return CFrameWndEx::OnCreateClient(lpcs, pContext);
}

LRESULT CMainFrame::OnChangeRibbonCategory(WPARAM wp, LPARAM lp)
{
	return 0;
}
