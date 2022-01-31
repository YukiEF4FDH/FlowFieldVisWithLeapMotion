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

//
//

#include "stdafx.h"
#include "FlowFieldVisWithLeapMotion.h"

#include "FlowFieldVisWithLeapMotionDoc.h"
#include "FlowFieldVisWithLeapMotionView.h"
#include "MainFrm.h"

#include "OpenGLFont.h"


extern OpenGLFont g_GLFont;

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CFlowFieldVisWithLeapMotionView

IMPLEMENT_DYNCREATE(CFlowFieldVisWithLeapMotionView, CView)

BEGIN_MESSAGE_MAP(CFlowFieldVisWithLeapMotionView, CView)
	// Standard printing commands
	ON_WM_CREATE()
	ON_WM_DESTROY()
	ON_WM_SIZE()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_MBUTTONDOWN()
	ON_WM_MBUTTONUP()
	ON_WM_MOUSEWHEEL()
	ON_WM_MOUSEMOVE()
	ON_WM_RBUTTONDOWN()
	ON_WM_TIMER()

	// Chen ===========
	ON_COMMAND(ID_QUERY_BY_LM, &OnQueryByLM)
	ON_COMMAND(ID_QUERY_BY_SKETCH, &OnQueryBySketch)
	ON_COMMAND(ID_QUERY_BY_3DFILE, &OnQueryBy3DFile)
	ON_COMMAND(ID_QUERY_BY_2DFILE, &OnQueryBy2DFile)
	ON_COMMAND(ID_CLEAR_LM_QUERY_PANEL, &OnClearLMQueryPanel)
	ON_COMMAND(ID_CLEAR_SKETCH_QUERY_PANEL, &OnClearSketchQueryPanel)
	ON_COMMAND(ID_SAVE_3D_CURVE, &OnSave3DCurve)
	ON_COMMAND(ID_SAVE_2D_CURVE, &OnSave2DCurve)
	//ON_COMMAND(ID_UPDATE_FILTER_RESULT, &OnUpdateFilterResult)
	ON_COMMAND(ID_SIMILILARITY_THRESHOLD1, &OnSimilarityThreshold1)
	ON_COMMAND(ID_SIMILILARITY_THRESHOLD2, &OnSimilarityThreshold2)
	ON_COMMAND(ID_GRANULARITY_OF_BIN_SLIDER, &OnGranularityOfBinSlider)
	ON_COMMAND(ID_GRANULARITY_OF_BIN_EDITOR, &OnGranularityOfBinEditor)
	ON_COMMAND(ID_CURSIMILILARITY_THRESHOLD1, &OnCurSimilarityThreshold1)
	ON_COMMAND(ID_CURSIMILILARITY_THRESHOLD2, &OnCurSimilarityThreshold2)
	ON_COMMAND(ID_GRANULARITY_OF_CUR_BIN_SLIDER, &OnGranularityOfCurBinSlider)
	ON_COMMAND(ID_GRANULARITY_OF_CUR_BIN_EDITOR, &OnGranularityOfCurBinEditor)
	ON_COMMAND(ID_2DSIMILILARITY_THRESHOLD1, &On2DSimilarityThreshold1)
	ON_COMMAND(ID_2DSIMILILARITY_THRESHOLD2, &On2DSimilarityThreshold2)
	ON_COMMAND(ID_2DCURSIMILILARITY_THRESHOLD1, &On2DCurSimilarityThreshold1)
	ON_COMMAND(ID_2DCURSIMILILARITY_THRESHOLD2, &On2DCurSimilarityThreshold2)
	ON_COMMAND(ID_OPEN_CONFIG_Q, &OnFileOpenConf)
	// Chen ===========
END_MESSAGE_MAP()

CFlowFieldVisWithLeapMotionView::CFlowFieldVisWithLeapMotionView()
{
	// TODO: add construction code here
}

CFlowFieldVisWithLeapMotionView::~CFlowFieldVisWithLeapMotionView()
{
}

BOOL CFlowFieldVisWithLeapMotionView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs
	return CView::PreCreateWindow(cs);
}

// CFlowFieldVisWithLeapMotionView drawing

void CFlowFieldVisWithLeapMotionView::OnDraw(CDC* pDC)
{
	CFlowFieldVisWithLeapMotionDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	HDC hDC = pDC->GetSafeHdc();

	RECT rect;
	rect.left=0.0f; rect.right=m_Width;
	rect.top=0.0f; rect.bottom=m_Height;
	glColor3f(1.0,1.0,1.0);
	glBegin(GL_POLYGON);
	glVertex2f(rect.left, rect.top);
	glVertex2f(rect.right, rect.top);
	glVertex2f(rect.right, rect.bottom);
	glVertex2f(rect.left, rect.bottom);
	glEnd();

	DrawLeftQueryDTI(hDC);
	//DrawLeft(hDC);
	DrawRightLM(hDC);
	DrawSketch(hDC);
	DrawGrad(hDC);	

	SwapBuffers(hDC);
}

// CFlowFieldVisWithLeapMotionView diagnostics

#ifdef _DEBUG
void CFlowFieldVisWithLeapMotionView::AssertValid() const
{
	CView::AssertValid();
}

void CFlowFieldVisWithLeapMotionView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CFlowFieldVisWithLeapMotionDoc* CFlowFieldVisWithLeapMotionView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CFlowFieldVisWithLeapMotionDoc)));
	return (CFlowFieldVisWithLeapMotionDoc*)m_pDocument;
}
#endif //_DEBUG


// CFlowFieldVisWithLeapMotionView message handlers
int CFlowFieldVisWithLeapMotionView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CView::OnCreate(lpCreateStruct) == -1)
		return -1;

	HWND hWnd = this->GetSafeHwnd();
	HDC hDC = this->GetDC()->GetSafeHdc();
	if (CreateOpenGL(hDC) == FALSE) {
		AfxMessageBox(_T("Cannot Create OpenGL!!"));
		return -1;
	}

	return 0;
}

void CFlowFieldVisWithLeapMotionView::OnDestroy()
{
	CView::OnDestroy();

	if(wglGetCurrentContext() != NULL) {
		wglMakeCurrent(NULL,NULL);
	}
	if(m_GLRC != NULL) {
		wglDeleteContext(m_GLRC);
		m_GLRC = NULL;
	}
}

void CFlowFieldVisWithLeapMotionView::OnSize(UINT nType, int cx, int cy)
{
	CView::OnSize(nType, cx, cy);

	m_Width = cx;
	m_Height = cy;

	m_MessageHandle.SetRect(m_Width, m_Height);
}


BOOL CFlowFieldVisWithLeapMotionView::CreateOpenGL(HDC hDC)
{
	PIXELFORMATDESCRIPTOR pfd = 
	{
		sizeof(PIXELFORMATDESCRIPTOR),
		1,
		PFD_DRAW_TO_WINDOW|PFD_SUPPORT_OPENGL|
		PFD_DOUBLEBUFFER|PFD_SUPPORT_GDI,
		PFD_TYPE_RGBA,
		24,
		0,0,0,0,0,0,
		0,
		0,
		0,
		0,0,0,0,
		32,
		0,
		0,
		PFD_MAIN_PLANE,
		0,
		0,0,0
	};

	m_PixelIndex = ChoosePixelFormat(hDC,&pfd);
	if (m_PixelIndex == 0)
		return FALSE;

	if (SetPixelFormat(hDC, m_PixelIndex, &pfd) == FALSE)
		return FALSE;

	m_GLRC = wglCreateContext(hDC);

	if (m_GLRC == NULL) {
		return FALSE;
	}

	if (wglMakeCurrent(hDC,m_GLRC) == FALSE) {
		return FALSE;
	}

	InitGL();

	return TRUE;
}

void CFlowFieldVisWithLeapMotionView::InitGL()
{
	glShadeModel(GL_SMOOTH);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClearDepth(1.0f);

	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint(GL_LINE_SMOOTH_HINT, GL_DONT_CARE);
}

// chen=====
void  CFlowFieldVisWithLeapMotionView::DrawLeftQueryDTI(HDC hDC)
{
	RECT rect = m_MessageHandle.GetLeftQueryDTIRect();
	glViewport(rect.left, rect.top, rect.right-rect.left, rect.bottom-rect.top);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, (GLfloat)rect.right/(GLfloat)(rect.bottom-rect.top), 0.001f, 1000.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	m_MessageHandle.DrawDTIFibers();

	glViewport(rect.left, rect.top, 100, 100);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	m_MessageHandle.DrawCoord();//坐标系
}
//chen=====

void CFlowFieldVisWithLeapMotionView::DrawLeft(HDC hDC)
{
	//RECT rect = m_MessageHandle.GetDTIRect();
	//glViewport(rect.left, rect.top, rect.right-rect.left, rect.bottom-rect.top);
	//glMatrixMode(GL_PROJECTION);
	//glLoadIdentity();
	//gluPerspective(45.0f, (GLfloat)rect.right/(GLfloat)(rect.bottom-rect.top), 0.001f, 1000.0f);
	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();

	//m_MessageHandle.DrawDTIFibers();

	//glViewport(rect.left, rect.top, 100, 100);
	//glMatrixMode(GL_PROJECTION);
	//glLoadIdentity();
	//glOrtho(-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f);
	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();

	//m_MessageHandle.DrawCoord();//坐标系

	//glViewport(rect.left, rect.top, rect.right-rect.left, rect.bottom-rect.top);
	//glMatrixMode(GL_PROJECTION);
	//glLoadIdentity();
	//gluOrtho2D(-1.0, 1.0, -1.0, 1.0);
	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();

	//float color[3];
	//color[0] = color[1] = color[2] = 1.0f;
	//char str[100];
	//CMainFrame *pFrame = (CMainFrame *)AfxGetApp()->m_pMainWnd;
	//if(CurrentMode == Mode::Normal)
	//{
	//	sprintf(str,"Mode: Normal");
	//	g_GLFont.DrawFont2D(str, -0.95f, 0.9f, color, 0.001f);
	//}
	//else if(CurrentMode == Mode::BoxBasicInteractions)
	//{
	//	if(pFrame->GetComboBoxPos(ID_TOOL_CUBEID)+1>0)
	//	{
	//		sprintf(str,"Mode: Box Basic Interactions");
	//		g_GLFont.DrawFont2D(str, -0.95f, 0.9f, color, 0.001f);
	//		sprintf(str,"Selected Box: %d",pFrame->GetComboBoxPos(ID_TOOL_CUBEID)+1);
	//		g_GLFont.DrawFont2D(str, -0.95f, 0.8f, color, 0.001f);
	//	}
	//	else
	//	{
	//		sprintf(str,"Mode: Box Basic Interactions");
	//		g_GLFont.DrawFont2D(str, -0.95f, 0.9f, color, 0.001f);
	//	}
	//}
	//else if(CurrentMode == Mode::BoxLogicOperations)
	//{
	//	if(pFrame->GetComboBoxPos(ID_TOOL_CLUSTERID)+1>0)
	//	{
	//		sprintf(str,"Mode: Box Logic Operations");
	//		g_GLFont.DrawFont2D(str, -0.95f, 0.9f, color, 0.001f);
	//		string expression = m_MessageHandle.GetCluster(pFrame->GetComboBoxPos(ID_TOOL_CLUSTERID)).expression;
	//		sprintf(str,"Expression: %s",expression.substr(0,(expression.size()-1)).c_str());
	//		g_GLFont.DrawFont2D(str, -0.95f, 0.8f, color, 0.001f);
	//	}
	//	else
	//	{
	//		sprintf(str,"Mode: Box Logic Operations");
	//		g_GLFont.DrawFont2D(str, -0.95f, 0.9f, color, 0.001f);
	//	}
	//}
	//else
	//{
	//	if(pFrame->GetComboBoxPos(ID_TOOL_CLASSID)+1>0)
	//	{
	//		sprintf(str,"Mode: Cluster Exploration");
	//		g_GLFont.DrawFont2D(str, -0.95f, 0.9f, color, 0.001f);
	//		sprintf(str,"Selected Cluster: %d",pFrame->GetComboBoxPos(ID_TOOL_CLASSID)+1);
	//		g_GLFont.DrawFont2D(str, -0.95f, 0.8f, color, 0.001f);
	//	}
	//	else
	//	{
	//		sprintf(str,"Mode: Cluster Exploration");
	//		g_GLFont.DrawFont2D(str, -0.95f, 0.9f, color, 0.001f);
	//	}
	//}
}

void CFlowFieldVisWithLeapMotionView::DrawGrad(HDC hDC)
{
	glViewport(0, 0, m_Width, m_Height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(-1.0f, -1.0f, 1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	float x_scale = 2.0f / static_cast<float>(m_Width);
	float y_scale = 2.0f / static_cast<float>(m_Height);
	glTranslatef(-1.0f, -1.0f, 0.0f);
	glScalef(x_scale, y_scale, 1.0f);

	glLineWidth(6.0f);
	glColor3f(0.70f, 0.80f, 0.99f);

	RECT rect = m_MessageHandle.GetLeftQueryDTIRect();
	glBegin(GL_LINE_STRIP);
	glVertex2f(rect.left, rect.top);
	glVertex2f(rect.left, rect.bottom);
	glVertex2f(rect.right, rect.bottom);
	glVertex2f(rect.right, rect.top);
	glVertex2f(rect.left, rect.top);
	glEnd();

	rect = m_MessageHandle.GetRightLMRect();
	glBegin(GL_LINE_STRIP);
	glVertex2f(rect.left, rect.top);
	glVertex2f(rect.left, rect.bottom);
	glVertex2f(rect.right, rect.bottom);
	glVertex2f(rect.right, rect.top);
	glVertex2f(rect.left, rect.top);
	glEnd();

	rect = m_MessageHandle.GetSketchRect();
	glBegin(GL_LINE_STRIP);
	glVertex2f(rect.left, rect.top);
	glVertex2f(rect.left, rect.bottom);
	glVertex2f(rect.right, rect.bottom);
	glVertex2f(rect.right, rect.top);
	glVertex2f(rect.left, rect.top);
	glEnd();
}

void CFlowFieldVisWithLeapMotionView::OnFileOpenConf()
{
	m_MessageHandle.OnOpenConfig();
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnLButtonDown(UINT nFlags, CPoint point)
{

	m_MessageHandle.OnLeftButtonDown(point);
	Invalidate(0);

	CView::OnLButtonDown(nFlags, point);
}

void CFlowFieldVisWithLeapMotionView::OnLButtonUp(UINT nFlags, CPoint point)
{
	m_MessageHandle.OnLeftButtonUp(point);
	Invalidate(0);

	CView::OnLButtonUp(nFlags, point);
}

void CFlowFieldVisWithLeapMotionView::OnRButtonDown(UINT nFlags, CPoint point)
{
	m_MessageHandle.OnMiddleButtonDown(point);

	CView::OnMButtonDown(nFlags, point);
}

void CFlowFieldVisWithLeapMotionView::OnRButtonUp(UINT nFlags, CPoint point)
{
	m_MessageHandle.OnMiddleButtonUp(point);

	CView::OnMButtonUp(nFlags, point);
}

BOOL CFlowFieldVisWithLeapMotionView::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	CPoint point = pt;
	this->ScreenToClient(&point);
	m_MessageHandle.OnMouseWheel(point, zDelta);
	Invalidate(0);

	return CView::OnMouseWheel(nFlags, zDelta, pt);
}

void CFlowFieldVisWithLeapMotionView::OnMouseMove(UINT nFlags, CPoint point)
{
	m_MessageHandle.OnMouseMove(nFlags, point);
	Invalidate(0);

	CView::OnMouseMove(nFlags, point);
}

//Chen====
void CFlowFieldVisWithLeapMotionView::SetQueryFlag(bool flag)
{
	m_MessageHandle.SetQueryFlag(flag);
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::DrawRightLM(HDC hDC)
{
	RECT rect = m_MessageHandle.GetRightLMRect();
	glColor3f(1.0,1.0,1.0);
	glBegin(GL_POLYGON);
	glVertex2f(rect.left, rect.top);
	glVertex2f(rect.right, rect.top);
	glVertex2f(rect.right, rect.bottom);
	glVertex2f(rect.left, rect.bottom);
	glEnd();
	glViewport(rect.left, rect.top, rect.right-rect.left, rect.bottom-rect.top);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, (GLfloat)(rect.right-rect.left)/(GLfloat)(rect.bottom-rect.top), 0.001f, 1000.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	m_MessageHandle.DrawRightLM();

	glViewport(rect.left, rect.top, 100, 100);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	m_MessageHandle.DrawCoord();//坐标系

	glViewport(rect.left, rect.top, rect.right-rect.left, rect.bottom-rect.top);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(-1.0, 1.0, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	float color[3];
	color[0] = color[1] = color[2] = 0.0f;
	g_GLFont.DrawFont2D("Leap Motion Query 3D", -0.95f, 0.9f, color, 0.001f);
}

void CFlowFieldVisWithLeapMotionView::DrawSketch(HDC hDC)
{
	RECT rect = m_MessageHandle.GetSketchRect();
	glViewport(rect.left, rect.top, rect.right-rect.left, rect.bottom-rect.top);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(-1.0, -1.0, 1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	m_MessageHandle.DrawSketch();

	glViewport(rect.left, rect.top, rect.right-rect.left, rect.bottom-rect.top);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(-1.0, 1.0, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	float color[3];
	color[0] = color[1] = color[2] = 0.0f;
	g_GLFont.DrawFont2D("Sketch Query 2D", -0.90f, 0.85f, color, 0.0015f);
}

//Chen===
void CFlowFieldVisWithLeapMotionView::OnClearLMQueryPanel()
{
	m_MessageHandle.OnClearLMQueryPanel();
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnClearSketchQueryPanel()
{
	m_MessageHandle.OnClearSketchQueryPanel();
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnSave3DCurve()
{
	m_MessageHandle.OnSave3DCurve();
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnSave2DCurve()
{
	m_MessageHandle.OnSave2DCurve();
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnQueryByLM()
{
	m_MessageHandle.QueryByLM();
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnQueryBy3DFile()
{
	m_MessageHandle.QueryBy3DFile();
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnQueryBySketch()
{
	m_MessageHandle.QueryBySketch();
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnQueryBy2DFile()
{
	m_MessageHandle.QueryBy2DFile();
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnSimilarityThreshold1()
{
	CMainFrame *pFrame = (CMainFrame *)(AfxGetApp()->m_pMainWnd);
	float simThreshold = pFrame->GetSliderPos(ID_SIMILILARITY_THRESHOLD1);
	CString str;
	float threshold = simThreshold;
	str.Format(_T("%f"), threshold);
	pFrame->SetEditText(ID_SIMILILARITY_THRESHOLD2, str);

	m_MessageHandle.SetAngleSimilarityThreshold(threshold);
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnSimilarityThreshold2()
{
	CMainFrame *pFrame = (CMainFrame *)(AfxGetApp()->m_pMainWnd);
	CString str = pFrame->GetEditText(ID_SIMILILARITY_THRESHOLD2);
	float simThreshold = _tstof(str);
	pFrame->SetSliderPos(ID_SIMILILARITY_THRESHOLD1,simThreshold);

	m_MessageHandle.SetAngleSimilarityThreshold(simThreshold);
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnGranularityOfBinSlider()
{
	CMainFrame *pFrame = (CMainFrame *)(AfxGetApp()->m_pMainWnd);
	int gran = pFrame->GetSliderPosInt(ID_GRANULARITY_OF_BIN_SLIDER);
	CString str;
	str.Format(_T("%d"), gran);
	pFrame->SetEditText(ID_GRANULARITY_OF_BIN_EDITOR, str);

	m_MessageHandle.SetGranularityOfAngleBin(gran);
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnGranularityOfBinEditor()
{
	CMainFrame *pFrame = (CMainFrame *)(AfxGetApp()->m_pMainWnd);
	CString str = pFrame->GetEditText(ID_GRANULARITY_OF_BIN_EDITOR);
	int gran = _tstoi(str);
	pFrame->SetSliderPosInt(ID_GRANULARITY_OF_BIN_SLIDER,gran);

	m_MessageHandle.SetGranularityOfAngleBin(gran);
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnCurSimilarityThreshold1()
{
	CMainFrame *pFrame = (CMainFrame *)(AfxGetApp()->m_pMainWnd);
	float simThreshold = pFrame->GetSliderPos(ID_CURSIMILILARITY_THRESHOLD1);
	CString str;
	float threshold = simThreshold;///20.0f;
	str.Format(_T("%f"), threshold);

	pFrame->SetEditText(ID_CURSIMILILARITY_THRESHOLD2, str);

	m_MessageHandle.SetCurvatureSimilarityThreshold(threshold);
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnCurSimilarityThreshold2()
{
	CMainFrame *pFrame = (CMainFrame *)(AfxGetApp()->m_pMainWnd);
	CString str = pFrame->GetEditText(ID_CURSIMILILARITY_THRESHOLD2);
	float simThreshold = _tstof(str);
	pFrame->SetSliderPos(ID_CURSIMILILARITY_THRESHOLD1,simThreshold);

	m_MessageHandle.SetCurvatureSimilarityThreshold(simThreshold);
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnGranularityOfCurBinSlider()
{
	CMainFrame *pFrame = (CMainFrame *)(AfxGetApp()->m_pMainWnd);
	int gran = pFrame->GetSliderPosInt(ID_GRANULARITY_OF_CUR_BIN_SLIDER);
	CString str;
	str.Format(_T("%d"), gran);
	pFrame->SetEditText(ID_GRANULARITY_OF_CUR_BIN_EDITOR, str);

	m_MessageHandle.SetGranularityOfCurvatureBin(gran);
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::OnGranularityOfCurBinEditor()
{
	CMainFrame *pFrame = (CMainFrame *)(AfxGetApp()->m_pMainWnd);
	CString str = pFrame->GetEditText(ID_GRANULARITY_OF_CUR_BIN_EDITOR);
	int gran = _tstoi(str);
	pFrame->SetSliderPosInt(ID_GRANULARITY_OF_CUR_BIN_SLIDER,gran);

	m_MessageHandle.SetGranularityOfCurvatureBin(gran);
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::On2DSimilarityThreshold1()
{
	CMainFrame *pFrame = (CMainFrame *)(AfxGetApp()->m_pMainWnd);
	float simThreshold = pFrame->GetSliderPos(ID_2DSIMILILARITY_THRESHOLD1);
	CString str;
	float threshold = simThreshold;
	str.Format(_T("%f"), threshold);
	pFrame->SetEditText(ID_2DSIMILILARITY_THRESHOLD2, str);

	m_MessageHandle.SetAngle2DSimilarityThreshold(threshold);
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::On2DSimilarityThreshold2()
{
	CMainFrame *pFrame = (CMainFrame *)(AfxGetApp()->m_pMainWnd);
	CString str = pFrame->GetEditText(ID_2DSIMILILARITY_THRESHOLD2);
	float simThreshold = _tstof(str);
	pFrame->SetSliderPos(ID_2DSIMILILARITY_THRESHOLD1,simThreshold);

	m_MessageHandle.SetAngle2DSimilarityThreshold(simThreshold);
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::On2DCurSimilarityThreshold1()
{
	CMainFrame *pFrame = (CMainFrame *)(AfxGetApp()->m_pMainWnd);
	float simThreshold = pFrame->GetSliderPos(ID_2DCURSIMILILARITY_THRESHOLD1);
	CString str;
	float threshold = simThreshold;///20.0f;
	str.Format(_T("%f"), threshold);
	//AfxMessageBox(str);
	pFrame->SetEditText(ID_2DCURSIMILILARITY_THRESHOLD2, str);

	m_MessageHandle.Set2DCurvatureSimilarityThreshold(threshold);
	Invalidate(0);
}

void CFlowFieldVisWithLeapMotionView::On2DCurSimilarityThreshold2()
{
	CMainFrame *pFrame = (CMainFrame *)(AfxGetApp()->m_pMainWnd);
	CString str = pFrame->GetEditText(ID_2DCURSIMILILARITY_THRESHOLD2);
	float simThreshold = _tstof(str);
	pFrame->SetSliderPos(ID_2DCURSIMILILARITY_THRESHOLD1,simThreshold);

	m_MessageHandle.Set2DCurvatureSimilarityThreshold(simThreshold);
	Invalidate(0);
}
//Chen===