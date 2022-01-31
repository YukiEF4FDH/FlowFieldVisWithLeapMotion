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

// FlowFieldVisWithLeapMotionView.h : interface of the CFlowFieldVisWithLeapMotionView class
//


#pragma once

#include "MessageHandle.h"
#include "FlowFieldVisWithLeapMotionDoc.h"
#include <conio.h>
class SampleListener;
class CFlowFieldVisWithLeapMotionView : public CView
{
// OpenGL
private:
	int m_Width;
	int m_Height;
	int m_PixelIndex;
	HGLRC m_GLRC;
private:
	BOOL CreateOpenGL(HDC hDC);
	void InitGL();
	void DrawLeft(HDC hDC);
	void DrawGrad(HDC hDC);

	//Chen======
	//query-based
	void DrawLeftQueryDTI(HDC hDC);
	void DrawRightLM(HDC hDC);
	void DrawSketch(HDC hdc);
	//Chen======

public:
	CMessageHandle m_MessageHandle;

public:
	void SetQueryFlag(bool flag);
protected:
	afx_msg void OnFileOpenConf();

	//Chen===========
	afx_msg void OnClearLMQueryPanel();
	afx_msg void OnClearSketchQueryPanel();

	afx_msg void OnSave3DCurve();
	afx_msg void OnSave2DCurve();

	afx_msg void OnQueryByLM();
	afx_msg void OnQueryBy3DFile();
	afx_msg void OnQueryBySketch();
	afx_msg void OnQueryBy2DFile();

	afx_msg void OnSimilarityThreshold1();		//AngleSimThreshold - Slider
	afx_msg void OnSimilarityThreshold2();		//AngleSimThreshold - Edior
	afx_msg void OnGranularityOfBinSlider();	//AngleBinGranularity - Slider
	afx_msg void OnGranularityOfBinEditor();	//AngleBinGranularity - Editor
	afx_msg void OnCurSimilarityThreshold1();	//CurvatureSimThreshold - Slider 
	afx_msg void OnCurSimilarityThreshold2();	//CurvatureSimThreshold - Editor
	afx_msg void OnGranularityOfCurBinSlider();	//CurvatureBinGranularity - Slider
	afx_msg void OnGranularityOfCurBinEditor();	//CurvatureBinGranularity - Editor
	afx_msg void On2DSimilarityThreshold1();		//AngleSimThreshold - Slider
	afx_msg void On2DSimilarityThreshold2();		//AngleSimThreshold - Edior
	afx_msg void On2DCurSimilarityThreshold1();	//CurvatureSimThreshold - Slider 
	afx_msg void On2DCurSimilarityThreshold2();	//CurvatureSimThreshold - Editor
	//Chen=======

protected: // create from serialization only
	CFlowFieldVisWithLeapMotionView();
	DECLARE_DYNCREATE(CFlowFieldVisWithLeapMotionView)

// Attributes
public:
	CFlowFieldVisWithLeapMotionDoc* GetDocument() const;

// Operations
public:

// Overrides
public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:

// Implementation
public:
	virtual ~CFlowFieldVisWithLeapMotionView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnDestroy();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
public:
	friend SampleListener;
};

#ifndef _DEBUG  // debug version in FlowFieldVisWithLeapMotionView.cpp
inline CFlowFieldVisWithLeapMotionDoc* CFlowFieldVisWithLeapMotionView::GetDocument() const
   { return reinterpret_cast<CFlowFieldVisWithLeapMotionDoc*>(m_pDocument); }
#endif

