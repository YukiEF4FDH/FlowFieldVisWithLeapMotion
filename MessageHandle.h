/// --------------------------XY Chen--------------------------
/// Build from DTI Fiber Explorer (MessageHandle.h)
// Copyright (c) 2008-2009, all rights reserved by
// - State Key Lab. of CAD&CG, Zhejiang University
//
// More information:
// http://www.cad.zju.edu.cn/home/chenwei/interface/
//
// Contact:
// Wei Chen:    chenwei@cad.zju.edu.cn or shearwarp@gmail.com
// Ziang Ding:  dingziang@cad.zju.edu.cn or dingziang@gmail.com
// Song Zhang:  szhang@cse.msstate.edu
/// --------------------------XY Chen--------------------------

#pragma once

// system includes
#include <vector>

// opencv includes
#include <highgui.h>

// project includes
#include "ArcBall.h"
#include "FieldLineProcessor.h"
#include "VolumeData.h"
#include <queue>
#include <LeapMath.h>
using namespace std;

// enums
enum INTERACT_FLAG{
	INTERACT_NONE,
	INTERACT_SKETCH,
	INTERACT_RIGHT_LM,
	INTERACT_QUERY_LEFT_DTI
};

enum POINT_POS {
	NONE_RECT,
	LEFT_QUERY_DTI_RECT,
	SKETCH_RECT,
	RIGHT_LM_RECT
};

struct Point2F{
	float x;
	float y;
};

// class
class CMessageHandle
{
private:
	ArcBallT	m_ArcBall;

	Point2fT	m_MousePt;
	Matrix3fT	m_LastRot;
	Matrix3fT	m_ThisRot;
	Matrix4fT	m_Transform;

	INTERACT_FLAG m_InteractFlag;

	int m_Width;
	int m_Height;

	CFieldLineProcessor m_LineProcessor;

	// Leap Motion
	vector<Point2F> sketchCurvePoints;
	bool startSketch;	

	bool m_queryFlag;
	RECT m_leftQueryDTIRect;
	RECT m_sketchRect;
	RECT m_rightLMRect;

	Point2fT	m_rightLeapMousePt;
	Matrix3fT	m_rightLeapLastRot;
	Matrix3fT	m_rightLeapThisRot;
	Matrix4fT	m_rightLeapTransform;
	float		m_rightLMScale;
	Point2fT	m_rightLMTransfer;
	float		m_sketchScale;
	Point2fT	m_sketchTransfer;
public:
	float		m_DTIScale;
	Point2fT	m_DTITransfer;

	CMessageHandle(void);
	~CMessageHandle(void);

	// OpenGL
public:
	void DrawDTIFibers();

	void DrawCoord();
private:
	void DrawCubeWithCoord();
	void DrawRect();

	// Mouse
public:
	void OnLeftButtonDown(CPoint point);
	void OnLeftButtonUp(CPoint point);
	void OnMiddleButtonDown(CPoint point);
	void OnMiddleButtonUp(CPoint point);
	void OnRightButtonDown(CPoint point);
	void OnRightButtonUp(CPoint point);
	void OnMouseMove(UINT nFlag, CPoint point);
	void Translate(CPoint point);
	void Rotate(CPoint point,const Leap::Vector derection);
	void OnMouseWheel(CPoint point,const float delta);
	void Scale(CPoint point, const float distance);
	void SetRect(const int width, const int height);
	void TranslateCoord(Leap::Vector* distance);
private:
	POINT_POS CheckPoint(CPoint point);

	// File
public:
	void OnOpenConfig();

	// Leap Motion
public:
	void OnClearLMQueryPanel();
	void OnClearSketchQueryPanel();

	void OnSave3DCurve();
	void OnSave2DCurve();

	void QueryByLM();
	void QueryBy3DFile();
	void QueryBySketch();
	void QueryBy2DFile();

	void SetAngleSimilarityThreshold(float simThreshold);
	void SetGranularityOfAngleBin(int gran);
	void SetCurvatureSimilarityThreshold(float simThreshold);
	void SetGranularityOfCurvatureBin(int gran);
	void SetAngle2DSimilarityThreshold(float simThreshold);
	void Set2DCurvatureSimilarityThreshold(float simThreshold);

	// Leap Motion
public:
	inline void SetQueryFlag(const bool flag) { m_queryFlag = flag; SetRect(m_Width, m_Height); }
	inline bool GetQueryFlag() { return m_queryFlag; }

	inline RECT GetLeftQueryDTIRect() { return m_leftQueryDTIRect; }
	inline RECT GetRightLMRect() { return m_rightLMRect; }
	inline RECT GetSketchRect() { return m_sketchRect; }
	void DrawRightLM();
	void DrawSketch();

	// 0: Merged A & B  1: A  2: B
	void UpdateLMCurve_Start(float lmTipXPos,float lmTipYPos,float lmTipZPos, int type);
	void UpdateLMCurve(float lmTipXPos,float lmTipYPos,float lmTipZPos, int type);

	//void PopLMCurve();

	void WriteTmpLMCurve();

	//bool LMCurveLengthIsSafe();
};
