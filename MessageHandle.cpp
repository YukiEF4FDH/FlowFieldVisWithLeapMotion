/// --------------------------XY Chen--------------------------
/// Build from DTI Fiber Explorer (MessageHandle.cpp)
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

// project includes
#include "StdAfx.h"
#include <conio.h>
#include "MessageHandle.h"
#include "MainFrm.h"
#include "colormap.h"
#include "OpenGLFont.h"
#include "InputExpression.h"
#include <stack>

// opencv includes
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "FlowFieldVisWithLeapMotionView.h"
#include "FlowFieldVisWithLeapMotionDoc.h"
#include "FlowFieldVisWithLeapMotion.h"
#include "MainFrm.h"
#include <cmath>
#include <queue>
#include <vector>

#include <LeapMath.h>

// global variables
extern OpenGLFont g_GLFont;
extern CFlowFieldVisWithLeapMotionApp theApp;
extern Leap::Vector TranslateDistance;

static int MAXLMCURVELENGTH = 300;

CMessageHandle::CMessageHandle(void):m_ArcBall(2.0f, 2.0f)
{
	m_MousePt.s.X = m_MousePt.s.Y = 0;

	memset(m_LastRot.M, 0, sizeof(float)*9);
	memset(m_ThisRot.M, 0, sizeof(float)*9);
	memset(m_Transform.M, 0, sizeof(float)*16);
	m_LastRot.M[0] = m_LastRot.M[4] = m_LastRot.M[8] = 1.0f;
	m_ThisRot.M[0] = m_ThisRot.M[4] = m_ThisRot.M[8] = 1.0f;
	m_Transform.M[0] = m_Transform.M[5] = m_Transform.M[10] = m_Transform.M[15] = 1.0f;

	m_DTIScale = 1.0f;
	m_DTITransfer.s.X = m_DTITransfer.s.Y = 0.0f;

	m_InteractFlag = INTERACT_NONE;

	startSketch = false;
	m_queryFlag = true;//false;

	m_sketchScale = 1.0f;
	m_sketchTransfer.s.X = m_sketchTransfer.s.Y = 0.0f;
}

CMessageHandle::~CMessageHandle(void)
{
}

void CMessageHandle::DrawDTIFibers()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	glTranslatef(m_DTITransfer.s.X, m_DTITransfer.s.Y, -2.5f);
	glScalef(m_DTIScale, m_DTIScale, m_DTIScale);
	glMultMatrixf(m_Transform.M);

	glEnable(GL_DEPTH_TEST);

	DrawCubeWithCoord();//大正方体
	glLineWidth(1.5f);
	m_LineProcessor.DrawDTI();  // 默认情况画DTI曲线的地方
	if (m_LineProcessor.f_m_lineNum > 0)  
		m_LineProcessor.DrawFilteredDTICurves();
	glDisable(GL_DEPTH_TEST);
}

void CMessageHandle::DrawCoord()
{
	glScalef(0.7f, 0.7f, 0.7f);
	glMultMatrixf(m_Transform.M);

	glLineWidth(1.5f);

	glBegin(GL_LINES);
	
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(1.0f, 0.0f, 0.0f);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 1.0f, 0.0f);

	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 1.0f);
	
	glEnd();

	float color[3];//显示XYZ字
	color[0] = 1.0f; color[1] = 0.0f; color[2] = 0.0f;
	g_GLFont.DrawFont3D("X", 2.0f, 0.0f, 0.0f, color, 0.5f);
	color[0] = 0.0f; color[1] = 1.0f; color[2] = 0.0f;
	g_GLFont.DrawFont3D("Y", 0.0f, 2.0f, 0.0f, color, 0.5f);
	color[0] = 0.0f; color[1] = 0.0f; color[2] = 1.0f;
	g_GLFont.DrawFont3D("Z", 0.0f, 0.0f, 2.0f, color, 0.5f);
}

void CMessageHandle::OnOpenConfig()
{
	OPENFILENAME ofn;
	TCHAR buffer[255];
	memset(&ofn, 0, sizeof(ofn));
	memset(buffer, 0, sizeof(char)*255);
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;
	ofn.hInstance = NULL;
	ofn.lpstrFilter = L"Config Files (*.conf)\0*.conf\0\All Files (*.*)\0*.*\0\0";
	ofn.lpstrFile = buffer;
	ofn.nMaxFile = 254;
	ofn.lpstrTitle = L"Open Config File";
	BOOL rs = GetOpenFileName(&ofn);
	if(rs) 
	{
		char   filename[255];
		int len = static_cast<int>(wcslen(ofn.lpstrFile)) + 1;
		WideCharToMultiByte(CP_ACP, 0, ofn.lpstrFile, len, filename, 2*len, NULL, NULL);
		m_LineProcessor.OpenConfig(filename);
	}
}

void CMessageHandle::DrawCubeWithCoord()
{
	glLineWidth(2.0f);
	glColor4f(0.0f, 0.0f, 0.0f, 0.3f);
	glBegin(GL_LINES);

	glVertex3f(-0.5f, 0.5f, -0.5f);
	glVertex3f(0.5f, 0.5f, -0.5f);
	glVertex3f(0.5f, 0.5f, -0.5f);
	glVertex3f(0.5f, -0.5f, -0.5f);
	glVertex3f(-0.5f, -0.5f, 0.5f);
	glVertex3f(-0.5f, 0.5f, 0.5f);
	glVertex3f(-0.5f, 0.5f, 0.5f);
	glVertex3f(0.5f, 0.5f, 0.5f);
	glVertex3f(0.5f, 0.5f, 0.5f);
	glVertex3f(0.5f, -0.5f, 0.5f);
	glVertex3f(0.5f, -0.5f, 0.5f);
	glVertex3f(-0.5f, -0.5f, 0.5f);
	glVertex3f(-0.5f, 0.5f, -0.5f);
	glVertex3f(-0.5f, 0.5f, 0.5f);
	glVertex3f(0.5f, 0.5f, -0.5f);
	glVertex3f(0.5f, 0.5f, 0.5f);
	glVertex3f(0.5f, -0.5f, -0.5f);
	glVertex3f(0.5f, -0.5f, 0.5f);
	glEnd();

	glColor4f(0.0f, 0.0f, 0.0f, 0.3f);//1.0f, 0.0f, 0.0f, 0.3f
	glBegin(GL_LINES);
	glVertex3f(-0.5f, -0.5f, -0.5f);
	glVertex3f(0.5f, -0.5f, -0.5f);
	glEnd();
	glColor4f(0.0f, 0.0f, 0.0f, 0.3f);//0.0f, 1.0f, 0.0f, 0.3f
	glBegin(GL_LINES);
	glVertex3f(-0.5f, -0.5f, -0.5f);
	glVertex3f(-0.5f, 0.5f, -0.5f);
	glEnd();
	glColor4f(0.0f, 0.0f, 0.0f, 0.3f);//0.0f, 0.0f, 1.0f, 0.3f
	glBegin(GL_LINES);
	glVertex3f(-0.5f, -0.5f, -0.5f);
	glVertex3f(-0.5f, -0.5f, 0.5f);
	glEnd();
}

void CMessageHandle::DrawRect()
{
	glLineWidth(2.0f);
	glColor4f(1.0f, 1.0f, 1.0f, 0.3f);
	glBegin(GL_LINES);
	glVertex2f(-0.5f, -0.5f);
	glVertex2f(-0.5f, 0.5f);
	glVertex2f(-0.5f, 0.5f);
	glVertex2f(0.5f, 0.5f);
	glVertex2f(0.5f, 0.5f);
	glVertex2f(0.5f, -0.5f);
	glVertex2f(0.5f, -0.5f);
	glVertex2f(-0.5f, -0.5f);
	glEnd();
}

void CMessageHandle::OnLeftButtonDown(CPoint point)
{
	POINT_POS pos = CheckPoint(point);

	if (CheckPoint(point) == LEFT_QUERY_DTI_RECT) 
	{
		m_InteractFlag = INTERACT_QUERY_LEFT_DTI;
		m_MousePt.s.X = static_cast<float>(point.x);
		m_MousePt.s.Y = static_cast<float>(point.y);
		m_LastRot = m_ThisRot;
		m_rightLeapLastRot = m_rightLeapThisRot;
		m_ArcBall.click(&m_MousePt);
	} 
	else if (CheckPoint(point) == RIGHT_LM_RECT) 
	{
		m_InteractFlag = INTERACT_RIGHT_LM;
		m_MousePt.s.X = static_cast<float>(point.x);
		m_MousePt.s.Y = static_cast<float>(point.y);
		m_LastRot = m_ThisRot;
		m_rightLeapLastRot = m_rightLeapThisRot;
		m_ArcBall.click(&m_MousePt);
	} 
	else if (CheckPoint(point) == SKETCH_RECT) 
	{
		m_InteractFlag = INTERACT_SKETCH;
		m_MousePt.s.X = static_cast<float>(point.x);
		m_MousePt.s.Y = static_cast<float>(point.y);
		float xx = (float) point.x; float yy = (float) point.y;

		if(!startSketch) startSketch = true;

		m_LineProcessor.ClearOriginalCurve(4);
		m_LineProcessor.ClearOriginalCurve(3);
		m_LineProcessor.UpdateSketchCurve(xx,yy);

		m_ArcBall.click(&m_MousePt);
	} 
	else 
	{
		m_InteractFlag = INTERACT_NONE;
	}
}

void CMessageHandle::OnLeftButtonUp(CPoint point)
{
	m_InteractFlag = INTERACT_NONE;
	if (startSketch) 
	{
		startSketch = false;
		m_LineProcessor.WriteTmpSketchCurve();
	}
	m_InteractFlag = INTERACT_NONE;
}

void CMessageHandle::OnMiddleButtonDown(CPoint point)
{
	POINT_POS res = CheckPoint(point);
	
	if (res == LEFT_QUERY_DTI_RECT)
	{
		m_InteractFlag = INTERACT_QUERY_LEFT_DTI;
		m_MousePt.s.X = static_cast<float>(point.x);
		m_MousePt.s.Y = static_cast<float>(point.y);
	}
	else if (res == RIGHT_LM_RECT)
	{
		m_InteractFlag = INTERACT_RIGHT_LM;
		m_MousePt.s.X = static_cast<float>(point.x);
		m_MousePt.s.Y = static_cast<float>(point.y);	
	}
	else 
		m_InteractFlag = INTERACT_NONE;
}

void CMessageHandle::OnMiddleButtonUp(CPoint point)
{
	m_InteractFlag = INTERACT_NONE;
}

void CMessageHandle::OnRightButtonDown(CPoint point)
{

}

void CMessageHandle::OnRightButtonUp(CPoint point)
{
	m_InteractFlag = INTERACT_NONE;
}

void CMessageHandle::OnMouseMove(UINT nFlag, CPoint point)
{
	POINT_POS pp = CheckPoint(point);
	if ( pp == LEFT_QUERY_DTI_RECT ) 
	{
		//m_DTIMousePos.s.X = static_cast<float>(point.x);
		//m_DTIMousePos.s.Y = static_cast<float>(point.y);
	} 
	else if ( pp ==RIGHT_LM_RECT )
	{
		//m_DTIMousePos.s.X = static_cast<float>(point.x);
		//m_DTIMousePos.s.Y = static_cast<float>(point.y);
	}
	else if (pp==SKETCH_RECT && startSketch)
	{
		float xx =  (float)point.x;		float yy =  (float)point.y;
		m_LineProcessor.UpdateSketchCurve(xx,yy);
		m_LineProcessor.DrawSketchCurve();
	}

	if (nFlag == MK_LBUTTON) 
	{
		if (m_InteractFlag == INTERACT_QUERY_LEFT_DTI) 
		{
			Quat4fT ThisQuat;
			m_MousePt.s.X = static_cast<float>(point.x);
			m_MousePt.s.Y = static_cast<float>(point.y);
			m_ArcBall.drag(&m_MousePt, &ThisQuat);//鼠标拖动，计算旋转
			Matrix3fSetRotationFromQuat4f(&m_ThisRot, &ThisQuat);//ThisQuat:一个四元数，用来存旋转的信息
			Matrix3fMulMatrix3f(&m_ThisRot, &m_LastRot);//累积旋转结果
			Matrix4fSetRotationFromMatrix3f(&m_Transform, &m_ThisRot);//得到最终的旋转结果
		}
		else if (m_InteractFlag == INTERACT_RIGHT_LM)
		{
			Quat4fT ThisQuat;
			m_MousePt.s.X = static_cast<float>(point.x);
			m_MousePt.s.Y = static_cast<float>(point.y);
			m_ArcBall.drag(&m_MousePt, &ThisQuat);//鼠标拖动，计算旋转
			Matrix3fSetRotationFromQuat4f(&m_ThisRot, &ThisQuat);//ThisQuat:一个四元数，用来存旋转的信息
			Matrix3fMulMatrix3f(&m_ThisRot, &m_LastRot);//累积旋转结果
			Matrix4fSetRotationFromMatrix3f(&m_Transform, &m_ThisRot);//得到最终的旋转结果
		}
	} 
	else if (nFlag == MK_MBUTTON) 
	{
		float x = static_cast<float>(point.x) - m_MousePt.s.X;
		float y = static_cast<float>(point.y) - m_MousePt.s.Y;
		x = x / 300.0f;
		y = y / 300.0f;
		if (m_InteractFlag == INTERACT_QUERY_LEFT_DTI) 
		{
			m_DTITransfer.s.X += x;
			m_DTITransfer.s.Y -= y;
			m_MousePt.s.X = static_cast<float>(point.x);
			m_MousePt.s.Y = static_cast<float>(point.y);
		} 
	}
}

void CMessageHandle::Translate(CPoint point)
{
	POINT_POS pp = CheckPoint(point);
	if (pp == LEFT_QUERY_DTI_RECT)
	{
		//m_DTIMousePos.s.X = static_cast<float>(point.x);
		//m_DTIMousePos.s.Y = static_cast<float>(point.y);
	} 
	else if (pp == RIGHT_LM_RECT)
	{
		//m_DTIMousePos.s.X = static_cast<float>(point.x);
		//m_DTIMousePos.s.Y = static_cast<float>(point.y);
	}
	float x = static_cast<float>(point.x) - m_MousePt.s.X;
	float y = static_cast<float>(point.y) - m_MousePt.s.Y;
	x = x / 300.0f;
	y = y / 300.0f;
	if (m_InteractFlag == INTERACT_QUERY_LEFT_DTI) 
	{
		m_DTITransfer.s.X += x;
		m_DTITransfer.s.Y -= y;
		m_MousePt.s.X = static_cast<float>(point.x);
		m_MousePt.s.Y = static_cast<float>(point.y);
	} 
}

void CMessageHandle::Rotate(CPoint point,const Leap::Vector derection)
{
	POINT_POS pp = CheckPoint(point);
	if (pp == LEFT_QUERY_DTI_RECT) 
	{
		//m_DTIMousePos.s.X = static_cast<float>(point.x);
		//m_DTIMousePos.s.Y = static_cast<float>(point.y);
	} 
	else if (pp == RIGHT_LM_RECT) 
	{
		//m_DTIMousePos.s.X = static_cast<float>(point.x);
		//m_DTIMousePos.s.Y = static_cast<float>(point.y);
	}

	if (m_InteractFlag == INTERACT_QUERY_LEFT_DTI)// || m_InteractFlag == RIGHT_LM_RECT) 
	{
		Quat4fT ThisQuat;
		m_MousePt.s.X = static_cast<float>(point.x);
		m_MousePt.s.Y = static_cast<float>(point.y);
		m_ArcBall.Mydrag(&derection, &ThisQuat);//旋转手势进行中，计算旋转
		Matrix3fSetRotationFromQuat4f(&m_ThisRot, &ThisQuat);//ThisQuat:一个四元数，用来存旋转的信息
		Matrix3fMulMatrix3f(&m_ThisRot, &m_LastRot);//累积旋转结果
		Matrix4fSetRotationFromMatrix3f(&m_Transform, &m_ThisRot);//得到最终的旋转结果
	}
	else if (m_InteractFlag == RIGHT_LM_RECT)
	{
		Quat4fT ThisQuat;
		m_MousePt.s.X = static_cast<float>(point.x);
		m_MousePt.s.Y = static_cast<float>(point.y);
		m_ArcBall.Mydrag(&derection, &ThisQuat);//旋转手势进行中，计算旋转
		Matrix3fSetRotationFromQuat4f(&m_ThisRot, &ThisQuat);//ThisQuat:一个四元数，用来存旋转的信息
		Matrix3fMulMatrix3f(&m_ThisRot, &m_LastRot);//累积旋转结果
		Matrix4fSetRotationFromMatrix3f(&m_Transform, &m_ThisRot);//得到最终的旋转结果
	}
}

void CMessageHandle::OnMouseWheel(CPoint point, const float delta)
{
	POINT_POS res = CheckPoint(point);
		
	if (delta > 0.0f) 
	{
		if (res==LEFT_QUERY_DTI_RECT|| res == RIGHT_LM_RECT)
			m_DTIScale *= static_cast<float>(delta) / 120.0f * 1.1f;
	} 
	else 
	{
		if (res==LEFT_QUERY_DTI_RECT|| res == RIGHT_LM_RECT)
			m_DTIScale /= static_cast<float>(-delta) / 120.0f * 1.1f;
	}
}

void CMessageHandle::Scale(CPoint point, const float distance)
{
	POINT_POS res = CheckPoint(point);
	
	if (distance > 0.0f) 
	{
		if (res == LEFT_QUERY_DTI_RECT || res == RIGHT_LM_RECT)
			m_DTIScale *= 1.1;
	}
	else 
	{
		if (res == LEFT_QUERY_DTI_RECT || res == RIGHT_LM_RECT)
			m_DTIScale /= 1.1;
	}
}

void CMessageHandle::SetRect(const int width, const int height)
{
	m_Width = width;
	m_Height = height;

	int w = m_Width/3;
	int h = m_Height/3;
	int h2 = h*2;
	int w2 = w*2;

	m_leftQueryDTIRect.left = 0;
	m_leftQueryDTIRect.right = w2;
	m_leftQueryDTIRect.top = 0;
	m_leftQueryDTIRect.bottom = m_Height;

	m_rightLMRect.left = w*2;
	m_rightLMRect.right = m_Width;
	m_rightLMRect.top = 0;
	m_rightLMRect.bottom = h*2;

	m_sketchRect.left = w*2;
	m_sketchRect.right = m_Width;
	m_sketchRect.top = h*2;
	m_sketchRect.bottom = m_Height;

	if (width > 100 && height > 100) 
	{
		m_ArcBall.setBounds(static_cast<float>(m_leftQueryDTIRect.right-m_leftQueryDTIRect.left), 
			static_cast<float>(m_leftQueryDTIRect.bottom-m_leftQueryDTIRect.top));
		m_ArcBall.setBounds(static_cast<float>(m_rightLMRect.right-m_rightLMRect.left), 
			static_cast<float>(m_rightLMRect.bottom-m_rightLMRect.top));
	}
}

POINT_POS CMessageHandle::CheckPoint(CPoint point)
{
	int x = point.x;
	int y = m_Height - point.y;

	if (x >= m_leftQueryDTIRect.left && x <= m_leftQueryDTIRect.right 
			&& y >= m_leftQueryDTIRect.top && y <= m_leftQueryDTIRect.bottom)
	{
		return LEFT_QUERY_DTI_RECT;
	}
	else if (x >= m_sketchRect.left && x <= m_sketchRect.right 
		&& y >= m_sketchRect.top && y <= m_sketchRect.bottom) 
	{
		return SKETCH_RECT;
	}
	else if (x >= m_rightLMRect.left && x <= m_rightLMRect.right 
		&& y >= m_rightLMRect.top && y <= m_rightLMRect.bottom) 
	{
		return RIGHT_LM_RECT;
	}
	else return NONE_RECT;
}

void CMessageHandle::UpdateLMCurve_Start(float XPos,float YPos,float ZPos, int type)
{
	m_LineProcessor.ClearOriginalCurve(1);
	m_LineProcessor.ClearOriginalCurve(2);
	m_LineProcessor.UpdateLMCurve(XPos,YPos,ZPos, type);
}

void CMessageHandle::UpdateLMCurve(float XPos,float YPos,float ZPos, int type)
{
	m_LineProcessor.UpdateLMCurve(XPos,YPos,ZPos, type);
}

//void CMessageHandle::PopLMCurve()
//{
//	m_LineProcessor.PopLMCurve();
//}

void CMessageHandle::WriteTmpLMCurve()
{
	m_LineProcessor.WriteTmpLMCurve();
}

//bool CMessageHandle::LMCurveLengthIsSafe()
//{
//	return m_LineProcessor.lmCurveLength< MAXLMCURVELENGTH;
//}

void CMessageHandle::DrawRightLM()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(m_DTITransfer.s.X, m_DTITransfer.s.Y, -2.5f);
	glMultMatrixf(m_Transform.M);

	glEnable(GL_DEPTH_TEST);

	DrawCubeWithCoord();
	 
	m_LineProcessor.DrawLMCurve();
	m_LineProcessor.DrawFile3DCurve();

	glDisable(GL_DEPTH_TEST);
}

void CMessageHandle::DrawSketch()
{
	glPushMatrix();

	RECT rect;
	rect.left=0.0f-m_Width; rect.right=m_Width;
	rect.top=0.0f-m_Height; rect.bottom=m_Height;
	glColor3f(1.0,1.0,1.0);
	glBegin(GL_POLYGON);
	glVertex2f(rect.left, rect.top);
	glVertex2f(rect.right, rect.top);
	glVertex2f(rect.right, rect.bottom);
	glVertex2f(rect.left, rect.bottom);
	glEnd();
	m_LineProcessor.DrawSketchCurve();
	m_LineProcessor.DrawFile2DCurve();

	glPopMatrix();
}

void CMessageHandle::OnClearLMQueryPanel()
{
	m_LineProcessor.ClearOriginalCurve(1);
	m_LineProcessor.ClearTSCurve(1);
	m_LineProcessor.ClearOriginalCurve(2);
	m_LineProcessor.ClearTSCurve(2);

	m_LineProcessor.ClearCurvatureBinSeq();

	m_LineProcessor.ClearAngleInfo(1);
	m_LineProcessor.ClearAngleInfo(2);
	m_LineProcessor.ClearAngleInfo(5);
	m_LineProcessor.ClearCurvatureInfo(1);
	m_LineProcessor.ClearCurvatureInfo(2);
	m_LineProcessor.ClearCurvatureInfo(5);

	m_LineProcessor.ClearDTISimilarityInfo();

	m_LineProcessor.ClearFilteredDTICurves();
}

void CMessageHandle::OnClearSketchQueryPanel()
{
	m_LineProcessor.ClearOriginalCurve(4);
	m_LineProcessor.ClearTSCurve(4);
	m_LineProcessor.ClearOriginalCurve(3);
	m_LineProcessor.ClearTSCurve(3);

	m_LineProcessor.ClearCurvatureBinSeq();

	m_LineProcessor.ClearAngleInfo(4);
	m_LineProcessor.ClearAngleInfo(3);
	m_LineProcessor.ClearAngleInfo(5);
	m_LineProcessor.ClearCurvatureInfo(4);
	m_LineProcessor.ClearCurvatureInfo(3);
	m_LineProcessor.ClearCurvatureInfo(5);

	m_LineProcessor.ClearDTISimilarityInfo();

	m_LineProcessor.ClearFilteredDTICurves();
}

void CMessageHandle::OnSave3DCurve()
{
	if (m_LineProcessor.getLMCurveLength()<m_LineProcessor.MINCURVELENGTH)
	{
		string str  = "Leap Motion curve is too short!\nPlease re-input, then save it.";
		CString cstr;
		cstr = str.c_str();
		AfxMessageBox(cstr);
		return;
	}

	OPENFILENAME ofn;
	TCHAR buffer[255];
	memset(&ofn, 0, sizeof(ofn));
	memset(buffer, 0, sizeof(char)*255);
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;
	ofn.hInstance = NULL;
	ofn.lpstrFilter = _T("Config Files (*.txt)\0*.txt\0\All Files (*.*)\0*.*\0\0");
	ofn.lpstrFile = buffer;
	ofn.nMaxFile = 254;
	ofn.lpstrTitle = L"Save Sketch 2D Curve File";
	BOOL rs = GetSaveFileName(&ofn);
	if(rs) 
	{
		char   filename[255];
		int len = static_cast<int>(wcslen(ofn.lpstrFile)) + 1;
		WideCharToMultiByte(CP_ACP, 0, ofn.lpstrFile, len, filename, 2*len, NULL, NULL);
		m_LineProcessor.WriteLMCurve(filename);
	}
}

void CMessageHandle::OnSave2DCurve()
{
	if (m_LineProcessor.getSketchCurveLength()<m_LineProcessor.MINCURVELENGTH)
	{
		string str  = "Sketch curve is too short!\nPlease re-input, then save it.";
		CString cstr;
		cstr = str.c_str();
		AfxMessageBox(cstr);
		return;
	}

	OPENFILENAME ofn;
	TCHAR buffer[255];
	memset(&ofn, 0, sizeof(ofn));
	memset(buffer, 0, sizeof(char)*255);
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;
	ofn.hInstance = NULL;
	ofn.lpstrFilter = _T("Config Files (*.txt)\0*.txt\0\All Files (*.*)\0*.*\0\0");
	ofn.lpstrFile = buffer;
	ofn.nMaxFile = 254;
	ofn.lpstrTitle = L"Save Sketch 2D Curve File";
	BOOL rs = GetSaveFileName(&ofn);
	if(rs) 
	{
		char   filename[255];
		int len = static_cast<int>(wcslen(ofn.lpstrFile)) + 1;
		WideCharToMultiByte(CP_ACP, 0, ofn.lpstrFile, len, filename, 2*len, NULL, NULL);
		m_LineProcessor.WriteSketchCurve(filename);
	}
}

void CMessageHandle::QueryByLM()
{
	if (m_LineProcessor.getLMCurveLength()<m_LineProcessor.MINCURVELENGTH)
	{
		string str  = "Leap Motion curve is too short!\nPlease re-input it.";
		CString cstr;
		cstr = str.c_str();
		AfxMessageBox(cstr);
		m_LineProcessor.ClearOriginalCurve(1);
		return;
	}

	m_LineProcessor.TranslateAndScaleLMCurve();
	m_LineProcessor.RotateTSCurve(m_Transform,1);
	m_LineProcessor.WriteTmpTranslatedAndScaledLMCurve();
	m_LineProcessor.TranslateAndScaleDTICurves();
	m_LineProcessor.WriteTmpTranslatedAndScaledDTICurves();
		
	m_LineProcessor.CaculateAnglesOnTSLMCurve(); 
	m_LineProcessor.CaculateAnglesOnDTICurves();
	m_LineProcessor.CaculateCurvatureOnTranslatedLMCurve();
	m_LineProcessor.CaculateCurvatureOnTranslatedAndScaledDTICurve();

	m_LineProcessor.InitCurvatureBin();
	m_LineProcessor.MapAnglesToAngleBinSeqOnTranslatedLMCurve();
	m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnTranslatedLMCurve();
	m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnDTICurves();

	m_LineProcessor.GetSimilarityForDTICurves(1); 

	m_LineProcessor.FilterDTICurves(1);
}

void CMessageHandle::QueryBy3DFile()
{
	OPENFILENAME ofn;
	TCHAR buffer[255];
	memset(&ofn, 0, sizeof(ofn));
	memset(buffer, 0, sizeof(char)*255);
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;
	ofn.hInstance = NULL;
	ofn.lpstrFilter = L"Config Files (*.txt)\0*.txt\0\All Files (*.*)\0*.*\0\0";
	ofn.lpstrFile = buffer;
	ofn.nMaxFile = 254;
	ofn.lpstrTitle = L"Open Config File";
	BOOL rs = GetOpenFileName(&ofn);
	if(rs) 
	{
		char   filename[255];
		int len = static_cast<int>(wcslen(ofn.lpstrFile)) + 1;
		WideCharToMultiByte(CP_ACP, 0, ofn.lpstrFile, len, filename, 2*len, NULL, NULL);

		m_LineProcessor.ReadFile3DCurve(filename);
		m_LineProcessor.ClearOriginalCurve(1);
		m_LineProcessor.TranslateAndScaleFile3DCurve();
		m_LineProcessor.RotateTSCurve(m_Transform,2);
		m_LineProcessor.WriteTmpTranslatedAndScaledFile3DCurve();
		m_LineProcessor.TranslateAndScaleDTICurves();
		m_LineProcessor.WriteTmpTranslatedAndScaledDTICurves();
		
		m_LineProcessor.CaculateAnglesOnTSFile3DCurve(); 
		m_LineProcessor.CaculateCurvatureOnTranslatedFile3DCurve();
		m_LineProcessor.CaculateAnglesOnDTICurves();
		m_LineProcessor.CaculateCurvatureOnTranslatedAndScaledDTICurve();

		m_LineProcessor.InitCurvatureBin();
		m_LineProcessor.MapAnglesToAngleBinSeqOnTranslatedFile3DCurve();
		m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnTranslatedFile3DCurve();
		m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnDTICurves();
		
		m_LineProcessor.GetSimilarityForDTICurves(2);

		m_LineProcessor.FilterDTICurves(2);
	}
}

void CMessageHandle::QueryBySketch()
{
	if (m_LineProcessor.getSketchCurveLength()<m_LineProcessor.MINCURVELENGTH)
	{
		string str  = "Leap Motion curve is too short!\nPlease re-input it.";
		CString cstr;
		cstr = str.c_str();
		AfxMessageBox(cstr);
		return;
	}
	
	m_LineProcessor.TranslateAndScaleSketchCurve();
	m_LineProcessor.RotateTSCurve(m_Transform,4);
	m_LineProcessor.TranslateAndScaleDTICurves();
	m_LineProcessor.WriteTmpTranslatedAndScaledDTICurves();

	m_LineProcessor.CaculateAnglesOnTSSketchCurve();
	m_LineProcessor.CaculateAnglesOnDTICurves();
	m_LineProcessor.CaculateCurvatureOnTSSketchCurve();
	m_LineProcessor.CaculateCurvatureOnTranslatedAndScaledDTICurve();

	m_LineProcessor.InitCurvatureBin();
	m_LineProcessor.MapAnglesToAngleBinSeqOnTranslatedSketchCurve();
	m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnTSSketchCurve();
	m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnDTICurves();

	m_LineProcessor.GetSimilarityForDTICurves(4);

	m_LineProcessor.FilterDTICurves(4);

}

void CMessageHandle::QueryBy2DFile()
{
	OPENFILENAME ofn;
	TCHAR buffer[255];
	memset(&ofn, 0, sizeof(ofn));
	memset(buffer, 0, sizeof(char)*255);
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;
	ofn.hInstance = NULL;
	ofn.lpstrFilter = L"Config Files (*.txt)\0*.txt\0\All Files (*.*)\0*.*\0\0";
	ofn.lpstrFile = buffer;
	ofn.nMaxFile = 254;
	ofn.lpstrTitle = L"Open Config File";
	BOOL rs = GetOpenFileName(&ofn);
	if(rs) 
	{
		char   filename[255];
		int len = static_cast<int>(wcslen(ofn.lpstrFile)) + 1;
		WideCharToMultiByte(CP_ACP, 0, ofn.lpstrFile, len, filename, 2*len, NULL, NULL);
		m_LineProcessor.ReadFile2DCurve(filename); // 此处应该可以归一化？
		m_LineProcessor.ClearOriginalCurve(4);
		m_LineProcessor.TranslateAndScaleFile2DCurve();
		m_LineProcessor.RotateTSCurve(m_Transform,3);
		m_LineProcessor.TranslateAndScaleDTICurves();
		m_LineProcessor.WriteTmpTranslatedAndScaledDTICurves();

		m_LineProcessor.CaculateAnglesOnTSFile2DCurve(); 
		m_LineProcessor.CaculateAnglesOnDTICurves();
		m_LineProcessor.CaculateCurvatureOnTSFile2DCurve();
		m_LineProcessor.CaculateCurvatureOnTranslatedAndScaledDTICurve();

		m_LineProcessor.InitCurvatureBin();
		m_LineProcessor.MapAnglesToAngleBinSeqOnTranslatedFile2DCurve();
		m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnTSFile2DCurve();
		m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnDTICurves();

		m_LineProcessor.GetSimilarityForDTICurves(3);

		m_LineProcessor.FilterDTICurves(3);
	}
}

void CMessageHandle::SetAngleSimilarityThreshold(float simThreshold)
{
	m_LineProcessor.SetAngleSimilarityThreshold(simThreshold);
	if (m_LineProcessor.t_s_lmCurveAngles.size()>0)
		m_LineProcessor.FilterDTICurves(1);
	else if (m_LineProcessor.t_s_file3DCurveAngles.size()>0)
		m_LineProcessor.FilterDTICurves(2);
}

void CMessageHandle::SetCurvatureSimilarityThreshold(float simThreshold)
{
	m_LineProcessor.SetCurvatureSimilarityThreshold(simThreshold);
	if (m_LineProcessor.t_s_lmCurveCurvature.size()>0)
		m_LineProcessor.FilterDTICurves(1);
	else if (m_LineProcessor.t_s_file3DCurveCurvature.size()>0)
		m_LineProcessor.FilterDTICurves(2);
}

void CMessageHandle::SetGranularityOfAngleBin(int gran)
{
	m_LineProcessor.SetGranularityOfAngleBin(gran);
	m_LineProcessor.CaculateAnglesOnDTICurves();
	if (m_LineProcessor.t_s_lmCurveAngles.size()>0)
	{
		m_LineProcessor.MapAnglesToAngleBinSeqOnTranslatedLMCurve();
		m_LineProcessor.GetSimilarityForDTICurves(1); 
		m_LineProcessor.FilterDTICurves(1);
	}
	else if (m_LineProcessor.t_s_file3DCurveAngles.size()>0)
	{
		m_LineProcessor.MapAnglesToAngleBinSeqOnTranslatedFile3DCurve();
		m_LineProcessor.GetSimilarityForDTICurves(2);
		m_LineProcessor.FilterDTICurves(2);
	}
	else if (m_LineProcessor.t_s_file2DCurveAngles.size()>0)
	{
		m_LineProcessor.MapAnglesToAngleBinSeqOnTranslatedFile2DCurve();
		m_LineProcessor.GetSimilarityForDTICurves(3);
		m_LineProcessor.FilterDTICurves(3);
	}
	else if (m_LineProcessor.t_s_sketchCurveAngles.size()>0)
	{
		m_LineProcessor.MapAnglesToAngleBinSeqOnTranslatedSketchCurve();
		m_LineProcessor.GetSimilarityForDTICurves(4);
		m_LineProcessor.FilterDTICurves(4);
	}
}

void CMessageHandle::SetGranularityOfCurvatureBin(int gran)
{
	m_LineProcessor.SetGranularityOfCurvatureBin(gran);
	if (m_LineProcessor.t_s_lmCurveCurvature.size()>0)
	{
		m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnTranslatedLMCurve();
		m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnDTICurves();
		m_LineProcessor.GetSimilarityForDTICurves(1); 
		m_LineProcessor.FilterDTICurves(1);
	}
	else if (m_LineProcessor.t_s_file3DCurveCurvature.size()>0)
	{
		m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnTranslatedFile3DCurve();
		m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnDTICurves();
		m_LineProcessor.GetSimilarityForDTICurves(2); 
		m_LineProcessor.FilterDTICurves(2);
	}
	else if (m_LineProcessor.t_s_file2DCurveCurvature.size()>0)
	{
		m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnTSFile2DCurve();
		m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnDTICurves();
		m_LineProcessor.GetSimilarityForDTICurves(3); 
		m_LineProcessor.FilterDTICurves(3);
	}
	else if (m_LineProcessor.t_s_sketchCurveCurvature.size()>0)
	{
		m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnTSSketchCurve();
		m_LineProcessor.MapCurvaturesToCurvatureBinSeqOnDTICurves();
		m_LineProcessor.GetSimilarityForDTICurves(4); 
		m_LineProcessor.FilterDTICurves(4);
	}
}

void CMessageHandle::SetAngle2DSimilarityThreshold(float simThreshold)
{
	m_LineProcessor.SetAngle2DSimilarityThreshold(simThreshold);
	if (m_LineProcessor.t_s_file2DCurveAngles.size()>0)
		m_LineProcessor.FilterDTICurves(3);
	else if (m_LineProcessor.t_s_sketchCurveAngles.size()>0)
		m_LineProcessor.FilterDTICurves(4);
}

void CMessageHandle::Set2DCurvatureSimilarityThreshold(float simThreshold)
{
	m_LineProcessor.SetCurvature2DSimilarityThreshold(simThreshold);
	if (m_LineProcessor.t_s_file2DCurveCurvature.size()>0)
		m_LineProcessor.FilterDTICurves(3);
	else if (m_LineProcessor.t_s_sketchCurveCurvature.size()>0)
		m_LineProcessor.FilterDTICurves(4);
}

void CMessageHandle::TranslateCoord(Leap::Vector* distance)
{
	if(distance)
	{
		(*distance).x = m_Transform.M[0]*TranslateDistance.x + m_Transform.M[1]*TranslateDistance.y + m_Transform.M[2]*TranslateDistance.z + m_Transform.M[3]*1;
		(*distance).y = m_Transform.M[4]*TranslateDistance.x + m_Transform.M[5]*TranslateDistance.y + m_Transform.M[6]*TranslateDistance.z + m_Transform.M[7]*1;
		(*distance).z = m_Transform.M[8]*TranslateDistance.x + m_Transform.M[9]*TranslateDistance.y + m_Transform.M[10]*TranslateDistance.z + m_Transform.M[11]*1;
	}
}