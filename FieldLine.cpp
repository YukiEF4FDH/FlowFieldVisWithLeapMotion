/// --------------------------XY Chen--------------------------
/// Build from DTI Fiber Explorer (Line.cpp)
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
#include "stdafx.h"
#include "FieldLine.h"

// system includes
#include <stdio.h>
#include <float.h>

// opengl includes
#include <gl\glut.h>

// implementations
CFieldLine::CFieldLine()
{
	m_points = NULL;

	m_pointNum = 0;
	m_length = 0.0f;
}

CFieldLine::~CFieldLine()
{
	SAFE_DELETE(m_points);
	m_pointNum = 0;
	m_length = 0.0f;
}

void CFieldLine::CreateFieldLine(vector<Point3F> &points)
{
	m_pointNum = static_cast<int>(points.size());
	SAFE_DELETE(m_points);
	m_points = new Point3F[m_pointNum];

	for (int i=0; i<m_pointNum; i++) 
		m_points[i] = points.at(i);

	m_length = Length();
}

void CFieldLine::CopyFieldLines(CFieldLine *line)
{
	m_pointNum = line->m_pointNum;
	SAFE_DELETE(m_points);
	m_points = new Point3F[m_pointNum];

	for (int i=0; i<m_pointNum; i++) 
	{
		m_points[i] = line->m_points[i];
	}

	m_length = line->m_length;
}

float CFieldLine::Length()
{
	return m_length;
}

void CFieldLine::Delete()
{
	SAFE_DELETE(m_points);
	m_pointNum = 0;
	m_length = 0.0f;
}