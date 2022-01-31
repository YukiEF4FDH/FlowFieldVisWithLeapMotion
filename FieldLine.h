/// --------------------------XY Chen--------------------------
/// Build from DTI Fiber Explorer (Line.h)
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
#include <windows.h>
#include <vector>
#include <math.h>

// opengl includes
#include <gl\gl.h>
#include <gl\glu.h>
#include <gl\glaux.h>

using namespace std;

// defines
#ifndef SAFE_DELETE
#define SAFE_DELETE(p) { if(p) {delete[] p; p = NULL;} }
#endif

// structs
struct Point3F {
	float x;
	float y;
	float z;
};

struct Color3F {
	float r;
	float g;
	float b;
};

// class
class CFieldLine
{
public:
	Point3F		*m_points;
	int			m_pointNum;

	float		m_length;

public:
	CFieldLine(void);
	~CFieldLine(void);

	void CreateFieldLine(vector<Point3F> &points);
	void CopyFieldLines(CFieldLine *line);

	void Delete();
	float Length();
public:
	inline float GetLength() { return m_length; }
};
