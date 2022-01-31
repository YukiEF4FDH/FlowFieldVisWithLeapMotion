/// ----------------------------------------------------------------
/// project: DTI Fiber Explorer
/// file: LineProcessor.h
/// Headfile of the line class
/// ----------------------------------------------------------------

// =================================================================
//
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
//
// Algorithm Design: Dr. Wei Chen;
// System Development: Zi'ang Ding;
// DTI Datasets: Dr. Song Zhang;
// DTI Feedback: Dr.Song Zhang;
// DTI Evaluation: Anna M. Brandt, Stephen Correia, John Allen Crow
// Project Support: Prof.Qunsheng Peng.
//
// Acknowledgement:
// This work is partially supported by 973 program of China (2009CB320800),
// NSF of China (No.60873123), the Research Initiation Program at 
// Mississippi State University. 
//
//
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; version 2 of the License
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
// 
// =================================================================

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

//#ifndef MAX
//#define MAX(a, b) ( (a) > (b) ? (a) : (b) )
//#endif
//
//#ifndef MIN
//#define MIN(a, b) ( (a) < (b) ? (a) : (b) )
//#endif
//
//#ifndef ZERO
//#define ZERO(v) ( v.x = v.y = v.z = 0.0f )
//#endif
//
//#ifndef SUB
//#define SUB(res, op1, op2) { res.x=op1.x-op2.x; res.y=op1.y-op2.y; res.z=op1.z-op2.z; }
//#endif
//
//#ifndef ADD
//#define ADD(res, op1, op2) { res.x=op1.x+op2.x; res.y=op1.y+op2.y; res.z=op1.z+op2.z; }
//#endif
//
//#ifndef DIV
//#define DIV(res, op1, op2) { res.x=op1.x/op2; res.y=op1.y/op2; res.z=op1.z/op2; }
//#endif
//
//#ifndef MUL
//#define MUL(res, op1, op2) { res.x=op1.x*op2; res.y=op1.y*op2; res.z=op1.z*op2; }
//#endif
//
//#ifndef DOT
//#define DOT(op1, op2) ( op1.x * op2.x + op1.y * op2.y + op1.z * op2.z )
//#endif
//
//#ifndef CROSS
//#define CROSS(res, op1, op2) { res.x = op1.y*op2.z-op1.z*op2.y; \
//								res.y = op1.z*op2.x-op1.x*op2.z; \
//								res.z = op1.x*op2.y-op1.y*op2.x; }
//#endif
//
//#ifndef LENGTH
//#define LENGTH(a) ( sqrt(a.x * a.x + a.y * a.y + a.z * a.z) )
//#endif
//
//#ifndef STREAMTUBE_SEG
//#define STREAMTUBE_SEG 8
//#endif


//// constant global variables
//const int MAX_LOOP = 25;

//// enums
//enum C_VALUE{
//	C_L,
//	C_P,
//	C_S
//};

// structs
struct Point3F {
	float x;
	float y;
	float z;
};

//typedef Point3F Vector3F;

struct Color3F {
	float r;
	float g;
	float b;
};

//struct TF {
//	float x_scale;
//	float y_scale;
//	float z_scale;
//
//	float m[16];
//};

// class
class CLine
{
public:
	Point3F		*m_points;
	//TF			*m_TFs;
	int			m_pointNum;

	float		m_length;
	//float		m_FA;
//	float		m_trace;
//	float		m_curvature;
	//float		m_c[3];

	//float		*m_curvatures;
	//float		*m_torsions;
	//int			m_curvatureNum;

	//Point3F		*m_streamtube;
	//Vector3F	*m_normals;


public:
	CLine(void);
	~CLine(void);

	void CreateLine(vector<Point3F> &points);
	//void CLine::CreateLine(vector<Point3F> &points, vector<TF> tfs);
	void CopyLines(CLine *line);
//	void CalculateTangential();
	//void DrawLines();
	//void DrawPoints();
	//void DrawEllipsoids();
	//void DrawStreamTubes();

	void Delete();
	//void Denoise();
	//void Simplification(const float threshold);
	float Length();
	//void SetLenda(Point3F first, Point3F last);
	//void CalculateCurvatureAndTorsion();

	//Point3F ProjectPoints(Point3F point);
	//Point3F ProjectLendas(float lenda);

public:
	//inline Point3F FirstPoint() { return m_points[0]; }
	//inline Point3F LastPoint() { return m_points[m_pointNum-1]; }
	inline float GetLength() { return m_length; }
	//inline void SetFA(const float fa) { m_FA = fa; }
	//inline float GetFA() { return m_FA; }
//	inline float GetCurvature() { return m_curvature; }

private:
	//inline void NORMALIZE(Vector3F &v)
	//{
	//	float l = LENGTH(v);
	//	v.x =v.x / l;
	//	v.y =v.y / l;
	//	v.z =v.z / l;
	//}

	//float CalculateCurvature(Point3F pa, Point3F pb, Point3F pc);

	//void CreateStreamTube();
};
