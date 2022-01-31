/// ----------------------------------------------------------------
/// project: DTI Fiber Explorer
/// file: LineProcessor.h
/// source file of the line class
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

// project includes
#include "stdafx.h"
#include "Line.h"

// system includes
#include <stdio.h>
#include <float.h>

// opengl includes
#include <gl\glut.h>


//// global variables
//float circle[] = {
//	1.0f, 0.0f, 0.0f, 
//	0.717f, 0.717f, 0.0f,
//	0.0f, 1.0f, 0.0f,
//	-0.717f, 0.717f, 0.0f,
//	-1.0f, 0.0f, 0.0f,
//	-0.717f, -0.717f, 0.0f,
//	0.0f, -1.0f, 0.0f,
//	0.717f, -0.717f, 0.0f,
//};

// implementations
CLine::CLine()
{
	m_points = NULL;
	//m_TFs = NULL;
	//m_curvatures = NULL;
	//m_torsions = NULL;

	m_pointNum = 0;
	m_length = 0.0f;
//	m_FA = 0.0f;
//	m_trace = 0.0f;
//	m_curvature = 0.0f;
	//m_c[0] = m_c[1] = m_c[2] = 0.0f;

//	m_curvatureNum = 0;

	//m_streamtube = NULL;
	//m_normals = NULL;
}

CLine::~CLine()
{
	SAFE_DELETE(m_points);
	//SAFE_DELETE(m_TFs);
//	SAFE_DELETE(m_curvatures);
//	SAFE_DELETE(m_torsions);
	m_pointNum = 0;
	m_length = 0.0f;
//	m_FA = 0.0f;
//	m_trace = 0.0f;
//	m_curvature = 0.0f;
	//m_c[0] = m_c[1] = m_c[2] = 0.0f;
//	m_curvatureNum = 0;

	//SAFE_DELETE(m_streamtube);
	//SAFE_DELETE(m_normals);
}

void CLine::CreateLine(vector<Point3F> &points)
{
	m_pointNum = static_cast<int>(points.size());
	SAFE_DELETE(m_points);
	//SAFE_DELETE(m_TFs);
	m_points = new Point3F[m_pointNum];
	//m_TFs = new TF[m_pointNum];

	//float m[16] = {
	//	1.0f, 0.0f, 0.0f, 0.0f,
	//	0.0f, 1.0f, 0.0f, 0.0f,
	//	0.0f, 0.0f, 1.0f, 0.0f,
	//	0.0f, 0.0f, 0.0f, 1.0f
	//};
	for (int i=0; i<m_pointNum; i++) {
		m_points[i] = points.at(i);
		//m_TFs[i].x_scale = 1.0f;
		//m_TFs[i].y_scale = 1.0f;
		//m_TFs[i].z_scale = 1.0f;
		//memcpy(m_TFs[i].m, m, sizeof(float)*16);
	}

	m_length = Length();

	//CreateStreamTube();
}

//void CLine::CreateLine(vector<Point3F> &points, vector<TF> tfs)
//{
//	m_pointNum = static_cast<int>(points.size());
//	SAFE_DELETE(m_points);
//	//SAFE_DELETE(m_TFs);
//	m_points = new Point3F[m_pointNum];
//	//m_TFs = new TF[m_pointNum];
//
//	for (int i=0; i<m_pointNum; i++) {
//		m_points[i] = points.at(i);
//		//m_TFs[i].x_scale = tfs.at(i).x_scale;
//		//m_TFs[i].y_scale = tfs.at(i).y_scale;
//		//m_TFs[i].z_scale = tfs.at(i).z_scale;
//		//memcpy(m_TFs[i].m, tfs.at(i).m, sizeof(float)*16);
//	}
//
//	m_length = Length();
//	CreateStreamTube();
//}

void CLine::CopyLines(CLine *line)
{
	m_pointNum = line->m_pointNum;
	//m_curvatureNum = line->m_curvatureNum;
	SAFE_DELETE(m_points);
	//SAFE_DELETE(m_curvatures);
	//SAFE_DELETE(m_torsions);
	m_points = new Point3F[m_pointNum];
	//m_curvatures = new float[m_curvatureNum];
	//m_torsions = new float[m_curvatureNum];

	for (int i=0; i<m_pointNum; i++) {
		m_points[i] = line->m_points[i];
	}
	//for (int i=0; i<m_curvatureNum; i++) {
	//	m_curvatures[i] = line->m_curvatures[i];
	//	m_torsions[i] = line->m_torsions[i];
	//}

	m_length = line->m_length;
//	m_FA = line->m_FA;
	//m_trace = line->m_trace;
	//m_curvature = line->m_curvature;
	//m_c[0] = line->m_c[0];
	//m_c[1] = line->m_c[1];
	//m_c[2] = line->m_c[2];
}

//void CLine::DrawLines()
//{
//	//glBegin(GL_LINE_STRIP);
//	//for (int i=0; i<m_pointNum; i++) {
//	//	//glNormal3f(m_tangentials[i].x, m_tangentials[i].y,
//	//	//	m_tangentials[i].z);
//	//	glVertex3f(m_points[i].x, m_points[i].y, m_points[i].z);
//	//}
//	//glEnd();
//}

//void CLine::DrawPoints()
//{
//	glBegin(GL_POINTS);
//	for (int i=0; i<m_pointNum; i++) {
//		//glNormal3f(m_tangentials[i].x, m_tangentials[i].y,
//		//	m_tangentials[i].z);
//		glVertex3f(m_points[i].x, m_points[i].y, m_points[i].z);
//	}
//	glEnd();
//}

//void CLine::DrawEllipsoids()
//{
//	//for (int i=0; i<m_pointNum; i++) {
//	//	glPushMatrix();
//	//	glTranslatef(m_points[i].x, m_points[i].y, m_points[i].z);
//	//	glMultMatrixf(m_TFs[i].m);
//	//	glScalef(1.0f, m_TFs[i].y_scale/m_TFs[i].x_scale, m_TFs[i].z_scale/m_TFs[i].x_scale);
//	//	glutSolidSphere(0.5f, 20, 20);
//	//	glPopMatrix();
//	//}
//}

//void CLine::DrawStreamTubes()
//{
//	//glBegin(GL_TRIANGLE_STRIP);
//	//for (int i=1; i<m_pointNum; i++) {
//	//	for (int j=0; j<STREAMTUBE_SEG; j++) {
//	//		int index = (i - 1) * STREAMTUBE_SEG + j;
//	//		glNormal3f(m_normals[index].x, m_normals[index].y,
//	//			m_normals[index].z);
//	//		glVertex3f(m_streamtube[index].x, m_streamtube[index].y, 
//	//			m_streamtube[index].z);
//	//		index = i * STREAMTUBE_SEG + j;
//	//		glNormal3f(m_normals[index].x, m_normals[index].y,
//	//			m_normals[index].z);
//	//		glVertex3f(m_streamtube[index].x, m_streamtube[index].y, 
//	//			m_streamtube[index].z);
//	//	}
//	//	int index = (i - 1) * STREAMTUBE_SEG;
//	//	glNormal3f(m_normals[index].x, m_normals[index].y,
//	//		m_normals[index].z);
//	//	glVertex3f(m_streamtube[index].x, m_streamtube[index].y, 
//	//		m_streamtube[index].z);
//	//	index = i * STREAMTUBE_SEG;
//	//	glNormal3f(m_normals[index].x, m_normals[index].y,
//	//		m_normals[index].z);
//	//	glVertex3f(m_streamtube[index].x, m_streamtube[index].y, 
//	//		m_streamtube[index].z);
//	//}
//	//glEnd();
//}

//void CLine::Denoise()
//{
//	float deltac = 0.1f;
//	float deltas = 0.2f;
//
//	Vector3F *normals = new Vector3F[m_pointNum];
//	for (int i=1; i<m_pointNum-1; i++) {
//		Point3F prevPt = m_points[i-1];
//		Point3F nextPt = m_points[i+1];
//		Point3F thisPt = m_points[i];
//		Vector3F thisNormal;
//		ADD(thisNormal, prevPt, nextPt);
//		DIV(thisNormal, thisNormal, 2.0f);
//		SUB(thisNormal, thisPt, thisNormal);
//		float length = LENGTH(thisNormal);
//		if (length == 0) {
//			ZERO(thisNormal);
//		} else {
//			DIV(thisNormal, thisNormal, length);
//		}
//		normals[i] = thisNormal;
//	}
//
//	Point3F *points = new Point3F[m_pointNum];
//	for (int i=1; i<m_pointNum-1; i++) {
//		float sum = 0.0f;
//		float  normalizer = 0.0f;
//		Point3F prevPt = m_points[i-1];
//		Point3F nextPt = m_points[i+1];
//		Point3F thisPt = m_points[i];
//		Vector3F thisNormal = normals[i];
//
//		Vector3F tmp;
//		SUB(tmp, prevPt, thisPt);
//		float t = LENGTH(tmp);
//		float h = DOT(thisNormal, tmp);
//		float wc = exp(- t * t / (2 * deltac * deltac));
//		float ws = exp(- h * h / (2 * deltas * deltas));
//		sum += wc * ws * h;
//		normalizer += wc * ws;
//
//		SUB(tmp, nextPt, thisPt);
//		t = LENGTH(tmp);
//		h = DOT(thisNormal, tmp);
//		wc = exp(- t * t / (2 * deltac * deltac));
//		ws = exp(- h * h / (2 * deltas * deltas));
//		sum += wc * ws * h;
//		normalizer += wc * ws;
//
//		float scale = sum / normalizer;
//		MUL(tmp, thisNormal, scale);
//		ADD(thisPt, thisPt, tmp);
//		points[i] = thisPt;
//	}
//
//	for (int i=1; i<m_pointNum-1; i++) {
//		m_points[i] = points[i];
//	}
//	SAFE_DELETE(points);
//	SAFE_DELETE(normals);
//}
//
//void CLine::Simplification(const float threshold)
//{
//	int pointNum = m_pointNum;
//	vector<Point3F> pointVec;
//	pointVec.push_back(m_points[0]);
//	for (int i=1; i<pointNum-1; i++) {
//		Point3F prevPt = pointVec.back();
//		Point3F thisPt = m_points[i];
//		Point3F nextPt = m_points[i+1];
//		Vector3F v1, v2;
//		SUB(v1, thisPt, prevPt);
//		SUB(v2, nextPt, thisPt);
//		float v1Len = LENGTH(v1);
//		float v2Len = LENGTH(v2);
//		DIV(v1, v1, v1Len);
//		DIV(v2, v2, v2Len);
//		float cosValue = DOT(v1, v2);
//		if (cosValue < threshold)
//			pointVec.push_back(thisPt);
//	}
//	pointVec.push_back(m_points[pointNum-1]);
//
//	SAFE_DELETE(m_points);
//	//SAFE_DELETE(m_tangentials);
//	//SAFE_DELETE(m_lendas);
//	pointNum = static_cast<int>(pointVec.size());
//	m_points = new Point3F[pointNum];
//	for (int i=0; i<pointNum; i++) {
//		m_points[i] = pointVec[i];
//	}
//	m_pointNum = pointNum;
//	//m_tangentials = new Vector3F[m_pointNum];
//	//m_lendas = new float[m_pointNum];
//	//CalculateTangential();
//}

float CLine::Length()
{
	//if (m_length == 0.0f) {
	//	float length = 0.0f;
	//	for (int i=1; i<m_pointNum; i++) {
	//		Vector3F v;
	//		SUB(v, m_points[i], m_points[i-1]);
	//		length += LENGTH(v);
	//	}
	//	return length;
	//} else {
		return m_length;
	//}
}

//void CLine::CalculateTangential()
//{
//	for (int i=1; i<m_pointNum-1; i++) {
//		Vector3F tangential;
//		SUB(tangential, m_points[i+1], m_points[i-1]);
//		float length = LENGTH(tangential);
//		DIV(tangential, tangential, length);
//		m_tangentials[i] = tangential;
//	}
//	m_tangentials[0] = m_tangentials[1];
//	m_tangentials[m_pointNum-1] = m_tangentials[m_pointNum-2];
//}
//
//void CLine::SetLenda(Point3F first, Point3F last)
//{
//	Point3F pt = FirstPoint();
//	Vector3F v1, v2;
//	SUB(v1, pt, first);
//	SUB(v2, pt, last);
//	float dis1 = LENGTH(v1);
//	float dis2 = LENGTH(v2);
//	if (dis1 < dis2) {
//		m_lendas[0] = 0.0f;
//		m_lendas[m_pointNum-1] = 1.0f;
//	} else {
//		m_lendas[0] = 1.0f;
//		m_lendas[m_pointNum-1] = 0.0f;
//	}
//
//	float *length = new float[m_pointNum-1];
//	float total_length = 0.0f;
//	for (int i=1; i<m_pointNum; i++) {
//		Vector3F v;
//		SUB(v, m_points[i], m_points[i-1]);
//		total_length += LENGTH(v);
//		length[i-1] = total_length;
//	}
//	if (m_lendas[0] == 0.0f) {
//		for (int i=1; i<m_pointNum-1; i++) {
//			m_lendas[i] = length[i-1] / total_length;
//		}
//	} else {
//		for (int i=1; i<m_pointNum-1; i++) {
//			m_lendas[i] = 1.0f - length[i-1] / total_length;
//		}
//	}
//	SAFE_DELETE(length);
//}
//
//Point3F CLine::ProjectPoints(Point3F point)
//{
//	int project = 0;
//	float minDis = FLT_MAX;
//	for (int i=0; i<m_pointNum; i++) {
//		Vector3F v;
//		SUB(v, m_points[i], point);
//		float dis = LENGTH(v);
//		if (dis < minDis) {
//			minDis = dis;
//			project = i;
//		}
//	}
//	return m_points[project];
//}
//
//Point3F CLine::ProjectLendas(float lenda)
//{
//	int project = 0;
//	float minDis = FLT_MAX;
//	for (int i=0; i<m_pointNum; i++) {
//		float dis = abs(m_lendas[i] - lenda);
//		if (dis < minDis) {
//			minDis = dis;
//			project = i;
//		}
//	}
//	return m_points[project];
//}

void CLine::Delete()
{
	SAFE_DELETE(m_points);
	//SAFE_DELETE(m_tangentials);
	//SAFE_DELETE(m_lendas);
	m_pointNum = 0;
	m_length = 0.0f;
//	m_FA = 0.0f;
//	m_trace = 0.0f;
//	m_curvature = 0.0f;
	//m_c[0] = m_c[1] = m_c[2] = 0.0f;
}


//void CLine::CalculateCurvatureAndTorsion()
//{
//	m_curvatureNum = 20;
//	m_curvatures = new float[m_curvatureNum];
//	m_torsions = new float[m_curvatureNum];
//
//	memset(m_curvatures, 0, sizeof(float)*m_curvatureNum);
//	memset(m_torsions, 0, sizeof(float)*m_curvatureNum);
//
//	float *curvatures = new float[m_pointNum];
//	float *lengths = new float[m_pointNum];
//	lengths[0] = 0.0f;
//	for (int i=1; i<m_pointNum-1; i++) {
//		Point3F lastPt = m_points[i-1];
//		Point3F thisPt = m_points[i];
//		Point3F nextPt = m_points[i+1];
//		curvatures[i] = CalculateCurvature(lastPt, thisPt, nextPt);
//		Vector3F va;
//		SUB(va, lastPt, thisPt);
//		float a = LENGTH(va);
//		lengths[i] = lengths[i-1] + a;
//	}
//
//	for (int i=1; i<m_pointNum-1; i++) {
//		lengths[i] /= m_length;
//	}
//	lengths[m_pointNum-1] = 1.0f;
//
//	if (m_pointNum > 2) {
//		curvatures[0] = curvatures[1];
//		curvatures[m_pointNum-1] = curvatures[m_pointNum-2];
//	} else {
//		curvatures[0] = curvatures[m_pointNum-1] = 0.0f;
//	}
//
//	int leftID = 0;
//	int rightID = 0;
//	for (int i=0; i<m_curvatureNum; i++) {
//		float l = static_cast<float>(i) / static_cast<float>(m_curvatureNum - 1);
//		for (int j=leftID; j<m_pointNum; j++) {
//			if (lengths[j] > l) {
//				leftID = j - 1;
//				rightID = j;
//				break;
//			}
//		}
//		float dl = MAX(l - lengths[leftID], 0.0f);
//		float dr = MAX(lengths[rightID] - l, 0.0f);
//		float pl = dr / (dl + dr);
//		float pr = dl / (dl + dr);
//		float cur = curvatures[leftID] * pl + curvatures[rightID] * pr;
//		m_curvatures[i] = cur;
//	}
//
////	m_curvature = 0.0f;
//	//for (int i=0; i<m_pointNum; i++) {
//	//	m_curvature += curvatures[i];
//	//}
//	//m_curvature /= static_cast<float>(m_pointNum);
//
//	//SAFE_DELETE(curvatures);
//	SAFE_DELETE(lengths);
//}

//void CLine::CalculateCurvatureAndTorsion()
//{
//	SAFE_DELETE(m_curvatures);
//	SAFE_DELETE(m_torsions);
//	m_curvatures = new float[m_pointNum];
//	m_torsions = new float[m_pointNum];
//
//	Vector3F *p_n = new Vector3F[m_pointNum];
//	Vector3F *p_b = new Vector3F[m_pointNum];
//	for (int i=1; i<m_pointNum-1; i++) {
//		Point3F lastPt = m_points[i-1];
//		Point3F thisPt = m_points[i];
//		Point3F nextPt = m_points[i+1];
//		Vector3F va, vb, vc;
//		SUB(va, lastPt, thisPt);
//		SUB(vb, nextPt, thisPt);
//		SUB(vc, nextPt, lastPt);
//		float a = LENGTH(va);
//		float b = LENGTH(vb);
//		float c = LENGTH(vc);
//		float cos_value = DOT(va, vb);
//		cos_value /= (a * b);
//		cos_value = - cos_value;
//		m_curvatures[i] = acos(cos_value);
//
//		NORMALIZE(va);
//		NORMALIZE(vb);
//		NORMALIZE(vc);
//		CROSS(p_n[i], vb, va);
//		CROSS(p_b[i], vc, p_n[i]);
//	}
//
//	for (int i=2; i<m_pointNum-2; i++) {
//		Vector3F bb;
//		SUB(bb, p_b[i+1], p_b[i-1]);
//		float temp = DOT(p_n[i], bb);
//		m_torsions[i] = -temp;
//	}
//
//	if (m_pointNum > 2) {
//		m_curvatures[0] = m_curvatures[1];
//		m_curvatures[m_pointNum-1] = m_curvatures[m_pointNum-2];
//	} else {
//		for (int i=0; i<m_pointNum; i++) {
//			m_curvatures[i] = 0.0f;
//		}
//	}
//
//	if (m_pointNum > 4) {
//		m_torsions[0] = m_torsions[1] = m_torsions[2];
//		m_torsions[m_pointNum-1] = m_torsions[m_pointNum-2] = m_torsions[m_pointNum-3];
//	} else {
//		for (int i=0; i<m_pointNum; i++) {
//			m_torsions[i] = 0.0f;
//		}
//	}
//
//	m_curvature = 0.0f;
//	for (int i=0; i<m_pointNum; i++) {
//		m_curvature += m_curvatures[i];
//	}
//	m_curvature = m_curvature / static_cast<float>(m_pointNum);
//
//	SAFE_DELETE(p_n);
//	SAFE_DELETE(p_b);
//}

//
//float CLine::CalculateCurvature(Point3F pa, Point3F pb, Point3F pc)
//{
//	Vector3F ab, cb;
//	SUB(ab, pa, pb);
//	SUB(cb, pc, pb);
//
//	Vector3F normal;
//	CROSS(normal, ab, cb);
//
//	Vector3F v1, v2;
//	CROSS(v1, normal, ab);
//	CROSS(v2, normal, cb);
//
//	Point3F p1, p2;
//	ADD(p1, pa, pb);
//	DIV(p1, p1, 2.0f);
//	ADD(p2, pc, pb);
//	DIV(p2, p2, 2.0f);
//
//	float t = (p2.x - p1.x) * v2.y - (p2.y - p1.y) * v2.x;
//	t = t / (v1.x * v2.y - v1.y * v2.x);
//
//	Point3F c;
//	c.x = p1.x + v1.x * t;
//	c.y = p1.y + v1.y * t;
//	c.z = p1.z + v1.z * t;
//
//	Vector3F r;
//	SUB(r, pa, c);
//	float radius = LENGTH(r);
//
//	//Vector3F tempV;
//	//SUB(tempV, pb, c);
//	//float tempr = LENGTH(tempV);
//	//SUB(tempV, pc, c);
//	//tempr = LENGTH(tempV);
//
//	radius = MAX(radius, 0.0000001f);
//	float cur = 1.0f / radius;
//	return cur;
//}
//
//
//void CLine::CreateStreamTube()
//{
//	//m_streamtube = new Point3F[m_pointNum*STREAMTUBE_SEG];
//	//m_normals = new Vector3F[m_pointNum*STREAMTUBE_SEG];
//	//Vector3F normal;
//	//normal.x = normal.y = 0.0f; normal.z = 1.0f;
//	//for (int i=0; i<m_pointNum; i++) {
//	//	Vector3F n, v1, v2;
//	//	if (i == 0) {
//	//		SUB(n, m_points[i+1], m_points[i]);
//	//		float len = LENGTH(n);
//	//		DIV(n, n, len);
//	//	} else if (i == m_pointNum - 1) {
//	//		SUB(n, m_points[i], m_points[i-1]);
//	//		float len = LENGTH(n);
//	//		DIV(n, n, len);
//	//	} else {
//	//		SUB(v1, m_points[i], m_points[i-1]);
//	//		SUB(v2, m_points[i+1], m_points[i]);
//	//		ADD(n, v1, v2);
//	//		float len = LENGTH(n);
//	//		DIV(n, n, len);
//	//	}
//	//	Vector3F tmp;
//	//	CROSS(tmp, normal, n);
//	//	float l = LENGTH(tmp);
//	//	if (l != 0.0f)
//	//		DIV(tmp, tmp, l);
//	//	float angle = DOT(normal, n);
//	//	angle = acos(angle);
//	//	float w = cos(angle/2.0f);
//	//	float x = tmp.x * sin(angle/2.0f); 
//	//	float y = tmp.y * sin(angle/2.0f); 
//	//	float z = tmp.z * sin(angle/2.0f);
//	//	float m[16];
//	//	m[ 0] = w * w + x * x - y * y - z * z;
//	//	m[ 1] = 2.0f * x * y + 2.0f * w * z;
//	//	m[ 2] = 2.0f * x * z - 2.0f * w * y;
//	//	m[ 3] = 0.0f;
//	//	m[ 4] = 2.0f * x * y - 2.0f * w * z;
//	//	m[ 5] = w * w - x * x + y * y - z * z;
//	//	m[ 6] = 2.0f * y * z + 2.0f * w * x;
//	//	m[ 7] = 0.0f;
//	//	m[ 8] = 2.0f * x * z + 2.0f * w * y;
//	//	m[ 9] = 2.0f * y * z - 2.0f * w * x;
//	//	m[10] = w * w - x * x - y * y + z * z;
//	//	m[11] = 0.0f;
//	//	m[12] = 0.0f;
//	//	m[13] = 0.0f;
//	//	m[14] = 0.0f;
//	//	m[15] = w * w + x * x + y * y + z * z;
//
//	//	for (int j=0; j<STREAMTUBE_SEG; j++) {
//	//		float in[4];
//	//		in[0] = circle[j*3+0] * 0.1f;
//	//		in[1] = circle[j*3+1] * 0.1f;
//	//		in[2] = circle[j*3+2] * 0.1f;
//	//		in[3] = 1.0f;
//	//		float out[4];
//	//		out[0] = m[0] * in[0] + m[4] * in[1] + m[ 8] * in[2] + m[12] * in[3];
//	//		out[1] = m[1] * in[0] + m[5] * in[1] + m[ 9] * in[2] + m[13] * in[3];
//	//		out[2] = m[2] * in[0] + m[6] * in[1] + m[10] * in[2] + m[14] * in[3];
//	//		out[3] = m[3] * in[0] + m[7] * in[1] + m[11] * in[2] + m[15] * in[3];
//	//		
//	//		m_streamtube[i*STREAMTUBE_SEG+j].x = out[0] / out[3] + m_points[i].x;
//	//		m_streamtube[i*STREAMTUBE_SEG+j].y = out[1] / out[3] + m_points[i].y;
//	//		m_streamtube[i*STREAMTUBE_SEG+j].z = out[2] / out[3] + m_points[i].z;
//
//	//		m_normals[i*STREAMTUBE_SEG+j].x = out[0] / out[3];
//	//		m_normals[i*STREAMTUBE_SEG+j].y = out[1] / out[3];
//	//		m_normals[i*STREAMTUBE_SEG+j].z = out[2] / out[3];
//	//	}
//	//}
//}