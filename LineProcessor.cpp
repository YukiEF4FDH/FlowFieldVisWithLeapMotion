/// ----------------------------------------------------------------
/// project: DTI Fiber Explorer
/// file: LineProcessor.cpp
/// Source file of the line processor
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


// System Includes
#include <float.h>
#include <omp.h>

// Project Includes
#include "stdafx.h"
#include "LineProcessor.h"
#include "MessageHandle.h"
//#include "colormap.h"
#include "OpenGLFont.h"
#include <cmath>

// OpenCV Includes
#include <cv.h>
#include <cxcore.h>

// CUDA Includes
#include <vector_types.h>
//#include <cutil_inline.h>

#include <deque>

// global variables
//extern CColorMap g_colorMap;
extern OpenGLFont g_GLFont;

#include <fstream>
#include <string>
#include <Eigen/Dense>
using Eigen::MatrixXd;
typedef Eigen::Vector3d Point;
#include <conio.h>
// CUDA functions


//extern "C"
//{
//	int SMACOF(float* result, float *configuration, float* proximity, int n, 
//		int lowDim, double* GPUtimeCost, double* CPUtimeCost, int maxIteration, 
//		float epsilon);
//}

//extern "C" float * CalculateDM(float3 *points, int point_num, int2 *pos, int line_num);


CLineProcessor::CLineProcessor(void)
{
	MAXCURVELENGTH = 1000;
	MINCURVELENGTH = 3;
	MAXCURVELENGTH = 1000;
	MINCURVELENGTH = 3;
	MAXCURVELENGTH = 1000;
	MINCURVELENGTH = 3;
	MAXCURVELENGTH = 1000;
	MINCURVELENGTH = 3;

	m_lines = NULL;
	m_lineNum = 0;
//	m_points = NULL;
	m_pointNum = 0;
	//m_selectFlags = NULL;
	//m_selectFlagsBackup = NULL;
	//m_showFlags = NULL;
	//m_availableFlags = NULL;

	//m_DMFlag = false;
	//m_DMSpace = NULL;
	//m_DMCurvature = NULL;
	//m_DMTorsion = NULL;
	//m_distanceMatrix = NULL;
	//m_results = NULL;
	//m_classNum = 0;
	
	//m_minLength = m_maxLength = 0.0f;
	//m_minFA = m_maxFA = 0.0f;
	//m_minCur = m_maxCur = 0.0f;

	m_center.x = m_center.y = m_center.z = 0.0f;

	//m_lengthHistogramNum = 50;
	//m_lengthHistogram = new float[m_lengthHistogramNum];
	//m_lengthStep = new float[m_lengthHistogramNum+1];
	//memset(m_lengthHistogram, 0, sizeof(float)*m_lengthHistogramNum);

	//m_FAHistogramNum = 50;
	//m_FAHistogram = new float[m_FAHistogramNum];
	//m_FAStep = new float[m_FAHistogramNum+1];
	//memset(m_FAHistogram, 0, sizeof(float)*m_FAHistogramNum);

	//m_CurHistogramNum = 50;
	//m_CurHistogram = new float[m_CurHistogramNum];
	//m_CurStep = new float[m_CurHistogramNum+1];
	/*memset(m_CurHistogram, 0, sizeof(float)*m_CurHistogramNum);

	m_hierachicalResults = NULL;

	m_principalCurves = NULL;*/
//	m_principalPoints = NULL;

	//m_configuration = NULL;
	//m_configurationFlag = false;

	m_first = NULL;
	m_vertCount = NULL;

	//m_firstTime = false;

	m_colors = NULL;
	m_vertices = NULL;

	//m_abstractResults = NULL;

	m_originalLines = NULL;
//	m_originalPoints2D = NULL;
	//m_originalDMSpace = NULL;
	//m_originalDMCurvature = NULL;	
	//m_originalDMTorsion = NULL;		
	m_originalLineNum = 0;

	m_abstractLines = NULL;
//	m_abstractPoints2D = NULL;
	//m_abstractDMSpace = NULL;
	//m_abstractDMCurvature = NULL;	
	//m_abstractDMTorsion = NULL;		
	m_abstractLineNum = 0;

	//m_vertexBuffer = 0;
	//m_colorBuffer = 0;

	// Chen=======
	sketchXPos.clear();
	sketchYPos.clear();
	sketchCurveLength = 0;

	file2DXPos.clear();
	file2DYPos.clear();
	file2DCurveLength = 0;

	lmTipXPos.clear();
	lmTipYPos.clear();
	lmTipZPos.clear();
	lmCurveLength = 500;

	file3DXPos.clear();
	file3DYPos.clear();
	file3DZPos.clear();
	file3DCurveLength = 0;

	t_s_file3DXPos.clear();
	t_s_file3DYPos.clear();
	t_s_file3DZPos.clear();
	
	t_s_lmTipXPos.clear();
	t_s_lmTipYPos.clear();
	t_s_lmTipZPos.clear();

	for(int i=1; i<=361; i+=10) // default granularity = 10
		AngleBinSeq.push_back(i-1);


	t_s_file3DCurveAngles.clear();
	t_s_file3DCurveAngleSeq.clear();

	t_s_lmCurveAngles.clear();
	t_s_lmCurveAngleSeq.clear();
	
	DTICurveAngles.clear();
	DTICurvesAngleSeq.clear();

	aSimilarityForDTICurves.clear();

	ANGLE_SIMILARITY_THRESHOLD_3D = 0.5; // default = 0.5
	CURVATURE_SIMILARITY_THRESHOLD_3D = 0.1;
	ANGLE_SIMILARITY_THRESHOLD_2D = 0.1;
	CURVATURE_SIMILARITY_THRESHOLD_2D = 0.1;
	FILTERED_FLAG = false;

	ts_m_lines = NULL;
	ts_m_vertices = NULL;

	//ts_f_m_vertices = NULL;
	//ts_f_m_lineNum = 0;
	//ts_f_m_first = NULL;
	//ts_f_m_vertCount = NULL;

	f_m_vertices = NULL;
	f_m_lineNum = 0;
	f_m_first = NULL;
	f_m_vertCount = NULL;

	n1 = 1; 
	n2 = 1;

	t_s_file3DCurveS.clear();
	t_s_file3DXFirstOrderDiscreteDerivative.clear();
	t_s_file3DYFirstOrderDiscreteDerivative.clear();
	t_s_file3DZFirstOrderDiscreteDerivative.clear();

	MAX_CUR = 0.0;
	MIN_CUR = INT_MAX;
	// Chen==========
}

CLineProcessor::~CLineProcessor(void)
{
	SAFE_DELETE(m_lines);
	m_lineNum = 0;
//	SAFE_DELETE(m_points);
	m_pointNum = 0;
	//SAFE_DELETE(m_DMSpace);
	//SAFE_DELETE(m_DMCurvature);
	//SAFE_DELETE(m_DMTorsion);
	//SAFE_DELETE(m_distanceMatrix);
	//SAFE_DELETE(m_results);
	//m_classNum = 0;

	//SAFE_DELETE(m_selectFlags);
	//SAFE_DELETE(m_selectFlagsBackup);
	//SAFE_DELETE(m_showFlags);
	//SAFE_DELETE(m_availableFlags);

	//SAFE_DELETE(m_lengthHistogram);
	//SAFE_DELETE(m_lengthStep);

	//SAFE_DELETE(m_FAHistogram);
	//SAFE_DELETE(m_FAStep);

	//SAFE_DELETE(m_CurHistogram);
	//SAFE_DELETE(m_CurStep);

	//SAFE_DELETE(m_hierachicalResults);

	//SAFE_DELETE(m_principalCurves);
	//SAFE_DELETE(m_principalPoints);

	//SAFE_DELETE(m_configuration);

	SAFE_DELETE(m_first);
	SAFE_DELETE(m_vertCount);

	SAFE_DELETE(m_colors);
	SAFE_DELETE(m_vertices);

	m_lineLevel = LEVEL_ORIGINAL;
	//m_abstractFlag = false;

	//SAFE_DELETE(m_abstractResults);
	
	SAFE_DELETE(m_originalLines);
	//SAFE_DELETE(m_originalPoints2D);
	//SAFE_DELETE(m_originalDMSpace);
	//SAFE_DELETE(m_originalDMCurvature);
	//SAFE_DELETE(m_originalDMTorsion);
	m_originalLineNum = m_originalPointNum = 0;

	SAFE_DELETE(m_abstractLines);
	//SAFE_DELETE(m_abstractPoints2D);
	//SAFE_DELETE(m_abstractDMSpace);
	//SAFE_DELETE(m_abstractDMCurvature);
	//SAFE_DELETE(m_abstractDMTorsion);
	m_abstractLineNum = m_abstractPointNum = 0;

	SAFE_DELETE(ts_m_lines);
	SAFE_DELETE(ts_m_vertices);


	//SAFE_DELETE(ts_f_m_vertices);
	//SAFE_DELETE(ts_f_m_first);
	//SAFE_DELETE(ts_f_m_vertCount);

	SAFE_DELETE(f_m_vertices);
	SAFE_DELETE(f_m_first);
	SAFE_DELETE(f_m_vertCount);
}


void CLineProcessor::InitILRender()
{
	glewInit();

	m_render.setErrorCallback(NULL);
	//m_ILID = ILines::ILRender::IL_INVALID_IDENTIFIER;
	float ka = 0.05f, kd = 0.8f, ks = 1.0f, gloss = 10.0f;
	int	texDim = 256;
	GLfloat		lightDirection[4] = { 0.0f, 0.0f, 1.0f };
	GLfloat		lightPosition[4] =  { 0.0f, 0.0f, 0.0f, 1.0f };
	if (ILines::ILRender::isLightingModelSupported(ILines::ILLightingModel::IL_CYLINDER_PHONG)) {
		m_render.setupTextures(ka, kd, ks, gloss, texDim, ILines::ILLightingModel::IL_CYLINDER_PHONG, 
			false, lightDirection);
	}
}
//
void CLineProcessor::CreateVertexBuffer()
{
	SAFE_DELETE(m_first);
	SAFE_DELETE(m_vertCount);
	m_first = new int[m_lineNum];
	m_vertCount = new int[m_lineNum];
	int total_num = 0;
	for (int i=0; i<m_lineNum; i++) {
		m_first[i] = total_num;
		m_vertCount[i] = m_lines[i].m_pointNum;
		total_num += m_lines[i].m_pointNum;
	}
	m_vertices = new float[total_num*3];
	int index = 0;
	for (int i=0; i<m_lineNum; i++) {
		int point_num = m_lines[i].m_pointNum;
		for (int j=0; j<point_num; j++) {
			m_vertices[index*3+0] = m_lines[i].m_points[j].x;
			m_vertices[index*3+1] = m_lines[i].m_points[j].y;
			m_vertices[index*3+2] = m_lines[i].m_points[j].z;
			index++;
		}
	}

	m_min.x = FLT_MAX; m_min.y = FLT_MAX; m_min.z = FLT_MAX;
	m_max.x = -FLT_MAX; m_max.y = -FLT_MAX; m_max.z = -FLT_MAX;
	for (int i=0; i<m_lineNum; i++) {
		int point_num = m_lines[i].m_pointNum;
		for (int j=0; j<point_num; j++) {
			Point3F pt = m_lines[i].m_points[j];
			if (pt.x < m_min.x)
				m_min.x = pt.x;
			if (pt.y < m_min.y)
				m_min.y = pt.y;
			if (pt.z < m_min.z)
				m_min.z = pt.z;

			if (pt.x > m_max.x)
				m_max.x = pt.x;
			if (pt.y > m_max.y)
				m_max.y = pt.y;
			if (pt.z > m_max.z)
				m_max.z = pt.z;
		}
	}

	//if (m_vertexBuffer != 0)
	//	glDeleteBuffers(1, &m_vertexBuffer);

	//glGenBuffers(1, &m_vertexBuffer);
	//glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	//glBufferData(GL_ARRAY_BUFFER, sizeof(float)*total_num*3, vertices, GL_STATIC_DRAW);

	//SAFE_DELETE(vertices);
}

void CLineProcessor::CreateColorBuffer()
{
	int total_num = 0;
	for (int i=0; i<m_lineNum; i++) {
		total_num += m_vertCount[i];
	}
	SAFE_DELETE(m_colors);
	m_colors = new float[total_num*4];
}


//int CLineProcessor::OpenDataFile(const char *filename)
//{
	//FILE *fp = fopen(filename, "r");
	//if (fp == NULL)
	//	return 0;

	//// read the line number
	//fscanf(fp, "%d\n", &m_lineNum);
	//// read lines
	//m_lines = new CLine[m_lineNum];
	//for (int i=0; i<m_lineNum; i++) {
	//	int pointNum;
	//	fscanf(fp, "%d\n", &pointNum);
	//	vector<Point3F> points;
	//	vector<TF> TFs;
	//	float cl = 0.0f;
	//	float cp = 0.0f;
	//	float cs = 0.0f;
	//	float trace = 0.0f;
	//	float fa = 0.0f;
	//	for (int j=0; j<pointNum; j++) {
	//		Point3F pt;
	//		TF tf;
	//		fscanf(fp, "%f %f %f", &pt.x, &pt.y, &pt.z);
	//		pt.y = -pt.y;
	//		float l1, l2, l3;
	//		fscanf(fp, "%f %f %f", &l1, &l2, &l3);
	//		float m11, m12, m13, m21, m22, m23, m31, m32, m33;
	//		fscanf(fp, "%f %f %f %f %f %f %f %f %f", &m11, &m12, &m13, 
	//			&m21, &m22, &m23, &m31, &m32, &m33);
	//		float ffa;
	//		fscanf(fp, "%f\n", &ffa);

	//		tf.x_scale = l1;
	//		tf.y_scale = l2;
	//		tf.z_scale = l3;
	//		tf.m[0] = m11;		tf.m[1] = m12;		tf.m[2] = m13;		tf.m[3] = 0.0f;
	//		tf.m[4] = m21;		tf.m[5] = m22;		tf.m[6] = m23;		tf.m[7] = 0.0f;
	//		tf.m[8] = m31;		tf.m[9] = m32;		tf.m[10] = m33;		tf.m[11] = 0.0f;
	//		tf.m[12] = 0.0f;	tf.m[13] = 0.0f;	tf.m[14] = 0.0f;	tf.m[15] = 1.0f;
	//		//tf.m[0] = m11;		tf.m[1] = m21;		tf.m[2] = m21;		tf.m[3] = 0.0f;
	//		//tf.m[4] = m12;		tf.m[5] = m22;		tf.m[6] = m32;		tf.m[7] = 0.0f;
	//		//tf.m[8] = m13;		tf.m[9] = m23;		tf.m[10] = m33;		tf.m[11] = 0.0f;
	//		//tf.m[12] = 0.0f;	tf.m[13] = 0.0f;	tf.m[14] = 0.0f;	tf.m[15] = 1.0f;
	//		points.push_back(pt);
	//		TFs.push_back(tf);
	//		cl += (l1 - l2) / (l1 + l2 + l3);
	//		cp += 2.0f * (l2 - l3) / (l1 + l2 + l3);
	//		cs += 3.0f * l3 / (l1 + l2 + l3);
	//		trace += m11 + m22 + m33;
	//		fa += ffa;
	//	}
	//	cl /= pointNum;
	//	cp /= pointNum;
	//	cs /= pointNum;
	//	trace /= pointNum;
	//	fa /= pointNum;

	//	m_lines[i].CreateFieldLine(points, TFs);
	//	
	//	m_lines[i].CalculateCurvatureAndTorsion();
	//	
	//	m_lines[i].m_trace = trace;
	//	m_lines[i].m_FA = fa;
	//	m_lines[i].m_c[C_L] = cl;
	//	m_lines[i].m_c[C_P] = cp;
	//	m_lines[i].m_c[C_S] = cs;
	//}
	//fclose(fp);

	//CreateFlags(NULL);
	//SetMinMaxLength();
	//SetFA();
	//SetCurvature();
	//SetCenter();

	//m_classNum = 0;

	//InitILRender();
	//CreateVertexBuffer();
	//CreateColorBuffer();

	//return m_lineNum;
//	return 0;
//}

//int CLineProcessor::OpenDataFile(const char *filename)
//{
//	FILE *fp = fopen(filename, "r");
//	if (fp == NULL)
//		return 0;
//
//	// read the line number
//	fscanf(fp, "%d\n", &m_lineNum);
//	// read lines
//	m_lines = new CLine[m_lineNum];
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum;
//		fscanf(fp, "%d\n", &pointNum);
//		vector<Point3F> points;
//		float fa = 0.0f;
//		for (int j=0; j<pointNum; j++) {
//			Point3F pt;
//			fscanf(fp, "%f %f %f", &pt.x, &pt.y, &pt.z);
//			points.push_back(pt);
//			float l1, l2, l3;
//			fscanf(fp, "%f %f %f\n", &l1, &l2, &l3);
//			float l = (l1 + l2 + l3) / 3.0f;
//			float temp = sqrt(l1 * l1 + l2 * l2 + l3 * l3);
//			l1 = sqrt((l1 - l) * (l1 - l));
//			l2 = sqrt((l2 - l) * (l2 - l));
//			l3 = sqrt((l3 - l) * (l3 - l));
//			float f = 1.22474487f * (l1 + l2 + l3) / temp;
//			fa += f;
//		}
//		fa = fa / static_cast<float>(pointNum);
//		m_lines[i].CreateFieldLine(points);
//		m_lines[i].CalculateCurvatureAndTorsion();
//		m_lines[i].m_trace = 1.0f;
//		m_lines[i].m_FA = fa;
//		m_lines[i].m_c[C_L] = 1.0f;
//		m_lines[i].m_c[C_P] = 1.0f;
//		m_lines[i].m_c[C_S] = 1.0f;
//	}
//	fclose(fp);
//
//	CreateFlags(NULL);
//	SetMinMaxLength();
//	SetFA();
//	SetCurvature();
//	SetCenter();
//
//	m_classNum = 0;
//
//	InitILRender();
//	CreateVertexBuffer();
//	CreateColorBuffer();
//
//	return m_lineNum;
//}
//
//int CLineProcessor::OpenHHR(const char *filename)
//{
//	FILE *fp = fopen(filename, "r");					/* Open the file */
//	if (fp == NULL) 
//		return 0;
//
//	// read the line number
//	fscanf(fp, "%d\n", &m_lineNum);
//	m_lineNum = 4096;
//	// read lines
//	SAFE_DELETE(m_lines);
//	m_lines = new CFieldLine[m_lineNum];
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum;
//		fscanf(fp, "%d\n", &pointNum);
//		vector<Point3F> points;
//		float fa = 0.0f;
//		fscanf(fp, "%f\n", &fa);
//		//fa = 0.0f;
//		for (int j=0; j<pointNum; j++) {
//			Point3F pt;
//			fscanf(fp, "%f %f %f", &pt.x, &pt.y, &pt.z);
//			points.push_back(pt);
//
//		}
//		//fa = fa / static_cast<float>(pointNum);
//		m_lines[i].CreateFieldLine(points);
//		//m_lines[i].Simplification(0.95f);
////		m_lines[i].CalculateCurvatureAndTorsion();
////		m_lines[i].m_trace = 1.0f;
////		m_lines[i].m_FA = fa;
//		//m_lines[i].m_c[C_L] = 1.0f;
//		//m_lines[i].m_c[C_P] = 1.0f;
//		//m_lines[i].m_c[C_S] = 1.0f;
//	}
//	fclose(fp);
//
//	//CreateFlags(NULL);
//	SetMinMaxLength();
//	//SetFA();
//	//SetCurvature();
//	SetCenter();
//
//	//m_classNum = 0;
//
//	InitILRender();
//	CreateVertexBuffer();
//	CreateColorBuffer();
//
//	return m_lineNum;
//}


//void CLineProcessor::SaveHHR(const char *filename)
//{
//	string s = filename;
//	size_t pos = s.find_last_of(".");
//	if (pos == string::npos) {
//		s.append(".hhr");
//	} else {
//		s = s.substr(0, pos);
//		s.append(".hhr");
//	}
//	FILE *fp = fopen(s.c_str(), "w");
//	if (fp == NULL)
//		return;
//
//	int lineNum = 0;
//	for (int i=0; i<m_lineNum; i++) {
//		if (m_showFlags[i] == true && m_availableFlags[i] == true)
//			lineNum++;
//	}
//
//	fprintf(fp, "%d\n", lineNum);
//	for (int i=0; i<m_lineNum; i++) {
//		if (m_showFlags[i] == true && m_availableFlags[i] == true) {
//			int pointNum = m_lines[i].m_pointNum;
//			fprintf(fp, "%d\n", pointNum);
////			fprintf(fp, "%f\n", m_lines[i].m_FA);
//			for (int j=0; j<pointNum; j++) {
//				fprintf(fp, "%f %f %f\n", m_lines[i].m_points[j].x, 
//					m_lines[i].m_points[j].y, m_lines[i].m_points[j].z);
//			}
//		}
//	}
//	fclose(fp);
//}

//void CLineProcessor::OpenDistanceMatrix(const char *filename)
//{
//	//FILE *fp = fopen(filename, "r");
//	//if (fp == NULL)
//	//	return;
//	//char buffer[256];
//	//fscanf(fp, "%s\n", buffer);
//	//if (buffer[0] != 'D' || buffer[1] != 'M')
//	//	return;
//	//int num = 0;
//	//fscanf(fp, "%d\n", &num);
//	//if (num != m_lineNum)
//	//	return;
//
//	////SAFE_DELETE(m_DMSpace);
//	////SAFE_DELETE(m_DMCurvature);
//	////SAFE_DELETE(m_DMTorsion);
//	////m_DMSpace = new float[m_lineNum*m_lineNum];
//	////m_DMCurvature = new float[m_lineNum*m_lineNum];
//	////m_DMTorsion = new float[m_lineNum*m_lineNum];
//
//	//fscanf(fp, "%s\n", buffer);
//	//if (buffer[0] != 'S')
//	//	return;
//	//for (int i=0; i<m_lineNum; i++) {
//	//	for (int j=0; j<m_lineNum; j++) {
//	//		float dis;
//	//		fscanf(fp, "%f\t", &dis);
//	//		int offset = i * m_lineNum + j;
//	//		//m_DMSpace[offset] = dis;
//	//	}
//	//}
//	//fclose(fp);
//
//	////memset(m_DMCurvature, 0, sizeof(float)*m_lineNum*m_lineNum);
//	////memset(m_DMTorsion, 0, sizeof(float)*m_lineNum*m_lineNum);
//
//	////m_DMFlag = true;
//	////CalculateDistanceMatrixMean();
//}
//
//void CLineProcessor::SaveDistanceMatrix(const char *filename)
//{
//	//FILE *fp = fopen(filename, "w");
//	//if (fp == NULL)
//	//	return;
//
//	//fprintf(fp, "DM\n");
//	//fprintf(fp, "%d\n", m_lineNum);
//	//
//	//fprintf(fp, "SPACE\n");
//	//for (int i=0; i<m_lineNum; i++) {
//	//	for (int j=0; j<m_lineNum; j++) {
//	//		int offset = i * m_lineNum + j;
//	//		fprintf(fp, "%f\t", m_DMSpace[offset]);
//	//	}
//	//	fprintf(fp, "\n");
//	//}
//
//	//fclose(fp);
//}
//
//void CLineProcessor::OpenResult(const char *filename)
//{
//	//FILE *fp = fopen(filename, "r");
//	//if (fp == NULL)
//	//	return;
//	//char buffer[256];
//	//fscanf(fp, "%s\n", buffer);
//	//if (buffer[0] != 'R' || buffer[1] != 'E' || buffer[2] != 'S')
//	//	return;
//	//int num = 0;
//	//fscanf(fp, "%d\n", &num);
//	//if (num != m_lineNum)
//	//	return;
//	//fscanf(fp, "%d\n", &m_classNum);
//	//for (int i=0; i<m_lineNum; i++) {
//	//	int res = 0;
//	//	fscanf(fp, "%d\n", &res);
//	//	m_results[i] = res;
//	//}
//	//fclose(fp);
//}
//
//void CLineProcessor::SaveResult(const char *filename)
//{
//	//FILE *fp = fopen(filename, "w");
//	//if (fp == NULL)
//	//	return;
//
//	//fprintf(fp, "RES\n");
//	//fprintf(fp, "%d\n", m_lineNum);
//	//fprintf(fp, "%d\n", m_classNum);
//	//for (int i=0; i<m_lineNum; i++) {
//	//	fprintf(fp, "%d\n", m_results[i]);
//	//}
//
//	//fclose(fp);
//}
//

//
//void CLineProcessor::OpenMDS(const char *filename)
//{
//	//FILE *fp = fopen(filename, "r");
//	//if (fp == NULL)
//	//	return;
//
//	//char buffer[256];
//	//fscanf(fp, "%s\n", buffer);
//	//if (buffer[0] != 'M' || buffer[1] != 'D' || buffer[2] != 'S') {
//	//	fclose(fp);
//	//	return;
//	//}
//	//int num = 0;
//	//fscanf(fp, "%d\n", &num);
//	//if (num != m_lineNum) {
//	//	fclose(fp);
//	//	return;
//	//}
//
//	//m_pointNum = m_lineNum;
//	//SAFE_DELETE(m_points);
//	//m_points = new Point2F[m_pointNum];
//
//	//// read the 2D part
//	//for (int i=0; i<m_pointNum; i++) {
//	//	float x = 0.0f;
//	//	float y = 0.0f;
//	//	fscanf(fp, "%f\t%f\n", &x, &y);
//	//	m_points[i].x = x;
//	//	m_points[i].y = y;
//
//	//}
//
//	//SAFE_DELETE(m_configuration);
//	//m_configuration = new float[m_pointNum*3];
//	//srand((unsigned)time(NULL));
//	//for (int i=0; i<m_pointNum; i++) {
//	//	m_configuration[i*3+0] = m_points[i].x;
//	//	m_configuration[i*3+1] = m_points[i].y;
//	//	m_configuration[i*3+2] = (float)rand() / (float)RAND_MAX;
//	//}
//	//m_configurationFlag = true;
//}
//
//void CLineProcessor::SaveMDS(const char *filename)
//{
//	//FILE *fp = fopen(filename, "w");
//	//if (fp == NULL)
//	//	return;
//
//	//fprintf(fp, "MDS\n");
//	//fprintf(fp, "%d\n", m_pointNum);
//	//
//	//// save 2D mds
//	//for (int i=0; i<m_pointNum; i++) {
//	//	float x = m_points[i].x;
//	//	float y = m_points[i].y;
//	//	fprintf(fp, "%f\t%f\n", x, y);
//	//}
//
//	//fclose(fp);
//}

void CLineProcessor::OpenConfig(const char *filename)
{
	ClearAllFilterInfo();

	FILE *fp = fopen(filename, "r");
	if (fp == NULL) {
		return;
	}

	fscanf(fp, "%d\t%d\n", &m_originalLineNum, &m_originalPointNum);
	//fscanf(fp, "%d\t%d\n", &m_abstractLineNum, &m_abstractPointNum);

	fscanf(fp, "%f\t%f\t%f\n", &m_center.x, &m_center.y, &m_center.z);
	fscanf(fp, "%f\n", &m_scale);

	string path = filename;
	size_t pos = path.find_last_of("\\");
	path = path.substr(0, pos+1);

	char buffer[256];
	string pathname;

	// open original lines
	fscanf(fp, "%s\n", buffer);
	pathname = path + buffer;
	OpenOriginalLines(pathname.c_str());
	//// open original MDS
	//fscanf(fp, "%s\n", buffer);
	//pathname = path + buffer;
	////OpenOriginalMDS(pathname.c_str());
	//// open original distance matrix
	//fscanf(fp, "%s\n", buffer);
	//pathname = path + buffer;
	////OpenOriginalDistanceMatrix(pathname.c_str());

	//// open abstract lines
	//fscanf(fp, "%s\n", buffer);
	//pathname = path + buffer;
	//OpenAbstractLines(pathname.c_str());
	//// open abstract MDS
	//fscanf(fp, "%s\n", buffer);
	//pathname = path + buffer;
	////OpenAbstractMDS(pathname.c_str());
	// //open abstract distance matrix
	//fscanf(fp, "%s\n", buffer);
	//pathname = path + buffer;
	////OpenAbstractDistanceMatrix(pathname.c_str());

	//// open abstract results
	//fscanf(fp, "%s\n", buffer);
	////pathname = path + buffer;
	////OpenAbstractResults(pathname.c_str());
	//// open results
	//fscanf(fp, "%s\n", buffer);
	////pathname = path + buffer;
	////OpenResult(pathname.c_str());
	////
	fclose(fp);

	//SAFE_DELETE(m_results);
	//m_results = new int[m_originalLineNum];
	//for (int i=0; i<m_originalLineNum; i++) 
	//	m_results[i] = -1;

	//m_classNum = 0;

	InitILRender();

	//m_abstractFlag = true;
	//m_DMFlag = true;
	//m_lineLevel = LEVEL_ORIGINAL;
	LoadOriginalFieldLines();


}
//
//void CLineProcessor::OpenProtoConfig(const char *filename)
//{
//	ClearAllFilterInfo();
//
//	FILE *fp = fopen(filename, "r");
//	if (fp == NULL) {
//		return;
//	}
//
//	fscanf(fp, "%d\t%d\n", &m_originalLineNum, &m_originalPointNum);
//	fscanf(fp, "%d\t%d\n", &m_abstractLineNum, &m_abstractPointNum);
//
//	fscanf(fp, "%f\t%f\t%f\n", &m_center.x, &m_center.y, &m_center.z);
//	fscanf(fp, "%f\n", &m_scale);
//
//	string path = filename;
//	size_t pos = path.find_last_of("\\");
//	path = path.substr(0, pos+1);
//
//	char buffer[256];
//	string pathname;
//
//	// open original lines
//	fscanf(fp, "%s\n", buffer);
//	pathname = path + buffer;
//	OpenOriginalLines(pathname.c_str());
//	//// open original MDS
//	//fscanf(fp, "%s\n", buffer);
//	//pathname = path + buffer;
//	//OpenOriginalMDS(pathname.c_str());
//	//SAFE_DELETE(m_originalPoints2D);
//	// open original distance matrix
//	fscanf(fp, "%s\n", buffer);
//	pathname = path + buffer;
//	//OpenOriginalDistanceMatrix(pathname.c_str());
//	//SAFE_DELETE(m_originalDMSpace);
//	//SAFE_DELETE(m_originalDMCurvature);
//	//SAFE_DELETE(m_originalDMTorsion);
//
//	// open abstract lines
//	fscanf(fp, "%s\n", buffer);
//	pathname = path + buffer;
//	OpenAbstractLines(pathname.c_str());
//	//// open abstract MDS
//	//fscanf(fp, "%s\n", buffer);
//	//pathname = path + buffer;
//	//OpenAbstractMDS(pathname.c_str());
//	//SAFE_DELETE(m_abstractPoints2D);
//	// open abstract distance matrix
//	fscanf(fp, "%s\n", buffer);
//	pathname = path + buffer;
//	//OpenAbstractDistanceMatrix(pathname.c_str());
//	//SAFE_DELETE(m_abstractDMSpace);
//	//SAFE_DELETE(m_abstractDMCurvature);
//	//SAFE_DELETE(m_abstractDMTorsion);
//
//	// open abstract results
//	//fscanf(fp, "%s\n", buffer);
//	//pathname = path + buffer;
//	//OpenAbstractResults(pathname.c_str());
//	//SAFE_DELETE(m_abstractResults);
//	//// open results
//	//fscanf(fp, "%s\n", buffer);
//	//pathname = path + buffer;
//	//OpenResult(pathname.c_str());
//	//SAFE_DELETE(m_results);
//	
//	fclose(fp);
//
//	//SAFE_DELETE(m_results);
//	//m_results = new int[m_originalLineNum];
//	//for (int i=0; i<m_originalLineNum; i++) 
//	//	m_results[i] = -1;
//
//	//m_classNum = 0;
//
//	InitILRender();
//
//	//m_abstractFlag = true;
//	//m_DMFlag = true;
//	m_lineLevel = LEVEL_ABSTRACT;
//	ProtoChangeLevel();
//}

//
//void CLineProcessor::SaveConfig(const char *filename)
//{
//	//string str = filename;
//	//size_t pp = str.find_last_of(".");
//	//str = str.substr(0, pp);
//	//str = str + ".conf";
//	//FILE *fp = fopen(str.c_str(), "w");
//	//if (fp == NULL)
//	//	return;
//
//	//fprintf(fp, "%d\t%d\n", m_originalLineNum, m_originalPointNum);
//	//fprintf(fp, "%d\t%d\n", m_abstractLineNum, m_abstractPointNum);
//
//	//fprintf(fp, "%f\t%f\t%f\n", m_center.x, m_center.y, m_center.z);
//	//fprintf(fp, "%f\n", m_scale);
//
//	//string path = filename;
//	//size_t pos = path.find_last_of("\\");
//	//path = path.substr(0, pos+1);
//
//	//string name = filename;
//	//size_t pos1 = name.find_last_of("\\");
//	//size_t pos2 = name.find_last_of(".");
//	//size_t length = pos2 - pos1 - 1;
//	//name = name.substr(pos1+1, length);
//
//	//string pathname;
//	//string fn;
//	//// save abstract lines
//	//fn = name + "_original" + ".hhr";
//	//fprintf(fp, "%s\n", fn.c_str());
//	//pathname = path + fn;
//	//SaveOriginalLines(pathname.c_str());
//	//// save abstract MDS
//	//fn = name + "_original" + ".mds";
//	//fprintf(fp, "%s\n", fn.c_str());
//	//pathname = path + fn;
//	//SaveOriginalMDS(pathname.c_str());
//	//// save abstract distance matrix
//	//fn = name + "_original" + ".dm";
//	//fprintf(fp, "%s\n", fn.c_str());
//	//pathname = path + fn;
//	//SaveOriginalDistanceMatrix(pathname.c_str());
//
//	//// save abstract lines
//	//fn = name + "_abstract" + ".hhr";
//	//fprintf(fp, "%s\n", fn.c_str());
//	//pathname = path + fn;
//	//SaveAbstractLines(pathname.c_str());
//	//// save abstract MDS
//	//fn = name + "_abstract" + ".mds";
//	//fprintf(fp, "%s\n", fn.c_str());
//	//pathname = path + fn;
//	//SaveAbstractMDS(pathname.c_str());
//	//// save abstract distance matrix
//	//fn = name + "_abstract" + ".dm";
//	//fprintf(fp, "%s\n", fn.c_str());
//	//pathname = path + fn;
//	//SaveAbstractDistanceMatrix(pathname.c_str());
//
//	//// save abstract results
//	//fn = name + ".ar";
//	//fprintf(fp, "%s\n", fn.c_str());
//	//pathname = path + fn;
//	//SaveAbstractResults(pathname.c_str());
//	//
//	//fclose(fp);
//}
//
//void CLineProcessor::OpenAbstractResults(const char *filename)
//{
//	//FILE *fp = fopen(filename, "r");
//	//if (fp == NULL)
//	//	return;
//
//	//char buffer[256];
//	//fscanf(fp, "%s\n", buffer);
//	//if (buffer[0] != 'A' || buffer[1] != 'R') {
//	//	fclose(fp);
//	//	return;
//	//}
//	//int num = 0;
//	//fscanf(fp, "%d\n", &num);
//	//if (num != m_originalLineNum) {
//	//	fclose(fp);
//	//	return;
//	//}
//
//	//SAFE_DELETE(m_abstractResults);
//	//m_abstractResults = new int[m_originalLineNum];
//	//for (int i=0; i<m_originalLineNum; i++) {
//	//	int k = 0;
//	//	fscanf(fp, "%d", &k);
//	//	m_abstractResults[i] = k;
//	//}
//
//	//fclose(fp);
//}

//void CLineProcessor::SaveAbstractResults(const char *filename)
//{
//	FILE *fp = fopen(filename, "w");
//	if (fp == NULL)
//		return;
//
//	fprintf(fp, "AR\n");
//	fprintf(fp, "%d\n", m_originalLineNum);
//	for (int i=0; i<m_originalLineNum; i++) {
//		int k = m_abstractResults[i];
//		fprintf(fp, "%d\t", k);
//	}
//
//	fclose(fp);
//}
//
void CLineProcessor::OpenOriginalLines(const char *filename)
{
	FILE *fp = fopen(filename, "r");
	if (fp == NULL)
		return;

	// read the line number
	fscanf(fp, "%d\n", &m_originalLineNum);
	// read lines
	SAFE_DELETE(m_originalLines);
	m_originalLines = new CFieldLine[m_originalLineNum];
	for (int i=0; i<m_originalLineNum; i++) {
		int pointNum;
		fscanf(fp, "%d\n", &pointNum);
		vector<Point3F> points;
		float fa = 0.0f;
		fscanf(fp, "%f\n", &fa);
		float trace = 0.0;
		for (int j=0; j<pointNum; j++) {
			Point3F pt;
			fscanf(fp, "%f %f %f", &pt.x, &pt.y, &pt.z);
			points.push_back(pt);
		}
		m_originalLines[i].CreateFieldLine(points);
//		m_originalLines[i].CalculateCurvatureAndTorsion();
//		m_originalLines[i].m_trace = 1.0f;
//		m_originalLines[i].m_FA = fa;
		//m_originalLines[i].m_c[C_L] = 1.0f;
		//m_originalLines[i].m_c[C_P] = 1.0f;
		//m_originalLines[i].m_c[C_S] = 1.0f;
	}
	fclose(fp);
}

//void CLineProcessor::SaveOriginalLines(const char *filename)
//{
//	//FILE *fp = fopen(filename, "w");
//	//if (fp == NULL)
//	//	return;
//
//	//int lineNum = m_originalLineNum;
//	//fprintf(fp, "%d\n", lineNum);
//	//for (int i=0; i<m_originalLineNum; i++) {
//	//	int pointNum = m_originalLines[i].m_pointNum;
//	//	fprintf(fp, "%d\n", pointNum);
//	//	fprintf(fp, "%f\n", m_originalLines[i].m_FA);
//	//	for (int j=0; j<pointNum; j++) {
//	//		fprintf(fp, "%f %f %f\n", m_originalLines[i].m_points[j].x, 
//	//			m_originalLines[i].m_points[j].y, m_originalLines[i].m_points[j].z);
//	//	}
//	//}
//
//	//fclose(fp);
//}

//void CLineProcessor::OpenOriginalMDS(const char *filename)
//{
//	//FILE *fp = fopen(filename, "r");
//	//if (fp == NULL)
//	//	return;
//
//	//char buffer[256];
//	//fscanf(fp, "%s\n", buffer);
//	//if (buffer[0] != 'M' || buffer[1] != 'D' || buffer[2] != 'S') {
//	//	fclose(fp);
//	//	return;
//	//}
//
//	//m_originalPointNum = m_originalLineNum;
//	//int num = 0;
//	//fscanf(fp, "%d\n", &num);
//	//if (num != m_originalPointNum) {
//	//	fclose(fp);
//	//	return;
//	//}
//
//	//SAFE_DELETE(m_originalPoints2D);
//	//m_originalPoints2D = new Point2F[m_originalPointNum];
//
//	//// 2D MDS
//	//for (int i=0; i<m_originalPointNum; i++) {
//	//	float x = 0.0f;
//	//	float y = 0.0f;
//	//	fscanf(fp, "%f\t%f\n", &x, &y);
//	//	m_originalPoints2D[i].x = x;
//	//	m_originalPoints2D[i].y = y;
//	//}
//
//	//NormalizePoints(m_originalPoints2D, m_originalPointNum);
//}
//
//void CLineProcessor::SaveOriginalMDS(const char *filename)
//{
//	//FILE *fp = fopen(filename, "w");
//	//if (fp == NULL)
//	//	return;
//
//	//fprintf(fp, "MDS\n");
//	//fprintf(fp, "%d\n", m_originalPointNum);
//
//	//// 2D MDS
//	//for (int i=0; i<m_originalPointNum; i++) {
//	//	float x = m_originalPoints2D[i].x;
//	//	float y = m_originalPoints2D[i].y;
//	//	fprintf(fp, "%f\t%f\n", x, y);
//	//}
//
//	//fclose(fp);
//}

//void CLineProcessor::OpenOriginalDistanceMatrix(const char *filename)
//{
//	//FILE *fp = fopen(filename, "r");
//	//if (fp == NULL)
//	//	return;
//
//	//char buffer[256];
//	//fscanf(fp, "%s\n", buffer);
//	//if (buffer[0] != 'D' || buffer[1] != 'M') {
//	//	fclose(fp);
//	//	return;
//	//}
//	//int num = 0;
//	//fscanf(fp, "%d\n", &num);
//	//if (num != m_originalLineNum) {
//	//	fclose(fp);
//	//	return;
//	//}
//
//	//fscanf(fp, "%s\n", buffer);
//	//SAFE_DELETE(m_originalDMSpace);
//	//m_originalDMSpace = new float[m_originalLineNum*m_originalLineNum];
//	//for (int i=0; i<m_originalLineNum; i++) {
//	//	for (int j=0; j<m_originalLineNum; j++) {
//	//		float v;
//	//		fscanf(fp, "%f", &v);
//	//		m_originalDMSpace[i*m_originalLineNum+j] = v;
//	//	}
//	//}
//
//	//SAFE_DELETE(m_originalDMCurvature);
//	//m_originalDMCurvature = new float[m_originalLineNum*m_originalLineNum];
//	//memset(m_originalDMCurvature, 0, sizeof(float)*m_originalLineNum*m_originalLineNum);
//	//SAFE_DELETE(m_originalDMTorsion);
//	//m_originalDMTorsion = new float[m_originalLineNum*m_originalLineNum];
//	//memset(m_originalDMTorsion, 0, sizeof(float)*m_originalLineNum*m_originalLineNum);
//
//	////fscanf(fp, "%s\n", buffer);
//	////SAFE_DELETE(m_originalDMCurvature);
//	////m_originalDMCurvature = new float[m_originalLineNum*m_originalLineNum];
//	////for (int i=0; i<m_originalLineNum; i++) {
//	////	for (int j=0; j<m_originalLineNum; j++) {
//	////		float v;
//	////		fscanf(fp, "%f", &v);
//	////		m_originalDMCurvature[i*m_originalLineNum+j] = v;
//	////	}
//	////}
//
//	////fscanf(fp, "%s\n", buffer);
//	////SAFE_DELETE(m_originalDMTorsion);
//	////m_originalDMTorsion = new float[m_originalLineNum*m_originalLineNum];
//	////for (int i=0; i<m_originalLineNum; i++) {
//	////	for (int j=0; j<m_originalLineNum; j++) {
//	////		float v;
//	////		fscanf(fp, "%f", &v);
//	////		m_originalDMTorsion[i*m_originalLineNum+j] = v;
//	////	}
//	////}
//
//	//fclose(fp);
//}
//
//void CLineProcessor::SaveOriginalDistanceMatrix(const char *filename)
//{
//	//FILE *fp = fopen(filename, "w");
//	//if (fp == NULL)
//	//	return;
//
//	//fprintf(fp, "DM\n");
//	//fprintf(fp, "%d\n", m_originalLineNum);
//
//	//fprintf(fp, "SPACE\n");
//	//for (int i=0; i<m_originalLineNum; i++) {
//	//	for (int j=0; j<m_originalLineNum; j++) {
//	//		float v = m_originalDMSpace[i*m_originalLineNum+j];
//	//		fprintf(fp, "%f\t", v);
//	//	}
//	//	fprintf(fp, "\n");
//	//}
//	////fprintf(fp, "Cur\n");
//	////for (int i=0; i<m_originalLineNum; i++) {
//	////	for (int j=0; j<m_originalLineNum; j++) {
//	////		float v = m_originalDMCurvature[i*m_originalLineNum+j];
//	////		fprintf(fp, "%f\t", v);
//	////	}
//	////	fprintf(fp, "\n");
//	////}
//	////fprintf(fp, "Tor\n");
//	////for (int i=0; i<m_originalLineNum; i++) {
//	////	for (int j=0; j<m_originalLineNum; j++) {
//	////		float v = m_originalDMTorsion[i*m_originalLineNum+j];
//	////		fprintf(fp, "%f\t", v);
//	////	}
//	////	fprintf(fp, "\n");
//	////}
//	//fclose(fp);
//}
//
//void CLineProcessor::OpenAbstractLines(const char *filename)
//{
//	FILE *fp = fopen(filename, "r");
//	if (fp == NULL)
//		return;
//
//	// read the line number
//	fscanf(fp, "%d\n", &m_abstractLineNum);
//	// read lines
//	SAFE_DELETE(m_abstractLines);
//	m_abstractLines = new CFieldLine[m_abstractLineNum];
//	for (int i=0; i<m_abstractLineNum; i++) {
//		int pointNum;
//		fscanf(fp, "%d\n", &pointNum);
//		vector<Point3F> points;
//		float fa = 0.0f;
//		fscanf(fp, "%f\n", &fa);
//		float trace = 0.0;
//		for (int j=0; j<pointNum; j++) {
//			Point3F pt;
//			fscanf(fp, "%f %f %f", &pt.x, &pt.y, &pt.z);
//			points.push_back(pt);
//		}
//		m_abstractLines[i].CreateFieldLine(points);
////		m_abstractLines[i].CalculateCurvatureAndTorsion();
////		m_abstractLines[i].m_trace = 1.0f;
////		m_abstractLines[i].m_FA = fa;
//		//m_abstractLines[i].m_c[C_L] = 1.0f;
//		//m_abstractLines[i].m_c[C_P] = 1.0f;
//		//m_abstractLines[i].m_c[C_S] = 1.0f;
//	}
//	fclose(fp);
//}
////
//void CLineProcessor::SaveAbstractLines(const char *filename)
//{
////	FILE *fp = fopen(filename, "w");
////	if (fp == NULL)
////		return;
////
////	int lineNum = m_abstractLineNum;
////	fprintf(fp, "%d\n", lineNum);
////	for (int i=0; i<m_abstractLineNum; i++) {
////		int pointNum = m_abstractLines[i].m_pointNum;
////		fprintf(fp, "%d\n", pointNum);
//////		fprintf(fp, "%f\n", m_abstractLines[i].m_FA);
////		for (int j=0; j<pointNum; j++) {
////			fprintf(fp, "%f %f %f\n", m_abstractLines[i].m_points[j].x, 
////				m_abstractLines[i].m_points[j].y, m_abstractLines[i].m_points[j].z);
////		}
////	}
////
////	fclose(fp);
//}
//
//void CLineProcessor::OpenAbstractMDS(const char *filename)
//{
////	FILE *fp = fopen(filename, "r");
////	if (fp == NULL)
////		return;
////
////	char buffer[256];
////	fscanf(fp, "%s\n", buffer);
////	if (buffer[0] != 'M' || buffer[1] != 'D' || buffer[2] != 'S') {
////		fclose(fp);
////		return;
////	}
////
////	m_abstractPointNum = m_abstractLineNum;
////	int num = 0;
////	fscanf(fp, "%d\n", &num);
////	if (num != m_abstractPointNum) {
////		fclose(fp);
////		return;
////	}
////
////	SAFE_DELETE(m_abstractPoints2D);
////	m_abstractPoints2D = new Point2F[m_abstractPointNum];
////
////	// 2D MDS
////	for (int i=0; i<m_abstractPointNum; i++) {
////		float x = 0.0f;
////		float y = 0.0f;
////		fscanf(fp, "%f\t%f\n", &x, &y);
////		m_abstractPoints2D[i].x = x;
////		m_abstractPoints2D[i].y = y;
////	}
////	fclose(fp);
////
////	NormalizePoints(m_abstractPoints2D, m_abstractPointNum);
////}
////
////void CLineProcessor::SaveAbstractMDS(const char *filename)
////{
////	FILE *fp = fopen(filename, "w");
////	if (fp == NULL)
////		return;
////
////	fprintf(fp, "MDS\n");
////	fprintf(fp, "%d\n", m_abstractPointNum);
////
////	// 2D MDS
////	for (int i=0; i<m_abstractPointNum; i++) {
////		float x = m_abstractPoints2D[i].x;
////		float y = m_abstractPoints2D[i].y;
////		fprintf(fp, "%f\t%f\n", x, y);
////	}
////
////	fclose(fp);
//}
//
//void CLineProcessor::OpenAbstractDistanceMatrix(const char *filename)
//{
//	//FILE *fp = fopen(filename, "r");
//	//if (fp == NULL)
//	//	return;
//
//	//char buffer[256];
//	//fscanf(fp, "%s\n", buffer);
//	//if (buffer[0] != 'D' || buffer[1] != 'M') {
//	//	fclose(fp);
//	//	return;
//	//}
//	//int num = 0;
//	//fscanf(fp, "%d\n", &num);
//	//if (num != m_abstractLineNum) {
//	//	fclose(fp);
//	//	return;
//	//}
//
//	//fscanf(fp, "%s\n", buffer);
//	//SAFE_DELETE(m_abstractDMSpace);
//	//m_abstractDMSpace = new float[m_abstractLineNum*m_abstractLineNum];
//	//for (int i=0; i<m_abstractLineNum; i++) {
//	//	for (int j=0; j<m_abstractLineNum; j++) {
//	//		float v;
//	//		fscanf(fp, "%f", &v);
//	//		m_abstractDMSpace[i*m_abstractLineNum+j] = v;
//	//	}
//	//}
//
//
//	//fscanf(fp, "%s\n", buffer);
//	//SAFE_DELETE(m_abstractDMCurvature);
//	//m_abstractDMCurvature = new float[m_abstractLineNum*m_abstractLineNum];
//	//for (int i=0; i<m_abstractLineNum; i++) {
//	//	for (int j=0; j<m_abstractLineNum; j++) {
//	//		float v;
//	//		fscanf(fp, "%f", &v);
//	//		m_abstractDMCurvature[i*m_abstractLineNum+j] = v;
//	//	}
//	//}
//
//	//fscanf(fp, "%s\n", buffer);
//	//SAFE_DELETE(m_abstractDMTorsion);
//	//m_abstractDMTorsion = new float[m_abstractLineNum*m_abstractLineNum];
//	//for (int i=0; i<m_abstractLineNum; i++) {
//	//	for (int j=0; j<m_abstractLineNum; j++) {
//	//		float v;
//	//		fscanf(fp, "%f", &v);
//	//		m_abstractDMTorsion[i*m_abstractLineNum+j] = v;
//	//	}
//	//}
//
//	//fclose(fp);
//}
//
//void CLineProcessor::SaveAbstractDistanceMatrix(const char *filename)
//{
//	//FILE *fp = fopen(filename, "w");
//	//if (fp == NULL)
//	//	return;
//
//	//fprintf(fp, "DM\n");
//	//fprintf(fp, "%d\n", m_abstractLineNum);
//
//	//fprintf(fp, "SPACE\n");
//	//for (int i=0; i<m_abstractLineNum; i++) {
//	//	for (int j=0; j<m_abstractLineNum; j++) {
//	//		float v = m_abstractDMSpace[i*m_abstractLineNum+j];
//	//		fprintf(fp, "%f\t", v);
//	//	}
//	//	fprintf(fp, "\n");
//	//}
//	//fprintf(fp, "Cur\n");
//	//for (int i=0; i<m_abstractLineNum; i++) {
//	//	for (int j=0; j<m_abstractLineNum; j++) {
//	//		float v = m_abstractDMCurvature[i*m_abstractLineNum+j];
//	//		fprintf(fp, "%f\t", v);
//	//	}
//	//	fprintf(fp, "\n");
//	//}
//	//fprintf(fp, "Tor\n");
//	//for (int i=0; i<m_abstractLineNum; i++) {
//	//	for (int j=0; j<m_abstractLineNum; j++) {
//	//		float v = m_abstractDMTorsion[i*m_abstractLineNum+j];
//	//		fprintf(fp, "%f\t", v);
//	//	}
//	//	fprintf(fp, "\n");
//	//}
//	//fclose(fp);
//}
//
//void CLineProcessor::GetCurrentClass(CLASS *CurrentClass,const int Classify_id)
//{
//	int num = 0;
//	if (m_lineNum > 0)
//	{
//		
//		int index = 0;
//		for (int i=0; i<m_lineNum; i++) 
//		{
//			int id = m_results[i];
//			if(id == Classify_id)
//			{
//				Color3F c;
//				if (id == -1)
//				{
//					c.r = c.g = c.b = 1.0f;
//				}
//				else
//				{
//					c = g_colorMap.GetClusterColor(id);
//				}
//				CurrentClass->color = c;
//				CurrentClass->lines.push_back(i);num++;
//			}
//		}
//	}
//}
void CLineProcessor::DrawDTI()
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	glEnable(GL_DEPTH_TEST);

	glScalef(m_scale, m_scale, m_scale);
	glTranslatef(-m_center.x, -m_center.y, -m_center.z);

	if (m_lineNum > 0)
	{
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, m_vertices);

		//if (colorFlag == true)
		//{
		//	//glDisableClientState(GL_COLOR_ARRAY);
		//} 
		//else
		//{
			int index = 0;
			for (int i=0; i<m_lineNum; i++) 
			{
				//if (colorPres == true)
				//{
				//	//float alpha = 1.0f;
				//	//if (m_selectFlags[i] == true)
				//	//	alpha = 1.0f;
				//	//else 
				//	//	alpha = 1.0f;
				//	//if (m_availableFlags[i] != true || m_showFlags[i] != true)
				//	//	alpha = 0.0f;
				//	//for (int j=0; j<m_vertCount[i]; j++)
				//	//{
				//	//	Color3F c;
				//	//	c.r = (m_lines[i].m_points[j].x - m_min.x) / (m_max.x - m_min.x);
				//	//	c.g = (m_lines[i].m_points[j].y - m_min.y) / (m_max.y - m_min.y);
				//	//	c.b = (m_lines[i].m_points[j].z - m_min.z) / (m_max.z - m_min.z);
				//	//	m_colors[index*4+0] = c.r;
				//	//	m_colors[index*4+1] = c.g;
				//	//	m_colors[index*4+2] = c.b;
				//	//	m_colors[index*4+3] = alpha;
				//	//	index++;
				//	//}
				//}
				//else 
				//{
					//int id = m_results[i];
					Color3F c;
					//if (id == -1)
					//{
					//	c.r = c.g = c.b = 1.0f;
					//}
					//else
					//{
					//	c = g_colorMap.GetClusterColor(id);
					//}
					c.r = c.g = c.b = 0.4f;
					float alpha = 1.0f;
					//if (m_selectFlags[i] == true)
					//	alpha = 1.0f;
					//else 
					//	alpha = 1.0f;
					//if (m_availableFlags[i] != true || m_showFlags[i] != true)
					//	alpha = 0.0f;
					if (FILTERED_FLAG) alpha = 0.2f;
					for (int j=0; j<m_vertCount[i]; j++)
					{
						m_colors[index*4+0] = c.r;//1.0f;//c.r;
						m_colors[index*4+1] = c.g;//1.0f;//c.g;
						m_colors[index*4+2] = c.b;//1.0f;//c.b;
						m_colors[index*4+3] = alpha;
						index++;
					}
				//}
			}
			glColorPointer(4, GL_FLOAT, sizeof(float)*4, m_colors);
		//}

		glDepthMask(GL_FALSE);
		m_render.multiDrawArrays(m_first, m_vertCount, m_lineNum);
		glDepthMask(GL_TRUE);
	//}
	
	//if (m_principalCurves != NULL) {
		/*for (int i=0; i<m_classNum+1; i++) {
			int pointNum = m_principalCurves[i].m_pointNum;
			if (pointNum > 0) {
				int first[1];
				first[0] = 0;
				int vertCount[1];
				vertCount[0] = pointNum;
				float *vertices = new float[pointNum*3];
				for (int j=0; j<pointNum; j++) {
					vertices[j*3+0] = m_principalCurves[i].m_points[j].x;
					vertices[j*3+1] = m_principalCurves[i].m_points[j].y;
					vertices[j*3+2] = m_principalCurves[i].m_points[j].z;
				}
				glVertexPointer(3, GL_FLOAT, 0, vertices);

				glLineWidth(2.0f);
				float *color = new float[pointNum*4];
				Color3F c = g_colorMap.GetClusterColor(i);
				for (int j=0; j<pointNum; j++) {
					color[j*4+0] = c.r; color[j*4+1] = c.g; 
					color[j*4+2] = c.b; color[j*4+3] = 1.0;
				}
				glColorPointer(4, GL_FLOAT, 4*sizeof(GLfloat), color);
				
				glDepthMask(GL_FALSE);
				m_render.multiDrawArrays(first, vertCount, 1);
				glDepthMask(GL_TRUE);

				SAFE_DELETE(vertices);
				SAFE_DELETE(color);
			}
		}*/
	}
	glPopMatrix();
}

//void CLineProcessor::DrawMDS(POINT_SIZE pointSize, const float alpha)
//{
	//for (int i=0; i<m_pointNum; i++) {
	//	if (m_availableFlags[i] != true || m_showFlags[i] != true)
	//		continue;
	//	// select the point color
	//	int id = m_results[i];
	//	if (id == -1) {
	//		if (!m_selectFlags[i]) {
	//			glColor4f(1.0f, 1.0f, 1.0f, 0.3f * alpha);
	//		} else {
	//			glColor4f(1.0f, 1.0f, 1.0f, 1.0f * alpha);
	//		}
	//	} else {
	//		if (!m_selectFlags[i]) {
	//			g_colorMap.SetOpenGLClusterColor(0.3f * alpha, id);
	//		} else {
	//			g_colorMap.SetOpenGLClusterColor(1.0f * alpha, id);
	//		}
	//	}

	//	//float ff = m_lines[i].m_FA;
	//	//float color = (ff - m_minFA) / (m_maxFA - m_minFA);
	//	//float r = 1.0f;
	//	//float g = 1.0f - color;
	//	//float b = color;
	//	//glColor4f(r, g, b, 1.0f);

	//	float size = 6.0f;
	//	if (pointSize == POINTSIZE_NONE) 
	//	{
	//		size = 6.0f;
	//	} 
	//	else if (pointSize == POINTSIZE_LENGTH) 
	//	{
	//		float length = m_lines[i].m_length;
	//		size = (length - m_minLength) / (m_maxLength - m_minLength);
	//		size = size * 8.0f + 2.0f;
	//	} 
	//	else if (pointSize == POINTSIZE_FA) 
	//	{
	//		float fa = m_lines[i].m_FA;
	//		size = (fa - m_minFA) / (m_maxFA - m_minFA);
	//		size = size * 8.0f + 2.0f;
	//	} 
	//	else if (pointSize == POINTSIZE_CURVATURE) 
	//	{
	//		float curvature = m_lines[i].m_curvature;
	//		size = (curvature - m_minCur) / (m_maxCur - m_minCur);
	//		size = size * 8.0f + 2.0f;
	//	} 
	//	else 
	//	{
	//		size = 6.0f;
	//	}
	//	//size = size * 1.5f;
	//	glPointSize(size);

	//	glBegin(GL_POINTS);
	//	glVertex2f(m_points[i].x, m_points[i].y);
	//	glEnd();
	//}

	//if (m_principalCurves != NULL) {
	//	for (int i=0; i<=m_classNum; i++) {
	//		int pointNum = m_principalCurves[i].m_pointNum;
	//		if (pointNum > 0) {
	//			glPointSize(6.0f);
	//			g_colorMap.SetOpenGLClusterColor(1.0f, i);
	//			glBegin(GL_POINTS);
	//			glVertex2f(m_principalPoints[i].x, m_principalPoints[i].y);
	//			glEnd();
	//		}
	//	}
	//}
//}

//void CLineProcessor::DrawDTIEllipsoids()
//{
	//// set light
	//GLfloat light_pos[] = {-10.0f, -10.0f, -10.0f, 0.0f};
	//GLfloat light_diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
	//GLfloat light_specular[] = {1.0f, 1.0f, 1.0f, 1.0f};
	//glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
	//glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	//glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

	//GLfloat mat_diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
	//GLfloat mat_specular[] = {1.0f, 1.0f, 1.0f, 1.0f};
	//GLfloat mat_shininess[] = {50.0f};
	//glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);

	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);
	//glEnable(GL_DEPTH_TEST);
	//glDisable(GL_BLEND);
	//glDisable(GL_ALPHA_TEST);
	//glEnable(GL_NORMALIZE);

	//glMatrixMode(GL_MODELVIEW);
	//glPushMatrix();

	//glScalef(m_scale, m_scale, m_scale);
	//glTranslatef(-m_center.x, -m_center.y, -m_center.z);

	//for (int i=0; i<m_lineNum; i++) {
	//	if (m_availableFlags[i] == true && m_showFlags[i] == true) {
	//		int id = m_results[i];
	//		Color3F c;
	//		if (id == -1) {
	//			c.r = c.g = c.b = 1.0f;
	//		} else {
	//			c = g_colorMap.GetClusterColor(id);
	//		}
	//		glColor4f(c.r, c.g, c.b, 1.0f);
	//		m_lines[i].DrawEllipsoids();
	//	}
	//}

	//glPopMatrix();

	//glDisable(GL_LIGHTING);
	//glDisable(GL_LIGHT0);
	//glDisable(GL_DEPTH_TEST);
	//glEnable(GL_BLEND);
	//glEnable(GL_ALPHA_TEST);
//}

//Chen============
void CLineProcessor::ClearOriginalCurve(int type)
{
	if (type==1)
	{
		lmTipXPos.clear(); lmTipYPos.clear();  lmTipZPos.clear(); lmCurveLength = 0;
	}
	else if (type==2)
	{
		file3DXPos.clear(); file3DYPos.clear();  file3DZPos.clear(); file3DCurveLength = 0;
	}
	else if (type==3)
	{
		file2DXPos.clear(); file2DYPos.clear(); file2DCurveLength = 0;
	}
	else if (type==4)
	{
		sketchXPos.clear(); sketchYPos.clear(); sketchCurveLength = 0;
	}
}

void CLineProcessor::ClearTSCurve(int type)
{
	if (type==1)
	{
		t_s_lmTipXPos.clear(); t_s_lmTipYPos.clear();  t_s_lmTipZPos.clear();
	}
	else if (type==2)
	{
		t_s_file3DXPos.clear(); t_s_file3DYPos.clear();  t_s_file3DZPos.clear();
	}
	else if (type==3)
	{
		t_s_file2DXPos.clear(); t_s_file2DYPos.clear(); t_s_file2DZPos.clear();
	}
	else if (type==4)
	{
		t_s_sketchXPos.clear(); t_s_sketchYPos.clear(); t_s_sketchZPos.clear();
	}
}

void CLineProcessor::ClearCurvatureBinSeq()
{
	CurvatureBinSeq.clear(); MAX_CUR = 0; MIN_CUR = 0;
}

void CLineProcessor::ClearAngleBinSeq()
{
	AngleBinSeq.clear();
}

void CLineProcessor::ClearAngleInfo(int type)
{
	if (type==1)
	{
		t_s_lmCurveAngles.clear(); t_s_lmCurveAngleSeq.clear();
	}
	else if (type==2)
	{
		t_s_file3DCurveAngles.clear(); t_s_file3DCurveAngleSeq.clear();
	}
	else if (type==3)
	{
		t_s_file2DCurveAngles.clear(); t_s_file2DCurveAngleSeq.clear();
	}
	else if (type==4)
	{
		t_s_sketchCurveAngles.clear(); t_s_sketchCurveAngleSeq.clear();
	}
	else if (type==5)
	{
		DTICurveAngles.clear(); DTICurvesAngleSeq.clear();
	}
}

void CLineProcessor::ClearCurvatureInfo(int type)
{
	n1 = 1; n2 = 1;
	if (type==1)
	{
		t_s_lmXFirstOrderDiscreteDerivative.clear();
		t_s_lmYFirstOrderDiscreteDerivative.clear();
		t_s_lmZFirstOrderDiscreteDerivative.clear();

		t_s_lmXSecondOrderDiscreteDerivative.clear();
		t_s_lmYSecondOrderDiscreteDerivative.clear();
		t_s_lmZSecondOrderDiscreteDerivative.clear();

		t_s_lmCurveS.clear();

		t_s_lmCurveCurvature.clear();
		t_s_lmCurveCurvatureSeq.clear();
	}
	else if (type==2)
	{
		t_s_file3DXFirstOrderDiscreteDerivative.clear();
		t_s_file3DYFirstOrderDiscreteDerivative.clear();
		t_s_file3DZFirstOrderDiscreteDerivative.clear();

		t_s_file3DXSecondOrderDiscreteDerivative.clear();
		t_s_file3DYSecondOrderDiscreteDerivative.clear();
		t_s_file3DZSecondOrderDiscreteDerivative.clear();

		t_s_file3DCurveS.clear();

		t_s_file3DCurveCurvature.clear();
		t_s_file3DCurveCurvatureSeq.clear();
	}
	else if (type==3)
	{
		t_s_file2DXFirstOrderDiscreteDerivative.clear();
		t_s_file2DYFirstOrderDiscreteDerivative.clear();
		t_s_file2DZFirstOrderDiscreteDerivative.clear();

		t_s_file2DXSecondOrderDiscreteDerivative.clear();
		t_s_file2DYSecondOrderDiscreteDerivative.clear();
		t_s_file2DZSecondOrderDiscreteDerivative.clear();

		t_s_file2DCurveS.clear();

		t_s_file2DCurveCurvature.clear();
		t_s_file2DCurveCurvatureSeq.clear();
	}
	else if (type==4)
	{
		t_s_sketchXFirstOrderDiscreteDerivative.clear();
		t_s_sketchYFirstOrderDiscreteDerivative.clear();
		t_s_sketchZFirstOrderDiscreteDerivative.clear();

		t_s_sketchXSecondOrderDiscreteDerivative.clear();
		t_s_sketchYSecondOrderDiscreteDerivative.clear();
		t_s_sketchZSecondOrderDiscreteDerivative.clear();

		t_s_sketchCurveS.clear();

		t_s_sketchCurveCurvature.clear();
		t_s_sketchCurveCurvatureSeq.clear();
	}
	else if (type==5)
	{
		DTIXFirstOrderDiscreteDerivative.clear();
		DTIYFirstOrderDiscreteDerivative.clear();
		DTIZFirstOrderDiscreteDerivative.clear();

		DTIXSecondOrderDiscreteDerivative.clear();
		DTIYSecondOrderDiscreteDerivative.clear();
		DTIZSecondOrderDiscreteDerivative.clear();

		DTICurveSs.clear();

		DTICurvesCurvature.clear();
		DTICurvesCurvatureSeq.clear();
	}
}

void CLineProcessor::ClearDTISimilarityInfo()
{
	aSimilarityForDTICurves.clear();
	cSimilarityForDTICurves.clear();
}

void CLineProcessor::ClearFilteredDTICurves()
{
	SAFE_DELETE(ts_m_lines);
	SAFE_DELETE(ts_m_vertices);

	SAFE_DELETE(f_m_vertices);
	SAFE_DELETE(f_m_first);
	SAFE_DELETE(f_m_vertCount);
	f_m_lineNum = 0;
	FILTERED_FLAG = FALSE;
}

void CLineProcessor::ClearAllFilterInfo()
{
	ClearOriginalCurve(1);	ClearOriginalCurve(2);	ClearOriginalCurve(3);	ClearOriginalCurve(4);
	ClearTSCurve(1);	ClearTSCurve(2);	ClearTSCurve(3);	ClearTSCurve(4);
	ClearCurvatureBinSeq();
	ClearAngleInfo(1);	ClearAngleInfo(2);	ClearAngleInfo(3);	ClearAngleInfo(4);	ClearAngleInfo(5);
	ClearCurvatureInfo(1);	ClearCurvatureInfo(2);	ClearCurvatureInfo(3);	ClearCurvatureInfo(4);	ClearCurvatureInfo(5);
	ClearDTISimilarityInfo();
	ClearFilteredDTICurves();
}

void CLineProcessor::UpdateLMCurve(float XPos,float YPos,float ZPos)
{ 
	while(lmTipXPos.size()>MAXCURVELENGTH) 
	{
		lmTipXPos.pop_front();
		lmTipYPos.pop_front();
		lmTipZPos.pop_front();
	}
	lmTipXPos.push_back(XPos);
	lmTipYPos.push_back(YPos);
	lmTipZPos.push_back(ZPos);	
}

void CLineProcessor::UpdateSketchCurve(float XPos, float YPos)
{
	while (sketchXPos.size()>MAXCURVELENGTH)
	{
		sketchXPos.pop_front(); sketchYPos.pop_front();
	}
	sketchXPos.push_back(XPos);
	sketchYPos.push_back(YPos);
}

int CLineProcessor::getLMCurveLength()
{
	return lmTipXPos.size();
}

int CLineProcessor::getSketchCurveLength()
{
	return sketchXPos.size();
}

void CLineProcessor::WriteTmpLMCurve()
{
	if(lmTipXPos.size()<MINCURVELENGTH || lmTipXPos.size()>MAXCURVELENGTH) return;
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\LMCurve_tmp.txt", ios::ate);
	for(int i=0; i<lmTipXPos.size(); i++)
		outFile << lmTipXPos.at(i)<< " " << lmTipYPos.at(i) << " " << lmTipZPos.at(i) <<endl; 
	outFile.close();
}

void CLineProcessor::WriteTmpSketchCurve()
{
	if(sketchXPos.size()<MINCURVELENGTH || sketchXPos.size()>MAXCURVELENGTH) return;
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\SketchCurve_tmp.txt", ios::ate);
	for(int i=0; i<sketchXPos.size(); i++)
		outFile << sketchXPos.at(i)<< " " << sketchYPos.at(i) <<endl; 
	outFile.close();
}

void CLineProcessor::WriteTmpTranslatedAndScaledLMCurve()
{
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedLMCurve_tmp.txt", ios::ate);
	outFile << 1 << endl << t_s_lmTipXPos.size() << endl << "1.000000" << endl; 
	for(int i=0; i<t_s_lmTipXPos.size(); i++)
		outFile << t_s_lmTipXPos.at(i) << " " << t_s_lmTipYPos.at(i) << " " << t_s_lmTipZPos.at(i) << endl; 
	outFile.close();
}

void CLineProcessor::WriteTmpTranslatedAndScaledFile3DCurve()
{
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile3DCurve_tmp.txt", ios::ate);
	outFile << 1 << endl << t_s_file3DXPos.size() << endl << "1.000000" << endl; 
	for(int i=0; i<t_s_file3DXPos.size(); i++)
		outFile << t_s_file3DXPos.at(i) << " " << t_s_file3DYPos.at(i) << " " << t_s_file3DZPos.at(i) << endl; 
	outFile.close();
}

void CLineProcessor::WriteTmpTranslatedAndScaledDTICurves()
{
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedDTICurves_tmp.txt", ios::ate);
	outFile << m_lineNum << endl; 

	for (int i=0, index=0; i<m_lineNum; i++) 
	{
		int point_num = m_lines[i].m_pointNum;
		outFile << point_num << endl << "1.000000" << endl; 

		for (int j=0; j<point_num; j++) 
		{
			outFile << ts_m_vertices[index*3+0] << " " << ts_m_vertices[index*3+1] << " " << ts_m_vertices[index*3+2] << endl; 
			index++;
		}
	}
	outFile.close();
}

void CLineProcessor::WriteLMCurve(char filename[])
{
	string str = filename;
	str = str + ".txt";
	ofstream outFile(str.c_str(), ios::ate);

	for(int i=0; i<lmTipXPos.size(); i++)
		outFile<<lmTipXPos.at(i)<<" " << lmTipYPos.at(i) <<" "<<lmTipZPos.at(i)<<endl; 
	outFile.close();
}

void CLineProcessor::WriteSketchCurve(char filename[])
{
	string str = filename;
	str = str + ".txt";
	ofstream outFile(str.c_str(), ios::ate);

	for(int i=0; i<sketchXPos.size(); i++)
		outFile << sketchXPos.at(i)<< " " << sketchYPos.at(i) <<endl; 
	outFile.close();
}

vector<string> CLineProcessor::SplitString(const string &str, const string &pattern)
{
    char * strc = new char[strlen(str.c_str())+1];
    strcpy(strc, str.c_str());
    vector<string> res;
    char* temp = strtok(strc, pattern.c_str());
    while(temp != NULL)
    {
        res.push_back(string(temp));
        temp = strtok(NULL, pattern.c_str());
    }
    delete[] strc;
    return res;
}

void CLineProcessor::ReadTmpLMCurve()
{
	string filePath = "D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\LMCurve_tmp.txt";
	ifstream inFile(filePath);
	assert(inFile.is_open());
	string line;

	while(getline(inFile,line))
    {
		vector<string> strs = SplitString(line," ");
		float x = atof(strs.at(0).c_str()); lmTipXPos.push_back(x);
		float y = atof(strs.at(1).c_str()); lmTipYPos.push_back(y);
		float z = atof(strs.at(2).c_str()); lmTipZPos.push_back(z);
    }
	inFile.close();
}

void CLineProcessor::ReadTmpSketchCurve()
{
	ClearOriginalCurve(4);

	string filePath = "D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\SketchCurve_tmp.txt";
	ifstream inFile(filePath);
	assert(inFile.is_open());

	string line;
	while(getline(inFile,line))
    {
		vector<string> strs = SplitString(line," ");
		float x = atof(strs.at(0).c_str()); sketchXPos.push_back(x);
		float y = atof(strs.at(1).c_str()); sketchYPos.push_back(y);
    }
	inFile.close();
}

void CLineProcessor::ReadFile3DCurve(char filename[])
{
	ClearOriginalCurve(2);

	string filePath = filename;
	ifstream inFile(filePath);
	assert(inFile.is_open());
	string line;

	while(getline(inFile,line))
    {
		vector<string> strs = SplitString(line," ");
		float x = atof(strs.at(0).c_str()); file3DXPos.push_back(x);
		float y = atof(strs.at(1).c_str()); file3DYPos.push_back(y);
		float z = atof(strs.at(2).c_str()); file3DZPos.push_back(z);
    }
	inFile.close();
}

void CLineProcessor::ReadFile2DCurve(char filename[])
{
	ClearOriginalCurve(3);

	string filePath = filename;
	ifstream inFile(filePath);
	assert(inFile.is_open());

	string line;
	while(getline(inFile,line))
    {
		vector<string> strs = SplitString(line," ");
		float x = atof(strs.at(0).c_str()); file2DXPos.push_back(x);
		float y = atof(strs.at(1).c_str()); file2DYPos.push_back(y);
    }
	inFile.close();
}

void CLineProcessor::DrawLMCurve()
{
	if(lmTipXPos.size()<=3) return;
	glTranslatef(0.5, -1.0, 0.5); // 手指在LM移动的时候Y轴位置比较高…
	glScalef(0.01f, 0.01f, 0.01f); // 归一化（原坐标大概都是几十）
	glLineWidth(2.5f);
	glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
	glBegin(GL_LINES);

	for(int i=1; i<lmTipXPos.size(); i++)
	{
		glVertex3f(lmTipXPos.at(i-1), lmTipYPos.at(i-1), lmTipZPos.at(i-1));
		glVertex3f(lmTipXPos.at(i), lmTipYPos.at(i), lmTipZPos.at(i));
	}
	glEnd();
}

void CLineProcessor::DrawFile3DCurve()//(Matrix4fT m_Transform)
{
	if(file3DXPos.size()<=3) return;
	//glMultMatrixf(m_Transform.M);

	glTranslatef(0.5, -1.0, 0.5); // 手指在LM移动的时候Y轴位置比较高…
	glScalef(0.01f, 0.01f, 0.01f); // 归一化（原坐标大概都是几十）
	glLineWidth(2.5f);
	glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
	glBegin(GL_LINES);


	for(int i=1; i<file3DXPos.size(); i++)
	{
		glVertex3f(file3DXPos.at(i-1), file3DYPos.at(i-1), file3DZPos.at(i-1));
		glVertex3f(file3DXPos.at(i), file3DYPos.at(i),file3DZPos.at(i));
	}
	glEnd();

}

void CLineProcessor::DrawFile2DCurve()
{
	if(file2DXPos.size()<=3) return;
	glTranslatef(-4.8f, 0.9f, 0.0f);
	glScalef(0.003f, 0.007f, 1.0f); 
	glLineWidth(2.5f);
	glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
	glBegin(GL_LINES);

	for(int i=1; i<file2DXPos.size(); i++)
	{
		glVertex2f(file2DXPos.at(i-1), -file2DYPos.at(i-1));
		glVertex2f(file2DXPos.at(i), -file2DYPos.at(i));
	}
	glEnd();
}

void CLineProcessor::DrawSketchCurve()
{
	if(sketchXPos.size()<=3) return;

	glTranslatef(-4.8f, 0.9f, 0.0f);
	glScalef(0.003f, 0.007f, 1.0f); 
	glLineWidth(2.5f);
	glColor4f(0.1f, 0.7f, 0.2f, 1.0f);
	glBegin(GL_LINES);

	for(int i=1; i<sketchXPos.size(); i++)
	{
		glVertex2f(sketchXPos.at(i-1), -sketchYPos.at(i-1));
		glVertex2f(sketchXPos.at(i), -sketchYPos.at(i));
	}
	glEnd();
}

void CLineProcessor::DrawTranslatedAndScaledLMCurve()
{
	if(t_s_lmTipXPos.size()<=3) return;

	glLineWidth(2.5f);
	glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
	glBegin(GL_LINES);

	for(int i=1; i<t_s_lmTipXPos.size(); i++)
	{
		glVertex3f(t_s_lmTipXPos.at(i-1), t_s_lmTipYPos.at(i-1), t_s_lmTipZPos.at(i-1));
		glVertex3f(t_s_lmTipXPos.at(i), t_s_lmTipYPos.at(i), t_s_lmTipZPos.at(i));
	}
	glEnd();
}

void CLineProcessor::DrawTranslatedAndScaledFile3DCurve()
{
	if(t_s_file3DXPos.size()<=3) return;

	glLineWidth(2.5f);
	glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
	glBegin(GL_LINES);

	for(int i=1; i<t_s_file3DXPos.size(); i++)
	{
		glVertex3f(t_s_file3DXPos.at(i-1), t_s_file3DYPos.at(i-1), t_s_file3DZPos.at(i-1));
		glVertex3f(t_s_file3DXPos.at(i), t_s_file3DYPos.at(i), t_s_file3DZPos.at(i));
	}
	glEnd();
}

void CLineProcessor::DrawTranslatedAndScaledFile2DCurve()
{
	if(t_s_file2DXPos.size()<=3) return;

	glLineWidth(2.5f);
	glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
	glBegin(GL_LINES);

	for(int i=1; i<t_s_file2DXPos.size(); i++)
	{
		glVertex3f(t_s_file2DXPos.at(i-1), t_s_file2DYPos.at(i-1), t_s_file2DZPos.at(i-1));
		glVertex3f(t_s_file2DXPos.at(i), t_s_file2DYPos.at(i), t_s_file2DZPos.at(i));
	}
	glEnd();
}

void CLineProcessor::DrawTranslatedAndScaledSketchCurve()
{
	if(t_s_sketchXPos.size()<=3) return;

	glLineWidth(2.5f);
	glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
	glBegin(GL_LINES);

	for(int i=1; i<t_s_sketchXPos.size(); i++)
	{
		glVertex3f(t_s_sketchXPos.at(i-1), t_s_sketchYPos.at(i-1), t_s_sketchZPos.at(i-1));
		glVertex3f(t_s_sketchXPos.at(i), t_s_sketchYPos.at(i), t_s_sketchZPos.at(i));
	}
	glEnd();
}

void CLineProcessor::DrawTranslatedAndScaledAndRotatedFile2DCurve()
{
	if(t_s_r_file2DXPos.size()<=3) return;

	glLineWidth(2.5f);
	glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
	glBegin(GL_LINES);

	for(int i=1; i<t_s_r_file2DXPos.size(); i++)
	{
		glVertex3f(t_s_r_file2DXPos.at(i-1), t_s_r_file2DYPos.at(i-1), t_s_r_file2DZPos.at(i-1));
		glVertex3f(t_s_r_file2DXPos.at(i), t_s_r_file2DYPos.at(i), t_s_r_file2DZPos.at(i));
	}
	glEnd();
}

void CLineProcessor::DrawTranslatedAndScaledDTICurves()
{
	//glMatrixMode(GL_MODELVIEW);
	//glPushMatrix();

	//if (sizeof(ts_m_vertices)>0&&m_lineNum > 0)
	//{
	//	glEnableClientState(GL_VERTEX_ARRAY);
	//	glEnableClientState(GL_COLOR_ARRAY);
	//	glVertexPointer(3, GL_FLOAT, 0, ts_m_vertices);

	//		int index = 0;
	//		for (int i=0; i<m_lineNum; i++) 
	//		{
	//			
	//				int id = m_results[i];
	//				Color3F c;
	//				if (id == -1)
	//				{
	//					c.r = c.g = c.b = 1.0f;
	//				}
	//				else
	//				{
	//					c = g_colorMap.GetClusterColor(id);
	//				}
	//				float alpha = 1.0f;
	//				if (m_selectFlags[i] == true)
	//					alpha = 1.0f;
	//				else 
	//					alpha = 1.0f;
	//				if (m_availableFlags[i] != true || m_showFlags[i] != true)
	//					alpha = 0.0f;
	//				for (int j=0; j<m_vertCount[i]; j++)
	//				{
	//					m_colors[index*4+0] = 1.0f;//c.r;
	//					m_colors[index*4+1] = 0.0f;//c.g;
	//					m_colors[index*4+2] = 0.0f;//c.b;
	//					m_colors[index*4+3] = alpha;
	//					index++;
	//				}
	//			
	//		}
	//		glColorPointer(4, GL_FLOAT, sizeof(float)*4, m_colors);
	//	

	//	glDepthMask(GL_FALSE);
	//	m_render.multiDrawArrays(m_first, m_vertCount, m_lineNum);
	//	glDepthMask(GL_TRUE);
	//}
	//
	//glPopMatrix();

	if (ts_m_vertices==NULL) return;
	glLineWidth(2.5f);
	glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
	glBegin(GL_LINES);

	int index = 1;
	for (int i=0; i<m_lineNum; i++)  // -1
	{
		int point_num = m_lines[i].m_pointNum;
		for (int j=0; j<point_num; j++) 
		{
			glVertex3f(ts_m_vertices[(index-1)*3+0], ts_m_vertices[(index-1)*3+1] ,ts_m_vertices[(index-1)*3+2] ); 
			glVertex3f(ts_m_vertices[(index)*3+0], ts_m_vertices[(index)*3+1] ,ts_m_vertices[(index)*3+2] ); 
			index++;
		}
	}
	glEnd();
}

void CLineProcessor::DrawFilteredDTICurves()
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glScalef(m_scale, m_scale, m_scale);
	glTranslatef(-m_center.x, -m_center.y, -m_center.z);

	if (f_m_lineNum > 0)
	{
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, f_m_vertices);

		int index = 0;
		for (int i=0; i<f_m_lineNum; i++) 
		{
				for (int j=0; j<f_m_vertCount[i]; j++)
				{
					m_colors[index*4+0] = 1.0f;//c.r;
					m_colors[index*4+1] = 0.0f;//c.g;
					m_colors[index*4+2] = 0.0f;//c.b;
					m_colors[index*4+3] = 1.0f;
					index++;
				}
		}
		glColorPointer(4, GL_FLOAT, sizeof(float)*4, m_colors);

		glDepthMask(GL_FALSE);
		m_render.multiDrawArrays(f_m_first, f_m_vertCount, f_m_lineNum);
		glDepthMask(GL_TRUE);
	}
	glPopMatrix();
}

void CLineProcessor::TranslateAndScaleLMCurve()
{
	float x0 = lmTipXPos.at(0);
	float y0 = lmTipYPos.at(0);
	float z0 = lmTipZPos.at(0);
	for(int i=0; i<lmTipXPos.size(); i++)
	{
		t_s_lmTipXPos.push_back(m_scale*(lmTipXPos.at(i)-x0));
		t_s_lmTipYPos.push_back(m_scale*(lmTipYPos.at(i)-y0));
		t_s_lmTipZPos.push_back(m_scale*(lmTipZPos.at(i)-z0));
	}
}

void CLineProcessor::TranslateAndScaleFile3DCurve()
{
	float x0 = file3DXPos.at(0);
	float y0 = file3DYPos.at(0);
	float z0 = file3DZPos.at(0);
	for(int i=0; i<file3DXPos.size(); i++)
	{
		t_s_file3DXPos.push_back(m_scale*(file3DXPos.at(i)-x0));
		t_s_file3DYPos.push_back(m_scale*(file3DYPos.at(i)-y0));
		t_s_file3DZPos.push_back(m_scale*(file3DZPos.at(i)-z0));
	}
}

void CLineProcessor::TranslateAndScaleDTICurves()
{
	int total_num = 0;
	for (int i=0; i<m_lineNum; i++) 
	{
		total_num += m_lines[i].m_pointNum;
	}

	ts_m_lines = new CFieldLine[m_lineNum];
	ts_m_vertices = new float[total_num*3];

	for (int i=0, index=0; i<m_lineNum; i++) 
	{
		int point_num = m_lines[i].m_pointNum;
		int mid  = point_num/2;

		float x0 = m_lines[i].m_points[0].x;
		float y0 = m_lines[i].m_points[0].y;
		float z0 = m_lines[i].m_points[0].z;

		for (int j=0; j<point_num; j++) {
			ts_m_vertices[index*3+0] = m_scale*(m_lines[i].m_points[j].x - x0);//abs(m_scale*(m_lines[i].m_points[j].x - x0)); 
			ts_m_vertices[index*3+1] = m_scale*(m_lines[i].m_points[j].y - y0);//abs(m_scale*(m_lines[i].m_points[j].y - y0)); 
			ts_m_vertices[index*3+2] = m_scale*(m_lines[i].m_points[j].z - z0);//abs(m_scale*(m_lines[i].m_points[j].z - z0)); 
			index++;
		}
	}
}

float CLineProcessor::getDegAngle(Point p1, Point p2, Point p3) 
{
    Eigen::Vector3d v1 = p1 - p2;
    Eigen::Vector3d v2 = p3 - p2;
    //one method, radian_angle belong to 0~pi
    //double radian_angle = atan2(v1.cross(v2).transpose() * (v1.cross(v2) / v1.cross(v2).norm()), v1.transpose() * v2);
    //another method, radian_angle belong to 0~pi
    float radian_angle = atan2(v1.cross(v2).norm(), v1.transpose() * v2);
    if (v1.cross(v2).z() < 0) {
        radian_angle = 2 * M_PI - radian_angle;
    }
    return radian_angle * 180 / M_PI;
}

void CLineProcessor::CaculateAnglesOnTSLMCurve()
{
	t_s_lmCurveAngles.clear();
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedLMCurveAngles_tmp.txt", ios::ate);
	for (int i=0; i<t_s_lmTipXPos.size()-2; i++)
	{
		Point p1(t_s_lmTipXPos.at(i),	 t_s_lmTipYPos.at(i),     t_s_lmTipZPos.at(i)), 
			  p2(t_s_lmTipXPos.at(i+1),   t_s_lmTipYPos.at(i+1), t_s_lmTipZPos.at(i+1)), 
			  p3(t_s_lmTipXPos.at(i+2),   t_s_lmTipYPos.at(i+2),  t_s_lmTipZPos.at(i+2));
		float angle = getDegAngle(p1, p2, p3);
		t_s_lmCurveAngles.push_back(angle);
		outFile << angle << endl; 
	}
	outFile.close();
}

void CLineProcessor::CaculateAnglesOnTSFile3DCurve()
{
	t_s_file3DCurveAngles.clear();
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile3DCurveAngles_tmp.txt", ios::ate);
	for (int i=0; i<t_s_file3DXPos.size()-2; i++)
	{
		Point p1(t_s_file3DXPos.at(i),	 t_s_file3DYPos.at(i),     t_s_file3DZPos.at(i)), 
			  p2(t_s_file3DXPos.at(i+1),   t_s_file3DYPos.at(i+1),   t_s_file3DZPos.at(i+1)), 
			  p3(t_s_file3DXPos.at(i+2),   t_s_file3DYPos.at(i+2),   t_s_file3DZPos.at(i+2));
		float angle = getDegAngle(p1, p2, p3);
		t_s_file3DCurveAngles.push_back(angle);
		outFile << angle << endl; 
	}
	outFile.close();
}

void CLineProcessor::CaculateAnglesOnTSFile2DCurve()
{
	t_s_file2DCurveAngles.clear();
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile2DCurveAngles_tmp.txt", ios::ate);
	for (int i=0; i<t_s_file2DXPos.size()-2; i++)
	{
		Point p1(t_s_file2DXPos.at(i),	 t_s_file2DYPos.at(i),     t_s_file2DZPos.at(i)), 
			  p2(t_s_file2DXPos.at(i+1),   t_s_file2DYPos.at(i+1),   t_s_file2DZPos.at(i+1)), 
			  p3(t_s_file2DXPos.at(i+2),   t_s_file2DYPos.at(i+2),   t_s_file2DZPos.at(i+2));
		float angle = getDegAngle(p1, p2, p3);
		t_s_file2DCurveAngles.push_back(angle);
		outFile << angle << endl; 
	}
	outFile.close();
}

void CLineProcessor::CaculateAnglesOnTSSketchCurve()
{
	t_s_sketchCurveAngles.clear();
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedSketchCurveAngles_tmp.txt", ios::ate);
	for (int i=0; i<t_s_sketchXPos.size()-2; i++)
	{
		Point p1(t_s_sketchXPos.at(i),	 t_s_sketchYPos.at(i),     t_s_sketchZPos.at(i)), 
			  p2(t_s_sketchXPos.at(i+1),   t_s_sketchYPos.at(i+1),   t_s_sketchZPos.at(i+1)), 
			  p3(t_s_sketchXPos.at(i+2),   t_s_sketchYPos.at(i+2),   t_s_sketchZPos.at(i+2));
		float angle = getDegAngle(p1, p2, p3);
		t_s_sketchCurveAngles.push_back(angle);
		outFile << angle << endl; 
	}
	outFile.close();
}
void CLineProcessor::CaculateAnglesOnDTICurves()
{
	for(int i=0; i<DTICurvesAngleSeq.size(); i++) DTICurvesAngleSeq.at(i).clear();
	DTICurvesAngleSeq.clear();

	ofstream outFile1("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\DTICurvesAngles_tmp.txt", ios::ate);
	ofstream outFile2("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\DTICurvesAngleSeq_tmp.txt", ios::ate);

	for(int i=0,index=0; i<m_lineNum; i++)
	{
		int point_num = m_lines[i].m_pointNum;
		deque<int> seq;
		outFile1<<"line "<<i<<endl;
		outFile2<<"line "<<i<<endl;
		CaculateAnglesOnDTICurve(index,point_num,m_vertices,seq, outFile1, outFile2);
		DTICurvesAngleSeq.push_back(seq);
	}
	outFile1.close();
	outFile2.close();
}

void CLineProcessor::CaculateAnglesOnDTICurve(int &index, int point_num, float *m_vertices, deque<int> &seq, ofstream &out1, ofstream &out2)
{
	DTICurveAngles.clear();
	for (int i=0; i<point_num-2; i++)
	{
		Point p1(m_vertices[index*3+0],		 m_vertices[index*3+1],		  m_vertices[index*3+2]), 
			  p2(m_vertices[(index+1)*3+0],	 m_vertices[(index+1)*3+1],   m_vertices[(index+1)*3+2]), 
			  p3(m_vertices[(index+2)*3+0],	 m_vertices[(index+2)*3+1],   m_vertices[(index+2)*3+2]);
		float angle = getDegAngle(p1, p2, p3);
		DTICurveAngles.push_back(angle);
		out1<<angle<<endl;
		index++;
	}
	index+=2;
	for(int i=0; i<DTICurveAngles.size(); i++)
	{
		float angle = DTICurveAngles.at(i);
		for(int j=1; j<AngleBinSeq.size(); j++)
			if (AngleBinSeq.at(j-1)<=angle&&AngleBinSeq.at(j)>angle)
			{
				seq.push_back(j-1);
				out2<<j-1<<endl;
				break;
			}
	}
}

void CLineProcessor::CaculateCurvatureOnTranslatedLMCurve()
{
	CaculateSOnTranslatedLMCurve();

	CaculateFirstOrderDiscreteDerivative(t_s_lmTipXPos,t_s_lmCurveS, t_s_lmXFirstOrderDiscreteDerivative);
	CaculateFirstOrderDiscreteDerivative(t_s_lmTipYPos,t_s_lmCurveS, t_s_lmYFirstOrderDiscreteDerivative);
	CaculateFirstOrderDiscreteDerivative(t_s_lmTipZPos,t_s_lmCurveS, t_s_lmZFirstOrderDiscreteDerivative);
	ofstream outFile1("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedLMCurveFirstOrderDiscreteDerivative_tmp.txt", ios::ate);
	for(int i=0; i<t_s_lmXFirstOrderDiscreteDerivative.size(); i++)
	{
		outFile1<<t_s_lmXFirstOrderDiscreteDerivative.at(i)<<" "<<t_s_lmYFirstOrderDiscreteDerivative.at(i)<<" "<< t_s_lmZFirstOrderDiscreteDerivative.at(i)<<endl;
	}
	outFile1.close();

	CaculateSecondOrderDiscreteDerivative(t_s_lmXFirstOrderDiscreteDerivative,t_s_lmCurveS, t_s_lmXSecondOrderDiscreteDerivative);
	CaculateSecondOrderDiscreteDerivative(t_s_lmYFirstOrderDiscreteDerivative,t_s_lmCurveS, t_s_lmYSecondOrderDiscreteDerivative);
	CaculateSecondOrderDiscreteDerivative(t_s_lmZFirstOrderDiscreteDerivative,t_s_lmCurveS, t_s_lmZSecondOrderDiscreteDerivative);
	ofstream outFile2("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedLMCurveSecondOrderDiscreteDerivative_tmp.txt", ios::ate);
	for(int i=0; i<t_s_lmXSecondOrderDiscreteDerivative.size(); i++)
	{
		outFile2<<t_s_lmXSecondOrderDiscreteDerivative.at(i)<<" "<<t_s_lmYSecondOrderDiscreteDerivative.at(i)<<" "<< t_s_lmZSecondOrderDiscreteDerivative.at(i)<<endl;
	}
	outFile2.close();

	CaculateCurvature(t_s_lmXFirstOrderDiscreteDerivative,t_s_lmYFirstOrderDiscreteDerivative,t_s_lmZFirstOrderDiscreteDerivative,
	t_s_lmXSecondOrderDiscreteDerivative,t_s_lmYSecondOrderDiscreteDerivative,t_s_lmZSecondOrderDiscreteDerivative,
	t_s_lmCurveCurvature);
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedLMCurveCurvature_tmp.txt", ios::ate);
	for(int i=0; i<t_s_lmCurveCurvature.size();i++)
		outFile<<t_s_lmCurveCurvature.at(i)<<endl;
	outFile.close();
}

void CLineProcessor::CaculateCurvatureOnTranslatedFile3DCurve()
{
	CaculateSOnTranslatedFile3DCurve(); 

	CaculateFirstOrderDiscreteDerivative(t_s_file3DXPos,t_s_file3DCurveS, t_s_file3DXFirstOrderDiscreteDerivative);
	CaculateFirstOrderDiscreteDerivative(t_s_file3DYPos,t_s_file3DCurveS, t_s_file3DYFirstOrderDiscreteDerivative);
	CaculateFirstOrderDiscreteDerivative(t_s_file3DZPos,t_s_file3DCurveS, t_s_file3DZFirstOrderDiscreteDerivative);
	ofstream outFile1("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile3DCurveFirstOrderDiscreteDerivative_tmp.txt", ios::ate);
	for(int i=0; i<t_s_file3DXFirstOrderDiscreteDerivative.size(); i++)
	{
		outFile1<<t_s_file3DXFirstOrderDiscreteDerivative.at(i)<<" "<<t_s_file3DYFirstOrderDiscreteDerivative.at(i)<<" "<< t_s_file3DZFirstOrderDiscreteDerivative.at(i)<<endl;
	}
	outFile1.close();

	CaculateSecondOrderDiscreteDerivative(t_s_file3DXFirstOrderDiscreteDerivative,t_s_file3DCurveS, t_s_file3DXSecondOrderDiscreteDerivative);
	CaculateSecondOrderDiscreteDerivative(t_s_file3DYFirstOrderDiscreteDerivative,t_s_file3DCurveS, t_s_file3DYSecondOrderDiscreteDerivative);
	CaculateSecondOrderDiscreteDerivative(t_s_file3DZFirstOrderDiscreteDerivative,t_s_file3DCurveS, t_s_file3DZSecondOrderDiscreteDerivative);
	ofstream outFile2("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile3DCurveSecondOrderDiscreteDerivative_tmp.txt", ios::ate);
	for(int i=0; i<t_s_file3DXSecondOrderDiscreteDerivative.size(); i++)
	{
		outFile2<<t_s_file3DXSecondOrderDiscreteDerivative.at(i)<<" "<<t_s_file3DYSecondOrderDiscreteDerivative.at(i)<<" "<< t_s_file3DZSecondOrderDiscreteDerivative.at(i)<<endl;
	}
	outFile2.close();

	CaculateCurvature(t_s_file3DXFirstOrderDiscreteDerivative,t_s_file3DYFirstOrderDiscreteDerivative,t_s_file3DZFirstOrderDiscreteDerivative,
		t_s_file3DXSecondOrderDiscreteDerivative,t_s_file3DYSecondOrderDiscreteDerivative,t_s_file3DZSecondOrderDiscreteDerivative,
		t_s_file3DCurveCurvature);
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile3DCurveCurvature_tmp.txt", ios::ate);
	for(int i=0; i<t_s_file3DCurveCurvature.size();i++)
		outFile<<t_s_file3DCurveCurvature.at(i)<<endl;
	outFile.close();
}

void CLineProcessor::CaculateCurvatureOnTSFile2DCurve()
{
	CaculateSOnTSFile2DCurve(); 

	CaculateFirstOrderDiscreteDerivative(t_s_file2DXPos,t_s_file2DCurveS, t_s_file2DXFirstOrderDiscreteDerivative);
	CaculateFirstOrderDiscreteDerivative(t_s_file2DYPos,t_s_file2DCurveS, t_s_file2DYFirstOrderDiscreteDerivative);
	CaculateFirstOrderDiscreteDerivative(t_s_file2DZPos,t_s_file2DCurveS, t_s_file2DZFirstOrderDiscreteDerivative);
	ofstream outFile1("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile2DCurveFirstOrderDiscreteDerivative_tmp.txt", ios::ate);
	for(int i=0; i<t_s_file2DXFirstOrderDiscreteDerivative.size(); i++)
	{
		outFile1<<t_s_file2DXFirstOrderDiscreteDerivative.at(i)<<" "<<t_s_file2DYFirstOrderDiscreteDerivative.at(i)<<" "<< t_s_file2DZFirstOrderDiscreteDerivative.at(i)<<endl;
	}
	outFile1.close();

	CaculateSecondOrderDiscreteDerivative(t_s_file2DXFirstOrderDiscreteDerivative,t_s_file2DCurveS, t_s_file2DXSecondOrderDiscreteDerivative);
	CaculateSecondOrderDiscreteDerivative(t_s_file2DYFirstOrderDiscreteDerivative,t_s_file2DCurveS, t_s_file2DYSecondOrderDiscreteDerivative);
	CaculateSecondOrderDiscreteDerivative(t_s_file2DZFirstOrderDiscreteDerivative,t_s_file2DCurveS, t_s_file2DZSecondOrderDiscreteDerivative);
	ofstream outFile2("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile2DCurveSecondOrderDiscreteDerivative_tmp.txt", ios::ate);
	for(int i=0; i<t_s_file2DXSecondOrderDiscreteDerivative.size(); i++)
	{
		outFile2<<t_s_file2DXSecondOrderDiscreteDerivative.at(i)<<" "<<t_s_file2DYSecondOrderDiscreteDerivative.at(i)<<" "<< t_s_file2DZSecondOrderDiscreteDerivative.at(i)<<endl;
	}
	outFile2.close();

	CaculateCurvature(t_s_file2DXFirstOrderDiscreteDerivative,t_s_file2DYFirstOrderDiscreteDerivative,t_s_file2DZFirstOrderDiscreteDerivative,
		t_s_file2DXSecondOrderDiscreteDerivative,t_s_file2DYSecondOrderDiscreteDerivative,t_s_file2DZSecondOrderDiscreteDerivative,
		t_s_file2DCurveCurvature);
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile2DCurveCurvature_tmp.txt", ios::ate);
	for(int i=0; i<t_s_file2DCurveCurvature.size();i++)
		outFile<<t_s_file2DCurveCurvature.at(i)<<endl;
	outFile.close();
}

void CLineProcessor::CaculateCurvatureOnTSSketchCurve()
{
	CaculateSOnTSSketchCurve(); 

	CaculateFirstOrderDiscreteDerivative(t_s_sketchXPos,t_s_sketchCurveS, t_s_sketchXFirstOrderDiscreteDerivative);
	CaculateFirstOrderDiscreteDerivative(t_s_sketchYPos,t_s_sketchCurveS, t_s_sketchYFirstOrderDiscreteDerivative);
	CaculateFirstOrderDiscreteDerivative(t_s_sketchZPos,t_s_sketchCurveS, t_s_sketchZFirstOrderDiscreteDerivative);
	ofstream outFile1("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedSketchCurveFirstOrderDiscreteDerivative_tmp.txt", ios::ate);
	for(int i=0; i<t_s_sketchXFirstOrderDiscreteDerivative.size(); i++)
	{
		outFile1<<t_s_sketchXFirstOrderDiscreteDerivative.at(i)<<" "<<t_s_sketchYFirstOrderDiscreteDerivative.at(i)<<" "<< t_s_sketchZFirstOrderDiscreteDerivative.at(i)<<endl;
	}
	outFile1.close();

	CaculateSecondOrderDiscreteDerivative(t_s_sketchXFirstOrderDiscreteDerivative,t_s_sketchCurveS, t_s_sketchXSecondOrderDiscreteDerivative);
	CaculateSecondOrderDiscreteDerivative(t_s_sketchYFirstOrderDiscreteDerivative,t_s_sketchCurveS, t_s_sketchYSecondOrderDiscreteDerivative);
	CaculateSecondOrderDiscreteDerivative(t_s_sketchZFirstOrderDiscreteDerivative,t_s_sketchCurveS, t_s_sketchZSecondOrderDiscreteDerivative);
	ofstream outFile2("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedSketchCurveSecondOrderDiscreteDerivative_tmp.txt", ios::ate);
	for(int i=0; i<t_s_sketchXSecondOrderDiscreteDerivative.size(); i++)
	{
		outFile2<<t_s_sketchXSecondOrderDiscreteDerivative.at(i)<<" "<<t_s_sketchYSecondOrderDiscreteDerivative.at(i)<<" "<< t_s_sketchZSecondOrderDiscreteDerivative.at(i)<<endl;
	}
	outFile2.close();

	CaculateCurvature(t_s_sketchXFirstOrderDiscreteDerivative,t_s_sketchYFirstOrderDiscreteDerivative,t_s_sketchZFirstOrderDiscreteDerivative,
		t_s_sketchXSecondOrderDiscreteDerivative,t_s_sketchYSecondOrderDiscreteDerivative,t_s_sketchZSecondOrderDiscreteDerivative,
		t_s_sketchCurveCurvature);
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedSkecthCurveCurvature_tmp.txt", ios::ate);
	for(int i=0; i<t_s_sketchCurveCurvature.size();i++)
		outFile<<t_s_sketchCurveCurvature.at(i)<<endl;
	outFile.close();
}


void CLineProcessor::CaculateCurvatureOnTranslatedAndScaledDTICurve()
{
	//CaculateSsOnDTICurves();

	ofstream outFile1("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\DTICurveFirstOrderDiscreteDerivative_tmp.txt", ios::ate);
	ofstream outFile2("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\DTICurveSecondOrderDiscreteDerivative_tmp.txt", ios::ate);
	ofstream outFile3("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\DTICurvesCurvature_tmp.txt", ios::ate);
	ofstream outFile4("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\DTICurveSs_tmp.txt", ios::ate);
	//ofstream outFile5("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\DTICurvesCurvatureSeq_tmp.txt", ios::ate);

	for(int i=0,index=0; i<m_lineNum; i++)
	{
		outFile1<<"line "<<i<<endl;
		outFile2<<"line "<<i<<endl;
		outFile3<<"line "<<i<<endl;
		outFile4<<"line "<<i<<endl;
		//outFile5<<"line "<<i<<endl;

		int point_num = m_lines[i].m_pointNum;
		deque<float> xPos; deque<float> yPos; deque<float> zPos;
		for(int j=0; j<point_num; j++)
		{
			xPos.push_back(ts_m_vertices[index*3+0]); 
			yPos.push_back(ts_m_vertices[index*3+1]); 
			zPos.push_back(ts_m_vertices[index*3+2]); 
			index++;
		}

		// Caculate S
		for(int i=0; i<DTICurveSs.size(); i++) DTICurveSs.at(i).clear();
		DTICurveSs.clear();
		deque <float> s;  s.push_back(0.0); outFile4<<0.0<<endl;
		float denominator = 0.0;
		for (int k=0; k<point_num-1; k++) // 分母：k: 0~n-2
		{
			float xk = xPos.at(k+1)-xPos.at(k);
			float yk = yPos.at(k+1)-yPos.at(k);
			float zk = zPos.at(k+1)-zPos.at(k);

			denominator+=sqrt(pow(xk,2)+pow(yk,2)+pow(zk,2));
		}
		for(int ii=1; ii<xPos.size(); ii++)  // 分子：i: 1~n-1
		{
			float numerator = 0.0;
			for(int k=0; k<ii; k++) // k: 0~i-1
			{
				float xk = xPos.at(k+1)-xPos.at(k);
				float yk = yPos.at(k+1)-yPos.at(k);
				float zk = zPos.at(k+1)-zPos.at(k);

				numerator+=sqrt(pow(xk,2)+pow(yk,2)+pow(zk,2));
			}
			float si = numerator/denominator;
			s.push_back(si); // Si: i = 0~n-1
			outFile4<<si<<endl;
		}
		DTICurveSs.push_back(s);

		// Caculate D1
		deque<float> xD1; deque<float> yD1; deque<float> zD1;

		CaculateFirstOrderDiscreteDerivative(xPos,s, xD1);
		CaculateFirstOrderDiscreteDerivative(yPos,s, yD1);
		CaculateFirstOrderDiscreteDerivative(zPos,s, zD1);

		DTIXFirstOrderDiscreteDerivative.push_back(xD1);
		DTIYFirstOrderDiscreteDerivative.push_back(yD1);
		DTIZFirstOrderDiscreteDerivative.push_back(zD1);

		for(int j=0; j<xD1.size();j++)
		{
			outFile1<<xD1.at(j)<<" "<<yD1.at(j)<<" "<<zD1.at(j)<<endl;
		}
		
		// Caculate D2
		deque<float> xD2; deque<float> yD2; deque<float> zD2;

		CaculateSecondOrderDiscreteDerivative(xD1,s, xD2);
		CaculateSecondOrderDiscreteDerivative(yD1,s, yD2);
		CaculateSecondOrderDiscreteDerivative(zD1,s, zD2);

		DTIXSecondOrderDiscreteDerivative.push_back(xD2);
		DTIYSecondOrderDiscreteDerivative.push_back(yD2);
		DTIZSecondOrderDiscreteDerivative.push_back(zD2);
		
		for(int j=0; j<xD2.size();j++)
		{
			outFile2<<xD2.at(j)<<" "<<yD2.at(j)<<" "<<zD2.at(j)<<endl;
		}

		// Caculate Curvature
		deque<float> c;
		CaculateCurvature(xD1,yD1,zD1,xD2,yD2,zD2,c);
		DTICurvesCurvature.push_back(c);
		for(int i=0; i<c.size();i++)
			outFile3<<c.at(i)<<endl;

		//deque<int> seq;
		//for(int i=0; i<c.size(); i++)
		//{
		//	float angle = c.at(i);
		//	for(int j=1; j<AngleBinSeq.size(); j++)
		//		if (AngleBinSeq.at(j-1)<=angle&&AngleBinSeq.at(j)>angle)
		//		{
		//			seq.push_back(j-1);
		//			outFile5<<j-1<<endl;
		//			break;
		//		}
		//}
		//DTICurvesCurvatureSeq.push_back(seq);
	}
	outFile1.close();
	outFile2.close();
	outFile3.close();
	outFile4.close();
	//outFile5.close();
}

void CLineProcessor::CaculateSOnTranslatedLMCurve()
{
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedLMCurveS_tmp.txt", ios::ate);

	t_s_lmCurveS.clear();
	t_s_lmCurveS.push_back(0);  // s0 = 0
	outFile<<0<<endl;
	
	float denominator = 0.0;
	for (int k=0; k<t_s_lmTipXPos.size()-1; k++) // 分母：k: 0~n-2
	{
		float xk = t_s_lmTipXPos.at(k+1)-t_s_lmTipXPos.at(k);
		float yk = t_s_lmTipYPos.at(k+1)-t_s_lmTipYPos.at(k);
		float zk = t_s_lmTipZPos.at(k+1)-t_s_lmTipZPos.at(k);

		denominator+=sqrt(pow(xk,2)+pow(yk,2)+pow(zk,2));
	}
	 
	for(int i=1; i<t_s_lmTipXPos.size(); i++)  // 分子：i: 1~n-1
	{
		float numerator = 0.0;
		for(int k=0; k<i; k++) // k: 0~i-1
		{
			float xk = t_s_lmTipXPos.at(k+1)-t_s_lmTipXPos.at(k);
			float yk = t_s_lmTipYPos.at(k+1)-t_s_lmTipYPos.at(k);
			float zk = t_s_lmTipZPos.at(k+1)-t_s_lmTipZPos.at(k);

			numerator+=sqrt(pow(xk,2)+pow(yk,2)+pow(zk,2));
		}
		float si = numerator/denominator;
		t_s_lmCurveS.push_back(si); // Si: i = 0~n-1
		outFile<< si <<endl;
	}
	outFile.close();
}

void CLineProcessor::CaculateSOnTranslatedFile3DCurve()
{
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile3DCurveS_tmp.txt", ios::ate);

	t_s_file3DCurveS.clear();
	t_s_file3DCurveS.push_back(0);  // s0 = 0
	outFile<<0<<endl;
	
	float denominator = 0.0;
	for (int k=0; k<t_s_file3DXPos.size()-1; k++) // 分母：k: 0~n-2
	{
		float xk = t_s_file3DXPos.at(k+1)-t_s_file3DXPos.at(k);
		float yk = t_s_file3DYPos.at(k+1)-t_s_file3DYPos.at(k);
		float zk = t_s_file3DZPos.at(k+1)-t_s_file3DZPos.at(k);

		denominator+=sqrt(pow(xk,2)+pow(yk,2)+pow(zk,2));
	}
	 
	for(int i=1; i<t_s_file3DXPos.size(); i++)  // 分子：i: 1~n-1
	{
		float numerator = 0.0;
		for(int k=0; k<i; k++) // k: 0~i-1
		{
			float xk = t_s_file3DXPos.at(k+1)-t_s_file3DXPos.at(k);
			float yk = t_s_file3DYPos.at(k+1)-t_s_file3DYPos.at(k);
			float zk = t_s_file3DZPos.at(k+1)-t_s_file3DZPos.at(k);

			numerator+=sqrt(pow(xk,2)+pow(yk,2)+pow(zk,2));
		}
		float si = numerator/denominator;
		t_s_file3DCurveS.push_back(si); // Si: i = 0~n-1
		outFile<< si <<endl;
	}
	outFile.close();
}

void CLineProcessor::CaculateSOnTSFile2DCurve()
{
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile2DCurveS_tmp.txt", ios::ate);

	t_s_file2DCurveS.clear();
	t_s_file2DCurveS.push_back(0);  // s0 = 0
	outFile<<0<<endl;
	
	float denominator = 0.0;
	for (int k=0; k<t_s_file2DXPos.size()-1; k++) // 分母：k: 0~n-2
	{
		float xk = t_s_file2DXPos.at(k+1)-t_s_file2DXPos.at(k);
		float yk = t_s_file2DYPos.at(k+1)-t_s_file2DYPos.at(k);
		float zk = t_s_file2DZPos.at(k+1)-t_s_file2DZPos.at(k);

		denominator+=sqrt(pow(xk,2)+pow(yk,2)+pow(zk,2));
	}
	 
	for(int i=1; i<t_s_file2DXPos.size(); i++)  // 分子：i: 1~n-1
	{
		float numerator = 0.0;
		for(int k=0; k<i; k++) // k: 0~i-1
		{
			float xk = t_s_file2DXPos.at(k+1)-t_s_file2DXPos.at(k);
			float yk = t_s_file2DYPos.at(k+1)-t_s_file2DYPos.at(k);
			float zk = t_s_file2DZPos.at(k+1)-t_s_file2DZPos.at(k);

			numerator+=sqrt(pow(xk,2)+pow(yk,2)+pow(zk,2));
		}
		float si = numerator/denominator;
		t_s_file2DCurveS.push_back(si); // Si: i = 0~n-1
		outFile<< si <<endl;
	}
	outFile.close();
}

void CLineProcessor::CaculateSOnTSSketchCurve()
{
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedSketchCurveS_tmp.txt", ios::ate);

	t_s_sketchCurveS.clear();
	t_s_sketchCurveS.push_back(0);  // s0 = 0
	outFile<<0<<endl;
	
	float denominator = 0.0;
	for (int k=0; k<t_s_sketchXPos.size()-1; k++) // 分母：k: 0~n-2
	{
		float xk = t_s_sketchXPos.at(k+1)-t_s_sketchXPos.at(k);
		float yk = t_s_sketchYPos.at(k+1)-t_s_sketchYPos.at(k);
		float zk = t_s_sketchZPos.at(k+1)-t_s_sketchZPos.at(k);

		denominator+=sqrt(pow(xk,2)+pow(yk,2)+pow(zk,2));
	}
	 
	for(int i=1; i<t_s_sketchXPos.size(); i++)  // 分子：i: 1~n-1
	{
		float numerator = 0.0;
		for(int k=0; k<i; k++) // k: 0~i-1
		{
			float xk = t_s_sketchXPos.at(k+1)-t_s_sketchXPos.at(k);
			float yk = t_s_sketchYPos.at(k+1)-t_s_sketchYPos.at(k);
			float zk = t_s_sketchZPos.at(k+1)-t_s_sketchZPos.at(k);

			numerator+=sqrt(pow(xk,2)+pow(yk,2)+pow(zk,2));
		}
		float si = numerator/denominator;
		t_s_sketchCurveS.push_back(si); // Si: i = 0~n-1
		outFile<< si <<endl;
	}
	outFile.close();
}

void CLineProcessor::CaculateFirstOrderDiscreteDerivative(deque<float> Pos, deque<float> S, deque<float> &D1)
{
	// D.push_back(-1.0); // D0 = null
	for(int i=0+n1; i<Pos.size()-n1; i++)
	{
		float numerator = 0.0; float denominator = 0.0;
		for(int j=i-n1; j<=i+n1; j++)
		{
			denominator+=pow(S.at(j)-S.at(i),2);
			numerator+=((S.at(j)-S.at(i))*(Pos.at(j)-Pos.at(i)));
		}
		D1.push_back((float)numerator/(float)denominator);
	}
}

void CLineProcessor::CaculateSecondOrderDiscreteDerivative(deque<float> D1, deque<float> S, deque<float> &D2)
{
	// D.push_back(-1.0); // D0 = null
	for(int i=0+n2; i<D1.size()-n2; i++) 
	{
		float numerator = 0.0; float denominator = 0.0;
		for(int j=i-n2; j<=i+n2; j++)
		{
			denominator+=pow(S.at(j)-S.at(i),2);
			numerator+=((S.at(j)-S.at(i))*(D1.at(j)-D1.at(i)));
		}
		D2.push_back((float)numerator/(float)denominator);
	}
}

void CLineProcessor::CaculateCurvature(deque<float> XD1,deque<float> YD1,deque<float> ZD1, deque<float> XD2,deque<float> YD2, deque<float> ZD2, deque<float>&C)
{
	C.clear();
	for(int i2=0,i1=0+n2; i2<XD2.size()&&i1<XD1.size()-n2; i1++,i2++)
	{
		float x1 = XD1.at(i1); float y1 = YD1.at(i1); float z1 = ZD1.at(i1);
		float x2 = XD2.at(i2); float y2 = YD2.at(i2); float z2 = ZD2.at(i2);

		float x3 = y1*z2-y2*z1; float y3 = z1*x2-x1*z2; float z3 = x1*y2-x2*y1;

		float numerator = sqrt(pow(x3,2)+pow(y3,2)+pow(z3,2));
		float denominator = pow(sqrt(pow(x1,2)+pow(y1,2)+pow(z1,2)),3);
		float c = numerator/denominator;

		if (c>MAX_CUR) MAX_CUR = c;
		if (c<MIN_CUR) MIN_CUR = c;

		C.push_back(c);
	}
}

void CLineProcessor::MapAnglesToAngleBinSeqOnTranslatedLMCurve()
{
	t_s_lmCurveAngleSeq.clear();
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedLMCurveAngleSeq_tmp.txt", ios::ate);
	for(int i=0; i<t_s_lmCurveAngles.size(); i++)
	{
		float angle = t_s_lmCurveAngles.at(i);
		for(int j=1; j<AngleBinSeq.size(); j++)
			if (AngleBinSeq.at(j-1)<=angle&&AngleBinSeq.at(j)>angle)
			{
				t_s_lmCurveAngleSeq.push_back(j-1);
				outFile << j-1 << endl; 
				break;
			}
	}
	outFile.close();
}

void CLineProcessor::MapAnglesToAngleBinSeqOnTranslatedFile3DCurve()
{
	t_s_file3DCurveAngleSeq.clear();
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile3DCurveAngleSeq_tmp.txt", ios::ate);
	for(int i=0; i<t_s_file3DCurveAngles.size(); i++)
	{
		float angle = t_s_file3DCurveAngles.at(i);
		for(int j=1; j<AngleBinSeq.size(); j++)
			if (AngleBinSeq.at(j-1)<=angle&&AngleBinSeq.at(j)>angle)
			{
				t_s_file3DCurveAngleSeq.push_back(j-1);
				outFile << j-1 << endl; 
				break;
			}
	}
	outFile.close();
}

void CLineProcessor::MapAnglesToAngleBinSeqOnTranslatedFile2DCurve()
{
	t_s_file2DCurveAngleSeq.clear();
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile2DCurveAngleSeq_tmp.txt", ios::ate);
	for(int i=0; i<t_s_file2DCurveAngles.size(); i++)
	{
		float angle = t_s_file2DCurveAngles.at(i);
		for(int j=1; j<AngleBinSeq.size(); j++)
			if (AngleBinSeq.at(j-1)<=angle&&AngleBinSeq.at(j)>angle)
			{
				t_s_file2DCurveAngleSeq.push_back(j-1);
				outFile << j-1 << endl; 
				break;
			}
	}
	outFile.close();
}

void CLineProcessor::MapAnglesToAngleBinSeqOnTranslatedSketchCurve()
{
	t_s_sketchCurveAngleSeq.clear();
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedSketchCurveAngleSeq_tmp.txt", ios::ate);
	for(int i=0; i<t_s_sketchCurveAngles.size(); i++)
	{
		float angle = t_s_sketchCurveAngles.at(i);
		for(int j=1; j<AngleBinSeq.size(); j++)
			if (AngleBinSeq.at(j-1)<=angle&&AngleBinSeq.at(j)>angle)
			{
				t_s_sketchCurveAngleSeq.push_back(j-1);
				outFile << j-1 << endl; 
				break;
			}
	}
	outFile.close();
}

void CLineProcessor::MapCurvaturesToCurvatureBinSeqOnTranslatedLMCurve()
{
	t_s_lmCurveCurvatureSeq.clear();
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedLMCurveCurvatureSeq_tmp.txt", ios::ate);
	for(int i=0; i<t_s_lmCurveCurvature.size(); i++)
	{
		float angle = t_s_lmCurveCurvature.at(i);
		for(int j=1; j<CurvatureBinSeq.size(); j++)
			if (CurvatureBinSeq.at(j-1)<=angle&&CurvatureBinSeq.at(j)>angle)
			{
				t_s_lmCurveCurvatureSeq.push_back(j-1);
				outFile << j-1 << endl; 
				break;
			}
	}
	outFile.close();
}

void CLineProcessor::MapCurvaturesToCurvatureBinSeqOnTranslatedFile3DCurve()
{
	t_s_file3DCurveCurvatureSeq.clear();
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile3DCurveCurvatureSeq_tmp.txt", ios::ate);
	for(int i=0; i<t_s_file3DCurveCurvature.size(); i++)
	{
		float angle = t_s_file3DCurveCurvature.at(i);
		for(int j=1; j<CurvatureBinSeq.size(); j++)
			if (CurvatureBinSeq.at(j-1)<=angle&&CurvatureBinSeq.at(j)>angle)
			{
				t_s_file3DCurveCurvatureSeq.push_back(j-1);
				outFile << j-1 << endl; 
				break;
			}
	}
	outFile.close();
}

void CLineProcessor::MapCurvaturesToCurvatureBinSeqOnTSFile2DCurve()
{
	t_s_file2DCurveCurvatureSeq.clear();
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile2DCurveCurvatureSeq_tmp.txt", ios::ate);
	for(int i=0; i<t_s_file2DCurveCurvature.size(); i++)
	{
		float angle = t_s_file2DCurveCurvature.at(i);
		for(int j=1; j<CurvatureBinSeq.size(); j++)
			if (CurvatureBinSeq.at(j-1)<=angle&&CurvatureBinSeq.at(j)>angle)
			{
				t_s_file2DCurveCurvatureSeq.push_back(j-1);
				outFile << j-1 << endl; 
				break;
			}
	}
	outFile.close();
}

void CLineProcessor::MapCurvaturesToCurvatureBinSeqOnTSSketchCurve()
{
	t_s_sketchCurveCurvatureSeq.clear();
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedSketchCurveCurvatureSeq_tmp.txt", ios::ate);
	for(int i=0; i<t_s_sketchCurveCurvature.size(); i++)
	{
		float angle = t_s_sketchCurveCurvature.at(i);
		for(int j=1; j<CurvatureBinSeq.size(); j++)
			if (CurvatureBinSeq.at(j-1)<=angle&&CurvatureBinSeq.at(j)>angle)
			{
				t_s_sketchCurveCurvatureSeq.push_back(j-1);
				outFile << j-1 << endl; 
				break;
			}
	}
	outFile.close();
}

void CLineProcessor::MapCurvaturesToCurvatureBinSeqOnDTICurves()
{
	ofstream outFile5("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\DTICurvesCurvatureSeq_tmp.txt", ios::ate);
	for(int ii=0,index=0; ii<m_lineNum; ii++)
	{
		outFile5<<"line "<<ii<<endl;
		deque<float> c = DTICurvesCurvature.at(ii);
		deque<int> seq;
		for(int i=0; i<c.size(); i++)
		{
			float angle = c.at(i);
			for(int j=1; j<CurvatureBinSeq.size(); j++)
				if (CurvatureBinSeq.at(j-1)<=angle&&CurvatureBinSeq.at(j)>angle)
				{
					seq.push_back(j-1);
					outFile5<<j-1<<endl;
					break;
				}
		}
		DTICurvesCurvatureSeq.push_back(seq);
	}
	outFile5.close();
}

int CLineProcessor::LCSS(deque<int> A,deque<int> B)
{
	int **table = new int*[A.size()];
	for(int i=0; i<A.size(); i++) table[i] = new int[B.size()];

	for(int i=0; i<A.size(); i++)
	{
		for (int j=0; j<B.size(); j++)
		{
			if(i==0||j==0) 
				if(A.at(i)==B.at(j)) table[i][j]=1;
				else table[i][j]=0;
			else if (i>0&&j>0&&A.at(i)==B.at(j))
				table[i][j] = 1+table[i-1][j-1];
			else
				table[i][j] = table[i][j-1]>=table[i-1][j]?table[i][j-1]:table[i-1][j];
		}
	}

	int lcss = table[A.size()-1][B.size()-1];

	 for(int i=0;i<A.size();i++)
        delete []table[i];
    delete []table;

	return lcss;
}

void CLineProcessor::GetSimilarityForDTICurves(int type)
{
	aSimilarityForDTICurves.clear(); cSimilarityForDTICurves.clear();
	ofstream outFile1("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\aLCSSForDTICurves_tmp.txt", ios::ate);
	ofstream outFile2("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\aSimilarityForDTICurves_tmp.txt", ios::ate);
	ofstream outFile3("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\cLCSSForDTICurves_tmp.txt", ios::ate);
	ofstream outFile4("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\cSimilarityForDTICurves_tmp.txt", ios::ate);
	if (type==1) // Query by LM Curve
	{
		for(int i=0; i<DTICurvesAngleSeq.size(); i++)
		{
			float lcss = 0.0f; float sim = 0.0f;
			lcss =(float)LCSS(DTICurvesAngleSeq.at(i),t_s_lmCurveAngleSeq);
			outFile1 << lcss << endl;
			sim = lcss/(float)max(DTICurvesAngleSeq.at(i).size(),
				  t_s_lmCurveAngleSeq.size());
			outFile2 << sim << endl;
			aSimilarityForDTICurves.push_back(sim);
		}	

		for(int i=0; i<DTICurvesCurvatureSeq.size(); i++)
		{
			float lcss = 0.0f; float sim = 0.0f;
			lcss =(float)LCSS(DTICurvesCurvatureSeq.at(i),t_s_lmCurveCurvatureSeq);
			outFile3 << lcss << endl;
			sim = lcss/(float)max(DTICurvesCurvatureSeq.at(i).size(),
				  t_s_lmCurveCurvatureSeq.size());
			outFile4 << sim << endl;
			cSimilarityForDTICurves.push_back(sim);
		}
	}
	else if (type==2) // Query by File 3D Curve
	{
		for(int i=0; i<DTICurvesAngleSeq.size(); i++)
		{
			float lcss = 0.0f; float sim = 0.0f;
			lcss =(float)LCSS(DTICurvesAngleSeq.at(i),t_s_file3DCurveAngleSeq);
			outFile1 << lcss << endl;
			sim = lcss/(float)max(DTICurvesAngleSeq.at(i).size(),
				  t_s_file3DCurveAngleSeq.size());
			outFile2 << sim << endl;
			aSimilarityForDTICurves.push_back(sim);
		}

		for(int i=0; i<DTICurvesCurvatureSeq.size(); i++)
		{
			float lcss = 0.0f; float sim = 0.0f;
			lcss =(float)LCSS(DTICurvesCurvatureSeq.at(i),t_s_file3DCurveCurvatureSeq);
			outFile3 << lcss << endl;
			sim = lcss/(float)max(DTICurvesCurvatureSeq.at(i).size(),
				  t_s_file3DCurveCurvatureSeq.size());
			outFile4 << sim << endl;
			cSimilarityForDTICurves.push_back(sim);
		}
	}
	else if (type==3)
	{
		for(int i=0; i<DTICurvesAngleSeq.size(); i++)
		{
			float lcss = 0.0f; float sim = 0.0f;
			lcss =(float)LCSS(DTICurvesAngleSeq.at(i),t_s_file2DCurveAngleSeq);
			outFile1 << lcss << endl;
			sim = lcss/(float)max(DTICurvesAngleSeq.at(i).size(),
				  t_s_file2DCurveAngleSeq.size());
			outFile2 << sim << endl;
			aSimilarityForDTICurves.push_back(sim);
		}
		for(int i=0; i<DTICurvesCurvatureSeq.size(); i++)
		{
			float lcss = 0.0f; float sim = 0.0f;
			lcss =(float)LCSS(DTICurvesCurvatureSeq.at(i),t_s_file2DCurveCurvatureSeq);
			outFile3 << lcss << endl;
			sim = lcss/(float)max(DTICurvesCurvatureSeq.at(i).size(),
				  t_s_file2DCurveCurvatureSeq.size());
			outFile4 << sim << endl;
			cSimilarityForDTICurves.push_back(sim);
		}
	}
	else if (type==4)
	{
		for(int i=0; i<DTICurvesAngleSeq.size(); i++)
		{
			float lcss = 0.0f; float sim = 0.0f;
			lcss =(float)LCSS(DTICurvesAngleSeq.at(i),t_s_sketchCurveAngleSeq);
			outFile1 << lcss << endl;
			sim = lcss/(float)max(DTICurvesAngleSeq.at(i).size(),
				  t_s_sketchCurveAngleSeq.size());
			outFile2 << sim << endl;
			aSimilarityForDTICurves.push_back(sim);
		}	

		for(int i=0; i<DTICurvesCurvatureSeq.size(); i++)
		{
			float lcss = 0.0f; float sim = 0.0f;
			lcss =(float)LCSS(DTICurvesCurvatureSeq.at(i),t_s_sketchCurveCurvatureSeq);
			outFile3 << lcss << endl;
			sim = lcss/(float)max(DTICurvesCurvatureSeq.at(i).size(),
				  t_s_sketchCurveCurvatureSeq.size());
			outFile4 << sim << endl;
			cSimilarityForDTICurves.push_back(sim);
		}
	}
	outFile1.close(); outFile2.close();outFile3.close(); outFile4.close();
}

void CLineProcessor::FilterDTICurves(int type)
{
	f_m_lineNum = 0;
	SAFE_DELETE(f_m_first);
	SAFE_DELETE(f_m_vertCount);
	SAFE_DELETE(f_m_vertices);

	deque<int> filteredDTICurves_index;
	for(int i=0; i<aSimilarityForDTICurves.size(); i++)
	{
		if (type==1||type==2)
		{
			if(aSimilarityForDTICurves.at(i)>=ANGLE_SIMILARITY_THRESHOLD_3D&&cSimilarityForDTICurves.at(i)>=CURVATURE_SIMILARITY_THRESHOLD_3D)
			{
				filteredDTICurves_index.push_back(i);
				f_m_lineNum++;
			}
		}
		else if (type==3||type==4)
		{
			if(aSimilarityForDTICurves.at(i)>=ANGLE_SIMILARITY_THRESHOLD_2D&&cSimilarityForDTICurves.at(i)>=CURVATURE_SIMILARITY_THRESHOLD_2D)
			{
				filteredDTICurves_index.push_back(i);
				f_m_lineNum++;
			}
		}
	}

	f_m_first = new int [f_m_lineNum];
	f_m_vertCount = new int [f_m_lineNum];

	int total_num = 0;
	for(int i=0; i<f_m_lineNum; i++) // 找出原m_line里面下标为index序列中下标的所有曲线
	{
		int index = filteredDTICurves_index.at(i);
		
		f_m_first[i] = total_num;
		f_m_vertCount[i] = m_lines[index].m_pointNum;
		total_num+=m_lines[index].m_pointNum;
	}

	f_m_vertices = new float[total_num*3];
	int index = 0; int newIndex = 0;
	for(int i=0; i<f_m_lineNum; i++)
	{
		newIndex = filteredDTICurves_index.at(i);

		int pointNum = m_lines[newIndex].m_pointNum;
		for(int j=0; j<pointNum; j++)
		{
			f_m_vertices[index*3+0] = m_lines[newIndex].m_points[j].x;
			f_m_vertices[index*3+1] = m_lines[newIndex].m_points[j].y;
			f_m_vertices[index*3+2] = m_lines[newIndex].m_points[j].z;
			index++;
		}
	}
	FILTERED_FLAG = true;
}

void CLineProcessor::SetAngleSimilarityThreshold(float simThreshold)
{
	ANGLE_SIMILARITY_THRESHOLD_3D = simThreshold;
}

void CLineProcessor::SetAngle2DSimilarityThreshold(float simThreshold)
{
	ANGLE_SIMILARITY_THRESHOLD_2D = simThreshold;
}


void CLineProcessor::SetCurvature2DSimilarityThreshold(float simThreshold)
{
	CURVATURE_SIMILARITY_THRESHOLD_2D = simThreshold;
}

void CLineProcessor::SetCurvatureSimilarityThreshold(float simThreshold)
{
	CURVATURE_SIMILARITY_THRESHOLD_3D = simThreshold;
}

void CLineProcessor::InitCurvatureBin()
{
	CurvatureBinSeq.clear();
	for(int i = (int)MIN_CUR; i<=(((int)MAX_CUR)+1+5); i+=5)
		CurvatureBinSeq.push_back(i);
}

void  CLineProcessor::SetGranularityOfAngleBin(int gran)
{
	AngleBinSeq.clear();
	for(int i=1; i<=360+1+gran; i+=gran) // default granularity = 10
		AngleBinSeq.push_back(i-1);
}

void  CLineProcessor::SetGranularityOfCurvatureBin(int gran)
{
	if (CurvatureBinSeq.size()<=0) return;
	CurvatureBinSeq.clear();
	for(int i = (int)MIN_CUR; i<=(((int)MAX_CUR)+1+gran); i+=gran) // default granularity = 5
		CurvatureBinSeq.push_back(i);
}

void CLineProcessor::TranslateAndScaleFile2DCurve()
{
	float x0 = file2DXPos.at(0);
	float y0 = file2DYPos.at(0);

	for(int i=0; i<file2DXPos.size(); i++)
	{
		t_s_file2DXPos.push_back(m_scale*((file2DXPos.at(i))-x0)*0.6);
		t_s_file2DYPos.push_back(m_scale*((file2DYPos.at(i))-y0)*0.6);
		t_s_file2DZPos.push_back(0);
	}
}

void CLineProcessor::TranslateAndScaleSketchCurve()
{
	float x0 = sketchXPos.at(0);
	float y0 = sketchYPos.at(0);

	for(int i=0; i<sketchXPos.size(); i++)
	{
		t_s_sketchXPos.push_back(m_scale*((sketchXPos.at(i))-x0)*0.6);
		t_s_sketchYPos.push_back(m_scale*((sketchYPos.at(i))-y0)*0.6);
		t_s_sketchZPos.push_back(0);
	}
}

void CLineProcessor::RotateTSCurve(Matrix4fT m_Transform, int type)
{
	if (type == 1)
	{
		for (int i=0; i<t_s_lmTipXPos.size(); i++)
		{
			t_s_r_file2DXPos.push_back(m_Transform.M[0]*t_s_lmTipXPos.at(i)+m_Transform.M[1]*t_s_lmTipYPos.at(i)+m_Transform.M[2]*t_s_lmTipZPos.at(i)+m_Transform.M[3]*1);
			t_s_r_file2DYPos.push_back(m_Transform.M[4]*t_s_lmTipXPos.at(i)+m_Transform.M[5]*t_s_lmTipYPos.at(i)+m_Transform.M[6]*t_s_lmTipZPos.at(i)+m_Transform.M[7]*1);
			t_s_r_file2DZPos.push_back(m_Transform.M[8]*t_s_lmTipXPos.at(i)+m_Transform.M[9]*t_s_lmTipYPos.at(i)+m_Transform.M[10]*t_s_lmTipZPos.at(i)+m_Transform.M[11]*1);
		}
		t_s_lmTipXPos.clear();  t_s_lmTipYPos.clear();  t_s_lmTipZPos.clear(); 
		for (int i=0; i<t_s_r_file2DXPos.size(); i++)
		{
			t_s_lmTipXPos.push_back(t_s_r_file2DXPos.at(i));
			t_s_lmTipYPos.push_back(t_s_r_file2DYPos.at(i));
			t_s_lmTipZPos.push_back(t_s_r_file2DZPos.at(i));
		}
		t_s_r_file2DXPos.clear(); t_s_r_file2DYPos.clear(); t_s_r_file2DZPos.clear(); 		
	}
	else if (type == 2)
	{
		for (int i=0; i<t_s_file3DXPos.size(); i++)
		{
			t_s_r_file2DXPos.push_back(m_Transform.M[0]*t_s_file3DXPos.at(i)+m_Transform.M[1]*t_s_file3DYPos.at(i)+m_Transform.M[2]*t_s_file3DZPos.at(i)+m_Transform.M[3]*1);
			t_s_r_file2DYPos.push_back(m_Transform.M[4]*t_s_file3DXPos.at(i)+m_Transform.M[5]*t_s_file3DYPos.at(i)+m_Transform.M[6]*t_s_file3DZPos.at(i)+m_Transform.M[7]*1);
			t_s_r_file2DZPos.push_back(m_Transform.M[8]*t_s_file3DXPos.at(i)+m_Transform.M[9]*t_s_file3DYPos.at(i)+m_Transform.M[10]*t_s_file3DZPos.at(i)+m_Transform.M[11]*1);
		}
		t_s_file3DXPos.clear();  t_s_file3DYPos.clear();  t_s_file3DZPos.clear(); 
		for (int i=0; i<t_s_r_file2DXPos.size(); i++)
		{
			t_s_file3DXPos.push_back(t_s_r_file2DXPos.at(i));
			t_s_file3DYPos.push_back(t_s_r_file2DYPos.at(i));
			t_s_file3DZPos.push_back(t_s_r_file2DZPos.at(i));
		}
		t_s_r_file2DXPos.clear(); t_s_r_file2DYPos.clear(); t_s_r_file2DZPos.clear(); 	
	}
	else if (type == 3)
	{
		for (int i=0; i<t_s_file2DXPos.size(); i++)
		{
			t_s_r_file2DXPos.push_back(m_Transform.M[0]*t_s_file2DXPos.at(i)+m_Transform.M[1]*t_s_file2DYPos.at(i)+m_Transform.M[2]*t_s_file2DZPos.at(i)+m_Transform.M[3]*1);
			t_s_r_file2DYPos.push_back(m_Transform.M[4]*t_s_file2DXPos.at(i)+m_Transform.M[5]*t_s_file2DYPos.at(i)+m_Transform.M[6]*t_s_file2DZPos.at(i)+m_Transform.M[7]*1);
			t_s_r_file2DZPos.push_back(m_Transform.M[8]*t_s_file2DXPos.at(i)+m_Transform.M[9]*t_s_file2DYPos.at(i)+m_Transform.M[10]*t_s_file2DZPos.at(i)+m_Transform.M[11]*1);
		}
		t_s_file2DXPos.clear();  t_s_file2DYPos.clear();  t_s_file2DZPos.clear(); 
		for (int i=0; i<t_s_r_file2DXPos.size(); i++)
		{
			t_s_file2DXPos.push_back(t_s_r_file2DXPos.at(i));
			t_s_file2DYPos.push_back(t_s_r_file2DYPos.at(i));
			t_s_file2DZPos.push_back(t_s_r_file2DZPos.at(i));
		}
		t_s_r_file2DXPos.clear(); t_s_r_file2DYPos.clear(); t_s_r_file2DZPos.clear(); 
	}
	else if (type == 4)
	{
		for (int i=0; i<t_s_sketchXPos.size(); i++)
		{
			t_s_r_file2DXPos.push_back(m_Transform.M[0]*t_s_sketchXPos.at(i)+m_Transform.M[1]*t_s_sketchYPos.at(i)+m_Transform.M[2]*t_s_sketchZPos.at(i)+m_Transform.M[3]*1);
			t_s_r_file2DYPos.push_back(m_Transform.M[4]*t_s_sketchXPos.at(i)+m_Transform.M[5]*t_s_sketchYPos.at(i)+m_Transform.M[6]*t_s_sketchZPos.at(i)+m_Transform.M[7]*1);
			t_s_r_file2DZPos.push_back(m_Transform.M[8]*t_s_sketchXPos.at(i)+m_Transform.M[9]*t_s_sketchYPos.at(i)+m_Transform.M[10]*t_s_sketchZPos.at(i)+m_Transform.M[11]*1);
		}
		t_s_sketchXPos.clear();  t_s_sketchYPos.clear();  t_s_sketchZPos.clear(); 
		for (int i=0; i<t_s_r_file2DXPos.size(); i++)
		{
			t_s_sketchXPos.push_back(t_s_r_file2DXPos.at(i));
			t_s_sketchYPos.push_back(t_s_r_file2DYPos.at(i));
			t_s_sketchZPos.push_back(t_s_r_file2DZPos.at(i));
		}
		t_s_r_file2DXPos.clear(); t_s_r_file2DYPos.clear(); t_s_r_file2DZPos.clear(); 	
	}
}

//void CLineProcessor::CaculateAnglesOnTranslatedDTICurves()
//{
//	ofstream outFile1("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedDTICurvesAngles_tmp.txt", ios::ate);
//	ofstream outFile2("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedDTICurvesAngleSeq_tmp.txt", ios::ate);
//
//	for(int i=0,index=0; i<m_lineNum; i++)
//	{
//		int point_num = m_lines[i].m_pointNum;
//		deque<int> seq;
//		outFile1<<"line "<<i<<endl;
//		outFile2<<"line "<<i<<endl;
//		CaculateAnglesOnTranslatedDTICurve(index,point_num,ts_m_vertices,seq, outFile1, outFile2);
//		t_DTICurvesAngleSeq.push_back(seq);
//	}
//	outFile1.close();
//	outFile2.close();
//}

//
//void CLineProcessor::CaculateAnglesOnTranslatedDTICurve(int &index, int point_num, float *ts_m_vertices, deque<int> &seq, ofstream &out1, ofstream &out2)
//{
//	t_DTICurveAngles.clear();
//	for (int i=0; i<point_num-2; i++)
//	{
//		Point p1(ts_m_vertices[index*3+0],		 ts_m_vertices[index*3+1],		  ts_m_vertices[index*3+2]), 
//			  p2(ts_m_vertices[(index+1)*3+0],	 ts_m_vertices[(index+1)*3+1],     ts_m_vertices[(index+1)*3+2]), 
//			  p3(ts_m_vertices[(index+2)*3+0],	 ts_m_vertices[(index+2)*3+1],     ts_m_vertices[(index+2)*3+2]);
//		float angle = getDegAngle(p1, p2, p3);
//		t_DTICurveAngles.push_back(angle);
//		out1<<angle<<endl;
//		index++;
//	}
//	index+=2;
//	for(int i=0; i<t_DTICurveAngles.size(); i++)
//	{
//		float angle = t_DTICurveAngles.at(i);
//		for(int j=1; j<AngleBinSeq.size(); j++)
//			if (AngleBinSeq.at(j-1)<=angle&&AngleBinSeq.at(j)>angle)
//			{
//				seq.push_back(j-1);
//				out2<<j-1<<endl;
//				break;
//			}
//	}
//}

//int CLineProcessor::LCSS(int i, int j, deque<int> seqA, deque<int> seqB)
//{
//	if (i==0||j==0) 
//		if (seqA.at(i)==seqB.at(j)) return 1;
//		else return 0;
//	else if (i>0&&j>0&&seqA.at(i)==seqB.at(j))
//		return 1+LCSS(i-1,j-1,seqA, seqB);
//	else // i>0&&j>0&&A.at(i)!=B.at(j)
//		return max(LCSS(i-1,j,seqA, seqB), LCSS(i,j-1,seqA, seqB));
//}

//void CLineProcessor::GetSimilarityForTranslatedDTICurves()
//{
//	ofstream outFile1("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\LCSSForTranslatedDTICurves_tmp.txt", ios::ate);
//	ofstream outFile2("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\SimilarityForTranslatedDTICurves_tmp.txt", ios::ate);
//	for(int i=0; i<t_DTICurvesAngleSeq.size(); i++)
//	{
//		float lcss = 0.0f; float sim = 0.0f;
//		lcss =(float)LCSS(t_DTICurvesAngleSeq.at(i),t_s_file3DCurveAngleSeq);
//		outFile1 << lcss << endl;
//		sim = lcss/(float)max(t_DTICurvesAngleSeq.at(i).size(),
//			  t_s_file3DCurveAngleSeq.size());
//		outFile2 << sim << endl;
//		similarityForTranslatedDTICurves.push_back(sim);
//	}
//	outFile1.close(); outFile2.close();
//}

//void CLineProcessor::FilterTranslatedDTICurves()
//{
//	ts_f_m_lineNum = 0;
//	SAFE_DELETE(ts_f_m_first);
//	SAFE_DELETE(ts_f_m_vertCount);
//
//	deque<int> filteredTranslatedDTICurves_index;
//	for(int i=0; i<similarityForTranslatedDTICurves.size(); i++)
//	{
//		if(similarityForTranslatedDTICurves.at(i)>=ANGLE_SIMILARITY_THRESHOLD_3D)
//		{
//			filteredTranslatedDTICurves_index.push_back(i);
//			ts_f_m_lineNum++;
//		}
//	}
//
//	ts_f_m_first = new int [ts_f_m_lineNum];
//	ts_f_m_vertCount = new int [ts_f_m_lineNum];
//
//	int total_num = 0;
//	for(int i=0; i<ts_f_m_lineNum; i++) // 找出原m_line里面下标为index序列中下标的所有曲线
//	{
//		int index = filteredTranslatedDTICurves_index.at(i);
//		
//		ts_f_m_first[i] = total_num;
//		ts_f_m_vertCount[i] = m_lines[index].m_pointNum;
//		total_num+=m_lines[index].m_pointNum;
//	}
//
//	ts_f_m_vertices = new float[total_num*3];
//	int index = 0; int newIndex = 0;
//	for(int i=0; i<ts_f_m_lineNum; i++)
//	{
//		newIndex = filteredTranslatedDTICurves_index.at(i);
//
//		int pointNum = m_lines[newIndex].m_pointNum;
//		for(int j=0; j<pointNum; j++)
//		{
//			ts_f_m_vertices[index*3+0] = m_lines[newIndex].m_points[j].x;
//			ts_f_m_vertices[index*3+1] = m_lines[newIndex].m_points[j].y;
//			ts_f_m_vertices[index*3+2] = m_lines[newIndex].m_points[j].z;
//			index++;
//		}
//	}
//	FILTERED_FLAG = true;
//}

//Chen============

//void CLineProcessor::DrawDTIStreamTubes()
//{
	//// set light
	//GLfloat light_pos[] = {-10.0f, -10.0f, -10.0f, 0.0f};
	//GLfloat light_diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
	//GLfloat light_specular[] = {1.0f, 1.0f, 1.0f, 1.0f};
	//glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
	//glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	//glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

	//GLfloat mat_diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
	//GLfloat mat_specular[] = {1.0f, 1.0f, 1.0f, 1.0f};
	//GLfloat mat_shininess[] = {50.0f};
	//glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
	//glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);

	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);
	//glEnable(GL_DEPTH_TEST);
	//glDisable(GL_BLEND);
	//glDisable(GL_ALPHA_TEST);
	//glEnable(GL_NORMALIZE);

	//glMatrixMode(GL_MODELVIEW);
	//glPushMatrix();

	//glScalef(m_scale, m_scale, m_scale);
	//glTranslatef(-m_center.x, -m_center.y, -m_center.z);

	//for (int i=0; i<m_lineNum; i++) {
	//	if (m_availableFlags[i] == true && m_showFlags[i] == true) {
	//		int id = m_results[i];
	//		Color3F c;
	//		if (id == -1) {
	//			c.r = c.g = c.b = 1.0f;
	//		} else {
	//			c = g_colorMap.GetClusterColor(id);
	//		}
	//		glColor4f(c.r, c.g, c.b, 1.0f);
	//		m_lines[i].DrawStreamTubes();
	//	}
	//}

	//glPopMatrix();

	//glDisable(GL_LIGHTING);
	//glDisable(GL_LIGHT0);
	//glDisable(GL_DEPTH_TEST);
	//glEnable(GL_BLEND);
	//glEnable(GL_ALPHA_TEST);
//}
//
//
//void CLineProcessor::DrawNameDTI()
//{
	//glMatrixMode(GL_MODELVIEW);
	//glPushMatrix();
	//glScalef(m_scale, m_scale, m_scale);
	//glTranslatef(-m_center.x, -m_center.y, -m_center.z);
	//for (int i=0; i<m_lineNum; i++) {
	//	if (m_availableFlags[i] != true || m_showFlags[i] != true)
	//		continue;
	//	glColor3f(1.0f, 1.0f, 1.0f);
	//	glLoadName(i);
	//	glLineWidth(1.0f);
	//	glPointSize(1.0f);
	//	m_lines[i].DrawLines();
	//	m_lines[i].DrawPoints();
	//}
	//glPopMatrix();
//}

//void CLineProcessor::DrawNameMDS()
//{
//	for (int i=0; i<m_pointNum; i++) {
//		glColor3f(1.0f, 1.0f, 1.0f);
//		glLoadName(i);
//		glPointSize(4.0f);
//		glBegin(GL_POINTS);
//		glVertex2f(m_points[i].x, m_points[i].y);
//		glEnd();
//	}
//}

//void CLineProcessor::DrawLinesInCube(vector<int> &lines, Color3F color)
//{
	//int num = static_cast<int>(lines.size());
	//if(num == 0)
	//	return;
	//glMatrixMode(GL_MODELVIEW);
	//glPushMatrix();
	//glScalef(m_scale, m_scale, m_scale);
	//glTranslatef(-m_center.x, -m_center.y, -m_center.z);
	//glColor4f(color.r, color.g, color.b, 1.0f);
	//for (int i=0; i<num; i++)
	//{
	//	int id = lines.at(i);
	//	if (m_availableFlags[id] != true || m_showFlags[id] != true)
	//		continue;
	//	m_lines[id].DrawLines();
	//}
	//glPopMatrix();
//}

//void CLineProcessor::DrawPointsInCube(vector<int> &points, Color3F color)
//{
//	int num = static_cast<int>(points.size());

	//glMatrixMode(GL_MODELVIEW);
	//glPushMatrix();

	//glScalef(m_scale, m_scale, m_scale);
	//glTranslatef(-m_center.x, -m_center.y, -m_center.z);
//	glColor4f(color.r, color.g, color.b, 1.0f);
//	for (int i=0; i<num; i++) {
//		int id = points.at(i);
//		if (m_availableFlags[id] != true || m_showFlags[id] != true)
//			continue;
//		glBegin(GL_POINTS);
//		glVertex2f(m_points[id].x, m_points[id].y);
//		glEnd();
//	}
//
//	//glPopMatrix();
//}
//
//void CLineProcessor::SingleSelectDTI(GLuint *selectBuffer, const GLint hits, 
//								  BOOL classFlag, BOOL mode, bool res)
//{
//	GLuint *ptr, names;
//	ptr = selectBuffer;
//	for (int i=0; i<hits; i++) {
//		names = *ptr;
//		ptr++;						// name
//		ptr++;						// z1
//		ptr++;						// z2
//		for (int j=0; j<(int)names; j++) {
//			if (mode) {
//				m_selectFlags[static_cast<int>(*ptr)] = 
//					!m_selectFlags[static_cast<int>(*ptr)];
//			} else {
//				m_selectFlags[static_cast<int>(*ptr)] = res;
//			}
//			int id = m_results[static_cast<int>(*ptr)];
//			if (classFlag && id >= 0) {
//				for (int k=0; k<m_lineNum; k++) {
//					if (m_results[k] == id)
//						m_selectFlags[k] = m_selectFlags[static_cast<int>(*ptr)];
//				}
//			}
//			ptr++;
//		}
//
//		if (mode)
//			break;
//	}
//}
//
//void CLineProcessor::SingleSelectMDS(Point2F point, const float r, const BOOL classFlag, 
//									 const BOOL mode, const bool res)
//{
//	if (mode) {
//		float min_dis = FLT_MAX;
//		int pos = -1;
//		for (int i=0; i<m_pointNum; i++) {
//			if (m_availableFlags[i] != true || m_showFlags[i] != true)
//				continue;
//			float dis = (m_points[i].x - point.x) * (m_points[i].x - point.x);
//			dis += (m_points[i].y - point.y) * (m_points[i].y - point.y);
//			if (dis < r) {
//				if (dis < min_dis) {
//					min_dis = dis;
//					pos = i;
//				}
//			}
//		}
//		if (pos == -1)
//			return;
//		m_selectFlags[pos] = !m_selectFlags[pos];
//		int id = m_results[pos];
//		if (classFlag && id >= 0) {
//			for (int k=0; k<m_lineNum; k++) {
//				if (m_results[k] == id)
//					m_selectFlags[k] = m_selectFlags[pos];
//			}
//		}
//	} else {
//		for (int i=0; i<m_pointNum; i++) {
//			if (m_availableFlags[i] != true || m_showFlags[i] != true)
//				continue;
//			float dis = (m_points[i].x - point.x) * (m_points[i].x - point.x);
//			dis += (m_points[i].y - point.y) * (m_points[i].y - point.y);
//			if (dis < r) {
//				m_selectFlags[i] = res;
//				int id = m_results[i];
//				if (classFlag && id >= 0) {
//					for (int k=0; k<m_lineNum; k++) {
//						if (m_results[k] == id)
//							m_selectFlags[k] = m_selectFlags[i];
//					}
//				}
//			}
//		}
//	}
//}
//
//void CLineProcessor::ApplyCubeSelections(vector<int> &lines)
//{
//	int size = static_cast<int>(lines.size());
//	for (int i=0; i<size; i++) {
//		int id = lines.at(i);
//		m_selectFlags[id] = true;
//	}
//
//}
//
//void CLineProcessor::DrawLength(const float left, const float right)
//{
//	int l_pos = static_cast<int>(left * static_cast<float>(m_lengthHistogramNum));
//	int r_pos = static_cast<int>(right * static_cast<float>(m_lengthHistogramNum));
//
//	glLineWidth(3.0f);
//	float x_pos = -0.7f;
//	float x_step = 1.4f / static_cast<float>(m_lengthHistogramNum);
//	float y_pos1 = -0.6f;
//	for (int i=0; i<m_lengthHistogramNum; i++) {
//		float y_pos2 = m_lengthHistogram[i] * 1.4f + y_pos1;
//
//		float r = 1.0f / static_cast<float>(m_lengthHistogramNum);
//		r = r * static_cast<float>(i);
//		if (i >= l_pos && i <= r_pos) {
//			glColor3f(r, 1.0f, 1.0f-r);
//		} else {
//			//r = r / 2.0f + 0.25f;
//			glColor3f(r, r, r);
//		}
//
//		glBegin(GL_LINES);
//		glVertex2f(x_pos, y_pos1);
//		glVertex2f(x_pos, y_pos2);
//		glEnd();
//
//		x_pos += x_step;
//	}
//
//	int maxNum = 0;
//	float max_value = 0.0f;
//	float sum = 0.0f;
//	for (int i=0; i<m_lengthHistogramNum; i++) {
//		if (m_lengthHistogram[i] > max_value)
//			max_value = m_lengthHistogram[i];
//		sum += m_lengthHistogram[i];
//	}
//
//	if (m_lineNum == 0)
//		maxNum = 0;
//	else 
//	maxNum = static_cast<int>(max_value / sum * static_cast<float>(m_lineNum));
//	DrawXYCoord(m_minLength, m_maxLength, maxNum, 0);
//}
//
//void CLineProcessor::DrawFA(const float left, const float right)
//{
//	int l_pos = static_cast<int>(left * static_cast<float>(m_FAHistogramNum));
//	int r_pos = static_cast<int>(right * static_cast<float>(m_FAHistogramNum));
//
//	glLineWidth(3.0f);
//	float x_pos = -0.7f;
//	float x_step = 1.4f / static_cast<float>(m_FAHistogramNum);
//	float y_pos1 = -0.6f;
//	for (int i=0; i<m_FAHistogramNum; i++) {
//		float y_pos2 = m_FAHistogram[i] * 1.4f + y_pos1;
//
//		float r = 1.0f / static_cast<float>(m_FAHistogramNum);
//		r = r * static_cast<float>(i);
//		if (i >= l_pos && i <= r_pos) {
//			glColor3f(1.0f-r, r, 0.0f);
//		} else {
//			glColor3f(r, r, r);
//		}
//
//		glBegin(GL_LINES);
//		glVertex2f(x_pos, y_pos1);
//		glVertex2f(x_pos, y_pos2);
//		glEnd();
//
//		x_pos += x_step;
//	}
//
//	// draw the coord
//	int maxNum = 0;
//	float max_value = 0.0f;
//	float sum = 0.0f;
//	for (int i=0; i<m_FAHistogramNum; i++) {
//		if (m_FAHistogram[i] > max_value)
//			max_value = m_FAHistogram[i];
//		sum += m_FAHistogram[i];
//	}
//	if (m_lineNum == 0)
//		maxNum = 0;
//	else 
//		maxNum = static_cast<int>(max_value / sum * static_cast<float>(m_lineNum));
//	DrawXYCoord(m_minFA, m_maxFA, maxNum, 0);
//}
//
//void CLineProcessor::DrawCurvature(const float left, const float right)
//{
//	int l_pos = static_cast<int>(left * static_cast<float>(m_CurHistogramNum));
//	int r_pos = static_cast<int>(right * static_cast<float>(m_CurHistogramNum));
//
//	glLineWidth(3.0f);
//	float x_pos = -0.7f;
//	float x_step = 1.4f / static_cast<float>(m_CurHistogramNum);
//	float y_pos1 = -0.6f;
//	for (int i=0; i<m_CurHistogramNum; i++) {
//		float y_pos2 = m_CurHistogram[i] * 1.4f + y_pos1;
//
//		float r = 1.0f / static_cast<float>(m_CurHistogramNum);
//		r = r * static_cast<float>(i);
//		if (i >= l_pos && i <= r_pos) {
//			glColor3f(1.0f, 1.0-r, r);
//		} else {
//			glColor3f(r, r, r);
//		}
//
//		glBegin(GL_LINES);
//		glVertex2f(x_pos, y_pos1);
//		glVertex2f(x_pos, y_pos2);
//		glEnd();
//
//		x_pos += x_step;
//	}
//
//	// draw the coord
//	int maxNum = 0;
//	float max_value = 0.0f;
//	float sum = 0.0f;
//	for (int i=0; i<m_CurHistogramNum; i++) {
//		if (m_CurHistogram[i] > max_value)
//			max_value = m_CurHistogram[i];
//		sum += m_CurHistogram[i];
//	}
//		if (m_lineNum == 0)
//		maxNum = 0;
//	else 
//	maxNum = static_cast<int>(max_value / sum * static_cast<float>(m_lineNum));
//	DrawXYCoord(m_minCur, m_maxCur, maxNum, 0);
//}
//
//void CLineProcessor::DrawXYCoord(const float x_left, const float x_right,
//								 const int y_top, const int y_bottom)
//{
//	glDisable(GL_LINE_SMOOTH);
//	glLineWidth(1.0f);
//	glColor3f(1.0f, 1.0f, 1.0f);
//	glBegin(GL_LINES);
//	for (int i=1; i<16; i++) {
//		float x = 0.1f * static_cast<float>(i) - 0.8f;
//		float y = -0.7f;
//		glVertex2f(x, y);
//		if (i % 5 == 0) {
//			glVertex2f(x, y+0.06f);
//		} else {
//			glVertex2f(x, y+0.04f);
//		}
//
//		x = -0.8f;
//		y = 0.1 * static_cast<float>(i) - 0.7f;
//		glVertex2f(x, y);
//		if (i % 5 == 0) {
//			glVertex2f(x+0.06f, y);
//		} else {
//			glVertex2f(x+0.04f, y);
//		}
//	}
//	glEnd();
//
//	// draw the coord
//	glLineWidth(2.0f);
//	glColor3f(1.0f, 0.0f, 0.0f);
//	glBegin(GL_LINE_STRIP);
//	glVertex2f(-0.8f, -0.7f);
//	glVertex2f(0.9f, -0.7f);
//	glEnd();
//	glBegin(GL_TRIANGLES);
//	glVertex2f(0.90f, -0.7f);
//	glVertex2f(0.8f, -0.67f);
//	glVertex2f(0.8f, -0.73f);
//	glEnd();
//
//	glColor3f(0.0f, 1.0f, 0.0f);
//	glBegin(GL_LINE_STRIP);
//	glVertex2f(-0.8f, -0.7f);
//	glVertex2f(-0.8f, 0.9f);
//	glEnd();
//	glBegin(GL_TRIANGLES);
//	glVertex2f(-0.8f, 0.9f);
//	glVertex2f(-0.83f, 0.8f);
//	glVertex2f(-0.77f, 0.8f);
//	glEnd();
//
//	glEnable(GL_LINE_SMOOTH);
//
//	float color[3];
//	color[0] = color[1] = color[2] = 1.0f;
//	char buffer[128];
//	sprintf(buffer, "%.3f", x_left);
//	g_GLFont.DrawFont2D(buffer, -0.8f, -0.8f, color, 0.0015f);
//	sprintf(buffer, "%.3f", x_right);
//	g_GLFont.DrawFont2D(buffer, 0.5f, -0.8f, color, 0.0015f);
//	sprintf(buffer, "%d", y_top);
//	g_GLFont.DrawFont2D(buffer, -0.95f, 0.7f, color, 0.0015f);
//	sprintf(buffer, "%d", y_bottom);
//	g_GLFont.DrawFont2D(buffer, -0.9f, -0.7f, color, 0.0015f);
//}

//void CLineProcessor::ResetAvailableFlags(const bool flag)
//{
//	//for (int i=0; i<m_lineNum; i++)
//	//	m_availableFlags[i] = flag;
//}
//
//void CLineProcessor::LengthFilter(const float min_pos, const float max_pos)
//{
//	//float min_scale = min_pos;
//	//float max_scale = max_pos;
//	//float min_length = m_minLength + (m_maxLength - m_minLength) * min_scale;
//	//float max_length = m_minLength + (m_maxLength - m_minLength) * max_scale;
//
//	//for (int i=0; i<m_lineNum; i++) {
//	//	float length = m_lines[i].GetLength();
//	//	if (length < min_length)
//	//		m_availableFlags[i] = false;
//	//	else if (length > max_length)
//	//		m_availableFlags[i] = false;
//	//}
//}

//void CLineProcessor::FAFilter(const float min_pos, const float max_pos)
//{
	//if (m_maxFA == m_minFA)
	//	return;

	//float min_scale = min_pos;
	//float max_scale = max_pos;
	//float min_fa = m_minFA + (m_maxFA - m_minFA) * min_scale;
	//float max_fa = m_minFA + (m_maxFA - m_minFA) * max_scale;

	//for (int i=0; i<m_lineNum; i++) {
	//	float fa = m_lines[i].GetFA();
	//	if (fa < min_fa)
	//		m_availableFlags[i] = false;
	//	else if (fa > max_fa)
	//		m_availableFlags[i] = false;
	//}
//}

//void CLineProcessor::CurvatureFilter(const float min_pos, const float max_pos)
//{
	//if (m_minCur == m_maxCur)
	//	return;

	//float min_scale = min_pos;
	//float max_scale = max_pos;
	//float min_cur = m_minCur + (m_maxCur - m_minCur) * min_scale;
	//float max_cur = m_minCur + (m_maxCur - m_minCur) * max_scale;

	//for (int i=0; i<m_lineNum; i++) {
	//	float cur = m_lines[i].GetCurvature();
	//	if (cur < min_cur)
	//		m_availableFlags[i] = false;
	//	else if (cur > max_cur)
	//		m_availableFlags[i] = false;
	//}
//}

//
//void CLineProcessor::HideLines()
//{
//	for (int i=0; i<m_lineNum; i++) {
//		if (m_selectFlags[i] == true)
//			m_showFlags[i] = false;
//	}
//	ClearSelect();
//}
//
//void CLineProcessor::HideOthers()
//{
//	for (int i=0; i<m_lineNum; i++) {
//		if (m_selectFlags[i] == false)
//			m_showFlags[i] = false;
//	}
//	ClearSelect();
//}
//
//void CLineProcessor::ShowAllLines()
//{
//	for (int i=0; i<m_lineNum; i++) {
//		m_showFlags[i] = true;
//	}
//}
//
//void CLineProcessor::ClearSelect()
//{
//	for (int i=0; i<m_lineNum; i++) {
//		m_selectFlags[i] = false;
//	}
//	SetSelect();
//}
//
//void CLineProcessor::ResetSelect()
//{
//	for (int i=0; i<m_lineNum; i++) {
//		m_selectFlags[i] = m_selectFlagsBackup[i];
//	}
//}
//
//void CLineProcessor::SetSelect()
//{
//	for (int i=0; i<m_lineNum; i++)
//	{
//		m_selectFlagsBackup[i] = m_selectFlags[i];
//	}
//}
//
//void CLineProcessor::CreateNewClass()
//{
//	int count = 0;
//	for (int i=0; i<m_lineNum; i++) {
//		if (m_selectFlags[i] == true)
//			count++;
//	}
//	if (count == 0)
//		return;
//	for (int i=0; i<m_lineNum; i++)
//	{
//		if (m_selectFlags[i] == true)
//			m_results[i] = m_classNum;
//	}
//	m_classNum++;
//	ClearSelect();
//}
//
//void CLineProcessor::SetNewClassColor(Color3F color)
//{
//	int id = -1;
//	for (int i=0; i<m_lineNum; i++) {
//		if (m_selectFlags[i] == true && m_results[i] != -1) {
//			id = m_results[i];
//			break;
//		}
//	}
//	if (id == -1) {
//		g_colorMap.SetClusterColor(m_classNum, color);
//	} else {
//		g_colorMap.SetClusterColor(id, color);
//	}
//}
//
//void CLineProcessor::AddLinesToClass()
//{
//	int id = -1;
//	for (int i=0; i<m_lineNum; i++) {
//		if (m_selectFlags[i] == true && m_results[i] != -1) {
//			id = m_results[i];
//			break;
//		}
//	}
//	if (id == -1)
//		return;
//	for (int i=0; i<m_lineNum; i++) {
//		if (m_selectFlags[i] == true)
//			m_results[i] = id;
//	}
//	ClearSelect();
//}
//
//void CLineProcessor::MergeClass()
//{
//	for (int i=0; i<m_lineNum; i++) {
//		if (m_selectFlags[i] == true)
//			m_results[i] = m_classNum;
//	}
//	m_classNum++;
//	ResetResults();
//	ClearSelect();
//}
//
//void CLineProcessor::ClearClass()
//{
//	for (int i=0; i<m_lineNum; i++) {
//		if (m_selectFlags[i] == true)
//			m_results[i] = -1;
//	}
//	ResetResults();
//	ClearSelect();
//}
//
//void CLineProcessor::CheckFibersWithCube(CUBE &cube)
//{
//	Point3F center = cube.center;
//	center.x = center.x / m_scale + m_center.x;
//	center.y = center.y / m_scale + m_center.y;
//	center.z = center.z / m_scale + m_center.z;
//	float size = cube.size;
//	size = size / m_scale;
//	Point3F pt[8];
//	pt[0].x = center.x + size; pt[0].y = center.y + size; pt[0].z = center.z + size;
//	pt[1].x = center.x + size; pt[1].y = center.y - size; pt[1].z = center.z + size;
//	pt[2].x = center.x - size; pt[2].y = center.y - size; pt[2].z = center.z + size;
//	pt[3].x = center.x - size; pt[3].y = center.y + size; pt[3].z = center.z + size;
//	pt[4].x = center.x + size; pt[4].y = center.y + size; pt[4].z = center.z - size;
//	pt[5].x = center.x + size; pt[5].y = center.y - size; pt[5].z = center.z - size;
//	pt[6].x = center.x - size; pt[6].y = center.y - size; pt[6].z = center.z - size;
//	pt[7].x = center.x - size; pt[7].y = center.y + size; pt[7].z = center.z - size;
//
//	bool *mask = new bool[m_lineNum];
//	memset(mask, 0, sizeof(bool)*m_lineNum);
//	cube.lines.clear();
//	cube.lines.empty();
//
//	for (int i=0; i<m_lineNum; i++) 
//	{
//		for (int j=0; j<m_lines[i].m_pointNum; j++)
//		{
//			Point3F p = m_lines[i].m_points[j];
//			if (p.x >= center.x - size && p.x <= center.x + size && 
//				p.y >= center.y - size && p.y <= center.y + size &&
//				p.z >= center.z - size && p.z <= center.z + size) {
//					mask[i] = true;
//					cube.lines.push_back(i);
//					break;
//			}
//		}
//	}
//
//	CheckFiberXPlane(pt[0], pt[1], pt[5], pt[4], mask, cube.lines);
//	CheckFiberXPlane(pt[2], pt[3], pt[7], pt[6], mask, cube.lines);
//
//	CheckFiberYPlane(pt[0], pt[3], pt[7], pt[4], mask, cube.lines);
//	CheckFiberYPlane(pt[1], pt[2], pt[6], pt[5], mask, cube.lines);
//
//	CheckFiberZPlane(pt[0], pt[1], pt[2], pt[3], mask, cube.lines);
//	CheckFiberZPlane(pt[4], pt[5], pt[6], pt[7], mask, cube.lines);
//
//	SAFE_DELETE(mask);
//}
//
//void CLineProcessor::CheckFiberXPlane(Point3F pt1, Point3F pt2, Point3F pt3,Point3F pt4, bool *mask, vector<int> &res)
//{
//	//float x = pt1.x;
//	//for (int i=0; i<m_lineNum; i++)
//	//{
//	//	if (mask[i] == true)
//	//		continue;
//
//	//	int pointNum = m_lines[i].m_pointNum;
//	//	for (int j=1; j<pointNum; j++) 
//	//	{
//	//		Point3F sPt = m_lines[i].m_points[j-1];
//	//		Point3F ePt = m_lines[i].m_points[j];
//	//		if ((x-sPt.x) * (x-ePt.x) > 0)
//	//			continue;
//
//	//		float k = (x - sPt.x) / (ePt.x - sPt.x);
//	//		float y = k * (ePt.y - sPt.y) + sPt.y;
//	//		float z = k * (ePt.z - sPt.z) + sPt.z;
//	//		if ((y-sPt.y) * (y-ePt.y) > 0)
//	//			continue;
//	//		if ((z-sPt.z) * (z-ePt.z) > 0)
//	//			continue;
//
//	//		if ((y-pt1.y) * (y-pt3.y) > 0)
//	//			continue;
//	//		if ((z-pt1.z) * (z-pt3.z) > 0)
//	//			continue;
//
//	//		mask[i] = true;
//	//		res.push_back(i);
//	//		break;
//	//	}
//	//}
//}
//
//void CLineProcessor::CheckFiberYPlane(Point3F pt1, Point3F pt2, Point3F pt3, 
//									  Point3F pt4, bool *mask, vector<int> &res)
//{
//	//float y = pt1.y;
//	//for (int i=0; i<m_lineNum; i++) {
//	//	if (mask[i] == true)
//	//		continue;
//
//	//	int pointNum = m_lines[i].m_pointNum;
//	//	for (int j=1; j<pointNum; j++) {
//	//		Point3F sPt = m_lines[i].m_points[j-1];
//	//		Point3F ePt = m_lines[i].m_points[j];
//	//		if ((y-sPt.y) * (y-ePt.y) > 0)
//	//			continue;
//
//	//		float k = (y - sPt.y) / (ePt.y - sPt.y);
//	//		float x = k * (ePt.x - sPt.x) + sPt.x;
//	//		float z = k * (ePt.z - sPt.z) + sPt.z;
//	//		if ((x-sPt.x) * (x-ePt.x) > 0)
//	//			continue;
//	//		if ((z-sPt.z) * (z-ePt.z) > 0)
//	//			continue;
//
//	//		if ((x-pt1.x) * (x-pt3.x) > 0)
//	//			continue;
//	//		if ((z-pt1.z) * (z-pt3.z) > 0)
//	//			continue;
//
//	//		mask[i] = true;
//	//		res.push_back(i);
//	//		break;
//	//	}
//	//}
//}
//
//void CLineProcessor::CheckFiberZPlane(Point3F pt1, Point3F pt2, Point3F pt3, 
//									  Point3F pt4, bool *mask, vector<int> &res)
//{
//	//float z = pt1.z;
//	//for (int i=0; i<m_lineNum; i++) {
//	//	if (mask[i] == true)
//	//		continue;
//
//	//	int pointNum = m_lines[i].m_pointNum;
//	//	for (int j=1; j<pointNum; j++) {
//	//		Point3F sPt = m_lines[i].m_points[j-1];
//	//		Point3F ePt = m_lines[i].m_points[j];
//	//		if ((z-sPt.z) * (z-ePt.z) > 0)
//	//			continue;
//
//	//		float k = (z - sPt.z) / (ePt.z - sPt.z);
//	//		float y = k * (ePt.y - sPt.y) + sPt.y;
//	//		float x = k * (ePt.x - sPt.x) + sPt.x;
//	//		if ((y-sPt.y) * (y-ePt.y) > 0)
//	//			continue;
//	//		if ((x-sPt.x) * (x-ePt.x) > 0)
//	//			continue;
//
//	//		if ((y-pt1.y) * (y-pt3.y) > 0)
//	//			continue;
//	//		if ((x-pt1.x) * (x-pt3.x) > 0)
//	//			continue;
//
//	//		mask[i] = true;
//	//		res.push_back(i);
//	//		break;
//	//	}
//	//}
//}
//


//void CLineProcessor::CalculateDistanceMatrix()
//{
	//if (m_DMFlag == false) {
	//	SAFE_DELETE(m_DMSpace);
	//	m_DMSpace = new float[m_lineNum*m_lineNum];

	//	//int line_num = m_lineNum;
	//	//int2 *pos = new int2[line_num];
	//	//int point_num = 0;
	//	//for (int i=0; i<line_num; i++) {
	//	//	pos[i].x = point_num;
	//	//	pos[i].y = m_lines[i].m_pointNum;
	//	//	point_num += m_lines[i].m_pointNum;
	//	//}
	//	//float3 *points = new float3[point_num];
	//	//int index = 0;
	//	//for (int i=0; i<line_num; i++) {
	//	//	for (int j=0; j<m_lines[i].m_pointNum; j++) {
	//	//		points[index].x = m_lines[i].m_points[j].x;
	//	//		points[index].y = m_lines[i].m_points[j].y;
	//	//		points[index].z = m_lines[i].m_points[j].z;
	//	//		index++;
	//	//	}
	//	//}
	//	//cudaSetDevice( cutGetMaxGflopsDeviceId() );
	//	//float *res = CalculateDM(points, point_num, pos, line_num);
	//	//for (int i=0; i<m_lineNum; i++) {
	//	//	for (int j=0; j<m_lineNum; j++) {
	//	//		m_DMSpace[i*m_lineNum+j] = res[i*m_lineNum+j];
	//	//	}
	//	//}
	//	//SAFE_DELETE(points);
	//	//SAFE_DELETE(pos);
	//	//free(res);
	//	// calculate the distance matrix
	//	SYSTEMTIME start;
	//	GetLocalTime(&start);
	//	for (int i=0; i<m_lineNum; i++) {
	//		for (int j=0; j<m_lineNum; j++) {
	//			int index = i * m_lineNum + j;
	//			if (i == j) {
	//				m_DMSpace[index] = 0.0f;
	//			} else {
	//				float ds = 0.0f;
	//				ds = TwoLinesDistance(&m_lines[i], &m_lines[j], DEFAULT_TT);
	//				m_DMSpace[index] = ds;
	//			}
	//		}
	//	}
	//	SYSTEMTIME end;
	//	GetLocalTime(&end);
	//	int minute = end.wMinute - start.wMinute;
	//	int sec = end.wSecond - start.wSecond;
	//	int msec = end.wMilliseconds - start.wMilliseconds;
	//	int time = (minute * 60 + sec) * 1000 + msec;
	//	CString buffer;
	//	buffer.Format(_T("%d"), time);
	//	MessageBox(NULL, buffer, _T("DM time"), MB_OK);
	//}

	//
	//SAFE_DELETE(m_DMCurvature);
	//SAFE_DELETE(m_DMTorsion);
	//m_DMCurvature = new float[m_lineNum*m_lineNum];
	//m_DMTorsion = new float[m_lineNum*m_lineNum];

	//// calculate the curvature matrix
	//for (int i=0; i<m_lineNum; i++) {
	//	for (int j=0; j<m_lineNum; j++) {
	//		int index = i * m_lineNum + j;
	//		if (i == j) {
	//			m_DMCurvature[index] = 0.0f;
	//		} else {
	//			float dc = CurvatureDistance(&m_lines[i], &m_lines[j]);
	//			m_DMCurvature[index] = dc;
	//		}
	//	}
	//}

	//// calculate the fa matrix
	//for (int i=0; i<m_lineNum; i++) {
	//	for (int j=0; j<m_lineNum; j++) {
	//		int index = i * m_lineNum + j;
	//		//float dt = TorsionDistance(&m_lines[i], &m_lines[j]);
	//		//m_DMTorsion[index] = abs(m_lines[i].m_FA - m_lines[j].m_FA);
	//	}
	//}

	//NormalizeDistanceMatrix(m_DMSpace, m_lineNum);
	//NormalizeDistanceMatrix(m_DMCurvature, m_lineNum);
	////NormalizeDistanceMatrix(m_DMTorsion, m_lineNum);

	//m_DMFlag = true;
//}
//
//void CLineProcessor::CalculateDistanceMatrixMean()
//{
//	SAFE_DELETE(m_distanceMatrix);
//	m_distanceMatrix = new float[m_lineNum*m_lineNum];
//	for (int i=0; i<m_lineNum; i++) {
//		for (int j=0; j<m_lineNum; j++) {
//			int index1 = i * m_lineNum + j;
//			int index2 = j * m_lineNum + i;
//			m_distanceMatrix[index1] = (m_DMSpace[index1] + m_DMSpace[index2]) / 2.0f;
//		}
//	}
//}
//
//void CLineProcessor::CalculateDistanceMatrixMax()
//{
//	SAFE_DELETE(m_distanceMatrix);
//	m_distanceMatrix = new float[m_lineNum*m_lineNum];
//	for (int i=0; i<m_lineNum; i++) {
//		for (int j=0; j<m_lineNum; j++) {
//			int index1 = i * m_lineNum + j;
//			int index2 = j * m_lineNum + i;
//			m_distanceMatrix[index1] = MAX(m_DMSpace[index1], m_DMSpace[index2]);
//		}
//	}
//}
//
//void CLineProcessor::CalculateDistanceMatrixMin()
//{
//	SAFE_DELETE(m_distanceMatrix);
//	m_distanceMatrix = new float[m_lineNum*m_lineNum];
//	for (int i=0; i<m_lineNum; i++) {
//		for (int j=0; j<m_lineNum; j++) {
//			int index1 = i * m_lineNum + j;
//			int index2 = j * m_lineNum + i;
//			m_distanceMatrix[index1] = MIN(m_DMSpace[index1], m_DMSpace[index2]);
//		}
//	}
//}
//
//void CLineProcessor::CalculateDistanceMatrixFa()
//{
//	//SAFE_DELETE(m_distanceMatrix);
//	//m_distanceMatrix = new float[m_lineNum*m_lineNum];
//	//for (int i=0; i<m_lineNum; i++) {
//	//	for (int j=0; j<m_lineNum; j++) {
//	//		int index = i * m_lineNum + j;
//	//		m_distanceMatrix[index] = m_DMFa[index];
//	//	}
//	//}
//}
//
//void CLineProcessor::CalculateDistanceMatrixTrace()
//{
//	//SAFE_DELETE(m_distanceMatrix);
//	//m_distanceMatrix = new float[m_lineNum*m_lineNum];
//	//for (int i=0; i<m_lineNum; i++) {
//	//	for (int j=0; j<m_lineNum; j++) {
//	//		int index1 = i * m_lineNum + j;
//	//		int index2 = j * m_lineNum + i;
//	//		float mean = (m_DMSpace[index1] + m_DMSpace[index2]) / 2;
//	//		float cur = abs(m_lines[i].m_curvature - m_lines[j].m_curvature);
//	//		float tor = abs(m_lines[i].m_torsion - m_lines[j].m_torsion);
//	//		m_distanceMatrix[index1] = mean + alpha * cur + beta * tor;
//	//	}
//	//}
//}
//
//void CLineProcessor::CalculateDistanceMatrixWeighted(const float alpha, const float beta)
//{
//	SAFE_DELETE(m_distanceMatrix);
//	m_distanceMatrix = new float[m_lineNum*m_lineNum];
//	for (int i=0; i<m_lineNum; i++) {
//		for (int j=0; j<m_lineNum; j++) {
//			int index1 = i * m_lineNum + j;
//			int index2 = j * m_lineNum + i;
//			float mean = (m_DMSpace[index1] + m_DMSpace[index2]) / 2.0f;
//			float cur = (m_DMCurvature[index1] + m_DMCurvature[index2]) / 2.0f;
//			float tor = (m_DMTorsion[index1] + m_DMTorsion[index2]) / 2.0f;
//			m_distanceMatrix[index1] = mean + beta * cur + alpha * tor;
//		}
//	}
//}

//void CLineProcessor::CalculateDistanceMatrix()
//{
//	SAFE_DELETE(m_distanceMatrix);
//	m_distanceMatrix = new float[m_lineNum*m_lineNum];
//	omp_set_num_threads(THREAD_NUM);
//#pragma omp parallel for
//	for (int i=0; i<m_lineNum; i++) {
//		for (int j=i+1; j<m_lineNum; j++) {
//			int offset = i * m_lineNum + j;
//			m_distanceMatrix[offset] = CalculateTwoLinesDistance(&m_lines[i], 
//				&m_lines[j], DEFAULT_TT);
//		}
//		m_distanceMatrix[i*m_lineNum+i] = 0.0f;
//	}
//
//	for (int i=0; i<m_lineNum; i++) {
//		for (int j=0; j<i; j++) {
//			m_distanceMatrix[i*m_lineNum+j] = m_distanceMatrix[j*m_lineNum+i];
//		}
//	}
//}
//
//void CLineProcessor::DBCClassify(const float dis, const int param)
//{
//	m_classNum = 0;
//	LINE_GROUP *lineGroup = new LINE_GROUP[m_lineNum];
//	MakeGroup(lineGroup, dis);
//	DBC(lineGroup, param);
//	SAFE_DELETE(lineGroup);
//}
//
//void CLineProcessor::SemiClassify()
//{
//	if (m_distanceMatrix == NULL) {
//		MessageBox(NULL, _T("Please calculate the distance matrix before you do this!"),
//			_T("Warning!"), MB_OK);
//		return;
//	}
//	if (m_classNum < 1) {
//		MessageBox(NULL, _T("Please give some input before you do this!"),
//			_T("Warning!"), MB_OK);
//		return;
//	}
//	ClassifyLines();
//}
// 
//float CLineProcessor::CalculateTwoLinesDistance(CLine *line1, CLine *line2, 
//												const float t)
//{
//	SetSrcDst(line1, line2);
//	return 0;
//	//return TwoLinesDistance(line1, line2, t);
//}

//void CLineProcessor::SetSrcDst(CLine *line1, CLine *line2)
//{
//	float length1 = line1->GetLength();
//	float length2 = line2->GetLength();
//	if (length1 > length2) {
//		CLine *tmp = line1;
//		line1 = line2;
//		line2 = tmp;
//	}
//}

//float CLineProcessor::TwoLinesDistance(CLine *line1, CLine *line2, const float t)
//{
//	float dis = 0.0f;
//	for (int i=0; i<line1->m_pointNum; i++) {
//		dis += PointToLineDistance(line1->m_points[i], line2);
//	}
//	dis /= static_cast<float>(line1->m_pointNum);
//	return dis;
//}
//
//float CLineProcessor::PointToLineDistance(Point3F point, CLine *line)
//{
//	float minDis = FLT_MAX;
//	int pos = -1;
//	for (int j=0; j<line->m_pointNum; j++) {
//		Vector3F v;
//		SUB(v, point, line->m_points[j]);
//		float dis = LENGTH(v);
//		if (dis < minDis) {
//			minDis = dis;
//			pos = j;
//		}
//	}
//	return minDis;
//}
//
//float CLineProcessor::CurvatureDistance(CLine *line1, CLine *line2)
//{
//	//int num = line1->m_curvatureNum;
//	//float dis_left = 0.0f;
//	//for (int i=0; i<num; i++) {
//	//	dis_left += abs(line1->m_curvatures[i] - line2->m_curvatures[i]);
//	//}
//	//float dis_right = 0.0f;
//	//for (int i=0; i<num; i++) {
//	//	dis_right += abs(line1->m_curvatures[i] - line2->m_curvatures[num-1-i]);
//	//}
//	//float dis = MIN(dis_left, dis_right);
//	//return dis;
//	return 0;
//}

//
//float CLineProcessor::TorsionDistance(CLine *line1, CLine *line2)
//{
//	return 1.0f;
//}
//

//
//void CLineProcessor::ClassifyLines()
//{
//	float *last_energy = new float[m_lineNum];
//	float *this_energy = new float[m_lineNum];
//
//	InitEnergy(last_energy, this_energy);
//
//	while(true) {
//		if (TransformEnergy(last_energy, this_energy) != TRUE)
//			break;
//	}
//
//	SAFE_DELETE(last_energy);
//	SAFE_DELETE(this_energy);
//}
//
//void CLineProcessor::InitEnergy(float *last_energy, float *this_energy)
//{
//	//for (int i=0; i<m_lineNum; i++) {
//	//	if (m_results[i] != -1) {
//	//		last_energy[i] = 0.0f;
//	//		this_energy[i] = 0.0f;
//	//	} else {
//	//		last_energy[i] = FLT_MAX / 2.0f;
//	//		this_energy[i] = FLT_MAX / 2.0f;
//	//	}
//	//}
//}
//
//BOOL CLineProcessor::TransformEnergy(float *last_energy, float *this_energy)
//{
//	//int *this_results = new int[m_lineNum];
//	//for (int i=0; i<m_lineNum; i++)
//	//	this_results[i] = m_results[i];
//
//	//for (int i=0; i<m_lineNum; i++) {
//	//	for (int j=0; j<m_lineNum; j++) {
//	//		if (i == j)
//	//			continue;
//	//		int offset = i * m_lineNum + j;
//	//		float dis = m_distanceMatrix[offset];
//	//		float energy = last_energy[j] + dis;
//	//		if (energy < 0.0f)
//	//			DebugBreak();
//
//	//		if (energy < this_energy[i]) {
//	//			this_results[i] = m_results[j];
//	//			this_energy[i] = energy;
//	//		}
//	//	}
//	//}
//
//	//int count = 0;
//	//for (int i=0; i<m_lineNum; i++) {
//	//	last_energy[i] = this_energy[i];
//	//	count += abs(m_results[i] - this_results[i]);
//	//	m_results[i] = this_results[i];
//	//}
//	//SAFE_DELETE(this_results);
//	//
//	//if (count == 0)
//	//	return FALSE;
//	//else
//	//	return TRUE;
//	return TRUE;
//}
//
//void CLineProcessor::CreateLines(CFieldLine **lines, int *num)
//{
//	//// Calculate the line number of all classes
//	//m_lineNum = 0;
//	//for (int i=0; i<m_classNum; i++)
//	//	m_lineNum += num[i];
//
//	//// Create lines
//	//m_lines = new CLine[m_lineNum];
//	//int index = 0;
//	//for (int k=0; k<m_classNum; k++) {
//	//	for (int i=0; i<num[k]; i++) {
//	//		m_lines[index].CopyFieldLines(&(lines[k][i]));
//	//		index++;
//	//	}
//	//}
//}
//
//void CLineProcessor::CreateFlags(int *num)
//{
//	//SAFE_DELETE(m_selectFlags);
//	//SAFE_DELETE(m_selectFlagsBackup);
//	//SAFE_DELETE(m_showFlags);
//	//SAFE_DELETE(m_availableFlags);
//	//SAFE_DELETE(m_results);
//	// Create and set all flags
//	//m_selectFlags = new bool[m_lineNum];
//	//m_selectFlagsBackup = new bool[m_lineNum];
//	for (int i=0; i<m_lineNum; i++) {
//		//m_selectFlags[i] = true;
//		//m_selectFlagsBackup[i] = true;
//		//m_selectFlags[i] = false;
//		//m_selectFlagsBackup[i] = false;
//	}
//	//m_showFlags = new bool[m_lineNum];
//	//for (int i=0; i<m_lineNum; i++)
//	//	m_showFlags[i] = true;
//	//m_availableFlags = new bool[m_lineNum];
//	//for (int i=0; i<m_lineNum; i++)
//	//	m_availableFlags[i] = true;
//	//m_results = new int[m_lineNum];
//	//for (int i=0; i<m_lineNum; i++)
//	//	m_results[i] = -1;
//
//	//int index = 0;
//	//for (int k=0; k<m_classNum; k++) {
//	//	for (int i=0; i<num[k]; i++) {
//	//		m_results[index] = k;
//	//		index++;
//	//	}
//	//}
////}
//
//void CLineProcessor::SetMinMaxLength()
//{
//	// set the min, max length
//	//m_maxLength = 0.0f;
//	//m_minLength = FLT_MAX;
//	//for (int i=0; i<m_lineNum; i++) {
//	//	float length = m_lines[i].Length();
//	//	if (length > m_maxLength)
//	//		m_maxLength = length;
//	//	if (length < m_minLength)
//	//		m_minLength = length;
//	//}
//
//	//float step = (m_maxLength - m_minLength) / static_cast<float>(m_lengthHistogramNum);
//	//for (int i=0; i<=m_lengthHistogramNum; i++) {
//	//	m_lengthStep[i] = m_minLength + static_cast<float>(i) * step;
//	//}
//
//	//int *count = new int[m_lengthHistogramNum];
//	//memset(count, 0, sizeof(int)*m_lengthHistogramNum);
//	//for (int i=0; i<m_lineNum; i++) {
//	//	float length = m_lines[i].Length();
//	//	for (int j=0; j<m_lengthHistogramNum; j++) {
//	//		if (length >= m_lengthStep[j] && length < m_lengthStep[j+1])
//	//			count[j]++;
//	//	}
//	//}
//	//count[m_lengthHistogramNum-1]++;
//
//	//int min_c = 1000000;
//	//int max_c = 0;
//	//for (int i=0; i<m_lengthHistogramNum; i++) {
//	//	if (count[i] < min_c)
//	//		min_c = count[i];
//	//	if (count[i] > max_c)
//	//		max_c = count[i];
//	//}
//
//	//float d_c = static_cast<float>(max_c - min_c);
//	//memset(m_lengthHistogram, 0, sizeof(float)*m_lengthHistogramNum);
//	//for (int i=0; i<m_lengthHistogramNum; i++) {
//	//	m_lengthHistogram[i] = static_cast<float>(count[i]) / d_c;
//	//}
//
//	//SAFE_DELETE(count); 
//}
//
//void CLineProcessor::SetFA()
//{
//	//m_minFA = FLT_MAX;
//	//m_maxFA = 0.0f;
//	//for (int i=0; i<m_lineNum; i++) {
//	//	float fa = m_lines[i].GetFA();
//	//	if (fa > m_maxFA)
//	//		m_maxFA = fa;
//	//	if (fa < m_minFA)
//	//		m_minFA = fa;
//	//}
//
//	//float step = (m_maxFA - m_minFA) / static_cast<float>(m_FAHistogramNum);
//	//for (int i=0; i<=m_FAHistogramNum; i++) {
//	//	m_FAStep[i] = m_minFA + static_cast<float>(i) * step;
//	//}
//	//m_FAStep[m_FAHistogramNum] += 0.0001f;
//
//	//int *count = new int[m_FAHistogramNum];
//	//memset(count, 0, sizeof(int)*m_FAHistogramNum);
//	//for (int i=0; i<m_lineNum; i++) {
//	//	float fa = m_lines[i].GetFA();
//	//	for (int j=0; j<m_FAHistogramNum; j++) {
//	//		if (fa >= m_FAStep[j] && fa < m_FAStep[j+1])
//	//			count[j]++;
//	//	}
//	//}
//	////count[m_FAHistogramNum-1]++;
//
//	//int min_c = 1000000;
//	//int max_c = 0;
//	//for (int i=0; i<m_FAHistogramNum; i++) {
//	//	if (count[i] < min_c)
//	//		min_c = count[i];
//	//	if (count[i] > max_c)
//	//		max_c = count[i];
//	//}
//
//	//float d_c = static_cast<float>(max_c - min_c);
//	//memset(m_FAHistogram, 0, sizeof(float)*m_FAHistogramNum);
//	//for (int i=0; i<m_FAHistogramNum; i++) {
//	//	m_FAHistogram[i] = static_cast<float>(count[i]) / d_c;
//	//}
//
//	//SAFE_DELETE(count);
//}
//
//void CLineProcessor::SetCurvature()
//{
//	//m_minCur = FLT_MAX;
//	//m_maxCur = 0.0f;
//	//for (int i=0; i<m_lineNum; i++) {
//	//	float cur = m_lines[i].m_curvature;
//	//	if (cur > m_maxCur)
//	//		m_maxCur = cur;
//	//	if (cur < m_minCur)
//	//		m_minCur = cur;
//	//}
//
//	//float step = (m_maxCur - m_minCur) / static_cast<float>(m_CurHistogramNum);
//	//for (int i=0; i<=m_CurHistogramNum; i++) {
//	//	m_CurStep[i] = m_minCur + static_cast<float>(i) * step;
//	//}
//	//m_CurStep[m_CurHistogramNum] += 0.0001f;
//
//	//int *count = new int[m_CurHistogramNum];
//	//memset(count, 0, sizeof(int)*m_CurHistogramNum);
//	//for (int i=0; i<m_lineNum; i++) {
//	//	float cur = m_lines[i].m_curvature;
//	//	for (int j=0; j<m_CurHistogramNum; j++) {
//	//		if (cur >= m_CurStep[j] && cur < m_CurStep[j+1])
//	//			count[j]++;
//	//	}
//	//}
//
//	//int min_c = 1000000;
//	//int max_c = 0;
//	//for (int i=0; i<m_CurHistogramNum; i++) {
//	//	if (count[i] < min_c)
//	//		min_c = count[i];
//	//	if (count[i] > max_c)
//	//		max_c = count[i];
//	//}
//
//	//float d_c = static_cast<float>(max_c - min_c);
//	//memset(m_CurHistogram, 0, sizeof(float)*m_CurHistogramNum);
//	//for (int i=0; i<m_CurHistogramNum; i++) {
//	//	m_CurHistogram[i] = static_cast<float>(count[i]) / d_c;
//	//}
//
//	//SAFE_DELETE(count);
//}


void CLineProcessor::SetCenter()
{
	Point3F minPt;
	Point3F maxPt;
	minPt.x = minPt.y = minPt.z = FLT_MAX;
	maxPt.x = maxPt.y = maxPt.z = -FLT_MAX;

	for (int i=0; i<m_lineNum; i++) {
		int pointNum = m_lines[i].m_pointNum;
		for (int j=0; j<pointNum; j++) {
			Point3F pt = m_lines[i].m_points[j];
			if (pt.x < minPt.x)
				minPt.x = pt.x;
			if (pt.x > maxPt.x)
				maxPt.x = pt.x;
			if (pt.y < minPt.y)
				minPt.y = pt.y;
			if (pt.y > maxPt.y)
				maxPt.y = pt.y;
			if (pt.z < minPt.z)
				minPt.z = pt.z;
			if (pt.z > maxPt.z)
				maxPt.z = pt.z;
		}
	}

	float r = MAX(maxPt.x - minPt.x, maxPt.y - minPt.y);
	r = MAX(r, maxPt.z - minPt.z);
	r = r * 1.4f;

	m_scale = 1.0f / r;

	m_center.x = (minPt.x + maxPt.x) / 2.0f;
	m_center.y = (minPt.y + maxPt.y) / 2.0f;
	m_center.z = (minPt.z + maxPt.z) / 2.0f;
}

//bool CLineProcessor::CheckFlags(bool *flags, const int size, const bool mark)
//{
	//int count = 0;
	//for (int i=0; i<size; i++) {
	//	if (flags[i] == mark)
	//		count++;
	//}
	//if (count > 0)
	//	return true;
	//else
	//	return false;
//}
//
//void CLineProcessor::ResetResults()
//{
//	//int *tmp = new int[m_classNum];
//	//memset(tmp, 0, sizeof(int)*m_classNum);
//	//for (int i=0; i<m_lineNum; i++) {
//	//	int id = m_results[i];
//	//	if (id != -1)
//	//		tmp[id] = 1;
//	//}
//
//	//int *temp = new int[m_classNum];
//	//for (int i=0; i<m_classNum; i++)
//	//	temp[i] = -1;
//
//	//int prev = 0;
//	//for (int i=0; i<m_classNum; i++) {
//	//	temp[i] = prev + tmp[i];
//	//	prev = temp[i];
//	//}
//
//	//m_classNum = prev;
//	//for (int i=0; i<m_lineNum; i++) {
//	//	int id = m_results[i];
//	//	if (id != -1) {
//	//		m_results[i] = temp[id] - 1;
//	//	}
//	//}
//
//	//SAFE_DELETE(tmp);
//	//SAFE_DELETE(temp);
//}

//void CLineProcessor::MakeGroup(LINE_GROUP *lineGroup, const float dis)
//{
//	for (int i=0; i<m_lineNum; i++) {
//		lineGroup[i].lines.clear();
//		for (int j=0; j<m_lineNum; j++) {
//			if (i == j)
//				continue;
//			int index = i * m_lineNum + j;
//			if (m_distanceMatrix[index] < dis)
//				lineGroup[i].lines.push_back(j);
//		}
//	}
//}

//void CLineProcessor::DBC(LINE_GROUP *lineGroup, const int param)
//{
	//bool *flags = new bool[m_lineNum];
	//for (int i=0; i<m_lineNum; i++) {
	//	int size = static_cast<int>(lineGroup[i].lines.size());
	//	if (size >= param)
	//		flags[i] = true;
	//	else
	//		flags[i] = false;
	//}
	//bool *temp = new bool[m_lineNum];
	//memset(temp, 0, sizeof(bool)*m_lineNum);
	//while (CheckFlags(temp, m_lineNum, false) == true) {
	//	int pos = -1;
	//	int max_size = 0;
	//	for (int i=0; i<m_lineNum; i++) {
	//		int size = static_cast<int>(lineGroup[i].lines.size());
	//		if (temp[i] == false && size > max_size) {
	//			max_size = size;
	//			pos = i;
	//		}
	//	}
	//	if (pos == -1)
	//		break;

	//	temp[pos] = true;
	//	m_results[pos] = m_classNum;
	//	DBCStep(lineGroup, flags, temp, pos);
	//	m_classNum++;
	//}

	//SAFE_DELETE(flags);
	//SAFE_DELETE(temp);
//}

//void CLineProcessor::DBCStep(LINE_GROUP *lineGroup, bool *groupFlag, 
//							 bool *avaliableFlag, const int pos)
//{
//	//int size = static_cast<int>(lineGroup[pos].lines.size());
//	////avaliableFlag[pos] = false;
//	////m_results[pos] = m_classNum;
//	//for (int i=0; i<size; i++) {
//	//	int p = lineGroup[pos].lines.at(i);
//	//	if (avaliableFlag[p] == true)
//	//		continue;
//	//	m_results[p] = m_classNum;
//	//	avaliableFlag[p] = true;
//	//	if (groupFlag[p] == false)
//	//		DBCStep(lineGroup, groupFlag, avaliableFlag, p);
//	//}
//}

//
//void CLineProcessor::ClassifyMDSByKMeans(const int k)
//{
//	//if (m_points == NULL || m_pointNum == 0)
//	//{
//	//	MessageBox(NULL, _T("Please calculate the MDS before you do this!"), _T("Warning!"), MB_OK);
//	//	return;
//	//}
//
//	//int num = 0;
//	//for (int i=0; i<m_lineNum; i++)
//	//{
//	//	if (m_availableFlags[i] && m_showFlags[i])
//	//		num++;
//	//}
//	//CvMat *pInputMat = cvCreateMat(num, 1, CV_32FC2);
//	//int index = 0;
//	//int *ID = new int[num];
//	//for (int i=0; i<m_pointNum; i++)
//	//{
//	//	if (m_availableFlags[i] && m_showFlags[i])
//	//	{
//	//		cvSet1D(pInputMat, index, cvScalar(m_points[i].x, m_points[i].y));
//	//		ID[index] = i;
//	//		index++;
//	//	}
//	//}
//
//	//CvMat *pResultMat = cvCreateMat(num, 1, CV_32SC1);
//	//cvKMeans2(pInputMat, k, pResultMat, cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100, 0.0001));
//
//	//m_classNum = k;
//	//for (int i=0; i<num; i++)
//	//{
//	//	CvScalar res = cvGet1D(pResultMat, i);
//	//	int id = ID[i];
//	//	m_results[id] = static_cast<int>(res.val[0]);
//	//}
//
//	//SAFE_DELETE(ID);
//	//cvReleaseMat(&pResultMat);
//	//cvReleaseMat(&pInputMat);
//}
//
//void CLineProcessor::ClassifyMDSByMeanShift(const float r)
//{
//	//if (m_points == NULL || m_pointNum == 0) {
//	//	MessageBox(NULL, _T("Please calculate the MDS before you do this!"), _T("Warning!"), MB_OK);
//	//	return;
//	//}
//
//	//m_classNum = 0;
//	//bool *flags = new bool[m_pointNum];
//	//memset(flags, 0, sizeof(bool)*m_pointNum);
//
//	//// outer loop
//	//while(true) {
//	//	// get a point
//	//	int pos = -1;
//	//	for (int i=0; i<m_pointNum; i++) {
//	//		if (flags[i] == false) {
//	//			pos = i;
//	//			break;
//	//		}
//	//	}
//	//	if (pos == -1)
//	//		break;
//	//	Point2F center = m_points[pos];
//
//	//	// inner loop
//	//	while(true) {
//	//		Point2F new_c;
//	//		new_c.x = new_c.y = 0.0f;
//	//		int count = 0;
//	//		for (int i=0; i<m_pointNum; i++) {
//	//			if (flags[i] == true)
//	//				continue;
//	//			float dis = (m_points[i].x - center.x) * (m_points[i].x - center.x);
//	//			dis += (m_points[i].y - center.y) * (m_points[i].y - center.y);
//	//			if (dis < r) {
//	//				new_c.x += m_points[i].x;
//	//				new_c.y += m_points[i].y;
//	//				count++;
//	//			}
//	//		}
//	//		new_c.x = new_c.x / static_cast<float>(count);
//	//		new_c.y = new_c.y / static_cast<float>(count);
//	//		float d = (center.x - new_c.x) * (center.x - new_c.x);
//	//		d += (center.y - new_c.y) * (center.y - new_c.y);
//	//		if (d < 0.00000001f)
//	//			break;				// find a mass center
//	//		else
//	//			center = new_c;
//	//	}
//
//	//	for (int i=0; i<m_pointNum; i++) {
//	//		if (flags[i] == true)
//	//			continue;
//	//		float dis = (m_points[i].x - center.x) * (m_points[i].x - center.x);
//	//		dis += (m_points[i].y - center.y) * (m_points[i].y - center.y);
//	//		if (dis < r) {
//	//			flags[i] = true;
//	//			m_results[i] = m_classNum;
//	//		}
//	//	}
//	//	m_classNum++;
//	//}
//
//	//SAFE_DELETE(flags);
//}
//
//void CLineProcessor::SingleLinkage()
//{
//	//if (m_distanceMatrix == NULL) {
//	//	MessageBox(NULL, _T("Please calculate the distance matrix before you do this!"),
//	//		_T("Warning!"), MB_OK);
//	//	return;
//	//}
//	//SAFE_DELETE(m_hierachicalResults);
//	//m_hierachicalResults = new int[m_lineNum*m_lineNum];
//	//float *tempDM = new float[m_lineNum*m_lineNum];
//	//memcpy(tempDM, m_distanceMatrix, sizeof(float)*m_lineNum*m_lineNum);
//
//	//for (int i=0; i<m_lineNum; i++) {
//	//	m_hierachicalResults[i] = i;
//	//}
//
//	//for (int level=1; level<m_lineNum; level++) {
//	//	int *lastLevelResults = &(m_hierachicalResults[(level-1)*m_lineNum]);
//	//	int *thisLevelResults = &(m_hierachicalResults[level*m_lineNum]);
//
//	//	int left, right;
//	//	float dis = FindMinDistance(left, right, tempDM);
//	//	int left_class_id = lastLevelResults[left];
//	//	int right_class_id = lastLevelResults[right];
//
//	//	bool *mask = new bool[m_lineNum];
//	//	for (int i=0; i<m_lineNum; i++) {
//	//		if (lastLevelResults[i] == left_class_id 
//	//			|| lastLevelResults[i] == right_class_id) {
//	//				mask[i] = true;
//	//		} else {
//	//			mask[i] = false;
//	//		}
//	//	}
//	//	for (int i=0; i<m_lineNum; i++) {
//	//		if (mask[i] == true) {
//	//			thisLevelResults[i] = m_lineNum + 2;
//	//		} else {
//	//			thisLevelResults[i] = lastLevelResults[i];
//	//		}
//	//	}
//
//	//	ResetDM(tempDM, mask);
//	//	ResetResults(thisLevelResults);
//
//	//	SAFE_DELETE(mask);
//	//}
//
//	//SAFE_DELETE(tempDM);
//}
//
//void CLineProcessor::ChangeSingleLinkageLevel(const int level)
//{
//	//if (level >= m_lineNum || m_hierachicalResults == NULL)
//	//	return;
//	//else {
//	//	int l = m_lineNum - level - 1;
//	//	for (int i=0; i<m_lineNum; i++) {
//	//		m_results[i] = m_hierachicalResults[l*m_lineNum+i];
//	//	}
//	//	m_classNum = level;
//	//	if (m_classNum > 0) {
//	//		SAFE_DELETE(m_principalCurves);
//	//		SAFE_DELETE(m_principalPoints);
//	//		m_principalCurves = new CLine[m_classNum+1];
//	//		m_principalPoints = new Point2F[m_classNum+1];
//	//	} else {
//	//		SAFE_DELETE(m_principalCurves);
//	//		SAFE_DELETE(m_principalPoints);
//	//	}
//	//}
//}


//float CLineProcessor::FindMinDistance(int &left, int &right, float *dm)
//{
//	//float min_dis = FLT_MAX;
//	//for (int i=0; i<m_lineNum; i++) {
//	//	for (int j=i+1; j<m_lineNum; j++) {
//	//		int index = i * m_lineNum + j;
//	//		if (dm[index] < min_dis) {
//	//			min_dis = dm[index];
//	//			left = i;
//	//			right = j;
//	//		}
//	//	}
//	//}
//	//return min_dis;
//}
//
//void CLineProcessor::ResetDM(float *dm, bool *mask)
//{
//	//for (int i=0; i<m_lineNum; i++) {
//	//	for (int j=0; j<m_lineNum; j++) {
//	//		int index = i * m_lineNum + j;
//	//		if (i == j) {
//	//			dm[index] = 0.0f;
//	//			continue;
//	//		}
//	//		bool i_mask = mask[i];
//	//		bool j_mask = mask[j];
//	//		if (i_mask == true && j_mask == true) {
//	//			dm[index] = FLT_MAX;
//	//		} else if (i_mask == true && j_mask == false) {
//	//			dm[index] = FindMin(dm, mask, j);
//	//		} else if (i_mask == false && j_mask == true) {
//	//			dm[index] = FindMin(dm, mask, i);
//	//		}
//	//	}
//	//}
//}

//float CLineProcessor::FindMin(float *dm, bool *mask, const int id)
//{
//	//float min_value = FLT_MAX;
//	//for (int i=0; i<m_lineNum; i++) {
//	//	int index = id * m_lineNum + i;
//	//	if (mask[i] == true && dm[index] < min_value) {
//	//		min_value = dm[index];
//	//	}
//	//}
//	//return min_value;
//}
//
//void CLineProcessor::ResetResults(int *results)
//{
//	//int max_value = 0;
//	//for (int i=0; i<m_lineNum; i++) {
//	//	if (results[i] > max_value)
//	//		max_value = results[i];
//	//}
//	//max_value++;
//
//	//int *tmp = new int[max_value];
//	//memset(tmp, 0, sizeof(int)*max_value);
//	//for (int i=0; i<m_lineNum; i++) {
//	//	int id = results[i];
//	//	tmp[id] = 1;
//	//}
//
//	//int *temp = new int[max_value];
//	//for (int i=0; i<max_value; i++)
//	//	temp[i] = -1;
//	//int prev = 0;
//	//for (int i=0; i<max_value; i++) {
//	//	temp[i] = prev + tmp[i];
//	//	prev = temp[i];
//	//}
//
//	//for (int i=0; i<m_lineNum; i++) {
//	//	int id = results[i];
//	//	results[i] = temp[id] - 1;
//	//}
//
//	//SAFE_DELETE(tmp);
//	//SAFE_DELETE(temp);
//}
//
//void CLineProcessor::CalculatePrincipalCurve(const bool flag)
//{
////	int classID = -1;
////	for (int i=0; i<m_lineNum; i++) {
////		if (m_selectFlags[i] == true) {
////			classID = m_results[i];
////			break;
////		}
////	}
////	if (classID == -1)
////		return;
////
////	vector<int> src;
////	for (int i=0; i<m_lineNum; i++) {
////		if (m_results[i] == classID)
////			src.push_back(i);
////	}
////	vector<Point3F> res;
////	PrincipalCurve(src, res);
////
////	CLine p;
////	m_principalCurves[classID].CreateFieldLine(res);
////	//m_principalCurve.push_back(p);
////	//m_principalCurveID.push_back(classID);
////
////	for (int i=0; i<m_lineNum; i++) {
////		if (m_results[i] == classID)
////			m_showFlags[i] = false;
////	}
//////	ClearSelect();
////
////	CalculateMDSWithPrincipalCurves(flag);
//}

//void CLineProcessor::ClearAllPrincipalCurves()
//{
////	SAFE_DELETE(m_principalCurves);
//////	SAFE_DELETE(m_principalPoints);
////
////	m_principalCurves = new CLine[m_classNum+1];
/////	m_principalPoints = new Point2F[m_classNum+1];
//}
//
//void CLineProcessor::PrincipalCurve(vector<int> &src, vector<Point3F> &res)
//{
//	//int lineNum = static_cast<int>(src.size());
//	//int pointNum = 0;
//	//for (int i=0; i<lineNum; i++) {
//	//	int id = src.at(i);
//	//	pointNum += m_lines[id].m_pointNum;
//	//}
//	//pointNum /= lineNum;
//
//	//bool *flags = new bool[lineNum];
//	//{
//	//	flags[0] = true;
//	//	Point3F s_pt = m_lines[src.at(0)].m_points[0];
//	//	/*Point3F e_pt = m_lines[src.at(0)].m_points[m_lines[src.at(0)].m_pointNum-1];*/
//	//	for (int i=1; i<lineNum; i++) {
//	//		int id = src.at(i);
//	//		int num = m_lines[id].m_pointNum;
//	//		Point3F pt1 = m_lines[id].m_points[0];
//	//		Point3F pt2 = m_lines[id].m_points[num-1];
//	//		float dis1 = abs(pt1.x - s_pt.x) + abs(pt1.y - s_pt.y) + abs(pt1.z - s_pt.z);
//	//		float dis2 = abs(pt2.x - s_pt.x) + abs(pt2.y - s_pt.y) + abs(pt2.z - s_pt.z);
//	//		if (dis1 > dis2)
//	//			flags[i] = false;
//	//		else
//	//			flags[i] = true;
//	//	}
//	//}
//
//	//Point3F *points = new Point3F[pointNum];
//	//for (int i=0; i<pointNum; i++) {
//	//	points[i].x = points[i].y = points[i].z = 0.0f;
//	//	for (int j=0; j<lineNum; j++) {
//	//		int id = src.at(j);
//	//		int num = m_lines[id].m_pointNum;
//	//		float scale = static_cast<float>(i) / static_cast<float>(pointNum-1);
//	//		if (flags[j] == false)
//	//			scale = 1.0f - scale;
//	//		int first_id = static_cast<int>(ceil(scale * static_cast<float>(num - 1)));
//	//		int second_id = static_cast<int>(floor(scale * static_cast<float>(num - 1)));
//	//		if (first_id == second_id)  {
//	//			points[i].x += m_lines[id].m_points[first_id].x;
//	//			points[i].y += m_lines[id].m_points[first_id].y;
//	//			points[i].z += m_lines[id].m_points[first_id].z;
//	//		} else {
//	//			float param1 = static_cast<float>(first_id) / static_cast<float>((num - 1));
//	//			float param2 = static_cast<float>(second_id) / static_cast<float>((num - 1));
//	//			float dp = abs(param2 - param1);
//	//			param1 = abs(param1 - scale) / dp;
//	//			param2 = abs(param2 - scale) / dp;
//	//			points[i].x += param2 * m_lines[id].m_points[first_id].x + param1 * m_lines[id].m_points[second_id].x;
//	//			points[i].y += param2 * m_lines[id].m_points[first_id].y + param1 * m_lines[id].m_points[second_id].y;
//	//			points[i].z += param2 * m_lines[id].m_points[first_id].z + param1 * m_lines[id].m_points[second_id].z;
//	//		}
//	//	}
//	//	points[i].x /= static_cast<float>(lineNum);
//	//	points[i].y /= static_cast<float>(lineNum);
//	//	points[i].z /= static_cast<float>(lineNum);
//	//}
//
//	//for (int i=0; i<pointNum; i++) {
//	//	res.push_back(points[i]);
//	//}
//
//	//SAFE_DELETE(flags);
//	//SAFE_DELETE(points);
//}

//void CLineProcessor::CalculateMDS()
//{
//	//if (m_distanceMatrix == NULL) {
//	//	MessageBox(NULL, _T("Please calculate the distance matrix before you do this!"),
//	//		_T("Warning!"), MB_OK);
//	//	return;
//	//}
//
//	//SAFE_DELETE(m_points);
//	//m_pointNum = m_lineNum;
//	//m_points = new Point2F[m_lineNum];
//
//	//srand((unsigned)time(NULL));
//	//if (m_configurationFlag == false) {
//	//	SAFE_DELETE(m_configuration);
//	//	m_configuration = new float[m_lineNum*3];
//	//	for (int i=0; i<m_lineNum; i++) {
//	//		m_configuration[i*3+0] = (float)rand() / (float)RAND_MAX;
//	//		m_configuration[i*3+1] = (float)rand() / (float)RAND_MAX;
//	//		m_configuration[i*3+2] = (float)rand() / (float)RAND_MAX;
//	//	}
//
//	//	m_configurationFlag = true;
//	//}
//
//	//SYSTEMTIME start;
//	//GetLocalTime(&start);
//	//Calculate2DMDS();
//	//SYSTEMTIME end;
//	//GetLocalTime(&end);
//	//int minute = end.wMinute - start.wMinute;
//	//int sec = end.wSecond - start.wSecond;
//	//int msec = end.wMilliseconds - start.wMilliseconds;
//	//int time = (minute * 60 + sec) * 1000 + msec;
//	//CString buffer;
//	//buffer.Format(_T("%d"), time);
//	//MessageBox(NULL, buffer, _T("MDS time"), MB_OK);
//
//	////MakeMDSLOD();
//}
//
//void CLineProcessor::Calculate2DMDS()
//{
//	//double gpu_time = 0.0;
//	//double cpu_time = 0.0;
//	//int max_iteration = 500;
//	//float epsilon = 10E-5;
//	//float *configuration = new float[m_lineNum*2];
//	//float *MDSResults = new float[m_lineNum*2];
//
//	////srand((unsigned)time(NULL));
//	//for (int p = 0, y = 0; y < m_lineNum; ++y) {
//	//	for (int x = 0; x < 2; ++x, ++p) {
//	//		configuration[p] = m_configuration[y*3+x];
//	//	}
//	//}
//
//	//SMACOF(MDSResults, configuration, m_distanceMatrix, m_lineNum, 2, 
//	//	&gpu_time, &cpu_time, max_iteration, epsilon);
//
//	//for (int i=0; i<m_lineNum; i++) {
//	//	m_points[i].x = MDSResults[i*2+0];
//	//	m_points[i].y = MDSResults[i*2+1];
//	//}
//
//	//SAFE_DELETE(MDSResults);
//	//SAFE_DELETE(configuration);
//
//	////NormalizePoints(m_points, m_pointNum);
//}


//void CLineProcessor::CalculateMDSWithPrincipalCurves(const bool flag)
//{
//}


//
//void CLineProcessor::GetSelectionInformations(int &num, float &length, float &fa)
//{
//	//num = 0;
//	//length = 0.0f;
//	//fa = 0.0f;
//	//for (int i=0; i<m_lineNum; i++) {
//	//	if (m_selectFlags[i] == true) {
//	//		num++;
//	//		length += m_lines[i].m_length;
//	//		fa += m_lines[i].m_FA;
//	//	}
//	//}
//	//length = length / static_cast<float>(num);
//	//fa = fa / static_cast<float>(num);
//}



//
//void CLineProcessor::AbstractLines()
//{
////	if (m_abstractFlag == true) {
////		return;
////	}
////
////	// save the abstract informations
////	SAFE_DELETE(m_originalLines);
//////	SAFE_DELETE(m_originalPoints2D);
////	//SAFE_DELETE(m_originalDMSpace);
////	//SAFE_DELETE(m_originalDMCurvature);
////	//SAFE_DELETE(m_originalDMTorsion);
////	m_originalLineNum = m_originalPointNum = m_lineNum;
////	m_originalLineNum = m_lineNum;
////	// lines
////	m_originalLines = new CLine[m_originalLineNum];
////	for (int i=0; i<m_originalLineNum; i++) {
////		m_originalLines[i].CopyFieldLines(&(m_lines[i]));
////	}
////	// MDS
////	//m_originalPoints2D = new Point2F[m_originalPointNum];
////	//for (int i=0; i<m_originalPointNum; i++) {
////	//	m_originalPoints2D[i].x = m_points[i].x;
////	//	m_originalPoints2D[i].y = m_points[i].y;
////	//}
////	// distance matrix
////	//m_originalDMSpace = new float[m_originalLineNum*m_originalLineNum];
////	//m_originalDMCurvature = new float[m_originalLineNum*m_originalLineNum];
////	//m_originalDMTorsion = new float[m_originalLineNum*m_originalLineNum];
////	//memcpy(m_originalDMSpace, m_DMSpace, sizeof(float)*m_originalLineNum*m_originalLineNum);
////	//memcpy(m_originalDMCurvature, m_DMCurvature, sizeof(float)*m_originalLineNum*m_originalLineNum);
////	//memcpy(m_originalDMTorsion, m_DMTorsion, sizeof(float)*m_originalLineNum*m_originalLineNum);
////
////
////	// abstract lines
////	//bool *flags = new bool[m_lineNum];
////	//memset(flags, 0, sizeof(bool)*m_lineNum);
////	//LINE_GROUP *groups = new LINE_GROUP[m_lineNum];
////	//MakeGroup(groups, 0.04f);
////	//for (int i=0; i<m_lineNum; i++)
////	//	groups[i].lines.push_back(i);
////
////	//SAFE_DELETE(m_abstractResults);
////	m_originalLineNum = m_lineNum;
////	//m_abstractResults = new int[m_originalLineNum];
////
////	CLine *lines = new CLine[m_lineNum];
////	int index = 0;
////	//while(true) {
////	//	if (CheckFlags(flags, m_lineNum, false) == false)
////	//		break;
////	//	AbstractOneGroup(flags, groups, lines, index);
////	//	index++;
////	//}
////
////	m_lineNum = index;
////	SAFE_DELETE(m_lines);
////	m_lines = new CLine[m_lineNum];
////	for (int i=0; i<m_lineNum; i++) {
////		m_lines[i].CopyFieldLines(&(lines[i]));
////	}
////
////	SAFE_DELETE(lines);
////	//SAFE_DELETE(flags);
////	//SAFE_DELETE(groups);
////
////	// set the configuration
////	//float *configuration = new float[m_lineNum*3];
////	//int *count = new int[m_lineNum];
////	//memset(configuration, 0, sizeof(float)*m_lineNum*3);
////	//memset(count, 0, sizeof(int)*m_lineNum);
////	//for (int i=0; i<m_originalLineNum; i++) {
////	//	int id = m_abstractResults[i];
////	//	configuration[id*3+0] += m_points[i].x;
////	//	configuration[id*3+1] += m_points[i].y;
////	//	configuration[id*3+2] += m_configuration[i*3+2];
////	//	count[id]++;
////	//}
////	//for (int i=0; i<m_lineNum; i++) {
////	//	configuration[i*3+0] /= static_cast<float>(count[i]);
////	//	configuration[i*3+1] /= static_cast<float>(count[i]);
////	//	configuration[i*3+2] /= static_cast<float>(count[i]);
////	//}
////	//SAFE_DELETE(m_configuration);
////	//m_configuration = new float[m_lineNum*3];
////	//for (int i=0; i<m_lineNum; i++) {
////	//	m_configuration[i*3+0] = configuration[i*3+0];
////	//	m_configuration[i*3+1] = configuration[i*3+1];
////	//	m_configuration[i*3+2] = configuration[i*3+2];
////	//}
////	//SAFE_DELETE(configuration);
////	//SAFE_DELETE(count);
////
////	//m_DMFlag = false;
////
////	//CalculateDistanceMatrix();
////	//CalculateDistanceMatrixMean();
////	//CalculateMDS();
////
////	// save the abstract informations
////	SAFE_DELETE(m_abstractLines);
//////	SAFE_DELETE(m_abstractPoints2D);
////	//SAFE_DELETE(m_abstractDMSpace);
////	//SAFE_DELETE(m_abstractDMCurvature);
////	//SAFE_DELETE(m_abstractDMTorsion);
////	m_abstractLineNum = m_abstractPointNum = m_lineNum;
////	// lines
////	m_abstractLines = new CLine[m_abstractLineNum];
////	for (int i=0; i<m_abstractLineNum; i++) {
////		m_abstractLines[i].CopyFieldLines(&(m_lines[i]));
////	}
////	// MDS
////	//m_abstractPoints2D = new Point2F[m_abstractPointNum];
////	//for (int i=0; i<m_abstractPointNum; i++) {
////	//	m_abstractPoints2D[i].x = m_points[i].x;
////	//	m_abstractPoints2D[i].y = m_points[i].y;
////	//}
////	// distance matrix
////	//m_abstractDMSpace = new float[m_abstractPointNum*m_abstractPointNum];
////	//m_abstractDMCurvature = new float[m_abstractPointNum*m_abstractPointNum];
////	//m_abstractDMTorsion = new float[m_abstractPointNum*m_abstractPointNum];
////	//memcpy(m_abstractDMSpace, m_DMSpace, sizeof(float)*m_abstractPointNum*m_abstractPointNum);
////	//memcpy(m_abstractDMCurvature, m_DMCurvature, sizeof(float)*m_abstractPointNum*m_abstractPointNum);
////	//memcpy(m_abstractDMTorsion, m_DMTorsion, sizeof(float)*m_abstractPointNum*m_abstractPointNum);
////
////	m_abstractFlag = true;
////	m_lineLevel = LEVEL_ORIGINAL;
////	ChangeLevel();
//}
//
//void CLineProcessor::ChangeLevel()
//{
//	//if (m_abstractFlag == false)
//	//	return;
//
//	//if (m_lineLevel == LEVEL_ORIGINAL) {				// abstract to abstract
//	//	SAFE_DELETE(m_lines);
//	////	SAFE_DELETE(m_points);
//	//	//SAFE_DELETE(m_DMSpace);
//	//	//SAFE_DELETE(m_DMCurvature);
//	//	//SAFE_DELETE(m_DMTorsion);
//	//	//SAFE_DELETE(m_distanceMatrix);
//
//	//	m_lineNum = m_abstractLineNum;
//	//	m_pointNum = m_abstractPointNum;
//	//	m_pointNum = m_lineNum+1;
//	//	m_lines  = new CFieldLine[m_lineNum];
//	//	for (int i=0; i<m_lineNum; i++) {
//	//		m_lines[i].CopyFieldLines(&(m_abstractLines[i]));
//	//	}
//	//	//m_points = new Point2F[m_pointNum];
//	//	//for (int i=0; i<m_pointNum; i++) {
//	//	//	m_points[i].x = m_abstractPoints2D[i].x;
//	//	//	m_points[i].y = m_abstractPoints2D[i].y;
//	//	//}
//	//	//m_DMSpace = new float[m_lineNum*m_lineNum];
//	//	//m_DMCurvature = new float[m_lineNum*m_lineNum];
//	//	//m_DMTorsion = new float[m_lineNum*m_lineNum];
//	//	//memcpy(m_DMSpace, m_abstractDMSpace, sizeof(float)*m_lineNum*m_lineNum);
//	//	//memcpy(m_DMCurvature, m_abstractDMCurvature, sizeof(float)*m_lineNum*m_lineNum);
//	//	//memcpy(m_DMTorsion, m_abstractDMTorsion, sizeof(float)*m_lineNum*m_lineNum);
//
//	//	//int *results = new int[m_lineNum];
//	//	//for (int i=0; i<m_lineNum; i++) {
//	//	//	int id = -1;
//	//	//	for (int j=0; j<m_originalLineNum; j++) {
//	//	//		if (m_abstractResults[j] == i) {
//	//	//			id = m_results[j];
//	//	//		}
//	//	//	}
//	//	//	results[i] = id;
//	//	//}
//	//	//CreateFlags(NULL);
//	//	//SAFE_DELETE(m_results);
//	//	//m_results = new int[m_lineNum];
//	//	//memcpy(m_results, results, sizeof(int)*m_lineNum);
//	//	//SAFE_DELETE(results);
//
//	//	m_lineLevel = LEVEL_ABSTRACT;
//
//	//} 
//	//else if (m_lineLevel == LEVEL_ABSTRACT) {			// abstract to abstract
//		SAFE_DELETE(m_lines);
//	//	SAFE_DELETE(m_points);
//		//SAFE_DELETE(m_DMSpace);
//		//SAFE_DELETE(m_DMCurvature);
//		//SAFE_DELETE(m_DMTorsion);
//		//SAFE_DELETE(m_distanceMatrix);
//
//		m_lineNum = m_originalLineNum;
//		m_pointNum = m_originalPointNum;
//		m_pointNum = m_lineNum+1;
//		m_lines  = new CFieldLine[m_lineNum];
//		for (int i=0; i<m_lineNum; i++) 
//		{
//			m_lines[i].CopyFieldLines(&(m_originalLines[i]));
//		}
//		//m_points = new Point2F[m_pointNum];
//		//for (int i=0; i<m_pointNum; i++) {
//		//	m_points[i].x = m_originalPoints2D[i].x;
//		//	m_points[i].y = m_originalPoints2D[i].y;
//		//}
//		//m_DMSpace = new float[m_lineNum*m_lineNum];
//		//m_DMCurvature = new float[m_lineNum*m_lineNum];
//		//m_DMTorsion = new float[m_lineNum*m_lineNum];
//		//memcpy(m_DMSpace, m_originalDMSpace, sizeof(float)*m_lineNum*m_lineNum);
//		//memcpy(m_DMCurvature, m_originalDMCurvature, sizeof(float)*m_lineNum*m_lineNum);
//		//memcpy(m_DMTorsion, m_originalDMTorsion, sizeof(float)*m_lineNum*m_lineNum);
//		
//		//int *results = new int[m_originalLineNum];
//		//for (int i=0; i<m_originalLineNum; i++) {
//		//	int id = m_abstractResults[i];
//		//	results[i] = m_results[id];
//		//}
//		//CreateFlags(NULL);
//		//SAFE_DELETE(m_results);
//		//m_results = new int[m_lineNum];
//		//memcpy(m_results, results, sizeof(int)*m_lineNum);
//		//SAFE_DELETE(results);
//
//		m_lineLevel = LEVEL_ORIGINAL;
//	//} 
//	//else 
//	//{
//	//	return;
//	//}
//
////	CalculateDistanceMatrixMean();
//
//	SetMinMaxLength();
//	SetCenter();
//	//SetFA();
//	//SetCurvature();
//
//	CreateVertexBuffer();
//	CreateColorBuffer();
//
//	// set the configuration
//	//SAFE_DELETE(m_configuration);
//	//m_configuration = new float[m_pointNum*3];
//	//srand((unsigned)time(NULL));
//	//for (int i=0; i<m_pointNum; i++) {
//	//	m_configuration[i*3+0] = m_points[i].x;
//	//	m_configuration[i*3+1] = m_points[i].y;
//	//	m_configuration[i*3+2] = (float)rand() / (float)RAND_MAX;
//	//}
//	//m_configurationFlag = true;
//}

void CLineProcessor::LoadOriginalFieldLines()
{
			SAFE_DELETE(m_lines);
	//	SAFE_DELETE(m_points);
		//SAFE_DELETE(m_DMSpace);
		//SAFE_DELETE(m_DMCurvature);
		//SAFE_DELETE(m_DMTorsion);
		//SAFE_DELETE(m_distanceMatrix);

		m_lineNum = m_originalLineNum;
		m_pointNum = m_originalPointNum;
		m_pointNum = m_lineNum+1;
		m_lines  = new CFieldLine[m_lineNum];
		for (int i=0; i<m_lineNum; i++) 
		{
			m_lines[i].CopyFieldLines(&(m_originalLines[i]));
		}
		//m_points = new Point2F[m_pointNum];
		//for (int i=0; i<m_pointNum; i++) {
		//	m_points[i].x = m_originalPoints2D[i].x;
		//	m_points[i].y = m_originalPoints2D[i].y;
		//}
		//m_DMSpace = new float[m_lineNum*m_lineNum];
		//m_DMCurvature = new float[m_lineNum*m_lineNum];
		//m_DMTorsion = new float[m_lineNum*m_lineNum];
		//memcpy(m_DMSpace, m_originalDMSpace, sizeof(float)*m_lineNum*m_lineNum);
		//memcpy(m_DMCurvature, m_originalDMCurvature, sizeof(float)*m_lineNum*m_lineNum);
		//memcpy(m_DMTorsion, m_originalDMTorsion, sizeof(float)*m_lineNum*m_lineNum);
		
		//int *results = new int[m_originalLineNum];
		//for (int i=0; i<m_originalLineNum; i++) {
		//	int id = m_abstractResults[i];
		//	results[i] = m_results[id];
		//}
		//CreateFlags(NULL);
		//SAFE_DELETE(m_results);
		//m_results = new int[m_lineNum];
		//memcpy(m_results, results, sizeof(int)*m_lineNum);
		//SAFE_DELETE(results);

		m_lineLevel = LEVEL_ORIGINAL;
	//} 
	//else 
	//{
	//	return;
	//}

//	CalculateDistanceMatrixMean();

	//SetMinMaxLength();
	SetCenter();
	//SetFA();
	//SetCurvature();

	CreateVertexBuffer();
	CreateColorBuffer();
}
//
//void CLineProcessor::ProtoChangeLevel()
//{
//	//if (m_abstractFlag == false)
//	//	return;
//
//	if (m_lineLevel == LEVEL_ORIGINAL) {				// abstract to abstract
//		SAFE_DELETE(m_lines);
////		SAFE_DELETE(m_points);
//		//SAFE_DELETE(m_DMSpace);
//		//SAFE_DELETE(m_DMCurvature);
//		//SAFE_DELETE(m_DMTorsion);
//		//SAFE_DELETE(m_distanceMatrix);
//
//		m_lineNum = m_abstractLineNum;
//		m_pointNum = 0;
//		//m_pointNum = m_lineNum+1;
//		m_lines  = new CFieldLine[m_lineNum];
//		for (int i=0; i<m_lineNum; i++) {
//			m_lines[i].CopyFieldLines(&(m_abstractLines[i]));
//		}
//		//m_points = new Point2F[m_pointNum];
//		//for (int i=0; i<m_pointNum; i++) {
//		//	m_points[i].x = m_abstractPoints2D[i].x;
//		//	m_points[i].y = m_abstractPoints2D[i].y;
//		//}
//		//m_DMSpace = new float[m_lineNum*m_lineNum];
//		//m_DMCurvature = new float[m_lineNum*m_lineNum];
//		//m_DMTorsion = new float[m_lineNum*m_lineNum];
//		//memcpy(m_DMSpace, m_abstractDMSpace, sizeof(float)*m_lineNum*m_lineNum);
//		//memcpy(m_DMCurvature, m_abstractDMCurvature, sizeof(float)*m_lineNum*m_lineNum);
//		//memcpy(m_DMTorsion, m_abstractDMTorsion, sizeof(float)*m_lineNum*m_lineNum);
//
//		//int *results = new int[m_lineNum];
//		//for (int i=0; i<m_lineNum; i++) {
//		//	int id = -1;
//		//	for (int j=0; j<m_originalLineNum; j++) {
//		//		if (m_abstractResults[j] == i) {
//		//			id = m_results[j];
//		//		}
//		//	}
//		//	results[i] = id;
//		//}
//		//CreateFlags(NULL);
//		//SAFE_DELETE(m_results);
//		//m_results = new int[m_lineNum];
//		//memcpy(m_results, results, sizeof(int)*m_lineNum);
//		//SAFE_DELETE(results);
//
//		m_lineLevel = LEVEL_ABSTRACT;
//
//	} else if (m_lineLevel == LEVEL_ABSTRACT) {			// abstract to abstract
//		SAFE_DELETE(m_lines);
//	//	SAFE_DELETE(m_points);
//		//SAFE_DELETE(m_DMSpace);
//		//SAFE_DELETE(m_DMCurvature);
//		//SAFE_DELETE(m_DMTorsion);
//		//SAFE_DELETE(m_distanceMatrix);
//
//		m_lineNum = m_originalLineNum;
//		m_pointNum = 0;
//		//m_pointNum = m_lineNum+1;
//		m_lines  = new CFieldLine[m_lineNum];
//		for (int i=0; i<m_lineNum; i++) {
//			m_lines[i].CopyFieldLines(&(m_originalLines[i]));
//		}
//		//m_points = new Point2F[m_pointNum];
//		//for (int i=0; i<m_pointNum; i++) {
//		//	m_points[i].x = m_originalPoints2D[i].x;
//		//	m_points[i].y = m_originalPoints2D[i].y;
//		//}
//		//m_DMSpace = new float[m_lineNum*m_lineNum];
//		//m_DMCurvature = new float[m_lineNum*m_lineNum];
//		//m_DMTorsion = new float[m_lineNum*m_lineNum];
//		//memcpy(m_DMSpace, m_originalDMSpace, sizeof(float)*m_lineNum*m_lineNum);
//		//memcpy(m_DMCurvature, m_originalDMCurvature, sizeof(float)*m_lineNum*m_lineNum);
//		//memcpy(m_DMTorsion, m_originalDMTorsion, sizeof(float)*m_lineNum*m_lineNum);
//		
//		//int *results = new int[m_originalLineNum];
//		//for (int i=0; i<m_originalLineNum; i++) {
//		//	int id = m_abstractResults[i];
//		//	results[i] = m_results[id];
//		//}
//		//CreateFlags(NULL);
//		//SAFE_DELETE(m_results);
//		//m_results = new int[m_lineNum];
//		//memcpy(m_results, results, sizeof(int)*m_lineNum);
//		//SAFE_DELETE(results);
//
//		m_lineLevel = LEVEL_ORIGINAL;
//	} else {
//		return;
//	}
//
//	//CalculateDistanceMatrixMean();
//
//	SetMinMaxLength();
//	SetCenter();
//	//SetFA();
//	//SetCurvature();
//
//	CreateVertexBuffer();
//	CreateColorBuffer();
//
//	// set the configuration
//	//SAFE_DELETE(m_configuration);
//	//m_configuration = new float[m_pointNum*3];
//	//srand((unsigned)time(NULL));
//	//for (int i=0; i<m_pointNum; i++) {
//	//	m_configuration[i*3+0] = m_points[i].x;
//	//	m_configuration[i*3+1] = m_points[i].y;
//	//	m_configuration[i*3+2] = (float)rand() / (float)RAND_MAX;
//	//}
//	//m_configurationFlag = true;
//}



//void CLineProcessor::AbstractOneGroup(bool *flags, LINE_GROUP *groups, 
//									  CLine *lines, const int index)
//{
//
	//int max_group_id = -1;
	//int max_value = 0;
	//for (int i=0; i<m_lineNum; i++) {
	//	if (flags[i] == true)
	//		continue;
	//	int value = static_cast<int>(groups[i].lines.size());
	//	if (value > max_value) {
	//		max_value = value;
	//		max_group_id = i;
	//	}
	//}

	//if (max_group_id > -1) {
	//	vector<Point3F> points;
	//	PrincipalCurve(groups[max_group_id].lines, points);
	//	lines[index].CreateFieldLine(points);
	//	lines[index].CalculateCurvatureAndTorsion();
	//	float fa = 0.0f;
	//	int size = static_cast<int>(groups[max_group_id].lines.size());
	//	for (int i=0; i<size; i++) {
	//		int id = groups[max_group_id].lines.at(i);
	//		fa += m_lines[id].m_FA;
	//	}
	//	fa = fa / static_cast<float>(size);
	//	lines[index].m_FA = fa;
	//	lines[index].m_trace = 1.0f;
	//	//lines[index].m_c[C_L] = 1.0f;
	//	//lines[index].m_c[C_P] = 1.0f;
	//	//lines[index].m_c[C_S] = 1.0f;

	//	//int size = static_cast<int>(groups[max_group_id].lines.size());
	//	for (int i=0; i<size; i++) {
	//		int id = groups[max_group_id].lines.at(i);
	//		m_abstractResults[id] = index;
	//		flags[id] = true;
	//	}
	//} else {
	//	DebugBreak();
	//}
//}



//void CLineProcessor::MakeMDSLOD()
//{
//	// save the MDS points
//	int pointNum = m_pointNum;
//	Point2F *points = new Point2F[m_pointNum];
//	memcpy(points, m_points, sizeof(Point2F)*m_pointNum);
//	// save the results
//	int classNum = m_classNum;
//	int *results_backup = new int[m_lineNum];
//	memcpy(results_backup, m_results, sizeof(int)*m_lineNum);
//
//	// lod level 1
//	ClassifyMDSByMeanShift(0.0002f);
//	SAFE_DELETE(m_pointsLevel1);
//	m_pointsLevel1 = new int[m_lineNum];
//	for (int i=0; i<m_lineNum; i++) {
//		m_pointsLevel1[i] = m_results[i];
//	}
//	m_level1Num = m_classNum;
//
//	// lod level 2
//	SAFE_DELETE(m_points);
//	m_pointNum = m_classNum;
//	m_points = new Point2F[m_pointNum];
//	int *count = new int[m_pointNum];
//	memset(m_points, 0, sizeof(Point2F)*m_pointNum);
//	memset(count, 0, sizeof(int)*m_pointNum);
//	for (int i=0; i<m_lineNum; i++) {
//		int id = m_pointsLevel1[i];
//		count[id]++;
//		m_points[id].x += points[i].x;
//		m_points[id].y += points[i].y;
//	}
//	for (int i=0; i<m_pointNum; i++) {
//		m_points[i].x /= static_cast<float>(count[i]);
//		m_points[i].y /= static_cast<float>(count[i]);
//	}
//	ClassifyMDSByMeanShift(0.01f);
//	SAFE_DELETE(m_pointsLevel2);
//	m_pointsLevel2 = new int[m_lineNum];
//	for (int i=0; i<m_pointNum; i++) {
//		int id = m_results[i];
//		for (int j=0; j<m_lineNum; j++) {
//			if (m_pointsLevel1[j] == id) {
//				m_pointsLevel2[j] = i;
//			}
//		}
//	}
//	SAFE_DELETE(count);
//	m_level2Num = m_classNum;
//
//	// reset the points
//	m_pointNum = pointNum;
//	SAFE_DELETE(m_points);
//	m_points = new Point2F[m_pointNum];
//	memcpy(m_points, points, sizeof(Point2F)*m_pointNum);
//	SAFE_DELETE(points);
//	// reset the results
//	m_classNum = classNum;
//	SAFE_DELETE(m_results);
//	m_results = new int[m_lineNum];
//	memcpy(m_results, results_backup, sizeof(int)*m_lineNum);
//	SAFE_DELETE(results_backup);
//}

//
//void CLineProcessor::NormalizePoints(Point2F *points, const int num)
//{
	//float min_x = FLT_MAX;
	//float max_x = -FLT_MAX;
	//float min_y =  FLT_MAX;
	//float max_y =  -FLT_MAX;

	//for (int i=0; i<num; i++) {
	//	float x = points[i].x;
	//	float y = points[i].y;
	//	if (x > max_x)
	//		max_x = x;
	//	if (x < min_x)
	//		min_x = x;
	//	if (y > max_y)
	//		max_y = y;
	//	if (y < min_y)
	//		min_y = y;
	//}

	//for (int i=0; i<num; i++) {
	//	float x = points[i].x;
	//	float y = points[i].y;
	//	x = (x - min_x) / (max_x - min_x);
	//	y = (y - min_y) / (max_y - min_y);
	//	x = (x - 0.5f) * 1.0f;
	//	y = (y - 0.5f) * 1.0f;
	//	points[i].x = x;
	//	points[i].y = y;
	//}
//}
//
//void CLineProcessor::NormalizeDistanceMatrix(float *dm, const int num)
//{
//	float min_v = FLT_MAX;
//	float max_v = 0;
//	int size = num * num;
//	for (int i=0; i<size; i++) {
//		float v = dm[i];
//		if (v > max_v)
//			max_v = v;
//		if (v < min_v)
//			min_v = v;
//	}
//
//	for (int i=0; i<size; i++) {
//		float v = dm[i];
//		v = (v - min_v) / (max_v - min_v);
//		dm[i] = v;
//	}
//}

void CLineProcessor::Delete()
{
	SAFE_DELETE(m_lines);
	m_lineNum = 0;
//	SAFE_DELETE(m_points);
	m_pointNum = 0;
	//SAFE_DELETE(m_DMSpace);
	//SAFE_DELETE(m_DMCurvature);
	//SAFE_DELETE(m_DMTorsion);
	//SAFE_DELETE(m_distanceMatrix);
	//SAFE_DELETE(m_results);
	//m_classNum = 0;

	//SAFE_DELETE(m_selectFlags);
	//SAFE_DELETE(m_selectFlagsBackup);
	//SAFE_DELETE(m_showFlags);
	//SAFE_DELETE(m_availableFlags);

	//SAFE_DELETE(m_lengthHistogram);
	//SAFE_DELETE(m_lengthStep);

	//SAFE_DELETE(m_FAHistogram);
	//SAFE_DELETE(m_FAStep);

	//SAFE_DELETE(m_CurHistogram);
	//SAFE_DELETE(m_CurStep);

	//SAFE_DELETE(m_hierachicalResults);

	//SAFE_DELETE(m_principalCurves);
	//SAFE_DELETE(m_principalPoints);

	//SAFE_DELETE(m_configuration);

	//SAFE_DELETE(m_first);
	//SAFE_DELETE(m_vertCount);

	//SAFE_DELETE(m_colors);
	//SAFE_DELETE(m_vertices);

	//m_lineLevel = LEVEL_ORIGINAL;
	//m_abstractFlag = false;

	//SAFE_DELETE(m_abstractResults);
	//
	//SAFE_DELETE(m_originalLines);
	//SAFE_DELETE(m_originalPoints2D);
	//SAFE_DELETE(m_originalDMSpace);
	//SAFE_DELETE(m_originalDMCurvature);
	//SAFE_DELETE(m_originalDMTorsion);
	//m_originalLineNum = m_originalPointNum = 0;

	//SAFE_DELETE(m_abstractLines);
	//SAFE_DELETE(m_abstractPoints2D);
	//SAFE_DELETE(m_abstractDMSpace);
	//SAFE_DELETE(m_abstractDMCurvature);
	//SAFE_DELETE(m_abstractDMTorsion);
	//m_abstractLineNum = m_abstractPointNum = 0;

	//m_minLength = m_maxLength = 0.0f;
	//m_minFA = m_maxFA = 0.0f;
	//m_minCur = m_maxCur = 0.0f;

	//m_center.x = m_center.y = m_center.z = 0.0f;

	/*m_lengthHistogramNum = 50;
	m_lengthHistogram = new float[m_lengthHistogramNum];
	m_lengthStep = new float[m_lengthHistogramNum+1];
	memset(m_lengthHistogram, 0, sizeof(float)*m_lengthHistogramNum);

	m_FAHistogramNum = 50;
	m_FAHistogram = new float[m_FAHistogramNum];
	m_FAStep = new float[m_FAHistogramNum+1];
	memset(m_FAHistogram, 0, sizeof(float)*m_FAHistogramNum);

	m_CurHistogramNum = 50;
	m_CurHistogram = new float[m_CurHistogramNum];
	m_CurStep = new float[m_CurHistogramNum+1];
	memset(m_CurHistogram, 0, sizeof(float)*m_CurHistogramNum);*/
}
//
//int CLineProcessor::GetLines(CFieldLine **o_lines)
//{
//	//int num = 0;
//	//for (int i=0; i<m_lineNum; i++) {
//	//	if (m_availableFlags[i] && m_showFlags[i])
//	//		num++;
//	//}
//	//if (num == 0)
//	//	return 0;
//
//	//*o_lines = new CLine[num];
//	//int index = 0;
//	//for (int i=0; i<m_lineNum; i++) {
//	//	if (m_availableFlags[i] && m_showFlags[i]) {
//	//		(*o_lines)[index].CopyFieldLines(&m_lines[i]);
//	//		index++;
//	//	}
//	//}
//	//return num;
//	return 0;
////}
//
//void CLineProcessor::InitLines(CFieldLine *lines, const int num, int *results, 
//							   const int classNum)
//{
//	m_lineNum = num;
//	// read lines
//	SAFE_DELETE(m_lines);
//	m_lines = new CFieldLine[m_lineNum];
//	for (int i=0; i<m_lineNum; i++) {
//		m_lines[i].CopyFieldLines(&lines[i]);
//	}
//
//	//CreateFlags(NULL);
//	SetMinMaxLength();
//	//SetFA();
//	//SetCurvature();
//	//SetCenter();
//
//	//m_classNum = classNum;
//	//for (int i=0; i<m_lineNum; i++) {
//	//	m_results[i] = results[i];
//	//}
//
//	InitILRender();
//	CreateVertexBuffer();
//	CreateColorBuffer();
//}


//int CLineProcessor::GetSrcLines(CLine **o_lines, int **o_results)
//{
	//int bufferSize = 1024;
	//CLine *lineBuffer = new CLine[bufferSize];
	//int *results = new int[bufferSize];
	//float minDis = 0.08f;
	//int index = 0;
	//for (int i=0; i<m_classNum; i++) {
	//	CLine *tempLines = NULL;
	//	int tempNum = GroupLines(i, minDis, &tempLines);
	//	for (int j=0; j<tempNum; j++) {
	//		lineBuffer[index].CopyFieldLines(&tempLines[j]);
	//		results[index] = i;
	//		index++;
	//		if (index >= bufferSize) {
	//			MessageBox(NULL, _T("Internal Error!\nExit"), _T("Error!"), MB_OK);
	//			exit(1);
	//		}
	//	}
	//	SAFE_DELETE(tempLines);
	//}

	//*o_lines = new CLine[index];
	//*o_results = new int[index];
	//for (int i=0; i<index; i++) {
	//	(*o_lines)[i].CopyFieldLines(&lineBuffer[i]);
	//	(*o_results)[i] = results[i];
	//}

	//SAFE_DELETE(results);
	//SAFE_DELETE(lineBuffer);
	//return index;
//}

//void CLineProcessor::AtlasClassify(CLine *srcLines, int *srcResults, 
//								   const int srcNum, const int srcClassNum)
//{
	//CLine *oldLines = m_lines;
	//int oldNum = m_lineNum;
	//float *oldDM = m_distanceMatrix;

	//int num = m_lineNum + srcNum;
	//CLine *lines = new CLine[num];
	//for (int i=0; i<m_lineNum; i++) {
	//	lines[i].CopyFieldLines(&m_lines[i]);
	//}
	//for (int i=0; i<srcNum; i++) {
	//	lines[i+m_lineNum].CopyFieldLines(&srcLines[i]);
	//}
	//m_lineNum = num;
	//m_lines = lines;

	//CreateFlags(NULL);

	//SAFE_DELETE(m_results);
	//m_results = new int[m_lineNum];
	//for (int i=0; i<m_lineNum; i++) {
	//	m_results[i] = -1;
	//}
	//for (int i=0; i<srcNum; i++) {
	//	m_results[i+oldNum] = srcResults[i];
	//}


	//float *distanceMatrix = new float[m_lineNum*m_lineNum];
	////for (int i=0; i<oldNum; i++) {
	////	for (int j=0; j<oldNum; j++) {
	////		distanceMatrix[i*m_lineNum+j] = (m_DMSpace[i*oldNum+j] + m_DMSpace[j*oldNum+i]) / 2.0f;
	////	}
	////}
	//for (int i=0; i<m_lineNum; i++) {
	//	for (int j=0; j<m_lineNum; j++) {
	//		float dis1 = TwoLinesDistance(&m_lines[i], &m_lines[j], DEFAULT_TT);
	//		float dis2 = TwoLinesDistance(&m_lines[j], &m_lines[i], DEFAULT_TT);
	//		distanceMatrix[i*m_lineNum+j] = (dis1 + dis2) / 2.0f;
	//		distanceMatrix[j*m_lineNum+i] = distanceMatrix[i*m_lineNum+j];
	//	}
	//}
	//m_distanceMatrix = distanceMatrix;
	//m_classNum = srcClassNum;
	//SemiClassify();

	//SAFE_DELETE(distanceMatrix);
	//SAFE_DELETE(lines);

	//m_lines = oldLines;
	//m_lineNum = oldNum;
	//m_distanceMatrix = oldDM;

	//int *results = new int[m_lineNum];
	//for (int i=0; i<m_lineNum; i++) {
	//	results[i] = m_results[i];
	//}

	//SAFE_DELETE(m_results);
	//m_results = results;

	//CreateVertexBuffer();
	//CreateColorBuffer();
//}
//
//int CLineProcessor::GroupLines(const int classID, const float dis, CLine **o_lines)
//{
	//LINE_GROUP *groups = new LINE_GROUP[m_lineNum];
	//MakeGroup(groups, dis);
	//for (int i=0; i<m_lineNum; i++)
	//	groups[i].lines.push_back(i);

	//bool *flag = new bool[m_lineNum];
	//memset(flag, 0, sizeof(bool)*m_lineNum);

	//CLine *lineBuffer = new CLine[512];
	//int index = 0;
	//while(true) {
	//	int maxSize = 0;
	//	int pos = -1;
	//	for (int i=0; i<m_lineNum; i++) {
	//		if (m_results[i] == classID && flag[i] == false) {
	//			int tempSize = 0;
	//			int size = static_cast<int>(groups[i].lines.size());
	//			for (int j=0; j<size; j++) {
	//				int id = groups[i].lines.at(j);
	//				if (m_results[id] == classID)
	//					tempSize++;
	//			}
	//			if (tempSize > maxSize) {
	//				maxSize = tempSize;
	//				pos = i;
	//			}
	//		}
	//	}

	//	if (pos == -1)
	//		break;

	//	lineBuffer[index].CopyFieldLines(&m_lines[pos]);
	//	int size = static_cast<int>(groups[pos].lines.size());
	//	for (int i=0; i<size; i++) {
	//		int id = groups[pos].lines.at(i);
	//		flag[id] = true;
	//	}
	//	index++;
	//}

	//*o_lines = new CLine[index];
	//for (int i=0; i<index; i++) {
	//	(*o_lines)[i].CopyFieldLines(&lineBuffer[i]);
	//}

	//SAFE_DELETE(lineBuffer);
	//SAFE_DELETE(groups);
	//SAFE_DELETE(flag);

	//return index;
//}
//
//
//void CLineProcessor::FilpYZ()
//{
//	//float min_x = FLT_MAX;
//	//float max_x = -FLT_MAX;
//
//	//for (int i=0; i<m_lineNum; i++) {
//	//	int pointNum = m_lines[i].m_pointNum;
//	//	for (int j=0; j<pointNum; j++) {
//	//		float x = m_lines[i].m_points[j].x;
//	//		if (x > max_x)
//	//			max_x = x;
//	//		if (x < min_x)
//	//			min_x = x;
//	//	}
//	//}
//
//	//for (int i=0; i<m_lineNum; i++) {
//	//	int pointNum = m_lines[i].m_pointNum;
//	//	for (int j=0; j<pointNum; j++) {
//	//		float x = m_lines[i].m_points[j].x;
//	//		x = max_x - x + min_x;
//	//		m_lines[i].m_points[j].x = x;
//	//	}
//	//}
//
//	//CreateVertexBuffer();
//}
//
//void CLineProcessor::FilpXZ()
//{
//	float min_y = FLT_MAX;
//	float max_y = -FLT_MAX;
//
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float y = m_lines[i].m_points[j].y;
//			if (y > max_y)
//				max_y = y;
//			if (y < min_y)
//				min_y = y;
//		}
//	}
//
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float y = m_lines[i].m_points[j].y;
//			y = max_y - y + min_y;
//			m_lines[i].m_points[j].y = y;
//		}
//	}
//
//	CreateVertexBuffer();
//}
//
//void CLineProcessor::FilpXY()
//{
//	float min_z = FLT_MAX;
//	float max_z = -FLT_MAX;
//
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float z = m_lines[i].m_points[j].z;
//			if (z > max_z)
//				max_z = z;
//			if (z < min_z)
//				min_z = z;
//		}
//	}
//
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float z = m_lines[i].m_points[j].z;
//			z = max_z - z + min_z;
//			m_lines[i].m_points[j].z = z;
//		}
//	}
//
//	CreateVertexBuffer();
//}
//
//void CLineProcessor::IncreaseX()
//{
//	float min_x = FLT_MAX;
//	float max_x = -FLT_MAX;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float x = m_lines[i].m_points[j].x;
//			if (x > max_x)
//				max_x = x;
//			if (x < min_x)
//				min_x = x;
//		}
//	}
//	float c_x = (max_x + min_x) / 2.0f;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float x = m_lines[i].m_points[j].x;
//			float dis = x - c_x;
//			dis = dis * 1.1f;
//			x = dis + c_x;
//			m_lines[i].m_points[j].x = x;
//		}
//	}
//
//	CreateVertexBuffer();
//}
//
//void CLineProcessor::DecreaseX()
//{
//	float min_x = FLT_MAX;
//	float max_x = -FLT_MAX;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float x = m_lines[i].m_points[j].x;
//			if (x > max_x)
//				max_x = x;
//			if (x < min_x)
//				min_x = x;
//		}
//	}
//	float c_x = (max_x + min_x) / 2.0f;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float x = m_lines[i].m_points[j].x;
//			float dis = x - c_x;
//			dis = dis * 0.9f;
//			x = dis + c_x;
//			m_lines[i].m_points[j].x = x;
//		}
//	}
//
//	CreateVertexBuffer();
//}
//
//void CLineProcessor::IncreaseY()
//{
//	float min_y = FLT_MAX;
//	float max_y = -FLT_MAX;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float y = m_lines[i].m_points[j].y;
//			if (y > max_y)
//				max_y = y;
//			if (y < min_y)
//				min_y = y;
//		}
//	}
//	float c_y = (max_y + min_y) / 2.0f;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float y = m_lines[i].m_points[j].y;
//			float dis = y - c_y;
//			dis = dis * 1.1f;
//			y = dis + c_y;
//			m_lines[i].m_points[j].y = y;
//		}
//	}
//
//	CreateVertexBuffer();
//}
//
//void CLineProcessor::DecreaseY()
//{
//	float min_y = FLT_MAX;
//	float max_y = -FLT_MAX;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float y = m_lines[i].m_points[j].y;
//			if (y > max_y)
//				max_y = y;
//			if (y < min_y)
//				min_y = y;
//		}
//	}
//	float c_y = (max_y + min_y) / 2.0f;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float y = m_lines[i].m_points[j].y;
//			float dis = y - c_y;
//			dis = dis * 0.9f;
//			y = dis + c_y;
//			m_lines[i].m_points[j].y = y;
//		}
//	}
//
//	CreateVertexBuffer();
//}
//
//void CLineProcessor::IncreaseZ()
//{
//	float min_z = FLT_MAX;
//	float max_z = -FLT_MAX;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float z = m_lines[i].m_points[j].z;
//			if (z > max_z)
//				max_z = z;
//			if (z < min_z)
//				min_z = z;
//		}
//	}
//	float c_z = (max_z + min_z) / 2.0f;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float z = m_lines[i].m_points[j].z;
//			float dis = z - c_z;
//			dis = dis * 1.1f;
//			z = dis + c_z;
//			m_lines[i].m_points[j].z = z;
//		}
//	}
//
//	CreateVertexBuffer();
//}
//
//void CLineProcessor::DecreaseZ()
//{
//	float min_z = FLT_MAX;
//	float max_z = -FLT_MAX;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float z = m_lines[i].m_points[j].z;
//			if (z > max_z)
//				max_z = z;
//			if (z < min_z)
//				min_z = z;
//		}
//	}
//	float c_z = (max_z + min_z) / 2.0f;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float z = m_lines[i].m_points[j].z;
//			float dis = z - c_z;
//			dis = dis * 0.9f;
//			z = dis + c_z;
//			m_lines[i].m_points[j].z = z;
//		}
//	}
//
//	CreateVertexBuffer();
//}
//
//void CLineProcessor::TuneLeft()
//{
//	float min_x = FLT_MAX;
//	float max_x = -FLT_MAX;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float x = m_lines[i].m_points[j].x;
//			if (x > max_x)
//				max_x = x;
//			if (x < min_x)
//				min_x = x;
//		}
//	}
//	float dx = max_x - min_x;
//	float step_x = dx / 20.0f;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float x = m_lines[i].m_points[j].x;
//			x -= step_x;
//			m_lines[i].m_points[j].x = x;
//		}
//	}
//	CreateVertexBuffer();
//}
//
//void CLineProcessor::TuneRight()
//{
//	float min_x = FLT_MAX;
//	float max_x = -FLT_MAX;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float x = m_lines[i].m_points[j].x;
//			if (x > max_x)
//				max_x = x;
//			if (x < min_x)
//				min_x = x;
//		}
//	}
//	float dx = max_x - min_x;
//	float step_x = dx / 20.0f;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float x = m_lines[i].m_points[j].x;
//			x += step_x;
//			m_lines[i].m_points[j].x = x;
//		}
//	}
//	CreateVertexBuffer();
//}
//
//void CLineProcessor::TuneUp()
//{
//	float min_y = FLT_MAX;
//	float max_y = -FLT_MAX;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float y = m_lines[i].m_points[j].y;
//			if (y > max_y)
//				max_y = y;
//			if (y < min_y)
//				min_y = y;
//		}
//	}
//	float dy = max_y - min_y;
//	float step_y = dy / 20.0f;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float y = m_lines[i].m_points[j].y;
//			y -= step_y;
//			m_lines[i].m_points[j].y = y;
//		}
//	}
//	CreateVertexBuffer();
//}
//
//void CLineProcessor::TuneDown()
//{
//	float min_y = FLT_MAX;
//	float max_y = -FLT_MAX;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float y = m_lines[i].m_points[j].y;
//			if (y > max_y)
//				max_y = y;
//			if (y < min_y)
//				min_y = y;
//		}
//	}
//	float dy = max_y - min_y;
//	float step_y = dy / 20.0f;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float y = m_lines[i].m_points[j].y;
//			y += step_y;
//			m_lines[i].m_points[j].y = y;
//		}
//	}
//	CreateVertexBuffer();
//}
//
//void CLineProcessor::TuneForward()
//{
//	float min_z = FLT_MAX;
//	float max_z = -FLT_MAX;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float z = m_lines[i].m_points[j].z;
//			if (z > max_z)
//				max_z = z;
//			if (z < min_z)
//				min_z = z;
//		}
//	}
//	float dz = max_z - min_z;
//	float step_z = dz / 20.0f;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float z = m_lines[i].m_points[j].z;
//			z -= step_z;
//			m_lines[i].m_points[j].z = z;
//		}
//	}
//	CreateVertexBuffer();
//}
//
//void CLineProcessor::TuneBackward()
//{
//	float min_z = FLT_MAX;
//	float max_z = -FLT_MAX;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float z = m_lines[i].m_points[j].z;
//			if (z > max_z)
//				max_z = z;
//			if (z < min_z)
//				min_z = z;
//		}
//	}
//	float dz = max_z - min_z;
//	float step_z = dz / 20.0f;
//	for (int i=0; i<m_lineNum; i++) {
//		int pointNum = m_lines[i].m_pointNum;
//		for (int j=0; j<pointNum; j++) {
//			float z = m_lines[i].m_points[j].z;
//			z += step_z;
//			m_lines[i].m_points[j].z = z;
//		}
//	}
//	CreateVertexBuffer();
//}
//
//void CLineProcessor::GetFAStatistics(vector<float> &fas)
//{
//	//if (m_classNum == 0) {
//	//	float fa = 0.0f;
//	//	for (int i=0; i<m_lineNum; i++) {
//	//		fa += m_lines[i].m_FA;
//	//	}
//	//	fa /= (float)m_lineNum;
//	//	fas.push_back(fa);
//	//} else {
//	//	for (int iClass=0; iClass<m_classNum; iClass++) {
//	//		float fa = 0.0f;
//	//		int count = 0;
//	//		for (int i=0; i<m_lineNum; i++) {
//	//			if (m_results[i] == iClass) {
//	//				fa += m_lines[i].m_FA;
//	//				count++;
//	//			}
//	//		}
//	//		fa /= (float)count;
//	//		fas.push_back(fa);
//	//	}
//	//}
//}
//
//void CLineProcessor::FilterFiber(const float x, const float y, const float z)
//{
//	//float xx = x / m_scale + m_center.x;
//	//float yy = y / m_scale + m_center.y;
//	//float zz = z / m_scale + m_center.z;
//
//	//for (int i=0; i<m_lineNum; i++) {
//	//	int pointNum = m_lines[i].m_pointNum;
//	//	bool flag = false;
//	//	for (int j=0; j<pointNum; j++) {
//	//		Point3F pt = m_lines[i].m_points[j];
//	//		pt.x = (pt.x - m_center.x) * m_scale;
//	//		pt.y = (pt.y - m_center.y) * m_scale;
//	//		pt.z = (pt.z - m_center.z) * m_scale;
//	//		if (pt.x < x || pt.y > y || pt.z < z) {
//	//			flag = true;
//	//			break;
//	//		}
//	//	}
//	//	if (flag == true) {
//	//		m_showFlags[i] = false;
//	//	} else {
//	//		m_showFlags[i] = true;
//	//	}
//	//}
//}