/// --------------------------XY Chen--------------------------
// Build from DTI Fiber Explorer (LineProcessor.cpp)
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

// System Includes
#include <float.h>
#include <omp.h>

// Project Includes
#include "stdafx.h"
#include "FieldLineProcessor.h"
#include "MessageHandle.h"
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
extern OpenGLFont g_GLFont;

#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <conio.h>

using Eigen::MatrixXd;

typedef Eigen::Vector3d Point;

#if defined(_MSC_VER)
#include <Windows.h>
#include <process.h>
#define LockMutex EnterCriticalSection
#define UnlockMutex LeaveCriticalSection
#else
#include <unistd.h>
#include <pthread.h>
#define LockMutex pthread_mutex_lock
#define UnlockMutex pthread_mutex_unlock
#endif

#if defined(_MSC_VER)
static HANDLE pollingThread;
static CRITICAL_SECTION dataLock;
#else
static pthread_t pollingThread;
static pthread_mutex_t dataLock;
#endif

CFieldLineProcessor::CFieldLineProcessor(void)
{
	LMFILTER_WINDOWLENGTH = 5;

	MAXCURVELENGTH = 300;
	MINCURVELENGTH = 3;

	m_lines = NULL;
	m_lineNum = 0;
	m_pointNum = 0;

	m_center.x = m_center.y = m_center.z = 0.0f;

	m_first = NULL;
	m_vertCount = NULL;

	m_colors = NULL;
	m_vertices = NULL;

	m_originalLines = NULL;
	m_originalLineNum = 0;

	m_abstractLines = NULL;	
	m_abstractLineNum = 0;

	sketchXPos.clear();
	sketchYPos.clear();
	sketchCurveLength = 0;

	file2DXPos.clear();
	file2DYPos.clear();
	file2DCurveLength = 0;

	lmTipXPos.clear();
	lmTipYPos.clear();
	lmTipZPos.clear();
	lmCurveLength = 0;

	lmTipXPosC.clear();
	lmTipYPosC.clear();
	lmTipZPosC.clear();
	lmCurveLengthC = 0;

	AlmTipXPos.clear();
	AlmTipYPos.clear();
	AlmTipZPos.clear();
	AlmCurveLength = 0;

	BlmTipXPos.clear();
	BlmTipYPos.clear();
	BlmTipZPos.clear();
	BlmCurveLength = 0;

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

#if defined(_MSC_VER)
	InitializeCriticalSection(&dataLock);
	//pollingThread = (HANDLE)_beginthread(serviceMessageLoop, 0, NULL);
#else
	pthread_create(&pollingThread, NULL, serviceMessageLoop, NULL);
#endif
}

CFieldLineProcessor::~CFieldLineProcessor(void)
{
	SAFE_DELETE(m_lines);
	m_lineNum = 0;
	m_pointNum = 0;

	SAFE_DELETE(m_first);
	SAFE_DELETE(m_vertCount);

	SAFE_DELETE(m_colors);
	SAFE_DELETE(m_vertices);

	m_lineLevel = LEVEL_ORIGINAL;
	
	SAFE_DELETE(m_originalLines);
	m_originalLineNum = m_originalPointNum = 0;

	SAFE_DELETE(m_abstractLines);
	m_abstractLineNum = m_abstractPointNum = 0;

	SAFE_DELETE(ts_m_lines);
	SAFE_DELETE(ts_m_vertices);

	SAFE_DELETE(f_m_vertices);
	SAFE_DELETE(f_m_first);
	SAFE_DELETE(f_m_vertCount);
}


void CFieldLineProcessor::InitILRender()
{
	glewInit();

	m_render.setErrorCallback(NULL);
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
void CFieldLineProcessor::CreateVertexBuffer()
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
}

void CFieldLineProcessor::CreateColorBuffer()
{
	int total_num = 0;
	for (int i=0; i<m_lineNum; i++) {
		total_num += m_vertCount[i];
	}
	SAFE_DELETE(m_colors);
	m_colors = new float[total_num*4];
}

void CFieldLineProcessor::OpenConfig(const char *filename)
{
	ClearAllFilterInfo();

	FILE *fp = fopen(filename, "r");
	if (fp == NULL) {
		return;
	}

	fscanf(fp, "%d\t%d\n", &m_originalLineNum, &m_originalPointNum);

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

	fclose(fp);

	InitILRender();

	LoadOriginalFieldLines();
}

void CFieldLineProcessor::OpenOriginalLines(const char *filename)
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
	}
	fclose(fp);
}

void CFieldLineProcessor::DrawDTI()
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

		int index = 0;
		for (int i=0; i<m_lineNum; i++) 
		{

				Color3F c;

				c.r = c.g = c.b = 0.4f;
				float alpha = 1.0f;

				if (FILTERED_FLAG) alpha = 0.2f;
				for (int j=0; j<m_vertCount[i]; j++)
				{
					m_colors[index*4+0] = c.r;//1.0f;//c.r;
					m_colors[index*4+1] = c.g;//1.0f;//c.g;
					m_colors[index*4+2] = c.b;//1.0f;//c.b;
					m_colors[index*4+3] = alpha;
					index++;
				}
		}
		glColorPointer(4, GL_FLOAT, sizeof(float)*4, m_colors);

		glDepthMask(GL_FALSE);
		m_render.multiDrawArrays(m_first, m_vertCount, m_lineNum);
		glDepthMask(GL_TRUE);
	}

	glPopMatrix();
}

void CFieldLineProcessor::ClearOriginalCurve(int type)
{
	if (type==1)
	{
		lmTipXPos.clear(); lmTipYPos.clear();  lmTipZPos.clear(); lmCurveLength = 0;
		lmTipXPosC.clear(); lmTipYPosC.clear();  lmTipZPosC.clear(); lmCurveLengthC = 0;

		AlmTipXPos.clear(); AlmTipYPos.clear();  AlmTipZPos.clear(); AlmCurveLength = 0;
		BlmTipXPos.clear(); BlmTipYPos.clear();  BlmTipZPos.clear(); BlmCurveLength = 0;
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

void CFieldLineProcessor::ClearTSCurve(int type)
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

void CFieldLineProcessor::ClearCurvatureBinSeq()
{
	CurvatureBinSeq.clear(); MAX_CUR = 0; MIN_CUR = 0;
}

void CFieldLineProcessor::ClearAngleBinSeq()
{
	AngleBinSeq.clear();
}

void CFieldLineProcessor::ClearAngleInfo(int type)
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

void CFieldLineProcessor::ClearCurvatureInfo(int type)
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

void CFieldLineProcessor::ClearDTISimilarityInfo()
{
	aSimilarityForDTICurves.clear();
	cSimilarityForDTICurves.clear();
}

void CFieldLineProcessor::ClearFilteredDTICurves()
{
	SAFE_DELETE(ts_m_lines);
	SAFE_DELETE(ts_m_vertices);

	SAFE_DELETE(f_m_vertices);
	SAFE_DELETE(f_m_first);
	SAFE_DELETE(f_m_vertCount);
	f_m_lineNum = 0;
	FILTERED_FLAG = FALSE;
}

void CFieldLineProcessor::ClearAllFilterInfo()
{
	ClearOriginalCurve(1);	ClearOriginalCurve(2);	ClearOriginalCurve(3);	ClearOriginalCurve(4);
	ClearTSCurve(1);	ClearTSCurve(2);	ClearTSCurve(3);	ClearTSCurve(4);
	ClearCurvatureBinSeq();
	ClearAngleInfo(1);	ClearAngleInfo(2);	ClearAngleInfo(3);	ClearAngleInfo(4);	ClearAngleInfo(5);
	ClearCurvatureInfo(1);	ClearCurvatureInfo(2);	ClearCurvatureInfo(3);	ClearCurvatureInfo(4);	ClearCurvatureInfo(5);
	ClearDTISimilarityInfo();
	ClearFilteredDTICurves();
}

void CFieldLineProcessor::LMSlideWindowFilter(float &newX, float &newY, float &newZ)
{
	if (lmCurveLength < LMFILTER_WINDOWLENGTH)
		return;

	float sumX = 0; float sumY = 0; float sumZ = 0;

	for (int i = lmCurveLength - 1, count = 1; count <= LMFILTER_WINDOWLENGTH - 1; count++,i--)
	{
		sumX += lmTipXPos.at(i); sumY += lmTipYPos.at(i); sumZ += lmTipZPos.at(i);
	}

	newX = (newX + sumX) / LMFILTER_WINDOWLENGTH;
	newY = (newY + sumY) / LMFILTER_WINDOWLENGTH;
	newZ = (newZ + sumZ) / LMFILTER_WINDOWLENGTH;
}

void CFieldLineProcessor::UpdateLMCurve(float XPos,float YPos,float ZPos, int type)
{ 
	LMSlideWindowFilter(XPos, YPos, ZPos);

	LockMutex(&dataLock);
	if (type==0)
	{
		while (lmCurveLength > MAXCURVELENGTH)
		{
			lmTipXPos.pop_front();
			lmTipYPos.pop_front();
			lmTipZPos.pop_front();
			lmCurveLength--;
		}
		while (lmCurveLengthC > MAXCURVELENGTH)
		{
			lmTipXPosC.pop_front();
			lmTipYPosC.pop_front();
			lmTipZPosC.pop_front();
			lmCurveLengthC--;
		}
		lmTipXPos.push_back(XPos);
		lmTipYPos.push_back(YPos);
		lmTipZPos.push_back(ZPos);
		lmCurveLength++;
		lmTipXPosC.push_back(XPos);
		lmTipYPosC.push_back(YPos);
		lmTipZPosC.push_back(ZPos);
		lmCurveLengthC++;
	}
	else if (type == 1)
	{
		while (AlmCurveLength > MAXCURVELENGTH)
		{
			AlmTipXPos.pop_front();
			AlmTipYPos.pop_front();
			AlmTipZPos.pop_front();
			AlmCurveLength--;
		}
		AlmTipXPos.push_back(XPos);
		AlmTipYPos.push_back(YPos);
		AlmTipZPos.push_back(ZPos);
		AlmCurveLength++;
	}
	else if (type == 2)
	{
		while (BlmCurveLength > MAXCURVELENGTH)
		{
			BlmTipXPos.pop_front();
			BlmTipYPos.pop_front();
			BlmTipZPos.pop_front();
			BlmCurveLength--;
		}
		BlmTipXPos.push_back(XPos);
		BlmTipYPos.push_back(YPos);
		BlmTipZPos.push_back(ZPos);
		BlmCurveLength++;
	}
	UnlockMutex(&dataLock);
	//lmCurveLength = lmTipXPos.size();
	//ofstream outFile("D:\\MyAssignments\\GraduationDesign\\tmp\\lmCurveLength.txt", ios::app);
	//outFile << lmCurveLength << endl;
	//outFile.close();

}

void CFieldLineProcessor::UpdateSketchCurve(float XPos, float YPos)
{
	while (sketchXPos.size()>MAXCURVELENGTH)
	{
		LockMutex(&dataLock);
		sketchXPos.pop_front(); sketchYPos.pop_front();
		UnlockMutex(&dataLock);
	}
	LockMutex(&dataLock);
	sketchXPos.push_back(XPos);
	sketchYPos.push_back(YPos);
	UnlockMutex(&dataLock);
}

//void CFieldLineProcessor::PopLMCurve()
//{
//	LockMutex(&dataLock);
//	if (lmCurveLength>0)
//	{
//		lmTipXPos.pop_front();
//		lmTipYPos.pop_front();
//		lmTipZPos.pop_front();
//		lmCurveLength--;
//	}
//	UnlockMutex(&dataLock);
//	LockMutex(&dataLock);
//	if (lmCurveLengthC > 0)
//	{
//		lmTipXPosC.pop_front();
//		lmTipYPosC.pop_front();
//		lmTipZPosC.pop_front();
//		lmCurveLengthC--;
//	}
//	UnlockMutex(&dataLock);
//}

int CFieldLineProcessor::getLMCurveLength()
{
	return lmCurveLength;
}

int CFieldLineProcessor::getSketchCurveLength()
{
	return sketchXPos.size();
}

void CFieldLineProcessor::WriteTmpLMCurve()
{
	//if(lmTipXPos.size()<MINCURVELENGTH || lmTipXPos.size()>MAXCURVELENGTH) return;
	//ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\LMCurve_tmp.txt", ios::ate);
	//for(int i=0; i<lmTipXPos.size(); i++)
	//	outFile << lmTipXPos.at(i)<< " " << lmTipYPos.at(i) << " " << lmTipZPos.at(i) <<endl; 
	//outFile.close();
}

void CFieldLineProcessor::WriteTmpSketchCurve()
{
	if(sketchXPos.size()<MINCURVELENGTH || sketchXPos.size()>MAXCURVELENGTH) return;
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\SketchCurve_tmp.txt", ios::ate);
	for(int i=0; i<sketchXPos.size(); i++)
		outFile << sketchXPos.at(i)<< " " << sketchYPos.at(i) <<endl; 
	outFile.close();
}

void CFieldLineProcessor::WriteTmpTranslatedAndScaledLMCurve()
{
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedLMCurve_tmp.txt", ios::ate);
	outFile << 1 << endl << t_s_lmTipXPos.size() << endl << "1.000000" << endl; 
	for(int i=0; i<t_s_lmTipXPos.size(); i++)
		outFile << t_s_lmTipXPos.at(i) << " " << t_s_lmTipYPos.at(i) << " " << t_s_lmTipZPos.at(i) << endl; 
	outFile.close();
}

void CFieldLineProcessor::WriteTmpTranslatedAndScaledFile3DCurve()
{
	ofstream outFile("D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\TranslatedFile3DCurve_tmp.txt", ios::ate);
	outFile << 1 << endl << t_s_file3DXPos.size() << endl << "1.000000" << endl; 
	for(int i=0; i<t_s_file3DXPos.size(); i++)
		outFile << t_s_file3DXPos.at(i) << " " << t_s_file3DYPos.at(i) << " " << t_s_file3DZPos.at(i) << endl; 
	outFile.close();
}

void CFieldLineProcessor::WriteTmpTranslatedAndScaledDTICurves()
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

void CFieldLineProcessor::WriteLMCurve(char filename[])
{
	//string str = filename;
	//str = str + ".txt";
	//ofstream outFile(str.c_str(), ios::ate);

	//for(int i=0; i<lmTipXPos.size(); i++)
	//	outFile<<lmTipXPos.at(i)<<" " << lmTipYPos.at(i) <<" "<<lmTipZPos.at(i)<<endl; 
	//outFile.close();
}

void CFieldLineProcessor::WriteSketchCurve(char filename[])
{
	string str = filename;
	str = str + ".txt";
	ofstream outFile(str.c_str(), ios::ate);

	for(int i=0; i<sketchXPos.size(); i++)
		outFile << sketchXPos.at(i)<< " " << sketchYPos.at(i) <<endl; 
	outFile.close();
}

vector<string> CFieldLineProcessor::SplitString(const string &str, const string &pattern)
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

void CFieldLineProcessor::ReadTmpLMCurve()
{
	//string filePath = "D:\\MyAssignments\\GraduationDesign\\2.16.20.10\\DTIFiberExplorer\\build\\LMCurve_tmp.txt";
	//ifstream inFile(filePath);
	//assert(inFile.is_open());
	//string line;

	//while(getline(inFile,line))
 //   {
	//	vector<string> strs = SplitString(line," ");
	//	float x = atof(strs.at(0).c_str()); lmTipXPos.push_back(x);
	//	float y = atof(strs.at(1).c_str()); lmTipYPos.push_back(y);
	//	float z = atof(strs.at(2).c_str()); lmTipZPos.push_back(z);
 //   }
	//inFile.close();
}

void CFieldLineProcessor::ReadTmpSketchCurve()
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

void CFieldLineProcessor::ReadFile3DCurve(char filename[])
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

void CFieldLineProcessor::ReadFile2DCurve(char filename[])
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

void CFieldLineProcessor::DrawLMCurve()
{
	if(lmCurveLength<=3) return;
	glTranslatef(0.5, -1.0, 0.5); // 手指在LM移动的时候Y轴位置比较高…
	glScalef(0.01f, 0.01f, 0.01f); // 归一化（原坐标大概都是几十）
	glLineWidth(2.5f);
	glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
	glBegin(GL_LINES);
	LockMutex(&dataLock);
	for(int i=1; i< lmCurveLength; i++)
	{
		glVertex3f(lmTipXPos.at(i-1), lmTipYPos.at(i-1), lmTipZPos.at(i-1));
		glVertex3f(lmTipXPos.at(i), lmTipYPos.at(i), lmTipZPos.at(i));
	}
	glEnd();

	glBegin(GL_LINES);
	if (AlmCurveLength <= 3) return;
	glColor4f(0.0f, 1.0f, 1.0f, 0.3f);
	for (int i = 1; i < AlmCurveLength; i++)
	{
		glVertex3f(AlmTipXPos.at(i - 1), AlmTipYPos.at(i - 1), AlmTipZPos.at(i - 1));
		glVertex3f(AlmTipXPos.at(i), AlmTipYPos.at(i), AlmTipZPos.at(i));
	}
	glEnd();

	glBegin(GL_LINES);
	if (BlmCurveLength <= 3) return;
	glColor4f(1.0f, 0.0f, 1.0f, 0.3f);
	for (int i = 1; i < BlmCurveLength; i++)
	{
		glVertex3f(BlmTipXPos.at(i - 1), BlmTipYPos.at(i - 1), BlmTipZPos.at(i - 1));
		glVertex3f(BlmTipXPos.at(i),BlmTipYPos.at(i), BlmTipZPos.at(i));
	}
	glEnd();
	UnlockMutex(&dataLock);
	
}

void CFieldLineProcessor::DrawFile3DCurve()//(Matrix4fT m_Transform)
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

void CFieldLineProcessor::DrawFile2DCurve()
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

void CFieldLineProcessor::DrawSketchCurve()
{
	if(sketchXPos.size()<=3) return;

	glTranslatef(-4.8f, 0.9f, 0.0f);
	glScalef(0.003f, 0.007f, 1.0f); 
	glLineWidth(2.5f);
	glColor4f(0.1f, 0.7f, 0.2f, 1.0f);
	glBegin(GL_LINES);

	LockMutex(&dataLock); 
	for(int i=1; i<sketchXPos.size(); i++)
	{
		glVertex2f(sketchXPos.at(i-1), -sketchYPos.at(i-1));
		glVertex2f(sketchXPos.at(i), -sketchYPos.at(i));
	}
	UnlockMutex(&dataLock);
	glEnd();
}

void CFieldLineProcessor::DrawTranslatedAndScaledLMCurve()
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

void CFieldLineProcessor::DrawTranslatedAndScaledFile3DCurve()
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

void CFieldLineProcessor::DrawTranslatedAndScaledFile2DCurve()
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

void CFieldLineProcessor::DrawTranslatedAndScaledSketchCurve()
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

void CFieldLineProcessor::DrawTranslatedAndScaledAndRotatedFile2DCurve()
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

void CFieldLineProcessor::DrawTranslatedAndScaledDTICurves()
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

void CFieldLineProcessor::DrawFilteredDTICurves()
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

void CFieldLineProcessor::TranslateAndScaleLMCurve()
{
	float x0 = lmTipXPosC.at(0);
	float y0 = lmTipYPosC.at(0);
	float z0 = lmTipZPosC.at(0);
	for(int i=0; i< lmCurveLengthC; i++)
	{
		t_s_lmTipXPos.push_back(m_scale*(lmTipXPosC.at(i)-x0));
		t_s_lmTipYPos.push_back(m_scale*(lmTipYPosC.at(i)-y0));
		t_s_lmTipZPos.push_back(m_scale*(lmTipZPosC.at(i)-z0));
	}
}

void CFieldLineProcessor::TranslateAndScaleFile3DCurve()
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

void CFieldLineProcessor::TranslateAndScaleDTICurves()
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

float CFieldLineProcessor::getDegAngle(Point p1, Point p2, Point p3) 
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

void CFieldLineProcessor::CaculateAnglesOnTSLMCurve()
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

void CFieldLineProcessor::CaculateAnglesOnTSFile3DCurve()
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

void CFieldLineProcessor::CaculateAnglesOnTSFile2DCurve()
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

void CFieldLineProcessor::CaculateAnglesOnTSSketchCurve()
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
void CFieldLineProcessor::CaculateAnglesOnDTICurves()
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

void CFieldLineProcessor::CaculateAnglesOnDTICurve(int &index, int point_num, float *m_vertices, deque<int> &seq, ofstream &out1, ofstream &out2)
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

void CFieldLineProcessor::CaculateCurvatureOnTranslatedLMCurve()
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

void CFieldLineProcessor::CaculateCurvatureOnTranslatedFile3DCurve()
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

void CFieldLineProcessor::CaculateCurvatureOnTSFile2DCurve()
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

void CFieldLineProcessor::CaculateCurvatureOnTSSketchCurve()
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


void CFieldLineProcessor::CaculateCurvatureOnTranslatedAndScaledDTICurve()
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

void CFieldLineProcessor::CaculateSOnTranslatedLMCurve()
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

void CFieldLineProcessor::CaculateSOnTranslatedFile3DCurve()
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

void CFieldLineProcessor::CaculateSOnTSFile2DCurve()
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

void CFieldLineProcessor::CaculateSOnTSSketchCurve()
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

void CFieldLineProcessor::CaculateFirstOrderDiscreteDerivative(deque<float> Pos, deque<float> S, deque<float> &D1)
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

void CFieldLineProcessor::CaculateSecondOrderDiscreteDerivative(deque<float> D1, deque<float> S, deque<float> &D2)
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

void CFieldLineProcessor::CaculateCurvature(deque<float> XD1,deque<float> YD1,deque<float> ZD1, deque<float> XD2,deque<float> YD2, deque<float> ZD2, deque<float>&C)
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

void CFieldLineProcessor::MapAnglesToAngleBinSeqOnTranslatedLMCurve()
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

void CFieldLineProcessor::MapAnglesToAngleBinSeqOnTranslatedFile3DCurve()
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

void CFieldLineProcessor::MapAnglesToAngleBinSeqOnTranslatedFile2DCurve()
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

void CFieldLineProcessor::MapAnglesToAngleBinSeqOnTranslatedSketchCurve()
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

void CFieldLineProcessor::MapCurvaturesToCurvatureBinSeqOnTranslatedLMCurve()
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

void CFieldLineProcessor::MapCurvaturesToCurvatureBinSeqOnTranslatedFile3DCurve()
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

void CFieldLineProcessor::MapCurvaturesToCurvatureBinSeqOnTSFile2DCurve()
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

void CFieldLineProcessor::MapCurvaturesToCurvatureBinSeqOnTSSketchCurve()
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

void CFieldLineProcessor::MapCurvaturesToCurvatureBinSeqOnDTICurves()
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

int CFieldLineProcessor::LCSS(deque<int> A,deque<int> B)
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

void CFieldLineProcessor::GetSimilarityForDTICurves(int type)
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

void CFieldLineProcessor::FilterDTICurves(int type)
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

void CFieldLineProcessor::SetAngleSimilarityThreshold(float simThreshold)
{
	ANGLE_SIMILARITY_THRESHOLD_3D = simThreshold;
}

void CFieldLineProcessor::SetAngle2DSimilarityThreshold(float simThreshold)
{
	ANGLE_SIMILARITY_THRESHOLD_2D = simThreshold;
}


void CFieldLineProcessor::SetCurvature2DSimilarityThreshold(float simThreshold)
{
	CURVATURE_SIMILARITY_THRESHOLD_2D = simThreshold;
}

void CFieldLineProcessor::SetCurvatureSimilarityThreshold(float simThreshold)
{
	CURVATURE_SIMILARITY_THRESHOLD_3D = simThreshold;
}

void CFieldLineProcessor::InitCurvatureBin()
{
	CurvatureBinSeq.clear();
	for(int i = (int)MIN_CUR; i<=(((int)MAX_CUR)+1+5); i+=5)
		CurvatureBinSeq.push_back(i);
}

void  CFieldLineProcessor::SetGranularityOfAngleBin(int gran)
{
	AngleBinSeq.clear();
	for(int i=1; i<=360+1+gran; i+=gran) // default granularity = 10
		AngleBinSeq.push_back(i-1);
}

void  CFieldLineProcessor::SetGranularityOfCurvatureBin(int gran)
{
	if (CurvatureBinSeq.size()<=0) return;
	CurvatureBinSeq.clear();
	for(int i = (int)MIN_CUR; i<=(((int)MAX_CUR)+1+gran); i+=gran) // default granularity = 5
		CurvatureBinSeq.push_back(i);
}

void CFieldLineProcessor::TranslateAndScaleFile2DCurve()
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

void CFieldLineProcessor::TranslateAndScaleSketchCurve()
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

void CFieldLineProcessor::RotateTSCurve(Matrix4fT m_Transform, int type)
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

void CFieldLineProcessor::SetCenter()
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

void CFieldLineProcessor::LoadOriginalFieldLines()
{
	SAFE_DELETE(m_lines);

	m_lineNum = m_originalLineNum;
	m_pointNum = m_originalPointNum;
	m_pointNum = m_lineNum+1;
	m_lines  = new CFieldLine[m_lineNum];
	for (int i=0; i<m_lineNum; i++) 
	{
		m_lines[i].CopyFieldLines(&(m_originalLines[i]));
	}

	m_lineLevel = LEVEL_ORIGINAL;

	SetCenter();

	CreateVertexBuffer();
	CreateColorBuffer();
}

void CFieldLineProcessor::Delete()
{
	SAFE_DELETE(m_lines);
	m_lineNum = 0;
	m_pointNum = 0;
}