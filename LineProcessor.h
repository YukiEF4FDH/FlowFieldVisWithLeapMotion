/// ----------------------------------------------------------------
/// project: DTI Fiber Explorer
/// file: LineProcessor.h
/// Headfile of the line processor
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

#include "FieldLine.h"

#include <vector>
#include <deque>
#include <..\src\IL\ILRender.h>
#include <vector>
#include <Eigen/Dense>
#include "ArcBall.h"
using Eigen::MatrixXd;
using namespace std;
//using namespace ILines;

//#ifndef GETDIS
//#define GETDIS(dis, i, j, size) ( dis[i*size+j] )
//#endif
//
//#ifndef THREAD_NUM
//#define THREAD_NUM 4
//#endif

//const float ALPHA = 1.0f;
//const float BETA = 0.01f;
//const float STEP_LENGTH = 0.01f;
//const float FIRST_THRESHOLD = 0.00001f;
//const float SECOND_THRESHOLD = 0.0001f;
//const float DEFAULT_TT = 0.0001f;

typedef Eigen::Vector3d Point;
/* enum */
//enum POINT_SIZE {
//	POINTSIZE_NONE,
//	POINTSIZE_LENGTH,
//	POINTSIZE_FA,
//	POINTSIZE_CURVATURE
//};

enum LINE_LEVEL {
	LEVEL_ORIGINAL,
	LEVEL_ABSTRACT
};

/* STRUCT */
//struct Point2F{
//	float x;
//	float y;
//};

//struct LINE_GROUP {
//	vector<int> lines;
//};

//struct CLASS
//{
//	Color3F	color;
//	vector<int> lines;
//};
//
//struct CUBE;

/* CLASS */
class CLineProcessor
{
public:
	Point3F m_max;
	Point3F m_min;
	int MAXCURVELENGTH;
	int MINCURVELENGTH;
private:
	CFieldLine	*m_lines;				// current lines
	int		m_lineNum;				// current line number
//	Point2F *m_points;				// current 2D mds points
	int		m_pointNum;				// current mds point number

	//bool	m_DMFlag;				// distance matrix flag, true for have calculated
	//float	*m_DMSpace;				// current distance matrix of space
	//float	*m_DMCurvature;			//
	//float	*m_DMTorsion;			//
	//float	*m_DMFa;				// current distance matrix of FA
	//float	*m_distanceMatrix;		// current distance matrix

	//int		*m_results;				// current classify result
	//int		m_classNum;				// current cluster number

	Point3F	m_center;				// the center of all DTI fibers
	float m_scale;					// the scale of all DTI fibers
	// Chen===
	//float m_LMScale;
	// Chen===

	//int *m_hierachicalResults;		// current hierachical results

	//CLine *m_principalCurves;		//
//	Point2F *m_principalPoints;		//

	//int *m_pointsLevel1;
	//int m_level1Num;
	//int *m_pointsLevel2;
	//int m_level2Num;

	//bool	*m_selectFlags;			//
	//bool	*m_selectFlagsBackup;	//
	//bool	*m_showFlags;			//
	//bool	*m_availableFlags;		//

	//float	m_minLength;			// minimal length of all lines
	//float	m_maxLength;			// maximal length of all lines
	//float	*m_lengthHistogram;		// histogram of length
	//int		m_lengthHistogramNum;	//
	//float	*m_lengthStep;			//

	//float	m_minFA;				//
	//float	m_maxFA;				//
	//float	*m_FAHistogram;			//
	//int		m_FAHistogramNum;		//
	//float	*m_FAStep;

	//float	m_minCur;				//
	//float	m_maxCur;				//
	//float	*m_CurHistogram;		//
	//int		m_CurHistogramNum;		//
	//float	*m_CurStep;				//

	//float *m_configuration;			//
	//bool m_configurationFlag;		//

	//int *m_abstractResults;			// abstraction results
	//bool m_abstractFlag;			//
	LINE_LEVEL m_lineLevel;			//

	CFieldLine *m_originalLines;			//
	int m_originalLineNum;			//
//	Point2F *m_originalPoints2D;	//
	int m_originalPointNum;			//
	//float *m_originalDMSpace;		//
	//float *m_originalDMCurvature;	//
	//float *m_originalDMTorsion;		//

	CFieldLine *m_abstractLines;			//
	int m_abstractLineNum;			//
//	Point2F *m_abstractPoints2D;	//
	int m_abstractPointNum;			//
	//float *m_abstractDMSpace;		//
	//float *m_abstractDMCurvature;	//
	//float *m_abstractDMTorsion;		//

	ILines::ILRender m_render;		// i
	//GLuint m_vertexBuffer;			//
	//GLuint m_colorBuffer;			//

	int	*m_first;					//
	int	*m_vertCount;				//
	//bool m_firstTime;				//
	float *m_colors;				//
	float *m_vertices;				//

public:
	CLineProcessor(void);
	~CLineProcessor(void);

	// Render
public:
	void InitILRender();
private:
	void CreateVertexBuffer();
	void CreateColorBuffer();

	// Files
public:
//	void GetCurrentClass(CLASS *CurrentClass,const int Classify_id);
	//int OpenHHR(const char *filename);
	//void SaveHHR(const char *filename);
	//void OpenResult(const char *filename);
	//void SaveResult(const char *filename);
	//void OpenDistanceMatrix(const char *filename);
	//void SaveDistanceMatrix(const char *filename);
	//void OpenMDS(const char *filename);
	//void SaveMDS(const char *filename);

	//int OpenDataFile(const char *filename);

	//void OpenFA(const char *filename);

	void OpenConfig(const char *filename);
	//void SaveConfig(const char *filename);

	//void OpenHierarchicalResults(const char *filename);
	//void SaveHierarchicalResults(const char *filename);

	//void OpenAbstractResults(const char *filename);
	//void SaveAbstractResults(const char *filename);

	void OpenOriginalLines(const char *filename);
	//void SaveOriginalLines(const char *filename);

	//void OpenOriginalMDS(const char *filename);
	//void SaveOriginalMDS(const char *filename);

	//void OpenOriginalDistanceMatrix(const char *filename);
	//void SaveOriginalDistanceMatrix(const char *filename);

	//void OpenAbstractLines(const char *filename);
	//void SaveAbstractLines(const char *filename);

	//void OpenAbstractMDS(const char *filename);
	//void SaveAbstractMDS(const char *filename);

	//void OpenAbstractDistanceMatrix(const char *filename);
	//void SaveAbstractDistanceMatrix(const char *filename);

	void Delete();
	//int GetLines(CFieldLine **o_lines);
	//void InitLines(CFieldLine *lines, const int num, int *results, 
	//	const int classNum);

	//void GetSelectionInformations(int &num, float &length, float &fa);

private:
	//void CreateLines(CFieldLine **lines, int *num);
	//void CreateFlags(int *num);
	//void SetMinMaxLength();
	//void SetFA();
	//void SetCurvature();
	void SetCenter();


public:
	inline int GetLineNum() { return m_lineNum; }

	// OpenGL
public:

	void DrawDTI();
//	void DrawMDS(POINT_SIZE pointSize, const float alpha = 1.0f);
	//void DrawNameDTI();
	//void DrawNameMDS();

	//void DrawDTIEllipsoids();
	//void DrawDTIStreamTubes();

	//Chen===========

	//void CLineProcessor::OpenProtoConfig(const char *filename);


	deque<float> sketchXPos;
	deque<float> sketchYPos;
	int sketchCurveLength;

	deque<float> file2DXPos;
	deque<float> file2DYPos;
	int file2DCurveLength;

	deque<float> lmTipXPos;
	deque<float> lmTipYPos;
	deque<float> lmTipZPos;
	int lmCurveLength;

	deque<float> file3DXPos;
	deque<float> file3DYPos;
	deque<float> file3DZPos;
	int file3DCurveLength;

	// Translated and Scaled File 3D Curve
	deque<float> t_s_file3DXPos;
	deque<float> t_s_file3DYPos;
	deque<float> t_s_file3DZPos;
	
	// Translated and Scaled File 2D Curve
	deque<float> t_s_file2DXPos;
	deque<float> t_s_file2DYPos;
	deque<float> t_s_file2DZPos;


	// Translated and Scaled Sketch Curve
	deque<float> t_s_sketchXPos;
	deque<float> t_s_sketchYPos;
	deque<float> t_s_sketchZPos;

	// Translated and Scaled LM Curve
	deque<float> t_s_lmTipXPos;
	deque<float> t_s_lmTipYPos;
	deque<float> t_s_lmTipZPos;

	// Translated and Scaled File 2D Curve
	deque<float> t_s_r_file2DXPos;
	deque<float> t_s_r_file2DYPos;
	deque<float> t_s_r_file2DZPos;

	// Max and Min value of all curvatures (to split Curvature Bins)
	float MAX_CUR;  
	float MIN_CUR;

	// Angles/Curvatures to Bins
	deque<int> AngleBinSeq; // default granularity = 10 (1-20)
	deque<int> CurvatureBinSeq; // default granularity = 5 (1-5)

	// Translated and Scaled File 3D Curve Angle Info
	deque<float> t_s_file3DCurveAngles;  // original angles
	deque<float> t_s_file2DCurveAngles;  // original angles
	deque<float> t_s_sketchCurveAngles;  // original angles
	deque<int>	 t_s_file3DCurveAngleSeq;// angles map to bin(int)
	deque<int>	 t_s_file2DCurveAngleSeq;  // original angles
	deque<int>	 t_s_sketchCurveAngleSeq;  // original angles
	deque<float> t_s_lmCurveAngles;  // original angles
	deque<int>	 t_s_lmCurveAngleSeq;// angles map to bin(int)
	// (Original) DTI Curves Angle Info
	deque<float> DTICurveAngles;
	deque<deque<int>> DTICurvesAngleSeq;

	//  累积弦长参数Si （2<=i<=n）Info (for Curvature caculation)
	deque<float> t_s_file3DCurveS;
	deque<float> t_s_lmCurveS;
	deque<float> t_s_file2DCurveS;
	deque<float> t_s_sketchCurveS;
	deque<deque<float>> DTICurveSs;

	// num of neighbor for the target points 
	int n1;  // FirstOrderDiscreteDerivative
	int n2;  // SecondOrderDiscreteDerivative

	// Translated and Scaled File 3D Curve FirstOrderDiscreteDerivative Info
	deque<float> t_s_file3DXFirstOrderDiscreteDerivative;
	deque<float> t_s_file3DYFirstOrderDiscreteDerivative;
	deque<float> t_s_file3DZFirstOrderDiscreteDerivative;

	// Translated and Scaled File 3D Curve FirstOrderDiscreteDerivative Info
	deque<float> t_s_file2DXFirstOrderDiscreteDerivative;
	deque<float> t_s_file2DYFirstOrderDiscreteDerivative;
	deque<float> t_s_file2DZFirstOrderDiscreteDerivative;

	// Translated and Scaled LM Curve FirstOrderDiscreteDerivative Info
	deque<float> t_s_lmXFirstOrderDiscreteDerivative;
	deque<float> t_s_lmYFirstOrderDiscreteDerivative;
	deque<float> t_s_lmZFirstOrderDiscreteDerivative;

		// Translated and Scaled LM Curve FirstOrderDiscreteDerivative Info
	deque<float> t_s_sketchXFirstOrderDiscreteDerivative;
	deque<float> t_s_sketchYFirstOrderDiscreteDerivative;
	deque<float> t_s_sketchZFirstOrderDiscreteDerivative;

	// Translated and Scaled DTI Curves FirstOrderDiscreteDerivative Info
	deque<deque<float>> DTIXFirstOrderDiscreteDerivative;
	deque<deque<float>> DTIYFirstOrderDiscreteDerivative;
	deque<deque<float>> DTIZFirstOrderDiscreteDerivative;

	// Translated and File 3D Curve SecondOrderDiscreteDerivative Info
	deque<float> t_s_file3DXSecondOrderDiscreteDerivative;
	deque<float> t_s_file3DYSecondOrderDiscreteDerivative;
	deque<float> t_s_file3DZSecondOrderDiscreteDerivative;
	
	// Translated and File 3D Curve SecondOrderDiscreteDerivative Info
	deque<float> t_s_file2DXSecondOrderDiscreteDerivative;
	deque<float> t_s_file2DYSecondOrderDiscreteDerivative;
	deque<float> t_s_file2DZSecondOrderDiscreteDerivative;

	// Translated and Scaled LM Curve SecondOrderDiscreteDerivative Info
	deque<float> t_s_lmXSecondOrderDiscreteDerivative;
	deque<float> t_s_lmYSecondOrderDiscreteDerivative;
	deque<float> t_s_lmZSecondOrderDiscreteDerivative;

		// Translated and Scaled LM Curve SecondOrderDiscreteDerivative Info
	deque<float> t_s_sketchXSecondOrderDiscreteDerivative;
	deque<float> t_s_sketchYSecondOrderDiscreteDerivative;
	deque<float> t_s_sketchZSecondOrderDiscreteDerivative;

	// Translated and Scaled DTI Curve SecondOrderDiscreteDerivative Info
	deque<deque<float>> DTIXSecondOrderDiscreteDerivative;
	deque<deque<float>> DTIYSecondOrderDiscreteDerivative;
	deque<deque<float>> DTIZSecondOrderDiscreteDerivative;

	// Translated and Scaled File 3D Curve Curvature Info
	deque<float> t_s_file3DCurveCurvature;   // original curvatures
	deque<int>	 t_s_file3DCurveCurvatureSeq;// curvatures map to bin(int) (0-0,10-1,20-2...)
		// Translated and Scaled File 3D Curve Curvature Info
	deque<float> t_s_file2DCurveCurvature;   // original curvatures
	deque<int>	 t_s_file2DCurveCurvatureSeq;// curvatures map to bin(int) (0-0,10-1,20-2...)
	// Translated and Scaled LM Curve Curvature Info
	deque<float> t_s_lmCurveCurvature;   // original curvatures
	deque<int>	 t_s_lmCurveCurvatureSeq;// curvatures map to bin(int) (0-0,10-1,20-2...)
		// Translated and Scaled LM Curve Curvature Info
	deque<float> t_s_sketchCurveCurvature;   // original curvatures
	deque<int>	 t_s_sketchCurveCurvatureSeq;// curvatures map to bin(int) (0-0,10-1,20-2...)


	// Translated and Scaled DTI Curve Curvature Info
	deque<deque<float>> DTICurvesCurvature;
	deque<deque<int>> DTICurvesCurvatureSeq;

	// Similarity based on LCSS
	deque<float> aSimilarityForDTICurves; // Bin: DTI Curves Angle Seq Info & File 3D Curve Angle Seq Info
	deque<float> cSimilarityForDTICurves; // Bin: DTI Curves Curvature Seq Info & File 3D Curve Curvature Seq Info

	float ANGLE_SIMILARITY_THRESHOLD_3D;     // default = 0.5
	float CURVATURE_SIMILARITY_THRESHOLD_3D; // default = 0.1
	float ANGLE_SIMILARITY_THRESHOLD_2D;	 // default = 0.1
	float CURVATURE_SIMILARITY_THRESHOLD_2D; // default = 0.1

	bool FILTERED_FLAG; // default = false;

	// Translate and Scale original DTLs to (0,0,0) and *m_scale
	CFieldLine	*ts_m_lines;	
	float	*ts_m_vertices;

	// Filtered original DTLs
	float	*f_m_vertices;
	int		*f_m_first;
	int		*f_m_vertCount;
	int		f_m_lineNum;

	void ClearOriginalCurve(int type);
	void ClearTSCurve(int type);
	void ClearAngleBinSeq();
	void ClearCurvatureBinSeq();
	void ClearAngleInfo(int type);
	void ClearCurvatureInfo(int type);
	void ClearDTISimilarityInfo();
	void ClearFilteredDTICurves();
	void ClearAllFilterInfo();

	void UpdateLMCurve(float lmTipXPos,float lmTipYPos,float lmTipZPos);
	void UpdateSketchCurve(float sketchXPos, float sketchYPos);

	int getLMCurveLength();
	int getSketchCurveLength();

	void WriteTmpLMCurve();
	void WriteTmpSketchCurve();
	void WriteTmpTranslatedAndScaledLMCurve();
	void WriteTmpTranslatedAndScaledFile3DCurve();
	void WriteTmpTranslatedAndScaledDTICurves();

	void WriteLMCurve(char filename[]);
	void WriteSketchCurve(char filename[]);

	vector<string> SplitString(const string &str, const string &pattern);

	void ReadTmpLMCurve();
	void ReadTmpSketchCurve();

	void ReadFile3DCurve(char filename[]);
	void ReadFile2DCurve(char filename[]);

	void DrawLMCurve();
	void DrawFile3DCurve();//Matrix4fT m_Transform);
	void DrawSketchCurve();
	void DrawFile2DCurve();
	void DrawTranslatedAndScaledLMCurve();
	void DrawTranslatedAndScaledSketchCurve();
	void DrawTranslatedAndScaledFile3DCurve();
	void DrawTranslatedAndScaledFile2DCurve();
	void DrawTranslatedAndScaledAndRotatedFile2DCurve();
	void DrawTranslatedAndScaledDTICurves();
	void DrawFilteredDTICurves();

	void TranslateAndScaleLMCurve();
	void TranslateAndScaleFile3DCurve();
	void TranslateAndScaleDTICurves();
	void TranslateAndScaleFile2DCurve();
	void TranslateAndScaleSketchCurve();

	float getDegAngle(Point p1, Point p2, Point p3);
	void CaculateAnglesOnTSLMCurve();
	void CaculateAnglesOnTSFile3DCurve();
	void CaculateAnglesOnTSFile2DCurve();
	void CaculateAnglesOnTSSketchCurve();
	void CaculateAnglesOnDTICurves();
	void CaculateAnglesOnDTICurve(int &index, int point_num, float *m_vertices, deque<int> &seq, ofstream &out1, ofstream &out2);

	void CaculateCurvatureOnTranslatedLMCurve();
	void CaculateCurvatureOnTranslatedFile3DCurve();
	void CaculateCurvatureOnTranslatedAndScaledDTICurve();
	void CaculateCurvatureOnTSFile2DCurve();
	void CaculateCurvatureOnTSSketchCurve();

	void CaculateSOnTranslatedLMCurve();
	void CaculateSOnTranslatedFile3DCurve();
	void CaculateSOnTSFile2DCurve();
	void CaculateSOnTSSketchCurve();

	void CaculateFirstOrderDiscreteDerivative(deque<float> Pos, deque<float> S, deque <float> &D1);
	void CaculateSecondOrderDiscreteDerivative(deque<float> D1, deque<float> S, deque <float> &D2);
	void CaculateCurvature(deque<float> XD1,deque<float> YD1,deque<float> ZD1, deque<float> XD2,deque<float> YD2,deque<float> ZD2, deque<float> &C);

	void MapAnglesToAngleBinSeqOnTranslatedLMCurve();
	void MapAnglesToAngleBinSeqOnTranslatedFile3DCurve();
	void MapAnglesToAngleBinSeqOnTranslatedFile2DCurve();
	void MapAnglesToAngleBinSeqOnTranslatedSketchCurve();

	void MapCurvaturesToCurvatureBinSeqOnTranslatedLMCurve();
	void MapCurvaturesToCurvatureBinSeqOnTranslatedFile3DCurve();
	void MapCurvaturesToCurvatureBinSeqOnTSFile2DCurve();
	void MapCurvaturesToCurvatureBinSeqOnTSSketchCurve();
	void MapCurvaturesToCurvatureBinSeqOnDTICurves();

	int LCSS(deque<int> A,deque<int> B);

	// type = 1: Query by LM Curve; type = 2: Query by File 3D Curve; 
	// type = 3: Query by file 2d;  type = 4: Query by Sketch
	void GetSimilarityForDTICurves(int type); 
	
	void FilterDTICurves(int type);

	void SetAngleSimilarityThreshold(float simThreshold);
	void SetCurvatureSimilarityThreshold(float simThreshold);
	void SetAngle2DSimilarityThreshold(float simThreshold);
	void SetCurvature2DSimilarityThreshold(float simThreshold);

	void InitCurvatureBin(); // based on MIN_CUR & MAX_CUR

	void SetGranularityOfAngleBin(int gran);
	void SetGranularityOfCurvatureBin(int gran);

	void RotateTSCurve(Matrix4fT m_Transform, int type);
	//Chen===========

	//void DrawMDSLevel1(POINT_SIZE pointSize, const float alpha = 1.0f);
	//void DrawMDSLevel2(POINT_SIZE pointSize);

	//void DrawLinesInCube(vector<int> &lines, Color3F color);
	//void DrawPointsInCube(vector<int> &points, Color3F color);

	//void SingleSelectDTI(GLuint *selectbuffer, const GLint hits,
	//	BOOL classFlag, BOOL mode, const bool res);
	//void SingleSelectMDS(Point2F point, const float r, 
	//	const BOOL classFlag, const BOOL mode, const bool res);
	//void brushselect();

	//void ApplyCubeSelections(vector<int> &lines);

	//void DrawLength(const float left, const float right);
	//void DrawFA(const float left, const float right);
	//void DrawCurvature(const float left, const float right);
	//void DrawXYCoord(const float x_left, const float x_right,
	//	const int y_top, const int y_bottom);


//private:


	// edit
//public:
	//void ResetAvailableFlags(const bool flag);
	//void LengthFilter(const float min_pos, const float max_pos);
//	void FAFilter(const float min_pos, const float max_pos);
//	void CurvatureFilter(const float min_pos, const float max_pos);

	//void HideLines();
	//void HideOthers();
	//void ShowAllLines();
	//void ClearSelect();
	//void ResetSelect();
	//void SetSelect();

	//void CreateNewClass();
	//void SetNewClassColor(Color3F color);
	//void AddLinesToClass();
	//void MergeClass();
	//void ClearClass();

	//void CheckFibersWithCube(CUBE &cube);
//private:
	//void ResetResults();

	//void CheckFiberXPlane(Point3F pt1, Point3F pt2, Point3F pt3, Point3F pt4,bool *mask, vector<int> &res);
	//void CheckFiberYPlane(Point3F pt1, Point3F pt2, Point3F pt3, Point3F pt4,bool *mask, vector<int> &res);
	//void CheckFiberZPlane(Point3F pt1, Point3F pt2, Point3F pt3, Point3F pt4,bool *mask, vector<int> &res);

	// FA
//public:
	//void GetFAStatistics(vector<float> &fas);



	// MDS
//public:
	//void CalculateMDS();
//private:
	//void Calculate2DMDS();

	//void CalculateMDSWithPrincipalCurves(const bool flag);

	// classify
//public:
	// DM
	//void CalculateDistanceMatrix();
	//void CalculateDistanceMatrixMean();
	//void CalculateDistanceMatrixMax();
	//void CalculateDistanceMatrixMin();
	//void CalculateDistanceMatrixFa();
	//void CalculateDistanceMatrixTrace();
	//void CalculateDistanceMatrixWeighted(const float alpha, const float beta);
	// DTI
	//void SemiClassify();
	/*void MakeHierarchical(float dis);*/
	//void DBCClassify(const float dis, const int param);
	//void SingleLinkage();
	//void ChangeSingleLinkageLevel(const int level);
	// MDS
	//void ClassifyMDSByKMeans(const int k);
	//void ClassifyMDSByMeanShift(const float r);
	//void ClassifyMDSByDBC();
	// Principal Curve
	//void CalculatePrincipalCurve(const bool flag);
	//void ClearAllPrincipalCurves();

	// atlas-based
//public:
	//inline int GetClassNum() { return m_classNum; }
//	int GetSrcLines(CLine **o_lines, int **o_results);
	//void AtlasClassify(CLine *srcLines, int *srcResutls, 
	//	const int srcNum, const int srcClassNum);

	//float GetDTIScale() { return m_scale; }
	//void SetDTIScale(const float scale) { m_scale = scale; }
	//Point3F GetDTICenter() { return m_center; }
	//void SetDTICenter(Point3F center) { m_center = center; }

	//void FilpYZ();
	//void FilpXZ();
	//void FilpXY();

	//void IncreaseX();
	//void IncreaseY();
	//void IncreaseZ();
	//void DecreaseX();
	//void DecreaseY();
	//void DecreaseZ();

	//void TuneLeft();
	//void TuneRight();
	//void TuneUp();
	//void TuneDown();
	//void TuneForward();
	//void TuneBackward();

//private:
	//int GroupLines(const int classID, const float dis, CLine **o_lines);


//private:
	//bool CheckFlags(bool *flags, const int size, const bool mark);

	//void MakeGroup(LINE_GROUP *lineGroup, const float dis);
	//void DBC(LINE_GROUP *lineGroup, const int param);
	//void DBCStep(LINE_GROUP *lineGroup, bool *groupFlag, 
	//	bool *avaliableFlag, const int pos);

	//void DoClustering(const int num, const float t);
	//float GetMinDistance(float *distance, const int size, int &left, 
	//	int &right);
	//void ResetDistance(float **distance, const int size, const int left, 
	//	const int right);

	//float CalculateTwoLinesDistance(CLine *line1, CLine *line2, const float t);
	//void SetSrcDst(CLine *line1, CLine *line2);
	//float TwoLinesDistance(CLine *line1, CLine *line2, const float t);
	//float PointToLineDistance(Point3F point, CLine *line);
	//float CurvatureDistance(CLine *line1, CLine *line2);
	//float TorsionDistance(CLine *line1, CLine *line2);
	//float PointToLineDistance(Point3F point, CLine *line, Point3F &nearestPt);

	//void ClassifyLines();
	//void InitEnergy(float *last_energy, float *this_energy);
	//BOOL TransformEnergy(float *last_energy, float *this_energy);

	//void PrincipalCurve(vector<int> &src, vector<Point3F> &res);


	//float FindMinDistance(OUT int &left, int &right, IN float *dm);
	//void ResetDM(float *dm, bool *mask);
	//void ResetResults(int *results);
	//float FindMin(float *dm, bool *mask, const int id);


	// Abstraction
public:
	//void AbstractLines();
	//void ChangeLevel();
	void LoadOriginalFieldLines();
	//void ProtoChangeLevel();
	//inline LINE_LEVEL GetAbstractFlag() { return m_lineLevel; }
//private:
	//void AbstractOneGroup(bool *flags, LINE_GROUP *groups, 
	//	CLine *lines, const int index);

	// MDS LOD
//private:
	//void MakeMDSLOD();

	//void NormalizePoints(Point2F *points, const int num);
	//void NormalizeDistanceMatrix(float *dm, const int num);

	//
//public:
	//void FilterFiber(const float x, const float y, const float z);
};
