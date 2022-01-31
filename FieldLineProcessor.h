/// --------------------------XY Chen--------------------------
// Build from DTI Fiber Explorer (LineProcessor.h)
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

#include "FieldLine.h"

#include <vector>
#include <deque>
#include <..\src\IL\ILRender.h>
#include <vector>
#include <Eigen/Dense>
#include "ArcBall.h"

using Eigen::MatrixXd;
using namespace std;

typedef Eigen::Vector3d Point;

enum LINE_LEVEL {
	LEVEL_ORIGINAL,
	LEVEL_ABSTRACT
};

// class
class CFieldLineProcessor
{
public:
	Point3F m_max;
	Point3F m_min;
	int MAXCURVELENGTH;
	int MINCURVELENGTH;
private:
	CFieldLine	*m_lines;				// current lines
	int		m_lineNum;				// current line number
	int		m_pointNum;				// current mds point number

	Point3F	m_center;				// the center of all DTI fibers
	float m_scale;					// the scale of all DTI fibers

	LINE_LEVEL m_lineLevel;			//

	CFieldLine *m_originalLines;			//
	int m_originalLineNum;			//
	int m_originalPointNum;			//

	CFieldLine *m_abstractLines;			//
	int m_abstractLineNum;			//
	int m_abstractPointNum;			//

	ILines::ILRender m_render;		// i

	int	*m_first;					//
	int	*m_vertCount;				//
	float *m_colors;				//
	float *m_vertices;				//

public:
	CFieldLineProcessor(void);
	~CFieldLineProcessor(void);

	// Render
public:
	void InitILRender();
private:
	void CreateVertexBuffer();
	void CreateColorBuffer();

	// Files
public:
	void OpenConfig(const char *filename);

	void OpenOriginalLines(const char *filename);

	void Delete();

	void LoadOriginalFieldLines();
private:
	void SetCenter();

public:
	inline int GetLineNum() { return m_lineNum; }

	// OpenGL
public:
	void DrawDTI();

	// Leap Motion
public:
	int LMFILTER_WINDOWLENGTH;

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

	deque<float> lmTipXPosC;
	deque<float> lmTipYPosC;
	deque<float> lmTipZPosC;
	int lmCurveLengthC;

	deque<float> AlmTipXPos;
	deque<float> AlmTipYPos;
	deque<float> AlmTipZPos;
	int AlmCurveLength;

	deque<float> BlmTipXPos;
	deque<float> BlmTipYPos;
	deque<float> BlmTipZPos;
	int BlmCurveLength;

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

public:
	void ClearOriginalCurve(int type);
	void ClearTSCurve(int type);
	void ClearAngleBinSeq();
	void ClearCurvatureBinSeq();
	void ClearAngleInfo(int type);
	void ClearCurvatureInfo(int type);
	void ClearDTISimilarityInfo();
	void ClearFilteredDTICurves();
	void ClearAllFilterInfo();

	void LMSlideWindowFilter(float &newX, float &newY, float &newZ);

	void UpdateLMCurve(float lmTipXPos,float lmTipYPos,float lmTipZPos, int type);
	void UpdateSketchCurve(float sketchXPos, float sketchYPos);

	//void PopLMCurve();

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
};
