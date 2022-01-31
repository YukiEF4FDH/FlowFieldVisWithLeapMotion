/// ----------------------------------------------------------------
/// project: DTI Fiber Explorer
/// file: FieldLineProcessor.h
/// Headfile of the color map
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

// project includes
#include "FieldLine.h"

// class
class CColorMap
{
private:
	Color3F *m_clusterColors;
	int m_clusterColorNum;

	Color3F *m_boxColors;
	int m_boxColorNum;

public:
	CList<COLORREF, COLORREF> m_lstClusterColors;
	CList<COLORREF, COLORREF> m_lstBoxColors;

public:
	CColorMap(void);
	~CColorMap(void);

public:
	inline Color3F GetClusterColor(const int i) { return m_clusterColors[i]; }
	inline Color3F GetBoxColor(const int i) { return m_boxColors[i]; }

	inline void SetClusterColor(const int i, Color3F color)
	{
		m_clusterColors[i] = color;
	}
	inline void SetBoxColor(const int i, Color3F color)
	{
		m_boxColors[i] = color;
	}
	
	inline void SetOpenGLClusterColor(const float a, const int i)
	{
		glColor4f(m_clusterColors[i].r, m_clusterColors[i].g, m_clusterColors[i].b, a);
	}
	inline void SetOpenGLBoxColor(const float a, const int i)
	{
		glColor4f(m_boxColors[i].r, m_boxColors[i].g, m_boxColors[i].b, a);
	}

	void SetOpenGLClusterColor(const float a, const int i, const int num);
};


