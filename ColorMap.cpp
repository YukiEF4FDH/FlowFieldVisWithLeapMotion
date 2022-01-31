/// ----------------------------------------------------------------
/// project: DTI Fiber Explorer
/// file: FieldLineProcessor.h
/// source file of the color map
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

// projects includes
#include "StdAfx.h"
#include "ColorMap.h"

// global variables
unsigned char g_clusterColorMap[33] = {
	166, 206, 227,
	 31, 120, 180,
	178, 223, 138,
	 51, 160,  44,
	251, 154, 153,
	227,  26,  28,
	253, 191, 111,
	255, 127,   0,
	202, 178, 214,
	106,  61, 154,
	255, 255, 153
};

unsigned char g_normalColorMap[75] = {
	  0,   0, 127,
	  0,   0, 255,
	  0, 127,   0,
	  0, 127, 127,
	  0, 127, 255,
	  0, 255,   0,
	  0, 255, 127,
	  0, 255, 255,
	127,   0,   0,
	127,   0, 127,
	127,   0, 255,
	127, 127,   0,
	127, 127, 127,
	127, 127, 255,
	127, 255,   0,
	127, 255, 127,
	127, 255, 255,
	255,   0,   0,
	255,   0, 127,
	255,   0, 255,
	255, 127, 127,
	255, 127, 255,
	255, 255,   0,
	255, 255, 127,
	255, 255, 255
};

unsigned char g_boxColorMap[24] = {
	228,  26,  28,
	 55, 126, 184,
	 77, 175,  74,
	152,  78, 163,
	255, 127,   0,
	255, 255,  51,
	166,  86,  40,
	247, 129, 191
};


CColorMap g_colorMap;


// implementations
CColorMap::CColorMap(void)
{
	m_clusterColorNum = 92;
	m_clusterColors = new Color3F[m_clusterColorNum];
	for (int i=0; i<11; i++) {
		float r = static_cast<float>(g_clusterColorMap[i*3+0]) / 255.0f;
		float g = static_cast<float>(g_clusterColorMap[i*3+1]) / 255.0f;
		float b = static_cast<float>(g_clusterColorMap[i*3+2]) / 255.0f;
		m_clusterColors[i].r = r; m_clusterColors[i].g = g; m_clusterColors[i].b = b;
	}
	for (int i=11; i<21; i++) {
		int ii = i - 11;
		int ij = i - 10;
		float r = m_clusterColors[ii].r + m_clusterColors[ij].r;
		float g = m_clusterColors[ii].g + m_clusterColors[ij].g;
		float b = m_clusterColors[ii].b + m_clusterColors[ij].b;
		r = r / 2.0f;
		g = g / 2.0f;
		b = b / 2.0f;
		m_clusterColors[i].r = r; m_clusterColors[i].g = g; m_clusterColors[i].b = b;
	}
	for (int i=21; i<46; i++) {
		int index = i - 21;
		float r = static_cast<float>(g_normalColorMap[index*3+0]) / 255.0f;
		float g = static_cast<float>(g_normalColorMap[index*3+1]) / 255.0f;
		float b = static_cast<float>(g_normalColorMap[index*3+2]) / 255.0f;
		m_clusterColors[i].r = r; m_clusterColors[i].g = g; m_clusterColors[i].b = b;
	}
	for (int i=46; i<92; i++) {
		m_clusterColors[i] = m_clusterColors[i-46];
	}

	m_boxColorNum = 8;
	m_boxColors = new Color3F[m_boxColorNum];
	for (int i=0; i<m_boxColorNum; i++) {
		float r = static_cast<float>(g_boxColorMap[i*3+0]) / 255.0f;
		float g = static_cast<float>(g_boxColorMap[i*3+1]) / 255.0f;
		float b = static_cast<float>(g_boxColorMap[i*3+2]) / 255.0f;
		m_boxColors[i].r = r; m_boxColors[i].g = g; m_boxColors[i].b = b;
	}

	for (int i=0; i<m_clusterColorNum; i++) {
		int r = static_cast<int>(m_clusterColors[i].r * 255.0f);
		int g = static_cast<int>(m_clusterColors[i].g * 255.0f);
		int b = static_cast<int>(m_clusterColors[i].b * 255.0f);
		COLORREF color = RGB(r, g, b);
		m_lstClusterColors.AddTail(color);
	}

	for (int i=0; i<m_boxColorNum; i++) {
		int r = g_boxColorMap[i*3+0];
		int g = g_boxColorMap[i*3+1];
		int b = g_boxColorMap[i*3+2];
		COLORREF color = RGB(r, g, b);
		m_lstBoxColors.AddTail(color);
	}
}

CColorMap::~CColorMap(void)
{
	SAFE_DELETE(m_clusterColors);
	SAFE_DELETE(m_boxColors);
	m_clusterColorNum = 0;
	m_boxColorNum = 0;
}