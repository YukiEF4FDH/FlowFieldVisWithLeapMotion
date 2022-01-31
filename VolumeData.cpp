/// ----------------------------------------------------------------
/// project: DTI Fiber Explorer
/// file: LineProcessor.h
/// source file of the volume data class
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
#include "StdAfx.h"
#include "VolumeData.h"

// system includes
#include "float.h"

// implementations
CVolumeData::CVolumeData(void)
{
	m_width = m_height = m_depth = 0;
	m_data = NULL;
}

CVolumeData::~CVolumeData(void)
{
	SAFE_DELETE(m_data);
	m_width = m_height = m_depth = 0;
}

void CVolumeData::OpenVolumeData(const char *filename)
{
	m_width = 256;
	m_height = 256;
	m_depth = 180;
	FILE *fp = fopen(filename, "rb");
	if (fp == NULL)
		return;
	
	int size = m_width * m_height * m_depth;
	m_data = new float[size];
	float *pData = new float[size];
	int num = fread(pData, sizeof(float), size, fp);
	fclose(fp);

	for (int i=0; i<num; i++) {
		m_data[i] = float(pData[i]) / 255.0f;
	}
}

float CVolumeData::GetData(int x, int y, int z)
{
	int offset = (z * m_height + y) * m_width + x;
	return m_data[offset];
}

int CVolumeData::GetVolumeSize()
{
	return m_width;
}

void CVolumeData::GetXYPlane(const int z, float *out)
{
	for (int y=0; y<m_height; y++) {
		for (int x=0; x<m_width; x++) {
			int index = (z * m_height + y) * m_width + x;
			int offset = y * m_width + x;
			out[offset] = m_data[index];
		}
	}
}

void CVolumeData::GetXZPlane(const int y, float *out)
{
	for (int z=0; z<m_depth; z++) {
		for (int x=0; x<m_width; x++) {
			int index = (z * m_height + y) * m_width + x;
			int offset = z * m_width + x;
			out[offset] = m_data[index];
		}
	}
}

void CVolumeData::GetYZPlane(const int x, float *out)
{
	for (int z=0; z<m_depth; z++) {
		for (int y=0; y<m_height; y++) {
			int index = (z * m_height + y) * m_width + x;
			int offset = z * m_height + y;
			out[offset] = m_data[index];
		}
	}
}