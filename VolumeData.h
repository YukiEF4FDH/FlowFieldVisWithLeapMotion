/// ----------------------------------------------------------------
/// project: DTI Fiber Explorer
/// file: LineProcessor.h
/// Headfile of the volume data class
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


// class
class CVolumeData
{
public:
	CVolumeData(void);
	~CVolumeData(void);

private:
	float *m_data;
	int m_width;
	int m_height;
	int m_depth;

public:
	void OpenVolumeData(const char *filename);

	float GetData(int x, int y, int z);
	
	int GetVolumeSize();

	void GetXYPlane(const int z, float *out);
	void GetXZPlane(const int y, float *out);
	void GetYZPlane(const int x, float *out);

	inline int GetVolumeWidth() { return m_width; }
	inline int GetVolumeHeight() { return m_height; }
	inline int GetVolumeDepth() { return m_depth; }
};
