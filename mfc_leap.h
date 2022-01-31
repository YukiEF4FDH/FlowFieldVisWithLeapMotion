#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"

#include <iostream>
#include <cstring>
#include "Leap.h"
using namespace Leap;

class SampleListener : public Listener
{
public:
	virtual void onConnect(const Controller&);
	virtual void onFrame(const Controller&);
	int ExtendFingerNum(Frame frame);
};

const std::string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
const std::string boneNames[] = {"Metacarpal", "Proximal", "Middle", "Distal"};
const std::string stateNames[] = {"STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END"};
