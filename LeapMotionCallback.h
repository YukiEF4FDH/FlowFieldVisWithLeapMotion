#pragma once
#ifndef __AFXWIN_H__
#error "在包含此文件之前包含“stdafx.h”以生成 PCH 文件"
#endif

#include "resource.h"

#include <iostream>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include "LeapC.h"
#include "Connection.h"

//static LEAP_CONNECTION* connectionHandle;
//static int count;

class Listener
{
public:
	LeapMotionConnection connection;
public:
	Listener();
	static void OnConnect();
	static void OnDevice(const LEAP_DEVICE_INFO* props, const LEAP_DEVICE device);
	static void OnFrame(const LEAP_TRACKING_EVENT* frame);
	static int ExtendFingerNum(LEAP_DIGIT* digits);
};