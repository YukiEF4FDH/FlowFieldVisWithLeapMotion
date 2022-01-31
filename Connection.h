#pragma once
#ifndef __AFXWIN_H__
#error "在包含此文件之前包含“stdafx.h”以生成 PCH 文件"
#endif
#include "LeapC.h"

extern LEAP_CONNECTION ConnectionHandle;
extern bool IsConnected;
extern uint32_t DeviceId;

class LeapMotionConnection
{
public:
	LeapMotionConnection();

	LEAP_CONNECTION* OpenConnectionWithConfig(const LEAP_CONNECTION_CONFIG* connectionConfig);
	void CloseConnection();
	void DestroyConnection();
	static const char* ResultString(eLeapRS r);

	typedef void (*connection_callback)           ();
	typedef void (*device_callback)               (const LEAP_DEVICE_INFO* device_info, const LEAP_DEVICE device);
	typedef void (*tracking_callback)             (const LEAP_TRACKING_EVENT* tracking_event);

	struct Callbacks {
		connection_callback           on_connection;
		device_callback               on_device_found;
		tracking_callback             on_frame;
	};

};

extern struct LeapMotionConnection::Callbacks ConnectionCallbacks;
extern void millisleep(int milliseconds);