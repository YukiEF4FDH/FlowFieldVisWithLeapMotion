#include "stdafx.h"
#include <iostream>
#include <cstring>
#include "LeapMotionCallback.h"
#include <LeapC.h>
#include "FlowFieldVisWithLeapMotionView.h"
#include "FlowFieldVisWithLeapMotionDoc.h"
#include "FlowFieldVisWithLeapMotion.h"
#include "MainFrm.h"
#include <queue>
#include <cmath>
#include <fstream>
using namespace std;

extern CFlowFieldVisWithLeapMotionApp theApp;
LEAP_CONNECTION* connectionHandle;
static bool isLeapQuery = false;
static int countForFrame;
Leap::Vector TranslateDistance;

static int countForFrame_DeviceA;
static int countForFrame_DeviceB;

static float AX; static float AY; static float AZ;
static float BX; static float BY; static float BZ;

// 设备A、设备B一前一后连续获取了各5帧的情况：Stable，可以开始融合数据。（一开始只有设备A有数据/中途有设备不稳定则停止融合）
static int const SENSITIVITY_FOR_STABLE = 5;
static int countForABStable = 0;
static int countForAStable = 0;
static int countForBStable = 0;
static bool isABStable = false;
static bool isAStable = false;
static bool isBStable = false;
static bool receivedA = false;
static bool receivedB = false;
static bool broke = false;

static int extendFingerNumA = 0;
static LEAP_DIGIT* digitsA;
static int extendFingerNumB = 0;
static LEAP_DIGIT* digitsB;

static int ABcount = 0;

Listener::Listener()
{
    connection = LeapMotionConnection();
    LEAP_CONNECTION_CONFIG connectionConfig;
    connectionConfig.server_namespace = NULL;
    connectionConfig.flags = eLeapConnectionConfig_MultiDeviceAware;
    connectionConfig.size = sizeof(connectionConfig);
    connectionHandle = connection.OpenConnectionWithConfig(&connectionConfig);
    LeapSetPolicyFlags(*connectionHandle, eLeapPolicyFlag_Images | eLeapPolicyFlag_MapPoints, 0);

    countForFrame = 0;
}

void Listener::OnConnect()
{
    //std::cout << "Connected" << endl;
}

void Listener::OnDevice(const LEAP_DEVICE_INFO* props, const LEAP_DEVICE device)
{
    //ofstream outFile("D:\\MyAssignments\\GraduationDesign\\tmp\\OnDevice.txt", ios::app);
    //outFile << "Found Device " << props->serial << endl;
    //outFile.close();
    //printf("Found device %s.\n", props->serial);
    ////Test();
    LeapSubscribeEvents(*connectionHandle, device);
    LeapSetPrimaryDevice(*connectionHandle, device, false);

    ofstream outFile("D:\\MyAssignments\\GraduationDesign\\tmp\\Info.txt", ios::app);
    outFile << "SetPrimaryDevice " << props->serial << endl;
    outFile.close();

    countForFrame_DeviceA = 0; countForFrame_DeviceB = 0;
    AX = 0.0; AY = 0.0; AZ = 0.0;
}

void Listener::OnFrame(const LEAP_TRACKING_EVENT* frame)
{
    CMainFrame* pFrame = (CMainFrame*)theApp.m_pMainWnd;
    CFlowFieldVisWithLeapMotionView* pview = (CFlowFieldVisWithLeapMotionView*)pFrame->GetActiveView();

    // 检查0： 设备A/B当前采集到的手的数目是否是一只
    if (frame->nHands != 1) // 不是一只手：其他状态位归零
    { 
        if (frame->info.frame_id == countForFrame_DeviceA + 1) // 中途打断的情况：仅继续记录A / B的帧数
        {
            countForFrame_DeviceA = frame->info.frame_id;
            broke = true;
        }
        else if (frame->info.frame_id == countForFrame_DeviceB + 1) // 中途打断的情况：仅继续记录A / B的帧数
        {
            countForFrame_DeviceB = frame->info.frame_id;
            broke = true;
        }
        else // 一开始的情况
        {
            countForFrame_DeviceA = 0;
            countForFrame_DeviceB = 0;
            broke = false;
        }
        receivedA = false;
        isAStable = false;
        countForAStable = 0;
        receivedB = false;
        isBStable = false;
        countForBStable = 0;
        isABStable = false;
        countForABStable = 0;

        return; 
    }

    // 检查1： 设备A/B当前采集到的这一只手是否是右手
    LEAP_HAND* hand = &frame->pHands[0]; // 左手或者是右手
    if (hand->type != eLeapHandType_Right)  // 不是右手：其他状态位归零
    {
        if (frame->info.frame_id == countForFrame_DeviceA + 1) // 中途打断的情况：仅继续记录A / B的帧数
        {
            countForFrame_DeviceA = frame->info.frame_id;
            broke = true;
        }
        else if (frame->info.frame_id == countForFrame_DeviceB + 1) // 中途打断的情况：仅继续记录A / B的帧数
        {
            countForFrame_DeviceB = frame->info.frame_id;
            broke = true;
        }
        else // 一开始的情况
        {
            countForFrame_DeviceA = 0;
            countForFrame_DeviceB = 0;
            broke = false;
        }
        receivedA = false;
        isAStable = false;
        countForAStable = 0;
        receivedB = false;
        isBStable = false;
        countForBStable = 0;
        isABStable = false;
        countForABStable = 0;

        return;
    }

    // 检查2： 设备A/B当前采集到的这一只右手的伸出手指数是否是一根或者五根
    LEAP_DIGIT* digits = new LEAP_DIGIT[5]; // hand的手指数组
    for (int i = 0; i < 5; i++) digits[i] = hand->digits[i];
    int extendFingerNum = ExtendFingerNum(digits);
    if (
        (!(extendFingerNum == 1 && digits[1].is_extended == 1)) // 既不是伸出一根手指 也不是伸出五根手指：其他状态位归零
        &&
        (!(extendFingerNum == 5))
        )
    {
        if (frame->info.frame_id == countForFrame_DeviceA + 1) // 中途打断的情况：仅继续记录A / B的帧数
        {
            countForFrame_DeviceA = frame->info.frame_id;
            broke = true;
        }
        else if (frame->info.frame_id == countForFrame_DeviceB + 1) // 中途打断的情况：仅继续记录A / B的帧数
        {
            countForFrame_DeviceB = frame->info.frame_id;
            broke = true;
        }
        else // 一开始的情况
        {
            countForFrame_DeviceA = 0;
            countForFrame_DeviceB = 0;
            broke = false;
        }
        receivedA = false;
        isAStable = false;
        countForAStable = 0;
        receivedB = false;
        isBStable = false;
        countForBStable = 0;
        isABStable = false;
        countForABStable = 0;

        return;
    }

    //=============================================================================

    if (countForFrame_DeviceA == 0 && countForFrame_DeviceB == 0 && !broke) // 首次连通 并 通过三道Check 的设备：定为设备A
    {
        countForFrame_DeviceA = frame->info.frame_id; // 确定设备A

        digitsA = digits; // 第一次由设备A采集到手指位置数据
        AX = digitsA[1].distal.next_joint.x; AY = digitsA[1].distal.next_joint.y; AZ = digitsA[1].distal.next_joint.z;
        extendFingerNumA = extendFingerNum;
  
        receivedA = true; // 设备A ：用作Merge的手指位置数据已采集到
        receivedB = false; // 等待设备B的数据

        countForAStable++;
    }
    else if (countForFrame_DeviceB == 0 && frame->info.frame_id != countForFrame_DeviceA + 1 && !broke) // 首次通过三道Check：设备B
    {
        countForFrame_DeviceB = frame->info.frame_id; // 确定设备B

        digitsB = digits; // 第一次由设备B采集到手指位置数据
        BX = digitsB[1].distal.next_joint.x; BY = digitsB[1].distal.next_joint.y; BZ = digitsB[1].distal.next_joint.z;
        extendFingerNumB = extendFingerNum;

        receivedB = true; // 设备B：用作Merge的手指数据已经采集到

        countForBStable++;

        if (receivedA) // 这一轮AB的数据都已采集到
        {
            countForABStable++;
            receivedA = false; // 等待下一轮AB的数据采集
        }
    }
    else if (frame->info.frame_id == countForFrame_DeviceA + 1) // 不是首次通过三道Check：设备A
    {
        countForFrame_DeviceA = frame->info.frame_id; 

        digitsA = digits;
        AX = digitsA[1].distal.next_joint.x; AY = digitsA[1].distal.next_joint.y; AZ = digitsA[1].distal.next_joint.z;
        extendFingerNumA = extendFingerNum;

        if (broke)
            broke = false; // 不再中断

        if (countForAStable < SENSITIVITY_FOR_STABLE) // 设备A的采集帧数是否满足到达稳定状态的条件？
            countForAStable++;
        else if (countForAStable >= SENSITIVITY_FOR_STABLE)
            isAStable = true;

        receivedA = true;
        receivedB = false;
    }
    else if (frame->info.frame_id == countForFrame_DeviceB + 1) // 不是首次通过三道Check：设备B
    {
        countForFrame_DeviceB = frame->info.frame_id;

        digitsB = digits;
        BX = digitsB[1].distal.next_joint.x; BY = digitsB[1].distal.next_joint.y; BZ = digitsB[1].distal.next_joint.z;
        extendFingerNumB = extendFingerNum;

        if (broke)
            broke = false;

        receivedB = true;

        if (countForBStable < SENSITIVITY_FOR_STABLE)
            countForBStable++;
        else if (countForBStable >= SENSITIVITY_FOR_STABLE)
            isBStable = true;

        if (receivedA && countForABStable < SENSITIVITY_FOR_STABLE)
        {
            countForABStable++;
            receivedA = false;
        }
        else if (receivedA && countForABStable >= SENSITIVITY_FOR_STABLE)
        {
            isABStable = true;
            receivedA = false;
        }
    }

    //=============================================================================

    if (extendFingerNumA == 5 || extendFingerNumB == 5)
        isLeapQuery = false;

    else if ((extendFingerNumA == 1 && digitsA[1].is_extended==1) || (extendFingerNumB == 1 && digitsB[1].is_extended == 1))
    {
        if (isABStable)
        {
            if (!isLeapQuery) // 一般不会走这里（因为都是设备A先稳定下来）
            {
                isLeapQuery = true;
                pview->m_MessageHandle.UpdateLMCurve_Start((AX + BX) / 2.0, (AY + BY) / 2.0, (AZ + BZ) / 2.0, 0);
                pview->m_MessageHandle.UpdateLMCurve_Start(AX, AY, AZ, 1);
                pview->m_MessageHandle.UpdateLMCurve_Start(BX, BY, BZ, 2);
                pview->Invalidate(0);
            }
            else
            {
                pview->m_MessageHandle.UpdateLMCurve((AX + BX) / 2.0, (AY + BY) / 2.0, (AZ + BZ) / 2.0, 0);
                pview->m_MessageHandle.UpdateLMCurve(AX, AY, AZ, 1);
                pview->m_MessageHandle.UpdateLMCurve(BX, BY, BZ, 2);               //else
                pview->Invalidate(0);

            }
        }
        else if (isAStable)
        {
            if (!isLeapQuery)
            {
                isLeapQuery = true;
                pview->m_MessageHandle.UpdateLMCurve_Start(AX, AY, AZ, 1);
                pview->Invalidate(0);
            }
            else
            {
                pview->m_MessageHandle.UpdateLMCurve(AX, AY, AZ, 1);
                pview->Invalidate(0);
            }
        }
    }
}

int Listener::ExtendFingerNum(LEAP_DIGIT* digits)
{
    int num = 0;
    for (int i = 0; i < 5; i++)
    {
        int extended = digits[i].is_extended;
        if (extended == 1) num++;
    }
    return num;
}