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

// �豸A���豸Bһǰһ��������ȡ�˸�5֡�������Stable�����Կ�ʼ�ں����ݡ���һ��ʼֻ���豸A������/��;���豸���ȶ���ֹͣ�ںϣ�
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

    // ���0�� �豸A/B��ǰ�ɼ������ֵ���Ŀ�Ƿ���һֻ
    if (frame->nHands != 1) // ����һֻ�֣�����״̬λ����
    { 
        if (frame->info.frame_id == countForFrame_DeviceA + 1) // ��;��ϵ��������������¼A / B��֡��
        {
            countForFrame_DeviceA = frame->info.frame_id;
            broke = true;
        }
        else if (frame->info.frame_id == countForFrame_DeviceB + 1) // ��;��ϵ��������������¼A / B��֡��
        {
            countForFrame_DeviceB = frame->info.frame_id;
            broke = true;
        }
        else // һ��ʼ�����
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

    // ���1�� �豸A/B��ǰ�ɼ�������һֻ���Ƿ�������
    LEAP_HAND* hand = &frame->pHands[0]; // ���ֻ���������
    if (hand->type != eLeapHandType_Right)  // �������֣�����״̬λ����
    {
        if (frame->info.frame_id == countForFrame_DeviceA + 1) // ��;��ϵ��������������¼A / B��֡��
        {
            countForFrame_DeviceA = frame->info.frame_id;
            broke = true;
        }
        else if (frame->info.frame_id == countForFrame_DeviceB + 1) // ��;��ϵ��������������¼A / B��֡��
        {
            countForFrame_DeviceB = frame->info.frame_id;
            broke = true;
        }
        else // һ��ʼ�����
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

    // ���2�� �豸A/B��ǰ�ɼ�������һֻ���ֵ������ָ���Ƿ���һ���������
    LEAP_DIGIT* digits = new LEAP_DIGIT[5]; // hand����ָ����
    for (int i = 0; i < 5; i++) digits[i] = hand->digits[i];
    int extendFingerNum = ExtendFingerNum(digits);
    if (
        (!(extendFingerNum == 1 && digits[1].is_extended == 1)) // �Ȳ������һ����ָ Ҳ������������ָ������״̬λ����
        &&
        (!(extendFingerNum == 5))
        )
    {
        if (frame->info.frame_id == countForFrame_DeviceA + 1) // ��;��ϵ��������������¼A / B��֡��
        {
            countForFrame_DeviceA = frame->info.frame_id;
            broke = true;
        }
        else if (frame->info.frame_id == countForFrame_DeviceB + 1) // ��;��ϵ��������������¼A / B��֡��
        {
            countForFrame_DeviceB = frame->info.frame_id;
            broke = true;
        }
        else // һ��ʼ�����
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

    if (countForFrame_DeviceA == 0 && countForFrame_DeviceB == 0 && !broke) // �״���ͨ �� ͨ������Check ���豸����Ϊ�豸A
    {
        countForFrame_DeviceA = frame->info.frame_id; // ȷ���豸A

        digitsA = digits; // ��һ�����豸A�ɼ�����ָλ������
        AX = digitsA[1].distal.next_joint.x; AY = digitsA[1].distal.next_joint.y; AZ = digitsA[1].distal.next_joint.z;
        extendFingerNumA = extendFingerNum;
  
        receivedA = true; // �豸A ������Merge����ָλ�������Ѳɼ���
        receivedB = false; // �ȴ��豸B������

        countForAStable++;
    }
    else if (countForFrame_DeviceB == 0 && frame->info.frame_id != countForFrame_DeviceA + 1 && !broke) // �״�ͨ������Check���豸B
    {
        countForFrame_DeviceB = frame->info.frame_id; // ȷ���豸B

        digitsB = digits; // ��һ�����豸B�ɼ�����ָλ������
        BX = digitsB[1].distal.next_joint.x; BY = digitsB[1].distal.next_joint.y; BZ = digitsB[1].distal.next_joint.z;
        extendFingerNumB = extendFingerNum;

        receivedB = true; // �豸B������Merge����ָ�����Ѿ��ɼ���

        countForBStable++;

        if (receivedA) // ��һ��AB�����ݶ��Ѳɼ���
        {
            countForABStable++;
            receivedA = false; // �ȴ���һ��AB�����ݲɼ�
        }
    }
    else if (frame->info.frame_id == countForFrame_DeviceA + 1) // �����״�ͨ������Check���豸A
    {
        countForFrame_DeviceA = frame->info.frame_id; 

        digitsA = digits;
        AX = digitsA[1].distal.next_joint.x; AY = digitsA[1].distal.next_joint.y; AZ = digitsA[1].distal.next_joint.z;
        extendFingerNumA = extendFingerNum;

        if (broke)
            broke = false; // �����ж�

        if (countForAStable < SENSITIVITY_FOR_STABLE) // �豸A�Ĳɼ�֡���Ƿ����㵽���ȶ�״̬��������
            countForAStable++;
        else if (countForAStable >= SENSITIVITY_FOR_STABLE)
            isAStable = true;

        receivedA = true;
        receivedB = false;
    }
    else if (frame->info.frame_id == countForFrame_DeviceB + 1) // �����״�ͨ������Check���豸B
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
            if (!isLeapQuery) // һ�㲻���������Ϊ�����豸A���ȶ�������
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