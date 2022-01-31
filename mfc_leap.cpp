#include "stdafx.h"

#include "mfc_leap.h"
#include <iostream>
#include <cstring>
#include "Leap.h"
#include "FlowFieldVisWithLeapMotionView.h"
#include "FlowFieldVisWithLeapMotionDoc.h"
#include "FlowFieldVisWithLeapMotion.h"
#include "MainFrm.h"
#include <queue>
#include <cmath>

using namespace Leap;	

extern CFlowFieldVisWithLeapMotionApp theApp;
Vector TranslateDistance;

static int FrameCount = 0;
static bool isLeapQuery = false;

void SampleListener::onConnect(const Controller& controller) 
{
	//controller.enableGesture(Gesture::TYPE_SWIPE);
}

void SampleListener::onFrame(const Controller& controller) 
{
	// Get the most recent frame and report some basic information
	const Frame frame = controller.frame();
	const Frame PreFrame = controller.frame(1);
	CMainFrame *pFrame = (CMainFrame*)theApp.m_pMainWnd;
	CFlowFieldVisWithLeapMotionView *pview = (CFlowFieldVisWithLeapMotionView *)pFrame->GetActiveView();
	CPoint point;
	GetCursorPos(&point);
	HandList hands = frame.hands();
	
	const float SENSITIVITY = 0.1;
	Hand hand = hands[0]; 
	Finger finger = hand.fingers()[1];
	HandList preHands = PreFrame.hands();
	Hand preHand = preHands[0]; 
	Finger preFinger = preHand.fingers()[1];
	
	if (ExtendFingerNum(frame)!=1)
	{
		isLeapQuery = false;
		if (isLeapQuery) pview->m_MessageHandle.WriteTmpLMCurve();
	}
	if ( hands.count()==1 && ExtendFingerNum(frame) == 1 && hand.isRight() && finger.type() == 1)
	{
		if (!isLeapQuery)
		{
			isLeapQuery = true;
			pview->m_MessageHandle.UpdateLMCurve_Start(finger.tipPosition().x,finger.tipPosition().y,finger.tipPosition().z);
			pview->Invalidate(0);
		}
		else if(abs(finger.tipPosition().x-preFinger.tipPosition().x)>=SENSITIVITY)
		{
			pview->m_MessageHandle.UpdateLMCurve(finger.tipPosition().x,finger.tipPosition().y,finger.tipPosition().z);
			pview->Invalidate(0);
		}
	}
}

int SampleListener::ExtendFingerNum(Frame frame)
{
	HandList Hands = frame.hands();
	FingerList fingers = Hands[0].fingers();
	int extendedFingers = 0;
	for (int i = 0; i < fingers.count(); i++)
	{
		if (fingers[i].isExtended())
			extendedFingers++;
	}
	return extendedFingers;
}