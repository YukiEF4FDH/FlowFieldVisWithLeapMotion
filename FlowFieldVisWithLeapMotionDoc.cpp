// This MFC Samples source code demonstrates using MFC Microsoft Office Fluent User Interface 
// (the "Fluent UI") and is provided only as referential material to supplement the 
// Microsoft Foundation Classes Reference and related electronic documentation 
// included with the MFC C++ library software.  
// License terms to copy, use or distribute the Fluent UI are available separately.  
// To learn more about our Fluent UI licensing program, please visit 
// http://msdn.microsoft.com/officeui.
//
// Copyright (C) Microsoft Corporation
// All rights reserved.

// FlowFieldVisWithLeapMotionDoc.cpp : implementation of the CFlowFieldVisWithLeapMotionDoc class
//

#include "stdafx.h"
#include "FlowFieldVisWithLeapMotion.h"

#include "FlowFieldVisWithLeapMotionDoc.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CFlowFieldVisWithLeapMotionDoc

IMPLEMENT_DYNCREATE(CFlowFieldVisWithLeapMotionDoc, CDocument)

BEGIN_MESSAGE_MAP(CFlowFieldVisWithLeapMotionDoc, CDocument)
END_MESSAGE_MAP()


// CFlowFieldVisWithLeapMotionDoc construction/destruction

CFlowFieldVisWithLeapMotionDoc::CFlowFieldVisWithLeapMotionDoc()
{
	// TODO: add one-time construction code here

}

CFlowFieldVisWithLeapMotionDoc::~CFlowFieldVisWithLeapMotionDoc()
{
}

BOOL CFlowFieldVisWithLeapMotionDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: add reinitialization code here
	// (SDI documents will reuse this document)

	return TRUE;
}




// CFlowFieldVisWithLeapMotionDoc serialization

void CFlowFieldVisWithLeapMotionDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: add storing code here
	}
	else
	{
		// TODO: add loading code here
	}
}


// CFlowFieldVisWithLeapMotionDoc diagnostics

#ifdef _DEBUG
void CFlowFieldVisWithLeapMotionDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CFlowFieldVisWithLeapMotionDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG


// CFlowFieldVisWithLeapMotionDoc commands
