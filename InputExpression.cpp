// InputExpression.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "FlowFieldVisWithLeapMotion.h"
#include "InputExpression.h"
#include "afxdialogex.h"


// InputExpression �Ի���

IMPLEMENT_DYNAMIC(InputExpression, CDialogEx)

InputExpression::InputExpression(CWnd* pParent /*=NULL*/)
	: CDialogEx(InputExpression::IDD, pParent)
	, Expression(_T(""))
{

}

InputExpression::~InputExpression()
{
}

void InputExpression::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT1, Expression);
}


BEGIN_MESSAGE_MAP(InputExpression, CDialogEx)
END_MESSAGE_MAP()


// InputExpression ��Ϣ�������
void InputExpression::OnBnClickedOk()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	UpdateData(true);
	CDialogEx::OnOK();
}


void InputExpression::OnBnClickedCancel()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CDialogEx::OnCancel();
}