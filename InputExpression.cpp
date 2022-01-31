// InputExpression.cpp : 实现文件
//

#include "stdafx.h"
#include "FlowFieldVisWithLeapMotion.h"
#include "InputExpression.h"
#include "afxdialogex.h"


// InputExpression 对话框

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


// InputExpression 消息处理程序
void InputExpression::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(true);
	CDialogEx::OnOK();
}


void InputExpression::OnBnClickedCancel()
{
	// TODO: 在此添加控件通知处理程序代码
	CDialogEx::OnCancel();
}