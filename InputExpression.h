#pragma once


// InputExpression 对话框

class InputExpression : public CDialogEx
{
	DECLARE_DYNAMIC(InputExpression)

public:
	InputExpression(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~InputExpression();

// 对话框数据
	enum { IDD = IDD_DIALOG1 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	CString Expression;
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedCancel();
};
