#pragma once


// InputExpression �Ի���

class InputExpression : public CDialogEx
{
	DECLARE_DYNAMIC(InputExpression)

public:
	InputExpression(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~InputExpression();

// �Ի�������
	enum { IDD = IDD_DIALOG1 };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	CString Expression;
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedCancel();
};
