#include "subwindow.h"
#include "ui_subwindow.h" 
SubWindow::SubWindow(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::SubWindowClass())//,polePieceMeasureWin(nullptr)  // 初始化主窗口实例指针为空
{
	ui->setupUi(this);
  }

SubWindow::~SubWindow()
{
	delete ui;
}

 