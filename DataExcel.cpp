#include "DataExcel.h"

DataExcel::DataExcel(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::DataExcelClass())
{
	ui->setupUi(this);
}

DataExcel::~DataExcel()
{
	delete ui;
}

